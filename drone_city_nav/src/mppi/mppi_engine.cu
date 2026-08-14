#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/mppi_input_validation.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/mppi/mppi_route_projection.hpp"
#include "drone_city_nav/mppi/mppi_separation_acquisition.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cuda_runtime.h>
#include <limits>
#include <mutex>
#include <ranges>
#include <stdexcept>
#include <utility>

#include "mppi_cuda_resources.cuh"

namespace drone_city_nav::mppi {
namespace {

constexpr int kThreadsPerBlock{256};
constexpr std::size_t kControlUpdateStepTile{32U};
constexpr std::size_t kControlUpdateRolloutLanes{8U};
constexpr std::size_t kControlUpdatePartitions{16U};
constexpr std::size_t kMaximumKnownSolids{2048U};
constexpr std::size_t kMaximumRoutePoints{512U};
constexpr std::size_t kMaximumDynamicAircraft{16U};
constexpr std::size_t kRepairCandidateCount{6U};
constexpr float kPi{3.14159265358979323846F};
constexpr float kInfinity{std::numeric_limits<float>::infinity()};
static_assert(kControlUpdateStepTile * kControlUpdateRolloutLanes ==
              static_cast<std::size_t>(kThreadsPerBlock));

using detail::checkCuda;
using detail::DeviceBuffer;
using detail::elapsedMs;
using detail::EsdfTexture;
using detail::Event;

struct DeviceBuffers {
  DeviceBuffer<float> noise_ax;
  DeviceBuffer<float> noise_ay;
  DeviceBuffer<float> noise_az;
  DeviceBuffer<float> noise_yaw;
  DeviceBuffer<float> soft_cost;
  DeviceBuffer<float> critical_exposure;
  DeviceBuffer<float> planning_exposure;
  DeviceBuffer<float> minimum_clearance;
  DeviceBuffer<std::uint8_t> worst_tier;
  DeviceBuffer<std::uint8_t> raw_collision;
  DeviceBuffer<std::uint8_t> solid_collision;
  DeviceBuffer<float> weights;
  DeviceBuffer<Control> nominal;
  DeviceBuffer<Control> updated;
  DeviceBuffer<Control> control_update_partials;
  DeviceBuffer<Control> best_eligible;
  DeviceBuffer<Control> repair_candidates;
  DeviceBuffer<int> best_tier;
  DeviceBuffer<int> best_rollout;
  DeviceBuffer<float> best_critical;
  DeviceBuffer<float> best_planning;
  DeviceBuffer<float> minimum_soft;
  DeviceBuffer<float> weight_sum;
  DeviceBuffer<KnownSolid> solids{kMaximumKnownSolids};
  DeviceBuffer<RouteSample3D> route_points{kMaximumRoutePoints};
  DeviceBuffer<DynamicAircraftSample> dynamic_aircraft_samples;
  DeviceBuffer<float> dynamic_aircraft_radii{kMaximumDynamicAircraft};
  DeviceBuffer<std::uint32_t> dynamic_aircraft_active_steps{kMaximumDynamicAircraft};

  DeviceBuffers(const std::size_t rollouts, const std::size_t steps)
      : noise_ax{rollouts * steps},
        noise_ay{rollouts * steps},
        noise_az{rollouts * steps},
        noise_yaw{rollouts * steps},
        soft_cost{rollouts},
        critical_exposure{rollouts},
        planning_exposure{rollouts},
        minimum_clearance{rollouts},
        worst_tier{rollouts},
        raw_collision{rollouts},
        solid_collision{rollouts},
        weights{rollouts},
        nominal{steps},
        updated{steps},
        control_update_partials{kControlUpdatePartitions * steps},
        best_eligible{steps},
        repair_candidates{kRepairCandidateCount * steps},
        best_tier{1U},
        best_rollout{1U},
        best_critical{1U},
        best_planning{1U},
        minimum_soft{1U},
        weight_sum{1U},
        dynamic_aircraft_samples{kMaximumDynamicAircraft * steps} {
  }

  [[nodiscard]] std::size_t bytes() const noexcept {
    return noise_ax.bytes() + noise_ay.bytes() + noise_az.bytes() + noise_yaw.bytes() +
           soft_cost.bytes() + critical_exposure.bytes() + planning_exposure.bytes() +
           minimum_clearance.bytes() + worst_tier.bytes() + raw_collision.bytes() +
           solid_collision.bytes() + weights.bytes() + nominal.bytes() +
           updated.bytes() + control_update_partials.bytes() + best_eligible.bytes() +
           repair_candidates.bytes() + best_tier.bytes() + best_rollout.bytes() +
           best_critical.bytes() + best_planning.bytes() + minimum_soft.bytes() +
           weight_sum.bytes() + solids.bytes() + route_points.bytes() +
           dynamic_aircraft_samples.bytes() + dynamic_aircraft_radii.bytes() +
           dynamic_aircraft_active_steps.bytes();
  }
};

#include "mppi_engine_host_validation.hpp"
#include "mppi_engine_kernels.cuh"

} // namespace

class MppiCudaEngine::Impl {
public:
  explicit Impl(BenchmarkConfig config)
      : config_{std::move(config)},
        buffers_{config_.rollouts, config_.steps},
        nominal_(config_.steps),
        updated_(config_.steps),
        best_eligible_(config_.steps),
        repair_candidates_(kRepairCandidateCount * config_.steps),
        reacquisition_candidate_(config_.steps),
        reacquisition_noise_ax_(config_.steps),
        reacquisition_noise_ay_(config_.steps),
        reacquisition_noise_az_(config_.steps),
        reacquisition_noise_yaw_(config_.steps),
        dynamic_aircraft_samples_(kMaximumDynamicAircraft * config_.steps),
        dynamic_aircraft_radii_(kMaximumDynamicAircraft),
        dynamic_aircraft_active_steps_(kMaximumDynamicAircraft),
        cooperative_candidates_(kCooperativeManeuverCandidateCount * config_.steps),
        cooperative_noise_ax_(kCooperativeManeuverCandidateCount * config_.steps),
        cooperative_noise_ay_(kCooperativeManeuverCandidateCount * config_.steps),
        cooperative_noise_az_(kCooperativeManeuverCandidateCount * config_.steps),
        cooperative_noise_yaw_(kCooperativeManeuverCandidateCount * config_.steps),
        zero_noise_(config_.steps) {
    if (!benchmarkConfigIsValid(config_)) {
      throw std::invalid_argument{"invalid MPPI engine configuration"};
    }
    checkCuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
              "cudaStreamCreateWithFlags");
    checkCuda(cudaMemcpyAsync(buffers_.nominal.get(), nominal_.data(),
                              nominal_.size() * sizeof(Control), cudaMemcpyHostToDevice,
                              stream_),
              "copy initial nominal controls");
    checkCuda(cudaStreamSynchronize(stream_), "initialize MPPI engine");
  }

  ~Impl() {
    if (stream_ != nullptr) {
      (void)cudaStreamDestroy(stream_);
    }
  }

  EsdfUploadResult updateEsdf(const EsdfSnapshot& snapshot) {
    std::scoped_lock lock{mutex_};
    const std::size_t expected =
        static_cast<std::size_t>(snapshot.grid.width) *
        static_cast<std::size_t>(snapshot.grid.height) *
        static_cast<std::size_t>(std::max(1, snapshot.grid.depth));
    if (snapshot.grid.width <= 1 || snapshot.grid.height <= 1 ||
        !(snapshot.grid.resolution_m > 0.0F) ||
        snapshot.distances_m.size() != expected) {
      return {};
    }
    const std::size_t inactive = 1U - active_texture_;
    const double upload_ms = textures_[inactive].upload(snapshot, stream_);
    esdf_host_[inactive].assign(snapshot.distances_m.begin(),
                                snapshot.distances_m.end());
    active_texture_ = inactive;
    return EsdfUploadResult{true, upload_ms, snapshot.revision};
  }

  void updateKnownSolids(std::span<const KnownSolid> solids) {
    std::scoped_lock lock{mutex_};
    if (solids.size() > kMaximumKnownSolids) {
      throw std::invalid_argument{"too many known solids for MPPI engine"};
    }
    known_solids_.assign(solids.begin(), solids.end());
    solid_count_ = solids.size();
    if (!solids.empty()) {
      checkCuda(cudaMemcpyAsync(buffers_.solids.get(), solids.data(),
                                solids.size_bytes(), cudaMemcpyHostToDevice, stream_),
                "upload known solids");
      checkCuda(cudaStreamSynchronize(stream_), "synchronize known solids upload");
    }
  }

  MppiTickResult plan(const MppiTickInput& input) {
    std::scoped_lock lock{mutex_};
    if (!textures_[active_texture_].ready()) {
      throw std::runtime_error{"MPPI engine has no ESDF"};
    }
    validateMppiTickInput(input, config_.steps, kMaximumDynamicAircraft);
    const auto host_started = std::chrono::steady_clock::now();
    const std::size_t active_rollouts =
        resolveMppiActiveRollouts(config_.rollouts, input.active_rollouts);
    const std::size_t noise_count = active_rollouts * config_.steps;
    const int noise_blocks =
        static_cast<int>((noise_count + kThreadsPerBlock - 1U) / kThreadsPerBlock);
    const int rollout_blocks =
        static_cast<int>((active_rollouts + kThreadsPerBlock - 1U) / kThreadsPerBlock);
    const int control_blocks =
        static_cast<int>((config_.steps + kThreadsPerBlock - 1U) / kThreadsPerBlock);
    const dim3 control_update_grid{
        static_cast<unsigned int>((config_.steps + kControlUpdateStepTile - 1U) /
                                  kControlUpdateStepTile),
        static_cast<unsigned int>(kControlUpdatePartitions), 1U};
    double elapsed_s = 0.0;
    if (has_updated_) {
      if (last_planning_stamp_ns_ > 0 &&
          input.planning_stamp_ns > last_planning_stamp_ns_) {
        elapsed_s =
            static_cast<double>(input.planning_stamp_ns - last_planning_stamp_ns_) /
            1.0e9;
      } else {
        elapsed_s = config_.dynamics.dt_s;
      }
    }
    const Control previous_applied_control = input.previous_applied_control.value_or(
        last_output_control_.value_or(Control{}));
    const float first_control_interval_s =
        has_updated_
            ? std::clamp(static_cast<float>(elapsed_s), 1.0e-3F, config_.dynamics.dt_s)
            : config_.dynamics.dt_s;
    const bool route_active = input.route.has_value() && input.route->points &&
                              input.route->points->size() >= 2U;
    const std::span<const RouteSample3D> active_route =
        route_active ? std::span<const RouteSample3D>{*input.route->points}
                     : std::span<const RouteSample3D>{};
    const MovingTargetReference moving_target =
        input.moving_target.value_or(MovingTargetReference{});
    const bool moving_target_enabled = input.moving_target.has_value();
    const bool external_nominal_reseeded =
        input.nominal_reseed_generation > nominal_reseed_generation_;
    if (external_nominal_reseeded) {
      nominal_ = buildGuideDirectedNominalSeed(
          input.initial_state, input.target, active_route,
          route_active ? input.route->initial_station_m : 0.0F,
          input.reference_speed_mps, config_.dynamics, config_.steps,
          previous_applied_control);
      nominal_reseed_generation_ = input.nominal_reseed_generation;
    } else if (has_updated_) {
      nominal_ = shiftControlSequence(updated_, config_.dynamics.dt_s, elapsed_s);
    }
    CooperativeSeparationAcquisitionLifecycleResult acquisition_lifecycle =
        cooperative_acquisition_lifecycle_.update(
            CooperativeSeparationAcquisitionLifecycleInput{
                .avoidance_active = input.cooperative_avoidance_active,
                .acquisition = input.cooperative_acquisition,
                .evaluation =
                    CooperativeSeparationAcquisitionEvaluationInput{
                        .initial_state = input.initial_state,
                        .target = input.target,
                        .route = active_route,
                        .initial_route_station_m =
                            route_active ? input.route->initial_station_m : 0.0F,
                        .reference_speed_mps =
                            std::max(0.0F, input.reference_speed_mps),
                        .previous_applied_control = previous_applied_control,
                        .first_control_interval_s = first_control_interval_s,
                        .grid = textures_[active_texture_].grid(),
                        .esdf = activeEsdfHost(),
                        .known_solids = known_solids_,
                        .aircraft = input.dynamic_aircraft,
                        .config = config_,
                    },
            });
    if (acquisition_lifecycle.nominal_reseed) {
      nominal_ = std::move(*acquisition_lifecycle.nominal_reseed);
    }
    CooperativeSeparationAcquisitionResult acquisition =
        std::move(acquisition_lifecycle.acquisition);
    const bool acquisition_reseeded = acquisition_lifecycle.acquisition_reseeded;
    const bool release_reseeded = acquisition_lifecycle.release_reseeded;
    if (route_active) {
      if (input.route->points->size() > kMaximumRoutePoints) {
        throw std::invalid_argument{"MPPI route exceeds device route capacity"};
      }
      if (!route_uploaded_ || route_generation_ != input.route->generation ||
          route_points_host_.get() != input.route->points.get()) {
        checkCuda(cudaMemcpyAsync(buffers_.route_points.get(),
                                  input.route->points->data(),
                                  input.route->points->size() * sizeof(RouteSample3D),
                                  cudaMemcpyHostToDevice, stream_),
                  "upload semantic route");
        route_points_host_ = input.route->points;
        route_generation_ = input.route->generation;
        route_point_count_ = input.route->points->size();
        route_uploaded_ = true;
      }
    }
    const bool target_directed_candidate =
        input.deterministic_candidate ==
        DeterministicCandidateKind::kTargetDirectedReacquisition;
    const bool route_directed_candidate =
        input.deterministic_candidate ==
            DeterministicCandidateKind::kRouteDirectedCruise &&
        route_active;
    const bool deterministic_candidate_enabled =
        active_rollouts > 0U && (target_directed_candidate || route_directed_candidate);
    if (deterministic_candidate_enabled) {
      reacquisition_candidate_ =
          route_directed_candidate
              ? buildRouteDirectedCruiseSeed(
                    input.initial_state, input.target, active_route,
                    input.route->initial_station_m, input.reference_speed_mps,
                    config_.dynamics, config_.steps, previous_applied_control)
              : buildGuideDirectedNominalSeed(input.initial_state, input.target, {},
                                              0.0F, input.reference_speed_mps,
                                              config_.dynamics, config_.steps,
                                              previous_applied_control);
      for (std::size_t step = 0U; step < config_.steps; ++step) {
        reacquisition_noise_ax_[step] =
            reacquisition_candidate_[step].ax - nominal_[step].ax;
        reacquisition_noise_ay_[step] =
            reacquisition_candidate_[step].ay - nominal_[step].ay;
        reacquisition_noise_az_[step] =
            reacquisition_candidate_[step].az - nominal_[step].az;
        reacquisition_noise_yaw_[step] =
            reacquisition_candidate_[step].yaw_accel - nominal_[step].yaw_accel;
      }
    }
    const bool cooperative_avoidance_enabled =
        !input.dynamic_aircraft.empty() &&
        active_rollouts >= kCooperativeManeuverCandidateCount +
                               (deterministic_candidate_enabled ? 1U : 0U);
    const std::size_t cooperative_candidate_offset =
        deterministic_candidate_enabled ? 1U : 0U;
    if (cooperative_avoidance_enabled) {
      cooperative_candidates_ = buildCooperativeManeuverCandidates(
          input.initial_state, input.target, nominal_, config_.dynamics,
          config_.cooperative, previous_applied_control, first_control_interval_s);
      for (std::size_t index = 0U; index < cooperative_candidates_.size(); ++index) {
        const std::size_t step = index % config_.steps;
        cooperative_noise_ax_[index] =
            cooperative_candidates_[index].ax - nominal_[step].ax;
        cooperative_noise_ay_[index] =
            cooperative_candidates_[index].ay - nominal_[step].ay;
        cooperative_noise_az_[index] =
            cooperative_candidates_[index].az - nominal_[step].az;
        cooperative_noise_yaw_[index] =
            cooperative_candidates_[index].yaw_accel - nominal_[step].yaw_accel;
      }
    }
    const std::size_t dynamic_aircraft_count = input.dynamic_aircraft.size();
    for (std::size_t aircraft_index = 0U; aircraft_index < dynamic_aircraft_count;
         ++aircraft_index) {
      const DynamicAircraftTrajectory& aircraft =
          input.dynamic_aircraft[aircraft_index];
      std::ranges::copy(*aircraft.samples, dynamic_aircraft_samples_.begin() +
                                               static_cast<std::ptrdiff_t>(
                                                   aircraft_index * config_.steps));
      dynamic_aircraft_radii_[aircraft_index] = aircraft.footprint_radius_m;
      dynamic_aircraft_active_steps_[aircraft_index] =
          static_cast<std::uint32_t>(aircraft.active_steps);
    }
    const Control cooperative_preferred_acceleration =
        input.cooperative_maneuver.has_value()
            ? resolveCooperativePreferredAcceleration(
                  *input.cooperative_maneuver, config_.dynamics, config_.cooperative)
            : Control{};
    const std::size_t cooperative_preference_steps = std::clamp<std::size_t>(
        static_cast<std::size_t>(std::ceil(config_.cooperative.candidate_duration_s /
                                           config_.dynamics.dt_s)),
        1U, config_.steps);

    started_.record(stream_);
    checkCuda(cudaMemcpyAsync(buffers_.nominal.get(), nominal_.data(),
                              nominal_.size() * sizeof(Control), cudaMemcpyHostToDevice,
                              stream_),
              "copy time-shifted nominal controls");
    if (dynamic_aircraft_count > 0U) {
      checkCuda(cudaMemcpyAsync(buffers_.dynamic_aircraft_samples.get(),
                                dynamic_aircraft_samples_.data(),
                                dynamic_aircraft_count * config_.steps *
                                    sizeof(DynamicAircraftSample),
                                cudaMemcpyHostToDevice, stream_),
                "upload dynamic aircraft trajectories");
      checkCuda(cudaMemcpyAsync(buffers_.dynamic_aircraft_radii.get(),
                                dynamic_aircraft_radii_.data(),
                                dynamic_aircraft_count * sizeof(float),
                                cudaMemcpyHostToDevice, stream_),
                "upload dynamic aircraft radii");
      checkCuda(cudaMemcpyAsync(buffers_.dynamic_aircraft_active_steps.get(),
                                dynamic_aircraft_active_steps_.data(),
                                dynamic_aircraft_count * sizeof(std::uint32_t),
                                cudaMemcpyHostToDevice, stream_),
                "upload dynamic aircraft active steps");
    }
    warm_done_.record(stream_);
    generateNoise<<<noise_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.noise_ax.get(), buffers_.noise_ay.get(), buffers_.noise_az.get(),
        buffers_.noise_yaw.get(), noise_count, config_.seed, tick_sequence_++,
        config_.noise);
    if (deterministic_candidate_enabled) {
      const std::size_t bytes = config_.steps * sizeof(float);
      checkCuda(cudaMemcpyAsync(buffers_.noise_ax.get(), reacquisition_noise_ax_.data(),
                                bytes, cudaMemcpyHostToDevice, stream_),
                "inject target-directed acceleration x");
      checkCuda(cudaMemcpyAsync(buffers_.noise_ay.get(), reacquisition_noise_ay_.data(),
                                bytes, cudaMemcpyHostToDevice, stream_),
                "inject target-directed acceleration y");
      checkCuda(cudaMemcpyAsync(buffers_.noise_az.get(), reacquisition_noise_az_.data(),
                                bytes, cudaMemcpyHostToDevice, stream_),
                "inject target-directed acceleration z");
      checkCuda(cudaMemcpyAsync(buffers_.noise_yaw.get(),
                                reacquisition_noise_yaw_.data(), bytes,
                                cudaMemcpyHostToDevice, stream_),
                "inject target-directed yaw acceleration");
    }
    if (cooperative_avoidance_enabled) {
      const std::size_t candidate_value_count =
          kCooperativeManeuverCandidateCount * config_.steps;
      const std::size_t destination_offset =
          cooperative_candidate_offset * config_.steps;
      const std::size_t bytes = candidate_value_count * sizeof(float);
      checkCuda(cudaMemcpyAsync(buffers_.noise_ax.get() + destination_offset,
                                cooperative_noise_ax_.data(), bytes,
                                cudaMemcpyHostToDevice, stream_),
                "inject cooperative acceleration x");
      checkCuda(cudaMemcpyAsync(buffers_.noise_ay.get() + destination_offset,
                                cooperative_noise_ay_.data(), bytes,
                                cudaMemcpyHostToDevice, stream_),
                "inject cooperative acceleration y");
      checkCuda(cudaMemcpyAsync(buffers_.noise_az.get() + destination_offset,
                                cooperative_noise_az_.data(), bytes,
                                cudaMemcpyHostToDevice, stream_),
                "inject cooperative acceleration z");
      checkCuda(cudaMemcpyAsync(buffers_.noise_yaw.get() + destination_offset,
                                cooperative_noise_yaw_.data(), bytes,
                                cudaMemcpyHostToDevice, stream_),
                "inject cooperative yaw acceleration");
    }
    noise_done_.record(stream_);
    simulate<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.noise_ax.get(), buffers_.noise_ay.get(), buffers_.noise_az.get(),
        buffers_.noise_yaw.get(), buffers_.nominal.get(), buffers_.soft_cost.get(),
        buffers_.critical_exposure.get(), buffers_.planning_exposure.get(),
        buffers_.minimum_clearance.get(), buffers_.worst_tier.get(),
        buffers_.raw_collision.get(), buffers_.solid_collision.get(), active_rollouts,
        config_.steps, input.initial_state, input.target, moving_target,
        moving_target_enabled, config_.dynamics, config_.risk, config_.footprint,
        config_.costs, config_.horizon_sampling, textures_[active_texture_].grid(),
        textures_[active_texture_].texture(), buffers_.solids.get(), solid_count_,
        buffers_.route_points.get(), route_active ? route_point_count_ : 0U,
        route_active ? input.route->initial_station_m : 0.0F,
        buffers_.dynamic_aircraft_samples.get(), buffers_.dynamic_aircraft_radii.get(),
        buffers_.dynamic_aircraft_active_steps.get(), dynamic_aircraft_count,
        config_.cooperative, cooperative_preferred_acceleration,
        cooperative_preference_steps, input.cooperative_maneuver.has_value(),
        previous_applied_control, first_control_interval_s, input.reference_speed_mps,
        config_.early_exit_on_collision, nullptr);
    simulation_done_.record(stream_);
    initializeReduction<<<1, 1, 0U, stream_>>>(
        buffers_.best_tier.get(), buffers_.best_critical.get(),
        buffers_.best_planning.get(), buffers_.minimum_soft.get(),
        buffers_.weight_sum.get(), buffers_.best_rollout.get());
    reduceTier<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.raw_collision.get(),
        buffers_.solid_collision.get(), active_rollouts,
        static_cast<int>(input.maximum_eligible_risk_tier), buffers_.best_tier.get());
    reduceCritical<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.critical_exposure.get(),
        buffers_.raw_collision.get(), buffers_.solid_collision.get(), active_rollouts,
        buffers_.best_tier.get(), buffers_.best_critical.get());
    reducePlanning<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.critical_exposure.get(),
        buffers_.planning_exposure.get(), buffers_.raw_collision.get(),
        buffers_.solid_collision.get(), active_rollouts, buffers_.best_tier.get(),
        buffers_.best_critical.get(), config_.risk.critical_exposure_tolerance_m,
        buffers_.best_planning.get());
    reduceSoft<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.critical_exposure.get(),
        buffers_.planning_exposure.get(), buffers_.soft_cost.get(),
        buffers_.raw_collision.get(), buffers_.solid_collision.get(), active_rollouts,
        buffers_.best_tier.get(), buffers_.best_critical.get(),
        buffers_.best_planning.get(), buffers_.minimum_soft.get(), config_.risk,
        static_cast<int>(input.maximum_eligible_risk_tier));
    reduction_done_.record(stream_);
    calculateWeights<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.critical_exposure.get(),
        buffers_.planning_exposure.get(), buffers_.soft_cost.get(),
        buffers_.raw_collision.get(), buffers_.solid_collision.get(),
        buffers_.weights.get(), active_rollouts, buffers_.best_tier.get(),
        buffers_.best_critical.get(), buffers_.best_planning.get(),
        buffers_.minimum_soft.get(), config_.risk, config_.costs.temperature,
        static_cast<int>(input.maximum_eligible_risk_tier), buffers_.weight_sum.get());
    selectBestEligibleRollout<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.weights.get(), buffers_.soft_cost.get(), buffers_.minimum_soft.get(),
        active_rollouts, buffers_.best_rollout.get());
    weights_done_.record(stream_);
    accumulateControlUpdatePartials<<<control_update_grid, kThreadsPerBlock, 0U,
                                      stream_>>>(
        buffers_.noise_ax.get(), buffers_.noise_ay.get(), buffers_.noise_az.get(),
        buffers_.noise_yaw.get(), buffers_.weights.get(),
        buffers_.control_update_partials.get(), active_rollouts, config_.steps,
        kControlUpdatePartitions);
    finalizeControlUpdate<<<control_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.nominal.get(), buffers_.updated.get(),
        buffers_.control_update_partials.get(), buffers_.weight_sum.get(),
        config_.steps, kControlUpdatePartitions);
    limitControls<<<1, 1, 0U, stream_>>>(buffers_.updated.get(), config_.steps,
                                         config_.dynamics, previous_applied_control,
                                         first_control_interval_s);
    buildBestEligibleControls<<<control_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.nominal.get(), buffers_.best_eligible.get(), buffers_.noise_ax.get(),
        buffers_.noise_ay.get(), buffers_.noise_az.get(), buffers_.noise_yaw.get(),
        buffers_.best_rollout.get(), active_rollouts, config_.steps);
    limitControls<<<1, 1, 0U, stream_>>>(buffers_.best_eligible.get(), config_.steps,
                                         config_.dynamics, previous_applied_control,
                                         first_control_interval_s);
    update_done_.record(stream_);
    int eligible_tier_value = static_cast<int>(RiskTier::kCollision);
    float best_critical_exposure_m = kInfinity;
    float best_planning_exposure_m = kInfinity;
    float eligible_weight_sum = 0.0F;
    int best_rollout_index = -1;
    std::uint8_t reacquisition_raw_collision = 1U;
    std::uint8_t reacquisition_solid_collision = 1U;
    float reacquisition_weight = 0.0F;
    checkCuda(cudaMemcpyAsync(updated_.data(), buffers_.updated.get(),
                              updated_.size() * sizeof(Control), cudaMemcpyDeviceToHost,
                              stream_),
              "copy selected controls");
    checkCuda(cudaMemcpyAsync(best_eligible_.data(), buffers_.best_eligible.get(),
                              best_eligible_.size() * sizeof(Control),
                              cudaMemcpyDeviceToHost, stream_),
              "copy best eligible controls");
    checkCuda(cudaMemcpyAsync(&eligible_tier_value, buffers_.best_tier.get(),
                              sizeof(eligible_tier_value), cudaMemcpyDeviceToHost,
                              stream_),
              "copy eligible risk tier");
    checkCuda(cudaMemcpyAsync(&best_critical_exposure_m, buffers_.best_critical.get(),
                              sizeof(best_critical_exposure_m), cudaMemcpyDeviceToHost,
                              stream_),
              "copy eligible critical exposure");
    checkCuda(cudaMemcpyAsync(&best_planning_exposure_m, buffers_.best_planning.get(),
                              sizeof(best_planning_exposure_m), cudaMemcpyDeviceToHost,
                              stream_),
              "copy eligible planning exposure");
    checkCuda(cudaMemcpyAsync(&eligible_weight_sum, buffers_.weight_sum.get(),
                              sizeof(eligible_weight_sum), cudaMemcpyDeviceToHost,
                              stream_),
              "copy eligible weight sum");
    if (deterministic_candidate_enabled) {
      checkCuda(cudaMemcpyAsync(&best_rollout_index, buffers_.best_rollout.get(),
                                sizeof(best_rollout_index), cudaMemcpyDeviceToHost,
                                stream_),
                "copy best rollout index");
      checkCuda(cudaMemcpyAsync(&reacquisition_raw_collision,
                                buffers_.raw_collision.get(),
                                sizeof(reacquisition_raw_collision),
                                cudaMemcpyDeviceToHost, stream_),
                "copy target-directed raw collision");
      checkCuda(cudaMemcpyAsync(&reacquisition_solid_collision,
                                buffers_.solid_collision.get(),
                                sizeof(reacquisition_solid_collision),
                                cudaMemcpyDeviceToHost, stream_),
                "copy target-directed solid collision");
      checkCuda(cudaMemcpyAsync(&reacquisition_weight, buffers_.weights.get(),
                                sizeof(reacquisition_weight), cudaMemcpyDeviceToHost,
                                stream_),
                "copy target-directed weight");
    }
    completed_.record(stream_);
    completed_.synchronize();
    checkCuda(cudaGetLastError(), "MPPI engine kernels");

    MppiTickResult result;
    const bool eligible_tier_available =
        eligible_tier_value >= static_cast<int>(RiskTier::kPreferred) &&
        eligible_tier_value < static_cast<int>(RiskTier::kCollision);
    if (input.maximum_eligible_risk_tier != RiskTier::kPreferred &&
        eligible_tier_available) {
      eligible_tier_value = static_cast<int>(input.maximum_eligible_risk_tier);
      best_critical_exposure_m = std::numeric_limits<float>::max() * 0.25F;
      best_planning_exposure_m = std::numeric_limits<float>::max() * 0.25F;
    }
    result.eligible_risk_contract = MppiEligibleRiskContract{
        .available = eligible_tier_available && eligible_weight_sum > 0.0F,
        .tier = eligible_tier_available ? static_cast<RiskTier>(eligible_tier_value)
                                        : RiskTier::kCollision,
        .best_critical_exposure_m = best_critical_exposure_m,
        .best_planning_exposure_m = best_planning_exposure_m,
        .critical_exposure_tolerance_m = config_.risk.critical_exposure_tolerance_m,
        .planning_exposure_tolerance_m = config_.risk.planning_exposure_tolerance_m,
        .weight_sum = eligible_weight_sum,
    };
    result.target_directed_candidate_injected = target_directed_candidate;
    result.target_directed_candidate_raw_safe = target_directed_candidate &&
                                                reacquisition_raw_collision == 0U &&
                                                reacquisition_solid_collision == 0U;
    result.target_directed_candidate_best_eligible =
        target_directed_candidate && best_rollout_index == 0;
    result.target_directed_candidate_weight =
        target_directed_candidate ? reacquisition_weight : 0.0F;
    result.route_directed_candidate_injected = route_directed_candidate;
    result.route_directed_candidate_raw_safe = route_directed_candidate &&
                                               reacquisition_raw_collision == 0U &&
                                               reacquisition_solid_collision == 0U;
    result.route_directed_candidate_best_eligible =
        route_directed_candidate && best_rollout_index == 0;
    result.route_directed_candidate_weight =
        route_directed_candidate ? reacquisition_weight : 0.0F;
    result.route_directed_candidate_generation =
        route_directed_candidate ? input.route->generation : 0U;
    result.cooperative_acquisition_reseeded = acquisition_reseeded;
    result.cooperative_release_reseeded = release_reseeded;
    result.cooperative_acquisition_available = acquisition.available;
    result.cooperative_acquisition_positive_progress = acquisition.positive_progress;
    result.cooperative_acquisition_backward_fallback = acquisition.backward_fallback;
    result.cooperative_acquisition_candidate_index = acquisition.candidate_index;
    result.cooperative_acquisition_head_progress_m = acquisition.head_progress_m;
    result.cooperative_acquisition_terminal_progress_m =
        acquisition.terminal_progress_m;
    result.cooperative_acquisition_separation_gain_m = acquisition.separation_gain_m;
    result.cooperative_candidates_injected = cooperative_avoidance_enabled;
    result.dynamic_aircraft_count = dynamic_aircraft_count;
    const auto evaluate_controls = [&](const std::span<const Control> controls) {
      EvaluatedControlSequence evaluation;
      evaluation.metrics = simulateReference(
          input.initial_state, controls, zero_noise_, config_.dynamics, config_.risk,
          config_.costs, textures_[active_texture_].grid(), activeEsdfHost(),
          input.target.x, input.target.y, config_.early_exit_on_collision,
          previous_applied_control, input.reference_speed_mps, config_.footprint,
          input.moving_target, &evaluation.trace, input.dynamic_aircraft,
          input.cooperative_maneuver, config_.cooperative);
      evaluation.known_solid_collision = hostSweptSolidCollision(
          evaluation.trace.horizon, controls, config_.footprint, known_solids_);
      evaluation.classification = classifyMppiPostUpdate(
          result.eligible_risk_contract,
          MppiPostUpdateObservation{
              .tier = evaluation.known_solid_collision ? RiskTier::kCollision
                                                       : evaluation.metrics.worst_tier,
              .raw_collision = evaluation.metrics.collision,
              .known_solid_collision = evaluation.known_solid_collision,
              .critical_exposure_m = evaluation.metrics.critical_exposure_m,
              .planning_exposure_m = evaluation.metrics.planning_exposure_m,
          });
      return evaluation;
    };
    const auto post_update_evaluation_started = std::chrono::steady_clock::now();
    EvaluatedControlSequence selected_evaluation = evaluate_controls(updated_);
    double repair_validation_ms{0.0};
    if (!selected_evaluation.classification.contract_preserved &&
        result.eligible_risk_contract.available) {
      std::vector<Control> limited_nominal = nominal_;
      limitControlSequence(limited_nominal, config_.dynamics, previous_applied_control,
                           first_control_interval_s);
      constexpr std::array backtrack_ratios{0.5F, 0.25F, 0.125F, 0.0625F, 0.0F};
      for (std::size_t candidate_index = 0U; candidate_index < backtrack_ratios.size();
           ++candidate_index) {
        const float ratio = backtrack_ratios[candidate_index];
        std::span<Control> candidate{
            repair_candidates_.data() + candidate_index * config_.steps, config_.steps};
        for (std::size_t index = 0U; index < candidate.size(); ++index) {
          candidate[index] =
              interpolateControl(limited_nominal[index], updated_[index], ratio);
        }
        limitControlSequence(candidate, config_.dynamics, previous_applied_control,
                             first_control_interval_s);
      }
      std::ranges::copy(
          best_eligible_,
          repair_candidates_.begin() +
              static_cast<std::ptrdiff_t>(backtrack_ratios.size() * config_.steps));

      repair_started_.record(stream_);
      checkCuda(cudaMemcpyAsync(buffers_.repair_candidates.get(),
                                repair_candidates_.data(),
                                repair_candidates_.size() * sizeof(Control),
                                cudaMemcpyHostToDevice, stream_),
                "upload post-update repair candidates");
      constexpr std::size_t candidate_count = kRepairCandidateCount;
      const int repair_blocks = static_cast<int>(
          (candidate_count + kThreadsPerBlock - 1U) / kThreadsPerBlock);
      simulate<<<repair_blocks, kThreadsPerBlock, 0U, stream_>>>(
          buffers_.noise_ax.get(), buffers_.noise_ay.get(), buffers_.noise_az.get(),
          buffers_.noise_yaw.get(), buffers_.nominal.get(), buffers_.soft_cost.get(),
          buffers_.critical_exposure.get(), buffers_.planning_exposure.get(),
          buffers_.minimum_clearance.get(), buffers_.worst_tier.get(),
          buffers_.raw_collision.get(), buffers_.solid_collision.get(), candidate_count,
          config_.steps, input.initial_state, input.target, moving_target,
          moving_target_enabled, config_.dynamics, config_.risk, config_.footprint,
          config_.costs, config_.horizon_sampling, textures_[active_texture_].grid(),
          textures_[active_texture_].texture(), buffers_.solids.get(), solid_count_,
          buffers_.route_points.get(), route_active ? route_point_count_ : 0U,
          route_active ? input.route->initial_station_m : 0.0F,
          buffers_.dynamic_aircraft_samples.get(),
          buffers_.dynamic_aircraft_radii.get(),
          buffers_.dynamic_aircraft_active_steps.get(), dynamic_aircraft_count,
          config_.cooperative, cooperative_preferred_acceleration,
          cooperative_preference_steps, input.cooperative_maneuver.has_value(),
          previous_applied_control, first_control_interval_s, input.reference_speed_mps,
          config_.early_exit_on_collision, buffers_.repair_candidates.get());
      checkCuda(cudaMemcpyAsync(
                    repair_critical_exposure_.data(), buffers_.critical_exposure.get(),
                    candidate_count * sizeof(float), cudaMemcpyDeviceToHost, stream_),
                "copy repair critical exposure");
      checkCuda(cudaMemcpyAsync(
                    repair_planning_exposure_.data(), buffers_.planning_exposure.get(),
                    candidate_count * sizeof(float), cudaMemcpyDeviceToHost, stream_),
                "copy repair planning exposure");
      checkCuda(cudaMemcpyAsync(repair_worst_tier_.data(), buffers_.worst_tier.get(),
                                candidate_count * sizeof(std::uint8_t),
                                cudaMemcpyDeviceToHost, stream_),
                "copy repair risk tiers");
      checkCuda(cudaMemcpyAsync(repair_raw_collision_.data(),
                                buffers_.raw_collision.get(),
                                candidate_count * sizeof(std::uint8_t),
                                cudaMemcpyDeviceToHost, stream_),
                "copy repair raw collisions");
      checkCuda(cudaMemcpyAsync(repair_solid_collision_.data(),
                                buffers_.solid_collision.get(),
                                candidate_count * sizeof(std::uint8_t),
                                cudaMemcpyDeviceToHost, stream_),
                "copy repair solid collisions");
      repair_done_.record(stream_);
      repair_done_.synchronize();
      checkCuda(cudaGetLastError(), "post-update repair validation");
      repair_validation_ms = elapsedMs(repair_started_, repair_done_);

      bool repaired{false};
      for (std::size_t candidate_index = 0U; candidate_index < candidate_count;
           ++candidate_index) {
        const bool solid_collision = repair_solid_collision_[candidate_index] != 0U;
        const MppiPostUpdateClassificationResult gpu_classification =
            classifyMppiPostUpdate(
                result.eligible_risk_contract,
                MppiPostUpdateObservation{
                    .tier = solid_collision ? RiskTier::kCollision
                                            : static_cast<RiskTier>(
                                                  repair_worst_tier_[candidate_index]),
                    .raw_collision = repair_raw_collision_[candidate_index] != 0U,
                    .known_solid_collision = solid_collision,
                    .critical_exposure_m = repair_critical_exposure_[candidate_index],
                    .planning_exposure_m = repair_planning_exposure_[candidate_index],
                });
        if (!gpu_classification.contract_preserved) {
          continue;
        }
        const std::span<const Control> candidate{
            repair_candidates_.data() + candidate_index * config_.steps, config_.steps};
        EvaluatedControlSequence confirmed = evaluate_controls(candidate);
        if (!confirmed.classification.contract_preserved) {
          continue;
        }
        std::ranges::copy(candidate, updated_.begin());
        selected_evaluation = std::move(confirmed);
        if (candidate_index < backtrack_ratios.size()) {
          result.post_update_repair = MppiPostUpdateRepair::kBacktracked;
          result.post_update_backtrack_ratio = backtrack_ratios[candidate_index];
        } else {
          result.post_update_repair = MppiPostUpdateRepair::kBestEligibleRollout;
          result.post_update_backtrack_ratio = 0.0F;
        }
        repaired = true;
        break;
      }
      if (!repaired) {
        result.post_update_repair = MppiPostUpdateRepair::kFailed;
      }
    }
    result.controls = updated_;
    result.warm_start_shift_s = elapsed_s;
    result.nominal_reseeded =
        external_nominal_reseeded || acquisition_reseeded || release_reseeded;
    result.esdf_revision = textures_[active_texture_].revision();
    result.active_rollouts = active_rollouts;
    result.timings.warm_start_ms = elapsedMs(started_, warm_done_);
    result.timings.noise_generation_ms = elapsedMs(warm_done_, noise_done_);
    result.timings.rollout_simulation_ms = elapsedMs(noise_done_, simulation_done_);
    result.timings.risk_reduction_ms = elapsedMs(simulation_done_, reduction_done_);
    result.timings.weight_calculation_ms = elapsedMs(reduction_done_, weights_done_);
    result.timings.control_update_ms = elapsedMs(weights_done_, update_done_);
    result.timings.repair_validation_ms = repair_validation_ms;
    result.timings.post_update_evaluation_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  post_update_evaluation_started)
            .count();
    result.timings.gpu_total_ms =
        elapsedMs(started_, completed_) + repair_validation_ms;
    const auto reconstruction_started = std::chrono::steady_clock::now();
    result.horizon = std::move(selected_evaluation.trace.horizon);
    if (result.horizon.size() != updated_.size() + 1U) {
      throw std::runtime_error{"MPPI control evaluation returned incomplete horizon"};
    }
    State state = result.horizon.front();
    const float initial_distance =
        std::hypot(input.target.x - state.x, input.target.y - state.y);
    float fixed_target_head_progress_m = 0.0F;
    float reconstruction_route_station_m =
        input.route.has_value() ? input.route->initial_station_m : 0.0F;
    std::optional<float> latest_route_station;
    const RolloutMetrics& metrics = selected_evaluation.metrics;
    result.raw_collision = metrics.collision;
    result.known_solid_collision = selected_evaluation.known_solid_collision;
    result.critical_exposure_m = metrics.critical_exposure_m;
    result.planning_exposure_m = metrics.planning_exposure_m;
    result.minimum_esdf_distance_m = metrics.minimum_clearance_m;
    result.minimum_target_separation_m = metrics.minimum_target_separation_m;
    result.minimum_peer_separation_m = metrics.minimum_peer_separation_m;
    result.peer_separation_cost = metrics.costs.peer_separation;
    result.predicted_capture_time_s = metrics.predicted_capture_time_s;
    result.selected_tier =
        result.known_solid_collision ? RiskTier::kCollision : metrics.worst_tier;
    result.post_update_classification = selected_evaluation.classification;
    Control previous_control = previous_applied_control;
    if (!updated_.empty()) {
      result.first_control_delta =
          std::hypot(std::hypot(updated_.front().ax - previous_applied_control.ax,
                                updated_.front().ay - previous_applied_control.ay),
                     updated_.front().az - previous_applied_control.az);
    }
    const std::size_t head_steps = std::clamp<std::size_t>(
        static_cast<std::size_t>(
            std::ceil(config_.costs.head_progress_horizon_s / config_.dynamics.dt_s)),
        1U, updated_.size());
    for (std::size_t index = 0U; index < updated_.size(); ++index) {
      const Control& control = updated_[index];
      result.maximum_acceleration_mps2 =
          std::max(result.maximum_acceleration_mps2,
                   std::hypot(std::hypot(control.ax, control.ay), control.az));
      result.maximum_jerk_mps3 = std::max(
          result.maximum_jerk_mps3,
          std::hypot(std::hypot(control.ax - previous_control.ax,
                                control.ay - previous_control.ay),
                     control.az - previous_control.az) /
              (index == 0U ? first_control_interval_s : config_.dynamics.dt_s));
      previous_control = control;
      state = result.horizon[index + 1U];
      if (input.route.has_value() && input.route->points) {
        latest_route_station = projectForwardRouteStation(
            *input.route->points, state, reconstruction_route_station_m);
        if (latest_route_station.has_value()) {
          reconstruction_route_station_m = *latest_route_station;
        }
      }
      if (index + 1U == head_steps) {
        if (latest_route_station.has_value()) {
          result.head_progress_m =
              *latest_route_station - input.route->initial_station_m;
        } else {
          fixed_target_head_progress_m =
              initial_distance -
              std::hypot(input.target.x - state.x, input.target.y - state.y);
        }
      }
    }
    if (latest_route_station.has_value()) {
      result.terminal_progress_m =
          *latest_route_station - input.route->initial_station_m;
    } else {
      const MppiProgressDiagnostics progress = resolveUnroutedProgressDiagnostics(
          metrics, moving_target_enabled, fixed_target_head_progress_m,
          initial_distance -
              std::hypot(input.target.x - state.x, input.target.y - state.y));
      result.head_progress_m = progress.head_progress_m;
      result.terminal_progress_m = progress.terminal_progress_m;
    }
    result.timings.horizon_reconstruction_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  reconstruction_started)
            .count();
    result.timings.host_total_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - host_started)
                                       .count();
    has_updated_ = true;
    last_planning_stamp_ns_ = input.planning_stamp_ns;
    if (!updated_.empty()) {
      last_output_control_ = updated_.front();
    }
    return result;
  }

  [[nodiscard]] std::size_t allocatedBytes() const noexcept {
    return buffers_.bytes();
  }

  [[nodiscard]] bool ready() const noexcept {
    return textures_[active_texture_].ready();
  }

private:
  [[nodiscard]] std::span<const float> activeEsdfHost() const {
    return active_texture_ == 0U ? esdf_host_[0] : esdf_host_[1];
  }

  BenchmarkConfig config_;
  DeviceBuffers buffers_;
  EsdfTexture textures_[2];
  std::vector<float> esdf_host_[2];
  std::size_t active_texture_{0U};
  std::vector<KnownSolid> known_solids_;
  std::size_t solid_count_{0U};
  std::shared_ptr<const std::vector<RouteSample3D>> route_points_host_;
  std::size_t route_point_count_{0U};
  std::uint64_t route_generation_{0U};
  bool route_uploaded_{false};
  std::vector<Control> nominal_;
  std::vector<Control> updated_;
  std::vector<Control> best_eligible_;
  std::vector<Control> repair_candidates_;
  std::vector<Control> reacquisition_candidate_;
  std::vector<float> reacquisition_noise_ax_;
  std::vector<float> reacquisition_noise_ay_;
  std::vector<float> reacquisition_noise_az_;
  std::vector<float> reacquisition_noise_yaw_;
  std::vector<DynamicAircraftSample> dynamic_aircraft_samples_;
  std::vector<float> dynamic_aircraft_radii_;
  std::vector<std::uint32_t> dynamic_aircraft_active_steps_;
  std::vector<Control> cooperative_candidates_;
  std::vector<float> cooperative_noise_ax_;
  std::vector<float> cooperative_noise_ay_;
  std::vector<float> cooperative_noise_az_;
  std::vector<float> cooperative_noise_yaw_;
  std::array<float, kRepairCandidateCount> repair_critical_exposure_{};
  std::array<float, kRepairCandidateCount> repair_planning_exposure_{};
  std::array<std::uint8_t, kRepairCandidateCount> repair_worst_tier_{};
  std::array<std::uint8_t, kRepairCandidateCount> repair_raw_collision_{};
  std::array<std::uint8_t, kRepairCandidateCount> repair_solid_collision_{};
  std::vector<Control> zero_noise_;
  std::optional<Control> last_output_control_;
  std::int64_t last_planning_stamp_ns_{0};
  std::uint64_t nominal_reseed_generation_{0U};
  CooperativeSeparationAcquisitionLifecycle cooperative_acquisition_lifecycle_;
  bool has_updated_{false};
  std::uint64_t tick_sequence_{0U};
  cudaStream_t stream_{nullptr};
  Event started_;
  Event noise_done_;
  Event simulation_done_;
  Event reduction_done_;
  Event weights_done_;
  Event update_done_;
  Event warm_done_;
  Event completed_;
  Event repair_started_;
  Event repair_done_;
  mutable std::mutex mutex_;
};

MppiCudaEngine::MppiCudaEngine(BenchmarkConfig config)
    : impl_{std::make_unique<Impl>(std::move(config))} {
}

MppiCudaEngine::~MppiCudaEngine() = default;
MppiCudaEngine::MppiCudaEngine(MppiCudaEngine&&) noexcept = default;
MppiCudaEngine& MppiCudaEngine::operator=(MppiCudaEngine&&) noexcept = default;

EsdfUploadResult MppiCudaEngine::updateEsdf(const EsdfSnapshot& snapshot) {
  return impl_->updateEsdf(snapshot);
}

void MppiCudaEngine::updateKnownSolids(std::span<const KnownSolid> solids) {
  impl_->updateKnownSolids(solids);
}

MppiTickResult MppiCudaEngine::plan(const MppiTickInput& input) {
  return impl_->plan(input);
}

std::size_t MppiCudaEngine::allocatedBytes() const noexcept {
  return impl_->allocatedBytes();
}

bool MppiCudaEngine::ready() const noexcept {
  return impl_->ready();
}

} // namespace drone_city_nav::mppi
