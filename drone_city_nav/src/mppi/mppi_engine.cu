#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
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
#include <string>
#include <utility>

namespace drone_city_nav::mppi {
namespace {

constexpr int kThreadsPerBlock{256};
constexpr std::size_t kMaximumKnownSolids{2048U};
constexpr std::size_t kMaximumRoutePoints{512U};
constexpr std::size_t kRepairCandidateCount{6U};
constexpr float kPi{3.14159265358979323846F};
constexpr float kInfinity{std::numeric_limits<float>::infinity()};

void checkCuda(const cudaError_t error, const char* const operation) {
  if (error != cudaSuccess) {
    throw std::runtime_error{std::string{operation} + ": " + cudaGetErrorString(error)};
  }
}

template<typename T> class DeviceBuffer {
public:
  DeviceBuffer() = default;

  explicit DeviceBuffer(const std::size_t count) {
    allocate(count);
  }

  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;

  ~DeviceBuffer() {
    if (data_ != nullptr) {
      (void)cudaFree(data_);
    }
  }

  void allocate(const std::size_t count) {
    if (data_ != nullptr) {
      (void)cudaFree(data_);
    }
    count_ = count;
    checkCuda(cudaMalloc(reinterpret_cast<void**>(&data_), count * sizeof(T)),
              "cudaMalloc");
  }

  [[nodiscard]] T* get() noexcept {
    return data_;
  }

  [[nodiscard]] const T* get() const noexcept {
    return data_;
  }

  [[nodiscard]] std::size_t bytes() const noexcept {
    return count_ * sizeof(T);
  }

private:
  T* data_{nullptr};
  std::size_t count_{0U};
};

class Event {
public:
  Event() {
    checkCuda(cudaEventCreate(&event_), "cudaEventCreate");
  }

  Event(const Event&) = delete;
  Event& operator=(const Event&) = delete;

  ~Event() {
    if (event_ != nullptr) {
      (void)cudaEventDestroy(event_);
    }
  }

  void record(cudaStream_t stream) {
    checkCuda(cudaEventRecord(event_, stream), "cudaEventRecord");
  }

  void synchronize() {
    checkCuda(cudaEventSynchronize(event_), "cudaEventSynchronize");
  }

  [[nodiscard]] cudaEvent_t get() const noexcept {
    return event_;
  }

private:
  cudaEvent_t event_{nullptr};
};

[[nodiscard]] double elapsedMs(const Event& start, const Event& end) {
  float elapsed_ms = 0.0F;
  checkCuda(cudaEventElapsedTime(&elapsed_ms, start.get(), end.get()),
            "cudaEventElapsedTime");
  return elapsed_ms;
}

class EsdfTexture {
public:
  EsdfTexture() = default;
  EsdfTexture(const EsdfTexture&) = delete;
  EsdfTexture& operator=(const EsdfTexture&) = delete;

  ~EsdfTexture() {
    reset();
  }

  double upload(const EsdfSnapshot& snapshot, cudaStream_t stream) {
    const auto started = std::chrono::steady_clock::now();
    if (array_ == nullptr || grid_.width != snapshot.grid.width ||
        grid_.height != snapshot.grid.height || grid_.depth != snapshot.grid.depth) {
      reset();
      const cudaChannelFormatDesc channel = cudaCreateChannelDesc<float>();
      const cudaExtent extent =
          make_cudaExtent(static_cast<std::size_t>(snapshot.grid.width),
                          static_cast<std::size_t>(snapshot.grid.height),
                          static_cast<std::size_t>(std::max(1, snapshot.grid.depth)));
      checkCuda(cudaMalloc3DArray(&array_, &channel, extent), "cudaMalloc3DArray");
      cudaResourceDesc resource{};
      resource.resType = cudaResourceTypeArray;
      resource.res.array.array = array_;
      cudaTextureDesc texture_description{};
      texture_description.addressMode[0] = cudaAddressModeBorder;
      texture_description.addressMode[1] = cudaAddressModeBorder;
      texture_description.addressMode[2] = cudaAddressModeBorder;
      texture_description.filterMode = cudaFilterModePoint;
      texture_description.readMode = cudaReadModeElementType;
      texture_description.normalizedCoords = 0;
      checkCuda(
          cudaCreateTextureObject(&texture_, &resource, &texture_description, nullptr),
          "cudaCreateTextureObject");
    }
    cudaMemcpy3DParms copy{};
    copy.srcPtr = make_cudaPitchedPtr(const_cast<float*>(snapshot.distances_m.data()),
                                      static_cast<std::size_t>(snapshot.grid.width) *
                                          sizeof(float),
                                      static_cast<std::size_t>(snapshot.grid.width),
                                      static_cast<std::size_t>(snapshot.grid.height));
    copy.dstArray = array_;
    copy.extent =
        make_cudaExtent(static_cast<std::size_t>(snapshot.grid.width),
                        static_cast<std::size_t>(snapshot.grid.height),
                        static_cast<std::size_t>(std::max(1, snapshot.grid.depth)));
    copy.kind = cudaMemcpyHostToDevice;
    checkCuda(cudaMemcpy3DAsync(&copy, stream), "cudaMemcpy3DAsync");
    checkCuda(cudaStreamSynchronize(stream), "synchronize ESDF upload");
    grid_ = snapshot.grid;
    revision_ = snapshot.revision;
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                     started)
        .count();
  }

  [[nodiscard]] cudaTextureObject_t texture() const noexcept {
    return texture_;
  }

  [[nodiscard]] const EsdfGrid& grid() const noexcept {
    return grid_;
  }

  [[nodiscard]] std::uint64_t revision() const noexcept {
    return revision_;
  }

  [[nodiscard]] bool ready() const noexcept {
    return texture_ != 0U;
  }

private:
  void reset() noexcept {
    if (texture_ != 0U) {
      (void)cudaDestroyTextureObject(texture_);
    }
    if (array_ != nullptr) {
      (void)cudaFreeArray(array_);
    }
    texture_ = 0U;
    array_ = nullptr;
  }

  cudaArray_t array_{nullptr};
  cudaTextureObject_t texture_{0U};
  EsdfGrid grid_{};
  std::uint64_t revision_{0U};
};

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
        best_eligible{steps},
        repair_candidates{kRepairCandidateCount * steps},
        best_tier{1U},
        best_rollout{1U},
        best_critical{1U},
        best_planning{1U},
        minimum_soft{1U},
        weight_sum{1U} {
  }

  [[nodiscard]] std::size_t bytes() const noexcept {
    return noise_ax.bytes() + noise_ay.bytes() + noise_az.bytes() + noise_yaw.bytes() +
           soft_cost.bytes() + critical_exposure.bytes() + planning_exposure.bytes() +
           minimum_clearance.bytes() + worst_tier.bytes() + raw_collision.bytes() +
           solid_collision.bytes() + weights.bytes() + nominal.bytes() +
           updated.bytes() + best_eligible.bytes() + repair_candidates.bytes() +
           best_tier.bytes() + best_rollout.bytes() + best_critical.bytes() +
           best_planning.bytes() + minimum_soft.bytes() + weight_sum.bytes() +
           solids.bytes() + route_points.bytes();
  }
};

#include "mppi_engine_kernels.cuh"

[[nodiscard]] SweptFootprintConfig
hostFootprintConfig(const FootprintConfig& footprint) noexcept {
  return SweptFootprintConfig{
      .radius_m = footprint.radius_m,
      .lower_extent_m = footprint.lower_extent_m,
      .upper_extent_m = footprint.upper_extent_m,
      .perimeter_samples = footprint.perimeter_samples,
      .radial_rings = footprint.radial_rings,
      .axial_samples = footprint.axial_samples,
  };
}

[[nodiscard]] FootprintBodyAxis hostBodyAxis(const Control& control) noexcept {
  return bodyAxisFromWorldAcceleration(Vec3{control.ax, control.ay, control.az});
}

[[nodiscard]] bool hostSolidCollision(const State& state,
                                      const FootprintBodyAxis& body_axis,
                                      const FootprintConfig& footprint_config,
                                      const std::span<const KnownSolid> solids) {
  const SweptFootprintConfig footprint = hostFootprintConfig(footprint_config);
  const auto overlaps_projection = [&](const double center_projection,
                                       const double axis_projection,
                                       const double solid_min, const double solid_max) {
    const double projection = std::clamp(axis_projection, -1.0, 1.0);
    const double radial_projection =
        footprint.radius_m * std::sqrt(std::max(0.0, 1.0 - projection * projection));
    const double maximum_axial = projection >= 0.0
                                     ? footprint.upper_extent_m * projection
                                     : -footprint.lower_extent_m * projection;
    const double minimum_axial = projection >= 0.0
                                     ? -footprint.lower_extent_m * projection
                                     : footprint.upper_extent_m * projection;
    return center_projection + maximum_axial + radial_projection >= solid_min &&
           center_projection + minimum_axial - radial_projection <= solid_max;
  };
  return std::ranges::any_of(solids, [&](const KnownSolid& solid) {
    if (!(footprint.radius_m > 0.0)) {
      if (state.z < solid.min_z_m || state.z > solid.max_z_m) {
        return false;
      }
      const float dx = state.x - solid.center_x_m;
      const float dy = state.y - solid.center_y_m;
      return std::abs(dx * solid.normal_x + dy * solid.normal_y) <=
                 solid.half_depth_m &&
             std::abs(dx * solid.lateral_x + dy * solid.lateral_y) <=
                 solid.half_width_m;
    }
    const float dx = state.x - solid.center_x_m;
    const float dy = state.y - solid.center_y_m;
    const double depth = dx * solid.normal_x + dy * solid.normal_y;
    const double lateral = dx * solid.lateral_x + dy * solid.lateral_y;
    return overlaps_projection(
               depth, body_axis.x * solid.normal_x + body_axis.y * solid.normal_y,
               -solid.half_depth_m, solid.half_depth_m) &&
           overlaps_projection(
               lateral, body_axis.x * solid.lateral_x + body_axis.y * solid.lateral_y,
               -solid.half_width_m, solid.half_width_m) &&
           overlaps_projection(state.z, body_axis.z, solid.min_z_m, solid.max_z_m);
  });
}

[[nodiscard]] bool hostSweptSolidCollision(const std::span<const State> horizon,
                                           const std::span<const Control> controls,
                                           const FootprintConfig& footprint,
                                           const std::span<const KnownSolid> solids) {
  if (horizon.size() != controls.size() + 1U) {
    return true;
  }
  for (std::size_t index = 0U; index < controls.size(); ++index) {
    const Control& control = controls[index];
    const FootprintBodyAxis body_axis = hostBodyAxis(control);
    const State& previous = horizon[index];
    const State& next = horizon[index + 1U];
    const float segment_length_m = std::hypot(next.x - previous.x, next.y - previous.y);
    const std::size_t samples = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(segment_length_m / 0.25F)));
    for (std::size_t sample = 1U; sample <= samples; ++sample) {
      const float ratio = static_cast<float>(sample) / static_cast<float>(samples);
      State state = next;
      state.x = std::lerp(previous.x, next.x, ratio);
      state.y = std::lerp(previous.y, next.y, ratio);
      state.z = std::lerp(previous.z, next.z, ratio);
      if (hostSolidCollision(state, body_axis, footprint, solids)) {
        return true;
      }
    }
  }
  return false;
}

struct EvaluatedControlSequence {
  ReferenceSimulationTrace trace;
  RolloutMetrics metrics{};
  MppiPostUpdateClassificationResult classification{};
  bool known_solid_collision{false};
};

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
    if (input.moving_target.has_value()) {
      const MovingTargetReference& moving_target = input.moving_target.value();
      if (!std::isfinite(moving_target.state.x) ||
          !std::isfinite(moving_target.state.y) ||
          !std::isfinite(moving_target.state.z) ||
          !std::isfinite(moving_target.state.vx) ||
          !std::isfinite(moving_target.state.vy) ||
          !std::isfinite(moving_target.state.vz) ||
          !(moving_target.capture_radius_m > 0.0F) ||
          (moving_target.bounded_vertical_motion &&
           (!std::isfinite(moving_target.vertical_deceleration_mps2) ||
            !(moving_target.vertical_deceleration_mps2 > 0.0F) ||
            !std::isfinite(moving_target.minimum_z_m) ||
            !std::isfinite(moving_target.maximum_z_m) ||
            !(moving_target.maximum_z_m > moving_target.minimum_z_m) ||
            moving_target.state.z < moving_target.minimum_z_m ||
            moving_target.state.z > moving_target.maximum_z_m))) {
        throw std::invalid_argument{"invalid moving target reference"};
      }
    }
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
    const MovingTargetReference moving_target =
        input.moving_target.value_or(MovingTargetReference{});
    const bool moving_target_enabled = input.moving_target.has_value();
    const bool nominal_reseeded =
        input.nominal_reseed_generation > nominal_reseed_generation_;
    if (nominal_reseeded) {
      const std::span<const RouteSample3D> route =
          route_active ? std::span<const RouteSample3D>{*input.route->points}
                       : std::span<const RouteSample3D>{};
      nominal_ = buildGuideDirectedNominalSeed(
          input.initial_state, input.target, route,
          route_active ? input.route->initial_station_m : 0.0F,
          input.reference_speed_mps, config_.dynamics, config_.steps,
          previous_applied_control);
      nominal_reseed_generation_ = input.nominal_reseed_generation;
    } else if (has_updated_) {
      nominal_ = shiftControlSequence(updated_, config_.dynamics.dt_s, elapsed_s);
    }
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

    started_.record(stream_);
    checkCuda(cudaMemcpyAsync(buffers_.nominal.get(), nominal_.data(),
                              nominal_.size() * sizeof(Control), cudaMemcpyHostToDevice,
                              stream_),
              "copy time-shifted nominal controls");
    warm_done_.record(stream_);
    generateNoise<<<noise_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.noise_ax.get(), buffers_.noise_ay.get(), buffers_.noise_az.get(),
        buffers_.noise_yaw.get(), noise_count, config_.seed, tick_sequence_++,
        config_.noise);
    noise_done_.record(stream_);
    simulate<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.noise_ax.get(), buffers_.noise_ay.get(), buffers_.noise_az.get(),
        buffers_.noise_yaw.get(), buffers_.nominal.get(), buffers_.soft_cost.get(),
        buffers_.critical_exposure.get(), buffers_.planning_exposure.get(),
        buffers_.minimum_clearance.get(), buffers_.worst_tier.get(),
        buffers_.raw_collision.get(), buffers_.solid_collision.get(), active_rollouts,
        config_.steps, input.initial_state, input.target, moving_target,
        moving_target_enabled, config_.dynamics, config_.risk, config_.footprint,
        config_.costs, textures_[active_texture_].grid(),
        textures_[active_texture_].texture(), buffers_.solids.get(), solid_count_,
        buffers_.route_points.get(), route_active ? route_point_count_ : 0U,
        route_active ? input.route->initial_station_m : 0.0F, previous_applied_control,
        first_control_interval_s, input.reference_speed_mps,
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
    updateControls<<<control_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.nominal.get(), buffers_.updated.get(), buffers_.noise_ax.get(),
        buffers_.noise_ay.get(), buffers_.noise_az.get(), buffers_.noise_yaw.get(),
        buffers_.weights.get(), buffers_.weight_sum.get(), active_rollouts,
        config_.steps);
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
    const auto evaluate_controls = [&](const std::span<const Control> controls) {
      EvaluatedControlSequence evaluation;
      evaluation.metrics = simulateReference(
          input.initial_state, controls, zero_noise_, config_.dynamics, config_.risk,
          config_.costs, textures_[active_texture_].grid(), activeEsdfHost(),
          input.target.x, input.target.y, config_.early_exit_on_collision,
          previous_applied_control, input.reference_speed_mps, config_.footprint,
          input.moving_target, &evaluation.trace);
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
          config_.costs, textures_[active_texture_].grid(),
          textures_[active_texture_].texture(), buffers_.solids.get(), solid_count_,
          buffers_.route_points.get(), route_active ? route_point_count_ : 0U,
          route_active ? input.route->initial_station_m : 0.0F,
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
    result.nominal_reseeded = nominal_reseeded;
    result.esdf_revision = textures_[active_texture_].revision();
    result.active_rollouts = active_rollouts;
    result.timings.warm_start_ms = elapsedMs(started_, warm_done_);
    result.timings.noise_generation_ms = elapsedMs(warm_done_, noise_done_);
    result.timings.rollout_simulation_ms = elapsedMs(noise_done_, simulation_done_);
    result.timings.risk_reduction_ms = elapsedMs(simulation_done_, reduction_done_);
    result.timings.weight_calculation_ms = elapsedMs(reduction_done_, weights_done_);
    result.timings.control_update_ms = elapsedMs(weights_done_, update_done_);
    result.timings.repair_validation_ms = repair_validation_ms;
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
        result.head_progress_m =
            latest_route_station.has_value()
                ? *latest_route_station - input.route->initial_station_m
                : initial_distance -
                      std::hypot(input.target.x - state.x, input.target.y - state.y);
      }
    }
    result.terminal_progress_m =
        latest_route_station.has_value()
            ? *latest_route_station - input.route->initial_station_m
            : initial_distance -
                  std::hypot(input.target.x - state.x, input.target.y - state.y);
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
  std::array<float, kRepairCandidateCount> repair_critical_exposure_{};
  std::array<float, kRepairCandidateCount> repair_planning_exposure_{};
  std::array<std::uint8_t, kRepairCandidateCount> repair_worst_tier_{};
  std::array<std::uint8_t, kRepairCandidateCount> repair_raw_collision_{};
  std::array<std::uint8_t, kRepairCandidateCount> repair_solid_collision_{};
  std::vector<Control> zero_noise_;
  std::optional<Control> last_output_control_;
  std::int64_t last_planning_stamp_ns_{0};
  std::uint64_t nominal_reseed_generation_{0U};
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
