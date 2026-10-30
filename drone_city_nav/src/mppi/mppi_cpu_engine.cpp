// The MPPI engine on the host (roadmap item 28, way A): see mppi_cpu_engine.hpp.
// The tick follows the CUDA engine's (mppi_engine.cu) stage by stage, the
// device's kernels replaced by loops over the shared body and plain
// reductions; the host-side parts (the nominal sequence, the candidates, the
// evaluation and the repair, the result's reconstruction) are the same
// functions the CUDA engine calls.
#include "drone_city_nav/mppi/mppi_cpu_engine.hpp"

#include "drone_city_nav/mppi/mppi_control_arbitration.hpp"
#include "drone_city_nav/mppi/mppi_control_limits.hpp"
#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_input_validation.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/mppi/mppi_temperature_regulator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "mppi_device_route_window.hpp"
#include "mppi_engine_host_evaluation.hpp"
#include "mppi_rollout_body.hpp"

namespace drone_city_nav::mppi {
namespace {

constexpr std::size_t kMaximumRoutePoints{512U};
constexpr std::size_t kMaximumDynamicAircraft{16U};
constexpr float kInfinity{std::numeric_limits<float>::infinity()};

using detail::buildRepairCandidateSequences;
using detail::ControlSequenceEvaluationContext;
using detail::evaluateControlSequence;
using detail::EvaluatedControlSequence;
using detail::fillResultFromEvaluation;
using detail::kMaximumRepairCandidateCount;
using detail::kRepairBacktrackRatios;
using detail::RepairCandidateInput;
using detail::ResultReconstructionInput;

// The ESDF as the host reads it: the snapshot's array, [z][y][x].
struct ArrayLookup {
  const float* distances;
  int width;
  int height;

  float operator()(const int x, const int y, const int z) const {
    return distances[(static_cast<std::size_t>(z) * static_cast<std::size_t>(height) +
                      static_cast<std::size_t>(y)) *
                         static_cast<std::size_t>(width) +
                     static_cast<std::size_t>(x)];
  }
};

double elapsedMs(const std::chrono::steady_clock::time_point from,
                 const std::chrono::steady_clock::time_point to) {
  return std::chrono::duration<double, std::milli>(to - from).count();
}

} // namespace

class MppiCpuEngine::Impl {
public:
  explicit Impl(BenchmarkConfig config)
      : config_{std::move(config)},
        nominal_(config_.steps),
        updated_(config_.steps),
        best_feasible_(config_.steps),
        repair_candidates_(kMaximumRepairCandidateCount * config_.steps),
        reacquisition_candidate_(config_.steps),
        zero_noise_(config_.steps),
        noise_ax_(config_.rollouts * config_.steps),
        noise_ay_(config_.rollouts * config_.steps),
        noise_az_(config_.rollouts * config_.steps),
        noise_yaw_(config_.rollouts * config_.steps),
        soft_cost_(config_.rollouts),
        critical_exposure_(config_.rollouts),
        planning_exposure_(config_.rollouts),
        minimum_clearance_(config_.rollouts),
        altitude_envelope_violation_(config_.rollouts),
        collision_violation_(config_.rollouts),
        worst_tier_(config_.rollouts),
        weights_(config_.rollouts),
        dynamic_aircraft_samples_(kMaximumDynamicAircraft * config_.steps),
        dynamic_aircraft_radii_(kMaximumDynamicAircraft),
        dynamic_aircraft_active_steps_(kMaximumDynamicAircraft) {
    if (!benchmarkConfigIsValid(config_)) {
      throw std::invalid_argument{"invalid MPPI engine configuration"};
    }
  }

  EsdfUploadResult updateEsdf(const EsdfSnapshot& snapshot) {
    const std::scoped_lock lock{mutex_};
    const auto started = std::chrono::steady_clock::now();
    const std::size_t expected =
        static_cast<std::size_t>(snapshot.grid.width) *
        static_cast<std::size_t>(snapshot.grid.height) *
        static_cast<std::size_t>(std::max(1, snapshot.grid.depth));
    if (snapshot.grid.width <= 1 || snapshot.grid.height <= 1 ||
        !(snapshot.grid.resolution_m > 0.0F) ||
        snapshot.distances_m.size() != expected) {
      return {};
    }
    esdf_.assign(snapshot.distances_m.begin(), snapshot.distances_m.end());
    grid_ = snapshot.grid;
    esdf_revision_ = snapshot.revision;
    esdf_ready_ = true;
    return EsdfUploadResult{true, elapsedMs(started, std::chrono::steady_clock::now()),
                            snapshot.revision};
  }

  MppiTickResult plan(const MppiTickInput& input) {
    const std::scoped_lock lock{mutex_};
    if (!esdf_ready_) {
      throw std::runtime_error{"MPPI engine has no ESDF"};
    }
    if (!mppiEsdfRevisionMatches(input.expected_esdf_revision, esdf_revision_)) {
      throw std::runtime_error{"MPPI engine ESDF revision does not match the "
                               "captured local world"};
    }
    validateMppiTickInput(input, config_.steps, kMaximumDynamicAircraft);
    const auto host_started = std::chrono::steady_clock::now();
    const std::size_t active_rollouts =
        resolveMppiActiveRollouts(config_.rollouts, input.active_rollouts);
    const std::size_t steps = config_.steps;
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
    const detail::DeviceRouteWindow3D route_window =
        route_active ? detail::selectDeviceRouteWindow3D(active_route,
                                                         input.route->initial_station_m,
                                                         kMaximumRoutePoints)
                     : detail::DeviceRouteWindow3D{};
    if (route_active) {
      const float required_end_station_m =
          std::min(active_route.back().station_m,
                   input.route->initial_station_m + detail::maximumFiniteHorizonTravelM(
                                                        input.initial_state, config_));
      if (!route_window.coversThroughStation(required_end_station_m)) {
        throw std::invalid_argument{
            "MPPI finite-horizon route window exceeds device route capacity"};
      }
    }
    const DynamicAircraftCostPolicy dynamic_aircraft_cost_policy =
        input.dynamic_aircraft_cost_policy.value_or(DynamicAircraftCostPolicy{
            .strong_separation_m = config_.cooperative.desired_minimum_separation_m,
            .anticipation_separation_m =
                config_.cooperative.desired_minimum_separation_m,
            .strong_weight = config_.costs.peer_separation_weight,
        });
    const bool external_nominal_reseeded =
        input.nominal_reseed_generation > nominal_reseed_generation_;
    if (external_nominal_reseeded) {
      nominal_ = buildGuideDirectedNominalSeed(
          input.initial_state, input.target, active_route,
          route_active ? input.route->initial_station_m : 0.0F,
          input.reference_speed_mps, config_.dynamics, steps, previous_applied_control,
          config_.stopping_capability);
      nominal_reseed_generation_ = input.nominal_reseed_generation;
    } else if (has_updated_) {
      nominal_ = shiftControlSequence(updated_, config_.dynamics.dt_s, elapsed_s);
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
              ? buildFiniteRouteDirectedSeed(
                    input.initial_state, input.target, active_route,
                    input.route->initial_station_m, input.reference_speed_mps,
                    config_.dynamics, steps, previous_applied_control,
                    config_.stopping_capability)
              : buildGuideDirectedNominalSeed(
                    input.initial_state, input.target, {}, 0.0F,
                    input.reference_speed_mps, config_.dynamics, steps,
                    previous_applied_control, config_.stopping_capability);
    }
    const std::size_t dynamic_aircraft_count = input.dynamic_aircraft.size();
    for (std::size_t aircraft_index = 0U; aircraft_index < dynamic_aircraft_count;
         ++aircraft_index) {
      const DynamicAircraftTrajectory& aircraft =
          input.dynamic_aircraft[aircraft_index];
      std::ranges::copy(*aircraft.samples,
                        dynamic_aircraft_samples_.begin() +
                            static_cast<std::ptrdiff_t>(aircraft_index * steps));
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
        1U, steps);

    // The population's noise, the deterministic candidate in row 0 when injected.
    const auto noise_started = std::chrono::steady_clock::now();
    const std::size_t noise_count = active_rollouts * steps;
    const std::uint64_t tick = tick_sequence_++;
    for (std::size_t index = 0U; index < noise_count; ++index) {
      const rollout::NoiseSample sample =
          rollout::noiseSample(index, config_.seed, tick, config_.noise);
      noise_ax_[index] = sample.ax;
      noise_ay_[index] = sample.ay;
      noise_az_[index] = sample.az;
      noise_yaw_[index] = sample.yaw;
    }
    if (deterministic_candidate_enabled) {
      for (std::size_t step = 0U; step < steps; ++step) {
        noise_ax_[step] = reacquisition_candidate_[step].ax - nominal_[step].ax;
        noise_ay_[step] = reacquisition_candidate_[step].ay - nominal_[step].ay;
        noise_az_[step] = reacquisition_candidate_[step].az - nominal_[step].az;
        noise_yaw_[step] =
            reacquisition_candidate_[step].yaw_accel - nominal_[step].yaw_accel;
      }
    }
    const auto noise_done = std::chrono::steady_clock::now();
    // One shape for every simulation this tick runs (the population, the
    // repair candidates, the reported sequences), as the CUDA engine's launch.
    const ArrayLookup lookup{esdf_.data(), grid_.width, grid_.height};
    const auto simulate = [&](const std::size_t rollouts,
                              const Control* const direct_controls,
                              RolloutCostTerms* const cost_terms) {
      const rollout::RolloutInputs inputs{
          noise_ax_.data(),
          noise_ay_.data(),
          noise_az_.data(),
          noise_yaw_.data(),
          nominal_.data(),
          steps,
          input.initial_state,
          input.target,
          config_.dynamics,
          config_.risk,
          config_.footprint,
          config_.altitude_envelope,
          config_.costs,
          config_.horizon_sampling,
          grid_,
          route_window.points.data(),
          route_active ? route_window.points.size() : 0U,
          route_active ? input.route->initial_station_m : 0.0F,
          dynamic_aircraft_samples_.data(),
          dynamic_aircraft_radii_.data(),
          dynamic_aircraft_active_steps_.data(),
          dynamic_aircraft_count,
          dynamic_aircraft_cost_policy,
          config_.cooperative,
          cooperative_preferred_acceleration,
          cooperative_preference_steps,
          input.cooperative_maneuver.has_value(),
          previous_applied_control,
          first_control_interval_s,
          input.reference_speed_mps,
          config_.early_exit_on_altitude_envelope_violation,
          direct_controls};
      const rollout::RolloutOutputs outputs{soft_cost_.data(),
                                            critical_exposure_.data(),
                                            planning_exposure_.data(),
                                            minimum_clearance_.data(),
                                            altitude_envelope_violation_.data(),
                                            collision_violation_.data(),
                                            worst_tier_.data(),
                                            cost_terms};
      for (std::size_t index = 0U; index < rollouts; ++index) {
        rollout::simulateRollout(index, inputs, outputs, lookup);
      }
    };
    simulate(active_rollouts, nullptr, nullptr);
    const auto simulation_done = std::chrono::steady_clock::now();

    // The weighting, as the device's reductions: the feasible minimum, the
    // spread above it, the weights and the lowest index at the minimum.
    const float regulated_temperature = temperature_;
    float minimum_soft_cost = kInfinity;
    float feasible_cost_excess_sum = 0.0F;
    unsigned int feasible_count = 0U;
    float feasible_weight_sum = 0.0F;
    float feasible_weight_square_sum = 0.0F;
    int best_rollout_index = INT_MAX;
    const auto run_weighting = [&](const bool ignore_collision) {
      minimum_soft_cost = kInfinity;
      feasible_cost_excess_sum = 0.0F;
      feasible_count = 0U;
      feasible_weight_sum = 0.0F;
      feasible_weight_square_sum = 0.0F;
      best_rollout_index = INT_MAX;
      for (std::size_t index = 0U; index < active_rollouts; ++index) {
        if (rollout::rolloutFeasible(altitude_envelope_violation_[index],
                                     collision_violation_[index], ignore_collision)) {
          minimum_soft_cost = std::min(minimum_soft_cost, soft_cost_[index]);
        }
      }
      for (std::size_t index = 0U; index < active_rollouts; ++index) {
        const bool feasible =
            rollout::rolloutFeasible(altitude_envelope_violation_[index],
                                     collision_violation_[index], ignore_collision);
        if (feasible) {
          feasible_cost_excess_sum +=
              std::max(0.0F, soft_cost_[index] - minimum_soft_cost);
          ++feasible_count;
        }
        const float weight = feasible
                                 ? std::exp(-(soft_cost_[index] - minimum_soft_cost) /
                                            regulated_temperature)
                                 : 0.0F;
        weights_[index] = weight;
        feasible_weight_sum += weight;
        feasible_weight_square_sum += weight * weight;
        if (weight > 0.0F && soft_cost_[index] <= minimum_soft_cost + 1.0e-5F &&
            static_cast<int>(index) < best_rollout_index) {
          best_rollout_index = static_cast<int>(index);
        }
      }
    };
    const bool body_collision_gate = config_.costs.body_collision_gate_enabled;
    run_weighting(!body_collision_gate);
    const bool collision_gate_lifted =
        !body_collision_gate || !(feasible_weight_sum > 0.0F) || feasible_count == 0U;
    if (body_collision_gate && collision_gate_lifted) {
      run_weighting(true);
    }
    const auto weights_done = std::chrono::steady_clock::now();
    // The weighted update, limited; the best feasible rollout's controls.
    const float denominator = std::max(feasible_weight_sum, 1.0e-12F);
    for (std::size_t step = 0U; step < steps; ++step) {
      Control sum{};
      for (std::size_t index = 0U; index < active_rollouts; ++index) {
        const float weight = weights_[index];
        const std::size_t at = index * steps + step;
        sum.ax += weight * noise_ax_[at];
        sum.ay += weight * noise_ay_[at];
        sum.az += weight * noise_az_[at];
        sum.yaw_accel += weight * noise_yaw_[at];
      }
      updated_[step] = Control{nominal_[step].ax + sum.ax / denominator,
                               nominal_[step].ay + sum.ay / denominator,
                               nominal_[step].az + sum.az / denominator,
                               nominal_[step].yaw_accel + sum.yaw_accel / denominator};
    }
    rollout::limitControlSequence(updated_.data(), steps, config_.dynamics,
                                  previous_applied_control, first_control_interval_s);
    for (std::size_t step = 0U; step < steps; ++step) {
      if (best_rollout_index < 0 ||
          static_cast<std::size_t>(best_rollout_index) >= active_rollouts) {
        best_feasible_[step] = nominal_[step];
        continue;
      }
      const std::size_t at =
          static_cast<std::size_t>(best_rollout_index) * steps + step;
      best_feasible_[step] = Control{
          nominal_[step].ax + noise_ax_[at], nominal_[step].ay + noise_ay_[at],
          nominal_[step].az + noise_az_[at], nominal_[step].yaw_accel + noise_yaw_[at]};
    }
    rollout::limitControlSequence(best_feasible_.data(), steps, config_.dynamics,
                                  previous_applied_control, first_control_interval_s);
    const auto update_done = std::chrono::steady_clock::now();

    MppiTickResult result;
    result.feasibility_contract = MppiFeasibilityContract{
        .available = std::isfinite(feasible_weight_sum) && feasible_weight_sum > 0.0F,
        .weight_sum = feasible_weight_sum,
    };
    result.effective_temperature = regulated_temperature;
    result.effective_sample_fraction = effectiveSampleFraction(
        feasible_weight_sum, feasible_weight_square_sum, feasible_count);
    temperature_ = regulatedTemperature(
        MppiTemperatureRegulatorConfig{
            .minimum_temperature = config_.costs.temperature,
            .target_effective_sample_fraction =
                config_.costs.target_effective_sample_fraction,
            .maximum_growth = config_.costs.maximum_temperature_growth,
        },
        temperature_, result.effective_sample_fraction);
    result.collision_gate_lifted = collision_gate_lifted;
    const bool deterministic_candidate_device_feasible =
        deterministic_candidate_enabled && altitude_envelope_violation_[0] == 0U &&
        (collision_gate_lifted || collision_violation_[0] == 0U);
    const float reacquisition_weight =
        deterministic_candidate_enabled ? weights_[0] : 0.0F;
    result.target_directed_candidate_injected = target_directed_candidate;
    result.target_directed_candidate_device_feasible =
        target_directed_candidate && deterministic_candidate_device_feasible;
    result.target_directed_candidate_best_feasible =
        target_directed_candidate && best_rollout_index == 0;
    result.target_directed_candidate_weight =
        target_directed_candidate ? reacquisition_weight : 0.0F;
    result.route_directed_candidate_injected = route_directed_candidate;
    result.route_directed_candidate_device_feasible =
        route_directed_candidate && deterministic_candidate_device_feasible;
    result.route_directed_candidate_best_feasible =
        route_directed_candidate && best_rollout_index == 0;
    result.route_directed_candidate_weight =
        route_directed_candidate ? reacquisition_weight : 0.0F;
    result.route_directed_candidate_generation =
        route_directed_candidate ? input.route->generation : 0U;
    const float feasible_mean_excess =
        feasible_count > 0U
            ? feasible_cost_excess_sum / static_cast<float>(feasible_count)
            : 0.0F;
    const float reacquisition_soft_cost =
        deterministic_candidate_enabled ? soft_cost_[0] : 0.0F;
    const bool candidate_within_tolerance =
        reacquisition_soft_cost <=
        minimum_soft_cost + config_.costs.route_directed_candidate_cost_tolerance *
                                feasible_mean_excess;
    result.route_directed_candidate_cost_excess =
        route_directed_candidate ? reacquisition_soft_cost - minimum_soft_cost : 0.0F;
    result.route_directed_candidate_critical_exposure_m =
        route_directed_candidate ? critical_exposure_[0] : 0.0F;
    result.route_directed_candidate_minimum_clearance_m =
        route_directed_candidate ? minimum_clearance_[0] : 0.0F;
    const bool arbitration_stochastic = !input.dynamic_aircraft.empty();
    const bool policy_prefers_route_candidate = input.prefer_route_directed_candidate &&
                                                !arbitration_stochastic &&
                                                candidate_within_tolerance;
    const bool sticky_prefers_route_candidate = stickyRouteCandidatePreference(
        control_arbitration_,
        MppiControlArbitrationInput{
            .candidate_preferable = policy_prefers_route_candidate,
            .switch_ticks = config_.costs.route_directed_candidate_switch_ticks,
        });
    const bool route_candidate_forced =
        input.force_route_directed_candidate && !arbitration_stochastic;
    if (result.route_directed_candidate_device_feasible &&
        (result.route_directed_candidate_best_feasible || route_candidate_forced ||
         sticky_prefers_route_candidate)) {
      std::ranges::copy(reacquisition_candidate_, updated_.begin());
      limitControlSequence(updated_, config_.dynamics, previous_applied_control,
                           first_control_interval_s);
      result.control_selection = MppiControlSelection::kRouteDirectedCandidate;
    }
    result.dynamic_aircraft_count = dynamic_aircraft_count;
    const ControlSequenceEvaluationContext evaluation_context{
        .input = input,
        .config = config_,
        .grid = grid_,
        .esdf = esdf_,
        .zero_noise = zero_noise_,
        .active_route = active_route,
        .previous_applied_control = previous_applied_control,
        .dynamic_aircraft_cost_policy = dynamic_aircraft_cost_policy,
        .feasibility_contract = result.feasibility_contract,
    };
    const auto evaluate_controls = [&](const std::span<const Control> controls) {
      return evaluateControlSequence(evaluation_context, controls);
    };
    const auto post_update_evaluation_started = std::chrono::steady_clock::now();
    EvaluatedControlSequence selected_evaluation = evaluate_controls(updated_);
    double repair_validation_ms{0.0};
    if (!selected_evaluation.classification.executable &&
        result.feasibility_contract.available) {
      const std::size_t candidate_count = buildRepairCandidateSequences(
          RepairCandidateInput{
              .nominal = nominal_,
              .updated = updated_,
              .best_feasible = best_feasible_,
              .deterministic_candidate =
                  deterministic_candidate_enabled
                      ? std::span<const Control>{reacquisition_candidate_}
                      : std::span<const Control>{},
              .dynamics = config_.dynamics,
              .previous_applied_control = previous_applied_control,
              .first_control_interval_s = first_control_interval_s,
          },
          steps, repair_candidates_);
      const auto repair_started = std::chrono::steady_clock::now();
      simulate(candidate_count, repair_candidates_.data(), nullptr);
      repair_validation_ms =
          elapsedMs(repair_started, std::chrono::steady_clock::now());
      bool repaired{false};
      for (std::size_t candidate_index = 0U; candidate_index < candidate_count;
           ++candidate_index) {
        const MppiPostUpdateClassificationResult sampler_classification =
            classifyMppiPostUpdate(
                result.feasibility_contract,
                MppiPostUpdateObservation{
                    .altitude_envelope_violation =
                        altitude_envelope_violation_[candidate_index] != 0U,
                });
        if (!sampler_classification.executable) {
          continue;
        }
        const std::span<const Control> candidate{
            repair_candidates_.data() + candidate_index * steps, steps};
        EvaluatedControlSequence confirmed = evaluate_controls(candidate);
        if (!confirmed.classification.executable) {
          continue;
        }
        std::ranges::copy(candidate, updated_.begin());
        selected_evaluation = std::move(confirmed);
        if (candidate_index < kRepairBacktrackRatios.size()) {
          result.post_update_repair = MppiPostUpdateRepair::kBacktracked;
          result.post_update_backtrack_ratio = kRepairBacktrackRatios[candidate_index];
        } else if (candidate_index == kRepairBacktrackRatios.size()) {
          result.post_update_repair = MppiPostUpdateRepair::kBestFeasibleRollout;
          result.post_update_backtrack_ratio = 0.0F;
        } else {
          result.post_update_repair = MppiPostUpdateRepair::kDeterministicCandidate;
          result.post_update_backtrack_ratio = 0.0F;
          if (route_directed_candidate) {
            result.control_selection = MppiControlSelection::kRouteDirectedCandidate;
          }
        }
        repaired = true;
        break;
      }
      if (!repaired) {
        result.post_update_repair = MppiPostUpdateRepair::kFailed;
      }
    }
    control_arbitration_.previous_selection = result.control_selection;
    {
      // The decision itemised: the selected sequence (and the route-directed
      // candidate) through the same body that ranked the population.
      std::ranges::copy(updated_, reported_controls_.begin());
      const std::size_t reported_count = route_directed_candidate ? 2U : 1U;
      if (route_directed_candidate) {
        std::ranges::copy(reacquisition_candidate_,
                          reported_controls_.begin() +
                              static_cast<std::ptrdiff_t>(steps));
      }
      simulate(reported_count, reported_controls_.data(), reported_terms_.data());
      result.selected_cost_terms = reported_terms_[0];
      result.route_directed_candidate_cost_terms_available = route_directed_candidate;
      if (route_directed_candidate) {
        result.route_directed_candidate_cost_terms = reported_terms_[1];
      }
    }
    result.controls = updated_;
    result.warm_start_shift_s = elapsed_s;
    result.nominal_reseeded = external_nominal_reseeded;
    result.esdf_revision = esdf_revision_;
    result.active_rollouts = active_rollouts;
    result.timings.noise_generation_ms = elapsedMs(noise_started, noise_done);
    result.timings.rollout_simulation_ms = elapsedMs(noise_done, simulation_done);
    result.timings.weight_calculation_ms = elapsedMs(simulation_done, weights_done);
    result.timings.control_update_ms = elapsedMs(weights_done, update_done);
    result.timings.repair_validation_ms = repair_validation_ms;
    result.timings.post_update_evaluation_ms =
        elapsedMs(post_update_evaluation_started, std::chrono::steady_clock::now());
    // The host did the device's work: the "GPU" total is the population's.
    result.timings.gpu_total_ms =
        elapsedMs(noise_started, update_done) + repair_validation_ms;
    fillResultFromEvaluation(result, std::move(selected_evaluation),
                             ResultReconstructionInput{
                                 .input = input,
                                 .config = config_,
                                 .controls = updated_,
                                 .previous_applied_control = previous_applied_control,
                                 .first_control_interval_s = first_control_interval_s,
                             });
    result.timings.host_total_ms =
        elapsedMs(host_started, std::chrono::steady_clock::now());
    has_updated_ = true;
    last_planning_stamp_ns_ = input.planning_stamp_ns;
    if (!updated_.empty()) {
      last_output_control_ = updated_.front();
    }
    return result;
  }

  [[nodiscard]] std::size_t allocatedBytes() const noexcept {
    return (noise_ax_.size() * 4U + soft_cost_.size() * 3U + weights_.size()) *
               sizeof(float) +
           (altitude_envelope_violation_.size() * 3U) * sizeof(std::uint8_t) +
           esdf_.size() * sizeof(float);
  }

  [[nodiscard]] bool ready() const noexcept {
    return esdf_ready_;
  }

private:
  BenchmarkConfig config_;
  std::vector<float> esdf_;
  EsdfGrid grid_{};
  std::uint64_t esdf_revision_{0U};
  bool esdf_ready_{false};
  std::uint64_t tick_sequence_{0U};
  std::uint64_t nominal_reseed_generation_{0U};
  std::int64_t last_planning_stamp_ns_{0};
  float temperature_{config_.costs.temperature};
  MppiControlArbitrationState control_arbitration_{};
  std::vector<Control> nominal_;
  std::vector<Control> updated_;
  std::vector<Control> best_feasible_;
  std::vector<Control> repair_candidates_;
  std::vector<Control> reacquisition_candidate_;
  std::vector<Control> zero_noise_;
  std::vector<Control> reported_controls_ = std::vector<Control>(2U * config_.steps);
  std::array<RolloutCostTerms, 2U> reported_terms_{};
  std::vector<float> noise_ax_;
  std::vector<float> noise_ay_;
  std::vector<float> noise_az_;
  std::vector<float> noise_yaw_;
  std::vector<float> soft_cost_;
  std::vector<float> critical_exposure_;
  std::vector<float> planning_exposure_;
  std::vector<float> minimum_clearance_;
  std::vector<std::uint8_t> altitude_envelope_violation_;
  std::vector<std::uint8_t> collision_violation_;
  std::vector<std::uint8_t> worst_tier_;
  std::vector<float> weights_;
  std::vector<DynamicAircraftSample> dynamic_aircraft_samples_;
  std::vector<float> dynamic_aircraft_radii_;
  std::vector<std::uint32_t> dynamic_aircraft_active_steps_;
  std::optional<Control> last_output_control_;
  bool has_updated_{false};
  mutable std::mutex mutex_;
};

MppiCpuEngine::MppiCpuEngine(BenchmarkConfig config)
    : impl_{std::make_unique<Impl>(std::move(config))} {
}

MppiCpuEngine::~MppiCpuEngine() = default;
MppiCpuEngine::MppiCpuEngine(MppiCpuEngine&&) noexcept = default;
MppiCpuEngine& MppiCpuEngine::operator=(MppiCpuEngine&&) noexcept = default;

EsdfUploadResult MppiCpuEngine::updateEsdf(const EsdfSnapshot& snapshot) {
  return impl_->updateEsdf(snapshot);
}

MppiTickResult MppiCpuEngine::plan(const MppiTickInput& input) {
  return impl_->plan(input);
}

std::size_t MppiCpuEngine::allocatedBytes() const noexcept {
  return impl_->allocatedBytes();
}

bool MppiCpuEngine::ready() const noexcept {
  return impl_->ready();
}

} // namespace drone_city_nav::mppi
