#include "mppi_engine_host_evaluation.hpp"

#include "drone_city_nav/mppi/mppi_control_limits.hpp"
#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/mppi/mppi_route_projection.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_city_nav::mppi::detail {

EvaluatedControlSequence
evaluateControlSequence(const ControlSequenceEvaluationContext& context,
                        const std::span<const Control> controls) {
  const MppiTickInput& input = context.input;
  const BenchmarkConfig& config = context.config;
  EvaluatedControlSequence evaluation;
  evaluation.metrics = simulateReference(
      input.initial_state, controls, context.zero_noise, config.dynamics, config.risk,
      config.costs, context.grid, context.esdf, input.target.x, input.target.y,
      config.early_exit_on_altitude_envelope_violation,
      context.previous_applied_control, input.reference_speed_mps, config.footprint,
      &evaluation.trace, input.dynamic_aircraft, input.cooperative_maneuver,
      config.cooperative, context.dynamic_aircraft_cost_policy,
      config.altitude_envelope, input.target.z);
  const bool route_active = !context.active_route.empty() && input.route.has_value();
  if (route_active && input.route->terminal_cross_track_tolerance_m &&
      !evaluation.trace.horizon.empty()) {
    const RouteConvergentFiniteHorizon finite_route = buildRouteConvergentFiniteHorizon(
        evaluation.trace.horizon, controls, context.previous_applied_control,
        config.dynamics, context.active_route, input.route->initial_station_m,
        *input.route->terminal_cross_track_tolerance_m,
        finiteHorizonArrivalSearchStepControls(config.dynamics.dt_s),
        makeFiniteHorizonConfig(config.stopping_capability));
    evaluation.terminal_route_cross_track_m =
        finite_route.closest_terminal_cross_track_m;
    evaluation.route_terminal_cross_track_violation = !finite_route.accepted();
    evaluation.route_terminal_arrival_shaping_attempts =
        finite_route.arrival_shaping_attempts;
    if (finite_route.accepted()) {
      evaluation.route_terminal_nominal_prefix_control_count =
          finite_route.nominal_prefix_control_count;
    }
  }
  evaluation.classification = classifyMppiPostUpdate(
      context.feasibility_contract,
      MppiPostUpdateObservation{
          .altitude_envelope_violation = evaluation.metrics.altitude_envelope_violation,
          .route_terminal_cross_track_violation =
              evaluation.route_terminal_cross_track_violation,
      });
  return evaluation;
}

std::size_t buildRepairCandidateSequences(const RepairCandidateInput& input,
                                          const std::size_t steps,
                                          const std::span<Control> candidates) {
  std::vector<Control> limited_nominal(input.nominal.begin(), input.nominal.end());
  limitControlSequence(limited_nominal, input.dynamics, input.previous_applied_control,
                       input.first_control_interval_s);
  std::size_t candidate_count{0U};
  const auto sequence = [&](const std::size_t index) {
    return candidates.subspan(index * steps, steps);
  };
  for (const float ratio : kRepairBacktrackRatios) {
    const std::span<Control> candidate = sequence(candidate_count);
    for (std::size_t index = 0U; index < steps; ++index) {
      candidate[index] =
          interpolateControl(limited_nominal[index], input.updated[index], ratio);
    }
    limitControlSequence(candidate, input.dynamics, input.previous_applied_control,
                         input.first_control_interval_s);
    ++candidate_count;
  }
  std::ranges::copy(input.best_feasible, sequence(candidate_count).begin());
  ++candidate_count;
  if (!input.deterministic_candidate.empty()) {
    std::ranges::copy(input.deterministic_candidate, sequence(candidate_count).begin());
    ++candidate_count;
  }
  return candidate_count;
}

void fillResultFromEvaluation(MppiTickResult& result,
                              EvaluatedControlSequence&& selected_evaluation,
                              const ResultReconstructionInput& in) {
  const auto reconstruction_started = std::chrono::steady_clock::now();
  result.horizon = std::move(selected_evaluation.trace.horizon);
  if (result.horizon.size() != in.controls.size() + 1U) {
    throw std::runtime_error{"MPPI control evaluation returned incomplete horizon"};
  }
  if (in.config.gaze_follows_motion) {
    result.gaze = applyGazeYawControls(
        in.controls, result.horizon, in.config.dynamics, kGazeLookaheadS,
        kGazeMinimumDisplacementM,
        gazeRestHeading(in.input.route, in.config.gaze_minimum_horizontal_share),
        in.config.gaze_survey_yaw_rate_radps);
    result.controls.assign(in.controls.begin(), in.controls.end());
  }
  State state = result.horizon.front();
  const float initial_distance =
      std::hypot(in.input.target.x - state.x, in.input.target.y - state.y);
  float fixed_target_head_progress_m = 0.0F;
  float reconstruction_route_station_m =
      in.input.route.has_value() ? in.input.route->initial_station_m : 0.0F;
  float reconstruction_traveled_distance_m = 0.0F;
  std::optional<float> latest_route_station;
  float latest_credited_route_progress_m = 0.0F;
  const RolloutMetrics& metrics = selected_evaluation.metrics;
  result.altitude_envelope_violation = metrics.altitude_envelope_violation;
  result.route_terminal_cross_track_violation =
      selected_evaluation.route_terminal_cross_track_violation;
  result.terminal_route_cross_track_m =
      selected_evaluation.terminal_route_cross_track_m;
  result.route_terminal_arrival_shaping_attempts =
      selected_evaluation.route_terminal_arrival_shaping_attempts;
  result.route_terminal_nominal_prefix_control_count =
      selected_evaluation.route_terminal_nominal_prefix_control_count;
  result.critical_exposure_m = metrics.critical_exposure_m;
  result.planning_exposure_m = metrics.planning_exposure_m;
  result.obstacle_approach_m2_s = metrics.costs.obstacle_approach_m2_s;
  result.minimum_esdf_distance_m = metrics.minimum_clearance_m;
  result.minimum_peer_separation_m = metrics.minimum_peer_separation_m;
  result.peer_separation_cost = metrics.costs.peer_separation;
  result.dynamic_aircraft_anticipation_cost =
      in.config.dynamics.dt_s * metrics.costs.dynamic_aircraft_anticipation;
  result.dynamic_aircraft_survival_cost =
      in.config.dynamics.dt_s * metrics.costs.dynamic_aircraft_survival;
  const float non_survival_cost =
      std::abs(metrics.soft_cost - result.dynamic_aircraft_survival_cost);
  result.dynamic_aircraft_survival_cost_ratio =
      result.dynamic_aircraft_survival_cost / std::max(1.0e-3F, non_survival_cost);
  result.selected_tier = metrics.worst_tier;
  result.post_update_classification = selected_evaluation.classification;
  Control previous_control = in.previous_applied_control;
  if (!in.controls.empty()) {
    result.first_control_delta =
        std::hypot(std::hypot(in.controls.front().ax - in.previous_applied_control.ax,
                              in.controls.front().ay - in.previous_applied_control.ay),
                   in.controls.front().az - in.previous_applied_control.az);
  }
  const std::size_t head_steps = std::clamp<std::size_t>(
      static_cast<std::size_t>(
          std::ceil(in.config.costs.head_progress_horizon_s / in.config.dynamics.dt_s)),
      1U, in.controls.size());
  for (std::size_t index = 0U; index < in.controls.size(); ++index) {
    const Control& control = in.controls[index];
    result.maximum_acceleration_mps2 =
        std::max(result.maximum_acceleration_mps2,
                 std::hypot(std::hypot(control.ax, control.ay), control.az));
    result.maximum_jerk_mps3 = std::max(
        result.maximum_jerk_mps3,
        std::hypot(std::hypot(control.ax - previous_control.ax,
                              control.ay - previous_control.ay),
                   control.az - previous_control.az) /
            (index == 0U ? in.first_control_interval_s : in.config.dynamics.dt_s));
    previous_control = control;
    const State previous_state = state;
    state = result.horizon[index + 1U];
    reconstruction_traveled_distance_m +=
        std::hypot(std::hypot(state.x - previous_state.x, state.y - previous_state.y),
                   state.z - previous_state.z);
    if (in.input.route.has_value() && in.input.route->points) {
      latest_route_station = projectForwardRouteStation(*in.input.route->points, state,
                                                        reconstruction_route_station_m);
      if (latest_route_station.has_value()) {
        reconstruction_route_station_m = *latest_route_station;
        latest_credited_route_progress_m = creditedRouteProgressM(
            *latest_route_station, in.input.route->initial_station_m,
            reconstruction_traveled_distance_m);
        result.route_progress_integral_m_s +=
            in.config.dynamics.dt_s * latest_credited_route_progress_m;
      }
    }
    if (index + 1U == head_steps) {
      if (latest_route_station.has_value()) {
        result.head_progress_m = latest_credited_route_progress_m;
      } else {
        fixed_target_head_progress_m =
            initial_distance -
            std::hypot(in.input.target.x - state.x, in.input.target.y - state.y);
      }
    }
  }
  if (latest_route_station.has_value()) {
    result.terminal_progress_m = latest_credited_route_progress_m;
  } else {
    const MppiProgressDiagnostics progress = resolveUnroutedProgressDiagnostics(
        fixed_target_head_progress_m,
        initial_distance -
            std::hypot(in.input.target.x - state.x, in.input.target.y - state.y));
    result.head_progress_m = progress.head_progress_m;
    result.terminal_progress_m = progress.terminal_progress_m;
  }
  result.timings.horizon_reconstruction_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                reconstruction_started)
          .count();
}

} // namespace drone_city_nav::mppi::detail
