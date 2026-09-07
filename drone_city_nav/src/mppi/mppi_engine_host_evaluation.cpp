#include "mppi_engine_host_evaluation.hpp"

#include "drone_city_nav/mppi/mppi_control_limits.hpp"
#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"

#include <algorithm>
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
      input.moving_target, &evaluation.trace, input.dynamic_aircraft,
      input.cooperative_maneuver, config.cooperative,
      context.dynamic_aircraft_cost_policy, config.altitude_envelope, input.target.z);
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

} // namespace drone_city_nav::mppi::detail
