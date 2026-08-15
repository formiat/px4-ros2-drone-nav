#include "drone_city_nav/cooperative_passage_execution.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace drone_city_nav {
namespace {

constexpr double kNanosecondsPerSecond{1.0e9};

[[nodiscard]] CooperativeConflictResourceId
conflictResourceId(const PassageTraversalId& passage_traversal_id) {
  const std::string& value = passage_traversal_id.value();
  const std::size_t separator = value.find(':');
  return CooperativeConflictResourceId{value.substr(0U, separator)};
}

[[nodiscard]] CooperativePassagePhase
cooperativePhase(const ConstrainedRoutePhase phase) noexcept {
  switch (phase) {
    case ConstrainedRoutePhase::kApproach:
      return CooperativePassagePhase::kApproach;
    case ConstrainedRoutePhase::kTraversal:
      return CooperativePassagePhase::kTraversal;
    case ConstrainedRoutePhase::kDeparture:
      return CooperativePassagePhase::kDeparture;
    case ConstrainedRoutePhase::kUnavailable:
    case ConstrainedRoutePhase::kUnconstrained:
      return CooperativePassagePhase::kNone;
  }
  return CooperativePassagePhase::kNone;
}

[[nodiscard]] std::int64_t futureStamp(const std::int64_t now_ns,
                                       const double delay_s) noexcept {
  return now_ns + static_cast<std::int64_t>(
                      std::llround(std::max(0.0, delay_s) * kNanosecondsPerSecond));
}

} // namespace

CooperativePassageUse
makeCooperativePassageUse(const ConstrainedRouteObservation& observation,
                          const CooperativePassageAssignment& assignment,
                          const std::int64_t now_ns, const double planned_speed_mps,
                          const CooperativePassageTimingConfig& config) noexcept {
  CooperativePassageUse result;
  const CooperativePassagePhase phase = cooperativePhase(observation.phase);
  if (!observation.span_available || phase == CooperativePassagePhase::kNone ||
      observation.passage_traversal_id != assignment.passage_traversal_id ||
      observation.route_generation != assignment.route_generation ||
      !assignment.corridorAvailable() ||
      !(assignment.desired_center_separation_m > 0.0) || now_ns <= 0 ||
      !(config.minimum_prediction_speed_mps > 0.0) ||
      !(config.maximum_prediction_horizon_s > 0.0)) {
    return result;
  }
  const double prediction_speed_mps =
      std::max(config.minimum_prediction_speed_mps,
               std::max(std::max(0.0, planned_speed_mps),
                        observation.actual_horizontal_speed_mps));
  const double entry_delay_s =
      std::clamp(std::max(0.0, observation.distance_to_entry_m) / prediction_speed_mps,
                 0.0, config.maximum_prediction_horizon_s);
  const double exit_delay_s =
      std::clamp(std::max(0.0, observation.distance_to_exit_m) / prediction_speed_mps,
                 entry_delay_s, config.maximum_prediction_horizon_s);
  result = CooperativePassageUse{
      .passage_traversal_id = observation.passage_traversal_id,
      .conflict_resource_id = conflictResourceId(observation.passage_traversal_id),
      .route_generation = observation.route_generation,
      .phase = phase,
      .lateral_offset_m = assignment.applied_lateral_offset_m,
      .minimum_lateral_offset_m = assignment.minimum_lateral_offset_m,
      .maximum_lateral_offset_m = assignment.maximum_lateral_offset_m,
      .desired_center_separation_m = assignment.desired_center_separation_m,
      .direction_sign = observation.direction_sign,
      .station_m = observation.station_m,
      .distance_to_entry_m = observation.distance_to_entry_m,
      .distance_to_exit_m = observation.distance_to_exit_m,
      .predicted_entry_ns = futureStamp(now_ns, entry_delay_s),
      .predicted_exit_ns = futureStamp(now_ns, exit_delay_s),
  };
  return result;
}

CooperativePassageYieldDecision evaluateCooperativePassageYield(
    const CooperativeManeuverCommandData& command, const CooperativePassageUse& passage,
    const ConstrainedRouteObservation& observation, const std::string_view vehicle_id,
    const std::int64_t now_ns, const double current_speed_mps,
    const CooperativePassageYieldConfig& config) noexcept {
  CooperativePassageYieldDecision result;
  if (!command.passage_yield_required) {
    result.status = CooperativePassageYieldStatus::kNotRequired;
    return result;
  }
  if (now_ns <= 0 || command.stamp_ns <= 0 || command.valid_until_ns < now_ns) {
    result.status = CooperativePassageYieldStatus::kStaleCommand;
    return result;
  }
  if (vehicle_id.empty() || command.vehicle_id != vehicle_id) {
    result.status = CooperativePassageYieldStatus::kVehicleMismatch;
    return result;
  }
  if (command.passage_route_generation == 0U ||
      command.passage_route_generation != passage.route_generation) {
    result.status = CooperativePassageYieldStatus::kRouteMismatch;
    return result;
  }
  if (!passage.active() ||
      command.passage_traversal_id != passage.passage_traversal_id ||
      command.passage_conflict_resource_id != passage.conflict_resource_id) {
    result.status = CooperativePassageYieldStatus::kPassageMismatch;
    return result;
  }
  if (std::abs(command.passage_lateral_offset_m - passage.lateral_offset_m) > 1.0e-6 ||
      std::abs(command.passage_minimum_lateral_offset_m -
               passage.minimum_lateral_offset_m) > 1.0e-6 ||
      std::abs(command.passage_maximum_lateral_offset_m -
               passage.maximum_lateral_offset_m) > 1.0e-6) {
    result.status = CooperativePassageYieldStatus::kCorridorMismatch;
    return result;
  }
  if (command.passage_entry_not_before_ns > 0 &&
      now_ns >= command.passage_entry_not_before_ns) {
    result.status = CooperativePassageYieldStatus::kEntryTimeSatisfied;
    return result;
  }
  if (passage.phase != CooperativePassagePhase::kApproach ||
      observation.phase != ConstrainedRoutePhase::kApproach ||
      !(observation.distance_to_entry_m > 0.0)) {
    result.status = CooperativePassageYieldStatus::kNotApproaching;
    return result;
  }
  if (!(config.stopping_buffer_m >= 0.0) || !(config.reaction_latency_s >= 0.0) ||
      !(config.maximum_braking_acceleration_mps2 > 0.0) ||
      !std::isfinite(current_speed_mps)) {
    return result;
  }

  const double available_braking_distance_m =
      std::max(0.0, observation.distance_to_entry_m - config.stopping_buffer_m);
  const double acceleration = config.maximum_braking_acceleration_mps2;
  const double latency = config.reaction_latency_s;
  const double maximum_speed_mps =
      -acceleration * latency +
      std::sqrt(acceleration * acceleration * latency * latency +
                2.0 * acceleration * available_braking_distance_m);
  result.status = CooperativePassageYieldStatus::kAccepted;
  result.active = true;
  result.entry_not_before_ns = command.passage_entry_not_before_ns;
  result.hold_station_m =
      std::max(0.0, observation.begin_station_m - config.stopping_buffer_m);
  result.maximum_speed_mps = std::max(0.0, maximum_speed_mps);
  result.hold_at_entry = observation.station_m + 0.25 >= result.hold_station_m ||
                         observation.distance_to_entry_m <= config.stopping_buffer_m;
  return result;
}

const char*
cooperativePassageYieldStatusName(const CooperativePassageYieldStatus status) noexcept {
  switch (status) {
    case CooperativePassageYieldStatus::kDisabled:
      return "disabled";
    case CooperativePassageYieldStatus::kNotRequired:
      return "not_required";
    case CooperativePassageYieldStatus::kStaleCommand:
      return "stale_command";
    case CooperativePassageYieldStatus::kVehicleMismatch:
      return "vehicle_mismatch";
    case CooperativePassageYieldStatus::kRouteMismatch:
      return "route_mismatch";
    case CooperativePassageYieldStatus::kPassageMismatch:
      return "passage_mismatch";
    case CooperativePassageYieldStatus::kCorridorMismatch:
      return "corridor_mismatch";
    case CooperativePassageYieldStatus::kEntryTimeSatisfied:
      return "entry_time_satisfied";
    case CooperativePassageYieldStatus::kNotApproaching:
      return "not_approaching";
    case CooperativePassageYieldStatus::kAccepted:
      return "accepted";
  }
  return "unknown";
}

} // namespace drone_city_nav
