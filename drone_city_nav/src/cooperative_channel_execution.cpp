#include "drone_city_nav/cooperative_channel_execution.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr double kNanosecondsPerSecond{1.0e9};

[[nodiscard]] CooperativeChannelPhase
cooperativePhase(const ConstrainedRoutePhase phase) noexcept {
  switch (phase) {
    case ConstrainedRoutePhase::kApproach:
      return CooperativeChannelPhase::kApproach;
    case ConstrainedRoutePhase::kTraversal:
      return CooperativeChannelPhase::kTraversal;
    case ConstrainedRoutePhase::kDeparture:
      return CooperativeChannelPhase::kDeparture;
    case ConstrainedRoutePhase::kUnavailable:
    case ConstrainedRoutePhase::kUnconstrained:
      return CooperativeChannelPhase::kNone;
  }
  return CooperativeChannelPhase::kNone;
}

[[nodiscard]] std::int64_t futureStamp(const std::int64_t now_ns,
                                       const double delay_s) noexcept {
  return now_ns + static_cast<std::int64_t>(
                      std::llround(std::max(0.0, delay_s) * kNanosecondsPerSecond));
}

} // namespace

CooperativeChannelUse
makeCooperativeChannelUse(const ConstrainedRouteObservation& observation,
                          const CooperativeChannelAssignment& assignment,
                          const std::int64_t now_ns, const double planned_speed_mps,
                          const CooperativeChannelTimingConfig& config) noexcept {
  CooperativeChannelUse result;
  const CooperativeChannelPhase phase = cooperativePhase(observation.phase);
  if (!observation.span_available || phase == CooperativeChannelPhase::kNone ||
      observation.channel_id != assignment.channel_id ||
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
  result = CooperativeChannelUse{
      .channel_id = observation.channel_id,
      .conflict_resource_id = channelConflictResourceId(observation.channel_id),
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

CooperativeChannelYieldDecision evaluateCooperativeChannelYield(
    const CooperativeManeuverCommandData& command, const CooperativeChannelUse& channel,
    const ConstrainedRouteObservation& observation, const std::string_view vehicle_id,
    const std::int64_t now_ns, const double current_speed_mps,
    const CooperativeChannelYieldConfig& config) noexcept {
  CooperativeChannelYieldDecision result;
  if (!command.channel_yield_required) {
    result.status = CooperativeChannelYieldStatus::kNotRequired;
    return result;
  }
  if (now_ns <= 0 || command.stamp_ns <= 0 || command.valid_until_ns < now_ns) {
    result.status = CooperativeChannelYieldStatus::kStaleCommand;
    return result;
  }
  if (vehicle_id.empty() || command.vehicle_id != vehicle_id) {
    result.status = CooperativeChannelYieldStatus::kVehicleMismatch;
    return result;
  }
  if (command.channel_route_generation == 0U ||
      command.channel_route_generation != channel.route_generation) {
    result.status = CooperativeChannelYieldStatus::kRouteMismatch;
    return result;
  }
  if (!channel.active() || command.channel_id != channel.channel_id ||
      command.channel_conflict_resource_id != channel.conflict_resource_id) {
    result.status = CooperativeChannelYieldStatus::kChannelMismatch;
    return result;
  }
  if (std::abs(command.channel_lateral_offset_m - channel.lateral_offset_m) > 1.0e-6 ||
      std::abs(command.channel_minimum_lateral_offset_m -
               channel.minimum_lateral_offset_m) > 1.0e-6 ||
      std::abs(command.channel_maximum_lateral_offset_m -
               channel.maximum_lateral_offset_m) > 1.0e-6) {
    result.status = CooperativeChannelYieldStatus::kCorridorMismatch;
    return result;
  }
  if (command.channel_entry_not_before_ns > 0 &&
      now_ns >= command.channel_entry_not_before_ns) {
    result.status = CooperativeChannelYieldStatus::kEntryTimeSatisfied;
    return result;
  }
  if (channel.phase != CooperativeChannelPhase::kApproach ||
      observation.phase != ConstrainedRoutePhase::kApproach ||
      !(observation.distance_to_entry_m > 0.0)) {
    result.status = CooperativeChannelYieldStatus::kNotApproaching;
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
  result.status = CooperativeChannelYieldStatus::kAccepted;
  result.active = true;
  result.entry_not_before_ns = command.channel_entry_not_before_ns;
  result.hold_station_m =
      std::max(0.0, observation.begin_station_m - config.stopping_buffer_m);
  result.maximum_speed_mps = std::max(0.0, maximum_speed_mps);
  result.hold_at_entry = observation.station_m + 0.25 >= result.hold_station_m ||
                         observation.distance_to_entry_m <= config.stopping_buffer_m;
  return result;
}

const char*
cooperativeChannelYieldStatusName(const CooperativeChannelYieldStatus status) noexcept {
  switch (status) {
    case CooperativeChannelYieldStatus::kDisabled:
      return "disabled";
    case CooperativeChannelYieldStatus::kNotRequired:
      return "not_required";
    case CooperativeChannelYieldStatus::kStaleCommand:
      return "stale_command";
    case CooperativeChannelYieldStatus::kVehicleMismatch:
      return "vehicle_mismatch";
    case CooperativeChannelYieldStatus::kRouteMismatch:
      return "route_mismatch";
    case CooperativeChannelYieldStatus::kChannelMismatch:
      return "channel_mismatch";
    case CooperativeChannelYieldStatus::kCorridorMismatch:
      return "corridor_mismatch";
    case CooperativeChannelYieldStatus::kEntryTimeSatisfied:
      return "entry_time_satisfied";
    case CooperativeChannelYieldStatus::kNotApproaching:
      return "not_approaching";
    case CooperativeChannelYieldStatus::kAccepted:
      return "accepted";
  }
  return "unknown";
}

} // namespace drone_city_nav
