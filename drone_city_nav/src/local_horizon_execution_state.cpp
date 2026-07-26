#include "drone_city_nav/local_horizon_execution_state.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {

LocalHorizonExecutionDecision
evaluateLocalHorizonExecution(const LocalHorizonExecutionInput& input,
                              const LocalHorizonExecutionConfig& config) {
  LocalHorizonExecutionDecision result;
  result.mission_goal_eligible =
      input.semantics == TrajectoryEndpointSemantics::kMissionGoal;
  if (input.successor_received) {
    result.clear_temporary_state = true;
    return result;
  }
  if (input.semantics == TrajectoryEndpointSemantics::kTemporaryReplanHold) {
    result.terminal_capture_enabled = true;
    result.latch_temporary_hold = input.endpoint_captured;
    return result;
  }
  if (input.semantics != TrajectoryEndpointSemantics::kLocalHorizon) {
    result.terminal_capture_enabled = true;
    return result;
  }
  const bool exhausted =
      input.remaining_s_m <= std::max(0.0, config.minimum_buffer_m) &&
      input.low_buffer_duration_s >= std::max(0.0, config.successor_timeout_s);
  result.terminal_capture_enabled = exhausted;
  result.latch_temporary_hold = exhausted && input.endpoint_captured;
  return result;
}

bool missionGoalSettlementOwned(const MissionGoalSettlementInput& input) noexcept {
  return input.active_artifact_available &&
         input.endpoint_semantics == TrajectoryEndpointSemantics::kMissionGoal &&
         std::isfinite(input.current_goal_distance_m) &&
         std::isfinite(input.endpoint_goal_distance_m) &&
         std::isfinite(input.tolerance_m) && input.tolerance_m >= 0.0 &&
         input.current_goal_distance_m <= input.tolerance_m &&
         input.endpoint_goal_distance_m <= input.tolerance_m;
}

const char*
trajectoryEndpointSemanticsName(const TrajectoryEndpointSemantics semantics) noexcept {
  switch (semantics) {
    case TrajectoryEndpointSemantics::kMissionGoal:
      return "mission_goal";
    case TrajectoryEndpointSemantics::kLocalHorizon:
      return "local_horizon";
    case TrajectoryEndpointSemantics::kTemporaryReplanHold:
      return "temporary_replan_hold";
  }
  return "unknown";
}

std::uint8_t trajectoryEndpointSemanticsToWire(
    const TrajectoryEndpointSemantics semantics) noexcept {
  return static_cast<std::uint8_t>(semantics);
}

std::optional<TrajectoryEndpointSemantics>
trajectoryEndpointSemanticsFromWire(const std::uint8_t value) noexcept {
  if (value > trajectoryEndpointSemanticsToWire(
                  TrajectoryEndpointSemantics::kTemporaryReplanHold)) {
    return std::nullopt;
  }
  return static_cast<TrajectoryEndpointSemantics>(value);
}

} // namespace drone_city_nav
