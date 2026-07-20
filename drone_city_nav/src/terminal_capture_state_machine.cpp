#include "drone_city_nav/terminal_capture_state_machine.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] double boundedFinite(const double value, const double fallback,
                                   const double min_value,
                                   const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

} // namespace

const char* terminalFlightStateName(const TerminalFlightState state) noexcept {
  switch (state) {
    case TerminalFlightState::kCruise:
      return "cruise";
    case TerminalFlightState::kVelocityTerminalCapture:
      return "velocity_terminal_capture";
    case TerminalFlightState::kPositionCapture:
      return "position_capture";
    case TerminalFlightState::kFinalHold:
      return "final_hold";
    case TerminalFlightState::kTemporaryReplanHold:
      return "temporary_replan_hold";
  }
  return "unknown";
}

TerminalStateMachineDecision
evaluateTerminalStateMachine(const TerminalStateMachineInput& input) {
  TerminalStateMachineDecision decision{};
  decision.activation_radius_m =
      boundedFinite(input.terminal_capture_radius_m, 8.0, 0.0, 1000.0);
  const double terminal_max_speed =
      boundedFinite(input.terminal_capture_max_speed_mps, 8.0, 0.0, 100.0);
  decision.max_entry_speed_mps =
      std::min(terminal_max_speed,
               boundedFinite(input.terminal_position_capture_max_entry_speed_mps, 3.0,
                             0.0, 100.0));
  decision.stuck_speed_mps =
      boundedFinite(input.terminal_stuck_speed_mps, 0.5, 0.0, 100.0);

  if (input.final_goal_hold_active) {
    decision.valid = true;
    decision.state = TerminalFlightState::kFinalHold;
    decision.reason = "final_hold";
    decision.position_capture_latched = false;
    return decision;
  }
  if (input.temporary_replan_hold_active) {
    decision.valid = true;
    decision.state = TerminalFlightState::kTemporaryReplanHold;
    decision.reason = "temporary_replan_hold";
    decision.position_capture_latched = false;
    return decision;
  }

  if (input.no_path_hold_active || !input.prerequisites_valid ||
      !std::isfinite(input.current_speed_mps)) {
    decision.position_capture_latched = false;
    return decision;
  }

  decision.valid = true;
  decision.goal_distance_m = input.goal_distance_m;
  decision.remaining_trajectory_distance_m = input.remaining_trajectory_distance_m;
  decision.current_speed_mps = input.current_speed_mps;
  decision.inside_terminal_zone =
      decision.goal_distance_m <= decision.activation_radius_m ||
      decision.remaining_trajectory_distance_m <= decision.activation_radius_m;
  decision.slow_enough_for_position_capture =
      decision.current_speed_mps <= decision.max_entry_speed_mps;
  decision.terminal_velocity_stuck =
      input.velocity_terminal_capture_active &&
      decision.current_speed_mps <= decision.stuck_speed_mps &&
      decision.goal_distance_m > input.acceptance_radius_m &&
      decision.inside_terminal_zone;

  if (decision.terminal_velocity_stuck) {
    decision.position_capture_active = true;
    decision.position_capture_latched = true;
    decision.state = TerminalFlightState::kPositionCapture;
    decision.reason = "terminal_stuck";
    return decision;
  }
  if (decision.slow_enough_for_position_capture &&
      decision.goal_distance_m <= decision.activation_radius_m) {
    decision.position_capture_active = true;
    decision.position_capture_latched = true;
    decision.state = TerminalFlightState::kPositionCapture;
    decision.reason = "near_goal_slow";
    return decision;
  }
  if (decision.slow_enough_for_position_capture &&
      decision.remaining_trajectory_distance_m <= decision.activation_radius_m) {
    decision.position_capture_active = true;
    decision.position_capture_latched = true;
    decision.state = TerminalFlightState::kPositionCapture;
    decision.reason = "near_end_slow";
    return decision;
  }
  if (input.previous_position_capture_latched) {
    decision.position_capture_active = true;
    decision.position_capture_latched = true;
    decision.state = TerminalFlightState::kPositionCapture;
    decision.reason = "latched";
    return decision;
  }
  if (input.velocity_terminal_capture_active || decision.inside_terminal_zone) {
    decision.state = TerminalFlightState::kVelocityTerminalCapture;
    decision.reason = "velocity_terminal_capture";
    return decision;
  }
  return decision;
}

} // namespace drone_city_nav
