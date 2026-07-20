#pragma once

#include <limits>

namespace drone_city_nav {

enum class TerminalFlightState {
  kCruise,
  kVelocityTerminalCapture,
  kPositionCapture,
  kFinalHold,
  kTemporaryReplanHold,
};

struct TerminalStateMachineInput {
  bool final_goal_hold_active{false};
  bool temporary_replan_hold_active{false};
  bool no_path_hold_active{false};
  bool prerequisites_valid{false};
  bool previous_position_capture_latched{false};
  bool velocity_terminal_capture_active{false};
  double goal_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double remaining_trajectory_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double current_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double acceptance_radius_m{1.0};
  double terminal_capture_radius_m{8.0};
  double terminal_capture_max_speed_mps{8.0};
  double terminal_position_capture_max_entry_speed_mps{3.0};
  double terminal_stuck_speed_mps{0.5};
};

struct TerminalStateMachineDecision {
  TerminalFlightState state{TerminalFlightState::kCruise};
  bool valid{false};
  bool inside_terminal_zone{false};
  bool slow_enough_for_position_capture{false};
  bool terminal_velocity_stuck{false};
  bool position_capture_active{false};
  bool position_capture_latched{false};
  const char* reason{"none"};
  double goal_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double remaining_trajectory_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double current_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double activation_radius_m{std::numeric_limits<double>::quiet_NaN()};
  double max_entry_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double stuck_speed_mps{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] const char* terminalFlightStateName(TerminalFlightState state) noexcept;

[[nodiscard]] TerminalStateMachineDecision
evaluateTerminalStateMachine(const TerminalStateMachineInput& input);

} // namespace drone_city_nav
