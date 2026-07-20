#include "drone_city_nav/terminal_capture_state_machine.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] TerminalStateMachineInput baseInput() {
  TerminalStateMachineInput input{};
  input.prerequisites_valid = true;
  input.goal_distance_m = 30.0;
  input.remaining_trajectory_distance_m = 30.0;
  input.current_speed_mps = 12.0;
  input.acceptance_radius_m = 1.0;
  input.terminal_capture_radius_m = 8.0;
  input.terminal_capture_max_speed_mps = 8.0;
  input.terminal_position_capture_max_entry_speed_mps = 3.0;
  input.terminal_stuck_speed_mps = 0.5;
  return input;
}

} // namespace

TEST(TerminalCaptureStateMachine, CruiseOutsideTerminalZone) {
  const TerminalStateMachineDecision decision =
      evaluateTerminalStateMachine(baseInput());

  EXPECT_TRUE(decision.valid);
  EXPECT_EQ(decision.state, TerminalFlightState::kCruise);
  EXPECT_STREQ(decision.reason, "none");
  EXPECT_FALSE(decision.position_capture_active);
}

TEST(TerminalCaptureStateMachine, UsesVelocityCaptureInsideTerminalZoneWhenFast) {
  TerminalStateMachineInput input = baseInput();
  input.goal_distance_m = 6.0;
  input.remaining_trajectory_distance_m = 6.0;
  input.current_speed_mps = 5.0;

  const TerminalStateMachineDecision decision = evaluateTerminalStateMachine(input);

  EXPECT_TRUE(decision.valid);
  EXPECT_EQ(decision.state, TerminalFlightState::kVelocityTerminalCapture);
  EXPECT_STREQ(decision.reason, "velocity_terminal_capture");
  EXPECT_TRUE(decision.inside_terminal_zone);
  EXPECT_FALSE(decision.position_capture_active);
}

TEST(TerminalCaptureStateMachine, EntersPositionCaptureNearGoalWhenSlowEnough) {
  TerminalStateMachineInput input = baseInput();
  input.goal_distance_m = 6.0;
  input.remaining_trajectory_distance_m = 12.0;
  input.current_speed_mps = 2.0;
  input.terminal_position_capture_max_entry_speed_mps = 2.5;

  const TerminalStateMachineDecision decision = evaluateTerminalStateMachine(input);

  EXPECT_TRUE(decision.valid);
  EXPECT_EQ(decision.state, TerminalFlightState::kPositionCapture);
  EXPECT_STREQ(decision.reason, "near_goal_slow");
  EXPECT_TRUE(decision.position_capture_active);
  EXPECT_TRUE(decision.position_capture_latched);
  EXPECT_NEAR(decision.max_entry_speed_mps, 2.5, 1.0e-9);
}

TEST(TerminalCaptureStateMachine, CapsPositionCaptureEntrySpeedByTerminalMaxSpeed) {
  TerminalStateMachineInput input = baseInput();
  input.goal_distance_m = 6.0;
  input.current_speed_mps = 4.0;
  input.terminal_capture_max_speed_mps = 4.0;
  input.terminal_position_capture_max_entry_speed_mps = 6.0;

  const TerminalStateMachineDecision decision = evaluateTerminalStateMachine(input);

  EXPECT_EQ(decision.state, TerminalFlightState::kPositionCapture);
  EXPECT_NEAR(decision.max_entry_speed_mps, 4.0, 1.0e-9);
}

TEST(TerminalCaptureStateMachine, EntersPositionCaptureWhenVelocityCaptureStuck) {
  TerminalStateMachineInput input = baseInput();
  input.goal_distance_m = 4.0;
  input.remaining_trajectory_distance_m = 4.0;
  input.current_speed_mps = 0.4;
  input.velocity_terminal_capture_active = true;
  input.acceptance_radius_m = 1.0;
  input.terminal_stuck_speed_mps = 0.5;

  const TerminalStateMachineDecision decision = evaluateTerminalStateMachine(input);

  EXPECT_EQ(decision.state, TerminalFlightState::kPositionCapture);
  EXPECT_STREQ(decision.reason, "terminal_stuck");
  EXPECT_TRUE(decision.terminal_velocity_stuck);
  EXPECT_NEAR(decision.stuck_speed_mps, 0.5, 1.0e-9);
}

TEST(TerminalCaptureStateMachine, KeepsLatchedPositionCapture) {
  TerminalStateMachineInput input = baseInput();
  input.previous_position_capture_latched = true;

  const TerminalStateMachineDecision decision = evaluateTerminalStateMachine(input);

  EXPECT_EQ(decision.state, TerminalFlightState::kPositionCapture);
  EXPECT_STREQ(decision.reason, "latched");
  EXPECT_TRUE(decision.position_capture_latched);
}

TEST(TerminalCaptureStateMachine, FinalHoldOverridesTerminalCapture) {
  TerminalStateMachineInput input = baseInput();
  input.final_goal_hold_active = true;
  input.previous_position_capture_latched = true;

  const TerminalStateMachineDecision decision = evaluateTerminalStateMachine(input);

  EXPECT_TRUE(decision.valid);
  EXPECT_EQ(decision.state, TerminalFlightState::kFinalHold);
  EXPECT_STREQ(decision.reason, "final_hold");
  EXPECT_FALSE(decision.position_capture_latched);
}

TEST(TerminalCaptureStateMachine, TemporaryReplanHoldDoesNotUseFinalHoldState) {
  TerminalStateMachineInput input = baseInput();
  input.temporary_replan_hold_active = true;

  const TerminalStateMachineDecision decision = evaluateTerminalStateMachine(input);

  EXPECT_TRUE(decision.valid);
  EXPECT_EQ(decision.state, TerminalFlightState::kTemporaryReplanHold);
  EXPECT_STREQ(decision.reason, "temporary_replan_hold");
}

TEST(TerminalCaptureStateMachine, InvalidPrerequisitesReturnInvalidCruiseDecision) {
  TerminalStateMachineInput input = baseInput();
  input.prerequisites_valid = false;

  const TerminalStateMachineDecision decision = evaluateTerminalStateMachine(input);

  EXPECT_FALSE(decision.valid);
  EXPECT_EQ(decision.state, TerminalFlightState::kCruise);
  EXPECT_STREQ(decision.reason, "none");
}

} // namespace drone_city_nav
