#include "drone_city_nav/local_horizon_execution_state.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {

TEST(LocalHorizonExecutionState, LocalHorizonNeverLatchesFinalGoal) {
  const LocalHorizonExecutionDecision decision =
      evaluateLocalHorizonExecution(LocalHorizonExecutionInput{
          .semantics = TrajectoryEndpointSemantics::kLocalHorizon,
          .remaining_s_m = 0.0,
          .low_buffer_duration_s = 1.0,
          .endpoint_captured = true,
      });
  EXPECT_FALSE(decision.mission_goal_eligible);
  EXPECT_TRUE(decision.latch_temporary_hold);
}

TEST(LocalHorizonExecutionState, MissionGoalStillLatchesFinalGoal) {
  const LocalHorizonExecutionDecision decision =
      evaluateLocalHorizonExecution(LocalHorizonExecutionInput{
          .semantics = TrajectoryEndpointSemantics::kMissionGoal,
          .endpoint_captured = true,
      });
  EXPECT_TRUE(decision.mission_goal_eligible);
  EXPECT_TRUE(decision.terminal_capture_enabled);
  EXPECT_FALSE(decision.latch_temporary_hold);
}

TEST(LocalHorizonExecutionState, LowBufferBeforeTimeoutKeepsCruise) {
  const LocalHorizonExecutionDecision decision =
      evaluateLocalHorizonExecution(LocalHorizonExecutionInput{
          .semantics = TrajectoryEndpointSemantics::kLocalHorizon,
          .remaining_s_m = 1.0,
          .low_buffer_duration_s = 0.49,
      });
  EXPECT_FALSE(decision.terminal_capture_enabled);
}

TEST(LocalHorizonExecutionState, SuccessorClearsTemporaryState) {
  const LocalHorizonExecutionDecision decision =
      evaluateLocalHorizonExecution(LocalHorizonExecutionInput{
          .semantics = TrajectoryEndpointSemantics::kLocalHorizon,
          .successor_received = true,
      });
  EXPECT_TRUE(decision.clear_temporary_state);
  EXPECT_FALSE(decision.terminal_capture_enabled);
}

} // namespace drone_city_nav
