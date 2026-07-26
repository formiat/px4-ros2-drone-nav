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

TEST(LocalHorizonExecutionState, EndpointSemanticsWireRoundTrip) {
  for (const TrajectoryEndpointSemantics semantics :
       {TrajectoryEndpointSemantics::kMissionGoal,
        TrajectoryEndpointSemantics::kLocalHorizon,
        TrajectoryEndpointSemantics::kTemporaryReplanHold}) {
    const std::uint8_t wire = trajectoryEndpointSemanticsToWire(semantics);
    const std::optional<TrajectoryEndpointSemantics> decoded =
        trajectoryEndpointSemanticsFromWire(wire);
    EXPECT_EQ(decoded, std::optional<TrajectoryEndpointSemantics>{semantics});
  }
  EXPECT_FALSE(trajectoryEndpointSemanticsFromWire(3U).has_value());
}

TEST(LocalHorizonExecutionState, ExplicitTemporaryHoldNeverCompletesMission) {
  const LocalHorizonExecutionDecision decision =
      evaluateLocalHorizonExecution(LocalHorizonExecutionInput{
          .semantics = TrajectoryEndpointSemantics::kTemporaryReplanHold,
          .endpoint_captured = true,
      });
  EXPECT_TRUE(decision.terminal_capture_enabled);
  EXPECT_TRUE(decision.latch_temporary_hold);
  EXPECT_FALSE(decision.mission_goal_eligible);
}

TEST(LocalHorizonExecutionState, GoalProximityWithoutAcceptedArtifactIsNotSettled) {
  EXPECT_FALSE(missionGoalSettlementOwned(MissionGoalSettlementInput{
      .active_artifact_available = false,
      .endpoint_semantics = TrajectoryEndpointSemantics::kMissionGoal,
      .current_goal_distance_m = 0.5,
      .endpoint_goal_distance_m = 0.0,
      .tolerance_m = 3.0,
  }));
}

TEST(LocalHorizonExecutionState, SettledRequiresOwnedMissionGoalEndpoint) {
  MissionGoalSettlementInput input{
      .active_artifact_available = true,
      .endpoint_semantics = TrajectoryEndpointSemantics::kMissionGoal,
      .current_goal_distance_m = 0.5,
      .endpoint_goal_distance_m = 0.2,
      .tolerance_m = 3.0,
  };
  EXPECT_TRUE(missionGoalSettlementOwned(input));

  input.endpoint_semantics = TrajectoryEndpointSemantics::kLocalHorizon;
  EXPECT_FALSE(missionGoalSettlementOwned(input));
  input.endpoint_semantics = TrajectoryEndpointSemantics::kMissionGoal;
  input.endpoint_goal_distance_m = 5.0;
  EXPECT_FALSE(missionGoalSettlementOwned(input));
}

} // namespace drone_city_nav
