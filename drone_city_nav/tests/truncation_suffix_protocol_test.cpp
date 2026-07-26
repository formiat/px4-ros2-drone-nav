#include "drone_city_nav/terminal_capture_state_machine.hpp"
#include "drone_city_nav/truncation_suffix_protocol.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

constexpr TruncationSuffixIdentity kExpected{
    .path_id = 5U, .generation = 4U, .prefix_fingerprint = 123U};

TEST(TruncationSuffixProtocol, KeepsWaitingForMatchingPendingAck) {
  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, kExpected, TruncationSuffixAckDecision::kPending);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kKeepWaiting);
  EXPECT_STREQ(result.reason, "matching_pending");
}

TEST(TruncationSuffixProtocol, AdoptsMatchingAcceptedAck) {
  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, kExpected, TruncationSuffixAckDecision::kAccepted);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kAdopt);
}

TEST(TruncationSuffixProtocol, RetriesMatchingRejectedAck) {
  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, kExpected, TruncationSuffixAckDecision::kRejected);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kRetry);
}

TEST(TruncationSuffixProtocol, IgnoresAckForOldPath) {
  TruncationSuffixIdentity received = kExpected;
  received.path_id = 3U;

  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, received, TruncationSuffixAckDecision::kAccepted);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kIgnore);
  EXPECT_STREQ(result.reason, "path_id_mismatch");
}

TEST(TruncationSuffixProtocol, IgnoresAckForOldGeneration) {
  TruncationSuffixIdentity received = kExpected;
  received.generation = 3U;

  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, received, TruncationSuffixAckDecision::kRejected);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kIgnore);
  EXPECT_STREQ(result.reason, "generation_mismatch");
}

TEST(TruncationSuffixProtocol, IgnoresAckForDifferentPrefix) {
  TruncationSuffixIdentity received = kExpected;
  received.prefix_fingerprint = 456U;

  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, received, TruncationSuffixAckDecision::kAccepted);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kIgnore);
  EXPECT_STREQ(result.reason, "prefix_fingerprint_mismatch");
}

TEST(TruncationSuffixProtocol, RejectsUnknownDecisionValue) {
  EXPECT_FALSE(truncationSuffixAckDecisionFromValue(99U).has_value());
}

TEST(TruncationSuffixProtocol, MissionGoalRolloutExplicitlyRequiresActivationAck) {
  EXPECT_TRUE(trajectoryActivationAckRequired(TrajectoryActivationAckContract{
      .explicitly_required = true,
      .endpoint_semantics = TrajectoryEndpointSemantics::kMissionGoal,
  }));
  EXPECT_FALSE(trajectoryActivationAckRequired(TrajectoryActivationAckContract{
      .endpoint_semantics = TrajectoryEndpointSemantics::kMissionGoal,
  }));
  EXPECT_TRUE(trajectoryActivationAckRequired(TrajectoryActivationAckContract{
      .endpoint_semantics = TrajectoryEndpointSemantics::kLocalHorizon,
  }));
}

TEST(TruncationSuffixProtocol, OrdinaryMissionGoalAckControlsPendingOwnership) {
  constexpr std::uint64_t kExpectedPathId{101U};

  const TruncationSuffixAckEvaluation pending = evaluateOrdinaryTrajectoryAck(
      kExpectedPathId, kExpectedPathId, TruncationSuffixAckDecision::kPending);
  EXPECT_EQ(pending.action, TruncationSuffixAckAction::kKeepWaiting);
  EXPECT_FALSE(trajectoryAckClearsPending(pending.action));

  const TruncationSuffixAckEvaluation accepted = evaluateOrdinaryTrajectoryAck(
      kExpectedPathId, kExpectedPathId, TruncationSuffixAckDecision::kAccepted);
  EXPECT_EQ(accepted.action, TruncationSuffixAckAction::kAdopt);
  EXPECT_TRUE(trajectoryAckClearsPending(accepted.action));

  const TruncationSuffixAckEvaluation rejected = evaluateOrdinaryTrajectoryAck(
      kExpectedPathId, kExpectedPathId, TruncationSuffixAckDecision::kRejected);
  EXPECT_EQ(rejected.action, TruncationSuffixAckAction::kRetry);
  EXPECT_TRUE(trajectoryAckClearsPending(rejected.action));

  const TruncationSuffixAckEvaluation stale = evaluateOrdinaryTrajectoryAck(
      kExpectedPathId, 100U, TruncationSuffixAckDecision::kAccepted);
  EXPECT_EQ(stale.action, TruncationSuffixAckAction::kIgnore);
  EXPECT_FALSE(trajectoryAckClearsPending(stale.action));
}

TEST(TruncationSuffixProtocol, DeferredRolloutOnlyActivatesFromTemporaryHold) {
  EXPECT_EQ(evaluateDeferredTrajectoryActivation(false, false),
            DeferredTrajectoryActivationAction::kWaitForTemporaryHold);
  EXPECT_EQ(evaluateDeferredTrajectoryActivation(true, false),
            DeferredTrajectoryActivationAction::kActivateFromTemporaryHold);
  EXPECT_EQ(evaluateDeferredTrajectoryActivation(false, true),
            DeferredTrajectoryActivationAction::kRejectAtFinalGoalHold);
  EXPECT_EQ(evaluateDeferredTrajectoryActivation(true, true),
            DeferredTrajectoryActivationAction::kRejectAtFinalGoalHold);
}

TEST(TruncationSuffixProtocol, FinalGoalHoldRejectsDeferredOwnershipForRetry) {
  constexpr std::uint64_t kDeferredPathId{103U};
  ASSERT_EQ(evaluateDeferredTrajectoryActivation(false, true),
            DeferredTrajectoryActivationAction::kRejectAtFinalGoalHold);

  const TruncationSuffixAckEvaluation rejected = evaluateOrdinaryTrajectoryAck(
      kDeferredPathId, kDeferredPathId, TruncationSuffixAckDecision::kRejected);
  EXPECT_EQ(rejected.action, TruncationSuffixAckAction::kRetry);
  EXPECT_TRUE(trajectoryAckClearsPending(rejected.action));

  const TerminalStateMachineDecision control =
      evaluateTerminalStateMachine(TerminalStateMachineInput{
          .final_goal_hold_active = true,
          .prerequisites_valid = true,
          .current_speed_mps = 0.0,
      });
  EXPECT_EQ(control.state, TerminalFlightState::kFinalHold);
  EXPECT_STREQ(control.reason, "final_hold");
}

TEST(TruncationSuffixProtocol, ParsesSuffixActivationModes) {
  EXPECT_EQ(truncationSuffixActivationModeFromValue(0U),
            TruncationSuffixActivationMode::kMovingJoin);
  EXPECT_EQ(truncationSuffixActivationModeFromValue(1U),
            TruncationSuffixActivationMode::kAfterHold);
  EXPECT_FALSE(truncationSuffixActivationModeFromValue(2U).has_value());
  EXPECT_STREQ(
      truncationSuffixActivationModeName(TruncationSuffixActivationMode::kAfterHold),
      "after_hold");
}

TEST(TruncationSuffixProtocol, PublishesMovingJoinBeforeTemporaryHold) {
  const TruncationSuffixPublicationContext context{
      .generation = kExpected.generation,
      .prefix_fingerprint = kExpected.prefix_fingerprint,
      .confirmed = true,
      .awaiting_ack = false,
  };
  ASSERT_TRUE(evaluateTruncationSuffixPublication(context, kExpected).allowed);

  EXPECT_EQ(resolveTruncationSuffixActivationMode(
                TruncationSuffixActivationMode::kMovingJoin, false),
            TruncationSuffixActivationMode::kMovingJoin);
}

TEST(TruncationSuffixProtocol, PublishesAfterHoldWhenTemporaryHoldWasReached) {
  const TruncationSuffixPublicationContext context{
      .generation = kExpected.generation,
      .prefix_fingerprint = kExpected.prefix_fingerprint,
      .confirmed = true,
      .awaiting_ack = false,
  };
  ASSERT_TRUE(evaluateTruncationSuffixPublication(context, kExpected).allowed);

  EXPECT_EQ(resolveTruncationSuffixActivationMode(
                TruncationSuffixActivationMode::kMovingJoin, true),
            TruncationSuffixActivationMode::kAfterHold);
  EXPECT_EQ(resolveTruncationSuffixActivationMode(
                TruncationSuffixActivationMode::kAfterHold, false),
            TruncationSuffixActivationMode::kAfterHold);
}

TEST(TruncationSuffixProtocol, ConsumerUpgradesMovingJoinAfterHoldTransition) {
  const TruncationSuffixActivationMode published_mode =
      resolveTruncationSuffixActivationMode(TruncationSuffixActivationMode::kMovingJoin,
                                            false);
  ASSERT_EQ(published_mode, TruncationSuffixActivationMode::kMovingJoin);

  const TruncationSuffixActivationMode consumed_mode =
      resolveTruncationSuffixActivationMode(published_mode, true);

  EXPECT_EQ(consumed_mode, TruncationSuffixActivationMode::kAfterHold);
}

TEST(TruncationSuffixProtocol, AllowsOnlyFirstPublicationWhileAwaitingAck) {
  TruncationSuffixPublicationContext context{
      .generation = kExpected.generation,
      .prefix_fingerprint = kExpected.prefix_fingerprint,
      .confirmed = true,
      .awaiting_ack = false,
  };

  const TruncationSuffixPublicationEvaluation first =
      evaluateTruncationSuffixPublication(context, kExpected);
  ASSERT_TRUE(first.allowed);
  context.awaiting_ack = true;
  const TruncationSuffixPublicationEvaluation duplicate =
      evaluateTruncationSuffixPublication(context, kExpected);

  EXPECT_FALSE(duplicate.allowed);
  EXPECT_STREQ(duplicate.reason, "already_awaiting_ack");
}

TEST(TruncationSuffixProtocol, RetryAfterRejectedAckReopensPublicationGate) {
  const TruncationSuffixAckEvaluation ack = evaluateTruncationSuffixAck(
      kExpected, kExpected, TruncationSuffixAckDecision::kRejected);
  ASSERT_EQ(ack.action, TruncationSuffixAckAction::kRetry);
  const TruncationSuffixPublicationContext retry_context{
      .generation = kExpected.generation,
      .prefix_fingerprint = kExpected.prefix_fingerprint,
      .confirmed = true,
      .awaiting_ack = false,
  };

  const TruncationSuffixPublicationEvaluation retry =
      evaluateTruncationSuffixPublication(retry_context, kExpected);

  EXPECT_TRUE(retry.allowed);
}

TEST(TruncationSuffixProtocol, RejectsLateResultFromOldGeneration) {
  const TruncationSuffixPublicationContext context{
      .generation = kExpected.generation + 1U,
      .prefix_fingerprint = kExpected.prefix_fingerprint,
      .confirmed = true,
      .awaiting_ack = false,
  };

  const TruncationSuffixPublicationEvaluation result =
      evaluateTruncationSuffixPublication(context, kExpected);

  EXPECT_FALSE(result.allowed);
  EXPECT_STREQ(result.reason, "generation_mismatch");
}

} // namespace
} // namespace drone_city_nav
