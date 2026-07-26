#include "drone_city_nav/no_static_planner_orchestrator.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

std::vector<TrajectoryPointSample> lineSamples(const double start_x,
                                               const double end_x) {
  std::vector<TrajectoryPointSample> samples(2U);
  samples[0].point = Point2{start_x, 0.0};
  samples[1].point = Point2{end_x, 0.0};
  populateTrajectorySampleGeometry(samples);
  return samples;
}

} // namespace

TEST(NoStaticPlannerOrchestrator, RejectsStaleResult) {
  NoStaticPlannerOrchestrator orchestrator;
  const NoStaticPlannerDecision decision =
      orchestrator.decide(NoStaticPlannerDecisionInput{
          .generation = 2U,
          .latest_generation = 3U,
          .grid_revision = 4U,
          .latest_grid_revision = 4U,
          .candidate_valid = true,
          .active_score = std::nullopt,
      });
  EXPECT_EQ(decision.action, NoStaticPlannerAction::kRejectStale);
}

TEST(NoStaticPlannerOrchestrator, HysteresisKeepsCurrentCandidate) {
  NoStaticPlannerOrchestrator orchestrator{
      NoStaticPlannerOrchestratorConfig{.minimum_score_improvement = 5.0}};
  const NoStaticPlannerDecision decision =
      orchestrator.decide(NoStaticPlannerDecisionInput{
          .generation = 3U,
          .latest_generation = 3U,
          .grid_revision = 4U,
          .latest_grid_revision = 4U,
          .candidate_valid = true,
          .candidate_score = 98.0,
          .active_score = 100.0,
      });
  EXPECT_EQ(decision.action, NoStaticPlannerAction::kKeep);
}

TEST(NoStaticPlannerOrchestrator, BlockedSuffixBypassesHysteresis) {
  NoStaticPlannerOrchestrator orchestrator;
  const NoStaticPlannerDecision decision =
      orchestrator.decide(NoStaticPlannerDecisionInput{
          .generation = 3U,
          .latest_generation = 3U,
          .grid_revision = 4U,
          .latest_grid_revision = 4U,
          .candidate_valid = true,
          .active_suffix_blocked = true,
          .candidate_score = 200.0,
          .active_score = 100.0,
      });
  EXPECT_EQ(decision.action, NoStaticPlannerAction::kPublish);
}

TEST(NoStaticPlannerOrchestrator, RepeatedFailureRequestsRecovery) {
  NoStaticPlannerOrchestrator orchestrator{
      NoStaticPlannerOrchestratorConfig{.failed_rollout_cycles_before_recovery = 2U}};
  NoStaticPlannerDecisionInput input{.generation = 1U,
                                     .latest_generation = 1U,
                                     .grid_revision = 1U,
                                     .latest_grid_revision = 1U,
                                     .active_score = std::nullopt};
  EXPECT_EQ(orchestrator.decide(input).action, NoStaticPlannerAction::kKeep);
  EXPECT_EQ(orchestrator.decide(input).action, NoStaticPlannerAction::kRequestRecovery);
}

TEST(NoStaticPlannerOrchestrator, ExhaustingSuffixBypassesHysteresis) {
  NoStaticPlannerOrchestrator orchestrator;
  const NoStaticPlannerDecision decision =
      orchestrator.decide(NoStaticPlannerDecisionInput{
          .generation = 3U,
          .latest_generation = 3U,
          .grid_revision = 4U,
          .latest_grid_revision = 4U,
          .candidate_valid = true,
          .active_suffix_exhausting = true,
          .candidate_score = 200.0,
          .active_score = 100.0,
      });
  EXPECT_EQ(decision.action, NoStaticPlannerAction::kPublish);
}

TEST(NoStaticPlannerOrchestrator, MissingActivePrefixBypassesHysteresis) {
  NoStaticPlannerOrchestrator orchestrator;
  const NoStaticPlannerDecision decision =
      orchestrator.decide(NoStaticPlannerDecisionInput{
          .generation = 3U,
          .latest_generation = 3U,
          .grid_revision = 4U,
          .latest_grid_revision = 4U,
          .candidate_valid = true,
          .active_prefix_available = false,
          .candidate_score = 200.0,
          .active_score = 100.0,
      });
  EXPECT_EQ(decision.action, NoStaticPlannerAction::kPublish);
}

TEST(NoStaticPlannerOrchestrator, StablePrefixRemainsUnchanged) {
  const std::vector<TrajectoryPointSample> active = lineSamples(0.0, 10.0);
  const std::vector<TrajectoryPointSample> successor = lineSamples(5.0, 15.0);

  const StablePrefixStitchResult result =
      stitchStableExecutablePrefix(active, 2.0, 3.0, successor);

  ASSERT_TRUE(result.valid);
  ASSERT_GE(result.samples.size(), 3U);
  EXPECT_NEAR(result.samples.front().point.x, 2.0, 1.0e-9);
  EXPECT_NEAR(result.join_s_m, 5.0, 1.0e-9);
  EXPECT_NEAR(result.samples[1].point.x, 5.0, 1.0e-9);
  EXPECT_NEAR(result.samples.back().point.x, 15.0, 1.0e-9);
}

TEST(NoStaticPlannerOrchestrator, StablePrefixRejectsMismatchedSuccessor) {
  const std::vector<TrajectoryPointSample> active = lineSamples(0.0, 10.0);
  const std::vector<TrajectoryPointSample> successor = lineSamples(7.0, 15.0);

  const StablePrefixStitchResult result =
      stitchStableExecutablePrefix(active, 2.0, 3.0, successor);

  EXPECT_FALSE(result.valid);
}

TEST(NoStaticPlannerOrchestrator, RecoveryFailureKeepsActivePrefix) {
  NoStaticPlannerOrchestrator orchestrator;
  EXPECT_EQ(orchestrator.decideRecoveryFailure(true).action,
            NoStaticPlannerAction::kKeep);
  EXPECT_EQ(orchestrator.decideRecoveryFailure(false).action,
            NoStaticPlannerAction::kHold);
}

TEST(NoStaticPlannerOrchestrator, RecoveryGuidePreservesPolylineLookahead) {
  NoStaticPlannerOrchestrator orchestrator;
  orchestrator.setRecoveryGuide(
      std::vector<Point2>{{0.0, 0.0}, {5.0, 0.0}, {5.0, 10.0}}, 42U);

  EXPECT_TRUE(orchestrator.hasRecoveryGuide());
  EXPECT_EQ(orchestrator.recoveryGuideRevision(), 42U);
  const Point2 target =
      orchestrator.recoveryPreferredTarget({1.0, 0.0}, 7.0, {100.0, 100.0});
  EXPECT_DOUBLE_EQ(target.x, 5.0);
  EXPECT_DOUBLE_EQ(target.y, 10.0);
}

TEST(NoStaticPlannerOrchestrator, RepeatedDirectionSwitchRequestsRecovery) {
  NoStaticPlannerOrchestrator orchestrator{
      NoStaticPlannerOrchestratorConfig{.direction_switches_before_recovery = 2U}};
  NoStaticPlannerDecisionInput input{
      .generation = 1U,
      .latest_generation = 1U,
      .grid_revision = 1U,
      .latest_grid_revision = 1U,
      .candidate_valid = true,
      .active_suffix_exhausting = true,
      .active_score = std::nullopt,
  };
  input.candidate_heading_offset_rad = 0.2;
  EXPECT_EQ(orchestrator.decide(input).action, NoStaticPlannerAction::kPublish);
  input.candidate_heading_offset_rad = -0.2;
  EXPECT_EQ(orchestrator.decide(input).action, NoStaticPlannerAction::kPublish);
  input.candidate_heading_offset_rad = 0.2;
  EXPECT_EQ(orchestrator.decide(input).action, NoStaticPlannerAction::kRequestRecovery);
}

TEST(NoStaticPlannerOrchestrator, TemporaryHoldRequestsRecoveryUntilGuideExists) {
  NoStaticPlannerOrchestrator orchestrator;
  const NoStaticPlannerDecisionInput input{
      .generation = 1U,
      .latest_generation = 1U,
      .grid_revision = 1U,
      .latest_grid_revision = 1U,
      .candidate_valid = true,
      .temporary_hold_active = true,
      .active_score = std::nullopt,
  };

  EXPECT_EQ(orchestrator.decide(input).action, NoStaticPlannerAction::kRequestRecovery);
  orchestrator.setRecoveryGuide(std::vector<Point2>{{0.0, 0.0}, {10.0, 0.0}}, 2U);
  EXPECT_EQ(orchestrator.decide(input).action, NoStaticPlannerAction::kPublish);
}

} // namespace drone_city_nav
