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

} // namespace drone_city_nav
