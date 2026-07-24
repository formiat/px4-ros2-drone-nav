#include "drone_city_nav/no_static_planner_orchestrator.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {

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

} // namespace drone_city_nav
