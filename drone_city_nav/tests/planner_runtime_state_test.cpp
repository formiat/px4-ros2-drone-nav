#include "drone_city_nav/planner_runtime_state.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {

TEST(PlannerRuntimeState, ComputesFiniteAgeAfterStamp) {
  EXPECT_DOUBLE_EQ(ageSecondsFromStamp(1'000'000'000LL, 2'500'000'000LL), 1.5);
}

TEST(PlannerRuntimeState, ReturnsInfinityForMissingOrFutureStamp) {
  EXPECT_TRUE(std::isinf(ageSecondsFromStamp(0, 2'000'000'000LL)));
  EXPECT_TRUE(std::isinf(ageSecondsFromStamp(3'000'000'000LL, 2'000'000'000LL)));
}

TEST(PlannerRuntimeState, ClassifiesPoseReadiness) {
  EXPECT_EQ(
      evaluatePlannerRuntimeReadiness(PlannerRuntimeReadinessInput{false, true, true})
          .reason,
      PlannerRuntimeReadinessReason::kNoPose);
  EXPECT_EQ(
      evaluatePlannerRuntimeReadiness(PlannerRuntimeReadinessInput{true, true, false})
          .reason,
      PlannerRuntimeReadinessReason::kStalePose);

  const PlannerRuntimeReadinessDecision ready =
      evaluatePlannerRuntimeReadiness(PlannerRuntimeReadinessInput{true, true, true});
  EXPECT_TRUE(ready.ready);
  EXPECT_EQ(ready.reason, PlannerRuntimeReadinessReason::kReady);
}

TEST(PlannerRuntimeState, ClassifiesPlanningGridReadinessAndMemoryMismatch) {
  ObstacleFieldBuildResult result;
  result.status = PlanningGridStatus::kNoReadySourceData;
  result.memory.seen = true;
  result.memory.geometry_matches = false;

  PlannerGridReadinessDecision decision = evaluatePlannerGridReadiness(result);
  EXPECT_FALSE(decision.ready);
  EXPECT_EQ(decision.reason, PlannerGridReadinessReason::kNoReadySourceData);
  EXPECT_TRUE(decision.memory_geometry_mismatch);

  result.status = PlanningGridStatus::kReady;
  decision = evaluatePlannerGridReadiness(result);
  EXPECT_EQ(decision.reason, PlannerGridReadinessReason::kMissingGrid);
  EXPECT_FALSE(decision.ready);

  result.raw_occupancy.emplace(GridBounds{0.0, 0.0, 1.0, 2, 2});
  decision = evaluatePlannerGridReadiness(result);
  EXPECT_EQ(decision.reason, PlannerGridReadinessReason::kReady);
  EXPECT_TRUE(decision.ready);
}

TEST(PlannerRuntimeState, MapsStablePathReasonsToRuntimeActions) {
  EXPECT_EQ(stablePathRuntimeAction(StablePathDecisionReason::kClear),
            StablePathRuntimeAction::kReuse);
  EXPECT_EQ(stablePathRuntimeAction(StablePathDecisionReason::kProhibitedConfirmed),
            StablePathRuntimeAction::kRunAStar);
  EXPECT_EQ(stablePathRuntimeAction(StablePathDecisionReason::kProjectionUnavailable),
            StablePathRuntimeAction::kRunAStar);
}

TEST(PlannerRuntimeState, PreservesStaticAStarAndSelectsNoStaticRollout) {
  EXPECT_EQ(plannerModePrimaryAction(true, true), PlannerModePrimaryAction::kAStar);
  EXPECT_EQ(plannerModePrimaryAction(false, false), PlannerModePrimaryAction::kAStar);
  EXPECT_EQ(plannerModePrimaryAction(false, true), PlannerModePrimaryAction::kRollout);
}

TEST(PlannerRuntimeState, AllowsAnyAStarOnlyForStaticOrExplicitNoStaticRecovery) {
  EXPECT_TRUE(astarPlanningAllowed(true, false));
  EXPECT_TRUE(astarPlanningAllowed(true, true));
  EXPECT_TRUE(astarPlanningAllowed(false, true));
  EXPECT_FALSE(astarPlanningAllowed(false, false));
}

TEST(PlannerRuntimeState, RejectsGenerationChangedAtPublicationBoundary) {
  EXPECT_TRUE(publicationGenerationIsCurrent(7U, 7U));
  EXPECT_FALSE(publicationGenerationIsCurrent(7U, 8U));
  EXPECT_FALSE(publicationGenerationIsCurrent(0U, 0U));
}

TEST(PlannerRuntimeState, PeriodicRequestsCoalesceWithoutInvalidatingRunningCycle) {
  PlanningRequestState state;
  state.schedule(PlanningWakeReason::kPeriodicTimer);
  const PlanningJobIdentity first = state.beginCycle();

  EXPECT_EQ(first.cycle_sequence, 1U);
  EXPECT_EQ(first.invalidation_generation, 1U);
  EXPECT_TRUE(state.running());

  for (int request = 0; request < 10; ++request) {
    state.schedule(PlanningWakeReason::kPeriodicTimer);
  }

  EXPECT_EQ(state.latestInvalidationGeneration(), 1U);
  EXPECT_TRUE(publicationGenerationIsCurrent(first.invalidation_generation,
                                             state.latestInvalidationGeneration()));
  state.finishCycle();

  const PlanningJobIdentity second = state.beginCycle();
  EXPECT_EQ(second.cycle_sequence, 2U);
  EXPECT_EQ(second.invalidation_generation, 1U);
  EXPECT_EQ(second.coalesced_requests, 9U);
  EXPECT_FALSE(state.pending());
}

TEST(PlannerRuntimeState, InvalidationRejectsOlderCandidateAndSchedulesReplacement) {
  PlanningRequestState state;
  state.schedule(PlanningWakeReason::kPeriodicTimer);
  const PlanningJobIdentity first = state.beginCycle();

  state.invalidate(PlanningInvalidationReason::kTruncationChanged);

  EXPECT_EQ(state.latestInvalidationGeneration(), 2U);
  EXPECT_EQ(state.latestInvalidationReason(),
            PlanningInvalidationReason::kTruncationChanged);
  EXPECT_FALSE(publicationGenerationIsCurrent(first.invalidation_generation,
                                              state.latestInvalidationGeneration()));

  state.finishCycle();
  const PlanningJobIdentity replacement = state.beginCycle();
  EXPECT_EQ(replacement.cycle_sequence, 2U);
  EXPECT_EQ(replacement.invalidation_generation, 2U);
  EXPECT_EQ(replacement.wake_reason, PlanningWakeReason::kInvalidation);
}

} // namespace drone_city_nav
