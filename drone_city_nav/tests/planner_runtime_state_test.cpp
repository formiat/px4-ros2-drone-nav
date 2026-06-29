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
  PlanningGridBuildResult result;
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
}

TEST(PlannerRuntimeState, MapsStablePathReasonsToRuntimeActions) {
  EXPECT_EQ(stablePathRuntimeAction(StablePathDecisionReason::kClear),
            StablePathRuntimeAction::kReuse);
  EXPECT_EQ(stablePathRuntimeAction(StablePathDecisionReason::kProhibitedConfirmed),
            StablePathRuntimeAction::kRunAStar);
  EXPECT_EQ(stablePathRuntimeAction(StablePathDecisionReason::kProjectionUnavailable),
            StablePathRuntimeAction::kRunAStar);
}

} // namespace drone_city_nav
