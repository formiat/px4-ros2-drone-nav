#include "drone_city_nav/global_guide_candidate.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(GlobalGuideCandidateTest, PrefersViableFrontierOverRawPrefix) {
  GlobalGuideCandidate viable;
  viable.status = LatticePlanStatus::kViableFrontier;
  viable.endpoint_displacement_m = 12.0;
  GlobalGuideCandidate prefix;
  prefix.status = LatticePlanStatus::kRawSafeDetourPrefix;
  prefix.endpoint_displacement_m = 30.0;

  EXPECT_TRUE(betterGlobalGuideCandidate(viable, prefix));
}

TEST(GlobalGuideCandidateTest, PrefersUsefulDisplacementBeforeGoalDistance) {
  GlobalGuideCandidate detour;
  detour.status = LatticePlanStatus::kViableFrontier;
  detour.endpoint_displacement_m = 20.0;
  detour.reachable_depth_m = 40.0;
  detour.remaining_goal_distance_m = 105.0;
  GlobalGuideCandidate short_goal_directed = detour;
  short_goal_directed.endpoint_displacement_m = 4.0;
  short_goal_directed.remaining_goal_distance_m = 90.0;

  EXPECT_TRUE(betterGlobalGuideCandidate(detour, short_goal_directed));
  EXPECT_FALSE(betterGlobalGuideCandidate(short_goal_directed, detour));
}

TEST(GlobalGuideCandidateTest, RequiresMeaningfulRawPrefixDuringAdaptiveSearch) {
  GlobalGuideCandidate prefix;
  prefix.status = LatticePlanStatus::kRawSafeDetourPrefix;
  prefix.guide_length_m = 8.0;
  prefix.endpoint_displacement_m = 6.0;

  EXPECT_FALSE(globalGuideCandidateReadyForActivation(prefix, true, 24.0, 12.0));
  prefix.guide_length_m = 24.0;
  prefix.endpoint_displacement_m = 12.0;
  EXPECT_TRUE(globalGuideCandidateReadyForActivation(prefix, true, 24.0, 12.0));
  EXPECT_FALSE(globalGuideCandidateReadyForActivation(prefix, false, 0.0, 0.0));
}

TEST(GlobalGuideCandidateTest, RejectsFrontierAlreadyInsideExhaustionWindow) {
  GlobalGuideCandidate frontier;
  frontier.status = LatticePlanStatus::kViableFrontier;
  frontier.guide_length_m = 8.0;
  frontier.endpoint_displacement_m = 8.0;

  EXPECT_FALSE(globalGuideCandidateReadyForActivation(frontier, false, 19.0, 4.0));
  frontier.guide_length_m = 20.0;
  EXPECT_TRUE(globalGuideCandidateReadyForActivation(frontier, false, 19.0, 4.0));
}

} // namespace
} // namespace drone_city_nav
