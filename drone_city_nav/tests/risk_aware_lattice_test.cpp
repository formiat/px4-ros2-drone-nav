#include "drone_city_nav/risk_aware_lattice.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] mppi::EsdfGrid makeGrid() {
  return mppi::EsdfGrid{40, 30, 1.0F, 0.0F, 0.0F};
}

TEST(RiskAwareLattice, BuildsGuideThroughOpenSpace) {
  const mppi::EsdfGrid grid = makeGrid();
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                20.0F);

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{34.5, 10.5}, RiskAwareLatticeConfig{});

  ASSERT_TRUE(result.valid);
  ASSERT_GE(result.guide.size(), 2U);
  EXPECT_TRUE(result.planning_goal_reached);
  EXPECT_EQ(result.status, LatticePlanStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.termination, LatticeSearchTermination::kPlanningGoalReached);
  EXPECT_LT(result.guide.back().x, 40.0);
  EXPECT_NEAR(result.guide.front().y, 10.5, 1.0);
}

TEST(RiskAwareLattice, RejectsRawCollisionAndRoutesAroundWall) {
  const mppi::EsdfGrid grid = makeGrid();
  std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height), 20.0F);
  for (int y = 7; y <= 13; ++y) {
    esdf[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) + 18U] =
        0.0F;
  }

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{34.5, 10.5}, RiskAwareLatticeConfig{});

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(std::ranges::any_of(result.guide, [](const Point2 point) {
    return point.y < 7.0 || point.y > 14.0;
  }));
}

TEST(RiskAwareLattice, FailsWhenStartIsOutsideWorldModel) {
  const mppi::EsdfGrid grid = makeGrid();
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                20.0F);

  const RiskAwareLatticeResult result =
      planRiskAwareMotionPrimitiveGuide(grid, esdf, Point2{-1.0, 10.0}, 0.0,
                                        Point2{20.0, 10.0}, RiskAwareLatticeConfig{});

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.status, LatticePlanStatus::kInvalidInput);
  EXPECT_EQ(result.termination, LatticeSearchTermination::kInvalidInput);
}

TEST(RiskAwareLattice, ClassifiesShortTwoPointGuideAsDeadEnd) {
  const mppi::EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height), 0.0F);
  for (int x = 2; x <= 6; ++x) {
    esdf[10U * static_cast<std::size_t>(grid.width) + static_cast<std::size_t>(x)] =
        20.0F;
  }

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{18.5, 10.5}, RiskAwareLatticeConfig{});

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.guide.size(), 2U);
  EXPECT_FALSE(result.planning_goal_reached);
  EXPECT_EQ(result.status, LatticePlanStatus::kDeadEnd);
  EXPECT_EQ(result.termination, LatticeSearchTermination::kOpenSetExhausted);
  EXPECT_EQ(result.terminal_successor_count, 0U);
}

TEST(RiskAwareLattice, ClassifiesUsefulBudgetLimitedGuideAsViableFrontier) {
  const mppi::EsdfGrid grid{200, 30, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(grid.width * grid.height),
                                20.0F);
  RiskAwareLatticeConfig config;
  config.maximum_expansions = 3U;

  const RiskAwareLatticeResult result = planRiskAwareMotionPrimitiveGuide(
      grid, esdf, Point2{2.5, 10.5}, 0.0, Point2{150.5, 10.5}, config);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.planning_goal_reached);
  EXPECT_EQ(result.status, LatticePlanStatus::kViableFrontier);
  EXPECT_EQ(result.termination, LatticeSearchTermination::kExpansionBudgetExhausted);
  EXPECT_GE(result.guide.size(), config.minimum_frontier_guide_points);
  EXPECT_GE(result.guide_length_m, config.minimum_frontier_guide_length_m);
  EXPECT_GE(result.achieved_progress_m, config.minimum_frontier_progress_m);
  EXPECT_GT(result.terminal_successor_count, 0U);
}

} // namespace
} // namespace drone_city_nav
