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
}

} // namespace
} // namespace drone_city_nav
