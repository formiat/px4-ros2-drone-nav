#include "drone_city_nav/corridor.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D corridorGrid() {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 20, 12}};
  for (int x = 0; x < grid.width(); ++x) {
    grid.setOccupied(GridIndex{x, 2});
    grid.setOccupied(GridIndex{x, 9});
  }
  return grid;
}

[[nodiscard]] CorridorConfig testConfig() {
  CorridorConfig config{};
  config.max_radius_m = 10.0;
  config.sample_step_m = 2.0;
  config.safety_margin_m = 0.0;
  return config;
}

} // namespace

TEST(Corridor, StraightPassageHasSymmetricBounds) {
  const OccupancyGrid2D grid = corridorGrid();
  const std::vector<Point2> route{{1.5, 5.5}, {18.5, 5.5}};

  const CorridorResult result = buildCorridor(route, grid, testConfig());

  ASSERT_TRUE(result.valid);
  ASSERT_GT(result.samples.size(), 2U);
  const CorridorSample middle = result.samples[result.samples.size() / 2U];
  EXPECT_NEAR(middle.left_bound_m, middle.right_bound_m, 1.0);
  EXPECT_GT(result.stats.mean_width_m, 5.0);
}

TEST(Corridor, OffsetRouteRecentersToFreeSpaceMidline) {
  const OccupancyGrid2D grid = corridorGrid();
  const std::vector<Point2> route{{1.5, 4.5}, {18.5, 4.5}};

  const CorridorResult result = buildCorridor(route, grid, testConfig());

  ASSERT_TRUE(result.valid);
  ASSERT_GT(result.samples.size(), 2U);
  EXPECT_GT(result.stats.centered_samples, 0U);
  EXPECT_GT(result.stats.max_centering_shift_m, 0.5);
  const CorridorSample middle = result.samples[result.samples.size() / 2U];
  EXPECT_GT(middle.center.y, middle.route_center.y);
  EXPECT_NEAR(middle.left_bound_m, middle.right_bound_m, 1.0);
}

TEST(Corridor, RouteInsideProhibitedIsInvalid) {
  OccupancyGrid2D grid = corridorGrid();
  grid.setOccupied(GridIndex{5, 5});
  CorridorConfig config = testConfig();
  config.center_recovery_max_m = 0.0;
  const std::vector<Point2> route{{5.5, 5.5}, {12.5, 5.5}};

  const CorridorResult result = buildCorridor(route, grid, config);

  EXPECT_FALSE(result.valid);
  EXPECT_GT(result.stats.route_prohibited_samples, 0U);
}

TEST(Corridor, ProhibitedRouteSampleCanRecoverToNearbyFreeCenter) {
  OccupancyGrid2D grid = corridorGrid();
  grid.setOccupied(GridIndex{5, 5});
  CorridorConfig config = testConfig();
  config.sample_step_m = 1.0;
  config.center_recovery_max_m = 1.5;
  const std::vector<Point2> route{{4.5, 5.5}, {8.5, 5.5}};

  const CorridorResult result = buildCorridor(route, grid, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.stats.route_prohibited_samples, 0U);
  EXPECT_GT(result.stats.center_recovered_samples, 0U);
  EXPECT_EQ(result.stats.center_unrecoverable_samples, 0U);
  EXPECT_GT(result.stats.max_center_recovery_m, 0.0);
  EXPECT_TRUE(std::any_of(result.samples.begin(), result.samples.end(),
                          [](const CorridorSample& sample) {
                            return sample.center_recovery_m > 0.0 &&
                                   sample.center.y != sample.route_center.y;
                          }));
}

TEST(Corridor, OutsideGridLimitsBounds) {
  const OccupancyGrid2D grid = corridorGrid();
  CorridorConfig config = testConfig();
  config.max_radius_m = 30.0;
  const std::vector<Point2> route{{1.5, 5.5}, {1.5, 8.5}};

  const CorridorResult result = buildCorridor(route, grid, config);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.stats.outside_grid_samples, 0U);
}

TEST(Corridor, LocalLateralLimitClipsSideOpening) {
  OccupancyGrid2D grid = corridorGrid();
  grid.setFree(GridIndex{10, 9});
  CorridorConfig config = testConfig();
  config.sample_step_m = 1.0;
  config.lateral_limit_window_m = 4.0;
  config.lateral_limit_ratio = 1.0;
  config.lateral_limit_margin_m = 0.0;
  const std::vector<Point2> route{{1.5, 5.5}, {18.5, 5.5}};

  const CorridorResult result = buildCorridor(route, grid, config);

  ASSERT_TRUE(result.valid);
  ASSERT_GT(result.stats.lateral_limited_samples, 0U);
  ASSERT_GT(result.stats.max_lateral_bound_reduction_m, 1.0);
  const auto opening_sample = std::min_element(
      result.samples.begin(), result.samples.end(),
      [](const CorridorSample& lhs, const CorridorSample& rhs) {
        return std::abs(lhs.center.x - 10.5) < std::abs(rhs.center.x - 10.5);
      });
  ASSERT_NE(opening_sample, result.samples.end());
  EXPECT_LE(opening_sample->left_bound_m, 3.5);
}

TEST(Corridor, CenteringSmoothsAlternatingSideOpenings) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 32, 22}};
  for (int x = 0; x < grid.width(); ++x) {
    grid.setOccupied(GridIndex{x, 5});
    grid.setOccupied(GridIndex{x, 15});
  }
  for (int x = 8; x <= 10; ++x) {
    grid.setFree(GridIndex{x, 15});
  }
  for (int x = 13; x <= 15; ++x) {
    grid.setFree(GridIndex{x, 5});
  }
  for (int x = 18; x <= 20; ++x) {
    grid.setFree(GridIndex{x, 15});
  }

  CorridorConfig config = testConfig();
  config.max_radius_m = 20.0;
  config.sample_step_m = 1.0;
  config.lateral_limit_window_m = 8.0;
  const std::vector<Point2> route{{1.5, 10.5}, {30.5, 10.5}};

  const CorridorResult result = buildCorridor(route, grid, config);

  ASSERT_TRUE(result.valid);
  ASSERT_GT(result.samples.size(), 2U);
  EXPECT_GT(result.stats.centered_samples, 0U);
  double max_adjacent_center_jump_m = 0.0;
  for (std::size_t i = 1U; i < result.samples.size(); ++i) {
    max_adjacent_center_jump_m =
        std::max(max_adjacent_center_jump_m, std::abs(result.samples[i].center.y -
                                                      result.samples[i - 1U].center.y));
  }
  EXPECT_LE(max_adjacent_center_jump_m, 0.75);
}

} // namespace drone_city_nav
