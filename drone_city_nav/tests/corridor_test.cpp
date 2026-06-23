#include "drone_city_nav/corridor.hpp"

#include <gtest/gtest.h>

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

TEST(Corridor, RouteInsideProhibitedIsInvalid) {
  OccupancyGrid2D grid = corridorGrid();
  grid.setOccupied(GridIndex{5, 5});
  const std::vector<Point2> route{{5.5, 5.5}, {12.5, 5.5}};

  const CorridorResult result = buildCorridor(route, grid, testConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_GT(result.stats.route_prohibited_samples, 0U);
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

} // namespace drone_city_nav
