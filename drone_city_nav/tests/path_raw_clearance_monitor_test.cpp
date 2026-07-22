#include "drone_city_nav/path_raw_clearance_monitor.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeGrid() {
  OccupancyGrid2D grid{GridBounds{-0.5, -10.5, 1.0, 81, 22}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  return grid;
}

[[nodiscard]] std::vector<Point2> linePath() {
  return std::vector<Point2>{{0.0, 0.0}, {40.0, 0.0}};
}

} // namespace

TEST(PathRawClearanceMonitor, DetectsSustainedLowClearanceAhead) {
  OccupancyGrid2D grid = makeGrid();
  for (int x = 20; x <= 30; ++x) {
    grid.setOccupied(GridIndex{x, 13});
  }

  const PathRawClearanceEvaluation result =
      evaluatePathRawClearance(grid, linePath(), PathRawClearanceMonitorConfig{});

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.current_position_arms);
  ASSERT_TRUE(result.violation.detected);
  EXPECT_GE(result.violation.entry_distance_m, 15.0);
  EXPECT_LE(result.violation.entry_distance_m, 17.0);
  EXPECT_GE(result.violation.length_m, 2.0);
  EXPECT_LT(result.violation.min_clearance_m, 5.0);
  EXPECT_TRUE(result.violation.nearest_raw_cell_available);
}

TEST(PathRawClearanceMonitor, IgnoresShortLowClearanceInterval) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{20, 13});

  const PathRawClearanceEvaluation result = evaluatePathRawClearance(
      grid, linePath(),
      PathRawClearanceMonitorConfig{.trigger_clearance_m = 5.0,
                                    .arm_clearance_m = 5.5,
                                    .min_violation_length_m = 8.0,
                                    .sample_step_m = 0.5});

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.current_position_arms);
  EXPECT_FALSE(result.violation.detected);
}

TEST(PathRawClearanceMonitor, DoesNotArmWhenPathStartsInsideClearanceBand) {
  OccupancyGrid2D grid = makeGrid();
  for (int x = 0; x <= 20; ++x) {
    grid.setOccupied(GridIndex{x, 13});
  }

  const PathRawClearanceEvaluation result =
      evaluatePathRawClearance(grid, linePath(), PathRawClearanceMonitorConfig{});

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.current_position_arms);
  EXPECT_TRUE(result.violation.detected);
}

} // namespace drone_city_nav
