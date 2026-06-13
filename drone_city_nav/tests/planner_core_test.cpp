#include "drone_city_nav/astar_planner.hpp"
#include "drone_city_nav/path_smoothing.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeGrid() {
  return OccupancyGrid2D{GridBounds{0.0, 0.0, 1.0, 20, 12}};
}

} // namespace

TEST(OccupancyGrid2D, RayMarksFreeCellsAndOccupiedEndpoint) {
  OccupancyGrid2D grid = makeGrid();

  grid.markRay(Point2{1.5, 1.5}, Point2{5.5, 1.5}, true);

  EXPECT_EQ(grid.state(GridIndex{1, 1}), CellState::kFree);
  EXPECT_EQ(grid.state(GridIndex{2, 1}), CellState::kFree);
  EXPECT_EQ(grid.state(GridIndex{3, 1}), CellState::kFree);
  EXPECT_EQ(grid.state(GridIndex{4, 1}), CellState::kFree);
  EXPECT_EQ(grid.state(GridIndex{5, 1}), CellState::kOccupied);
}

TEST(OccupancyGrid2D, FreeRayClearsStaleOccupiedCells) {
  OccupancyGrid2D grid = makeGrid();

  grid.markRay(Point2{1.5, 1.5}, Point2{5.5, 1.5}, true);
  ASSERT_EQ(grid.state(GridIndex{5, 1}), CellState::kOccupied);

  grid.markRay(Point2{1.5, 1.5}, Point2{8.5, 1.5}, false);

  EXPECT_EQ(grid.state(GridIndex{5, 1}), CellState::kFree);
}

TEST(OccupancyGrid2D, InflationBlocksSafetyRadius) {
  OccupancyGrid2D grid = makeGrid();

  grid.setOccupied(GridIndex{5, 5});
  grid.rebuildInflation(1.1);

  EXPECT_TRUE(grid.isBlocked(GridIndex{5, 5}));
  EXPECT_TRUE(grid.isBlocked(GridIndex{4, 5}));
  EXPECT_TRUE(grid.isBlocked(GridIndex{6, 5}));
  EXPECT_TRUE(grid.isBlocked(GridIndex{5, 4}));
  EXPECT_TRUE(grid.isBlocked(GridIndex{5, 6}));
  EXPECT_FALSE(grid.isBlocked(GridIndex{8, 5}));
}

TEST(AStarPlanner, FindsRouteAroundInflatedBuildingWall) {
  OccupancyGrid2D grid = makeGrid();
  for (int y = 0; y < 10; ++y) {
    grid.setOccupied(GridIndex{9, y});
  }
  grid.rebuildInflation(0.0);

  const GridIndex start{1, 5};
  const GridIndex goal{18, 5};
  const AStarResult result = AStarPlanner{}.plan(grid, start, goal);

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.path.empty());
  EXPECT_EQ(result.path.front(), start);
  EXPECT_EQ(result.path.back(), goal);
  for (const GridIndex cell : result.path) {
    EXPECT_FALSE(grid.isBlocked(cell));
  }
}

TEST(AStarPlanner, ClearanceCostPrefersRouteFartherFromObstacleWall) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 20, 8}};
  for (int x = 0; x < grid.width(); ++x) {
    grid.setOccupied(GridIndex{x, 0});
  }
  grid.rebuildInflation(0.0);

  AStarConfig config{};
  config.obstacle_clearance_cost_radius_m = 3.0;
  config.obstacle_clearance_cost_weight = 8.0;

  const GridIndex start{1, 1};
  const GridIndex goal{18, 1};
  const AStarResult result = AStarPlanner{}.plan(grid, start, goal, config);

  ASSERT_TRUE(result.success);
  const auto farthest_from_wall = std::ranges::max_element(
      result.path, {}, [](const GridIndex cell) { return cell.y; });
  ASSERT_NE(farthest_from_wall, result.path.end());
  EXPECT_GE(farthest_from_wall->y, 3);
}

TEST(PathSmoothing, RejectsShortcutTooCloseToObstacleWhenClearanceIsRequired) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 20, 8}};
  for (int x = 0; x < grid.width(); ++x) {
    grid.setOccupied(GridIndex{x, 0});
  }
  grid.rebuildInflation(0.0);

  EXPECT_TRUE(hasLineOfSight(grid, GridIndex{1, 1}, GridIndex{18, 1}));

  PathSmoothingConfig config{};
  config.minimum_obstacle_clearance_m = 2.5;
  EXPECT_FALSE(hasLineOfSight(grid, GridIndex{1, 1}, GridIndex{18, 1}, config));
}

TEST(PathSmoothing, KeepsCollisionFreeSegments) {
  OccupancyGrid2D grid = makeGrid();
  for (int y = 0; y < 10; ++y) {
    grid.setOccupied(GridIndex{9, y});
  }
  grid.rebuildInflation(0.0);

  const AStarResult result =
      AStarPlanner{}.plan(grid, GridIndex{1, 5}, GridIndex{18, 5});
  ASSERT_TRUE(result.success);

  const std::vector<GridIndex> smoothed = smoothPath(grid, result.path);

  ASSERT_GE(smoothed.size(), 2U);
  EXPECT_LE(smoothed.size(), result.path.size());
  for (std::size_t i = 1; i < smoothed.size(); ++i) {
    EXPECT_TRUE(hasLineOfSight(grid, smoothed[i - 1U], smoothed[i]));
  }
}

} // namespace drone_city_nav
