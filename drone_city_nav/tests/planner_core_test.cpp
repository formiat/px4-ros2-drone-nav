#include "drone_city_nav/astar_planner.hpp"
#include "drone_city_nav/grid_overlay.hpp"
#include "drone_city_nav/path_smoothing.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeGrid() {
  return OccupancyGrid2D{GridBounds{0.0, 0.0, 1.0, 20, 12}};
}

[[nodiscard]] int directionChanges(const std::vector<GridIndex>& path) {
  if (path.size() < 3U) {
    return 0;
  }

  GridIndex previous_direction{path[1].x - path[0].x, path[1].y - path[0].y};
  int changes = 0;
  for (std::size_t i = 2U; i < path.size(); ++i) {
    const GridIndex direction{path[i].x - path[i - 1U].x, path[i].y - path[i - 1U].y};
    if (direction != previous_direction) {
      ++changes;
    }
    previous_direction = direction;
  }
  return changes;
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

TEST(AStarPlanner, AvoidsStaticOnlyObstacleAfterOverlay) {
  OccupancyGrid2D planning_grid = makeGrid();
  OccupancyGrid2D static_grid = makeGrid();
  for (int y = 2; y < 10; ++y) {
    static_grid.setOccupied(GridIndex{9, y});
  }

  const GridOverlayStats overlay_stats =
      overlayOccupiedCells(planning_grid, static_grid);
  planning_grid.rebuildInflation(0.0);

  const GridIndex start{1, 5};
  const GridIndex goal{18, 5};
  const AStarResult result = AStarPlanner{}.plan(planning_grid, start, goal);

  EXPECT_EQ(overlay_stats.source_occupied_cells, 8U);
  ASSERT_TRUE(result.success);
  for (const GridIndex cell : result.path) {
    EXPECT_FALSE(static_grid.isOccupied(cell));
    EXPECT_FALSE(planning_grid.isBlocked(cell));
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

TEST(AStarPlanner, TurnCostPrefersFewerDirectionChanges) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 12, 8}};
  const std::vector<GridIndex> occupied_cells{
      {0, 2}, {1, 1}, {1, 2}, {1, 7}, {2, 6}, {2, 7}, {3, 0}, {3, 1}, {3, 2},
      {3, 4}, {4, 1}, {4, 5}, {5, 2}, {5, 6}, {6, 1}, {6, 2}, {6, 4}, {6, 7},
      {7, 2}, {7, 3}, {8, 1}, {8, 5}, {9, 5}, {9, 6}, {11, 6}};
  for (const GridIndex cell : occupied_cells) {
    grid.setOccupied(cell);
  }
  grid.rebuildInflation(0.0);

  const GridIndex start{1, 3};
  const GridIndex goal{10, 3};
  const AStarResult unpenalized_result = AStarPlanner{}.plan(grid, start, goal);

  AStarConfig turn_config{};
  turn_config.turn_cost_weight = 3.0;
  const AStarResult turn_penalized_result =
      AStarPlanner{}.plan(grid, start, goal, turn_config);

  ASSERT_TRUE(unpenalized_result.success);
  ASSERT_TRUE(turn_penalized_result.success);
  EXPECT_LT(directionChanges(turn_penalized_result.path),
            directionChanges(unpenalized_result.path));
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
