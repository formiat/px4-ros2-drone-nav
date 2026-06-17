#include "drone_city_nav/astar_planner.hpp"
#include "drone_city_nav/grid_overlay.hpp"
#include "drone_city_nav/path_smoothing.hpp"
#include "drone_city_nav/planner_core.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <numbers>
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

void expectPointNear(const Point2 actual, const Point2 expected) {
  EXPECT_NEAR(actual.x, expected.x, 1.0e-9);
  EXPECT_NEAR(actual.y, expected.y, 1.0e-9);
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

  EXPECT_TRUE(grid.isProhibited(GridIndex{5, 5}));
  EXPECT_TRUE(grid.isProhibited(GridIndex{4, 5}));
  EXPECT_TRUE(grid.isProhibited(GridIndex{6, 5}));
  EXPECT_TRUE(grid.isProhibited(GridIndex{5, 4}));
  EXPECT_TRUE(grid.isProhibited(GridIndex{5, 6}));
  EXPECT_FALSE(grid.isProhibited(GridIndex{8, 5}));
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
    EXPECT_FALSE(grid.isProhibited(cell));
  }
}

TEST(AStarPlanner, UsesPhysicalDistanceForBasePathCost) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 2.0, 8, 8}};
  grid.rebuildInflation(0.0);

  const AStarResult straight_result =
      AStarPlanner{}.plan(grid, GridIndex{1, 1}, GridIndex{4, 1});
  ASSERT_TRUE(straight_result.success);
  EXPECT_NEAR(straight_result.total_cost, 6.0, 1.0e-9);

  const AStarResult diagonal_result =
      AStarPlanner{}.plan(grid, GridIndex{1, 1}, GridIndex{4, 4});
  ASSERT_TRUE(diagonal_result.success);
  EXPECT_NEAR(diagonal_result.total_cost, 3.0 * std::numbers::sqrt2 * 2.0, 1.0e-9);
}

TEST(AStarPlanner, ReportsSuccessForStartEqualGoal) {
  OccupancyGrid2D grid = makeGrid();

  const AStarResult result =
      AStarPlanner{}.plan(grid, GridIndex{4, 4}, GridIndex{4, 4});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.status, AStarStatus::kSuccess);
  ASSERT_EQ(result.path.size(), 1U);
  EXPECT_EQ(result.path.front(), (GridIndex{4, 4}));
  EXPECT_STREQ(astarStatusName(result.status), "success");
}

TEST(AStarPlanner, ReportsProhibitedStartOrGoal) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{4, 4});
  grid.rebuildInflation(0.0);

  const AStarResult result =
      AStarPlanner{}.plan(grid, GridIndex{1, 1}, GridIndex{4, 4});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, AStarStatus::kProhibitedStartOrGoal);
}

TEST(AStarPlanner, ReportsExpansionBudgetExceededSeparately) {
  OccupancyGrid2D grid = makeGrid();
  AStarConfig config{};
  config.max_expansions = 1U;

  const AStarResult result =
      AStarPlanner{}.plan(grid, GridIndex{1, 1}, GridIndex{18, 10}, config);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, AStarStatus::kExpansionBudgetExceeded);
}

TEST(AStarPlanner, MetricClearanceCostPenalizesDiagonalProximity) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 6, 6}};
  grid.setOccupied(GridIndex{1, 1});
  grid.rebuildInflation(0.0);

  AStarConfig config{};
  config.obstacle_clearance_cost_radius_m = 1.5;
  config.obstacle_clearance_cost_weight = 10.0;

  const AStarResult diagonal_result =
      AStarPlanner{}.plan(grid, GridIndex{3, 3}, GridIndex{2, 2}, config);
  const AStarResult far_result =
      AStarPlanner{}.plan(grid, GridIndex{5, 5}, GridIndex{4, 4}, config);

  ASSERT_TRUE(diagonal_result.success);
  ASSERT_TRUE(far_result.success);
  EXPECT_GT(diagonal_result.total_cost, far_result.total_cost);
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
    EXPECT_FALSE(planning_grid.isProhibited(cell));
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

TEST(AStarPlanner, ClearanceCostDoesNotDoubleCountInflation) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 20, 8}};
  for (int x = 0; x < grid.width(); ++x) {
    grid.setOccupied(GridIndex{x, 0});
  }
  grid.rebuildInflation(2.1);

  AStarConfig config{};
  config.obstacle_clearance_cost_radius_m = 3.0;
  config.obstacle_clearance_cost_weight = 8.0;

  const GridIndex start{1, 3};
  const GridIndex goal{18, 3};
  const AStarResult result = AStarPlanner{}.plan(grid, start, goal, config);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.total_cost, 17.0, 1.0e-9);
  EXPECT_TRUE(std::ranges::all_of(result.path,
                                  [](const GridIndex cell) { return cell.y == 3; }));
}

TEST(PlannerCore, DetourLimitFallsBackToShortestSafePath) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 20, 8}};
  for (int x = 0; x < grid.width(); ++x) {
    grid.setOccupied(GridIndex{x, 0});
  }
  grid.rebuildInflation(0.0);

  PlannerCoreConfig config{};
  config.astar.obstacle_clearance_cost_radius_m = 3.0;
  config.astar.obstacle_clearance_cost_weight = 8.0;
  config.comfort_path_max_detour_ratio = 0.05;

  const GridIndex start{1, 1};
  const GridIndex goal{18, 1};
  const auto result = PlannerCore{config}.computePath(grid, grid.cellCenter(start),
                                                      grid.cellCenter(goal));

  if (!result.has_value()) {
    ADD_FAILURE() << "PlannerCore did not return a path";
    return;
  }
  const PathComputationResult& path_result = *result;
  EXPECT_TRUE(path_result.comfort_path_detour_limited);
  EXPECT_FALSE(path_result.comfort_path_selected);
  EXPECT_NEAR(path_result.shortest_path_length_m, 17.0, 1.0e-9);
  EXPECT_GT(path_result.comfort_path_length_m, path_result.comfort_path_length_limit_m);
  EXPECT_TRUE(std::ranges::all_of(path_result.astar.path,
                                  [](const GridIndex cell) { return cell.y == 1; }));
}

TEST(PlannerCore, DetourLimitAllowsComfortPathWithinBudget) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 20, 8}};
  for (int x = 0; x < grid.width(); ++x) {
    grid.setOccupied(GridIndex{x, 0});
  }
  grid.rebuildInflation(0.0);

  PlannerCoreConfig config{};
  config.astar.obstacle_clearance_cost_radius_m = 3.0;
  config.astar.obstacle_clearance_cost_weight = 8.0;
  config.comfort_path_max_detour_ratio = 1.0;

  const GridIndex start{1, 1};
  const GridIndex goal{18, 1};
  const auto result = PlannerCore{config}.computePath(grid, grid.cellCenter(start),
                                                      grid.cellCenter(goal));

  if (!result.has_value()) {
    ADD_FAILURE() << "PlannerCore did not return a path";
    return;
  }
  const PathComputationResult& path_result = *result;
  EXPECT_TRUE(path_result.comfort_path_detour_limited);
  EXPECT_TRUE(path_result.comfort_path_selected);
  EXPECT_LE(path_result.comfort_path_length_m, path_result.comfort_path_length_limit_m);
  const auto farthest_from_wall = std::ranges::max_element(
      path_result.astar.path, {}, [](const GridIndex cell) { return cell.y; });
  ASSERT_NE(farthest_from_wall, path_result.astar.path.end());
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

TEST(AStarPlanner, EvasiveManeuveringPrefersDirectionChanges) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 10, 7}};
  grid.rebuildInflation(0.0);

  const GridIndex start{1, 3};
  const GridIndex goal{8, 3};
  const AStarResult normal_result = AStarPlanner{}.plan(grid, start, goal);

  AStarConfig evasive_config{};
  evasive_config.evasive_maneuvering_enabled = true;
  evasive_config.evasive_maneuvering_straight_cost_weight = 1.0;
  evasive_config.turn_cost_weight = 100.0;
  const AStarResult evasive_result =
      AStarPlanner{}.plan(grid, start, goal, evasive_config);

  ASSERT_TRUE(normal_result.success);
  ASSERT_TRUE(evasive_result.success);
  EXPECT_EQ(directionChanges(normal_result.path), 0);
  EXPECT_GT(directionChanges(evasive_result.path),
            directionChanges(normal_result.path));
}

TEST(PathMetrics, CountsGridSegmentsTurnsAndLength) {
  const OccupancyGrid2D grid = makeGrid();
  const std::vector<GridIndex> path{{0, 0}, {1, 0}, {2, 0}, {2, 1}, {3, 1}};

  const PathMetrics metrics = gridPathMetrics(grid, path);

  EXPECT_EQ(metrics.points, 5U);
  EXPECT_EQ(metrics.segments, 4U);
  EXPECT_EQ(metrics.straight_segments, 3U);
  EXPECT_EQ(metrics.turns, 2U);
  EXPECT_DOUBLE_EQ(metrics.length_m, 4.0);
}

TEST(PathMetrics, CountsPointSegmentsTurnsAndSkipsDuplicatePoints) {
  const std::vector<Point2> path{
      {0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}, {3.0, 4.0}, {6.0, 4.0}};

  const PathMetrics metrics = pointPathMetrics(path);

  EXPECT_EQ(metrics.points, 5U);
  EXPECT_EQ(metrics.segments, 3U);
  EXPECT_EQ(metrics.straight_segments, 3U);
  EXPECT_EQ(metrics.turns, 2U);
  EXPECT_DOUBLE_EQ(metrics.length_m, 10.0);
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

TEST(PathSmoothing, ClearanceRequirementDoesNotDoubleCountInflation) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 20, 8}};
  for (int x = 0; x < grid.width(); ++x) {
    grid.setOccupied(GridIndex{x, 0});
  }
  grid.rebuildInflation(2.1);

  PathSmoothingConfig config{};
  config.minimum_obstacle_clearance_m = 3.0;
  EXPECT_TRUE(hasLineOfSight(grid, GridIndex{1, 3}, GridIndex{18, 3}, config));

  config.minimum_obstacle_clearance_m = 4.0;
  EXPECT_FALSE(hasLineOfSight(grid, GridIndex{1, 3}, GridIndex{18, 3}, config));
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

TEST(PathSmoothing, CollapseCollinearPathRemovesStraightInteriorPoints) {
  const std::vector<Point2> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};

  const std::vector<Point2> collapsed = collapseCollinearPath(path, 0.05);

  ASSERT_EQ(collapsed.size(), 2U);
  expectPointNear(collapsed[0], Point2{0.0, 0.0});
  expectPointNear(collapsed[1], Point2{3.0, 0.0});
}

TEST(PathSmoothing, CollapseCollinearPathPreservesCorners) {
  const std::vector<Point2> path{
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {2.0, 2.0}};

  const std::vector<Point2> collapsed = collapseCollinearPath(path, 0.05);

  ASSERT_EQ(collapsed.size(), 3U);
  expectPointNear(collapsed[0], Point2{0.0, 0.0});
  expectPointNear(collapsed[1], Point2{2.0, 0.0});
  expectPointNear(collapsed[2], Point2{2.0, 2.0});
}

TEST(PathSmoothing, CollapseCollinearPathUsesLateralTolerance) {
  const std::vector<Point2> nearly_straight_path{{0.0, 0.0}, {1.0, 0.02}, {2.0, 0.0}};
  const std::vector<Point2> bent_path{{0.0, 0.0}, {1.0, 0.2}, {2.0, 0.0}};

  EXPECT_EQ(collapseCollinearPath(nearly_straight_path, 0.05).size(), 2U);
  EXPECT_EQ(collapseCollinearPath(bent_path, 0.05).size(), 3U);
}

TEST(PlannerCore, ComputePathAdjustsProhibitedEndpoints) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{1, 1});
  grid.rebuildInflation(0.0);

  PlannerCoreConfig config{};
  config.nearest_free_radius_cells = 2;
  PlannerCore core{config};

  const auto result = core.computePath(grid, Point2{1.5, 1.5}, Point2{18.5, 5.5});

  ASSERT_TRUE(result.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const PathComputationResult& path_result = result.value();
  ASSERT_TRUE(path_result.start_cell.has_value());
  ASSERT_TRUE(path_result.allowed_start_cell.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const GridIndex start_cell = path_result.start_cell.value();
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const GridIndex allowed_start_cell = path_result.allowed_start_cell.value();
  EXPECT_EQ(start_cell, (GridIndex{1, 1}));
  EXPECT_NE(allowed_start_cell, start_cell);
  EXPECT_FALSE(grid.isProhibited(allowed_start_cell));
  EXPECT_FALSE(path_result.smoothed_cells.empty());
}

TEST(PlannerCore, ComputePathRejectsOutOfGridGoal) {
  OccupancyGrid2D grid = makeGrid();
  PlannerCore core{};

  const auto result = core.computePath(grid, Point2{1.5, 1.5}, Point2{100.0, 100.0});

  EXPECT_FALSE(result.has_value());
}

TEST(PlannerCore, StablePathKeepsClearRemainingPath) {
  OccupancyGrid2D grid = makeGrid();
  PlannerCoreConfig config{};
  config.stable_path_goal_tolerance_m = 1.0;
  config.stable_path_reuse_max_deviation_m = 5.0;
  PlannerCore core{config};
  const std::vector<Point2> path{Point2{1.0, 1.0}, Point2{5.0, 1.0}, Point2{9.0, 1.0}};

  const StablePathDecision decision =
      core.evaluateStablePath(grid, path, Point2{3.0, 1.0}, Point2{9.0, 1.0}, 0);

  EXPECT_TRUE(decision.keep_path);
  EXPECT_EQ(decision.reason, StablePathDecisionReason::kClear);
  ASSERT_GE(decision.remaining_path.size(), 2U);
  EXPECT_EQ(decision.prohibited_confirmations, 0);
}

TEST(PlannerCore, StablePathRequiresConfirmedProhibitedIntersection) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{5, 1});
  grid.rebuildInflation(0.0);
  PlannerCoreConfig config{};
  config.stable_path_goal_tolerance_m = 1.0;
  config.stable_path_reuse_max_deviation_m = 5.0;
  config.stable_path_prohibited_length_m = 0.5;
  config.stable_path_prohibited_confirmations_required = 2;
  PlannerCore core{config};
  const std::vector<Point2> path{Point2{1.5, 1.5}, Point2{8.5, 1.5}};

  const StablePathDecision first =
      core.evaluateStablePath(grid, path, Point2{2.5, 1.5}, Point2{8.5, 1.5}, 0);
  const StablePathDecision second = core.evaluateStablePath(
      grid, path, Point2{2.5, 1.5}, Point2{8.5, 1.5}, first.prohibited_confirmations);

  EXPECT_TRUE(first.keep_path);
  EXPECT_EQ(first.reason, StablePathDecisionReason::kProhibitedUnconfirmed);
  EXPECT_EQ(first.prohibited_confirmations, 1);
  EXPECT_FALSE(second.keep_path);
  EXPECT_EQ(second.reason, StablePathDecisionReason::kProhibitedConfirmed);
  EXPECT_EQ(second.prohibited_confirmations, 2);
}

TEST(PlannerCore, StablePathTreatsInflationAsProhibited) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{5, 1});
  grid.rebuildInflation(2.0);
  ASSERT_FALSE(grid.isOccupied(GridIndex{5, 3}));
  ASSERT_TRUE(grid.isInflated(GridIndex{5, 3}));

  PlannerCoreConfig config{};
  config.stable_path_goal_tolerance_m = 1.0;
  config.stable_path_reuse_max_deviation_m = 5.0;
  config.stable_path_prohibited_length_m = 0.5;
  config.stable_path_prohibited_confirmations_required = 1;
  PlannerCore core{config};
  const std::vector<Point2> path{Point2{1.5, 3.5}, Point2{8.5, 3.5}};

  const StablePathDecision decision =
      core.evaluateStablePath(grid, path, Point2{2.5, 3.5}, Point2{8.5, 3.5}, 0);

  EXPECT_FALSE(decision.keep_path);
  EXPECT_EQ(decision.reason, StablePathDecisionReason::kProhibitedConfirmed);
  EXPECT_GE(decision.prohibited_length_m, 1.0);
}

TEST(PlannerCore, StablePathRejectsLargeDeviationFromPath) {
  OccupancyGrid2D grid = makeGrid();
  PlannerCoreConfig config{};
  config.stable_path_goal_tolerance_m = 1.0;
  config.stable_path_reuse_max_deviation_m = 0.5;
  PlannerCore core{config};
  const std::vector<Point2> path{Point2{1.0, 1.0}, Point2{9.0, 1.0}};

  const StablePathDecision decision =
      core.evaluateStablePath(grid, path, Point2{3.0, 4.0}, Point2{9.0, 1.0}, 0);

  EXPECT_FALSE(decision.keep_path);
  EXPECT_EQ(decision.reason, StablePathDecisionReason::kDeviationTooLarge);
  EXPECT_GT(decision.deviation_m, config.stable_path_reuse_max_deviation_m);
}

} // namespace drone_city_nav
