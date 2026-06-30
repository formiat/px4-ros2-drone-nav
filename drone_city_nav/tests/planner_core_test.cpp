#include "drone_city_nav/astar_planner.hpp"
#include "drone_city_nav/grid_overlay.hpp"
#include "drone_city_nav/path_smoothing.hpp"
#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/static_city_map.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
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

[[nodiscard]] AStarConfig plainAStarConfig() {
  AStarConfig config{};
  config.evasive_maneuvering_enabled = false;
  config.initial_heading_bias_enabled = false;
  return config;
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

TEST(PathSafety, SegmentAllowedRejectsOccupiedAndInflatedCells) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{5, 5});
  grid.rebuildInflation(1.1);

  EXPECT_FALSE(pathSegmentIsAllowed(grid, Point2{4.5, 5.5}, Point2{6.5, 5.5}));
  EXPECT_TRUE(pathSegmentIsAllowed(grid, Point2{8.5, 5.5}, Point2{10.5, 5.5}));
}

TEST(PathSafety, SegmentTraversableAllowsLeavingProhibitedPrefix) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{5, 5});
  grid.rebuildInflation(0.0);

  EXPECT_FALSE(pathSegmentIsAllowed(grid, Point2{5.5, 5.5}, Point2{8.5, 5.5}));
  EXPECT_TRUE(pathSegmentIsTraversable(grid, Point2{5.5, 5.5}, Point2{8.5, 5.5}));
}

TEST(PathSafety, SegmentTraversableRejectsEnteringProhibitedCells) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{5, 5});
  grid.rebuildInflation(0.0);

  EXPECT_FALSE(pathSegmentIsTraversable(grid, Point2{4.5, 5.5}, Point2{6.5, 5.5}));
  EXPECT_FALSE(pathSegmentIsTraversable(grid, Point2{4.5, 5.5}, Point2{5.5, 5.5}));
}

TEST(PathSafety, SegmentTraversableRejectsReenteringProhibitedCells) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{5, 5});
  grid.setOccupied(GridIndex{8, 5});
  grid.rebuildInflation(0.0);

  EXPECT_FALSE(pathSegmentIsTraversable(grid, Point2{5.5, 5.5}, Point2{10.5, 5.5}));
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

  const AStarConfig config = plainAStarConfig();
  const AStarResult straight_result =
      AStarPlanner{}.plan(grid, GridIndex{1, 1}, GridIndex{4, 1}, config);
  ASSERT_TRUE(straight_result.success);
  EXPECT_NEAR(straight_result.total_cost, 6.0, 1.0e-9);

  const AStarResult diagonal_result =
      AStarPlanner{}.plan(grid, GridIndex{1, 1}, GridIndex{4, 4}, config);
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

TEST(AStarPlanner, FindsPathAcrossGeneratedCityMap) {
  const std::filesystem::path map_path =
      std::filesystem::path{DRONE_CITY_NAV_SOURCE_DIR} / "worlds" /
      "generated_city.map2d";
  const StaticCityMap static_map = loadStaticCityMap(map_path);
  OccupancyGrid2D grid = rasterizeStaticCityMap(static_map, 0.0);
  grid.rebuildInflation(5.0);

  const auto start = grid.worldToCell(Point2{54.0, 54.0});
  const auto goal = grid.worldToCell(Point2{216.0, 378.0});
  if (!start.has_value() || !goal.has_value()) {
    FAIL() << "Generated city mission endpoints must be inside the static map";
  }
  const GridIndex start_cell = start.value();
  const GridIndex goal_cell = goal.value();
  ASSERT_FALSE(grid.isProhibited(start_cell));
  ASSERT_FALSE(grid.isProhibited(goal_cell));

  AStarConfig config{};
  config.turn_cost_weight = 50.0;
  const AStarResult result = AStarPlanner{}.plan(grid, start_cell, goal_cell, config);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.status, AStarStatus::kSuccess);
  ASSERT_FALSE(result.path.empty());
  EXPECT_EQ(result.path.front(), start_cell);
  EXPECT_EQ(result.path.back(), goal_cell);
}

TEST(AStarPlanner, HeuristicWeightKeepsGeneratedCityRouteReachable) {
  const std::filesystem::path map_path =
      std::filesystem::path{DRONE_CITY_NAV_SOURCE_DIR} / "worlds" /
      "generated_city.map2d";
  const StaticCityMap static_map = loadStaticCityMap(map_path);
  OccupancyGrid2D grid = rasterizeStaticCityMap(static_map, 0.0);
  grid.rebuildInflation(5.0);

  const auto start = grid.worldToCell(Point2{54.0, 54.0});
  const auto goal = grid.worldToCell(Point2{216.0, 378.0});
  if (!start.has_value() || !goal.has_value()) {
    FAIL() << "Generated city mission endpoints must be inside the static map";
  }
  const GridIndex start_cell = start.value();
  const GridIndex goal_cell = goal.value();

  AStarConfig standard_config{};
  standard_config.turn_cost_weight = 50.0;
  const AStarResult standard_result =
      AStarPlanner{}.plan(grid, start_cell, goal_cell, standard_config);

  AStarConfig weighted_config = standard_config;
  weighted_config.heuristic_weight = 1.2;
  const AStarResult weighted_result =
      AStarPlanner{}.plan(grid, start_cell, goal_cell, weighted_config);

  ASSERT_TRUE(standard_result.success);
  ASSERT_TRUE(weighted_result.success);
  EXPECT_EQ(weighted_result.path.front(), start_cell);
  EXPECT_EQ(weighted_result.path.back(), goal_cell);
  EXPECT_LE(weighted_result.expanded_cells, standard_result.expanded_cells);
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
  const AStarConfig unpenalized_config = plainAStarConfig();
  const AStarResult unpenalized_result =
      AStarPlanner{}.plan(grid, start, goal, unpenalized_config);

  AStarConfig turn_config = plainAStarConfig();
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
  const AStarConfig normal_config = plainAStarConfig();
  const AStarResult normal_result =
      AStarPlanner{}.plan(grid, start, goal, normal_config);

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

TEST(AStarPlanner, InitialHeadingBiasPrefersVelocityAlignedFirstStep) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 7, 7}};
  for (int y = 1; y <= 5; ++y) {
    grid.setOccupied(GridIndex{3, y});
  }
  grid.rebuildInflation(0.0);

  const GridIndex start{3, 6};
  const GridIndex goal{3, 0};
  AStarConfig config{};
  config.initial_heading_bias_enabled = true;
  config.initial_heading_bias_min_speed_mps = 0.5;
  config.initial_heading_bias_weight = 50.0;
  config.initial_heading_bias_velocity_x_mps = 5.0;
  config.initial_heading_bias_velocity_y_mps = 0.0;

  const AStarResult result = AStarPlanner{}.plan(grid, start, goal, config);

  ASSERT_TRUE(result.success);
  ASSERT_GE(result.path.size(), 2U);
  EXPECT_EQ(result.path.front(), start);
  EXPECT_EQ(result.path.back(), goal);
  EXPECT_GT(result.path[1].x, start.x);
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
  EXPECT_DOUBLE_EQ(metrics.min_segment_length_m, 1.0);
  EXPECT_DOUBLE_EQ(metrics.mean_segment_length_m, 1.0);
  EXPECT_DOUBLE_EQ(metrics.max_segment_length_m, 1.0);
  EXPECT_EQ(metrics.segments_shorter_than_2m, 4U);
  EXPECT_EQ(metrics.segments_shorter_than_5m, 4U);
  EXPECT_EQ(metrics.segments_shorter_than_10m, 4U);
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
  EXPECT_DOUBLE_EQ(metrics.min_segment_length_m, 3.0);
  EXPECT_NEAR(metrics.mean_segment_length_m, 10.0 / 3.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(metrics.max_segment_length_m, 4.0);
  EXPECT_EQ(metrics.segments_shorter_than_2m, 0U);
  EXPECT_EQ(metrics.segments_shorter_than_5m, 3U);
  EXPECT_EQ(metrics.segments_shorter_than_10m, 3U);
}

TEST(PathSmoothing, ReportsOutsideGridLineOfSight) {
  const OccupancyGrid2D grid = makeGrid();

  const LineOfSightCheck check =
      checkLineOfSight(grid, GridIndex{-1, 0}, GridIndex{2, 0});

  EXPECT_FALSE(check.clear);
  EXPECT_EQ(check.reason, LineOfSightBlockReason::kOutsideGrid);
  EXPECT_EQ(check.checked_cells, 0U);
  EXPECT_EQ(check.prohibited_cells, 0U);
}

TEST(PathSmoothing, ReportsProhibitedLineOfSight) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{5, 5});
  grid.rebuildInflation(0.0);

  const LineOfSightCheck check =
      checkLineOfSight(grid, GridIndex{1, 5}, GridIndex{8, 5});

  EXPECT_FALSE(check.clear);
  EXPECT_EQ(check.reason, LineOfSightBlockReason::kProhibited);
  EXPECT_GT(check.checked_cells, 0U);
  EXPECT_GT(check.prohibited_cells, 0U);
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

TEST(PathSmoothing, ReportsSmoothingDiagnostics) {
  OccupancyGrid2D grid = makeGrid();
  for (int y = 0; y < 10; ++y) {
    grid.setOccupied(GridIndex{9, y});
  }
  grid.rebuildInflation(0.0);

  const AStarResult astar =
      AStarPlanner{}.plan(grid, GridIndex{1, 5}, GridIndex{18, 5});
  ASSERT_TRUE(astar.success);

  const PathSmoothingResult smoothing = smoothPathWithStats(grid, astar.path);

  EXPECT_EQ(smoothing.stats.input_points, astar.path.size());
  EXPECT_EQ(smoothing.stats.output_points, smoothing.path.size());
  EXPECT_GT(smoothing.stats.line_of_sight_checks, 0U);
  EXPECT_EQ(smoothing.stats.accepted_segments, smoothing.path.size() - 1U);
  EXPECT_GT(smoothing.stats.shortcut_segments, 0U);
  EXPECT_GT(smoothing.stats.rejected_segments, 0U);
  EXPECT_GT(smoothing.stats.rejected_prohibited, 0U);
  EXPECT_EQ(smoothing.stats.rejected_outside_grid, 0U);
  EXPECT_GT(smoothing.stats.rejected_prohibited_cells, 0U);
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

TEST(PlannerCore, ComputePathRejectsOccupiedStart) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{1, 1});
  grid.rebuildInflation(0.0);

  PlannerCore core{};

  const auto result = core.computePath(grid, Point2{1.5, 1.5}, Point2{18.5, 5.5});

  EXPECT_FALSE(result.has_value());
}

TEST(PlannerCore, ComputePathEscapesInflatedStart) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{5, 5});
  grid.rebuildInflation(1.1);
  ASSERT_FALSE(grid.isOccupied(GridIndex{4, 5}));
  ASSERT_TRUE(grid.isInflated(GridIndex{4, 5}));

  PlannerCore core{};

  const auto result = core.computePath(grid, Point2{4.5, 5.5}, Point2{18.5, 5.5});

  ASSERT_TRUE(result.has_value());
  const PathComputationResult& path_result =
      result.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_TRUE(path_result.start_escape_used);
  EXPECT_EQ(path_result.requested_start_cell, (GridIndex{4, 5}));
  ASSERT_TRUE(path_result.start_cell.has_value());
  const GridIndex escape_start =
      path_result.start_cell.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_FALSE(grid.isProhibited(escape_start));
  ASSERT_FALSE(path_result.astar.path.empty());
  EXPECT_EQ(path_result.astar.path.front(), escape_start);
  EXPECT_EQ(path_result.astar.path.back(), (GridIndex{18, 5}));
  EXPECT_TRUE(
      pathSegmentIsTraversable(grid, Point2{4.5, 5.5}, grid.cellCenter(escape_start)));
}

TEST(PlannerCore, ComputePathRejectsProhibitedGoal) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{18, 5});
  grid.rebuildInflation(0.0);

  PlannerCore core{};

  const auto result = core.computePath(grid, Point2{1.5, 1.5}, Point2{18.5, 5.5});

  EXPECT_FALSE(result.has_value());
}

TEST(PlannerCore, ComputePathRejectsOutOfGridGoal) {
  OccupancyGrid2D grid = makeGrid();
  PlannerCore core{};

  const auto result = core.computePath(grid, Point2{1.5, 1.5}, Point2{100.0, 100.0});

  EXPECT_FALSE(result.has_value());
}

TEST(PlannerCore, ComputePathReusesProhibitedClearanceFieldDiagnostics) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{10, 6});
  grid.rebuildInflation(1.1);
  PlannerCoreConfig config{};
  config.clearance_diagnostic_radius_m = 5.0;
  PlannerCore core{config};

  const auto first = core.computePath(grid, Point2{1.5, 1.5}, Point2{18.5, 1.5});
  const auto second = core.computePath(grid, Point2{1.5, 1.5}, Point2{18.5, 1.5});

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  const PathComputationResult& first_result =
      first.value(); // NOLINT(bugprone-unchecked-optional-access)
  const PathComputationResult& second_result =
      second.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_FALSE(first_result.prohibited_clearance_field_cache_hit);
  EXPECT_TRUE(second_result.prohibited_clearance_field_cache_hit);
  EXPECT_DOUBLE_EQ(
      first_result.raw_path_clearance_m,
      pathMinimumProhibitedClearanceM(grid, first_result.astar.path,
                                      config.clearance_diagnostic_radius_m));
  EXPECT_DOUBLE_EQ(
      first_result.smoothed_path_clearance_m,
      pathMinimumProhibitedClearanceM(grid, first_result.smoothed_cells,
                                      config.clearance_diagnostic_radius_m));
  EXPECT_DOUBLE_EQ(first_result.raw_path_clearance_m,
                   second_result.raw_path_clearance_m);
  EXPECT_DOUBLE_EQ(first_result.smoothed_path_clearance_m,
                   second_result.smoothed_path_clearance_m);
}

TEST(PlannerCore, ComputePathAcceptsPrebuiltProhibitedClearanceField) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{10, 6});
  grid.rebuildInflation(1.1);
  PlannerCoreConfig config{};
  config.clearance_diagnostic_radius_m = 5.0;
  const ClearanceField2D clearance = ClearanceField2D::build(
      grid, config.clearance_diagnostic_radius_m, ClearanceSource::kProhibited);
  PlannerCore core{config};

  const auto result = core.computePath(PathComputationInput{
      .grid = &grid,
      .current_position = Point2{1.5, 1.5},
      .goal = Point2{18.5, 1.5},
      .astar = config.astar,
      .prohibited_clearance_field = &clearance,
      .prohibited_clearance_field_cache_hit = true,
  });

  ASSERT_TRUE(result.has_value());
  const PathComputationResult& path_result =
      result.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_EQ(path_result.prohibited_clearance_field, &clearance);
  EXPECT_TRUE(path_result.prohibited_clearance_field_cache_hit);
  EXPECT_DOUBLE_EQ(
      path_result.raw_path_clearance_m,
      pathMinimumProhibitedClearanceM(grid, path_result.astar.path,
                                      config.clearance_diagnostic_radius_m));
}

TEST(PlannerCore, StablePathKeepsClearRemainingPath) {
  OccupancyGrid2D grid = makeGrid();
  PlannerCoreConfig config{};
  config.stable_path_goal_tolerance_m = 1.0;
  PlannerCore core{config};
  const std::vector<Point2> path{Point2{1.0, 1.0}, Point2{5.0, 1.0}, Point2{9.0, 1.0}};

  const StablePathDecision decision =
      core.evaluateStablePath(grid, path, Point2{3.0, 1.0}, Point2{9.0, 1.0});

  EXPECT_TRUE(decision.keep_path);
  EXPECT_EQ(decision.reason, StablePathDecisionReason::kClear);
  ASSERT_GE(decision.remaining_path.size(), 2U);
}

TEST(PlannerCore, StablePathRejectsProhibitedIntersectionImmediately) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{5, 1});
  grid.rebuildInflation(0.0);
  PlannerCoreConfig config{};
  config.stable_path_goal_tolerance_m = 1.0;
  PlannerCore core{config};
  const std::vector<Point2> path{Point2{1.5, 1.5}, Point2{8.5, 1.5}};

  const StablePathDecision decision =
      core.evaluateStablePath(grid, path, Point2{2.5, 1.5}, Point2{8.5, 1.5});

  EXPECT_FALSE(decision.keep_path);
  EXPECT_EQ(decision.reason, StablePathDecisionReason::kProhibitedConfirmed);
  EXPECT_EQ(decision.prohibited_segment_index, 0U);
  ASSERT_TRUE(decision.prohibited_intersection.has_value());
  const PathProhibitedIntersection intersection =
      decision.prohibited_intersection.value_or(PathProhibitedIntersection{});
  EXPECT_EQ(intersection.segment_index, 0U);
  const GridIndex expected_occupied_blocker{5, 1};
  EXPECT_EQ(intersection.cell, expected_occupied_blocker);
  EXPECT_TRUE(intersection.occupied);
  EXPECT_FALSE(intersection.inflated);
  EXPECT_NEAR(intersection.cell_center.x, 5.5, 1.0e-9);
  EXPECT_NEAR(intersection.cell_center.y, 1.5, 1.0e-9);
  EXPECT_NEAR(intersection.segment_t, 0.5, 1.0e-9);
  EXPECT_NEAR(intersection.path_distance_m, 3.0, 1.0e-9);
}

TEST(PlannerCore, StablePathTreatsInflationAsProhibited) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{5, 1});
  grid.rebuildInflation(2.0);
  ASSERT_FALSE(grid.isOccupied(GridIndex{5, 3}));
  ASSERT_TRUE(grid.isInflated(GridIndex{5, 3}));

  PlannerCoreConfig config{};
  config.stable_path_goal_tolerance_m = 1.0;
  PlannerCore core{config};
  const std::vector<Point2> path{Point2{1.5, 3.5}, Point2{8.5, 3.5}};

  const StablePathDecision decision =
      core.evaluateStablePath(grid, path, Point2{2.5, 3.5}, Point2{8.5, 3.5});

  EXPECT_FALSE(decision.keep_path);
  EXPECT_EQ(decision.reason, StablePathDecisionReason::kProhibitedConfirmed);
  EXPECT_EQ(decision.prohibited_segment_index, 0U);
  ASSERT_TRUE(decision.prohibited_intersection.has_value());
  const PathProhibitedIntersection intersection =
      decision.prohibited_intersection.value_or(PathProhibitedIntersection{});
  const GridIndex expected_inflated_blocker{4, 3};
  EXPECT_EQ(intersection.cell, expected_inflated_blocker);
  EXPECT_FALSE(intersection.occupied);
  EXPECT_TRUE(intersection.inflated);
  EXPECT_NEAR(intersection.cell_center.x, 4.5, 1.0e-9);
  EXPECT_NEAR(intersection.cell_center.y, 3.5, 1.0e-9);
}

TEST(PlannerCore, StablePathAllowsEscapeFromProhibitedStart) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{1, 1});
  grid.rebuildInflation(0.0);

  PlannerCoreConfig config{};
  config.stable_path_goal_tolerance_m = 1.0;
  PlannerCore core{config};
  const std::vector<Point2> path{Point2{1.5, 1.5}, Point2{8.5, 1.5}};

  const StablePathDecision decision =
      core.evaluateStablePath(grid, path, Point2{1.5, 1.5}, Point2{8.5, 1.5});

  EXPECT_TRUE(decision.keep_path);
  EXPECT_EQ(decision.reason, StablePathDecisionReason::kClear);
}

TEST(PlannerCore, StablePathKeepsClearPathDespiteLargeDeviation) {
  OccupancyGrid2D grid = makeGrid();
  PlannerCoreConfig config{};
  config.stable_path_goal_tolerance_m = 1.0;
  PlannerCore core{config};
  const std::vector<Point2> path{Point2{1.0, 1.0}, Point2{9.0, 1.0}};

  const StablePathDecision decision =
      core.evaluateStablePath(grid, path, Point2{3.0, 4.0}, Point2{9.0, 1.0});

  EXPECT_TRUE(decision.keep_path);
  EXPECT_EQ(decision.reason, StablePathDecisionReason::kClear);
  EXPECT_GT(decision.deviation_m, 0.5);
}

} // namespace drone_city_nav
