#include "drone_city_nav/planning_grid_builder.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] GridBounds bounds() {
  return GridBounds{0.0, 0.0, 1.0, 30, 30};
}

[[nodiscard]] OccupancyGrid2D freeGrid() {
  OccupancyGrid2D grid{bounds()};
  grid.reset(CellState::kFree);
  return grid;
}

[[nodiscard]] PlanningGridSources staticSources(const OccupancyGrid2D& grid) {
  PlanningGridSources sources{};
  sources.static_grid = &grid;
  sources.static_rectangles = 1U;
  sources.static_occupied_cells = 1U;
  return sources;
}

} // namespace

TEST(ObstacleFieldBuilder, MergesRawSourcesWithoutInflation) {
  OccupancyGrid2D static_grid = freeGrid();
  static_grid.setOccupied(GridIndex{5, 5});
  OccupancyGrid2D memory = freeGrid();
  memory.setOccupied(GridIndex{10, 10});
  PlanningGridSources sources = staticSources(static_grid);
  sources.memory_grid = &memory;
  sources.memory_producer_instance_id = 7U;
  sources.memory_sequence = 9U;

  const ObstacleFieldBuildResult result =
      buildObstacleField(ObstacleFieldBuilderConfig{}, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.raw_occupancy.has_value());
  const OccupancyGrid2D& raw = result.raw_occupancy.value();
  EXPECT_TRUE(raw.isOccupied(GridIndex{5, 5}));
  EXPECT_TRUE(raw.isOccupied(GridIndex{10, 10}));
  EXPECT_FALSE(raw.isOccupied(GridIndex{6, 5}));
  EXPECT_EQ(result.applied_memory_producer_instance_id, 7U);
  EXPECT_EQ(result.applied_memory_sequence, 9U);
}

TEST(ObstacleFieldBuilder, BuildsModeAwareRiskPolicy) {
  OccupancyGrid2D static_grid = freeGrid();
  static_grid.setOccupied(GridIndex{5, 5});
  const ObstacleFieldBuildResult result = buildObstacleField(
      ObstacleFieldBuilderConfig{
          .use_static_map = true,
          .fallback_bounds = bounds(),
          .local_planning_bounds = std::nullopt,
          .inflation_radius_m = 1.0,
          .planning_clearance_m = 5.0,
      },
      staticSources(static_grid));

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  EXPECT_DOUBLE_EQ(result.risk_policy.critical_distance_m, 1.0);
  EXPECT_DOUBLE_EQ(result.risk_policy.preferred_distance_m, 6.0);
}

TEST(ObstacleFieldBuilder, LocalWindowIncludesPreferredDistanceHalo) {
  OccupancyGrid2D static_grid = freeGrid();
  static_grid.setOccupied(GridIndex{20, 15});
  const GridBounds evaluation{10.0, 10.0, 1.0, 6, 6};
  const ObstacleFieldBuildResult result = buildObstacleField(
      ObstacleFieldBuilderConfig{
          .use_static_map = true,
          .fallback_bounds = bounds(),
          .local_planning_bounds = evaluation,
          .inflation_radius_m = 1.0,
          .planning_clearance_m = 5.0,
      },
      staticSources(static_grid));

  ASSERT_TRUE(result.raw_occupancy.has_value());
  ASSERT_TRUE(result.evaluation_bounds.has_value());
  EXPECT_EQ(result.evaluation_bounds.value().width_cells, evaluation.width_cells);
  EXPECT_GT(result.raw_occupancy.value().width(), evaluation.width_cells);
  EXPECT_TRUE(result.raw_occupancy.value().worldToCell(Point2{20.5, 15.5}).has_value());
  EXPECT_TRUE(result.cache.local_planning_window_applied);
}

TEST(ObstacleFieldBuilder, ReportsNoReadySource) {
  const ObstacleFieldBuildResult result = buildObstacleField(
      ObstacleFieldBuilderConfig{
          .use_static_map = false,
          .fallback_bounds = bounds(),
          .local_planning_bounds = std::nullopt,
      },
      PlanningGridSources{});

  EXPECT_EQ(result.status, PlanningGridStatus::kNoReadySourceData);
  EXPECT_FALSE(result.raw_occupancy.has_value());
}

TEST(ObstacleFieldBuilder, StaticCacheTracksOnlyRawFingerprint) {
  OccupancyGrid2D static_grid = freeGrid();
  static_grid.setOccupied(GridIndex{5, 5});
  ObstacleFieldBuilder builder;
  const ObstacleFieldBuilderConfig config{
      .use_static_map = true,
      .fallback_bounds = bounds(),
      .local_planning_bounds = std::nullopt,
      .inflation_radius_m = 1.0,
      .planning_clearance_m = 3.0,
  };

  const ObstacleFieldBuildResult first =
      builder.build(config, staticSources(static_grid));
  const ObstacleFieldBuildResult second =
      builder.build(config, staticSources(static_grid));
  ObstacleFieldBuilderConfig changed = config;
  changed.planning_clearance_m = 5.0;
  const ObstacleFieldBuildResult third =
      builder.build(changed, staticSources(static_grid));

  EXPECT_TRUE(first.cache.static_cache_rebuilt);
  EXPECT_TRUE(second.cache.static_cache_hit);
  EXPECT_TRUE(third.cache.static_cache_hit);
  EXPECT_FALSE(third.cache.static_cache_rebuilt);
  EXPECT_DOUBLE_EQ(third.risk_policy.preferred_distance_m, 6.0);
}

TEST(ObstacleFieldBuilder, MemoryGeometryMismatchIsSkipped) {
  OccupancyGrid2D static_grid = freeGrid();
  OccupancyGrid2D memory{GridBounds{0.0, 0.0, 0.5, 20, 20}};
  memory.reset(CellState::kFree);
  PlanningGridSources sources = staticSources(static_grid);
  sources.memory_grid = &memory;

  const ObstacleFieldBuildResult result =
      buildObstacleField(ObstacleFieldBuilderConfig{}, sources);

  ASSERT_TRUE(result.raw_occupancy.has_value());
  EXPECT_TRUE(result.memory.seen);
  EXPECT_FALSE(result.memory.geometry_matches);
  EXPECT_FALSE(result.memory.used);
}

} // namespace drone_city_nav
