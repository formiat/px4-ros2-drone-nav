#include "drone_city_nav/planning_grid_builder.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] GridBounds testBounds() {
  return GridBounds{0.0, 0.0, 1.0, 8, 8};
}

[[nodiscard]] PlanningGridBuilderConfig testConfig() {
  PlanningGridBuilderConfig config{};
  config.fallback_bounds = testBounds();
  config.inflation_radius_m = 1.1;
  config.planning_clearance_m = 1.0;
  return config;
}

} // namespace

TEST(PlanningGridBuilder, StaticOnlyBuildsInflatedGrid) {
  PlanningGridBuilderConfig config = testConfig();
  OccupancyGrid2D static_grid{testBounds()};
  static_grid.setOccupied(GridIndex{3, 3});
  PlanningGridSources sources{};
  sources.static_grid = &static_grid;

  const PlanningGridBuildResult result = buildPlanningGrid(config, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.grid.has_value());
  ASSERT_TRUE(result.planning_grid.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const OccupancyGrid2D& grid = result.grid.value();
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const OccupancyGrid2D& planning_grid = result.planning_grid.value();
  EXPECT_TRUE(result.static_source.used);
  EXPECT_TRUE(grid.isOccupied(GridIndex{3, 3}));
  EXPECT_TRUE(grid.isInflated(GridIndex{4, 3}));
  EXPECT_TRUE(planning_grid.isInflated(GridIndex{5, 3}));
  EXPECT_FALSE(grid.isInflated(GridIndex{5, 3}));
}

TEST(PlanningGridBuilder, MemoryGeometryMismatchIsReportedAndSkipped) {
  PlanningGridBuilderConfig config = testConfig();
  OccupancyGrid2D static_grid{testBounds()};
  OccupancyGrid2D memory_grid{GridBounds{0.0, 0.0, 0.5, 8, 8}};
  static_grid.setOccupied(GridIndex{1, 1});
  memory_grid.setOccupied(GridIndex{4, 4});
  PlanningGridSources sources{};
  sources.static_grid = &static_grid;
  sources.memory_grid = &memory_grid;

  const PlanningGridBuildResult result = buildPlanningGrid(config, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.grid.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const OccupancyGrid2D& grid = result.grid.value();
  EXPECT_TRUE(result.static_source.used);
  EXPECT_TRUE(result.memory.seen);
  EXPECT_FALSE(result.memory.geometry_matches);
  EXPECT_FALSE(result.memory.used);
  EXPECT_TRUE(grid.isOccupied(GridIndex{1, 1}));
  EXPECT_FALSE(grid.isOccupied(GridIndex{4, 4}));
}

TEST(PlanningGridBuilder, NoReadySourceDataReturnsHoldStatus) {
  PlanningGridBuilderConfig config = testConfig();
  config.use_static_map = false;

  const PlanningGridBuildResult result =
      buildPlanningGrid(config, PlanningGridSources{});

  EXPECT_EQ(result.status, PlanningGridStatus::kNoReadySourceData);
  EXPECT_FALSE(result.grid.has_value());
  EXPECT_FALSE(result.planning_grid.has_value());
}

TEST(PlanningGridBuilder, CurrentLidarOverlayWinsAsFreshSource) {
  PlanningGridBuilderConfig config = testConfig();
  config.use_static_map = false;
  OccupancyGrid2D current_lidar_grid{testBounds()};
  current_lidar_grid.setOccupied(GridIndex{5, 2});
  PlanningGridSources sources{};
  sources.current_lidar_grid = &current_lidar_grid;
  sources.current_lidar.enabled = true;
  sources.current_lidar.used = true;
  sources.current_lidar.fresh = true;
  sources.current_lidar.occupied_cells = 1U;

  const PlanningGridBuildResult result = buildPlanningGrid(config, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.grid.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const OccupancyGrid2D& grid = result.grid.value();
  EXPECT_TRUE(result.current_lidar.used);
  EXPECT_TRUE(grid.isOccupied(GridIndex{5, 2}));
}

TEST(PlanningGridBuilder, SourceUnionInflatesOnceAfterRawObstacleMerge) {
  PlanningGridBuilderConfig config = testConfig();
  OccupancyGrid2D static_grid{testBounds()};
  static_grid.setOccupied(GridIndex{3, 3});
  OccupancyGrid2D current_lidar_grid{testBounds()};
  current_lidar_grid.setOccupied(GridIndex{4, 3});
  current_lidar_grid.setOccupied(GridIndex{7, 7});
  PlanningGridSources sources{};
  sources.static_grid = &static_grid;
  sources.current_lidar_grid = &current_lidar_grid;
  sources.current_lidar.enabled = true;
  sources.current_lidar.used = true;
  sources.current_lidar.fresh = true;
  sources.current_lidar.occupied_cells = 2U;

  const PlanningGridBuildResult result = buildPlanningGrid(config, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.grid.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const OccupancyGrid2D& grid = result.grid.value();
  EXPECT_TRUE(grid.isOccupied(GridIndex{3, 3}));
  EXPECT_TRUE(grid.isProhibited(GridIndex{4, 3}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{4, 3}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{7, 7}));
  EXPECT_EQ(result.current_lidar.overlay_occupied_cells_applied, 2U);
}

TEST(PlanningGridBuilder, SourceInflatedCellsAreNotReusedAsRawObstacles) {
  PlanningGridBuilderConfig config = testConfig();
  config.use_static_map = false;
  OccupancyGrid2D memory_grid{testBounds()};
  memory_grid.setOccupied(GridIndex{3, 3});
  memory_grid.rebuildInflation(1.1);
  ASSERT_TRUE(memory_grid.isInflated(GridIndex{4, 3}));
  PlanningGridSources sources{};
  sources.memory_grid = &memory_grid;

  const PlanningGridBuildResult result = buildPlanningGrid(config, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.grid.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const OccupancyGrid2D& grid = result.grid.value();
  EXPECT_TRUE(grid.isOccupied(GridIndex{3, 3}));
  EXPECT_FALSE(grid.isOccupied(GridIndex{4, 3}));
  EXPECT_TRUE(grid.isInflated(GridIndex{4, 3}));
  EXPECT_EQ(result.memory.overlay.occupied_cells_applied, 1U);
}

TEST(PlanningGridBuilder, CurrentLidarOnlyUsesLidarBoundsWhenFallbackDiffers) {
  PlanningGridBuilderConfig config = testConfig();
  config.use_static_map = false;
  config.fallback_bounds = GridBounds{-100.0, -100.0, 5.0, 3, 3};
  const GridBounds lidar_bounds{10.0, 20.0, 0.5, 6, 4};
  OccupancyGrid2D current_lidar_grid{lidar_bounds};
  current_lidar_grid.setOccupied(GridIndex{2, 1});
  PlanningGridSources sources{};
  sources.current_lidar_grid = &current_lidar_grid;
  sources.current_lidar.enabled = true;
  sources.current_lidar.used = true;
  sources.current_lidar.fresh = true;
  sources.current_lidar.occupied_cells = 1U;

  const PlanningGridBuildResult result = buildPlanningGrid(config, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.grid.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const OccupancyGrid2D& grid = result.grid.value();
  EXPECT_DOUBLE_EQ(grid.originX(), lidar_bounds.origin_x);
  EXPECT_DOUBLE_EQ(grid.originY(), lidar_bounds.origin_y);
  EXPECT_DOUBLE_EQ(grid.resolution(), lidar_bounds.resolution_m);
  EXPECT_EQ(grid.width(), lidar_bounds.width_cells);
  EXPECT_EQ(grid.height(), lidar_bounds.height_cells);
  EXPECT_TRUE(grid.isOccupied(GridIndex{2, 1}));
}

} // namespace drone_city_nav
