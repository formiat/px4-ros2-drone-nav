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
  return config;
}

} // namespace

TEST(PlanningGridBuilder, StaticOnlyBuildsInflatedGrid) {
  PlanningGridBuilderConfig config = testConfig();
  config.use_obstacle_memory = false;
  config.use_current_lidar_obstacles = false;
  OccupancyGrid2D static_grid{testBounds()};
  static_grid.setOccupied(GridIndex{3, 3});
  PlanningGridSources sources{};
  sources.static_grid = &static_grid;

  const PlanningGridBuildResult result = buildPlanningGrid(config, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.grid.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const OccupancyGrid2D& grid = result.grid.value();
  EXPECT_TRUE(result.static_source.used);
  EXPECT_TRUE(grid.isOccupied(GridIndex{3, 3}));
  EXPECT_TRUE(grid.isInflated(GridIndex{4, 3}));
}

TEST(PlanningGridBuilder, MemoryGeometryMismatchIsReportedAndSkipped) {
  PlanningGridBuilderConfig config = testConfig();
  config.use_current_lidar_obstacles = false;
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

TEST(PlanningGridBuilder, NoEnabledSourcesReturnsHoldStatus) {
  PlanningGridBuilderConfig config = testConfig();
  config.use_static_map = false;
  config.use_obstacle_memory = false;
  config.use_current_lidar_obstacles = false;

  const PlanningGridBuildResult result =
      buildPlanningGrid(config, PlanningGridSources{});

  EXPECT_EQ(result.status, PlanningGridStatus::kNoEnabledSources);
  EXPECT_FALSE(result.grid.has_value());
}

TEST(PlanningGridBuilder, CurrentLidarOverlayWinsAsFreshSource) {
  PlanningGridBuilderConfig config = testConfig();
  config.use_static_map = false;
  config.use_obstacle_memory = false;
  config.use_current_lidar_obstacles = true;
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

} // namespace drone_city_nav
