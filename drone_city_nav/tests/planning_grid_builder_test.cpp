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

void expectSameGrid(const OccupancyGrid2D& lhs, const OccupancyGrid2D& rhs) {
  EXPECT_DOUBLE_EQ(lhs.originX(), rhs.originX());
  EXPECT_DOUBLE_EQ(lhs.originY(), rhs.originY());
  EXPECT_DOUBLE_EQ(lhs.resolution(), rhs.resolution());
  ASSERT_EQ(lhs.width(), rhs.width());
  ASSERT_EQ(lhs.height(), rhs.height());
  ASSERT_EQ(lhs.cellCount(), rhs.cellCount());
  for (int y = 0; y < lhs.height(); ++y) {
    for (int x = 0; x < lhs.width(); ++x) {
      const GridIndex cell{x, y};
      EXPECT_EQ(lhs.state(cell), rhs.state(cell));
      EXPECT_EQ(lhs.isInflated(cell), rhs.isInflated(cell));
      EXPECT_EQ(lhs.isProhibited(cell), rhs.isProhibited(cell));
    }
  }
}

void expectEquivalentReadyResult(const PlanningGridBuildResult& expected,
                                 const PlanningGridBuildResult& actual) {
  ASSERT_EQ(expected.status, PlanningGridStatus::kReady);
  ASSERT_EQ(actual.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(expected.grid.has_value());
  ASSERT_TRUE(actual.grid.has_value());
  ASSERT_TRUE(expected.planning_grid.has_value());
  ASSERT_TRUE(actual.planning_grid.has_value());
  const OccupancyGrid2D* expected_grid = expected.grid ? &*expected.grid : nullptr;
  const OccupancyGrid2D* actual_grid = actual.grid ? &*actual.grid : nullptr;
  const OccupancyGrid2D* expected_planning_grid =
      expected.planning_grid ? &*expected.planning_grid : nullptr;
  const OccupancyGrid2D* actual_planning_grid =
      actual.planning_grid ? &*actual.planning_grid : nullptr;
  ASSERT_NE(expected_grid, nullptr);
  ASSERT_NE(actual_grid, nullptr);
  ASSERT_NE(expected_planning_grid, nullptr);
  ASSERT_NE(actual_planning_grid, nullptr);
  expectSameGrid(*expected_grid, *actual_grid);
  expectSameGrid(*expected_planning_grid, *actual_planning_grid);
  EXPECT_EQ(expected.static_source.used, actual.static_source.used);
  EXPECT_EQ(expected.static_source.occupied_cells, actual.static_source.occupied_cells);
  EXPECT_EQ(expected.memory.used, actual.memory.used);
  EXPECT_EQ(expected.memory.geometry_matches, actual.memory.geometry_matches);
  EXPECT_EQ(expected.memory.overlay.occupied_cells_applied,
            actual.memory.overlay.occupied_cells_applied);
  EXPECT_EQ(expected.memory.overlay.free_cells_applied,
            actual.memory.overlay.free_cells_applied);
  EXPECT_EQ(expected.current_lidar.occupied_cells, actual.current_lidar.occupied_cells);
  EXPECT_EQ(expected.current_lidar.overlay_occupied_cells_applied,
            actual.current_lidar.overlay_occupied_cells_applied);
  EXPECT_EQ(expected.current_lidar.overlay_occupied_cells_preserved,
            actual.current_lidar.overlay_occupied_cells_preserved);
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
  sources.memory_producer_instance_id = 77U;
  sources.memory_sequence = 91U;

  const PlanningGridBuildResult result = buildPlanningGrid(config, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.grid.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const OccupancyGrid2D& grid = result.grid.value();
  EXPECT_TRUE(result.static_source.used);
  EXPECT_TRUE(result.memory.seen);
  EXPECT_FALSE(result.memory.geometry_matches);
  EXPECT_FALSE(result.memory.used);
  EXPECT_EQ(result.applied_memory_producer_instance_id, 0U);
  EXPECT_EQ(result.applied_memory_sequence, 0U);
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
  sources.lidar_update_ns = 12345;

  const PlanningGridBuildResult result = buildPlanningGrid(config, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.grid.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const OccupancyGrid2D& grid = result.grid.value();
  EXPECT_TRUE(result.current_lidar.used);
  EXPECT_EQ(result.applied_lidar_update_ns, 12345);
  EXPECT_TRUE(grid.isOccupied(GridIndex{5, 2}));
}

TEST(PlanningGridBuilder, StaleLidarDoesNotClaimAppliedIdentity) {
  PlanningGridBuilderConfig config = testConfig();
  OccupancyGrid2D static_grid{testBounds()};
  static_grid.setOccupied(GridIndex{1, 1});
  OccupancyGrid2D current_lidar_grid{testBounds()};
  current_lidar_grid.setOccupied(GridIndex{5, 2});
  PlanningGridSources sources{};
  sources.static_grid = &static_grid;
  sources.current_lidar_grid = &current_lidar_grid;
  sources.current_lidar.enabled = true;
  sources.current_lidar.used = true;
  sources.current_lidar.fresh = false;
  sources.lidar_update_ns = 98765;

  const PlanningGridBuildResult result = buildPlanningGrid(config, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.raw_grid.has_value());
  EXPECT_EQ(result.applied_lidar_update_ns, 0);
  EXPECT_FALSE(result.raw_grid->isOccupied(GridIndex{5, 2}));
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
  sources.memory_producer_instance_id = 13U;
  sources.memory_sequence = 29U;

  const PlanningGridBuildResult result = buildPlanningGrid(config, sources);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  ASSERT_TRUE(result.grid.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded by ASSERT_TRUE above.
  const OccupancyGrid2D& grid = result.grid.value();
  EXPECT_TRUE(grid.isOccupied(GridIndex{3, 3}));
  EXPECT_FALSE(grid.isOccupied(GridIndex{4, 3}));
  EXPECT_TRUE(grid.isInflated(GridIndex{4, 3}));
  EXPECT_EQ(result.memory.overlay.occupied_cells_applied, 1U);
  EXPECT_EQ(result.applied_memory_producer_instance_id, 13U);
  EXPECT_EQ(result.applied_memory_sequence, 29U);
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

TEST(PlanningGridBuilder, CachedStaticOnlyMatchesFullBuild) {
  const PlanningGridBuilderConfig config = testConfig();
  OccupancyGrid2D static_grid{testBounds()};
  static_grid.setOccupied(GridIndex{2, 2});
  PlanningGridSources sources{};
  sources.static_grid = &static_grid;
  PlanningGridBuilder builder;

  const PlanningGridBuildResult first = builder.build(config, sources);
  const PlanningGridBuildResult cached = builder.build(config, sources);
  const PlanningGridBuildResult full = buildPlanningGrid(config, sources);

  ASSERT_EQ(first.status, PlanningGridStatus::kReady);
  EXPECT_TRUE(first.cache.static_cache_eligible);
  EXPECT_FALSE(first.cache.static_cache_hit);
  EXPECT_TRUE(first.cache.static_cache_rebuilt);
  EXPECT_EQ(first.cache.static_distance_source_cells, 1U);
  EXPECT_TRUE(cached.cache.static_cache_eligible);
  EXPECT_TRUE(cached.cache.static_cache_hit);
  EXPECT_FALSE(cached.cache.static_cache_rebuilt);
  EXPECT_EQ(cached.cache.static_distance_source_cells, 1U);
  EXPECT_EQ(cached.cache.dynamic_distance_source_cells, 0U);
  EXPECT_DOUBLE_EQ(cached.cache.dynamic_distance_field_duration_ms, 0.0);
  expectEquivalentReadyResult(full, cached);
}

TEST(PlanningGridBuilder, CachedStaticPlusMemoryMatchesFullBuild) {
  const PlanningGridBuilderConfig config = testConfig();
  OccupancyGrid2D static_grid{testBounds()};
  static_grid.setOccupied(GridIndex{1, 1});
  OccupancyGrid2D memory_grid{testBounds()};
  memory_grid.setFree(GridIndex{2, 2});
  memory_grid.setOccupied(GridIndex{6, 6});
  PlanningGridSources warm_sources{};
  warm_sources.static_grid = &static_grid;
  PlanningGridSources sources = warm_sources;
  sources.memory_grid = &memory_grid;
  PlanningGridBuilder builder;

  ASSERT_EQ(builder.build(config, warm_sources).status, PlanningGridStatus::kReady);
  const PlanningGridBuildResult cached = builder.build(config, sources);
  const PlanningGridBuildResult full = buildPlanningGrid(config, sources);

  EXPECT_TRUE(cached.cache.static_cache_hit);
  EXPECT_EQ(cached.cache.dynamic_distance_source_cells, 1U);
  expectEquivalentReadyResult(full, cached);
}

TEST(PlanningGridBuilder, CachedStaticPlusCurrentLidarMatchesFullBuild) {
  const PlanningGridBuilderConfig config = testConfig();
  OccupancyGrid2D static_grid{testBounds()};
  static_grid.setOccupied(GridIndex{1, 1});
  OccupancyGrid2D current_lidar_grid{testBounds()};
  current_lidar_grid.setOccupied(GridIndex{6, 2});
  PlanningGridSources warm_sources{};
  warm_sources.static_grid = &static_grid;
  PlanningGridSources sources = warm_sources;
  sources.current_lidar_grid = &current_lidar_grid;
  sources.current_lidar.enabled = true;
  sources.current_lidar.used = true;
  sources.current_lidar.fresh = true;
  sources.current_lidar.occupied_cells = 1U;
  PlanningGridBuilder builder;

  ASSERT_EQ(builder.build(config, warm_sources).status, PlanningGridStatus::kReady);
  const PlanningGridBuildResult cached = builder.build(config, sources);
  const PlanningGridBuildResult full = buildPlanningGrid(config, sources);

  EXPECT_TRUE(cached.cache.static_cache_hit);
  EXPECT_EQ(cached.cache.dynamic_distance_source_cells, 1U);
  expectEquivalentReadyResult(full, cached);
}

TEST(PlanningGridBuilder, CachedStaticInvalidatesWhenStaticCellsChange) {
  const PlanningGridBuilderConfig config = testConfig();
  OccupancyGrid2D static_grid{testBounds()};
  static_grid.setOccupied(GridIndex{1, 1});
  PlanningGridSources sources{};
  sources.static_grid = &static_grid;
  PlanningGridBuilder builder;

  ASSERT_EQ(builder.build(config, sources).status, PlanningGridStatus::kReady);
  static_grid.setOccupied(GridIndex{6, 6});
  const PlanningGridBuildResult changed = builder.build(config, sources);
  const PlanningGridBuildResult full = buildPlanningGrid(config, sources);

  EXPECT_TRUE(changed.cache.static_cache_eligible);
  EXPECT_FALSE(changed.cache.static_cache_hit);
  EXPECT_TRUE(changed.cache.static_cache_rebuilt);
  expectEquivalentReadyResult(full, changed);
}

TEST(PlanningGridBuilder, CachedStaticInvalidatesWhenBoundsChange) {
  const PlanningGridBuilderConfig config = testConfig();
  OccupancyGrid2D static_grid{testBounds()};
  static_grid.setOccupied(GridIndex{1, 1});
  PlanningGridSources sources{};
  sources.static_grid = &static_grid;
  PlanningGridBuilder builder;

  ASSERT_EQ(builder.build(config, sources).status, PlanningGridStatus::kReady);
  OccupancyGrid2D shifted_static_grid{GridBounds{1.0, 2.0, 0.5, 8, 8}};
  shifted_static_grid.setOccupied(GridIndex{1, 1});
  sources.static_grid = &shifted_static_grid;
  const PlanningGridBuildResult changed = builder.build(config, sources);
  const PlanningGridBuildResult full = buildPlanningGrid(config, sources);

  EXPECT_TRUE(changed.cache.static_cache_eligible);
  EXPECT_FALSE(changed.cache.static_cache_hit);
  EXPECT_TRUE(changed.cache.static_cache_rebuilt);
  expectEquivalentReadyResult(full, changed);
}

TEST(PlanningGridBuilder, CachedStaticInvalidatesWhenInflationConfigChanges) {
  PlanningGridBuilderConfig config = testConfig();
  OccupancyGrid2D static_grid{testBounds()};
  static_grid.setOccupied(GridIndex{3, 3});
  PlanningGridSources sources{};
  sources.static_grid = &static_grid;
  PlanningGridBuilder builder;

  ASSERT_EQ(builder.build(config, sources).status, PlanningGridStatus::kReady);
  config.inflation_radius_m = 2.1;
  const PlanningGridBuildResult inflation_changed = builder.build(config, sources);
  const PlanningGridBuildResult inflation_full = buildPlanningGrid(config, sources);
  EXPECT_FALSE(inflation_changed.cache.static_cache_hit);
  EXPECT_TRUE(inflation_changed.cache.static_cache_rebuilt);
  expectEquivalentReadyResult(inflation_full, inflation_changed);

  ASSERT_EQ(builder.build(config, sources).status, PlanningGridStatus::kReady);
  config.planning_clearance_m = 2.0;
  const PlanningGridBuildResult clearance_changed = builder.build(config, sources);
  const PlanningGridBuildResult clearance_full = buildPlanningGrid(config, sources);
  EXPECT_FALSE(clearance_changed.cache.static_cache_hit);
  EXPECT_TRUE(clearance_changed.cache.static_cache_rebuilt);
  expectEquivalentReadyResult(clearance_full, clearance_changed);
}

TEST(PlanningGridBuilder, DynamicSourcesDoNotMutateCachedStaticGrids) {
  const PlanningGridBuilderConfig config = testConfig();
  OccupancyGrid2D static_grid{testBounds()};
  static_grid.setOccupied(GridIndex{1, 1});
  OccupancyGrid2D memory_grid{testBounds()};
  memory_grid.setOccupied(GridIndex{6, 6});
  OccupancyGrid2D current_lidar_grid{testBounds()};
  current_lidar_grid.setOccupied(GridIndex{7, 7});
  PlanningGridSources static_sources{};
  static_sources.static_grid = &static_grid;
  PlanningGridSources dynamic_sources = static_sources;
  dynamic_sources.memory_grid = &memory_grid;
  dynamic_sources.current_lidar_grid = &current_lidar_grid;
  dynamic_sources.current_lidar.enabled = true;
  dynamic_sources.current_lidar.used = true;
  dynamic_sources.current_lidar.fresh = true;
  dynamic_sources.current_lidar.occupied_cells = 1U;
  PlanningGridBuilder builder;

  ASSERT_EQ(builder.build(config, static_sources).status, PlanningGridStatus::kReady);
  const PlanningGridBuildResult dynamic_result = builder.build(config, dynamic_sources);
  ASSERT_EQ(dynamic_result.status, PlanningGridStatus::kReady);
  EXPECT_TRUE(dynamic_result.cache.static_cache_hit);
  const PlanningGridBuildResult static_after_dynamic =
      builder.build(config, static_sources);

  ASSERT_EQ(static_after_dynamic.status, PlanningGridStatus::kReady);
  EXPECT_TRUE(static_after_dynamic.cache.static_cache_hit);
  ASSERT_TRUE(static_after_dynamic.grid.has_value());
  const OccupancyGrid2D* grid =
      static_after_dynamic.grid ? &*static_after_dynamic.grid : nullptr;
  ASSERT_NE(grid, nullptr);
  EXPECT_FALSE(grid->isOccupied(GridIndex{6, 6}));
  EXPECT_FALSE(grid->isProhibited(GridIndex{6, 6}));
  EXPECT_FALSE(grid->isOccupied(GridIndex{7, 7}));
  EXPECT_FALSE(grid->isProhibited(GridIndex{7, 7}));
}

} // namespace drone_city_nav
