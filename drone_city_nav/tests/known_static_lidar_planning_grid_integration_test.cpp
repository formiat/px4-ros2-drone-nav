#include "drone_city_nav/current_lidar_overlay.hpp"
#include "drone_city_nav/known_passage_solid_volumes.hpp"
#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"
#include "drone_city_nav/obstacle_memory.hpp"
#include "drone_city_nav/planning_grid_builder.hpp"

#include <gtest/gtest.h>

#include <array>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] PassageStructure connector2223() {
  PassageOpening opening{};
  opening.id = "connector_22_23_opening";
  opening.structure_id = "physical_building_connector_22_23";
  opening.center = Point3{135.0, 324.0, 5.0};
  opening.normal_xy = Point2{1.0, 0.0};
  opening.width_m = 7.0;
  opening.height_m = 7.0;
  opening.depth_m = 24.0;
  opening.min_z_m = 1.5;
  opening.max_z_m = 8.5;

  PassageStructure structure{};
  structure.id = opening.structure_id;
  structure.center = Point2{135.0, 324.0};
  structure.size_x_m = 24.0;
  structure.size_y_m = 30.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 32.0;
  structure.openings.push_back(opening);
  return structure;
}

struct GridScenario {
  ObstacleMemoryGrid memory;
  OccupancyGrid2D current_lidar_grid;
  CurrentLidarOverlayStats current_lidar_stats;
  Point2 endpoint{};

  explicit GridScenario(const GridBounds& bounds)
      : memory{bounds},
        current_lidar_grid{bounds} {
  }
};

[[nodiscard]] GridScenario ingest(const KnownStaticLidarHitClassifier& classifier,
                                  const double altitude_m, const float range_m) {
  const GridBounds bounds{90.0, 300.0, 0.5, 120, 100};
  GridScenario scenario{bounds};
  const std::array<float, 1U> ranges{range_m};
  LaserScan2DView memory_scan{};
  memory_scan.ranges = ranges;
  memory_scan.angle_min_rad = 0.0;
  memory_scan.angle_increment_rad = 0.1;
  memory_scan.range_min_m = 0.1;
  memory_scan.range_max_m = 35.0;
  memory_scan.origin_altitude_m = altitude_m;
  memory_scan.altitude_valid = true;
  memory_scan.attitude_valid = true;
  memory_scan.min_projected_altitude_m = 0.0;
  memory_scan.max_projected_altitude_m = 40.0;
  const Pose2 pose{Point2{100.0, 324.0}, 0.0};
  const ObstacleMemoryStats memory_stats = scenario.memory.integrateScan(
      pose, memory_scan, ObstacleMemoryConfig{}, &classifier);
  (void)memory_stats;

  LidarProjectionConfig projection_config{};
  projection_config.max_lidar_range_m = 35.0;
  projection_config.min_projected_altitude_m = 0.0;
  projection_config.max_projected_altitude_m = 40.0;
  const LidarProjectionPose projection_pose{pose.position, altitude_m, 0.0, 0.0,
                                            0.0,           true,       true};
  scenario.current_lidar_stats = overlayCurrentLidarHits(
      scenario.current_lidar_grid, LidarScanView{ranges, 0.1, 35.0, 0.0, 0.1},
      projection_pose, projection_config, &classifier);
  scenario.current_lidar_stats.enabled = true;
  scenario.current_lidar_stats.fresh = true;
  scenario.endpoint = Point2{100.0 + static_cast<double>(range_m), 324.0};
  return scenario;
}

[[nodiscard]] PlanningGridBuildResult buildResult(const GridScenario& scenario) {
  PlanningGridBuilderConfig config{};
  config.use_static_map = false;
  config.fallback_bounds = scenario.memory.rawGrid().bounds();
  config.inflation_radius_m = 1.0;
  config.planning_clearance_m = 3.0;
  PlanningGridSources sources{};
  sources.memory_grid = &scenario.memory.rawGrid();
  sources.current_lidar_grid = &scenario.current_lidar_grid;
  sources.current_lidar = scenario.current_lidar_stats;
  return buildPlanningGrid(config, sources);
}

void expectEndpointProhibited(const PlanningGridBuildResult& result,
                              const Point2 endpoint, const bool expected) {
  ASSERT_TRUE(result.grid.has_value());
  ASSERT_TRUE(result.planning_grid.has_value());
  const OccupancyGrid2D& grid =
      result.grid.value(); // NOLINT(bugprone-unchecked-optional-access)
  const OccupancyGrid2D& planning_grid =
      result.planning_grid.value(); // NOLINT(bugprone-unchecked-optional-access)
  const auto endpoint_cell = grid.worldToCell(endpoint);
  ASSERT_TRUE(endpoint_cell.has_value());
  const GridIndex cell =
      endpoint_cell.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_EQ(grid.isProhibited(cell), expected);
  EXPECT_EQ(planning_grid.isProhibited(cell), expected);
}

} // namespace

TEST(KnownStaticLidarPlanningGridIntegration,
     Connector2223UpperHitIsAbsentAfterMergeAndInflation) {
  const KnownStaticLidarHitClassifier classifier{
      knownPassageSolidVolumes(connector2223())};
  const GridScenario scenario = ingest(classifier, 13.8, 23.0F);

  const PlanningGridBuildResult result = buildResult(scenario);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  EXPECT_EQ(
      scenario.current_lidar_stats.known_static_lidar.expected_static_hits_ignored, 1U);
  EXPECT_EQ(scenario.memory.countRawCells().occupied_cells, 0U);
  expectEndpointProhibited(result, scenario.endpoint, false);
}

TEST(KnownStaticLidarPlanningGridIntegration,
     Connector2223CloserBlockerRemainsProhibited) {
  const KnownStaticLidarHitClassifier classifier{
      knownPassageSolidVolumes(connector2223())};
  const GridScenario scenario = ingest(classifier, 13.8, 20.0F);

  const PlanningGridBuildResult result = buildResult(scenario);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  EXPECT_EQ(scenario.current_lidar_stats.known_static_lidar.unexpected_hits_kept, 1U);
  EXPECT_EQ(scenario.memory.countRawCells().occupied_cells, 1U);
  expectEndpointProhibited(result, scenario.endpoint, true);
}

TEST(KnownStaticLidarPlanningGridIntegration,
     Connector2223OpeningBlockerRemainsProhibited) {
  const KnownStaticLidarHitClassifier classifier{
      knownPassageSolidVolumes(connector2223())};
  const GridScenario scenario = ingest(classifier, 5.0, 23.0F);

  const PlanningGridBuildResult result = buildResult(scenario);

  ASSERT_EQ(result.status, PlanningGridStatus::kReady);
  EXPECT_EQ(scenario.current_lidar_stats.known_static_lidar.unexpected_hits_kept, 1U);
  EXPECT_EQ(scenario.memory.countRawCells().occupied_cells, 1U);
  expectEndpointProhibited(result, scenario.endpoint, true);
}

} // namespace drone_city_nav
