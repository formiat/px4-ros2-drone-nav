#include "drone_city_nav/astar_planner.hpp"
#include "drone_city_nav/grid_config.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/obstacle_memory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kFreshStampNs = 1'500'000'000LL;

[[nodiscard]] ObstacleMemoryGrid makeMemory() {
  return ObstacleMemoryGrid{GridBounds{0.0, 0.0, 1.0, 20, 12}};
}

[[nodiscard]] LaserScan2DView makeScan(const std::vector<float>& ranges,
                                       const double angle_min_rad = 0.0,
                                       const double angle_increment_rad = 0.1) {
  LaserScan2DView scan{};
  scan.ranges = std::span<const float>{ranges.data(), ranges.size()};
  scan.angle_min_rad = angle_min_rad;
  scan.angle_increment_rad = angle_increment_rad;
  scan.range_min_m = 0.1;
  scan.range_max_m = 20.0;
  return scan;
}

} // namespace

TEST(NavigationPose, Px4LocalPoseRequiresValidHeadingWhenConfigured) {
  const Px4LocalPositionSample invalid_heading_sample{
      3.0,           4.0,  -18.0, std::numeric_limits<double>::quiet_NaN(),
      kFreshStampNs, true, true,  false};

  const auto pose = makeNavigationPoseFromPx4LocalPosition(
      invalid_heading_sample, Px4LocalPoseConfig{true, 0.75});

  ASSERT_TRUE(pose.has_value());
  const NavigationPose2D pose_value =
      pose.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_TRUE(pose_value.position_valid);
  EXPECT_TRUE(pose_value.altitude_valid);
  EXPECT_FALSE(pose_value.yaw_valid);
  EXPECT_NEAR(pose_value.pose.position.x, 3.0, 1.0e-9);
  EXPECT_NEAR(pose_value.pose.position.y, 4.0, 1.0e-9);
  EXPECT_NEAR(pose_value.altitude_m, 18.0, 1.0e-9);
}

TEST(NavigationPose, Px4LocalPoseUsesInitialYawForMapAlignedScans) {
  const Px4LocalPositionSample invalid_heading_sample{
      3.0,           4.0,  -18.0, std::numeric_limits<double>::quiet_NaN(),
      kFreshStampNs, true, true,  false};

  const auto pose = makeNavigationPoseFromPx4LocalPosition(
      invalid_heading_sample, Px4LocalPoseConfig{false, 0.75});

  ASSERT_TRUE(pose.has_value());
  const NavigationPose2D pose_value =
      pose.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_TRUE(pose_value.position_valid);
  EXPECT_TRUE(pose_value.yaw_valid);
  EXPECT_NEAR(pose_value.pose.yaw_rad, 0.75, 1.0e-9);
}

TEST(NavigationPose, Px4LocalPoseUsesValidEstimatorHeadingWhenRequired) {
  const Px4LocalPositionSample valid_heading_sample{3.0,           4.0,  -18.0, -3.5,
                                                    kFreshStampNs, true, true,  true};

  const auto pose = makeNavigationPoseFromPx4LocalPosition(
      valid_heading_sample, Px4LocalPoseConfig{true, 0.75});

  ASSERT_TRUE(pose.has_value());
  const NavigationPose2D pose_value =
      pose.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_TRUE(pose_value.yaw_valid);
  EXPECT_NEAR(pose_value.pose.yaw_rad, normalizeYaw(-3.5), 1.0e-9);
}

TEST(NavigationPose, Px4LocalPoseAppliesMapOriginOffset) {
  const Px4LocalPositionSample sample{3.0,           4.0,  -18.0, 0.25,
                                      kFreshStampNs, true, true,  true};

  const auto pose = makeNavigationPoseFromPx4LocalPosition(
      sample, Px4LocalPoseConfig{true, 0.0, 18.0, 18.0});

  ASSERT_TRUE(pose.has_value());
  const NavigationPose2D pose_value =
      pose.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_TRUE(pose_value.position_valid);
  EXPECT_NEAR(pose_value.pose.position.x, 21.0, 1.0e-9);
  EXPECT_NEAR(pose_value.pose.position.y, 22.0, 1.0e-9);
}

TEST(NavigationPose, TimestampFreshnessRejectsMissingAndExpiredUpdates) {
  EXPECT_TRUE(timestampIsFresh(100, 150, 100));
  EXPECT_TRUE(timestampIsFresh(170, 150, 100, 25));
  EXPECT_TRUE(timestampIsFresh(0, 150, 0));
  EXPECT_FALSE(timestampIsFresh(0, 150, 100));
  EXPECT_FALSE(timestampIsFresh(100, 250, 100));
  EXPECT_FALSE(timestampIsFresh(200, 150, 100, 25));
}

TEST(NavigationPose, StatefulPx4UpdateInvalidatesCachedPoseAfterInvalidPosition) {
  NavigationPose2D state{};
  const Px4LocalPoseConfig config{true, 0.0};
  const Px4LocalPositionSample valid_sample{3.0,           4.0,  -18.0, 0.5,
                                            kFreshStampNs, true, true,  true};
  const Px4LocalPositionSample invalid_position_sample{
      std::numeric_limits<double>::quiet_NaN(),
      4.0,
      -18.0,
      0.5,
      kFreshStampNs,
      true,
      true,
      true};

  EXPECT_EQ(updateNavigationPoseFromPx4LocalPosition(valid_sample, config, state),
            Px4LocalPoseUpdateStatus::kAccepted);
  EXPECT_TRUE(state.position_valid);
  EXPECT_TRUE(state.yaw_valid);

  EXPECT_EQ(
      updateNavigationPoseFromPx4LocalPosition(invalid_position_sample, config, state),
      Px4LocalPoseUpdateStatus::kInvalidPosition);
  EXPECT_FALSE(state.position_valid);
  EXPECT_FALSE(state.yaw_valid);
  EXPECT_FALSE(state.altitude_valid);
}

TEST(NavigationPose, StatefulPx4UpdateInvalidatesCachedPoseAfterInvalidHeading) {
  NavigationPose2D state{};
  const Px4LocalPoseConfig config{true, 0.0};
  const Px4LocalPositionSample valid_sample{3.0,           4.0,  -18.0, 0.5,
                                            kFreshStampNs, true, true,  true};
  const Px4LocalPositionSample invalid_heading_sample{
      3.0,           4.0,  -18.0, std::numeric_limits<double>::quiet_NaN(),
      kFreshStampNs, true, true,  false};

  EXPECT_EQ(updateNavigationPoseFromPx4LocalPosition(valid_sample, config, state),
            Px4LocalPoseUpdateStatus::kAccepted);
  EXPECT_TRUE(state.position_valid);
  EXPECT_TRUE(state.yaw_valid);

  EXPECT_EQ(
      updateNavigationPoseFromPx4LocalPosition(invalid_heading_sample, config, state),
      Px4LocalPoseUpdateStatus::kInvalidYaw);
  EXPECT_FALSE(state.position_valid);
  EXPECT_FALSE(state.yaw_valid);
}

TEST(NavigationPose, ScanReadinessRequiresFreshPoseState) {
  NavigationPose2D state{};
  const Px4LocalPositionSample valid_sample{3.0,           4.0,  -18.0, 0.5,
                                            kFreshStampNs, true, true,  true};
  ASSERT_EQ(updateNavigationPoseFromPx4LocalPosition(
                valid_sample, Px4LocalPoseConfig{true, 0.0}, state),
            Px4LocalPoseUpdateStatus::kAccepted);

  EXPECT_TRUE(navigationPoseReadyForScan(state, 100, 150, 100));
  EXPECT_FALSE(navigationPoseReadyForScan(state, 100, 250, 100));
  state.yaw_valid = false;
  EXPECT_FALSE(navigationPoseReadyForScan(state, 200, 250, 100));
}

TEST(ObstacleMemoryGrid, ScanHitAtYawZeroOccupiesExpectedEndpoint) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> ranges{4.0F};

  const ObstacleMemoryStats stats =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, makeScan(ranges), {});

  EXPECT_EQ(stats.processed_beams, 1U);
  ASSERT_EQ(stats.newly_occupied_cells, 1U);
  ASSERT_EQ(stats.occupied_transitions.size(), 1U);
  const ObstacleMemoryOccupiedTransition& transition =
      stats.occupied_transitions.front();
  EXPECT_EQ(transition.beam_index, 0U);
  EXPECT_EQ(transition.cell, (GridIndex{9, 5}));
  EXPECT_NEAR(transition.endpoint_map_m.x, 9.5, 1.0e-9);
  EXPECT_NEAR(transition.endpoint_map_m.y, 5.5, 1.0e-9);
  EXPECT_TRUE(std::isnan(transition.endpoint_map_m.z));
  EXPECT_NEAR(transition.measured_range_m, 4.0, 1.0e-9);
  EXPECT_EQ(transition.score_before, 0);
  EXPECT_EQ(transition.score_after, 4);
  EXPECT_FALSE(transition.classifier_applied);
  EXPECT_EQ(stats.hit_beams, 1U);
  EXPECT_EQ(memory.rawGrid().state(GridIndex{9, 5}), CellState::kOccupied);
}

TEST(GridConfig, BoundedGridBoundsSanitizesInvalidAndHugeInputs) {
  const GridBounds invalid =
      boundedGridBounds(std::numeric_limits<double>::quiet_NaN(), 3.0, 0.0,
                        std::numeric_limits<double>::infinity(), -4.0);

  EXPECT_TRUE(gridBoundsUsable(invalid));
  EXPECT_EQ(invalid.origin_x, 0.0);
  EXPECT_EQ(invalid.origin_y, 3.0);
  EXPECT_EQ(invalid.resolution_m, 0.01);
  EXPECT_EQ(invalid.width_cells, 1);
  EXPECT_EQ(invalid.height_cells, 1);

  const GridBounds huge = boundedGridBounds(0.0, 0.0, 0.01, 1.0e9, 1.0e9);

  EXPECT_TRUE(gridBoundsUsable(huge));
  EXPECT_LE(gridBoundsCellCount(huge), kMaxGridCellCount);
}

TEST(OccupancyGrid2D, RejectsUnboundedCellCounts) {
  EXPECT_THROW((void)OccupancyGrid2D(GridBounds{0.0, 0.0, 0.01, 100000, 100000}),
               std::invalid_argument);
}

TEST(OccupancyGrid2D, WorldToCellRejectsNonFiniteAndOutOfRangeCoordinates) {
  const OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 10, 10}};

  EXPECT_FALSE(grid.worldToCell(Point2{std::numeric_limits<double>::quiet_NaN(), 0.0})
                   .has_value());
  EXPECT_FALSE(grid.worldToCell(Point2{0.0, std::numeric_limits<double>::infinity()})
                   .has_value());
  EXPECT_FALSE(
      grid.worldToCell(Point2{std::numeric_limits<double>::max(), 0.0}).has_value());
}

TEST(ObstacleMemoryGrid, ScanHitRotatesWithYaw) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> ranges{4.0F};

  const ObstacleMemoryStats stats = memory.integrateScan(
      Pose2{Point2{5.5, 5.5}, std::numbers::pi / 2.0}, makeScan(ranges), {});

  EXPECT_EQ(stats.processed_beams, 1U);
  EXPECT_EQ(memory.rawGrid().state(GridIndex{5, 9}), CellState::kOccupied);
}

TEST(ObstacleMemoryGrid, TiltedScanUsesPhysicalFrame) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> ranges{4.0F};
  LaserScan2DView scan = makeScan(ranges);
  scan.origin_altitude_m = 18.0;
  scan.pitch_rad = -0.4;
  scan.altitude_valid = true;
  scan.attitude_valid = true;
  scan.compensate_attitude = true;
  scan.min_projected_altitude_m = 1.0;
  scan.max_projected_altitude_m = 40.0;

  const ObstacleMemoryStats stats =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, scan, {});

  EXPECT_EQ(stats.processed_beams, 1U);
  EXPECT_EQ(stats.hit_beams, 1U);
  EXPECT_EQ(stats.altitude_rejected_beams, 0U);
  EXPECT_EQ(memory.rawGrid().state(GridIndex{9, 5}), CellState::kOccupied);
  EXPECT_NE(memory.rawGrid().state(GridIndex{5, 9}), CellState::kOccupied);
}

TEST(ObstacleMemoryGrid, PersistentObstacleSurvivesOneFreeMiss) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> hit_ranges{4.0F};
  const std::vector<float> miss_ranges{std::numeric_limits<float>::infinity()};

  const ObstacleMemoryStats hit_stats =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, makeScan(hit_ranges), {});
  EXPECT_EQ(hit_stats.hit_beams, 1U);
  ASSERT_EQ(memory.rawGrid().state(GridIndex{9, 5}), CellState::kOccupied);
  const ObstacleMemoryStats miss_stats =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, makeScan(miss_ranges), {});

  EXPECT_EQ(miss_stats.processed_beams, 1U);
  EXPECT_EQ(memory.rawGrid().state(GridIndex{9, 5}), CellState::kOccupied);
}

TEST(ObstacleMemoryGrid, RepeatedHitDoesNotReportAnotherOccupiedTransition) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> hit_ranges{4.0F};

  const ObstacleMemoryStats first =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, makeScan(hit_ranges), {});
  const ObstacleMemoryStats second =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, makeScan(hit_ranges), {});

  ASSERT_EQ(first.newly_occupied_cells, 1U);
  ASSERT_EQ(first.occupied_transitions.size(), 1U);
  EXPECT_EQ(second.newly_occupied_cells, 0U);
  EXPECT_TRUE(second.occupied_transitions.empty());
}

TEST(ObstacleMemoryGrid, InvalidRangeAndInvalidPoseDoNotChangeMap) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> invalid_ranges{0.01F};
  const std::vector<float> valid_ranges{4.0F};

  const ObstacleMemoryStats invalid_range_stats =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, makeScan(invalid_ranges), {});
  const ObstacleMemoryStats invalid_pose_stats = memory.integrateScan(
      Pose2{Point2{5.5, 5.5}, std::numeric_limits<double>::quiet_NaN()},
      makeScan(valid_ranges), {});

  EXPECT_EQ(invalid_range_stats.processed_beams, 0U);
  EXPECT_EQ(invalid_pose_stats.processed_beams, 0U);
  EXPECT_EQ(memory.countRawCells().occupied_cells, 0U);
  EXPECT_EQ(memory.countRawCells().free_cells, 0U);
}

TEST(ObstacleMemoryGrid, InvalidScoreOrderingDoesNotUpdateMap) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> ranges{4.0F};
  ObstacleMemoryConfig config{};
  config.max_score = 2;
  config.occupied_score = 3;

  const ObstacleMemoryStats stats =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, makeScan(ranges), config);

  EXPECT_EQ(stats.processed_beams, 0U);
  EXPECT_EQ(memory.countRawCells().occupied_cells, 0U);
}

TEST(ObstacleMemoryGrid, ClipsFreeRayToGridBoundary) {
  ObstacleMemoryGrid memory{GridBounds{0.0, 0.0, 1.0, 10, 10}};
  const std::vector<float> ranges{std::numeric_limits<float>::infinity()};

  const ObstacleMemoryStats stats =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, makeScan(ranges), {});

  EXPECT_GT(stats.clipped_rays, 0U);
  EXPECT_EQ(memory.rawGrid().state(GridIndex{9, 5}), CellState::kFree);
}

TEST(ObstacleMemoryGrid, HitOutsideGridClearsInBoundsRayWithoutOccupiedEndpoint) {
  ObstacleMemoryGrid memory{GridBounds{0.0, 0.0, 1.0, 10, 10}};
  const std::vector<float> ranges{20.0F};
  LaserScan2DView scan = makeScan(ranges);
  scan.range_max_m = 30.0;
  ObstacleMemoryConfig config{};
  config.max_lidar_range_m = 30.0;

  const ObstacleMemoryStats stats =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, scan, config);

  EXPECT_EQ(stats.outside_hit_endpoints, 1U);
  EXPECT_EQ(memory.countRawCells().occupied_cells, 0U);
  EXPECT_EQ(memory.rawGrid().state(GridIndex{9, 5}), CellState::kFree);
}

TEST(ObstacleMemoryGrid, StoresRawMemoryWithoutInflation) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> ranges{4.0F};

  const ObstacleMemoryStats stats =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, makeScan(ranges), {});
  EXPECT_EQ(stats.hit_beams, 1U);

  EXPECT_TRUE(memory.rawGrid().isOccupied(GridIndex{9, 5}));
  EXPECT_FALSE(memory.rawGrid().isProhibited(GridIndex{8, 5}));
}

TEST(ObstacleMemoryGrid, ResetClearsScoresAndRawStates) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> ranges{4.0F};
  ASSERT_EQ(memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, makeScan(ranges), {})
                .occupied_cells_updated,
            1U);
  ASSERT_EQ(memory.countRawCells().occupied_cells, 1U);

  memory.reset();

  const GridCellCounts counts = memory.countRawCells();
  EXPECT_EQ(counts.occupied_cells, 0U);
  EXPECT_EQ(counts.free_cells, 0U);
  EXPECT_EQ(counts.unknown_cells, memory.rawGrid().cellCount());
}

TEST(PlannerOnMemory, AStarAvoidsRememberedAndInflatedObstacle) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> ranges{5.0F};

  const ObstacleMemoryStats stats =
      memory.integrateScan(Pose2{Point2{4.5, 5.5}, 0.0}, makeScan(ranges), {});
  EXPECT_EQ(stats.hit_beams, 1U);
  OccupancyGrid2D planning_grid = memory.rawGrid();
  planning_grid.rebuildInflation(1.1);

  const GridIndex start{1, 5};
  const GridIndex goal{18, 5};
  const AStarResult result = AStarPlanner{}.plan(planning_grid, start, goal);

  ASSERT_TRUE(result.success);
  for (const GridIndex cell : result.path) {
    EXPECT_FALSE(planning_grid.isProhibited(cell));
  }
}

} // namespace drone_city_nav
