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

constexpr std::int64_t kNowNs = 2'000'000'000LL;
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

[[nodiscard]] GpsFixSample makeFix() {
  return GpsFixSample{40.0, -74.0, 100.0, kFreshStampNs, 0, true, 1.0};
}

[[nodiscard]] QuaternionSample yawQuaternion(const double yaw_rad) {
  return QuaternionSample{std::cos(yaw_rad / 2.0), 0.0, 0.0, std::sin(yaw_rad / 2.0)};
}

} // namespace

TEST(NavigationPose, ProjectsWgs84ToNorthEastLocalFrame) {
  const GeoReference origin{40.0, -74.0, 100.0, true};
  GpsFixSample north_fix = makeFix();
  north_fix.latitude_deg += 10.0 / 6'378'137.0 * 180.0 / std::numbers::pi;
  GpsFixSample east_fix = makeFix();
  east_fix.longitude_deg += 10.0 /
                            (6'378'137.0 * std::cos(40.0 * std::numbers::pi / 180.0)) *
                            180.0 / std::numbers::pi;

  const Point2 north = projectWgs84ToLocal(north_fix, origin);
  const Point2 east = projectWgs84ToLocal(east_fix, origin);

  EXPECT_NEAR(north.x, 10.0, 0.05);
  EXPECT_NEAR(north.y, 0.0, 0.05);
  EXPECT_NEAR(east.x, 0.0, 0.05);
  EXPECT_NEAR(east.y, 10.0, 0.05);
}

TEST(NavigationPose, HandlesManualAndAutoOrigin) {
  GpsCompassConfig manual_config{};
  manual_config.auto_initialize_origin = false;
  manual_config.origin_latitude_deg = 40.0;
  manual_config.origin_longitude_deg = -74.0;
  manual_config.origin_altitude_m = 100.0;
  GeoReference manual_origin{};

  const auto manual_pose = makeNavigationPoseFromGpsCompass(
      makeFix(), 0.0, kNowNs, manual_config, manual_origin);

  ASSERT_TRUE(manual_pose.has_value());
  const NavigationPose2D manual_pose_value =
      manual_pose.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_TRUE(manual_origin.initialized);
  EXPECT_NEAR(manual_pose_value.pose.position.x, 0.0, 1.0e-6);
  EXPECT_NEAR(manual_pose_value.pose.position.y, 0.0, 1.0e-6);

  GpsCompassConfig auto_config{};
  GeoReference auto_origin{};
  const auto auto_pose = makeNavigationPoseFromGpsCompass(makeFix(), 0.0, kNowNs,
                                                          auto_config, auto_origin);

  ASSERT_TRUE(auto_pose.has_value());
  const NavigationPose2D auto_pose_value =
      auto_pose.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_TRUE(auto_origin.initialized);
  EXPECT_NEAR(auto_pose_value.pose.position.x, 0.0, 1.0e-6);
  EXPECT_NEAR(auto_pose_value.pose.position.y, 0.0, 1.0e-6);
}

TEST(NavigationPose, RejectsInvalidGpsFixes) {
  GpsCompassConfig config{};
  GeoReference origin{};

  GpsFixSample no_fix = makeFix();
  no_fix.status = -1;
  EXPECT_FALSE(makeNavigationPoseFromGpsCompass(no_fix, 0.0, kNowNs, config, origin)
                   .has_value());

  GpsFixSample nan_fix = makeFix();
  nan_fix.latitude_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(makeNavigationPoseFromGpsCompass(nan_fix, 0.0, kNowNs, config, origin)
                   .has_value());

  GpsFixSample stale_fix = makeFix();
  stale_fix.stamp_ns = 1;
  EXPECT_FALSE(makeNavigationPoseFromGpsCompass(stale_fix, 0.0, kNowNs, config, origin)
                   .has_value());

  GpsFixSample noisy_fix = makeFix();
  noisy_fix.horizontal_variance_m2 = 1000.0;
  EXPECT_FALSE(makeNavigationPoseFromGpsCompass(noisy_fix, 0.0, kNowNs, config, origin)
                   .has_value());
}

TEST(NavigationPose, ConvertsCompassYawAndOffsets) {
  const auto yaw = yawFromQuaternion(yawQuaternion(std::numbers::pi / 2.0));

  ASSERT_TRUE(yaw.has_value());
  const double yaw_value = yaw.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_NEAR(yaw_value, std::numbers::pi / 2.0, 1.0e-9);

  GpsCompassConfig config{};
  config.yaw_offset_rad = 0.1;
  config.magnetic_declination_rad = 0.2;
  config.compass_to_body_yaw_offset_rad = 0.3;
  GeoReference origin{};
  const auto pose =
      makeNavigationPoseFromGpsCompass(makeFix(), 0.4, kNowNs, config, origin);

  ASSERT_TRUE(pose.has_value());
  const NavigationPose2D pose_value =
      pose.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_NEAR(pose_value.pose.yaw_rad, 1.0, 1.0e-9);
}

TEST(NavigationPose, CompassYawStateInvalidatesUnavailableAndInvalidYaw) {
  CompassYawState state{};

  EXPECT_EQ(updateCompassYawFromQuaternion(yawQuaternion(0.4), true, 100, state),
            CompassYawUpdateStatus::kAccepted);
  EXPECT_TRUE(state.valid);
  EXPECT_NEAR(state.yaw_rad, 0.4, 1.0e-9);

  EXPECT_EQ(updateCompassYawFromQuaternion(yawQuaternion(0.4), false, 200, state),
            CompassYawUpdateStatus::kUnavailable);
  EXPECT_FALSE(state.valid);

  EXPECT_EQ(updateCompassYawFromQuaternion(yawQuaternion(0.4), true, 300, state),
            CompassYawUpdateStatus::kAccepted);
  QuaternionSample invalid_quaternion = yawQuaternion(0.4);
  invalid_quaternion.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(updateCompassYawFromQuaternion(invalid_quaternion, true, 400, state),
            CompassYawUpdateStatus::kInvalidYaw);
  EXPECT_FALSE(state.valid);
}

TEST(NavigationPose, GpsCompassStateRequiresFreshCompassYaw) {
  GpsCompassConfig config{};
  GeoReference origin{};
  NavigationPose2D pose{};
  CompassYawState compass{};
  std::optional<GpsFixSample> gps = makeFix();

  ASSERT_EQ(
      updateCompassYawFromQuaternion(yawQuaternion(0.4), true, kNowNs - 100, compass),
      CompassYawUpdateStatus::kAccepted);
  EXPECT_EQ(updateNavigationPoseFromGpsCompassState(gps, compass, kNowNs, 1000, config,
                                                    origin, pose),
            GpsCompassPoseUpdateStatus::kAccepted);
  EXPECT_TRUE(navigationPoseReadyForScan(pose, kNowNs, kNowNs, 1000));

  gps->stamp_ns = kNowNs + 2'000'000'000LL - 100;
  EXPECT_EQ(updateNavigationPoseFromGpsCompassState(
                gps, compass, kNowNs + 2'000'000'000LL, 1000, config, origin, pose),
            GpsCompassPoseUpdateStatus::kStaleCompassYaw);
  EXPECT_FALSE(pose.position_valid);
  EXPECT_FALSE(pose.yaw_valid);
  EXPECT_FALSE(navigationPoseReadyForScan(pose, 0, kNowNs + 2'000'000'000LL, 1000));
}

TEST(NavigationPose, GpsCompassStateDoesNotRevivePoseAfterInvalidCompass) {
  GpsCompassConfig config{};
  GeoReference origin{};
  NavigationPose2D pose{};
  CompassYawState compass{};
  std::optional<GpsFixSample> gps = makeFix();

  ASSERT_EQ(
      updateCompassYawFromQuaternion(yawQuaternion(0.4), true, kNowNs - 100, compass),
      CompassYawUpdateStatus::kAccepted);
  ASSERT_EQ(updateNavigationPoseFromGpsCompassState(gps, compass, kNowNs, 1000, config,
                                                    origin, pose),
            GpsCompassPoseUpdateStatus::kAccepted);

  EXPECT_EQ(
      updateCompassYawFromQuaternion(yawQuaternion(0.4), false, kNowNs + 10, compass),
      CompassYawUpdateStatus::kUnavailable);
  gps->stamp_ns = kNowNs + 20;
  EXPECT_EQ(updateNavigationPoseFromGpsCompassState(gps, compass, kNowNs + 30, 1000,
                                                    config, origin, pose),
            GpsCompassPoseUpdateStatus::kMissingCompassYaw);
  EXPECT_FALSE(pose.position_valid);
  EXPECT_FALSE(pose.yaw_valid);

  EXPECT_EQ(
      updateCompassYawFromQuaternion(yawQuaternion(0.7), true, kNowNs + 40, compass),
      CompassYawUpdateStatus::kAccepted);
  EXPECT_EQ(updateNavigationPoseFromGpsCompassState(gps, compass, kNowNs + 50, 1000,
                                                    config, origin, pose),
            GpsCompassPoseUpdateStatus::kAccepted);
  EXPECT_TRUE(pose.position_valid);
  EXPECT_TRUE(pose.yaw_valid);
  EXPECT_NEAR(pose.pose.yaw_rad, 0.7, 1.0e-9);
}

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
  EXPECT_TRUE(timestampIsFresh(200, 150, 100));
  EXPECT_TRUE(timestampIsFresh(0, 150, 0));
  EXPECT_FALSE(timestampIsFresh(0, 150, 100));
  EXPECT_FALSE(timestampIsFresh(100, 250, 100));
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

TEST(ObstacleMemoryGrid, ScanHitRotatesWithYaw) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> ranges{4.0F};

  const ObstacleMemoryStats stats = memory.integrateScan(
      Pose2{Point2{5.5, 5.5}, std::numbers::pi / 2.0}, makeScan(ranges), {});

  EXPECT_EQ(stats.processed_beams, 1U);
  EXPECT_EQ(memory.rawGrid().state(GridIndex{5, 9}), CellState::kOccupied);
}

TEST(ObstacleMemoryGrid, TiltedScanUsesPhysicalFrameInsteadOfLegacySwap) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> ranges{4.0F};
  LaserScan2DView scan = makeScan(ranges);
  scan.origin_altitude_m = 18.0;
  scan.pitch_rad = -0.4;
  scan.altitude_valid = true;
  scan.attitude_valid = true;
  scan.compensate_attitude = true;
  scan.swap_lidar_xy_to_local_frame = true;
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

TEST(ObstacleMemoryGrid, ObstacleDepthClipsAtBoundary) {
  ObstacleMemoryGrid memory{GridBounds{0.0, 0.0, 1.0, 10, 10}};
  const std::vector<float> ranges{1.0F};
  ObstacleMemoryConfig config{};
  config.hit_obstacle_depth_m = 5.0;

  const ObstacleMemoryStats stats =
      memory.integrateScan(Pose2{Point2{8.5, 5.5}, 0.0}, makeScan(ranges), config);

  EXPECT_GT(stats.obstacle_depth_cells, 0U);
  EXPECT_EQ(memory.rawGrid().state(GridIndex{9, 5}), CellState::kOccupied);
}

TEST(ObstacleMemoryGrid, InflationBlocksRememberedObstacle) {
  ObstacleMemoryGrid memory = makeMemory();
  const std::vector<float> ranges{4.0F};

  const ObstacleMemoryStats stats =
      memory.integrateScan(Pose2{Point2{5.5, 5.5}, 0.0}, makeScan(ranges), {});
  EXPECT_EQ(stats.hit_beams, 1U);
  memory.rebuildInflation(1.1);

  EXPECT_TRUE(memory.inflatedGrid().isBlocked(GridIndex{9, 5}));
  EXPECT_TRUE(memory.inflatedGrid().isBlocked(GridIndex{8, 5}));
  EXPECT_FALSE(memory.inflatedGrid().isBlocked(GridIndex{12, 5}));
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
    EXPECT_FALSE(planning_grid.isBlocked(cell));
  }
}

} // namespace drone_city_nav
