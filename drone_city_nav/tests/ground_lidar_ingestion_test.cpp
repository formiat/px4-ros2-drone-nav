#include "drone_city_nav/current_lidar_overlay.hpp"
#include "drone_city_nav/obstacle_memory.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr double kGroundAltitudeM = 0.05;

[[nodiscard]] LidarProjectionPose downwardPose() {
  return LidarProjectionPose{Point2{2.0, 0.0}, 8.0, 0.0, 0.0, -0.7, true, true};
}

[[nodiscard]] LidarProjectionConfig projectionConfig() {
  LidarProjectionConfig config{};
  config.max_lidar_range_m = 30.0;
  config.min_projected_altitude_m = -100.0;
  config.max_projected_altitude_m = 100.0;
  config.compensate_attitude = true;
  return config;
}

[[nodiscard]] double expectedGroundRange() {
  const LidarProjectionPose pose = downwardPose();
  const LidarProjectionConfig config = projectionConfig();
  const LidarBeamProjection projection =
      projectLidarBeam(pose, config, 0.1, 30.0, 0.0, 0.1, 0U, 30.0F);
  EXPECT_EQ(projection.status, LidarBeamProjectionStatus::kAccepted);
  EXPECT_LT(projection.ray_direction_map.z, 0.0);
  return (kGroundAltitudeM - projection.ray_origin_map_m.z) /
         projection.ray_direction_map.z;
}

[[nodiscard]] LaserScan2DView memoryScan(const std::array<float, 1U>& ranges) {
  const LidarProjectionPose pose = downwardPose();
  LaserScan2DView scan{};
  scan.ranges = ranges;
  scan.angle_increment_rad = 0.1;
  scan.range_min_m = 0.1;
  scan.range_max_m = 30.0;
  scan.origin_altitude_m = pose.altitude_m;
  scan.roll_rad = pose.roll_rad;
  scan.pitch_rad = pose.pitch_rad;
  scan.min_projected_altitude_m = -100.0;
  scan.max_projected_altitude_m = 100.0;
  scan.altitude_valid = true;
  scan.attitude_valid = true;
  scan.compensate_attitude = true;
  return scan;
}

[[nodiscard]] LidarScanView overlayScan(const std::array<float, 1U>& ranges) {
  return LidarScanView{ranges, 0.1, 30.0, 0.0, 0.1, {}};
}

} // namespace

TEST(GroundLidarIngestion, ExpectedGroundMutatesNeitherMemoryNorOverlay) {
  const std::array<float, 1U> ranges{static_cast<float>(expectedGroundRange())};
  const GroundLidarRejectionConfig ground{
      .enabled = true,
      .ground_altitude_m = kGroundAltitudeM,
      .closer_range_tolerance_m = 0.5,
      .farther_range_tolerance_m = 1.5,
  };
  const GridBounds bounds{-5.0, -15.0, 0.5, 80, 60};

  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats = memory.integrateScan(
      Pose2{downwardPose().position, downwardPose().yaw_rad}, memoryScan(ranges),
      ObstacleMemoryConfig{}, nullptr, &ground);
  OccupancyGrid2D overlay{bounds};
  const CurrentLidarOverlayStats overlay_stats =
      overlayCurrentLidarHits(overlay, overlayScan(ranges), downwardPose(),
                              projectionConfig(), nullptr, &ground);

  EXPECT_EQ(memory_stats.free_cells_updated, 0U);
  EXPECT_EQ(memory_stats.occupied_cells_updated, 0U);
  EXPECT_EQ(memory_stats.ingestion_decisions.expected_ground_suppressed, 1U);
  EXPECT_EQ(memory.countRawCells().occupied_cells, 0U);
  EXPECT_EQ(memory.countRawCells().free_cells, 0U);
  EXPECT_EQ(overlay_stats.ingestion_decisions.expected_ground_suppressed, 1U);
  EXPECT_EQ(overlay_stats.occupied_cells, 0U);
}

TEST(GroundLidarIngestion, ExpectedGroundDoesNotClearExistingMemoryCell) {
  const GroundLidarRejectionConfig ground{
      .enabled = true,
      .ground_altitude_m = kGroundAltitudeM,
      .closer_range_tolerance_m = 0.5,
      .farther_range_tolerance_m = 1.5,
  };
  const GridBounds bounds{-5.0, -15.0, 0.5, 80, 60};
  ObstacleMemoryGrid memory{bounds};
  const std::array<float, 1U> seed_range{4.0F};
  LaserScan2DView seed_scan = memoryScan(seed_range);
  seed_scan.pitch_rad = 0.0;
  ASSERT_EQ(memory
                .integrateScan(Pose2{downwardPose().position, 0.0}, seed_scan,
                               ObstacleMemoryConfig{})
                .occupied_cells_updated,
            1U);
  ASSERT_EQ(memory.countRawCells().occupied_cells, 1U);

  const std::array<float, 1U> ground_range{static_cast<float>(expectedGroundRange())};
  const ObstacleMemoryStats stats = memory.integrateScan(
      Pose2{downwardPose().position, downwardPose().yaw_rad}, memoryScan(ground_range),
      ObstacleMemoryConfig{}, nullptr, &ground);

  EXPECT_EQ(stats.free_cells_updated, 0U);
  EXPECT_EQ(stats.ingestion_decisions.expected_ground_suppressed, 1U);
  EXPECT_EQ(memory.countRawCells().occupied_cells, 1U);
}

TEST(GroundLidarIngestion, CloserObstacleIsRetainedByMemoryAndOverlay) {
  const double obstacle_range_m = expectedGroundRange() - 2.0;
  ASSERT_GT(obstacle_range_m, 0.1);
  const std::array<float, 1U> ranges{static_cast<float>(obstacle_range_m)};
  const GroundLidarRejectionConfig ground{
      .enabled = true,
      .ground_altitude_m = kGroundAltitudeM,
      .closer_range_tolerance_m = 0.5,
      .farther_range_tolerance_m = 1.5,
  };
  const GridBounds bounds{-5.0, -15.0, 0.5, 80, 60};

  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats = memory.integrateScan(
      Pose2{downwardPose().position, downwardPose().yaw_rad}, memoryScan(ranges),
      ObstacleMemoryConfig{}, nullptr, &ground);
  OccupancyGrid2D overlay{bounds};
  const CurrentLidarOverlayStats overlay_stats =
      overlayCurrentLidarHits(overlay, overlayScan(ranges), downwardPose(),
                              projectionConfig(), nullptr, &ground);

  EXPECT_EQ(memory_stats.ingestion_decisions.closer_obstacles_retained, 1U);
  EXPECT_EQ(memory.countRawCells().occupied_cells, 1U);
  EXPECT_EQ(overlay_stats.ingestion_decisions.closer_obstacles_retained, 1U);
  EXPECT_EQ(overlay_stats.occupied_cells, 1U);
}

TEST(GroundLidarIngestion, GroundClassificationPrecedesAltitudeVeto) {
  const std::array<float, 1U> ranges{30.0F};
  const GroundLidarRejectionConfig ground{
      .enabled = true,
      .ground_altitude_m = kGroundAltitudeM,
      .closer_range_tolerance_m = 0.5,
      .farther_range_tolerance_m = 1.5,
  };
  LidarProjectionConfig config = projectionConfig();
  config.min_projected_altitude_m = 1.0;
  const GridBounds bounds{-5.0, -15.0, 0.5, 80, 60};

  ObstacleMemoryGrid memory{bounds};
  LaserScan2DView scan = memoryScan(ranges);
  scan.min_projected_altitude_m = 1.0;
  const ObstacleMemoryStats memory_stats =
      memory.integrateScan(Pose2{downwardPose().position, downwardPose().yaw_rad}, scan,
                           ObstacleMemoryConfig{}, nullptr, &ground);
  OccupancyGrid2D overlay{bounds};
  const CurrentLidarOverlayStats overlay_stats = overlayCurrentLidarHits(
      overlay, overlayScan(ranges), downwardPose(), config, nullptr, &ground);

  EXPECT_EQ(memory_stats.altitude_rejected_beams, 1U);
  EXPECT_EQ(memory_stats.ingestion_decisions.expected_ground_suppressed, 1U);
  EXPECT_EQ(memory_stats.ingestion_decisions.non_ground_altitude_rejected, 0U);
  EXPECT_EQ(memory_stats.free_cells_updated, 0U);
  EXPECT_EQ(overlay_stats.altitude_rejected_beams, 1U);
  EXPECT_EQ(overlay_stats.ingestion_decisions.expected_ground_suppressed, 1U);
  EXPECT_EQ(overlay_stats.ingestion_decisions.non_ground_altitude_rejected, 0U);
}

TEST(GroundLidarIngestion, FartherGroundHitIsDiagnosedBeforeAltitudeVeto) {
  const double farther_range_m = expectedGroundRange() + 2.0;
  ASSERT_LT(farther_range_m, 30.0);
  const std::array<float, 1U> ranges{static_cast<float>(farther_range_m)};
  const GroundLidarRejectionConfig ground{
      .enabled = true,
      .ground_altitude_m = kGroundAltitudeM,
      .closer_range_tolerance_m = 0.5,
      .farther_range_tolerance_m = 1.5,
  };
  LidarProjectionConfig config = projectionConfig();
  config.min_projected_altitude_m = 1.0;
  const GridBounds bounds{-5.0, -15.0, 0.5, 80, 60};

  ObstacleMemoryGrid memory{bounds};
  LaserScan2DView scan = memoryScan(ranges);
  scan.min_projected_altitude_m = 1.0;
  const ObstacleMemoryStats memory_stats =
      memory.integrateScan(Pose2{downwardPose().position, downwardPose().yaw_rad}, scan,
                           ObstacleMemoryConfig{}, nullptr, &ground);
  OccupancyGrid2D overlay{bounds};
  const CurrentLidarOverlayStats overlay_stats = overlayCurrentLidarHits(
      overlay, overlayScan(ranges), downwardPose(), config, nullptr, &ground);

  EXPECT_EQ(memory_stats.altitude_rejected_beams, 1U);
  EXPECT_EQ(memory_stats.ingestion_decisions.ambiguous_ground_suppressed, 1U);
  EXPECT_EQ(memory_stats.free_cells_updated, 0U);
  EXPECT_EQ(overlay_stats.altitude_rejected_beams, 1U);
  EXPECT_EQ(overlay_stats.ingestion_decisions.ambiguous_ground_suppressed, 1U);
  EXPECT_EQ(overlay_stats.occupied_cells, 0U);
}

} // namespace drone_city_nav
