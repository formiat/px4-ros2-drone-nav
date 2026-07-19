#include "drone_city_nav/current_lidar_overlay.hpp"
#include "drone_city_nav/obstacle_memory.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <span>
#include <string>

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

void useSourceAlignedPose(LaserScan2DView& scan, const LidarProjectionPose& pose) {
  scan.beam_projection_poses = std::span<const LidarProjectionPose>{&pose, 1U};
  scan.projection_pose_source = LidarProjectionPoseSource::kSourceTimestampAligned;
}

void useSourceAlignedPose(LidarScanView& scan, const LidarProjectionPose& pose) {
  scan.beam_projection_poses = std::span<const LidarProjectionPose>{&pose, 1U};
  scan.projection_pose_source = LidarProjectionPoseSource::kSourceTimestampAligned;
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
  LidarProjectionPose seed_pose = downwardPose();
  seed_pose.pitch_rad = 0.0;
  LaserScan2DView seed_scan = memoryScan(seed_range);
  seed_scan.pitch_rad = 0.0;
  useSourceAlignedPose(seed_scan, seed_pose);
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
  const double obstacle_range_m = expectedGroundRange() - 3.0;
  ASSERT_GT(obstacle_range_m, 0.1);
  const std::array<float, 1U> ranges{static_cast<float>(obstacle_range_m)};
  const GroundLidarRejectionConfig ground{
      .enabled = true,
      .ground_altitude_m = kGroundAltitudeM,
      .closer_range_tolerance_m = 0.5,
      .farther_range_tolerance_m = 1.5,
  };
  const GridBounds bounds{-5.0, -15.0, 0.5, 80, 60};
  const LidarProjectionPose pose = downwardPose();
  LaserScan2DView memory_scan = memoryScan(ranges);
  useSourceAlignedPose(memory_scan, pose);

  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats =
      memory.integrateScan(Pose2{pose.position, pose.yaw_rad}, memory_scan,
                           ObstacleMemoryConfig{}, nullptr, &ground);
  OccupancyGrid2D overlay{bounds};
  LidarScanView overlay_scan = overlayScan(ranges);
  useSourceAlignedPose(overlay_scan, pose);
  const CurrentLidarOverlayStats overlay_stats = overlayCurrentLidarHits(
      overlay, overlay_scan, pose, projectionConfig(), nullptr, &ground);

  EXPECT_EQ(memory_stats.ingestion_decisions.closer_obstacles_retained, 1U);
  EXPECT_EQ(memory.countRawCells().occupied_cells, 1U);
  ASSERT_EQ(memory.activeProvenance().size(), 1U);
  const LidarIngestionDecisionSnapshot& persisted_decision =
      memory.activeProvenance().begin()->second.occupancy_trigger.ingestion_decision;
  EXPECT_EQ(persisted_decision.action, LidarIngestionAction::kIntegrateFreeAndHit);
  EXPECT_EQ(persisted_decision.reason,
            LidarIngestionReason::kObstacleBeforeExpectedSurface);
  EXPECT_EQ(persisted_decision.expected_surface, LidarExpectedSurfaceKind::kGround);
  EXPECT_TRUE(std::isfinite(persisted_decision.expected_range_m));
  EXPECT_LT(persisted_decision.range_delta_m, -0.5);
  EXPECT_EQ(overlay_stats.ingestion_decisions.closer_obstacles_retained, 1U);
  EXPECT_EQ(overlay_stats.occupied_cells, 1U);
  ASSERT_EQ(overlay_stats.accepted_hits.size(), 1U);
  const CurrentLidarAcceptedHitProvenance& accepted_hit =
      overlay_stats.accepted_hits.front();
  EXPECT_TRUE(overlay.isOccupied(accepted_hit.cell));
  EXPECT_EQ(accepted_hit.ingestion_decision.action,
            LidarIngestionAction::kIntegrateFreeAndHit);
  EXPECT_EQ(accepted_hit.ingestion_decision.reason,
            LidarIngestionReason::kObstacleBeforeExpectedSurface);
  EXPECT_EQ(accepted_hit.ingestion_decision.expected_surface,
            LidarExpectedSurfaceKind::kGround);
  const std::string blocker_diagnostic =
      formatCurrentLidarAcceptedHitDiagnostic(overlay_stats, accepted_hit.cell);
  EXPECT_NE(blocker_diagnostic.find("action=integrate_free_and_hit"),
            std::string::npos);
  EXPECT_NE(blocker_diagnostic.find("reason=obstacle_before_expected_surface"),
            std::string::npos);
  EXPECT_NE(blocker_diagnostic.find("surface=ground"), std::string::npos);
  EXPECT_NE(blocker_diagnostic.find("measured_range="), std::string::npos);
  EXPECT_NE(blocker_diagnostic.find("expected_range="), std::string::npos);
  EXPECT_NE(blocker_diagnostic.find("delta="), std::string::npos);
  EXPECT_NE(blocker_diagnostic.find("endpoint=("), std::string::npos);
  EXPECT_NE(blocker_diagnostic.find("ray_origin=("), std::string::npos);
  EXPECT_NE(blocker_diagnostic.find("ray_dir=("), std::string::npos);
  EXPECT_NE(blocker_diagnostic.find("source_attitude=(valid=true"), std::string::npos);
  EXPECT_NE(blocker_diagnostic.find("applied_attitude=(applied=true"),
            std::string::npos);
}

TEST(GroundLidarIngestion, LowUncertainEndpointMutatesNeitherMemoryNorCurrentOverlay) {
  const LidarProjectionPose pose{Point2{2.0, 0.0}, 1.0, 0.0, 0.0, 0.0, true, true};
  const std::array<float, 1U> ranges{4.0F};
  const GroundLidarRejectionConfig ground{
      .enabled = true,
      .ground_altitude_m = kGroundAltitudeM,
      .closer_range_tolerance_m = 0.5,
      .farther_range_tolerance_m = 1.5,
  };
  const GridBounds bounds{-5.0, -5.0, 0.5, 40, 20};

  LaserScan2DView memory_scan = memoryScan(ranges);
  memory_scan.origin_altitude_m = pose.altitude_m;
  memory_scan.pitch_rad = pose.pitch_rad;
  useSourceAlignedPose(memory_scan, pose);
  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats =
      memory.integrateScan(Pose2{pose.position, pose.yaw_rad}, memory_scan,
                           ObstacleMemoryConfig{}, nullptr, &ground);

  LidarScanView overlay_scan = overlayScan(ranges);
  useSourceAlignedPose(overlay_scan, pose);
  OccupancyGrid2D overlay{bounds};
  const CurrentLidarOverlayStats overlay_stats = overlayCurrentLidarHits(
      overlay, overlay_scan, pose, projectionConfig(), nullptr, &ground);

  EXPECT_EQ(memory_stats.free_cells_updated, 0U);
  EXPECT_EQ(memory_stats.occupied_cells_updated, 0U);
  EXPECT_EQ(memory_stats.ingestion_decisions.ground_candidates_pending, 1U);
  EXPECT_EQ(memory.countRawCells().occupied_cells, 0U);
  EXPECT_EQ(memory.countRawCells().free_cells, 0U);
  EXPECT_EQ(overlay_stats.ingestion_decisions.ground_candidates_pending, 1U);
  EXPECT_EQ(overlay_stats.occupied_cells, 0U);
}

TEST(GroundLidarIngestion,
     ProjectionUncertainUnknownRequiresIndependentScansInBothPaths) {
  const GridBounds bounds{-5.0, -5.0, 0.5, 40, 20};
  ObstacleMemoryGrid memory{bounds};
  OccupancyGrid2D overlay{bounds};
  UncertainLidarHitTracker overlay_tracker;

  for (std::int64_t scan_index = 1; scan_index <= 3; ++scan_index) {
    const double viewpoint_shift_m = static_cast<double>(scan_index - 1) * 0.6;
    const LidarProjectionPose pose{
        Point2{2.0 + viewpoint_shift_m, 0.0}, 8.0, 0.0, 0.0, 0.0, true, true};
    const std::array<float, 1U> ranges{static_cast<float>(4.0 - viewpoint_shift_m)};
    LaserScan2DView memory_scan = memoryScan(ranges);
    memory_scan.origin_altitude_m = pose.altitude_m;
    memory_scan.pitch_rad = pose.pitch_rad;
    memory_scan.timing.first_beam_stamp_ns = scan_index * 100'000'000;
    memory_scan.timing.first_beam_stamp_valid = true;
    const ObstacleMemoryStats memory_stats = memory.integrateScan(
        Pose2{pose.position, pose.yaw_rad}, memory_scan, ObstacleMemoryConfig{});

    LidarScanView overlay_scan = overlayScan(ranges);
    overlay_scan.timing = memory_scan.timing;
    const CurrentLidarOverlayStats overlay_stats =
        overlayCurrentLidarHits(overlay, overlay_scan, pose, projectionConfig(),
                                nullptr, nullptr, &overlay_tracker);

    const bool confirmed = scan_index == 3;
    EXPECT_EQ(memory_stats.ingestion_decisions.projection_uncertain_pending,
              confirmed ? 0U : 1U);
    EXPECT_EQ(overlay_stats.ingestion_decisions.projection_uncertain_pending,
              confirmed ? 0U : 1U);
    EXPECT_EQ(memory_stats.ingestion_decisions.projection_uncertain_confirmed_obstacle,
              confirmed ? 1U : 0U);
    EXPECT_EQ(overlay_stats.ingestion_decisions.projection_uncertain_confirmed_obstacle,
              confirmed ? 1U : 0U);
    EXPECT_EQ(memory.countRawCells().occupied_cells, confirmed ? 1U : 0U);
    EXPECT_EQ(overlay_stats.occupied_cells, confirmed ? 1U : 0U);
    if (!confirmed) {
      EXPECT_EQ(memory_stats.free_cells_updated, 0U);
      EXPECT_EQ(memory.countRawCells().free_cells, 0U);
    }
  }
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
  EXPECT_EQ(memory_stats.ingestion_decisions.expected_ground_suppressed, 0U);
  EXPECT_EQ(memory_stats.ingestion_decisions.ambiguous_ground_suppressed, 1U);
  EXPECT_EQ(memory_stats.ingestion_decisions.non_ground_altitude_rejected, 0U);
  EXPECT_EQ(memory_stats.free_cells_updated, 0U);
  EXPECT_EQ(overlay_stats.altitude_rejected_beams, 1U);
  EXPECT_EQ(overlay_stats.ingestion_decisions.expected_ground_suppressed, 0U);
  EXPECT_EQ(overlay_stats.ingestion_decisions.ambiguous_ground_suppressed, 1U);
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
