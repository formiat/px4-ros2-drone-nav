#include "drone_city_nav/current_lidar_overlay.hpp"
#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"
#include "drone_city_nav/obstacle_memory.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

struct IngestionCase {
  KnownPassageSolidPartKind kind{KnownPassageSolidPartKind::kUpper};
  LidarProjectionPose pose{};
  LidarProjectionConfig config{};
};

[[nodiscard]] std::size_t partCount(const KnownStaticLidarPartCounters& counters,
                                    const KnownPassageSolidPartKind kind) {
  switch (kind) {
    case KnownPassageSolidPartKind::kLeft:
      return counters.left;
    case KnownPassageSolidPartKind::kRight:
      return counters.right;
    case KnownPassageSolidPartKind::kLower:
      return counters.lower;
    case KnownPassageSolidPartKind::kUpper:
      return counters.upper;
  }
  return 0U;
}

[[nodiscard]] const char* partId(const KnownPassageSolidPartKind kind) {
  switch (kind) {
    case KnownPassageSolidPartKind::kLeft:
      return "left_mass";
    case KnownPassageSolidPartKind::kRight:
      return "right_mass";
    case KnownPassageSolidPartKind::kLower:
      return "lower_mass";
    case KnownPassageSolidPartKind::kUpper:
      return "upper_mass";
  }
  return "unknown_mass";
}

[[nodiscard]] KnownPassageSolidVolume
volumeAtRange(const LidarBeamProjection& projection,
              const KnownPassageSolidPartKind kind, const double expected_range_m) {
  const double horizontal_norm =
      std::hypot(projection.ray_direction_map.x, projection.ray_direction_map.y);
  const Point2 normal{projection.ray_direction_map.x / horizontal_norm,
                      projection.ray_direction_map.y / horizontal_norm};
  const Point2 lateral{-normal.y, normal.x};
  constexpr double kDepthM = 2.0;
  const double center_distance_m = expected_range_m * horizontal_norm + kDepthM / 2.0;
  const double entry_z_m =
      projection.ray_origin_map_m.z + expected_range_m * projection.ray_direction_map.z;
  return KnownPassageSolidVolume{
      .structure_id = "component_structure",
      .opening_id = "component_opening",
      .part_id = partId(kind),
      .part_kind = kind,
      .center = Point2{projection.ray_origin_map_m.x + normal.x * center_distance_m,
                       projection.ray_origin_map_m.y + normal.y * center_distance_m},
      .normal_xy = normal,
      .lateral_xy = lateral,
      .depth_m = kDepthM,
      .width_m = 4.0,
      .min_z_m = entry_z_m - 2.0,
      .max_z_m = entry_z_m + 2.0,
  };
}

[[nodiscard]] LaserScan2DView memoryScan(const std::array<float, 1U>& ranges,
                                         const IngestionCase& test_case) {
  LaserScan2DView scan{};
  scan.ranges = ranges;
  scan.angle_min_rad = 0.0;
  scan.angle_increment_rad = 0.1;
  scan.range_min_m = 0.1;
  scan.range_max_m = test_case.config.max_lidar_range_m;
  scan.origin_altitude_m = test_case.pose.altitude_m;
  scan.roll_rad = test_case.pose.roll_rad;
  scan.pitch_rad = test_case.pose.pitch_rad;
  scan.lidar_z_offset_m = test_case.config.lidar_z_offset_m;
  scan.min_projected_altitude_m = -100.0;
  scan.max_projected_altitude_m = 100.0;
  scan.altitude_valid = test_case.pose.altitude_valid;
  scan.attitude_valid = test_case.pose.attitude_valid;
  scan.compensate_attitude = test_case.config.compensate_attitude;
  scan.lidar_mount_roll_rad = test_case.config.lidar_mount_roll_rad;
  scan.lidar_mount_pitch_rad = test_case.config.lidar_mount_pitch_rad;
  scan.lidar_mount_yaw_rad = test_case.config.lidar_mount_yaw_rad;
  return scan;
}

[[nodiscard]] IngestionCase
makeCase(const KnownPassageSolidPartKind kind, const double yaw_rad,
         const double roll_rad, const double pitch_rad, const double mount_roll_rad,
         const double mount_pitch_rad, const double mount_yaw_rad) {
  IngestionCase test_case{};
  test_case.kind = kind;
  test_case.pose = LidarProjectionPose{Point2{0.0, 0.0}, 10.0, yaw_rad, roll_rad,
                                       pitch_rad,        true, true};
  test_case.config.max_lidar_range_m = 20.0;
  test_case.config.compensate_attitude = true;
  test_case.config.lidar_mount_roll_rad = mount_roll_rad;
  test_case.config.lidar_mount_pitch_rad = mount_pitch_rad;
  test_case.config.lidar_mount_yaw_rad = mount_yaw_rad;
  test_case.config.min_projected_altitude_m = -100.0;
  test_case.config.max_projected_altitude_m = 100.0;
  return test_case;
}

[[nodiscard]] std::vector<KnownPassageSolidVolume>
openingBoundaryVolumes(const LidarBeamProjection& projection) {
  KnownPassageSolidVolume lower = volumeAtRange(
      projection, KnownPassageSolidPartKind::kLower, projection.used_range_m - 1.0);
  lower.min_z_m = 0.0;
  lower.max_z_m = projection.endpoint_map_m.z - 0.005;
  lower.opening_center = projection.endpoint;
  lower.opening_depth_m = lower.depth_m;
  lower.opening_width_m = lower.width_m;
  lower.opening_min_z_m = lower.max_z_m;
  lower.opening_max_z_m = lower.opening_min_z_m + 7.0;
  KnownPassageSolidVolume upper = lower;
  upper.part_id = "upper_mass";
  upper.part_kind = KnownPassageSolidPartKind::kUpper;
  upper.min_z_m = lower.opening_max_z_m;
  upper.max_z_m = upper.min_z_m + 4.0;
  return {std::move(lower), std::move(upper)};
}

class KnownStaticLidarIngestionTest : public ::testing::TestWithParam<IngestionCase> {};

} // namespace

TEST_P(KnownStaticLidarIngestionTest,
       ProjectionClassifierMemoryAndOverlayAgreeForExpectedSolid) {
  const IngestionCase test_case = GetParam();
  constexpr float kMeasuredRangeM = 6.0F;
  const LidarBeamProjection projection = projectLidarBeam(
      test_case.pose, test_case.config, 0.1, 20.0, 0.0, 0.1, 0U, kMeasuredRangeM);
  ASSERT_EQ(projection.status, LidarBeamProjectionStatus::kAccepted);
  ASSERT_GT(std::hypot(projection.ray_direction_map.x, projection.ray_direction_map.y),
            0.5);
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(volumeAtRange(projection, test_case.kind, kMeasuredRangeM));
  const KnownStaticLidarHitClassifier classifier{std::move(volumes)};
  const std::array<float, 1U> ranges{kMeasuredRangeM};
  const GridBounds bounds{-20.0, -20.0, 0.5, 80, 80};

  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats = memory.integrateScan(
      Pose2{test_case.pose.position, test_case.pose.yaw_rad},
      memoryScan(ranges, test_case), ObstacleMemoryConfig{}, &classifier);

  OccupancyGrid2D overlay_grid{bounds};
  const CurrentLidarOverlayStats overlay_stats =
      overlayCurrentLidarHits(overlay_grid, LidarScanView{ranges, 0.1, 20.0, 0.0, 0.1},
                              test_case.pose, test_case.config, &classifier);

  const auto endpoint_cell = overlay_grid.worldToCell(projection.endpoint);
  ASSERT_TRUE(endpoint_cell.has_value());
  const GridIndex endpoint_grid_cell =
      endpoint_cell.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_FALSE(memory.rawGrid().isOccupied(endpoint_grid_cell));
  EXPECT_FALSE(overlay_grid.isOccupied(endpoint_grid_cell));
  EXPECT_EQ(memory_stats.free_cells_updated, 0U);
  EXPECT_EQ(memory_stats.known_static_lidar.expected_static_hits_ignored, 1U);
  EXPECT_TRUE(memory.activeProvenance().empty());
  EXPECT_EQ(overlay_stats.known_static_lidar.expected_static_hits_ignored, 1U);
  EXPECT_EQ(partCount(memory_stats.known_static_lidar.expected_static_by_part,
                      test_case.kind),
            1U);
  EXPECT_EQ(partCount(overlay_stats.known_static_lidar.expected_static_by_part,
                      test_case.kind),
            1U);
}

INSTANTIATE_TEST_SUITE_P(
    PartsAndAttitudes, KnownStaticLidarIngestionTest,
    ::testing::Values(
        makeCase(KnownPassageSolidPartKind::kUpper, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        makeCase(KnownPassageSolidPartKind::kLower, std::numbers::pi / 2.0, 0.2, 0.0,
                 0.1, 0.0, 0.0),
        makeCase(KnownPassageSolidPartKind::kLeft, 0.4, -0.1, -0.15, 0.05, 0.05, 0.0),
        makeCase(KnownPassageSolidPartKind::kRight, -0.7, 0.1, 0.1, -0.05, 0.0, 0.2)));

TEST(KnownStaticLidarIngestion, CloserObstacleIsRetainedByBothPaths) {
  const IngestionCase test_case =
      makeCase(KnownPassageSolidPartKind::kUpper, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0);
  const LidarBeamProjection expected_projection =
      projectLidarBeam(test_case.pose, test_case.config, 0.1, 20.0, 0.0, 0.1, 0U, 6.0F);
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(volumeAtRange(expected_projection, test_case.kind, 6.0));
  const KnownStaticLidarHitClassifier classifier{std::move(volumes)};
  const std::array<float, 1U> ranges{4.0F};
  const GridBounds bounds{-20.0, -20.0, 0.5, 80, 80};

  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats = memory.integrateScan(
      Pose2{test_case.pose.position, test_case.pose.yaw_rad},
      memoryScan(ranges, test_case), ObstacleMemoryConfig{}, &classifier);
  OccupancyGrid2D overlay_grid{bounds};
  const CurrentLidarOverlayStats overlay_stats =
      overlayCurrentLidarHits(overlay_grid, LidarScanView{ranges, 0.1, 20.0, 0.0, 0.1},
                              test_case.pose, test_case.config, &classifier);

  EXPECT_EQ(memory.countRawCells().occupied_cells, 1U);
  EXPECT_EQ(overlay_stats.occupied_cells, 1U);
  EXPECT_EQ(memory_stats.known_static_lidar.unexpected_hits_kept, 1U);
  EXPECT_EQ(overlay_stats.known_static_lidar.unexpected_hits_kept, 1U);
  ASSERT_EQ(memory_stats.retained_known_static_hits.size(), 1U);
  ASSERT_EQ(overlay_stats.retained_known_static_hits.size(), 1U);
  EXPECT_EQ(memory_stats.retained_known_static_hits.front().classification,
            KnownStaticLidarHitClassification::kUnexpected);
  EXPECT_EQ(overlay_stats.retained_known_static_hits.front().classification,
            KnownStaticLidarHitClassification::kUnexpected);
  EXPECT_NEAR(memory_stats.retained_known_static_hits.front().measured_range_m, 4.0,
              1.0e-6);
  EXPECT_NEAR(overlay_stats.retained_known_static_hits.front().measured_range_m, 4.0,
              1.0e-6);
  ASSERT_EQ(memory_stats.newly_occupied_cells, 1U);
  ASSERT_EQ(memory_stats.occupied_transitions.size(), 1U);
  const ObstacleMemoryOccupiedTransition& transition =
      memory_stats.occupied_transitions.front();
  const AcceptedObstacleMemoryHit& trigger = transition.provenance.occupancy_trigger;
  EXPECT_TRUE(trigger.known_static.classifier_applied);
  EXPECT_TRUE(trigger.known_static.volume_matched);
  EXPECT_TRUE(trigger.known_static.confident_face_interior);
  EXPECT_EQ(trigger.known_static.classification,
            KnownStaticLidarHitClassification::kUnexpected);
  EXPECT_EQ(trigger.known_static.part_id, "upper_mass");
  EXPECT_NEAR(trigger.beam.measured_range_m, 4.0, 1.0e-6);
  EXPECT_NEAR(trigger.known_static.expected_range_m, 6.0, 1.0e-6);
  EXPECT_NEAR(trigger.known_static.range_delta_m, -2.0, 1.0e-6);
}

TEST(KnownStaticLidarIngestion,
     EndpointNearSurfaceBeyondEffectiveRangeIsSuppressedByBothPaths) {
  IngestionCase test_case =
      makeCase(KnownPassageSolidPartKind::kUpper, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  test_case.config.max_lidar_range_m = 35.0;
  test_case.config.range_hit_epsilon_m = 0.05;
  constexpr float kMeasuredRangeM = 34.94F;
  constexpr double kKnownSurfaceRangeM = 35.1;
  const LidarBeamProjection projection = projectLidarBeam(
      test_case.pose, test_case.config, 0.1, 35.0, 0.0, 0.1, 0U, kMeasuredRangeM);
  ASSERT_TRUE(projection.hit);
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(volumeAtRange(projection, test_case.kind, kKnownSurfaceRangeM));
  const KnownStaticLidarHitClassifier classifier{std::move(volumes)};
  EXPECT_FALSE(classifier
                   .nearestExpectedSurface(projection.ray_origin_map_m,
                                           projection.ray_direction_map, 35.0)
                   .has_value());

  const std::array<float, 1U> ranges{kMeasuredRangeM};
  const GridBounds bounds{-5.0, -5.0, 0.5, 100, 40};
  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats = memory.integrateScan(
      Pose2{test_case.pose.position, test_case.pose.yaw_rad},
      memoryScan(ranges, test_case), ObstacleMemoryConfig{}, &classifier);
  OccupancyGrid2D overlay_grid{bounds};
  const CurrentLidarOverlayStats overlay_stats =
      overlayCurrentLidarHits(overlay_grid, LidarScanView{ranges, 0.1, 35.0, 0.0, 0.1},
                              test_case.pose, test_case.config, &classifier);

  const std::optional<GridIndex> endpoint_cell =
      overlay_grid.worldToCell(projection.endpoint);
  ASSERT_TRUE(endpoint_cell.has_value());
  const GridIndex accepted_cell =
      endpoint_cell.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_FALSE(memory.rawGrid().isOccupied(accepted_cell));
  EXPECT_FALSE(overlay_grid.isOccupied(accepted_cell));
  EXPECT_EQ(memory_stats.occupied_cells_updated, 0U);
  EXPECT_EQ(overlay_stats.occupied_cells, 0U);
  EXPECT_EQ(memory_stats.known_static_lidar.expected_static_hits_ignored, 1U);
  EXPECT_EQ(overlay_stats.known_static_lidar.expected_static_hits_ignored, 1U);
  EXPECT_TRUE(memory_stats.occupied_transitions.empty());
  EXPECT_TRUE(overlay_stats.accepted_hits.empty());
}

TEST(KnownStaticLidarIngestion,
     DistantKnownSurfaceDoesNotCorruptAcceptedObstacleMetadata) {
  IngestionCase test_case =
      makeCase(KnownPassageSolidPartKind::kUpper, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  test_case.config.max_lidar_range_m = 30.0;
  constexpr float kMeasuredRangeM = 29.0F;
  const LidarBeamProjection projection = projectLidarBeam(
      test_case.pose, test_case.config, 0.1, 30.0, 0.0, 0.1, 0U, kMeasuredRangeM);
  ASSERT_TRUE(projection.hit);
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(volumeAtRange(projection, test_case.kind, 212.0));
  const KnownStaticLidarHitClassifier classifier{std::move(volumes)};
  const std::array<float, 1U> ranges{kMeasuredRangeM};
  const GridBounds bounds{-5.0, -5.0, 0.5, 80, 40};

  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats = memory.integrateScan(
      Pose2{test_case.pose.position, test_case.pose.yaw_rad},
      memoryScan(ranges, test_case), ObstacleMemoryConfig{}, &classifier);
  OccupancyGrid2D overlay_grid{bounds};
  const CurrentLidarOverlayStats overlay_stats =
      overlayCurrentLidarHits(overlay_grid, LidarScanView{ranges, 0.1, 30.0, 0.0, 0.1},
                              test_case.pose, test_case.config, &classifier);

  ASSERT_EQ(memory_stats.occupied_transitions.size(), 1U);
  const LidarIngestionDecisionSnapshot& memory_decision =
      memory_stats.occupied_transitions.front()
          .provenance.occupancy_trigger.ingestion_decision;
  EXPECT_EQ(memory_decision.reason, LidarIngestionReason::kNoExpectedSurface);
  EXPECT_EQ(memory_decision.expected_surface, LidarExpectedSurfaceKind::kNone);
  EXPECT_TRUE(std::isnan(memory_decision.expected_range_m));
  EXPECT_TRUE(std::isnan(memory_decision.range_delta_m));
  EXPECT_FALSE(memory_stats.occupied_transitions.front()
                   .provenance.occupancy_trigger.known_static.classifier_applied);
  EXPECT_EQ(memory_stats.ingestion_decisions.invariant_fallbacks, 0U);

  ASSERT_EQ(overlay_stats.accepted_hits.size(), 1U);
  const LidarIngestionDecisionSnapshot& overlay_decision =
      overlay_stats.accepted_hits.front().ingestion_decision;
  EXPECT_EQ(overlay_decision.reason, LidarIngestionReason::kNoExpectedSurface);
  EXPECT_EQ(overlay_decision.expected_surface, LidarExpectedSurfaceKind::kNone);
  EXPECT_TRUE(std::isnan(overlay_decision.expected_range_m));
  EXPECT_TRUE(std::isnan(overlay_decision.range_delta_m));
  EXPECT_EQ(overlay_stats.ingestion_decisions.invariant_fallbacks, 0U);
}

TEST(KnownStaticLidarIngestion,
     InvalidGroundProviderPreservesExpectedKnownStaticSuppression) {
  const IngestionCase test_case =
      makeCase(KnownPassageSolidPartKind::kUpper, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  constexpr float kMeasuredRangeM = 6.0F;
  const LidarBeamProjection projection = projectLidarBeam(
      test_case.pose, test_case.config, 0.1, 20.0, 0.0, 0.1, 0U, kMeasuredRangeM);
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(volumeAtRange(projection, test_case.kind, kMeasuredRangeM));
  const KnownStaticLidarHitClassifier classifier{std::move(volumes)};
  const std::optional<KnownStaticExpectedSurface> nearest =
      classifier.nearestExpectedSurface(projection.ray_origin_map_m,
                                        projection.ray_direction_map, 20.0);
  ASSERT_TRUE(nearest.has_value());
  const KnownStaticExpectedSurface& expected_surface =
      *nearest; // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_NEAR(expected_surface.range_m, kMeasuredRangeM, 1.0e-6);
  GroundLidarRejectionConfig invalid_ground{};
  invalid_ground.farther_range_tolerance_m = -1.0;
  const std::array<float, 1U> ranges{kMeasuredRangeM};
  const GridBounds bounds{-20.0, -20.0, 0.5, 80, 80};

  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats =
      memory.integrateScan(Pose2{test_case.pose.position, test_case.pose.yaw_rad},
                           memoryScan(ranges, test_case), ObstacleMemoryConfig{},
                           &classifier, &invalid_ground);
  OccupancyGrid2D overlay_grid{bounds};
  const CurrentLidarOverlayStats overlay_stats = overlayCurrentLidarHits(
      overlay_grid, LidarScanView{ranges, 0.1, 20.0, 0.0, 0.1}, test_case.pose,
      test_case.config, &classifier, &invalid_ground);

  EXPECT_EQ(memory_stats.known_static_lidar.expected_static_hits_ignored, 1U);
  EXPECT_EQ(overlay_stats.known_static_lidar.expected_static_hits_ignored, 1U);
  EXPECT_EQ(memory_stats.ingestion_decisions.ground_classification_unavailable, 1U);
  EXPECT_EQ(overlay_stats.ingestion_decisions.ground_classification_unavailable, 1U);
  EXPECT_EQ(memory.countRawCells().occupied_cells, 0U);
  EXPECT_EQ(overlay_stats.occupied_cells, 0U);
}

TEST(KnownStaticLidarIngestion, FartherKnownSurfaceReturnIsSuppressedByBothPaths) {
  const IngestionCase test_case =
      makeCase(KnownPassageSolidPartKind::kUpper, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0);
  const LidarBeamProjection expected_projection =
      projectLidarBeam(test_case.pose, test_case.config, 0.1, 20.0, 0.0, 0.1, 0U, 6.0F);
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(volumeAtRange(expected_projection, test_case.kind, 6.0));
  const KnownStaticLidarHitClassifier classifier{
      std::move(volumes),
      KnownStaticLidarHitClassifierConfig{.closer_range_tolerance_m = 0.5,
                                          .farther_range_tolerance_m = 1.5}};
  const std::array<float, 1U> ranges{7.4F};
  const GridBounds bounds{-20.0, -20.0, 0.5, 80, 80};

  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats = memory.integrateScan(
      Pose2{test_case.pose.position, test_case.pose.yaw_rad},
      memoryScan(ranges, test_case), ObstacleMemoryConfig{}, &classifier);
  OccupancyGrid2D overlay_grid{bounds};
  const CurrentLidarOverlayStats overlay_stats =
      overlayCurrentLidarHits(overlay_grid, LidarScanView{ranges, 0.1, 20.0, 0.0, 0.1},
                              test_case.pose, test_case.config, &classifier);

  EXPECT_EQ(memory.countRawCells().occupied_cells, 0U);
  EXPECT_EQ(overlay_stats.occupied_cells, 0U);
  EXPECT_EQ(memory_stats.known_static_lidar.expected_static_hits_ignored, 1U);
  EXPECT_EQ(overlay_stats.known_static_lidar.expected_static_hits_ignored, 1U);
  EXPECT_TRUE(memory_stats.retained_known_static_hits.empty());
  EXPECT_TRUE(overlay_stats.retained_known_static_hits.empty());
  EXPECT_EQ(memory_stats.newly_occupied_cells, 0U);
  EXPECT_TRUE(memory_stats.occupied_transitions.empty());
}

TEST(KnownStaticLidarIngestion, BoundaryAmbiguitySuppressesAllUpdates) {
  const IngestionCase test_case =
      makeCase(KnownPassageSolidPartKind::kRight, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0);
  const LidarBeamProjection projection =
      projectLidarBeam(test_case.pose, test_case.config, 0.1, 20.0, 0.0, 0.1, 0U, 6.0F);
  KnownPassageSolidVolume volume = volumeAtRange(projection, test_case.kind, 6.0);
  volume.center.x += volume.lateral_xy.x * (volume.width_m / 2.0);
  volume.center.y += volume.lateral_xy.y * (volume.width_m / 2.0);
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(std::move(volume));
  const KnownStaticLidarHitClassifier classifier{std::move(volumes)};
  const std::array<float, 1U> ranges{6.0F};
  const GridBounds bounds{-20.0, -20.0, 0.5, 80, 80};

  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats = memory.integrateScan(
      Pose2{test_case.pose.position, test_case.pose.yaw_rad},
      memoryScan(ranges, test_case), ObstacleMemoryConfig{}, &classifier);
  OccupancyGrid2D overlay_grid{bounds};
  const CurrentLidarOverlayStats overlay_stats =
      overlayCurrentLidarHits(overlay_grid, LidarScanView{ranges, 0.1, 20.0, 0.0, 0.1},
                              test_case.pose, test_case.config, &classifier);

  EXPECT_EQ(memory.countRawCells().occupied_cells, 0U);
  EXPECT_EQ(overlay_stats.occupied_cells, 0U);
  EXPECT_EQ(memory_stats.free_cells_updated, 0U);
  EXPECT_EQ(memory_stats.ambiguous_hits_pending_confirmation, 1U);
  EXPECT_EQ(overlay_stats.ambiguous_hits_pending_confirmation, 1U);
}

TEST(KnownStaticLidarIngestion,
     StaticAttachedHitRequiresIndependentViewpointsAndNeverBecomesObstacle) {
  const IngestionCase test_case =
      makeCase(KnownPassageSolidPartKind::kUpper, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const LidarBeamProjection expected_projection =
      projectLidarBeam(test_case.pose, test_case.config, 0.1, 20.0, 0.0, 0.1, 0U, 6.0F);
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.push_back(volumeAtRange(expected_projection, test_case.kind, 6.0));
  const KnownStaticLidarHitClassifier classifier{std::move(volumes)};
  const GridBounds bounds{-20.0, -20.0, 0.5, 80, 80};
  ObstacleMemoryGrid memory{bounds};
  OccupancyGrid2D overlay_grid{bounds};
  AmbiguousLidarHitTracker overlay_tracker;

  for (std::int64_t scan_index = 1; scan_index <= 3; ++scan_index) {
    const double viewpoint_shift_m = static_cast<double>(scan_index - 1) * 0.6;
    IngestionCase shifted_case = test_case;
    shifted_case.pose.position.x -= viewpoint_shift_m;
    const std::array<float, 1U> ranges{static_cast<float>(5.34 + viewpoint_shift_m)};
    LaserScan2DView memory_scan = memoryScan(ranges, shifted_case);
    memory_scan.timing.first_beam_stamp_ns = scan_index * 100'000'000;
    memory_scan.timing.first_beam_stamp_valid = true;
    const ObstacleMemoryStats memory_stats = memory.integrateScan(
        Pose2{shifted_case.pose.position, shifted_case.pose.yaw_rad}, memory_scan,
        ObstacleMemoryConfig{}, &classifier);

    LidarScanView overlay_scan{ranges, 0.1, 20.0, 0.0, 0.1};
    overlay_scan.timing = memory_scan.timing;
    const CurrentLidarOverlayStats overlay_stats = overlayCurrentLidarHits(
        overlay_grid, overlay_scan, shifted_case.pose, shifted_case.config, &classifier,
        nullptr, &overlay_tracker);

    const bool expected_confirmed = scan_index == 3;
    EXPECT_EQ(memory.countRawCells().occupied_cells, 0U);
    EXPECT_EQ(overlay_stats.occupied_cells, 0U);
    EXPECT_EQ(memory_stats.free_cells_updated, 0U);
    EXPECT_EQ(memory_stats.ambiguous_hits_confirmed, expected_confirmed ? 1U : 0U);
    EXPECT_EQ(overlay_stats.ambiguous_hits_confirmed, expected_confirmed ? 1U : 0U);
  }
}

TEST(KnownStaticLidarIngestion, OpeningObstacleIsIntegratedImmediatelyByBothPaths) {
  const IngestionCase test_case =
      makeCase(KnownPassageSolidPartKind::kUpper, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  constexpr float kMeasuredRangeM = 6.0F;
  const LidarBeamProjection projection = projectLidarBeam(
      test_case.pose, test_case.config, 0.1, 20.0, 0.0, 0.1, 0U, kMeasuredRangeM);
  KnownPassageSolidVolume volume =
      volumeAtRange(projection, KnownPassageSolidPartKind::kUpper, 6.0);
  volume.min_z_m = projection.endpoint_map_m.z + 3.0;
  volume.max_z_m = projection.endpoint_map_m.z + 6.0;
  volume.opening_center = projection.endpoint;
  volume.opening_depth_m = volume.depth_m;
  volume.opening_width_m = volume.width_m;
  volume.opening_min_z_m = projection.endpoint_map_m.z - 1.0;
  volume.opening_max_z_m = projection.endpoint_map_m.z + 1.0;
  const KnownStaticLidarHitClassifier classifier{{std::move(volume)}};
  const std::array<float, 1U> ranges{kMeasuredRangeM};
  const GridBounds bounds{-20.0, -20.0, 0.5, 80, 80};

  ObstacleMemoryGrid memory{bounds};
  const ObstacleMemoryStats memory_stats = memory.integrateScan(
      Pose2{test_case.pose.position, test_case.pose.yaw_rad},
      memoryScan(ranges, test_case), ObstacleMemoryConfig{}, &classifier);
  OccupancyGrid2D overlay_grid{bounds};
  const CurrentLidarOverlayStats overlay_stats =
      overlayCurrentLidarHits(overlay_grid, LidarScanView{ranges, 0.1, 20.0, 0.0, 0.1},
                              test_case.pose, test_case.config, &classifier);

  EXPECT_EQ(memory.countRawCells().occupied_cells, 1U);
  EXPECT_EQ(overlay_stats.occupied_cells, 1U);
  EXPECT_EQ(memory_stats.ingestion_decisions.opening_interior_obstacles_integrated, 1U);
  EXPECT_EQ(overlay_stats.ingestion_decisions.opening_interior_obstacles_integrated,
            1U);
  ASSERT_EQ(memory_stats.occupied_transitions.size(), 1U);
  EXPECT_EQ(memory_stats.occupied_transitions.front().trigger_decision.reason,
            LidarIngestionReason::kObstacleInsideOpening);
}

TEST(KnownStaticLidarIngestion,
     OpeningBoundaryStaysNonMutatingAndConfirmsStaticInBothPaths) {
  const IngestionCase test_case =
      makeCase(KnownPassageSolidPartKind::kLower, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  constexpr float kInitialRangeM = 6.25F;
  const LidarBeamProjection projection = projectLidarBeam(
      test_case.pose, test_case.config, 0.1, 20.0, 0.0, 0.1, 0U, kInitialRangeM);
  const KnownStaticLidarHitClassifier classifier{openingBoundaryVolumes(projection)};
  const GridBounds bounds{-20.0, -20.0, 0.5, 80, 80};
  ObstacleMemoryGrid memory{bounds};
  OccupancyGrid2D overlay_grid{bounds};
  AmbiguousLidarHitTracker overlay_tracker;

  for (std::int64_t scan_index = 1; scan_index <= 3; ++scan_index) {
    const double viewpoint_shift_m = static_cast<double>(scan_index - 1) * 0.6;
    IngestionCase shifted_case = test_case;
    shifted_case.pose.position.x -= viewpoint_shift_m;
    const std::array<float, 1U> ranges{
        static_cast<float>(kInitialRangeM + viewpoint_shift_m)};
    LaserScan2DView memory_scan = memoryScan(ranges, shifted_case);
    memory_scan.timing.first_beam_stamp_ns = scan_index * 100'000'000;
    memory_scan.timing.first_beam_stamp_valid = true;
    const ObstacleMemoryStats memory_stats = memory.integrateScan(
        Pose2{shifted_case.pose.position, shifted_case.pose.yaw_rad}, memory_scan,
        ObstacleMemoryConfig{}, &classifier);

    LidarScanView overlay_scan{ranges, 0.1, 20.0, 0.0, 0.1};
    overlay_scan.timing = memory_scan.timing;
    const CurrentLidarOverlayStats overlay_stats = overlayCurrentLidarHits(
        overlay_grid, overlay_scan, shifted_case.pose, shifted_case.config, &classifier,
        nullptr, &overlay_tracker);

    EXPECT_EQ(memory.countRawCells().occupied_cells, 0U);
    EXPECT_EQ(memory_stats.free_cells_updated, 0U);
    EXPECT_EQ(overlay_stats.occupied_cells, 0U);
    if (scan_index < 3) {
      EXPECT_EQ(memory_stats.ingestion_decisions.opening_boundary_pending, 1U);
      EXPECT_EQ(overlay_stats.ingestion_decisions.opening_boundary_pending, 1U);
    } else {
      EXPECT_EQ(memory_stats.ingestion_decisions.opening_boundary_confirmed_static, 1U);
      EXPECT_EQ(overlay_stats.ingestion_decisions.opening_boundary_confirmed_static,
                1U);
      const std::string diagnostics = formatLidarIngestionRepresentativeDiagnostics(
          memory_stats.ingestion_decisions);
      EXPECT_NE(diagnostics.find("endpoint_relation=inside_opening_boundary"),
                std::string::npos);
      EXPECT_NE(diagnostics.find("opening_z=[9.995,16.995]"), std::string::npos);
      EXPECT_NE(diagnostics.find("boundary_tolerance=0.3"), std::string::npos);
      EXPECT_NE(diagnostics.find("part=lower_mass"), std::string::npos);
    }
  }
}

} // namespace drone_city_nav
