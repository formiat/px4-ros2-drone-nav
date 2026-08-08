#include "drone_city_nav/lidar_acquisition_pose.hpp"
#include "drone_city_nav/obstacle_memory.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::array<float, 4> pitchQuaternion(const double pitch_rad) {
  return {static_cast<float>(std::cos(pitch_rad / 2.0)), 0.0F,
          static_cast<float>(std::sin(pitch_rad / 2.0)), 0.0F};
}

[[nodiscard]] Px4RosTimeMapper makeReadyIdentityTimeMapper() {
  Px4RosTimeMapper mapper{Px4RosTimeMapperConfig{2U, 8U, 0.999, 1.001, 50'000'000}};
  mapper.observeTimesync(1'000'000U, 0, 100U, 1'010'000'000);
  mapper.observeTimesync(1'100'000U, 0, 100U, 1'110'000'000);
  return mapper;
}

[[nodiscard]] LidarPoseHistory makeMovingPoseHistory() {
  LidarPoseHistory history;
  history.addPosition(1'020'000'000, Point3{9.0, 0.0, 10.0}, 0.0, true, 1'000'000'000,
                      1'000'000'000);
  history.addPosition(1'120'000'000, Point3{10.0, 0.0, 10.0}, 0.0, true, 1'100'000'000,
                      1'100'000'000);
  history.addAttitude(1'020'000'000, pitchQuaternion(0.0), 1'000'000'000,
                      1'000'000'000);
  history.addAttitude(1'120'000'000, pitchQuaternion(0.4), 1'100'000'000,
                      1'100'000'000);
  return history;
}

TEST(LidarAcquisitionPoseTest, AppliesOffsetBeforeSamplingPositionAndAttitude) {
  const Px4RosTimeMapper mapper = makeReadyIdentityTimeMapper();
  const LidarPoseHistory history = makeMovingPoseHistory();
  const LaserScanTiming timing{1'010'000'000, true, 0.0, 1'020'000'000, true};

  const LidarAcquisitionPoseResult result = resolveLidarAcquisitionBeamPoses(
      history, timing, 1U,
      LidarAcquisitionPoseConfig{.apply_sensor_time_offset = true,
                                 .sensor_time_offset_s = 0.1,
                                 .require_source_timestamp_alignment = true},
      std::nullopt, &mapper);

  ASSERT_TRUE(result.resolved());
  ASSERT_EQ(result.alignment.poses.size(), 1U);
  EXPECT_TRUE(result.alignment.sourceAligned());
  EXPECT_EQ(result.adjusted_timing.first_beam_stamp_ns, 1'110'000'000);
  EXPECT_NEAR(result.alignment.poses.front().position.x, 10.0, 1.0e-9);
  EXPECT_NEAR(result.alignment.poses.front().pitch_rad, 0.4, 1.0e-6);
}

TEST(LidarAcquisitionPoseTest, OffsetChangesIntegratedPhysicalWallLocation) {
  const Px4RosTimeMapper mapper = makeReadyIdentityTimeMapper();
  const LidarPoseHistory history = makeMovingPoseHistory();
  const LaserScanTiming timing{1'010'000'000, true, 0.0, 1'020'000'000, true};
  const LidarAcquisitionPoseResult acquisition = resolveLidarAcquisitionBeamPoses(
      history, timing, 1U,
      LidarAcquisitionPoseConfig{.apply_sensor_time_offset = true,
                                 .sensor_time_offset_s = 0.1,
                                 .require_source_timestamp_alignment = true},
      std::nullopt, &mapper);
  ASSERT_TRUE(acquisition.resolved());

  ObstacleMemoryGrid memory{GridBounds{0.0, -1.0, 0.5, 40, 4}};
  const std::vector<float> ranges{5.0F};
  LaserScan2DView scan{};
  scan.ranges = ranges;
  scan.angle_increment_rad = 1.0;
  scan.range_min_m = 0.1;
  scan.range_max_m = 20.0;
  scan.origin_altitude_m = 10.0;
  scan.altitude_valid = true;
  scan.attitude_valid = true;
  scan.timing = timing;
  scan.beam_projection_poses = acquisition.alignment.poses;
  scan.projection_pose_source = LidarProjectionPoseSource::kSourceTimestampAligned;
  const LidarProjectionPose& pose = acquisition.alignment.poses.front();

  const ObstacleMemoryStats stats = memory.integrateScan(
      Pose2{pose.position, pose.yaw_rad}, scan, ObstacleMemoryConfig{});

  EXPECT_EQ(stats.hit_beams, 1U);
  const auto lagged_wall_cell = memory.rawGrid().worldToCell(Point2{14.0, 0.0});
  ASSERT_TRUE(lagged_wall_cell.has_value());
  EXPECT_EQ(memory.rawGrid().state(lagged_wall_cell.value_or(GridIndex{})),
            CellState::kFree);
  std::optional<Point2> occupied_center;
  for (int y = 0; y < memory.rawGrid().height(); ++y) {
    for (int x = 0; x < memory.rawGrid().width(); ++x) {
      const GridIndex cell{x, y};
      if (memory.rawGrid().isOccupied(cell)) {
        occupied_center = memory.rawGrid().cellCenter(cell);
      }
    }
  }
  ASSERT_TRUE(occupied_center.has_value());
  EXPECT_NEAR(occupied_center.value_or(Point2{}).x, 15.0, 0.26);
}

TEST(LidarAcquisitionPoseTest, RejectsReceiveTimeFallbackForMapping) {
  LidarPoseHistory history;
  history.addPosition(1'000'000'000, Point3{1.0, 2.0, 3.0}, 0.0, true);
  history.addAttitude(1'000'000'000, pitchQuaternion(0.0));

  const LidarAcquisitionPoseResult result = resolveLidarAcquisitionBeamPoses(
      history, LaserScanTiming{1'000'000'000, true, 0.0, 1'010'000'000, true}, 1U,
      LidarAcquisitionPoseConfig{.apply_sensor_time_offset = false,
                                 .sensor_time_offset_s = 0.0,
                                 .require_source_timestamp_alignment = true});

  EXPECT_FALSE(result.resolved());
  EXPECT_EQ(result.status,
            LidarAcquisitionPoseStatus::kSourceTimestampAlignmentRequired);
}

TEST(LidarAcquisitionPoseTest, WaitsForPositionAndAttitudeToBracketAdjustedScan) {
  const Px4RosTimeMapper mapper = makeReadyIdentityTimeMapper();
  LidarPoseHistory history;
  history.addPosition(1'020'000'000, Point3{9.0, 0.0, 10.0}, 0.0, true, 1'000'000'000,
                      1'000'000'000);
  history.addAttitude(1'020'000'000, pitchQuaternion(0.0), 1'000'000'000,
                      1'000'000'000);
  const LaserScanTiming timing{1'010'000'000, true, 0.0, 1'020'000'000, true};
  const LidarAcquisitionPoseConfig config{.apply_sensor_time_offset = true,
                                          .sensor_time_offset_s = 0.1,
                                          .require_source_timestamp_alignment = true,
                                          .require_bracketed_pose = true};

  const LidarAcquisitionPoseResult waiting = resolveLidarAcquisitionBeamPoses(
      history, timing, 1U, config, std::nullopt, &mapper);
  EXPECT_FALSE(waiting.resolved());
  EXPECT_EQ(waiting.status, LidarAcquisitionPoseStatus::kTemporalBindingUnreliable);
  EXPECT_GT(waiting.alignment.maximum_position_extrapolation_ns, 0);
  EXPECT_GT(waiting.alignment.maximum_attitude_extrapolation_ns, 0);

  history.addPosition(1'120'000'000, Point3{10.0, 0.0, 10.0}, 0.0, true, 1'100'000'000,
                      1'100'000'000);
  history.addAttitude(1'120'000'000, pitchQuaternion(0.4), 1'100'000'000,
                      1'100'000'000);
  const LidarAcquisitionPoseResult resolved = resolveLidarAcquisitionBeamPoses(
      history, timing, 1U, config, std::nullopt, &mapper);
  ASSERT_TRUE(resolved.resolved());
  EXPECT_TRUE(resolved.alignment.bracketed());
  EXPECT_NEAR(resolved.alignment.poses.front().position.x, 10.0, 1.0e-9);
  EXPECT_NEAR(resolved.alignment.poses.front().pitch_rad, 0.4, 1.0e-6);
}

TEST(LidarAcquisitionPoseTest, RejectsSourceTimestampFromWrongClockDomain) {
  const Px4RosTimeMapper mapper = makeReadyIdentityTimeMapper();

  const LidarPoseSourceStampResult result =
      resolveLidarPoseSourceStamp(mapper, 1'700'000'000'000'000U, 1'120'000'000);

  EXPECT_FALSE(result.resolved());
  EXPECT_EQ(result.status, LidarPoseSourceStampStatus::kReceiveTimeMismatch);
}

} // namespace
} // namespace drone_city_nav
