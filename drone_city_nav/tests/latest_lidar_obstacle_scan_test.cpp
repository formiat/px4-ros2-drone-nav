#include "drone_city_nav/latest_lidar_obstacle_scan.hpp"

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(LatestLidarObstacleScanTest, PreservesHitInAcquisitionBodyFrame) {
  LidarProjectionPose pose;
  pose.position = Point2{10.0, 20.0};
  pose.altitude_m = 5.0;
  pose.yaw_rad = std::numbers::pi / 4.0;
  pose.roll_rad = 0.1;
  pose.pitch_rad = -0.2;
  pose.altitude_valid = true;
  pose.attitude_valid = true;
  const std::array<float, 1> ranges{5.0F};
  const std::array<LidarProjectionPose, 1> poses{pose};
  const LidarProjectionConfig projection_config{
      .max_lidar_range_m = 35.0,
      .range_hit_epsilon_m = 0.05,
      .min_projected_altitude_m = -100.0,
      .max_projected_altitude_m = 100.0,
      .compensate_attitude = true,
  };

  const LatestLidarObstacleScanBuildResult result =
      buildLatestLidarObstacleScan(LatestLidarObstacleScanBuildInput{
          .ranges = ranges,
          .beam_projection_poses = poses,
          .projection_config = projection_config,
          .range_min_m = 0.1,
          .range_max_m = 35.0,
          .angle_min_rad = 0.0,
          .angle_increment_rad = 0.01,
      });
  const LidarBeamProjection expected = projectLidarBeam(
      pose, projection_config, 0.1, 35.0, 0.0, 0.01, 0U, ranges.front());

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.acquisition_body_frame.valid);
  ASSERT_EQ(result.hit_points_body_frd.size(), 1U);
  const Point3 reconstructed = lidarBodyPointToMap(result.acquisition_body_frame,
                                                   result.hit_points_body_frd.front());
  EXPECT_NEAR(reconstructed.x, expected.endpoint_map_m.x, 1.0e-9);
  EXPECT_NEAR(reconstructed.y, expected.endpoint_map_m.y, 1.0e-9);
  EXPECT_NEAR(reconstructed.z, expected.endpoint_map_m.z, 1.0e-9);
}

TEST(LatestLidarObstacleScanTest, PreservesPositiveHandedBodyFrameExactly) {
  LidarProjectionPose pose;
  pose.position = Point2{10.0, 20.0};
  pose.altitude_m = 5.0;
  pose.yaw_rad = 0.5 * std::numbers::pi;
  pose.altitude_valid = true;
  pose.attitude_valid = true;
  pose.body_to_ned_quaternion = {1.0, 0.0, 0.0, 0.0};
  pose.body_to_ned_quaternion_valid = true;
  const std::array<float, 1> ranges{5.0F};
  const std::array<LidarProjectionPose, 1> poses{pose};
  const LidarProjectionConfig projection_config{
      .min_projected_altitude_m = -100.0,
      .max_projected_altitude_m = 100.0,
      .px4_to_map_m00 = 0.0,
      .px4_to_map_m01 = 1.0,
      .px4_to_map_m10 = 1.0,
      .px4_to_map_m11 = 0.0,
  };

  const LatestLidarObstacleScanBuildResult result =
      buildLatestLidarObstacleScan(LatestLidarObstacleScanBuildInput{
          .ranges = ranges,
          .beam_projection_poses = poses,
          .projection_config = projection_config,
          .range_min_m = 0.1,
          .range_max_m = 35.0,
          .angle_min_rad = 0.2,
          .angle_increment_rad = 0.01,
      });

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.acquisition_body_frame.valid);
  EXPECT_NEAR(result.acquisition_body_frame.x_axis_map.y, 1.0, 1.0e-9);
  EXPECT_NEAR(result.acquisition_body_frame.y_axis_map.x, 1.0, 1.0e-9);
  EXPECT_NEAR(result.acquisition_body_frame.z_axis_map.z, -1.0, 1.0e-9);
  ASSERT_EQ(result.hit_points_body_frd.size(), 1U);
  const Point3 reconstructed = lidarBodyPointToMap(result.acquisition_body_frame,
                                                   result.hit_points_body_frd.front());
  const LidarBeamProjection expected = projectLidarBeam(
      pose, projection_config, 0.1, 35.0, 0.2, 0.01, 0U, ranges.front());
  EXPECT_NEAR(reconstructed.x, expected.endpoint_map_m.x, 1.0e-9);
  EXPECT_NEAR(reconstructed.y, expected.endpoint_map_m.y, 1.0e-9);
  EXPECT_NEAR(reconstructed.z, expected.endpoint_map_m.z, 1.0e-9);
}

TEST(LatestLidarObstacleScanTest, UsesTimestampAlignedPoseForEveryBeam) {
  LidarProjectionPose first_pose;
  first_pose.position = Point2{10.0, 10.0};
  first_pose.altitude_m = 5.0;
  first_pose.altitude_valid = true;
  first_pose.attitude_valid = true;
  LidarProjectionPose second_pose = first_pose;
  second_pose.position.x = 11.0;
  const std::array<float, 2> ranges{5.0F, 5.0F};
  const std::array<LidarProjectionPose, 2> poses{first_pose, second_pose};
  const LidarProjectionConfig config{.min_projected_altitude_m = -100.0,
                                     .max_projected_altitude_m = 100.0};

  const LatestLidarObstacleScanBuildResult result =
      buildLatestLidarObstacleScan(LatestLidarObstacleScanBuildInput{
          .ranges = ranges,
          .beam_projection_poses = poses,
          .projection_config = config,
          .range_min_m = 0.1,
          .range_max_m = 35.0,
          .angle_min_rad = 0.0,
          .angle_increment_rad = 0.01,
      });

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.hit_points_body_frd.size(), 2U);
  EXPECT_NEAR(result.hit_points_body_frd[0].x, 5.0, 1.0e-9);
  EXPECT_GT(result.hit_points_body_frd[1].x, 5.9);
}

TEST(LatestLidarObstacleScanTest, RejectsScanWhenEveryBeamIsInvalid) {
  const std::vector<float> ranges{
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
  };
  LidarProjectionPose pose;
  pose.altitude_m = 5.0;
  pose.altitude_valid = true;
  pose.attitude_valid = true;
  const std::vector<LidarProjectionPose> poses(2U, pose);

  const LatestLidarObstacleScanBuildResult result =
      buildLatestLidarObstacleScan(LatestLidarObstacleScanBuildInput{
          .ranges = ranges,
          .beam_projection_poses = poses,
          .range_min_m = 0.2,
          .range_max_m = 20.0,
          .angle_min_rad = 0.0,
          .angle_increment_rad = 0.1,
      });

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.invalid_beam_count, ranges.size());
  EXPECT_TRUE(result.hit_points_body_frd.empty());
}

TEST(LatestLidarObstacleScanTest, CreatesStableNonzeroProducerEpochValues) {
  const std::uint64_t first = createLatestLidarObstacleProducerInstanceId();
  const std::uint64_t second = createLatestLidarObstacleProducerInstanceId();

  EXPECT_NE(first, 0U);
  EXPECT_NE(second, 0U);
  EXPECT_NE(first, second);
}

} // namespace
} // namespace drone_city_nav
