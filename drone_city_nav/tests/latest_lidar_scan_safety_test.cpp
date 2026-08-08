#include "drone_city_nav/latest_lidar_scan_safety.hpp"

#include <gtest/gtest.h>

#include <array>
#include <numbers>

namespace drone_city_nav {
namespace {

TEST(LatestLidarScanSafetyTest, PreservesAHitInTheAcquisitionBodyFrame) {
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

  const LatestLidarSafetyScanBuildResult result =
      buildLatestLidarSafetyScan(LatestLidarSafetyScanBuildInput{
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

TEST(LatestLidarScanSafetyTest, UsesEveryBeamPoseInsteadOfPersistentMemory) {
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

  const LatestLidarSafetyScanBuildResult result =
      buildLatestLidarSafetyScan(LatestLidarSafetyScanBuildInput{
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

} // namespace
} // namespace drone_city_nav
