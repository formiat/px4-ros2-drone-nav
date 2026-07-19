#include "drone_city_nav/lidar_pose_history.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

constexpr double kPi{std::numbers::pi};

std::array<float, 4> pitchQuaternion(const double pitch_rad) {
  return {static_cast<float>(std::cos(pitch_rad / 2.0)), 0.0F,
          static_cast<float>(std::sin(pitch_rad / 2.0)), 0.0F};
}

TEST(LidarPoseHistoryTest, InterpolatesXyzYawAndQuaternionAttitude) {
  LidarPoseHistory history;
  history.addPosition(1'000'000'000, Point3{0.0, 10.0, 20.0}, 3.0, true);
  history.addPosition(2'000'000'000, Point3{10.0, 20.0, 30.0}, -3.0, true);
  history.addAttitude(1'000'000'000, pitchQuaternion(0.0));
  history.addAttitude(2'000'000'000, pitchQuaternion(kPi / 2.0));

  const auto sample = history.sample(1'500'000'000);

  ASSERT_TRUE(sample.has_value());
  const TimestampAlignedLidarPose& value =
      sample.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_NEAR(value.pose.position.x, 5.0, 1.0e-9);
  EXPECT_NEAR(value.pose.position.y, 15.0, 1.0e-9);
  EXPECT_NEAR(value.pose.altitude_m, 25.0, 1.0e-9);
  EXPECT_NEAR(std::abs(value.pose.yaw_rad), kPi, 1.0e-6);
  EXPECT_NEAR(value.pose.pitch_rad, kPi / 4.0, 1.0e-6);
  EXPECT_TRUE(value.position_interpolated);
  EXPECT_TRUE(value.attitude_interpolated);
}

TEST(LidarPoseHistoryTest, RejectsExtrapolationBeyondConfiguredBound) {
  LidarPoseHistory history{LidarPoseHistoryConfig{3'000'000'000, 50'000'000}};
  history.addPosition(1'000'000'000, Point3{1.0, 2.0, 3.0}, 0.0, true);
  history.addAttitude(1'000'000'000, pitchQuaternion(0.0));

  EXPECT_TRUE(history.sample(1'040'000'000).has_value());
  EXPECT_FALSE(history.sample(1'060'000'000).has_value());
}

TEST(LidarPoseHistoryTest, RejectsInvalidSamples) {
  LidarPoseHistory history;
  history.addPosition(1'000'000'000, Point3{0.0, 0.0, 1.0}, 0.0, false);
  history.addAttitude(1'000'000'000, {1.0F, 0.0F, 0.0F, 0.0F});

  EXPECT_FALSE(history.sample(1'000'000'000).has_value());
}

TEST(LidarPoseHistoryTest, BuildsPoseForEachBeamAcquisitionTime) {
  LidarPoseHistory history;
  history.addPosition(1'000'000'000, Point3{0.0, 0.0, 10.0}, 0.0, true);
  history.addPosition(1'200'000'000, Point3{2.0, 0.0, 12.0}, 0.2, true);
  history.addAttitude(1'000'000'000, pitchQuaternion(0.0));
  history.addAttitude(1'200'000'000, pitchQuaternion(0.2));
  const LaserScanTiming timing{1'000'000'000, true, 0.1, 0, false};

  const auto poses = timestampAlignedLidarBeamPoses(history, timing, 3U);

  ASSERT_TRUE(poses.has_value());
  const std::vector<LidarProjectionPose>& values =
      poses.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_EQ(values.size(), 3U);
  EXPECT_NEAR(values[1].position.x, 1.0, 1.0e-9);
  EXPECT_NEAR(values[1].altitude_m, 11.0, 1.0e-9);
  EXPECT_NEAR(values[2].pitch_rad, 0.2, 1.0e-6);
}

} // namespace
} // namespace drone_city_nav
