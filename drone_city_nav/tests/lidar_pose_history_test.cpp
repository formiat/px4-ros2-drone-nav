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
  const LidarPoseSampleResult result = history.sampleWithDiagnostics(1'060'000'000);
  EXPECT_EQ(result.status, LidarPoseAlignmentStatus::kExtrapolationExceeded);
  EXPECT_EQ(result.position_stamp_error_ns, 60'000'000);
  EXPECT_EQ(result.attitude_stamp_error_ns, 60'000'000);
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

TEST(LidarPoseHistoryTest, DiagnosesClockDomainMismatchAtFirstBeam) {
  LidarPoseHistory history;
  history.addPosition(1'700'000'000'000'000'000, Point3{0.0, 0.0, 10.0}, 0.0, true);
  history.addAttitude(1'700'000'000'000'000'000, pitchQuaternion(0.0));

  const LidarBeamPoseAlignmentResult result =
      timestampAlignedLidarBeamPosesWithDiagnostics(
          history, LaserScanTiming{18'000'000'000, true, 0.0, 0, false}, 720U);

  EXPECT_EQ(result.status, LidarPoseAlignmentStatus::kExtrapolationExceeded);
  EXPECT_EQ(result.failed_beam_index, 0U);
  EXPECT_EQ(result.requested_stamp_ns, 18'000'000'000);
  EXPECT_GT(result.position_stamp_error_ns, 1'000'000'000'000'000'000);
  EXPECT_EQ(result.position_sample_count, 1U);
  EXPECT_EQ(result.attitude_sample_count, 1U);
}

TEST(LidarPoseHistoryTest, DiagnosesMissingPoseHistory) {
  LidarPoseHistory history;
  const LidarBeamPoseAlignmentResult result =
      timestampAlignedLidarBeamPosesWithDiagnostics(
          history, LaserScanTiming{1'000'000'000, true, 0.0, 0, false}, 1U);

  EXPECT_EQ(result.status, LidarPoseAlignmentStatus::kPositionHistoryEmpty);
  EXPECT_STREQ(lidarPoseAlignmentStatusName(result.status), "position_history_empty");
}

} // namespace
} // namespace drone_city_nav
