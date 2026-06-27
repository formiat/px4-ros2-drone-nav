#include "drone_city_nav/lidar_motion_compensation.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {

TEST(LidarMotionCompensation, ShiftsBackwardWhenLatencyExceedsPoseLag) {
  const LidarPoseMotionCompensationResult result = compensateLidarPoseForLatency(
      Point2{10.0, 5.0}, Point2{20.0, 0.0}, true, true, 0.01, 0.05);

  EXPECT_TRUE(result.applied);
  EXPECT_DOUBLE_EQ(result.signed_time_offset_s, -0.04);
  EXPECT_NEAR(result.position.x, 9.2, 1.0e-9);
  EXPECT_DOUBLE_EQ(result.position.y, 5.0);
  EXPECT_NEAR(result.applied_shift.x, -0.8, 1.0e-9);
  EXPECT_DOUBLE_EQ(result.applied_shift.y, 0.0);
  EXPECT_NEAR(result.applied_shift_m, 0.8, 1.0e-9);
}

TEST(LidarMotionCompensation, AccountsForPoseLagBeforeLatency) {
  const LidarPoseMotionCompensationResult result = compensateLidarPoseForLatency(
      Point2{10.0, 5.0}, Point2{20.0, 0.0}, true, true, 0.07, 0.05);

  EXPECT_TRUE(result.applied);
  EXPECT_NEAR(result.signed_time_offset_s, 0.02, 1.0e-12);
  EXPECT_NEAR(result.position.x, 10.4, 1.0e-9);
  EXPECT_DOUBLE_EQ(result.position.y, 5.0);
}

TEST(LidarMotionCompensation, DisabledCompensationKeepsPose) {
  const LidarPoseMotionCompensationResult result = compensateLidarPoseForLatency(
      Point2{10.0, 5.0}, Point2{20.0, 0.0}, false, true, 0.01, 0.05);

  EXPECT_FALSE(result.applied);
  EXPECT_DOUBLE_EQ(result.position.x, 10.0);
  EXPECT_DOUBLE_EQ(result.position.y, 5.0);
  EXPECT_DOUBLE_EQ(result.applied_shift_m, 0.0);
}

} // namespace drone_city_nav
