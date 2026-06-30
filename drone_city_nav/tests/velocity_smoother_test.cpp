#include "drone_city_nav/velocity_smoother.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] VelocityFollowerConfig testConfig() {
  VelocityFollowerConfig config{};
  config.max_accel_mps2 = 3.0;
  config.max_decel_mps2 = 20.0;
  config.max_lateral_accel_mps2 = 3.0;
  config.velocity_lateral_response_accel_mps2 = 5.0;
  config.max_velocity_jerk_mps3 = 12.0;
  config.max_lateral_velocity_jerk_mps3 = 14.0;
  return config;
}

} // namespace

TEST(VelocitySmoother, AccelerationLimitClampsSpeedIncrease) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 3.0;
  config.max_lateral_accel_mps2 = 3.0;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{10.0, 0.0},
                            .previous_velocity_setpoint = Point2{0.0, 0.0},
                            .previous_velocity_setpoint_valid = true,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.velocity_xy.x, 0.3, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_delta_mps, 0.3, 1.0e-9);
}

TEST(VelocitySmoother, DecelerationLimitClampsSpeedDecrease) {
  VelocityFollowerConfig config = testConfig();
  config.max_decel_mps2 = 20.0;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{0.0, 0.0},
                            .previous_velocity_setpoint = Point2{10.0, 0.0},
                            .previous_velocity_setpoint_valid = true,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.velocity_xy.x, 8.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_delta_mps, 2.0, 1.0e-9);
}

TEST(VelocitySmoother, JerkLimitSmoothsLateralDirectionChange) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_lateral_accel_mps2 = 100.0;
  config.velocity_lateral_response_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.max_velocity_jerk_mps3 = 1.0;
  config.max_lateral_velocity_jerk_mps3 = 1.0;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{0.0, 12.0},
                            .previous_velocity_setpoint = Point2{12.0, 0.0},
                            .previous_velocity_acceleration_setpoint = Point2{},
                            .previous_velocity_setpoint_valid = true,
                            .previous_velocity_acceleration_setpoint_valid = true,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_LE(std::abs(plan.velocity_setpoint_acceleration_xy.y), 0.1 + 1.0e-9);
  EXPECT_LE(std::abs(plan.velocity_xy.y), 0.01 + 1.0e-9);
  EXPECT_LT(plan.velocity_xy.x, 12.0);
}

TEST(VelocitySmoother, JerkLimitSmoothsLongitudinalBraking) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_lateral_accel_mps2 = 100.0;
  config.max_decel_mps2 = 20.0;
  config.max_velocity_jerk_mps3 = 1.0;
  config.max_lateral_velocity_jerk_mps3 = 1.0;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{8.0, 0.0},
                            .previous_velocity_setpoint = Point2{12.0, 0.0},
                            .previous_velocity_acceleration_setpoint = Point2{},
                            .previous_velocity_setpoint_valid = true,
                            .previous_velocity_acceleration_setpoint_valid = true,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.velocity_xy.x, 11.99, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_acceleration_xy.x, -0.1, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_acceleration_xy.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_jerk_mps3, 1.0, 1.0e-9);
}

TEST(VelocitySmoother, LateralResponseAccelIsSeparateFromSpeedProfileLateralAccel) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.max_lateral_accel_mps2 = 3.0;
  config.velocity_lateral_response_accel_mps2 = 8.0;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{10.0, 10.0},
                            .previous_velocity_setpoint = Point2{10.0, 0.0},
                            .previous_velocity_setpoint_valid = true,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.velocity_xy.x, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.8, 1.0e-9);
}

TEST(VelocitySmoother, PathFrameLateralSmoothingLimitsNormalComponentOnly) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.velocity_lateral_response_accel_mps2 = 2.0;
  config.lateral_smoothing_min_speed_mps = 100.0;
  config.lateral_smoothing_full_speed_mps = 100.0;
  config.max_velocity_heading_rate_rad_s = 100.0;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{12.0, 10.0},
                            .path_tangent = Point2{1.0, 0.0},
                            .previous_velocity_setpoint = Point2{12.0, 0.0},
                            .previous_velocity_setpoint_valid = true,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.path_frame_lateral_smoothing_applied);
  EXPECT_FALSE(plan.lateral_zero_crossing_limited);
  EXPECT_NEAR(plan.velocity_xy.x, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.2, 1.0e-9);
  EXPECT_NEAR(plan.smoother_lateral_response_accel_mps2, 2.0, 1.0e-9);
}

TEST(VelocitySmoother, LateralZeroCrossingReachesZeroBeforeChangingSign) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.velocity_lateral_response_accel_mps2 = 5.0;
  config.lateral_smoothing_min_speed_mps = 100.0;
  config.lateral_smoothing_full_speed_mps = 100.0;
  config.max_velocity_heading_rate_rad_s = 100.0;
  config.lateral_zero_crossing_max_cross_track_m = 1.0;
  config.lateral_zero_crossing_max_growth_m = 0.1;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{12.0, -5.0},
                            .path_tangent = Point2{1.0, 0.0},
                            .previous_velocity_setpoint = Point2{12.0, 0.1},
                            .previous_velocity_setpoint_valid = true,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.path_frame_lateral_smoothing_applied);
  EXPECT_TRUE(plan.lateral_zero_crossing_limited);
  EXPECT_NEAR(plan.velocity_xy.x, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.0, 1.0e-9);
}

TEST(VelocitySmoother, LateralZeroCrossingDoesNotBlockGrowingCrossTrack) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.velocity_lateral_response_accel_mps2 = 5.0;
  config.lateral_smoothing_min_speed_mps = 100.0;
  config.lateral_smoothing_full_speed_mps = 100.0;
  config.max_velocity_heading_rate_rad_s = 100.0;
  config.lateral_zero_crossing_max_cross_track_m = 1.0;
  config.lateral_zero_crossing_max_growth_m = 0.1;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{12.0, -5.0},
                            .path_tangent = Point2{1.0, 0.0},
                            .previous_velocity_setpoint = Point2{12.0, 0.1},
                            .previous_velocity_setpoint_valid = true,
                            .dt_s = 0.1,
                            .current_cross_track_error_m = 0.5,
                            .predicted_cross_track_error_m = 2.0},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.path_frame_lateral_smoothing_applied);
  EXPECT_FALSE(plan.lateral_zero_crossing_limited);
  EXPECT_NEAR(plan.velocity_xy.x, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, -0.4, 1.0e-9);
}

TEST(VelocitySmoother, SpeedAwareLateralSmoothingReducesHighSpeedNormalDelta) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.velocity_lateral_response_accel_mps2 = 10.0;
  config.lateral_smoothing_min_speed_mps = 0.0;
  config.lateral_smoothing_full_speed_mps = 20.0;
  config.lateral_smoothing_max_factor = 2.0;
  config.max_velocity_heading_rate_rad_s = 0.0;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{20.0, 10.0},
                            .path_tangent = Point2{1.0, 0.0},
                            .previous_velocity_setpoint = Point2{20.0, 0.0},
                            .previous_velocity_setpoint_valid = true,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.path_frame_lateral_smoothing_applied);
  EXPECT_NEAR(plan.lateral_smoothing_factor, 2.0, 1.0e-9);
  EXPECT_NEAR(plan.smoother_lateral_response_accel_mps2, 5.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.x, 20.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.5, 1.0e-9);
}

TEST(VelocitySmoother, VelocityHeadingRateLimitClampsCommandDirectionChange) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.velocity_lateral_response_accel_mps2 = 100.0;
  config.lateral_smoothing_min_speed_mps = 100.0;
  config.lateral_smoothing_full_speed_mps = 100.0;
  config.max_velocity_heading_rate_rad_s = 0.1;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{0.0, 10.0},
                            .path_tangent = Point2{1.0, 0.0},
                            .previous_velocity_setpoint = Point2{10.0, 0.0},
                            .previous_velocity_setpoint_valid = true,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.path_frame_lateral_smoothing_applied);
  EXPECT_TRUE(plan.velocity_heading_rate_limited);
  EXPECT_LE(std::abs(std::atan2(plan.velocity_xy.y, plan.velocity_xy.x)),
            0.01 + 1.0e-9);
}

TEST(VelocitySmoother, VelocityHeadingRateUsesConfiguredMinimum) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.velocity_lateral_response_accel_mps2 = 4.0;
  config.lateral_smoothing_min_speed_mps = 100.0;
  config.lateral_smoothing_full_speed_mps = 100.0;
  config.max_velocity_heading_rate_rad_s = 1.0;
  config.min_velocity_heading_rate_rad_s = 0.8;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{0.0, 20.0},
                            .path_tangent = Point2{0.0, 1.0},
                            .previous_velocity_setpoint = Point2{20.0, 0.0},
                            .previous_velocity_setpoint_valid = true,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.path_frame_lateral_smoothing_applied);
  EXPECT_TRUE(plan.velocity_heading_rate_limited);
  EXPECT_NEAR(plan.velocity_heading_rate_limit_rad_s, 0.8, 1.0e-9);
  EXPECT_LE(std::abs(std::atan2(plan.velocity_xy.y, plan.velocity_xy.x)),
            0.08 + 1.0e-9);
}

TEST(VelocitySmoother, LateralJerkCanBeHigherThanLongitudinalJerk) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.velocity_lateral_response_accel_mps2 = 100.0;
  config.max_velocity_jerk_mps3 = 1.0;
  config.max_lateral_velocity_jerk_mps3 = 10.0;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{12.0, 12.0},
                            .previous_velocity_setpoint = Point2{12.0, 0.0},
                            .previous_velocity_acceleration_setpoint = Point2{},
                            .previous_velocity_setpoint_valid = true,
                            .previous_velocity_acceleration_setpoint_valid = true,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.velocity_xy.x, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.1, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_acceleration_xy.x, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_acceleration_xy.y, 1.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_jerk_mps3, 10.0, 1.0e-9);
}

TEST(VelocitySmoother, AdaptiveResponseDoesNotRaiseLateralJerkLimit) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.velocity_lateral_response_accel_mps2 = 100.0;
  config.max_velocity_jerk_mps3 = 1.0;
  config.max_lateral_velocity_jerk_mps3 = 10.0;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{12.0, 12.0},
                            .previous_velocity_setpoint = Point2{12.0, 0.0},
                            .previous_velocity_acceleration_setpoint = Point2{},
                            .previous_velocity_setpoint_valid = true,
                            .previous_velocity_acceleration_setpoint_valid = true,
                            .dt_s = 0.1,
                            .lateral_response_factor = 2.5},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.velocity_xy.x, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.1, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_acceleration_xy.y, 1.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_jerk_mps3, 10.0, 1.0e-9);
}

TEST(VelocitySmoother, ResetStateDoesNotPullNewDesiredVelocityTowardOldState) {
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_lateral_accel_mps2 = 100.0;

  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy = Point2{0.0, 8.0},
                            .previous_velocity_setpoint = Point2{12.0, 0.0},
                            .previous_velocity_acceleration_setpoint = Point2{5.0, 0.0},
                            .previous_velocity_setpoint_valid = false,
                            .previous_velocity_acceleration_setpoint_valid = false,
                            .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.velocity_xy.x, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 8.0, 1.0e-9);
}

TEST(VelocitySmoother, NonFiniteDesiredVelocityReturnsInvalidPlan) {
  const VelocitySmootherPlan plan = smoothVelocityCommand(
      VelocitySmootherInput{.desired_velocity_xy =
                                Point2{std::numeric_limits<double>::quiet_NaN(), 0.0},
                            .dt_s = 0.1},
      testConfig());

  EXPECT_FALSE(plan.valid);
}

} // namespace drone_city_nav
