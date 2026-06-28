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
