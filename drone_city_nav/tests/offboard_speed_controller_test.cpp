#include "drone_city_nav/offboard_speed_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] SpeedControllerConfig testConfig() {
  SpeedControllerConfig config{};
  config.desired_speed_mps = 5.0;
  config.max_accel_mps2 = 10.0;
  config.goal_slowdown_radius_m = 10.0;
  config.braking_safety_margin_m = 1.0;
  config.turn_slowdown_angle_rad = 0.5;
  config.turn_slowdown_min_speed_mps = 2.0;
  config.max_commanded_target_step_m = 2.0;
  return config;
}

[[nodiscard]] SpeedControllerInput cruiseInput() {
  SpeedControllerInput input{};
  input.hold_position = false;
  input.controller_dt_s = 0.1;
  input.distance_to_goal_m = 100.0;
  input.turn_angle_rad = 0.0;
  input.actual_speed_mps = 0.0;
  return input;
}

} // namespace

TEST(OffboardSpeedController, CruiseSpeedAdvancesBySpeedTimesDt) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  OffboardSpeedController controller{config};

  const SpeedControllerOutput output = controller.update(cruiseInput());

  EXPECT_NEAR(output.requested_speed_mps, 5.0, 1.0e-9);
  EXPECT_NEAR(output.target_step_m, 0.5, 1.0e-9);
  EXPECT_STREQ(speedLimitReasonName(output.limit_reason), "cruise");
}

TEST(OffboardSpeedController, DebugBypassIgnoresAccelerationRamp) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 2.0;
  OffboardSpeedController controller{config};

  const SpeedControllerOutput output = controller.update(cruiseInput());

  EXPECT_NEAR(output.requested_speed_mps, 5.0, 1.0e-9);
  EXPECT_NEAR(output.target_step_m, 0.5, 1.0e-9);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kCruise);
}

TEST(OffboardSpeedController, DebugBypassIgnoresGoalBraking) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  OffboardSpeedController controller{config};
  SpeedControllerInput input = cruiseInput();
  input.distance_to_goal_m = 2.0;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_DOUBLE_EQ(output.allowed_speed_mps, 5.0);
  EXPECT_DOUBLE_EQ(output.requested_speed_mps, 5.0);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kCruise);
  EXPECT_LE(output.braking_distance_m, 2.0);
}

TEST(OffboardSpeedController, DebugBypassIgnoresTurnSlowdown) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  OffboardSpeedController controller{config};
  SpeedControllerInput input = cruiseInput();
  input.turn_angle_rad = 1.5707963267948966;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_NEAR(output.requested_speed_mps, 5.0, 1.0e-9);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kCruise);
}

TEST(OffboardSpeedController, HoldModeRequestsZeroSpeed) {
  OffboardSpeedController controller{testConfig()};
  SpeedControllerInput input = cruiseInput();
  input.hold_position = true;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_DOUBLE_EQ(output.requested_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(output.target_step_m, 0.0);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kHold);
}

TEST(OffboardSpeedController, DebugBypassIgnoresMaxCommandedTargetStep) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_commanded_target_step_m = 0.25;
  OffboardSpeedController controller{config};

  const SpeedControllerOutput output = controller.update(cruiseInput());

  EXPECT_NEAR(output.requested_speed_mps, 5.0, 1.0e-9);
  EXPECT_NEAR(output.target_step_m, 0.5, 1.0e-9);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kCruise);
}

TEST(OffboardSpeedController, TrackingOverspeedLimitIsDisabledByDefault) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  OffboardSpeedController controller{config};

  (void)controller.update(cruiseInput());

  config.max_accel_mps2 = 2.0;
  controller.setConfig(config);
  SpeedControllerInput input = cruiseInput();
  input.turn_angle_rad = std::numbers::pi;
  input.actual_speed_mps = 5.0;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_DOUBLE_EQ(output.allowed_speed_mps, 5.0);
  EXPECT_DOUBLE_EQ(output.requested_speed_mps, 5.0);
  EXPECT_DOUBLE_EQ(output.target_step_m, 0.5);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kCruise);
}

TEST(OffboardSpeedController, DebugBypassIgnoresTrackingOverspeedLimit) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  OffboardSpeedController controller{config};

  (void)controller.update(cruiseInput());

  config.max_accel_mps2 = 2.0;
  config.tracking_overspeed_limit_enabled = true;
  config.tracking_overspeed_limit_mps = 2.0;
  controller.setConfig(config);
  SpeedControllerInput input = cruiseInput();
  input.turn_angle_rad = std::numbers::pi;
  input.actual_speed_mps = 5.0;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_DOUBLE_EQ(output.allowed_speed_mps, 5.0);
  EXPECT_TRUE(std::isinf(output.limits.tracking_overspeed_limit_mps));
  EXPECT_DOUBLE_EQ(output.requested_speed_mps, 5.0);
  EXPECT_DOUBLE_EQ(output.target_step_m, 0.5);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kCruise);
}

TEST(OffboardSpeedController, DebugBypassIgnoresHardStepCapWithMinCommandSpeed) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.min_command_speed_mps = 4.0;
  config.max_commanded_target_step_m = 0.2;
  OffboardSpeedController controller{config};

  const SpeedControllerOutput output = controller.update(cruiseInput());

  EXPECT_NEAR(output.requested_speed_mps, 5.0, 1.0e-9);
  EXPECT_NEAR(output.target_step_m, 0.5, 1.0e-9);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kCruise);
}

TEST(OffboardSpeedController, DebugBypassIgnoresGoalStopWithMinCommandSpeed) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 10.0;
  config.min_command_speed_mps = 1.5;
  OffboardSpeedController controller{config};
  (void)controller.update(cruiseInput());
  (void)controller.update(cruiseInput());

  SpeedControllerInput input = cruiseInput();
  input.distance_to_goal_m = 0.5;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_DOUBLE_EQ(output.requested_speed_mps, 5.0);
  EXPECT_DOUBLE_EQ(output.target_step_m, 0.5);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kCruise);
}

TEST(OffboardSpeedController, NonFiniteInputsFailClosedToHold) {
  OffboardSpeedController controller{testConfig()};
  SpeedControllerInput input = cruiseInput();
  input.distance_to_goal_m = std::numeric_limits<double>::quiet_NaN();

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_DOUBLE_EQ(output.requested_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(output.target_step_m, 0.0);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kInvalidInput);
}

} // namespace drone_city_nav
