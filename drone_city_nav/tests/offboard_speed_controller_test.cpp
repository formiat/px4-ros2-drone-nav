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
  config.narrow_clearance_slowdown_radius_m = 6.0;
  config.narrow_clearance_min_speed_mps = 1.0;
  config.clearance_braking_margin_m = 1.0;
  config.max_commanded_target_step_m = 2.0;
  return config;
}

[[nodiscard]] SpeedControllerInput cruiseInput() {
  SpeedControllerInput input{};
  input.hold_position = false;
  input.controller_dt_s = 0.1;
  input.distance_to_goal_m = 100.0;
  input.turn_angle_rad = 0.0;
  input.local_clearance_m = std::numeric_limits<double>::infinity();
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

TEST(OffboardSpeedController, AccelerationRampLimitsSpeedIncrease) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 2.0;
  OffboardSpeedController controller{config};

  const SpeedControllerOutput output = controller.update(cruiseInput());

  EXPECT_NEAR(output.requested_speed_mps, 0.2, 1.0e-9);
  EXPECT_NEAR(output.target_step_m, 0.02, 1.0e-9);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kAcceleration);
}

TEST(OffboardSpeedController, BrakingDistanceLimitsGoalApproach) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  OffboardSpeedController controller{config};
  SpeedControllerInput input = cruiseInput();
  input.distance_to_goal_m = 2.0;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_LE(output.allowed_speed_mps, 1.1);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kGoal);
  EXPECT_LE(output.braking_distance_m, 2.0);
}

TEST(OffboardSpeedController, SharpTurnUsesTurnSlowdownLimit) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  OffboardSpeedController controller{config};
  SpeedControllerInput input = cruiseInput();
  input.turn_angle_rad = 1.5707963267948966;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_NEAR(output.requested_speed_mps, 2.0, 1.0e-9);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kTurn);
}

TEST(OffboardSpeedController, NarrowClearanceUsesClearanceLimit) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  OffboardSpeedController controller{config};
  SpeedControllerInput input = cruiseInput();
  input.local_clearance_m = 1.5;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_NEAR(output.requested_speed_mps, 2.0, 1.0e-9);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kClearance);
}

TEST(OffboardSpeedController, ClearanceBrakingLimitCanBeatLinearRamp) {
  SpeedControllerConfig config = testConfig();
  config.desired_speed_mps = 10.0;
  config.max_accel_mps2 = 2.0;
  config.narrow_clearance_slowdown_radius_m = 10.0;
  config.narrow_clearance_min_speed_mps = 0.0;
  config.max_commanded_target_step_m = 100.0;
  OffboardSpeedController controller{config};
  SpeedControllerInput input = cruiseInput();
  input.controller_dt_s = 10.0;
  input.local_clearance_m = 1.5;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_NEAR(output.allowed_speed_mps, std::sqrt(2.0), 1.0e-9);
  EXPECT_NEAR(output.requested_speed_mps, std::sqrt(2.0), 1.0e-9);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kClearance);
}

TEST(OffboardSpeedController, ClearanceInsideBrakingMarginStops) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  OffboardSpeedController controller{config};
  SpeedControllerInput input = cruiseInput();
  input.local_clearance_m = 0.5;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_DOUBLE_EQ(output.allowed_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(output.requested_speed_mps, 0.0);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kClearance);
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

TEST(OffboardSpeedController, MaxCommandedTargetStepHardCapWins) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_commanded_target_step_m = 0.25;
  OffboardSpeedController controller{config};

  const SpeedControllerOutput output = controller.update(cruiseInput());

  EXPECT_NEAR(output.requested_speed_mps, 2.5, 1.0e-9);
  EXPECT_NEAR(output.target_step_m, 0.25, 1.0e-9);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kHardStepCap);
}

TEST(OffboardSpeedController, ClearanceOverspeedRequestsStop) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  OffboardSpeedController controller{config};

  (void)controller.update(cruiseInput());

  config.max_accel_mps2 = 2.0;
  controller.setConfig(config);
  SpeedControllerInput input = cruiseInput();
  input.local_clearance_m = 1.5;
  input.actual_speed_mps = 5.0;

  const SpeedControllerOutput output = controller.update(input);
  const double expected_allowed_speed = std::numbers::sqrt2;

  EXPECT_NEAR(output.allowed_speed_mps, expected_allowed_speed, 1.0e-9);
  EXPECT_DOUBLE_EQ(output.requested_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(output.target_step_m, 0.0);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kTrackingOverspeed);
}

TEST(OffboardSpeedController, MinCommandSpeedDoesNotBypassHardStepCap) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.min_command_speed_mps = 4.0;
  config.max_commanded_target_step_m = 0.2;
  OffboardSpeedController controller{config};

  const SpeedControllerOutput output = controller.update(cruiseInput());

  EXPECT_NEAR(output.requested_speed_mps, 2.0, 1.0e-9);
  EXPECT_NEAR(output.target_step_m, 0.2, 1.0e-9);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kHardStepCap);
}

TEST(OffboardSpeedController, MinCommandSpeedDoesNotPreventGoalStop) {
  SpeedControllerConfig config = testConfig();
  config.max_accel_mps2 = 10.0;
  config.min_command_speed_mps = 1.5;
  OffboardSpeedController controller{config};
  (void)controller.update(cruiseInput());
  (void)controller.update(cruiseInput());

  SpeedControllerInput input = cruiseInput();
  input.distance_to_goal_m = 0.5;

  const SpeedControllerOutput output = controller.update(input);

  EXPECT_DOUBLE_EQ(output.requested_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(output.target_step_m, 0.0);
  EXPECT_EQ(output.limit_reason, SpeedLimitReason::kGoal);
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
