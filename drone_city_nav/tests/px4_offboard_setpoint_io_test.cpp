#include "drone_city_nav/px4_offboard_setpoint_io.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {

TEST(Px4OffboardSetpointIo, BuildsPositionAndVelocityControlModes) {
  const auto position =
      buildOffboardControlMode(11U, OffboardSetpointMode::kPositionHold);
  EXPECT_EQ(position.timestamp, 11U);
  EXPECT_TRUE(position.position);
  EXPECT_FALSE(position.velocity);
  EXPECT_FALSE(position.acceleration);

  const auto trajectory =
      buildOffboardControlMode(13U, OffboardSetpointMode::kTrajectoryPositionTracking);
  EXPECT_EQ(trajectory.timestamp, 13U);
  EXPECT_TRUE(trajectory.position);
  EXPECT_FALSE(trajectory.velocity);
  EXPECT_FALSE(trajectory.acceleration);

  const auto velocity =
      buildOffboardControlMode(12U, OffboardSetpointMode::kVelocityCruise);
  EXPECT_EQ(velocity.timestamp, 12U);
  EXPECT_FALSE(velocity.position);
  EXPECT_TRUE(velocity.velocity);
  EXPECT_FALSE(velocity.acceleration);
}

TEST(Px4OffboardSetpointIo, BrakingHoldOpposesTheVelocityAtTheBrakingAcceleration) {
  // Moving at 3-4-0 m/s (speed 5), the hold pins the position, asks for zero
  // velocity, and brakes along the velocity at the given magnitude: 4 m/s^2
  // splits as -2.4, -3.2 horizontally, nothing vertically.
  const auto braking = buildBrakingHoldTrajectorySetpoint(
      7U, Point2{1.0, 2.0}, 10.0, Point2{3.0, 4.0}, 0.0, 4.0, 0.5);
  EXPECT_FLOAT_EQ(braking.position[0], 1.0F);
  EXPECT_FLOAT_EQ(braking.position[1], 2.0F);
  EXPECT_FLOAT_EQ(braking.position[2], -10.0F);
  EXPECT_FLOAT_EQ(braking.velocity[0], 0.0F);
  EXPECT_FLOAT_EQ(braking.velocity[1], 0.0F);
  EXPECT_FLOAT_EQ(braking.velocity[2], 0.0F);
  EXPECT_FLOAT_EQ(braking.acceleration[0], -2.4F);
  EXPECT_FLOAT_EQ(braking.acceleration[1], -3.2F);
  EXPECT_FLOAT_EQ(braking.acceleration[2], 0.0F);
  EXPECT_FLOAT_EQ(braking.yaw, 0.5F);

  // A climb is braked too, and upward motion reads as a downward (NED
  // positive) acceleration.
  const auto climbing = buildBrakingHoldTrajectorySetpoint(
      7U, Point2{0.0, 0.0}, 10.0, Point2{0.0, 0.0}, 2.0, 4.0, 0.0);
  EXPECT_FLOAT_EQ(climbing.acceleration[0], 0.0F);
  EXPECT_FLOAT_EQ(climbing.acceleration[2], 4.0F);

  // At rest the hold is the plain position hold, with no motion feedforward.
  const auto resting = buildBrakingHoldTrajectorySetpoint(
      7U, Point2{1.0, 2.0}, 10.0, Point2{0.0, 0.0}, 0.0, 4.0, 0.0);
  EXPECT_FLOAT_EQ(resting.position[0], 1.0F);
  EXPECT_TRUE(std::isnan(resting.velocity[0]));
  EXPECT_TRUE(std::isnan(resting.acceleration[0]));
}

TEST(Px4OffboardSetpointIo, BuildsPositionSetpointWithNedAltitude) {
  const auto msg = buildPositionTrajectorySetpoint(100U, Point2{3.0, -4.0}, 12.5, 1.2);

  EXPECT_EQ(msg.timestamp, 100U);
  EXPECT_FLOAT_EQ(msg.position[0], 3.0F);
  EXPECT_FLOAT_EQ(msg.position[1], -4.0F);
  EXPECT_FLOAT_EQ(msg.position[2], -12.5F);
  EXPECT_TRUE(std::isnan(msg.velocity[0]));
  EXPECT_TRUE(std::isnan(msg.acceleration[0]));
  EXPECT_TRUE(std::isnan(msg.jerk[0]));
  EXPECT_FLOAT_EQ(msg.yaw, 1.2F);
  EXPECT_TRUE(std::isnan(msg.yawspeed));
}

TEST(Px4OffboardSetpointIo, KeepsPositionSetpointBelowTheLocalOrigin) {
  // A hold 3.77 m below the spawn level stays below it in NED instead of being
  // mirrored to 3.77 m above.
  const auto msg = buildPositionTrajectorySetpoint(102U, Point2{1.0, 2.0}, -3.77, 0.0);

  EXPECT_FLOAT_EQ(msg.position[2], 3.77F);
}

TEST(Px4OffboardSetpointIo, BuildsVelocitySetpointWithoutPosition) {
  const auto msg =
      buildVelocityTrajectorySetpoint(200U, Point2{5.0, -6.0}, -0.75, -1.0);

  EXPECT_EQ(msg.timestamp, 200U);
  EXPECT_TRUE(std::isnan(msg.position[0]));
  EXPECT_FLOAT_EQ(msg.velocity[0], 5.0F);
  EXPECT_FLOAT_EQ(msg.velocity[1], -6.0F);
  EXPECT_FLOAT_EQ(msg.velocity[2], -0.75F);
  EXPECT_TRUE(std::isnan(msg.acceleration[0]));
  EXPECT_TRUE(std::isnan(msg.jerk[0]));
  EXPECT_FLOAT_EQ(msg.yaw, -1.0F);
}

TEST(Px4OffboardSetpointIo, AddsVerticalFeedforwardToPositionSetpoint) {
  const auto msg =
      buildPositionTrajectorySetpoint(101U, Point2{3.0, -4.0}, 12.5, 1.2, -2.0);

  EXPECT_TRUE(std::isnan(msg.velocity[0]));
  EXPECT_TRUE(std::isnan(msg.velocity[1]));
  EXPECT_FLOAT_EQ(msg.velocity[2], 2.0F);
}

TEST(Px4OffboardSetpointIo, BuildsPathPositionWithDynamicFeedforward) {
  const auto msg =
      buildMppiPathTrajectorySetpoint(201U, Point2{3.0, -4.0}, 12.5, Point2{5.0, -6.0},
                                      0.75, Point2{1.5, -2.5}, -0.4, 1.2, -0.3);

  EXPECT_EQ(msg.timestamp, 201U);
  EXPECT_FLOAT_EQ(msg.position[0], 3.0F);
  EXPECT_FLOAT_EQ(msg.position[1], -4.0F);
  EXPECT_FLOAT_EQ(msg.position[2], -12.5F);
  EXPECT_FLOAT_EQ(msg.velocity[0], 5.0F);
  EXPECT_FLOAT_EQ(msg.velocity[1], -6.0F);
  EXPECT_FLOAT_EQ(msg.velocity[2], -0.75F);
  EXPECT_FLOAT_EQ(msg.acceleration[0], 1.5F);
  EXPECT_FLOAT_EQ(msg.acceleration[1], -2.5F);
  EXPECT_FLOAT_EQ(msg.acceleration[2], 0.4F);
  EXPECT_FLOAT_EQ(msg.yaw, 1.2F);
  EXPECT_FLOAT_EQ(msg.yawspeed, -0.3F);
}

TEST(Px4OffboardSetpointIo, BuildsVehicleCommandEndpoint) {
  const VehicleCommandEndpoint endpoint{2U, 3U, 4U, 5U};
  const auto msg = buildVehicleCommand(
      300U, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F,
      21196.0F, endpoint);

  EXPECT_EQ(msg.timestamp, 300U);
  EXPECT_EQ(msg.command,
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM);
  EXPECT_FLOAT_EQ(msg.param1, 1.0F);
  EXPECT_FLOAT_EQ(msg.param2, 21196.0F);
  EXPECT_EQ(msg.target_system, 2U);
  EXPECT_EQ(msg.target_component, 3U);
  EXPECT_EQ(msg.source_system, 4U);
  EXPECT_EQ(msg.source_component, 5U);
  EXPECT_TRUE(msg.from_external);
  EXPECT_STREQ(commandName(msg.command), "VEHICLE_CMD_COMPONENT_ARM_DISARM");
  EXPECT_STREQ(offboardSetpointModeName(OffboardSetpointMode::kPositionHold),
               "position_hold");
  EXPECT_STREQ(
      offboardSetpointModeName(OffboardSetpointMode::kTrajectoryPositionTracking),
      "trajectory_position_tracking");
  EXPECT_STREQ(offboardSetpointModeName(OffboardSetpointMode::kVelocityCruise),
               "velocity_cruise");
}

TEST(Px4OffboardSetpointIo, BuildsPx4ForceDisarmCommand) {
  const VehicleCommandEndpoint endpoint;

  const auto msg = buildVehicleCommand(
      301U, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F,
      kPx4ForceDisarmMagicParam2, endpoint);

  EXPECT_FLOAT_EQ(msg.param1, 0.0F);
  EXPECT_FLOAT_EQ(msg.param2, 21196.0F);
}

} // namespace drone_city_nav
