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

  const auto velocity =
      buildOffboardControlMode(12U, OffboardSetpointMode::kVelocityCruise);
  EXPECT_EQ(velocity.timestamp, 12U);
  EXPECT_FALSE(velocity.position);
  EXPECT_TRUE(velocity.velocity);
  EXPECT_FALSE(velocity.acceleration);
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
  EXPECT_STREQ(offboardSetpointModeName(OffboardSetpointMode::kVelocityCruise),
               "velocity_cruise");
}

} // namespace drone_city_nav
