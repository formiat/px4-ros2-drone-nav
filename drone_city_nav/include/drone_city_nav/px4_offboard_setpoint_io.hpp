#pragma once

#include "drone_city_nav/types.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <cstdint>
#include <limits>

namespace drone_city_nav {

enum class OffboardSetpointMode : std::uint8_t {
  kPositionHold,
  kTerminalPositionCapture,
  kVelocityCruise,
};

struct VehicleCommandEndpoint {
  std::uint8_t target_system{1U};
  std::uint8_t target_component{1U};
  std::uint8_t source_system{1U};
  std::uint16_t source_component{1U};
};

[[nodiscard]] const char* offboardSetpointModeName(OffboardSetpointMode mode) noexcept;

[[nodiscard]] const char* commandName(std::uint32_t command) noexcept;

[[nodiscard]] px4_msgs::msg::OffboardControlMode
buildOffboardControlMode(std::uint64_t timestamp_us, OffboardSetpointMode mode);

[[nodiscard]] px4_msgs::msg::TrajectorySetpoint buildPositionTrajectorySetpoint(
    std::uint64_t timestamp_us, Point2 local_target, double target_altitude_m,
    double yaw_rad,
    double vertical_velocity_up_mps = std::numeric_limits<double>::quiet_NaN());

[[nodiscard]] px4_msgs::msg::TrajectorySetpoint
buildVelocityTrajectorySetpoint(std::uint64_t timestamp_us, Point2 velocity_xy,
                                double vertical_velocity_ned_mps, double yaw_rad);

[[nodiscard]] px4_msgs::msg::VehicleCommand
buildVehicleCommand(std::uint64_t timestamp_us, std::uint32_t command, float param1,
                    float param2, const VehicleCommandEndpoint& endpoint);

} // namespace drone_city_nav
