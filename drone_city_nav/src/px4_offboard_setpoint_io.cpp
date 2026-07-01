#include "drone_city_nav/px4_offboard_setpoint_io.hpp"

#include <array>
#include <cmath>
#include <limits>

namespace drone_city_nav {

[[nodiscard]] const char*
offboardSetpointModeName(const OffboardSetpointMode mode) noexcept {
  switch (mode) {
    case OffboardSetpointMode::kPositionHold:
      return "position_hold";
    case OffboardSetpointMode::kTerminalPositionCapture:
      return "terminal_position_capture";
    case OffboardSetpointMode::kVelocityCruise:
      return "velocity_cruise";
  }
  return "unknown";
}

[[nodiscard]] const char* commandName(const std::uint32_t command) noexcept {
  switch (command) {
    case px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE:
      return "VEHICLE_CMD_DO_SET_MODE";
    case px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM:
      return "VEHICLE_CMD_COMPONENT_ARM_DISARM";
    default:
      return "UNKNOWN";
  }
}

[[nodiscard]] px4_msgs::msg::OffboardControlMode
buildOffboardControlMode(const std::uint64_t timestamp_us,
                         const OffboardSetpointMode mode) {
  px4_msgs::msg::OffboardControlMode msg;
  msg.timestamp = timestamp_us;
  msg.position = mode == OffboardSetpointMode::kPositionHold ||
                 mode == OffboardSetpointMode::kTerminalPositionCapture;
  msg.velocity = mode == OffboardSetpointMode::kVelocityCruise;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;
  return msg;
}

[[nodiscard]] px4_msgs::msg::TrajectorySetpoint
buildPositionTrajectorySetpoint(const std::uint64_t timestamp_us,
                                const Point2 local_target,
                                const double cruise_altitude_m, const double yaw_rad) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  px4_msgs::msg::TrajectorySetpoint msg;
  msg.timestamp = timestamp_us;
  msg.position = std::array<float, 3>{static_cast<float>(local_target.x),
                                      static_cast<float>(local_target.y),
                                      static_cast<float>(-std::abs(cruise_altitude_m))};
  msg.velocity = std::array<float, 3>{nan, nan, nan};
  msg.acceleration = std::array<float, 3>{nan, nan, nan};
  msg.jerk = std::array<float, 3>{nan, nan, nan};
  msg.yaw = static_cast<float>(yaw_rad);
  msg.yawspeed = nan;
  return msg;
}

[[nodiscard]] px4_msgs::msg::TrajectorySetpoint buildVelocityTrajectorySetpoint(
    const std::uint64_t timestamp_us, const Point2 velocity_xy,
    const double vertical_velocity_ned_mps, const double yaw_rad) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  px4_msgs::msg::TrajectorySetpoint msg;
  msg.timestamp = timestamp_us;
  msg.position = std::array<float, 3>{nan, nan, nan};
  msg.velocity = std::array<float, 3>{static_cast<float>(velocity_xy.x),
                                      static_cast<float>(velocity_xy.y),
                                      static_cast<float>(vertical_velocity_ned_mps)};
  msg.acceleration = std::array<float, 3>{nan, nan, nan};
  msg.jerk = std::array<float, 3>{nan, nan, nan};
  msg.yaw = static_cast<float>(yaw_rad);
  msg.yawspeed = nan;
  return msg;
}

[[nodiscard]] px4_msgs::msg::VehicleCommand
buildVehicleCommand(const std::uint64_t timestamp_us, const std::uint32_t command,
                    const float param1, const float param2,
                    const VehicleCommandEndpoint& endpoint) {
  px4_msgs::msg::VehicleCommand msg;
  msg.timestamp = timestamp_us;
  msg.command = command;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.target_system = endpoint.target_system;
  msg.target_component = endpoint.target_component;
  msg.source_system = endpoint.source_system;
  msg.source_component = endpoint.source_component;
  msg.from_external = true;
  return msg;
}

} // namespace drone_city_nav
