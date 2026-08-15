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
    case OffboardSetpointMode::kTrajectoryPositionTracking:
      return "trajectory_position_tracking";
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
                 mode == OffboardSetpointMode::kTrajectoryPositionTracking;
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
                                const double target_altitude_m, const double yaw_rad,
                                const double vertical_velocity_up_mps) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  px4_msgs::msg::TrajectorySetpoint msg;
  msg.timestamp = timestamp_us;
  msg.position = std::array<float, 3>{static_cast<float>(local_target.x),
                                      static_cast<float>(local_target.y),
                                      static_cast<float>(-std::abs(target_altitude_m))};
  msg.velocity =
      std::array<float, 3>{nan, nan,
                           std::isfinite(vertical_velocity_up_mps)
                               ? static_cast<float>(-vertical_velocity_up_mps)
                               : nan};
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

px4_msgs::msg::TrajectorySetpoint
buildMppiTrajectorySetpoint(const std::uint64_t timestamp_us, const Point2 velocity_xy,
                            const double vertical_velocity_up_mps,
                            const Point2 acceleration_xy,
                            const double vertical_acceleration_up_mps2,
                            const double yaw_rad, const double yaw_rate_radps) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  px4_msgs::msg::TrajectorySetpoint msg;
  msg.timestamp = timestamp_us;
  msg.position = std::array<float, 3>{nan, nan, nan};
  msg.velocity = std::array<float, 3>{static_cast<float>(velocity_xy.x),
                                      static_cast<float>(velocity_xy.y),
                                      static_cast<float>(-vertical_velocity_up_mps)};
  msg.acceleration = std::array<float, 3>{
      static_cast<float>(acceleration_xy.x), static_cast<float>(acceleration_xy.y),
      static_cast<float>(-vertical_acceleration_up_mps2)};
  msg.jerk = std::array<float, 3>{nan, nan, nan};
  msg.yaw = static_cast<float>(yaw_rad);
  msg.yawspeed = static_cast<float>(yaw_rate_radps);
  return msg;
}

px4_msgs::msg::TrajectorySetpoint buildMppiPathTrajectorySetpoint(
    const std::uint64_t timestamp_us, const Point2 local_position_xy,
    const double altitude_m, const Point2 velocity_xy,
    const double vertical_velocity_up_mps, const Point2 acceleration_xy,
    const double vertical_acceleration_up_mps2, const double yaw_rad,
    const double yaw_rate_radps) {
  px4_msgs::msg::TrajectorySetpoint msg = buildMppiTrajectorySetpoint(
      timestamp_us, velocity_xy, vertical_velocity_up_mps, acceleration_xy,
      vertical_acceleration_up_mps2, yaw_rad, yaw_rate_radps);
  msg.position = std::array<float, 3>{static_cast<float>(local_position_xy.x),
                                      static_cast<float>(local_position_xy.y),
                                      static_cast<float>(-altitude_m)};
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
