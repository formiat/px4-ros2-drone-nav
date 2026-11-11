#pragma once

#include "drone_city_nav/types.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <cstdint>
#include <limits>

namespace drone_city_nav {

inline constexpr float kPx4ForceDisarmMagicParam2{21196.0F};

enum class OffboardSetpointMode : std::uint8_t {
  kPositionHold,
  kTrajectoryPositionTracking,
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

// The hold a vehicle is handed while nothing owns its motion and it still
// moves. A position setpoint alone brakes at whatever the position loop makes
// of a zero error: measured, a vehicle handed such a hold at 1.9 m/s took
// nine tenths of a metre to stop, half the deceleration the navigation stack
// plans its stopping distances against, and met the structure it had been
// stopped for. The hold therefore carries the braking as well: the position
// it is pinned to, a zero velocity, and an acceleration opposing the
// vehicle's velocity. The acceleration is the one that brings the velocity to
// zero over `braking_response_s`, capped at `braking_acceleration_mps2`: full
// braking from speed, and a proportional one near rest. Full braking against a
// residual of a few tenths of a metre a second throws the vehicle the other
// way, and one recorded flight rang between the two for a second and a half at
// half a metre a second, never below the rest tolerance, and walked forty
// centimetres into the wall it had just stopped for. A vehicle already at rest
// gets the plain position hold.
[[nodiscard]] px4_msgs::msg::TrajectorySetpoint buildBrakingHoldTrajectorySetpoint(
    std::uint64_t timestamp_us, Point2 local_target, double target_altitude_m,
    Point2 local_velocity_xy, double vertical_velocity_up_mps,
    double braking_acceleration_mps2, double braking_response_s, double yaw_rad);

[[nodiscard]] px4_msgs::msg::TrajectorySetpoint
buildMppiTrajectorySetpoint(std::uint64_t timestamp_us, Point2 velocity_xy,
                            double vertical_velocity_up_mps, Point2 acceleration_xy,
                            double vertical_acceleration_up_mps2, double yaw_rad,
                            double yaw_rate_radps);

[[nodiscard]] px4_msgs::msg::TrajectorySetpoint buildMppiPathTrajectorySetpoint(
    std::uint64_t timestamp_us, Point2 local_position_xy, double altitude_m,
    Point2 velocity_xy, double vertical_velocity_up_mps, Point2 acceleration_xy,
    double vertical_acceleration_up_mps2, double yaw_rad, double yaw_rate_radps);

[[nodiscard]] px4_msgs::msg::VehicleCommand
buildVehicleCommand(std::uint64_t timestamp_us, std::uint32_t command, float param1,
                    float param2, const VehicleCommandEndpoint& endpoint);

} // namespace drone_city_nav
