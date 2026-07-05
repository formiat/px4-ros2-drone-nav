#pragma once

#include "drone_city_nav/types.hpp"
#include "drone_city_nav/velocity_control_config.hpp"

#include <limits>

namespace drone_city_nav {

struct VelocitySmootherInput {
  Point2 desired_velocity_xy{};
  Point2 path_tangent{};
  Point2 previous_velocity_setpoint{};
  Point2 previous_velocity_acceleration_setpoint{};
  bool previous_velocity_setpoint_valid{false};
  bool previous_velocity_acceleration_setpoint_valid{false};
  double dt_s{std::numeric_limits<double>::quiet_NaN()};
};

struct VelocitySmootherPlan {
  bool valid{false};
  Point2 velocity_xy{};
  Point2 velocity_setpoint_acceleration_xy{};
  double velocity_delta_mps{std::numeric_limits<double>::quiet_NaN()};
  double desired_velocity_delta_mps{std::numeric_limits<double>::quiet_NaN()};
  double velocity_setpoint_acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
  double velocity_setpoint_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
  bool path_frame_lateral_smoothing_applied{false};
  double smoother_lateral_response_accel_mps2{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] VelocitySmootherPlan
smoothVelocityCommand(const VelocitySmootherInput& input,
                      const VelocityFollowerConfig& config);

} // namespace drone_city_nav
