#pragma once

#include "drone_city_nav/types.hpp"
#include "drone_city_nav/velocity_control_config.hpp"

#include <limits>

namespace drone_city_nav {

struct VelocityVectorLimitResult {
  Point2 velocity{};
  double delta_mps{0.0};
};

struct VelocitySmootherInput {
  Point2 desired_velocity_xy{};
  Point2 path_tangent{};
  Point2 previous_velocity_setpoint{};
  Point2 previous_velocity_acceleration_setpoint{};
  bool previous_velocity_setpoint_valid{false};
  bool previous_velocity_acceleration_setpoint_valid{false};
  double dt_s{std::numeric_limits<double>::quiet_NaN()};
  double lateral_response_factor{1.0};
  double current_cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  double predicted_cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
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
  bool lateral_zero_crossing_limited{false};
  bool velocity_heading_rate_limited{false};
  double lateral_smoothing_factor{1.0};
  double smoother_lateral_response_accel_mps2{std::numeric_limits<double>::quiet_NaN()};
  double velocity_heading_rate_limit_rad_s{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] VelocityVectorLimitResult
limitVelocityVectorDelta(Point2 desired_velocity, Point2 previous_velocity,
                         bool previous_velocity_valid, double dt_s,
                         double max_delta_mps2);

[[nodiscard]] VelocityVectorLimitResult
limitVelocityVectorDelta(Point2 desired_velocity, Point2 previous_velocity,
                         bool previous_velocity_valid, double dt_s,
                         double max_accel_mps2, double max_decel_mps2);

[[nodiscard]] VelocitySmootherPlan
smoothVelocityCommand(const VelocitySmootherInput& input,
                      const VelocityFollowerConfig& config);

} // namespace drone_city_nav
