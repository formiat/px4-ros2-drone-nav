#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/types.hpp"
#include "drone_city_nav/velocity_control_config.hpp"

#include <limits>

namespace drone_city_nav {

struct VelocityCommandQuery {
  TrajectoryProjection projection{};
  Point2 current_position{};
  Point2 current_velocity{};
  bool current_velocity_valid{false};
  double scalar_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double dt_s{std::numeric_limits<double>::quiet_NaN()};
  double current_cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  double predicted_cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  double curvature_feedforward_context_scale{1.0};
};

struct VelocityCommandPlan {
  bool valid{false};
  Point2 desired_velocity_xy{};
  Point2 cross_track_feedback_velocity{};
  Point2 cross_track_derivative_damping_velocity{};
  Point2 curvature_feedforward_velocity{};
  Point2 raw_lateral_control_velocity{};
  Point2 lateral_control_velocity{};
  double cross_track_feedback_mps{0.0};
  double cross_track_p_gain_factor{1.0};
  double cross_track_derivative_damping_mps{0.0};
  double cross_track_derivative_damping_factor{1.0};
  double cross_track_derivative_gain_effective{0.0};
  double cross_track_lateral_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double curvature_feedforward_mps{0.0};
  double curvature_feedforward_angle_rad{0.0};
  double curvature_feedforward_raw_angle_rad{0.0};
  double curvature_feedforward_scale{1.0};
  double curvature_feedforward_context_scale{1.0};
  double raw_lateral_control_mps{0.0};
  double lateral_control_mps{0.0};
  double desired_velocity_tangent_mps{std::numeric_limits<double>::quiet_NaN()};
  double desired_velocity_normal_mps{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] VelocityCommandPlan
planVelocityCommand(const VelocityCommandQuery& query,
                    const VelocityFollowerConfig& config);

} // namespace drone_city_nav
