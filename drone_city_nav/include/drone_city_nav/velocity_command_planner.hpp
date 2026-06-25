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
  Point2 previous_cross_track_correction_velocity{};
  bool previous_cross_track_correction_velocity_valid{false};
};

struct VelocityCommandPlan {
  bool valid{false};
  Point2 desired_velocity_xy{};
  Point2 raw_cross_track_correction_velocity{};
  Point2 cross_track_correction_velocity{};
  double raw_cross_track_correction_mps{0.0};
  double cross_track_correction_mps{0.0};
  double cross_track_correction_delta_mps{std::numeric_limits<double>::quiet_NaN()};
  double cross_track_lateral_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double desired_velocity_tangent_mps{std::numeric_limits<double>::quiet_NaN()};
  double desired_velocity_normal_mps{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] VelocityCommandPlan
planVelocityCommand(const VelocityCommandQuery& query,
                    const VelocityFollowerConfig& config);

} // namespace drone_city_nav
