#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/velocity_control_config.hpp"

#include <limits>
#include <span>

namespace drone_city_nav {

struct VerticalTrackabilitySpeedCap {
  bool active{false};
  double speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double constraint_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double altitude_error_m{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] VerticalTrackabilitySpeedCap computeVerticalTrackabilitySpeedCap(
    std::span<const TrajectoryPointSample> trajectory_samples, double trajectory_s_m,
    double current_altitude_m, bool altitude_valid,
    double current_vertical_velocity_mps, bool vertical_velocity_valid,
    const VelocityFollowerConfig& config);

} // namespace drone_city_nav
