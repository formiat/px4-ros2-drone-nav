#pragma once

#include "drone_city_nav/offboard_velocity_follower.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace offboard_velocity_follower_test_helpers {

[[nodiscard]] inline VelocityFollowerConfig testConfig() {
  VelocityFollowerConfig config{};
  config.cruise_speed_mps = 12.0;
  config.min_turn_speed_mps = 2.0;
  config.max_accel_mps2 = 3.0;
  config.max_decel_mps2 = 4.0;
  config.max_lateral_accel_mps2 = 3.0;
  config.speed_profile_sample_step_m = 1.0;
  config.final_acceptance_radius_m = 1.0;
  config.final_hold_max_speed_mps = 0.8;
  config.terminal_capture_radius_m = 8.0;
  config.terminal_capture_gain_1ps = 1.0;
  config.terminal_capture_max_speed_mps = 8.0;
  config.terminal_capture_decel_mps2 = 4.0;
  config.terminal_capture_braking_margin_m = 2.0;
  config.tracking_prediction_horizon_s = 0.0;
  return config;
}

[[nodiscard]] inline std::vector<TrajectorySegment> lineTrajectory() {
  return lineTrajectoryFromPoints(std::vector<Point2>{{0.0, 0.0}, {100.0, 0.0}});
}

[[nodiscard]] inline std::vector<TrajectorySegment>
trajectoryWithArc(const double radius_m) {
  std::vector<TrajectorySegment> trajectory;
  trajectory.push_back(makeLineSegment(Point2{0.0, 0.0}, Point2{20.0, 0.0}));
  trajectory.push_back(makeArcSegment(Point2{20.0, 0.0},
                                      Point2{20.0 + radius_m, radius_m},
                                      Point2{20.0, radius_m}, -std::numbers::pi / 2.0));
  trajectory.push_back(makeLineSegment(Point2{20.0 + radius_m, radius_m},
                                       Point2{20.0 + radius_m, radius_m + 40.0}));
  assignTrajectoryStationing(trajectory);
  return trajectory;
}

[[nodiscard]] inline Point2 normalizedTestVector(const double x, const double y) {
  const double length = std::hypot(x, y);
  return Point2{x / length, y / length};
}

} // namespace offboard_velocity_follower_test_helpers
} // namespace drone_city_nav
