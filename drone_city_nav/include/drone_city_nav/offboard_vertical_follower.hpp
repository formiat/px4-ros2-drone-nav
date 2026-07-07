#pragma once

#include "drone_city_nav/trajectory.hpp"

#include <limits>
#include <span>
#include <string>

namespace drone_city_nav {

struct VerticalFollowerConfig {
  double altitude_feedback_kp_1ps{0.5};
  double max_vertical_speed_mps{4.0};
  double max_vertical_accel_mps2{3.5};
  double max_vertical_jerk_mps3{10.0};
  double target_vz_feedforward_scale{1.0};
};

struct VerticalFollowerState {
  bool previous_command_valid{false};
  double previous_commanded_vz_mps{0.0};
  double previous_vertical_accel_mps2{0.0};
};

struct VerticalSetpointPlan {
  bool valid{false};
  bool trajectory_target_valid{false};
  bool passage_mode{false};
  double target_z_m{std::numeric_limits<double>::quiet_NaN()};
  double actual_z_m{std::numeric_limits<double>::quiet_NaN()};
  double z_error_m{std::numeric_limits<double>::quiet_NaN()};
  double vertical_slope_dz_ds{0.0};
  double scalar_speed_mps{0.0};
  double target_vz_mps{0.0};
  double feedback_vz_mps{0.0};
  double desired_vz_mps{0.0};
  double commanded_vz_mps{0.0};
  double commanded_vz_ned_mps{0.0};
  double vertical_accel_mps2{std::numeric_limits<double>::quiet_NaN()};
  double vertical_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
  bool vertical_constraint_active{false};
  std::string passage_id;
  std::string reason;
};

[[nodiscard]] VerticalFollowerConfig
sanitizeVerticalFollowerConfig(VerticalFollowerConfig config) noexcept;

[[nodiscard]] VerticalSetpointPlan
planVerticalSetpoint(std::span<const TrajectoryPointSample> trajectory_samples,
                     double trajectory_s_m, double scalar_speed_mps,
                     double current_altitude_m, bool altitude_valid, double dt_s,
                     const VerticalFollowerState& previous_state,
                     const VerticalFollowerConfig& config);

} // namespace drone_city_nav
