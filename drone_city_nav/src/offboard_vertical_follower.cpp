#include "drone_city_nav/offboard_vertical_follower.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr double kMinDtS = 0.001;
constexpr double kMaxDtS = 1.0;

[[nodiscard]] double boundedFiniteDouble(const double value, const double fallback,
                                         const double min_value,
                                         const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double sanitizedDtS(const double dt_s) noexcept {
  return boundedFiniteDouble(dt_s, 0.02, kMinDtS, kMaxDtS);
}

[[nodiscard]] double sanitizedScalarSpeedMps(const double scalar_speed_mps) noexcept {
  if (!std::isfinite(scalar_speed_mps)) {
    return 0.0;
  }
  return std::max(0.0, scalar_speed_mps);
}

[[nodiscard]] double
limitedVerticalAcceleration(const double desired_accel_mps2, const double dt_s,
                            const VerticalFollowerState& previous_state,
                            const VerticalFollowerConfig& config) noexcept {
  double accel_mps2 = std::clamp(desired_accel_mps2, -config.max_vertical_accel_mps2,
                                 config.max_vertical_accel_mps2);
  if (previous_state.previous_command_valid &&
      std::isfinite(previous_state.previous_vertical_accel_mps2)) {
    const double max_accel_delta = config.max_vertical_jerk_mps3 * dt_s;
    accel_mps2 = std::clamp(
        accel_mps2, previous_state.previous_vertical_accel_mps2 - max_accel_delta,
        previous_state.previous_vertical_accel_mps2 + max_accel_delta);
  }
  return accel_mps2;
}

} // namespace

VerticalFollowerConfig
sanitizeVerticalFollowerConfig(VerticalFollowerConfig config) noexcept {
  config.altitude_feedback_kp_1ps =
      boundedFiniteDouble(config.altitude_feedback_kp_1ps, 0.5, 0.0, 10.0);
  config.max_vertical_speed_mps =
      boundedFiniteDouble(config.max_vertical_speed_mps, 4.0, 0.0, 100.0);
  config.max_vertical_accel_mps2 =
      boundedFiniteDouble(config.max_vertical_accel_mps2, 3.5, 0.0, 100.0);
  config.max_vertical_jerk_mps3 =
      boundedFiniteDouble(config.max_vertical_jerk_mps3, 10.0, 0.0, 1000.0);
  config.target_vz_feedforward_scale =
      boundedFiniteDouble(config.target_vz_feedforward_scale, 1.0, 0.0, 10.0);
  return config;
}

VerticalSetpointPlan
planVerticalSetpoint(const std::span<const TrajectoryPointSample> trajectory_samples,
                     const double trajectory_s_m, const double scalar_speed_mps,
                     const double current_altitude_m, const bool altitude_valid,
                     const double dt_s, const VerticalFollowerState& previous_state,
                     const VerticalFollowerConfig& raw_config) {
  const VerticalFollowerConfig config = sanitizeVerticalFollowerConfig(raw_config);
  VerticalSetpointPlan plan{};
  plan.reason = "invalid";
  plan.actual_z_m = current_altitude_m;
  plan.scalar_speed_mps = sanitizedScalarSpeedMps(scalar_speed_mps);

  if (!altitude_valid || !std::isfinite(current_altitude_m)) {
    plan.reason = "invalid_altitude";
    return plan;
  }

  const TrajectoryVerticalTarget target =
      trajectoryVerticalTargetAtS(trajectory_samples, trajectory_s_m);
  if (!target.valid || !std::isfinite(target.z_m)) {
    plan.reason = "invalid_trajectory_target";
    plan.actual_z_m = current_altitude_m;
    return plan;
  }

  plan.valid = true;
  plan.trajectory_target_valid = true;
  plan.reason = "trajectory";
  plan.target_z_m = target.z_m;
  plan.z_error_m = plan.target_z_m - current_altitude_m;
  plan.vertical_slope_dz_ds = target.vertical_slope_dz_ds;
  plan.vertical_constraint_active = target.vertical_constraint_active;
  plan.passage_id = target.vertical_profile_passage_id;
  plan.passage_mode = plan.vertical_constraint_active || !plan.passage_id.empty();
  plan.target_vz_mps = target.vertical_slope_dz_ds * plan.scalar_speed_mps *
                       config.target_vz_feedforward_scale;

  if (!(config.max_vertical_speed_mps > 0.0)) {
    plan.reason = "vertical_speed_disabled";
    plan.feedback_vz_mps = 0.0;
    plan.desired_vz_mps = 0.0;
    plan.commanded_vz_mps = 0.0;
    plan.commanded_vz_ned_mps = -plan.commanded_vz_mps;
    return plan;
  }

  plan.feedback_vz_mps =
      std::clamp(plan.z_error_m * config.altitude_feedback_kp_1ps,
                 -config.max_vertical_speed_mps, config.max_vertical_speed_mps);
  plan.desired_vz_mps =
      std::clamp(plan.target_vz_mps + plan.feedback_vz_mps,
                 -config.max_vertical_speed_mps, config.max_vertical_speed_mps);

  if (!previous_state.previous_command_valid ||
      !std::isfinite(previous_state.previous_commanded_vz_mps) ||
      !(config.max_vertical_accel_mps2 > 0.0)) {
    plan.commanded_vz_mps = plan.desired_vz_mps;
    plan.commanded_vz_ned_mps = -plan.commanded_vz_mps;
    return plan;
  }

  const double limited_dt_s = sanitizedDtS(dt_s);
  const double desired_accel_mps2 =
      (plan.desired_vz_mps - previous_state.previous_commanded_vz_mps) / limited_dt_s;
  const double accel_mps2 =
      config.max_vertical_jerk_mps3 > 0.0
          ? limitedVerticalAcceleration(desired_accel_mps2, limited_dt_s,
                                        previous_state, config)
          : std::clamp(desired_accel_mps2, -config.max_vertical_accel_mps2,
                       config.max_vertical_accel_mps2);
  plan.commanded_vz_mps =
      previous_state.previous_commanded_vz_mps + accel_mps2 * limited_dt_s;
  plan.commanded_vz_mps =
      std::clamp(plan.commanded_vz_mps, -config.max_vertical_speed_mps,
                 config.max_vertical_speed_mps);
  plan.commanded_vz_ned_mps = -plan.commanded_vz_mps;
  plan.vertical_accel_mps2 =
      (plan.commanded_vz_mps - previous_state.previous_commanded_vz_mps) / limited_dt_s;
  if (std::isfinite(previous_state.previous_vertical_accel_mps2)) {
    plan.vertical_jerk_mps3 =
        (plan.vertical_accel_mps2 - previous_state.previous_vertical_accel_mps2) /
        limited_dt_s;
  }
  return plan;
}

} // namespace drone_city_nav
