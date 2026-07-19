#include "drone_city_nav/velocity_control_config.hpp"

#include <array>
#include <bit>
#include <cstdint>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void mix(std::uint64_t& hash, const std::uint64_t value) noexcept {
  hash ^= value;
  hash *= kFnvPrime;
}

void mixDouble(std::uint64_t& hash, const double value) noexcept {
  mix(hash, std::bit_cast<std::uint64_t>(value));
}

} // namespace

std::uint64_t speedProfileConstructionConfigFingerprint(
    const VelocityFollowerConfig& config) noexcept {
  std::uint64_t hash = kFnvOffsetBasis;
  const std::array<double, 14U> values{
      config.cruise_speed_mps,
      config.min_turn_speed_mps,
      config.known_passage_traversal_speed_limit_mps,
      config.speed_profile_accel_mps2,
      config.speed_profile_decel_mps2,
      config.turn_speed_lateral_accel_mps2,
      config.vertical_profile_max_climb_speed_mps,
      config.vertical_profile_max_descent_speed_mps,
      config.vertical_profile_max_vertical_accel_mps2,
      config.vertical_profile_max_vertical_jerk_mps3,
      config.vertical_profile_max_climb_angle_rad,
      config.speed_profile_sample_step_m,
      config.no_static_speed_policy.max_speed_mps,
      config.no_static_speed_policy.braking_decel_mps2,
  };
  for (const double value : values) {
    mixDouble(hash, value);
  }
  mix(hash, config.no_static_speed_policy.enabled ? 1U : 0U);
  return hash;
}

std::uint64_t
runtimeSpeedPolicyConfigFingerprint(const VelocityFollowerConfig& config) noexcept {
  std::uint64_t hash = kFnvOffsetBasis;
  const std::array<double, 15U> values{
      config.cruise_speed_mps,
      config.speed_profile_decel_mps2,
      config.setpoint_forward_accel_mps2,
      config.setpoint_forward_decel_mps2,
      config.speed_profile_lookahead_time_s,
      config.speed_profile_lookahead_min_m,
      config.speed_profile_lookahead_max_m,
      config.vertical_trackability_max_climb_speed_mps,
      config.vertical_trackability_max_descent_speed_mps,
      config.vertical_trackability_max_vertical_accel_mps2,
      config.vertical_trackability_altitude_tolerance_m,
      config.vertical_trackability_response_time_s,
      config.vertical_trackability_min_speed_mps,
      config.no_static_speed_policy.max_speed_mps,
      config.no_static_speed_policy.braking_decel_mps2,
  };
  for (const double value : values) {
    mixDouble(hash, value);
  }
  mix(hash, config.no_static_speed_policy.enabled ? 1U : 0U);
  return hash;
}

std::uint64_t
runtimeVelocityControlConfigFingerprint(const VelocityFollowerConfig& config) noexcept {
  std::uint64_t hash = kFnvOffsetBasis;
  const std::array<double, 36U> values{
      config.setpoint_forward_accel_mps2,
      config.setpoint_forward_decel_mps2,
      config.setpoint_lateral_response_accel_mps2,
      config.cross_track_gain,
      config.cross_track_derivative_gain,
      config.cross_track_p_gain_schedule_start_m,
      config.cross_track_p_gain_schedule_full_m,
      config.cross_track_p_gain_schedule_min_factor,
      config.cross_track_p_gain_schedule_max_factor,
      config.tracking_prediction_horizon_s,
      config.max_lateral_control_angle_rad,
      config.curvature_feedforward_time_s,
      config.curvature_feedforward_deadband_angle_rad,
      config.curvature_feedforward_full_angle_rad,
      config.max_curvature_feedforward_angle_rad,
      config.max_velocity_jerk_mps3,
      config.max_lateral_velocity_jerk_mps3,
      config.cross_track_d_gain_schedule_min_speed_mps,
      config.cross_track_d_gain_schedule_full_speed_mps,
      config.cross_track_d_gain_schedule_max_factor,
      config.control_tangent_smoothing_back_m,
      config.control_tangent_smoothing_forward_m,
      config.control_tangent_smoothing_max_heading_span_rad,
      config.control_tangent_smoothing_max_abs_curvature_1pm,
      config.control_curve_smoothing_back_m,
      config.control_curve_smoothing_forward_m,
      config.control_curve_smoothing_max_heading_span_rad,
      config.final_acceptance_radius_m,
      config.final_hold_max_speed_mps,
      config.terminal_capture_radius_m,
      config.terminal_capture_gain_1ps,
      config.terminal_capture_max_speed_mps,
      config.terminal_capture_decel_mps2,
      config.terminal_capture_braking_margin_m,
      config.terminal_position_capture_max_entry_speed_mps,
      config.terminal_stuck_speed_mps,
  };
  for (const double value : values) {
    mixDouble(hash, value);
  }
  return hash;
}

} // namespace drone_city_nav
