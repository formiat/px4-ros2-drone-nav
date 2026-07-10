#pragma once

#include <cstdint>

namespace drone_city_nav {

struct VelocityFollowerConfig {
  double cruise_speed_mps{12.0};
  double min_turn_speed_mps{2.0};
  double speed_profile_accel_mps2{7.0};
  double speed_profile_decel_mps2{2.0};
  double turn_speed_lateral_accel_mps2{5.0};
  double vertical_profile_max_vertical_speed_mps{3.2};
  double vertical_profile_max_vertical_accel_mps2{3.0};
  double vertical_profile_max_vertical_jerk_mps3{9.0};
  double vertical_profile_max_climb_angle_rad{0.3490658503988659};
  double known_passage_traversal_speed_limit_mps{10.0};
  double vertical_trackability_max_vertical_speed_mps{4.0};
  double vertical_trackability_altitude_tolerance_m{0.4};
  double vertical_trackability_response_time_s{0.4};
  double vertical_trackability_min_speed_mps{1.0};
  double speed_profile_sample_step_m{1.0};
  double speed_profile_lookahead_time_s{1.0};
  double speed_profile_lookahead_min_m{5.0};
  double speed_profile_lookahead_max_m{35.0};
  double setpoint_forward_accel_mps2{7.0};
  double setpoint_forward_decel_mps2{20.0};
  double setpoint_lateral_response_accel_mps2{8.0};
  double cross_track_gain{0.5};
  double cross_track_derivative_gain{0.5};
  double cross_track_p_gain_schedule_start_m{0.0};
  double cross_track_p_gain_schedule_full_m{2.5};
  double cross_track_p_gain_schedule_min_factor{0.5};
  double cross_track_p_gain_schedule_max_factor{1.3};
  double tracking_prediction_horizon_s{0.35};
  double max_lateral_control_angle_rad{0.9599310885968813};
  double curvature_feedforward_time_s{0.25};
  double curvature_feedforward_deadband_angle_rad{0.03490658503988659};
  double curvature_feedforward_full_angle_rad{0.13962634015954636};
  double max_curvature_feedforward_angle_rad{0.5235987755982988};
  double max_velocity_jerk_mps3{12.0};
  double max_lateral_velocity_jerk_mps3{22.0};
  double cross_track_d_gain_schedule_min_speed_mps{8.0};
  double cross_track_d_gain_schedule_full_speed_mps{20.0};
  double cross_track_d_gain_schedule_max_factor{2.0};
  double control_tangent_smoothing_back_m{8.0};
  double control_tangent_smoothing_forward_m{18.0};
  double control_tangent_smoothing_max_heading_span_rad{0.20943951023931953};
  double control_tangent_smoothing_max_abs_curvature_1pm{0.015};
  double control_curve_smoothing_back_m{2.0};
  double control_curve_smoothing_forward_m{6.0};
  double control_curve_smoothing_max_heading_span_rad{0.7853981633974483};
  double final_acceptance_radius_m{1.0};
  double final_hold_max_speed_mps{0.8};
  double terminal_capture_radius_m{8.0};
  double terminal_capture_gain_1ps{1.0};
  double terminal_capture_max_speed_mps{8.0};
  double terminal_capture_decel_mps2{4.0};
  double terminal_capture_braking_margin_m{2.0};
  double terminal_position_capture_max_entry_speed_mps{3.0};
  double terminal_stuck_speed_mps{0.5};
};

[[nodiscard]] std::uint64_t speedProfileConstructionConfigFingerprint(
    const VelocityFollowerConfig& config) noexcept;

[[nodiscard]] std::uint64_t
runtimeSpeedPolicyConfigFingerprint(const VelocityFollowerConfig& config) noexcept;

[[nodiscard]] std::uint64_t
runtimeVelocityControlConfigFingerprint(const VelocityFollowerConfig& config) noexcept;

} // namespace drone_city_nav
