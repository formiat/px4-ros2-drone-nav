#pragma once

namespace drone_city_nav {

struct VelocityFollowerConfig {
  double cruise_speed_mps{12.0};
  double min_turn_speed_mps{2.0};
  double max_accel_mps2{3.0};
  double max_decel_mps2{4.0};
  double max_lateral_accel_mps2{3.0};
  double speed_profile_decel_mps2{2.0};
  double speed_profile_sample_step_m{1.0};
  double speed_profile_lookahead_time_s{1.0};
  double speed_profile_lookahead_min_m{5.0};
  double speed_profile_lookahead_max_m{35.0};
  double cross_track_gain{0.5};
  double cross_track_derivative_gain{0.5};
  double cross_track_progressive_feedback_start_m{0.3};
  double cross_track_progressive_feedback_full_m{2.5};
  double cross_track_progressive_feedback_min_factor{0.25};
  double cross_track_progressive_feedback_max_factor{1.3};
  double tracking_prediction_horizon_s{0.35};
  double max_lateral_control_angle_rad{0.9599310885968813};
  double velocity_lateral_response_accel_mps2{8.0};
  double curvature_feedforward_time_s{0.25};
  double curvature_feedforward_deadband_angle_rad{0.03490658503988659};
  double curvature_feedforward_full_angle_rad{0.13962634015954636};
  double max_curvature_feedforward_angle_rad{0.5235987755982988};
  double max_velocity_jerk_mps3{12.0};
  double max_lateral_velocity_jerk_mps3{22.0};
  double speed_aware_derivative_damping_min_speed_mps{8.0};
  double speed_aware_derivative_damping_full_speed_mps{20.0};
  double speed_aware_derivative_damping_max_factor{1.5};
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
};

} // namespace drone_city_nav
