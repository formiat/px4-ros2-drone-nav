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
  double tracking_prediction_horizon_s{0.45};
  double max_lateral_control_angle_rad{0.9599310885968813};
  double max_lateral_control_rate_mps2{5.0};
  double velocity_lateral_response_accel_mps2{5.0};
  double curvature_feedforward_time_s{0.25};
  double max_curvature_feedforward_angle_rad{0.5235987755982988};
  double max_velocity_jerk_mps3{12.0};
  double max_lateral_velocity_jerk_mps3{14.0};
  double adaptive_lateral_response_scale_m{3.0};
  double adaptive_lateral_response_max_factor{1.2};
  double final_acceptance_radius_m{1.0};
  double final_hold_max_speed_mps{0.8};
};

} // namespace drone_city_nav
