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
  double cross_track_derivative_gain{0.8};
  double max_cross_track_correction_angle_rad{0.7853981633974483};
  double max_cross_track_correction_rate_mps2{4.0};
  double cross_track_speed_guard_start_m{2.0};
  double cross_track_speed_guard_full_m{6.0};
  double cross_track_speed_guard_min_factor{0.35};
  double max_feedforward_accel_mps2{3.0};
  double max_feedforward_jerk_mps3{12.0};
  double max_velocity_jerk_mps3{12.0};
  double acceleration_feedforward_scale{1.0};
  double final_acceptance_radius_m{1.0};
  double final_hold_max_speed_mps{0.8};
};

} // namespace drone_city_nav
