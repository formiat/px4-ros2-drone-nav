#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace drone_city_nav {

TEST(Px4OffboardConfig, DefaultYamlKeepsPlannerOwnedTrajectoryOptimizerParameters) {
  const std::string config_path =
      std::string{DRONE_CITY_NAV_SOURCE_DIR} + "/config/urban_mvp.yaml";
  std::ifstream stream{config_path};
  ASSERT_TRUE(stream.is_open()) << config_path;

  std::ostringstream buffer;
  buffer << stream.rdbuf();
  const std::string yaml = buffer.str();

  EXPECT_EQ(yaml.find("trajectory_optimizer_weight_traversal_time:"),
            std::string::npos);
  const std::string removed_length_weight_param =
      std::string{"trajectory_optimizer_weight_"} + "length";
  EXPECT_EQ(yaml.find(removed_length_weight_param), std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_preferred_min_radius_m: 24.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_weight_radius_shortfall: 40.0"),
            std::string::npos);
  EXPECT_EQ(yaml.find("trajectory_optimizer_weight_edge_margin:"), std::string::npos);
  EXPECT_EQ(yaml.find("trajectory_optimizer_desired_edge_margin_m:"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_regularization_iterations:"),
            std::string::npos);
  EXPECT_EQ(
      yaml.find("trajectory_optimizer_regularization_max_traversal_time_regression_s:"),
      std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_window_pre_margin_m: 25.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_window_post_margin_m: 25.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_window_heading_threshold_deg: 10.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_window_width_change_threshold_m: 2.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_window_min_heading_span_deg: 10.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_window_min_curvature_1pm: 0.01"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_window_min_width_asymmetry_m: 1.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_dp_offset_step_m: 1.5"), std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_dp_coarse_offset_step_m: 2.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_dp_fine_offset_step_m: 0.75"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_dp_fine_radius_m: 1.5"), std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_async_refinement_workers: 1"),
            std::string::npos);
  EXPECT_NE(yaml.find("turn_smoothing_trigger_heading_delta_deg: 37.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("turn_smoothing_trigger_min_radius_m: 16.0"), std::string::npos);
  EXPECT_NE(yaml.find("turn_smoothing_trigger_speed_limit_mps: 12.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("turn_smoothing_entry_distance_m: 45.0"), std::string::npos);
  EXPECT_NE(yaml.find("turn_smoothing_outer_bias_ratio: 0.45"), std::string::npos);
  EXPECT_NE(yaml.find("turn_smoothing_max_passes: 8"), std::string::npos);
  EXPECT_NE(yaml.find("speed_profile_decel_mps2: 2.0"), std::string::npos);
  EXPECT_NE(yaml.find("speed_profile_lookahead_time_s: 1.0"), std::string::npos);
  EXPECT_NE(yaml.find("speed_profile_lookahead_min_m: 5.0"), std::string::npos);
  EXPECT_NE(yaml.find("speed_profile_lookahead_max_m: 35.0"), std::string::npos);
  EXPECT_NE(yaml.find("tracking_prediction_horizon_s: 0.35"), std::string::npos);
  EXPECT_NE(yaml.find("trajectory_diagnostics_topic: "
                      "/drone_city_nav/trajectory_diagnostics"),
            std::string::npos);
  EXPECT_NE(yaml.find("cross_track_gain: 0.5"), std::string::npos);
  EXPECT_NE(yaml.find("cross_track_derivative_gain: 0.5"), std::string::npos);
  EXPECT_NE(yaml.find("cross_track_anti_overshoot_time_s: 1.0"), std::string::npos);
  EXPECT_NE(yaml.find("cross_track_anti_overshoot_min_feedback_scale: 0.25"),
            std::string::npos);
  EXPECT_NE(yaml.find("max_lateral_control_angle_deg: 55.0"), std::string::npos);
  EXPECT_NE(yaml.find("max_lateral_control_rate_mps2: 5.0"), std::string::npos);
  EXPECT_NE(yaml.find("velocity_lateral_response_accel_mps2: 8.0"), std::string::npos);
  EXPECT_EQ(yaml.find("cross_track_speed_guard_start_m:"), std::string::npos);
  EXPECT_EQ(yaml.find("cross_track_speed_guard_full_m:"), std::string::npos);
  EXPECT_EQ(yaml.find("cross_track_speed_guard_min_factor:"), std::string::npos);
  EXPECT_NE(yaml.find("curvature_feedforward_time_s: 0.25"), std::string::npos);
  EXPECT_NE(yaml.find("curvature_feedforward_deadband_angle_deg: 2.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("curvature_feedforward_full_angle_deg: 8.0"), std::string::npos);
  EXPECT_NE(yaml.find("max_curvature_feedforward_angle_deg: 30.0"), std::string::npos);
  EXPECT_NE(yaml.find("max_velocity_jerk_mps3: 12.0"), std::string::npos);
  EXPECT_NE(yaml.find("max_lateral_velocity_jerk_mps3: 22.0"), std::string::npos);
  EXPECT_NE(yaml.find("lateral_smoothing_min_speed_mps: 8.0"), std::string::npos);
  EXPECT_NE(yaml.find("lateral_smoothing_full_speed_mps: 20.0"), std::string::npos);
  EXPECT_NE(yaml.find("lateral_smoothing_max_factor: 1.0"), std::string::npos);
  EXPECT_NE(yaml.find("speed_aware_derivative_damping_min_speed_mps: 8.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("speed_aware_derivative_damping_full_speed_mps: 20.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("speed_aware_derivative_damping_max_factor: 1.5"),
            std::string::npos);
  EXPECT_NE(yaml.find("control_tangent_smoothing_back_m: 8.0"), std::string::npos);
  EXPECT_NE(yaml.find("control_tangent_smoothing_forward_m: 18.0"), std::string::npos);
  EXPECT_NE(yaml.find("control_tangent_smoothing_max_heading_span_deg: 12.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("control_tangent_smoothing_max_abs_curvature_1pm: 0.015"),
            std::string::npos);
  EXPECT_NE(yaml.find("control_curve_smoothing_back_m: 2.0"), std::string::npos);
  EXPECT_NE(yaml.find("control_curve_smoothing_forward_m: 6.0"), std::string::npos);
  EXPECT_NE(yaml.find("control_curve_smoothing_max_heading_span_deg: 45.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("adaptive_lateral_response_scale_m: 3.0"), std::string::npos);
  EXPECT_NE(yaml.find("adaptive_lateral_response_max_factor: 1.2"), std::string::npos);
  EXPECT_NE(yaml.find("terminal_capture_decel_mps2: 4.0"), std::string::npos);
  EXPECT_NE(yaml.find("terminal_capture_braking_margin_m: 2.0"), std::string::npos);
  EXPECT_NE(yaml.find("trajectory_update_max_start_cross_track_m: 8.0"),
            std::string::npos);
  EXPECT_EQ(yaml.find("max_cross_track_correction_angle_deg:"), std::string::npos);
  EXPECT_EQ(yaml.find("max_cross_track_correction_rate_mps2:"), std::string::npos);
  EXPECT_EQ(yaml.find("curvature_velocity_anticipation_time_s:"), std::string::npos);
  EXPECT_EQ(yaml.find("max_curvature_velocity_anticipation_angle_deg:"),
            std::string::npos);
  EXPECT_EQ(yaml.find("max_curvature_velocity_anticipation_rate_mps2:"),
            std::string::npos);
  EXPECT_EQ(yaml.find("max_feedforward_accel_mps2:"), std::string::npos);
  EXPECT_EQ(yaml.find("max_feedforward_jerk_mps3:"), std::string::npos);
  EXPECT_EQ(yaml.find("acceleration_feedforward_scale:"), std::string::npos);
  EXPECT_EQ(yaml.find("executable_trajectory_max_step_m:"), std::string::npos);
  EXPECT_EQ(yaml.find("trajectory_result_stale_cross_track_m:"), std::string::npos);
}

} // namespace drone_city_nav
