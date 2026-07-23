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
  EXPECT_NE(yaml.find("trajectory_optimizer_preferred_min_radius_m: 30.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_optimizer_weight_radius_shortfall: 70.0"),
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
  EXPECT_NE(yaml.find("known_passages_enabled: true"), std::string::npos);
  EXPECT_NE(yaml.find("known_passages_path: worlds/known_passages.passages3d"),
            std::string::npos);
  EXPECT_NE(yaml.find("known_passage_validation_enabled: true"), std::string::npos);
  EXPECT_NE(yaml.find("known_passage_validation_min_opening_overlap_m: 0.5"),
            std::string::npos);
  EXPECT_NE(yaml.find("known_passage_validation_min_opening_depth_fraction: 0.75"),
            std::string::npos);
  EXPECT_NE(yaml.find("known_passage_validation_clearance_margin_m: 0.5"),
            std::string::npos);
  EXPECT_NE(yaml.find("known_passage_validation_max_diagnostics: 8"),
            std::string::npos);
  EXPECT_NE(yaml.find("known_static_lidar_hit_closer_range_tolerance_m: 0.5"),
            std::string::npos);
  EXPECT_NE(yaml.find("known_static_lidar_hit_farther_range_tolerance_m: 1.5"),
            std::string::npos);
  EXPECT_NE(yaml.find("known_static_lidar_hit_endpoint_volume_tolerance_m: 0.75"),
            std::string::npos);
  EXPECT_NE(yaml.find("known_static_opening_boundary_tolerance_m: 0.50"),
            std::string::npos);
  EXPECT_NE(yaml.find("ambiguous_lidar_hit_required_independent_scans: 3"),
            std::string::npos);
  EXPECT_NE(yaml.find("ground_lidar_candidate_endpoint_altitude_tolerance_m: 1.5"),
            std::string::npos);
  EXPECT_NE(yaml.find("ground_lidar_attached_endpoint_altitude_tolerance_m: 0.3"),
            std::string::npos);
  EXPECT_NE(yaml.find("lidar_uncertain_hit_confirmation_enabled: true"),
            std::string::npos);
  EXPECT_NE(
      yaml.find("lidar_uncertain_unknown_require_source_timestamp_alignment: true"),
      std::string::npos);
  EXPECT_NE(yaml.find("lidar_uncertain_unknown_reliable_range_margin_m: 0.5"),
            std::string::npos);
  EXPECT_EQ(yaml.find("passage_traversal_sensor_policy"), std::string::npos);
  EXPECT_NE(yaml.find("vertical_profile_enabled: true"), std::string::npos);
  EXPECT_NE(yaml.find("vertical_profile_gate_clearance_margin_m: 0.5"),
            std::string::npos);
  EXPECT_NE(yaml.find("vertical_profile_preferred_gate_clearance_margin_m: 2.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("vertical_profile_max_climb_speed_mps: 3.2"), std::string::npos);
  EXPECT_NE(yaml.find("vertical_profile_max_descent_speed_mps: 3.2"),
            std::string::npos);
  EXPECT_NE(yaml.find("vertical_profile_max_vertical_accel_mps2: 3.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("vertical_profile_max_vertical_jerk_mps3: 9.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("vertical_profile_max_climb_angle_deg: 35.0"), std::string::npos);
  EXPECT_NE(yaml.find("vertical_profile_pre_gate_hold_time_s: 1.0"), std::string::npos);
  EXPECT_NE(yaml.find("vertical_profile_pre_gate_hold_min_distance_m: 15.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("vertical_profile_pre_gate_hold_max_distance_m: 80.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("passage_insertion_repair_clearance_margin_m: 2.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("known_passage_traversal_speed_limit_mps: 10.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("no_static_passage_speed_limit_mps: 5.0"), std::string::npos);
  EXPECT_NE(yaml.find("passage_insertion_enabled: true"), std::string::npos);
  EXPECT_NE(yaml.find("passage_insertion_sample_step_m: 1.0"), std::string::npos);
  EXPECT_NE(yaml.find("passage_insertion_min_anchor_margin_m: 8.0"), std::string::npos);
  EXPECT_NE(yaml.find("passage_insertion_max_anchor_margin_m: 60.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("passage_insertion_max_candidates: 24"), std::string::npos);
  EXPECT_NE(yaml.find("passage_insertion_max_diagnostics: 8"), std::string::npos);
  EXPECT_NE(yaml.find("altitude_feedback_kp_1ps: 0.5"), std::string::npos);
  EXPECT_NE(yaml.find("vertical_trackability_altitude_tolerance_m: 0.4"),
            std::string::npos);
  EXPECT_NE(yaml.find("vertical_trackability_response_time_s: 0.4"), std::string::npos);
  EXPECT_NE(yaml.find("vertical_trackability_min_speed_mps: 1.0"), std::string::npos);
  EXPECT_NE(yaml.find("trajectory_handover_enabled: true"), std::string::npos);
  EXPECT_NE(yaml.find("safe_trajectory_truncation_enabled: true"), std::string::npos);
  EXPECT_NE(yaml.find("safe_trajectory_truncation_margin_m: 15.0"), std::string::npos);
  EXPECT_NE(yaml.find("safe_trajectory_terminal_raw_clearance_m: 5.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_handover_prefix_time_s: 0.6"), std::string::npos);
  EXPECT_NE(yaml.find("trajectory_handover_hard_window_exit_settle_distance_m: 3.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_handover_max_join_distance_m: 15.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("trajectory_continuity_defer_tangent_jump_deg: 30.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("vertical_setpoint_max_climb_speed_mps: 4.0"), std::string::npos);
  EXPECT_NE(yaml.find("vertical_setpoint_max_descent_speed_mps: 4.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("vertical_setpoint_max_accel_mps2: 3.5"), std::string::npos);
  EXPECT_NE(yaml.find("vertical_setpoint_max_jerk_mps3: 10.0"), std::string::npos);
  EXPECT_NE(yaml.find("vertical_target_vz_feedforward_scale: 1.0"), std::string::npos);
  EXPECT_NE(yaml.find("known_passage_markers_topic: "
                      "/drone_city_nav/known_passage_markers"),
            std::string::npos);
  EXPECT_NE(yaml.find("static_building_markers_topic: "
                      "/drone_city_nav/static_building_markers"),
            std::string::npos);
  EXPECT_NE(yaml.find("known_passage_debug_publish_period_s: 1.0"), std::string::npos);
  EXPECT_NE(yaml.find("cross_track_gain: 0.5"), std::string::npos);
  EXPECT_NE(yaml.find("cross_track_derivative_gain: 0.5"), std::string::npos);
  EXPECT_NE(yaml.find("cross_track_p_gain_schedule_start_m: 0.0"), std::string::npos);
  EXPECT_NE(yaml.find("cross_track_p_gain_schedule_full_m: 2.5"), std::string::npos);
  EXPECT_NE(yaml.find("cross_track_p_gain_schedule_min_factor: 0.5"),
            std::string::npos);
  EXPECT_NE(yaml.find("cross_track_p_gain_schedule_max_factor: 1.3"),
            std::string::npos);
  EXPECT_NE(yaml.find("max_lateral_control_angle_deg: 55.0"), std::string::npos);
  EXPECT_NE(yaml.find("setpoint_lateral_response_accel_mps2: 8.0"), std::string::npos);
  EXPECT_EQ(yaml.find("cross_track_anti_overshoot_time_s:"), std::string::npos);
  EXPECT_EQ(yaml.find("cross_track_anti_overshoot_min_feedback_scale:"),
            std::string::npos);
  EXPECT_EQ(yaml.find("max_lateral_control_rate_mps2:"), std::string::npos);
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
  EXPECT_EQ(yaml.find("lateral_smoothing_min_speed_mps:"), std::string::npos);
  EXPECT_EQ(yaml.find("lateral_smoothing_full_speed_mps:"), std::string::npos);
  EXPECT_EQ(yaml.find("lateral_smoothing_max_factor:"), std::string::npos);
  EXPECT_NE(yaml.find("cross_track_d_gain_schedule_min_speed_mps: 8.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("cross_track_d_gain_schedule_full_speed_mps: 20.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("cross_track_d_gain_schedule_max_factor: 2.0"),
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
  EXPECT_EQ(yaml.find("adaptive_lateral_response_scale_m:"), std::string::npos);
  EXPECT_EQ(yaml.find("adaptive_lateral_response_max_factor:"), std::string::npos);
  EXPECT_NE(yaml.find("terminal_capture_decel_mps2: 4.0"), std::string::npos);
  EXPECT_NE(yaml.find("terminal_capture_braking_margin_m: 2.0"), std::string::npos);
  EXPECT_NE(yaml.find("terminal_position_capture_max_entry_speed_mps: 3.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("terminal_stuck_speed_mps: 0.5"), std::string::npos);
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
