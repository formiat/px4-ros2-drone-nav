#include "drone_city_nav/offboard_blackbox.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <sstream>
#include <string>
#include <string_view>

namespace drone_city_nav {
namespace {

void expectJsonField(const std::string& json, const std::string_view needle) {
  EXPECT_NE(json.find(needle), std::string::npos) << "missing field: " << needle;
}

} // namespace

TEST(OffboardBlackbox, WritesJsonPrimitives) {
  std::ostringstream stream;

  writeBlackboxJsonBool(stream, true);
  stream << ",";
  writeBlackboxJsonBool(stream, false);
  stream << ",";
  writeBlackboxJsonNumberOrNull(stream, 4.25);
  stream << ",";
  writeBlackboxJsonNumberOrNull(stream, std::numeric_limits<double>::quiet_NaN());
  stream << ",";
  writeBlackboxStringField(stream, "mode", "velocity_cruise");

  EXPECT_EQ(stream.str(), "true,false,4.25,null,\"mode\":\"velocity_cruise\"");
}

TEST(OffboardBlackbox, WritesPathIdContract) {
  std::ostringstream stream;

  writeBlackboxPathId(stream, OffboardBlackboxPathId{7U, 42U, true, 123456U});

  EXPECT_EQ(stream.str(), "\"path_id\":{\"local_update\":7,\"planner\":42,"
                          "\"planner_seen\":true,\"stamp_ns\":123456}");
}

TEST(OffboardBlackbox, WritesFullRecordJsonLine) {
  OffboardBlackboxRecord record;
  record.time_ns = 123456789;
  record.path_id = OffboardBlackboxPathId{7U, 42U, true, 987U};
  record.pose_fresh = true;
  record.pose_age_s = 0.25;
  record.current_position = Point2{1.0, 2.0};
  record.current_altitude_m = 12.0;
  record.current_heading_rad = 0.5;
  record.attitude_valid = true;
  record.attitude_age_s = 0.1;
  record.current_attitude = AttitudeEuler{0.1, 0.2, 0.3};
  record.current_velocity_valid = true;
  record.current_velocity = Point2{3.0, 4.0};
  record.current_speed_mps = 5.0;
  record.target = Point2{10.0, 11.0};
  record.target_distance_m = 6.0;
  record.last_commanded_target_delta_m = 0.4;
  record.last_commanded_yaw_rad = 0.9;
  record.control_mode = "velocity";
  record.last_velocity_setpoint = Point2{8.0, 9.0};
  record.last_vertical_velocity_setpoint_mps = -0.5;
  record.last_velocity_setpoint_speed_mps = 12.0;
  record.velocity_plan.valid = true;
  record.velocity_plan.reason = VelocitySetpointReason::kTrajectorySpeedProfile;
  record.velocity_plan.final_command_speed_mps = 11.0;
  record.velocity_plan.profile_speed_limit_mps = 13.0;
  record.velocity_plan.limiting_constraint_type = SpeedConstraintType::kArc;
  record.velocity_plan.trajectory_segment_kind = TrajectorySegmentKind::kArc;
  record.velocity_plan.trajectory_curvature_1pm = 0.1;
  record.velocity_plan.trajectory_arc_radius_m = 10.0;
  record.velocity_plan.control_tangent_smoothed = true;
  record.velocity_plan.control_tangent_raw = Point2{0.9, 0.1};
  record.velocity_plan.control_tangent_smoothing_heading_span_rad = 0.2;
  record.velocity_plan.control_tangent_smoothing_max_abs_curvature_1pm = 0.01;
  record.velocity_plan.control_tangent_smoothing_window_start_s_m = 12.0;
  record.velocity_plan.control_tangent_smoothing_window_end_s_m = 30.0;
  record.velocity_plan.cross_track_derivative_damping_factor = 1.25;
  record.velocity_plan.cross_track_derivative_gain_effective = 0.75;
  record.velocity_plan.curvature_feedforward_raw_angle_rad = 0.2;
  record.velocity_plan.curvature_feedforward_scale = 0.5;
  record.velocity_plan.adaptive_lateral_response_factor = 2.5;
  record.velocity_plan.cross_track_feedback_scale = 0.625;
  record.velocity_plan.cross_track_closing_speed_target_mps = 5.0;
  record.velocity_plan.terminal_capture_active = true;
  record.velocity_plan.terminal_goal_distance_m = 3.5;
  record.velocity_plan.terminal_signed_along_track_distance_m = 3.25;
  record.velocity_plan.terminal_remaining_trajectory_distance_m = 6.5;
  record.velocity_plan.terminal_acceptance_radius_m = 1.0;
  record.velocity_plan.terminal_hold_max_speed_mps = 0.8;
  record.velocity_plan.terminal_hold_distance_met = false;
  record.velocity_plan.terminal_hold_speed_met = true;
  record.velocity_plan.terminal_capture_goal_distance_triggered = true;
  record.velocity_plan.terminal_capture_remaining_distance_triggered = false;
  record.velocity_plan.terminal_capture_gain_speed_limit_mps = 3.5;
  record.velocity_plan.terminal_capture_max_speed_mps = 8.0;
  record.velocity_plan.terminal_capture_decel_mps2 = 4.0;
  record.velocity_plan.terminal_capture_braking_margin_m = 2.0;
  record.velocity_plan.terminal_capture_braking_distance_m = 8.0;
  record.velocity_plan.terminal_capture_activation_distance_m = 10.0;
  record.velocity_plan.terminal_capture_braking_speed_limit_mps = 4.5;
  record.velocity_plan.terminal_capture_speed_limit_mps = 2.75;
  record.velocity_plan.desired_to_setpoint_tangent_error_mps = 0.4;
  record.velocity_plan.desired_to_setpoint_normal_error_mps = -0.3;
  record.velocity_plan.setpoint_to_actual_tangent_error_mps = 0.2;
  record.velocity_plan.setpoint_to_actual_normal_error_mps = -0.1;
  record.velocity_plan.desired_to_actual_tangent_error_mps = 0.6;
  record.velocity_plan.desired_to_actual_normal_error_mps = -0.4;
  record.velocity_plan.path_frame_lateral_smoothing_applied = true;
  record.velocity_plan.lateral_smoothing_factor = 1.6;
  record.velocity_plan.smoother_lateral_response_accel_mps2 = 3.125;
  record.velocity_smoother_reset_reason = "path_update";
  record.path_update_velocity_smoother_reset_count = 3U;
  record.last_altitude_error_m = 0.2;
  record.trajectory_valid = true;
  record.trajectory_metrics.length_m = 100.0;
  record.trajectory_metrics.line_segments = 2U;
  record.trajectory_metrics.arc_segments = 1U;
  record.final_trajectory_samples = 25U;
  record.trajectory_planner_stats.samples = 25U;
  record.trajectory_planner_stats.status = TrajectoryPlannerStatus::kOk;
  record.trajectory_planner_stats.quality = TrajectoryQuality::kRefined;
  record.trajectory_planner_stats.corridor.samples = 12U;
  record.trajectory_planner_stats.corridor.min_width_m = 5.0;
  record.trajectory_planner_stats.corridor.parallel_workers_used = 3U;
  record.trajectory_planner_stats.corridor.sample_build_duration_ms = 4.5;
  record.trajectory_planner_stats.corridor.clearance_field_reused = true;
  record.trajectory_planner_stats.trajectory_optimizer.iterations = 4U;
  record.trajectory_planner_stats.trajectory_optimizer.optimizer_samples = 8U;
  record.trajectory_planner_stats.trajectory_optimizer.final_cost = 1.5;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_point_build_duration_ms = 1.25;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_cost_breakdown_duration_ms = 2.25;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_shape_diagnostics_duration_ms = 3.25;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_speed_profile_duration_ms = 4.25;
  record.trajectory_planner_stats.trajectory_optimizer.candidate_speed_profile_calls =
      6U;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_speed_profile_samples_total = 240U;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_speed_profile_samples_max = 44U;
  record.trajectory_planner_stats.trajectory_optimizer.scratch_reused_candidates = 7U;
  record.trajectory_planner_stats.trajectory_optimizer
      .parallel_candidate_evaluation_used = true;
  record.trajectory_planner_stats.trajectory_optimizer.parallel_workers_used = 2U;
  record.trajectory_planner_stats.trajectory_optimizer.candidate_chunks = 9U;
  record.trajectory_planner_stats.trajectory_optimizer.candidate_parallel_batches = 8U;
  record.trajectory_planner_stats.trajectory_optimizer.candidate_threads_launched = 16U;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_batch_wall_duration_ms = 3.5;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_batch_wait_duration_ms = 3.0;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_worker_buffer_prepare_duration_ms = 0.4;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_thread_launch_duration_ms = 0.6;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_thread_join_wait_duration_ms = 2.5;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_offset_changed_samples_total = 72U;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_offset_changed_samples_max = 7U;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_offset_changed_span_samples_total = 90U;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_offset_changed_span_samples_max = 9U;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_local_speed_window_samples_total = 420U;
  record.trajectory_planner_stats.trajectory_optimizer
      .candidate_local_speed_window_samples_max = 35U;
  record.trajectory_planner_stats.trajectory_optimizer.local_candidate_evaluations =
      18U;
  record.trajectory_planner_stats.trajectory_optimizer
      .local_candidate_full_score_fallbacks = 15U;
  record.trajectory_planner_stats.trajectory_optimizer
      .local_candidate_full_score_required = 5U;
  record.trajectory_planner_stats.trajectory_optimizer
      .local_candidate_full_score_required_invalid_input = 1U;
  record.trajectory_planner_stats.trajectory_optimizer
      .local_candidate_full_score_required_boundary = 2U;
  record.trajectory_planner_stats.trajectory_optimizer
      .local_candidate_full_score_required_unsafe_base = 1U;
  record.trajectory_planner_stats.trajectory_optimizer
      .local_candidate_full_score_required_window_invalid = 1U;
  record.trajectory_planner_stats.trajectory_optimizer
      .local_candidate_acceptance_full_scores = 3U;
  record.trajectory_planner_stats.trajectory_optimizer
      .local_candidate_point_build_duration_ms = 0.75;
  record.trajectory_planner_stats.trajectory_optimizer
      .local_candidate_path_evaluation_duration_ms = 0.85;
  record.trajectory_planner_stats.trajectory_optimizer
      .local_candidate_traversal_estimate_duration_ms = 0.95;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_lower_bound_validation_full_scores = 10U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_lower_bound_validation_full_score_duration_ms = 1.5;
  record.trajectory_planner_stats.trajectory_optimizer.shadow_lower_bound_evaluations =
      12U;
  record.trajectory_planner_stats.trajectory_optimizer.shadow_lower_bound_unavailable =
      6U;
  record.trajectory_planner_stats.trajectory_optimizer.shadow_lower_bound_prunable = 5U;
  record.trajectory_planner_stats.trajectory_optimizer.shadow_lower_bound_false_prunes =
      1U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_lower_bound_winner_prunes = 1U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_lower_bound_prunable_full_score_duration_ms = 2.5;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_lower_bound_max_overestimate_score = 0.125;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_lower_bound_max_underestimate_score = 4.5;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_lower_bound_max_false_prune_improvement_score = 0.75;
  record.trajectory_planner_stats.trajectory_optimizer.shadow_local_speed_evaluations =
      14U;
  record.trajectory_planner_stats.trajectory_optimizer.shadow_local_speed_unavailable =
      4U;
  record.trajectory_planner_stats.trajectory_optimizer.shadow_local_speed_prunable = 3U;
  record.trajectory_planner_stats.trajectory_optimizer.shadow_local_speed_false_prunes =
      1U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_local_speed_winner_mismatches = 2U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_local_speed_abs_time_error_sum_s = 1.25;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_local_speed_abs_time_error_p95_s = 0.45;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_local_speed_max_time_overestimate_s = 0.5;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_local_speed_max_time_underestimate_s = 0.75;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_local_speed_abs_score_error_sum = 50.0;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_local_speed_abs_score_error_p95 = 18.0;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_local_speed_max_score_overestimate = 20.0;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_local_speed_max_score_underestimate = 30.0;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_local_speed_max_false_prune_improvement_score = 6.5;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_segment_score_evaluations = 13U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_segment_score_unavailable = 5U;
  record.trajectory_planner_stats.trajectory_optimizer.shadow_segment_score_prunable =
      4U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_segment_score_false_prunes = 0U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_segment_score_winner_mismatches = 1U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_segment_score_window_samples_total = 99U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_segment_score_window_samples_max = 11U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_segment_score_abs_error_sum = 0.25;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_segment_score_abs_error_p95 = 0.1;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_segment_score_max_overestimate = 0.15;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_segment_score_max_underestimate = 0.2;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_segment_score_max_false_prune_improvement_score = 0.0;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_boundary_clamped_local_candidates = 8U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_boundary_clamped_window_samples_total = 64U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_boundary_clamped_window_samples_max = 9U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_speed_profile_cache_queries = 15U;
  record.trajectory_planner_stats.trajectory_optimizer.shadow_speed_profile_cache_hits =
      2U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_speed_profile_cache_unique = 13U;
  record.trajectory_planner_stats.trajectory_optimizer.dp_segment_cache_misses = 92U;
  record.trajectory_planner_stats.trajectory_optimizer.candidate_segment_cache_misses =
      40U;
  record.trajectory_planner_stats.trajectory_optimizer.full_path_segment_cache_hits =
      11U;
  record.trajectory_planner_stats.trajectory_optimizer.full_path_segment_cache_misses =
      12U;
  record.trajectory_planner_stats.trajectory_optimizer.window_count = 2U;
  record.trajectory_planner_stats.trajectory_optimizer.active_window_count = 1U;
  record.trajectory_planner_stats.trajectory_optimizer
      .active_window_centerline_blocked = 1U;
  record.trajectory_planner_stats.trajectory_optimizer
      .active_window_heading_change_samples = 2U;
  record.trajectory_planner_stats.trajectory_optimizer
      .active_window_heading_span_samples = 3U;
  record.trajectory_planner_stats.trajectory_optimizer.active_window_curvature_samples =
      4U;
  record.trajectory_planner_stats.trajectory_optimizer
      .active_window_width_change_samples = 5U;
  record.trajectory_planner_stats.trajectory_optimizer
      .active_window_width_asymmetry_samples = 6U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_active_window_no_width_asymmetry_count = 2U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_active_window_no_width_asymmetry_samples = 15U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_active_window_no_width_triggers_count = 3U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_active_window_no_width_triggers_samples = 12U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_active_window_no_heading_span_count = 4U;
  record.trajectory_planner_stats.trajectory_optimizer
      .shadow_active_window_no_heading_span_samples = 14U;
  record.trajectory_planner_stats.trajectory_optimizer.centerline_blocked_windows = 3U;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_window_samples = 17U;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_window_merged_count = 2U;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_prohibited_cells = 7U;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_outside_grid_segments = 8U;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_segment_count = 2U;
  record.trajectory_planner_stats.trajectory_optimizer.centerline_blocked_span_count =
      1U;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_first_segment_index = 9U;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_last_segment_index = 10U;
  record.trajectory_planner_stats.trajectory_optimizer.centerline_blocked_first_s_m =
      12.5;
  record.trajectory_planner_stats.trajectory_optimizer.centerline_blocked_last_s_m =
      16.75;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_span_length_m = 4.25;
  record.trajectory_planner_stats.trajectory_optimizer.centerline_blocked_first_x_m =
      3.25;
  record.trajectory_planner_stats.trajectory_optimizer.centerline_blocked_first_y_m =
      -4.5;
  record.trajectory_planner_stats.trajectory_optimizer.centerline_blocked_last_x_m =
      5.5;
  record.trajectory_planner_stats.trajectory_optimizer.centerline_blocked_last_y_m =
      -6.75;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_first_outside_grid = true;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_last_outside_grid = true;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_span_diagnostic_count = 1U;
  record.trajectory_planner_stats.trajectory_optimizer
      .centerline_blocked_span_diagnostics[0] =
      TrajectoryOptimizerBlockedSpanDiagnostic{.begin_segment_index = 9U,
                                               .end_segment_index = 10U,
                                               .begin_s_m = 12.5,
                                               .end_s_m = 16.75,
                                               .length_m = 4.25,
                                               .begin_x_m = 3.25,
                                               .begin_y_m = -4.5,
                                               .end_x_m = 5.5,
                                               .end_y_m = -6.75,
                                               .prohibited_cells = 7U,
                                               .outside_grid_segments = 8U};
  record.trajectory_planner_stats.trajectory_optimizer.dp_states = 24U;
  record.trajectory_planner_stats.trajectory_optimizer.dp_transitions = 96U;
  record.trajectory_planner_stats.trajectory_optimizer.dp_coarse_states = 8U;
  record.trajectory_planner_stats.trajectory_optimizer.dp_coarse_transitions = 20U;
  record.trajectory_planner_stats.trajectory_optimizer.dp_fine_states = 16U;
  record.trajectory_planner_stats.trajectory_optimizer.dp_fine_transitions = 76U;
  record.trajectory_planner_stats.trajectory_optimizer.dp_coarse_to_fine_used = true;
  record.trajectory_planner_stats.trajectory_optimizer.async_refined = true;
  record.trajectory_shape_diagnostics.segment_count = 24U;
  record.trajectory_shape_diagnostics.max_heading_delta_rad = 0.3;
  record.path_valid = true;
  record.waypoint_index = 1U;
  record.waypoint_count = 25U;
  record.path_goal_distance_m = 7.0;
  record.mission_goal_distance_m = 8.0;
  record.upcoming_turn.valid = true;
  record.upcoming_turn.waypoint_index = 2U;
  record.upcoming_turn.distance_to_turn_m = 14.0;
  record.upcoming_turn.angle_rad = 1.0;
  record.upcoming_turn.turn_point = Point2{15.0, 16.0};
  record.final_trajectory_segment_type = "turn";
  record.path_tracking.valid = true;
  record.path_tracking.cross_track_error_m = 0.7;
  record.path_tracking.projection = Point2{3.0, 4.0};
  record.motion_phase = "cruise";
  record.final_goal_hold_active = false;
  record.terminal_position_capture_active = true;
  record.terminal_position_capture_reason = "terminal_stuck";
  record.terminal_position_capture_goal_distance_m = 7.0;
  record.terminal_position_capture_remaining_s_m = 2.0;
  record.terminal_position_capture_speed_mps = 0.4;
  record.terminal_position_capture_activation_radius_m = 8.0;
  record.terminal_position_capture_max_entry_speed_mps = 3.0;
  record.terminal_position_capture_stuck_speed_mps = 0.5;
  record.prohibited_grid_clearance_m = 9.0;
  record.nearest_prohibited_cell.valid = true;
  record.nearest_prohibited_cell.clearance_m = 4.0;
  record.nearest_prohibited_cell.point = Point2{20.0, 21.0};

  std::ostringstream stream;
  writeOffboardBlackboxRecord(stream, record);
  const std::string json = stream.str();

  ASSERT_FALSE(json.empty());
  EXPECT_EQ(json.front(), '{');
  EXPECT_EQ(json.back(), '\n');
  expectJsonField(json, "\"time_ns\":123456789");
  expectJsonField(json, "\"path_id\":{\"local_update\":7,\"planner\":42");
  expectJsonField(json, "\"pose\":{\"fresh\":true");
  expectJsonField(json, "\"x\":1");
  expectJsonField(json, "\"attitude\":{\"valid\":true");
  expectJsonField(json, "\"velocity\":{\"valid\":true");
  expectJsonField(json, "\"target\":{\"x\":10");
  expectJsonField(json, "\"velocity_command\":{\"control_mode\":\"velocity\"");
  expectJsonField(json, "\"control_tangent_smoothed\":true");
  expectJsonField(json, "\"control_tangent_raw_x\":0.9");
  expectJsonField(json, "\"control_tangent_smoothing_heading_span_rad\":0.2");
  expectJsonField(json, "\"control_tangent_smoothing_max_abs_curvature_1pm\":0.01");
  expectJsonField(json, "\"control_tangent_smoothing_window_start_s_m\":12");
  expectJsonField(json, "\"control_tangent_smoothing_window_end_s_m\":30");
  expectJsonField(json, "\"cross_track_derivative_damping_factor\":1.25");
  expectJsonField(json, "\"cross_track_derivative_gain_effective\":0.75");
  expectJsonField(json, "\"curvature_feedforward_raw_angle_rad\":0.2");
  expectJsonField(json, "\"curvature_feedforward_scale\":0.5");
  expectJsonField(json, "\"adaptive_lateral_response_factor\":2.5");
  expectJsonField(json, "\"cross_track_feedback_scale\":0.625");
  expectJsonField(json, "\"cross_track_closing_speed_target_mps\":5");
  expectJsonField(json, "\"path_frame_lateral_smoothing_applied\":true");
  expectJsonField(json, "\"lateral_smoothing_factor\":1.6");
  expectJsonField(json, "\"smoother_lateral_response_accel_mps2\":3.125");
  expectJsonField(json, "\"speed_limit_reason\":\"trajectory_profile\"");
  expectJsonField(json, "\"terminal_capture_active\":true");
  expectJsonField(json, "\"terminal_goal_distance_m\":3.5");
  expectJsonField(json, "\"terminal_signed_along_track_distance_m\":3.25");
  expectJsonField(json, "\"terminal_remaining_trajectory_distance_m\":6.5");
  expectJsonField(json, "\"terminal_acceptance_radius_m\":1");
  expectJsonField(json, "\"terminal_hold_max_speed_mps\":0.8");
  expectJsonField(json, "\"terminal_hold_distance_met\":false");
  expectJsonField(json, "\"terminal_hold_speed_met\":true");
  expectJsonField(json, "\"terminal_capture_goal_distance_triggered\":true");
  expectJsonField(json, "\"terminal_capture_remaining_distance_triggered\":false");
  expectJsonField(json, "\"terminal_capture_gain_speed_limit_mps\":3.5");
  expectJsonField(json, "\"terminal_capture_max_speed_mps\":8");
  expectJsonField(json, "\"terminal_capture_decel_mps2\":4");
  expectJsonField(json, "\"terminal_capture_braking_margin_m\":2");
  expectJsonField(json, "\"terminal_capture_braking_distance_m\":8");
  expectJsonField(json, "\"terminal_capture_activation_distance_m\":10");
  expectJsonField(json, "\"terminal_capture_braking_speed_limit_mps\":4.5");
  expectJsonField(json, "\"terminal_capture_speed_limit_mps\":2.75");
  expectJsonField(json, "\"desired_to_setpoint_tangent_error_mps\":0.4");
  expectJsonField(json, "\"desired_to_setpoint_normal_error_mps\":-0.3");
  expectJsonField(json, "\"setpoint_to_actual_tangent_error_mps\":0.2");
  expectJsonField(json, "\"setpoint_to_actual_normal_error_mps\":-0.1");
  expectJsonField(json, "\"desired_to_actual_tangent_error_mps\":0.6");
  expectJsonField(json, "\"desired_to_actual_normal_error_mps\":-0.4");
  expectJsonField(json, "\"trajectory_segment_type\":\"arc\"");
  expectJsonField(json, "\"trajectory_planner_status\":\"none\"");
  expectJsonField(json, "\"trajectory_quality\":\"refined\"");
  expectJsonField(json, "\"corridor_samples\":12");
  expectJsonField(json, "\"corridor_parallel_workers_used\":3");
  expectJsonField(json, "\"corridor_sample_build_duration_ms\":4.5");
  expectJsonField(json, "\"clearance_field_reused_by_corridor\":true");
  expectJsonField(json, "\"trajectory_optimizer_iterations\":4");
  expectJsonField(json,
                  "\"trajectory_optimizer_candidate_point_build_duration_ms\":1.25");
  expectJsonField(json,
                  "\"trajectory_optimizer_candidate_cost_breakdown_duration_ms\":2.25");
  expectJsonField(
      json, "\"trajectory_optimizer_candidate_shape_diagnostics_duration_ms\":3.25");
  expectJsonField(json,
                  "\"trajectory_optimizer_candidate_speed_profile_duration_ms\":4.25");
  expectJsonField(json, "\"trajectory_optimizer_candidate_speed_profile_calls\":6");
  expectJsonField(json,
                  "\"trajectory_optimizer_candidate_speed_profile_samples_total\":240");
  expectJsonField(json,
                  "\"trajectory_optimizer_candidate_speed_profile_samples_max\":44");
  expectJsonField(json, "\"trajectory_optimizer_scratch_reused_candidates\":7");
  expectJsonField(json,
                  "\"trajectory_optimizer_parallel_candidate_evaluation_used\":true");
  expectJsonField(json, "\"trajectory_optimizer_parallel_workers_used\":2");
  expectJsonField(json, "\"trajectory_optimizer_candidate_chunks\":9");
  expectJsonField(json, "\"trajectory_optimizer_candidate_parallel_batches\":8");
  expectJsonField(json, "\"trajectory_optimizer_candidate_threads_launched\":16");
  expectJsonField(json,
                  "\"trajectory_optimizer_candidate_batch_wall_duration_ms\":3.5");
  expectJsonField(json, "\"trajectory_optimizer_candidate_batch_wait_duration_ms\":3");
  expectJsonField(
      json, "\"trajectory_optimizer_candidate_worker_buffer_prepare_duration_ms\":0.4");
  expectJsonField(json,
                  "\"trajectory_optimizer_candidate_thread_launch_duration_ms\":0.6");
  expectJsonField(
      json, "\"trajectory_optimizer_candidate_thread_join_wait_duration_ms\":2.5");
  expectJsonField(json,
                  "\"trajectory_optimizer_candidate_offset_changed_samples_total\":72");
  expectJsonField(json,
                  "\"trajectory_optimizer_candidate_offset_changed_samples_max\":7");
  expectJsonField(
      json, "\"trajectory_optimizer_candidate_offset_changed_span_samples_total\":90");
  expectJsonField(
      json, "\"trajectory_optimizer_candidate_offset_changed_span_samples_max\":9");
  expectJsonField(
      json, "\"trajectory_optimizer_candidate_local_speed_window_samples_total\":420");
  expectJsonField(
      json, "\"trajectory_optimizer_candidate_local_speed_window_samples_max\":35");
  expectJsonField(json, "\"trajectory_optimizer_local_candidate_evaluations\":18");
  expectJsonField(json,
                  "\"trajectory_optimizer_local_candidate_full_score_fallbacks\":15");
  expectJsonField(json,
                  "\"trajectory_optimizer_local_candidate_full_score_required\":5");
  expectJsonField(
      json,
      "\"trajectory_optimizer_local_candidate_full_score_required_invalid_input\":1");
  expectJsonField(
      json, "\"trajectory_optimizer_local_candidate_full_score_required_boundary\":2");
  expectJsonField(
      json,
      "\"trajectory_optimizer_local_candidate_full_score_required_unsafe_base\":1");
  expectJsonField(
      json,
      "\"trajectory_optimizer_local_candidate_full_score_required_window_invalid\":1");
  expectJsonField(json,
                  "\"trajectory_optimizer_local_candidate_acceptance_full_scores\":3");
  expectJsonField(
      json, "\"trajectory_optimizer_local_candidate_point_build_duration_ms\":0.75");
  expectJsonField(
      json,
      "\"trajectory_optimizer_local_candidate_path_evaluation_duration_ms\":0.85");
  expectJsonField(
      json,
      "\"trajectory_optimizer_local_candidate_traversal_estimate_duration_ms\":0.95");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_lower_bound_validation_full_scores\":10");
  expectJsonField(json, "\"trajectory_optimizer_shadow_lower_bound_validation_full_"
                        "score_duration_ms\":1.5");
  expectJsonField(json, "\"trajectory_optimizer_shadow_lower_bound_evaluations\":12");
  expectJsonField(json, "\"trajectory_optimizer_shadow_lower_bound_unavailable\":6");
  expectJsonField(json, "\"trajectory_optimizer_shadow_lower_bound_prunable\":5");
  expectJsonField(json, "\"trajectory_optimizer_shadow_lower_bound_false_prunes\":1");
  expectJsonField(json, "\"trajectory_optimizer_shadow_lower_bound_winner_prunes\":1");
  expectJsonField(json, "\"trajectory_optimizer_shadow_lower_bound_prunable_full_score_"
                        "duration_ms\":2.5");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_lower_bound_max_overestimate_score\":0.125");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_lower_bound_max_underestimate_score\":4.5");
  expectJsonField(json, "\"trajectory_optimizer_shadow_lower_bound_max_false_prune_"
                        "improvement_score\":0.75");
  expectJsonField(json, "\"trajectory_optimizer_shadow_local_speed_evaluations\":14");
  expectJsonField(json, "\"trajectory_optimizer_shadow_local_speed_unavailable\":4");
  expectJsonField(json, "\"trajectory_optimizer_shadow_local_speed_prunable\":3");
  expectJsonField(json, "\"trajectory_optimizer_shadow_local_speed_false_prunes\":1");
  expectJsonField(json,
                  "\"trajectory_optimizer_shadow_local_speed_winner_mismatches\":2");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_local_speed_abs_time_error_sum_s\":1.25");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_local_speed_abs_time_error_p95_s\":0.45");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_local_speed_max_time_overestimate_s\":0.5");
  expectJsonField(
      json,
      "\"trajectory_optimizer_shadow_local_speed_max_time_underestimate_s\":0.75");
  expectJsonField(json,
                  "\"trajectory_optimizer_shadow_local_speed_abs_score_error_sum\":50");
  expectJsonField(json,
                  "\"trajectory_optimizer_shadow_local_speed_abs_score_error_p95\":18");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_local_speed_max_score_overestimate\":20");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_local_speed_max_score_underestimate\":30");
  expectJsonField(json, "\"trajectory_optimizer_shadow_local_speed_max_false_prune_"
                        "improvement_score\":6.5");
  expectJsonField(json, "\"trajectory_optimizer_shadow_segment_score_evaluations\":13");
  expectJsonField(json, "\"trajectory_optimizer_shadow_segment_score_unavailable\":5");
  expectJsonField(json, "\"trajectory_optimizer_shadow_segment_score_prunable\":4");
  expectJsonField(json, "\"trajectory_optimizer_shadow_segment_score_false_prunes\":0");
  expectJsonField(json,
                  "\"trajectory_optimizer_shadow_segment_score_winner_mismatches\":1");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_segment_score_window_samples_total\":99");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_segment_score_window_samples_max\":11");
  expectJsonField(json,
                  "\"trajectory_optimizer_shadow_segment_score_abs_error_sum\":0.25");
  expectJsonField(json,
                  "\"trajectory_optimizer_shadow_segment_score_abs_error_p95\":0.1");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_segment_score_max_overestimate\":0.15");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_segment_score_max_underestimate\":0.2");
  expectJsonField(json, "\"trajectory_optimizer_shadow_segment_score_max_false_prune_"
                        "improvement_score\":0");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_boundary_clamped_local_candidates\":8");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_boundary_clamped_window_samples_total\":64");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_boundary_clamped_window_samples_max\":9");
  expectJsonField(json,
                  "\"trajectory_optimizer_shadow_speed_profile_cache_queries\":15");
  expectJsonField(json, "\"trajectory_optimizer_shadow_speed_profile_cache_hits\":2");
  expectJsonField(json,
                  "\"trajectory_optimizer_shadow_speed_profile_cache_unique\":13");
  expectJsonField(json, "\"trajectory_optimizer_dp_segment_cache_misses\":92");
  expectJsonField(json, "\"trajectory_optimizer_candidate_segment_cache_misses\":40");
  expectJsonField(json, "\"trajectory_optimizer_full_path_segment_cache_hits\":11");
  expectJsonField(json, "\"trajectory_optimizer_full_path_segment_cache_misses\":12");
  expectJsonField(json, "\"trajectory_optimizer_window_count\":2");
  expectJsonField(json, "\"trajectory_optimizer_active_window_count\":1");
  expectJsonField(json, "\"trajectory_optimizer_active_window_centerline_blocked\":1");
  expectJsonField(json,
                  "\"trajectory_optimizer_active_window_heading_change_samples\":2");
  expectJsonField(json,
                  "\"trajectory_optimizer_active_window_heading_span_samples\":3");
  expectJsonField(json, "\"trajectory_optimizer_active_window_curvature_samples\":4");
  expectJsonField(json,
                  "\"trajectory_optimizer_active_window_width_change_samples\":5");
  expectJsonField(json,
                  "\"trajectory_optimizer_active_window_width_asymmetry_samples\":6");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_active_window_no_width_asymmetry_count\":2");
  expectJsonField(
      json,
      "\"trajectory_optimizer_shadow_active_window_no_width_asymmetry_samples\":15");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_active_window_no_width_triggers_count\":3");
  expectJsonField(
      json,
      "\"trajectory_optimizer_shadow_active_window_no_width_triggers_samples\":12");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_active_window_no_heading_span_count\":4");
  expectJsonField(
      json, "\"trajectory_optimizer_shadow_active_window_no_heading_span_samples\":14");
  expectJsonField(json, "\"trajectory_optimizer_centerline_blocked_windows\":3");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_window_samples\":17");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_window_merged_count\":2");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_prohibited_cells\":7");
  expectJsonField(
      json, "\"trajectory_optimizer_centerline_blocked_outside_grid_segments\":8");
  expectJsonField(json, "\"trajectory_optimizer_centerline_blocked_segment_count\":2");
  expectJsonField(json, "\"trajectory_optimizer_centerline_blocked_span_count\":1");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_first_segment_index\":9");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_last_segment_index\":10");
  expectJsonField(json, "\"trajectory_optimizer_centerline_blocked_first_s_m\":12.5");
  expectJsonField(json, "\"trajectory_optimizer_centerline_blocked_last_s_m\":16.75");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_span_length_m\":4.25");
  expectJsonField(json, "\"trajectory_optimizer_centerline_blocked_first_x_m\":3.25");
  expectJsonField(json, "\"trajectory_optimizer_centerline_blocked_first_y_m\":-4.5");
  expectJsonField(json, "\"trajectory_optimizer_centerline_blocked_last_x_m\":5.5");
  expectJsonField(json, "\"trajectory_optimizer_centerline_blocked_last_y_m\":-6.75");
  expectJsonField(
      json, "\"trajectory_optimizer_centerline_blocked_first_outside_grid\":true");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_last_outside_grid\":true");
  expectJsonField(
      json, "\"trajectory_optimizer_centerline_blocked_span_diagnostic_count\":1");
  expectJsonField(
      json, "\"trajectory_optimizer_centerline_blocked_span0_begin_segment_index\":9");
  expectJsonField(
      json, "\"trajectory_optimizer_centerline_blocked_span0_end_segment_index\":10");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_span0_begin_s_m\":12.5");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_span0_end_s_m\":16.75");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_span0_length_m\":4.25");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_span0_begin_x_m\":3.25");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_span0_begin_y_m\":-4.5");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_span0_end_x_m\":5.5");
  expectJsonField(json,
                  "\"trajectory_optimizer_centerline_blocked_span0_end_y_m\":-6.75");
  expectJsonField(
      json, "\"trajectory_optimizer_centerline_blocked_span0_prohibited_cells\":7");
  expectJsonField(
      json,
      "\"trajectory_optimizer_centerline_blocked_span0_outside_grid_segments\":8");
  expectJsonField(json, "\"trajectory_optimizer_dp_states\":24");
  expectJsonField(json, "\"trajectory_optimizer_dp_transitions\":96");
  expectJsonField(json, "\"trajectory_optimizer_dp_coarse_states\":8");
  expectJsonField(json, "\"trajectory_optimizer_dp_fine_transitions\":76");
  expectJsonField(json, "\"trajectory_optimizer_dp_coarse_to_fine_used\":true");
  expectJsonField(json, "\"trajectory_optimizer_async_refined\":true");
  expectJsonField(json, "\"trajectory_shape_segment_count\":24");
  expectJsonField(json, "\"path\":{\"valid\":true");
  expectJsonField(json, "\"final_trajectory_debug_segment_type\":\"turn\"");
  expectJsonField(json, "\"tracking\":{\"valid\":true");
  expectJsonField(json, "\"control\":{\"motion_phase\":\"cruise\"");
  expectJsonField(json, "\"terminal_position_capture_active\":true");
  expectJsonField(json, "\"terminal_position_capture_reason\":\"terminal_stuck\"");
  expectJsonField(json, "\"terminal_position_capture_goal_distance_m\":7");
  expectJsonField(json, "\"terminal_position_capture_remaining_s_m\":2");
  expectJsonField(json, "\"terminal_position_capture_speed_mps\":0.4");
  expectJsonField(json, "\"terminal_position_capture_activation_radius_m\":8");
  expectJsonField(json, "\"terminal_position_capture_max_entry_speed_mps\":3");
  expectJsonField(json, "\"terminal_position_capture_stuck_speed_mps\":0.5");
  expectJsonField(json, "\"obstacle\":{\"prohibited_grid_clearance_m\":9");
  expectJsonField(json, "\"nearest_prohibited_cell_valid\":true");
}

} // namespace drone_city_nav
