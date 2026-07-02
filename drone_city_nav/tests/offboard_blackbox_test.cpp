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
  record.trajectory_planner_stats.racing_line.iterations = 4U;
  record.trajectory_planner_stats.racing_line.optimizer_samples = 8U;
  record.trajectory_planner_stats.racing_line.final_cost = 1.5;
  record.trajectory_planner_stats.racing_line.candidate_point_build_duration_ms = 1.25;
  record.trajectory_planner_stats.racing_line.candidate_cost_breakdown_duration_ms =
      2.25;
  record.trajectory_planner_stats.racing_line.candidate_shape_diagnostics_duration_ms =
      3.25;
  record.trajectory_planner_stats.racing_line.candidate_speed_profile_duration_ms =
      4.25;
  record.trajectory_planner_stats.racing_line.scratch_reused_candidates = 7U;
  record.trajectory_planner_stats.racing_line.parallel_candidate_evaluation_used = true;
  record.trajectory_planner_stats.racing_line.parallel_workers_used = 2U;
  record.trajectory_planner_stats.racing_line.candidate_chunks = 9U;
  record.trajectory_planner_stats.racing_line.local_candidate_evaluations = 18U;
  record.trajectory_planner_stats.racing_line.local_candidate_full_score_fallbacks =
      15U;
  record.trajectory_planner_stats.racing_line.local_candidate_acceptance_full_scores =
      3U;
  record.trajectory_planner_stats.racing_line.top_n_full_score_candidates = 128U;
  record.trajectory_planner_stats.racing_line.top_n_full_score_selected = 96U;
  record.trajectory_planner_stats.racing_line.top_n_full_score_skipped = 384U;
  record.trajectory_planner_stats.racing_line.top_n_full_score_forced = 3U;
  record.trajectory_planner_stats.racing_line.top_n_best_full_score_local_rank = 42U;
  record.trajectory_planner_stats.racing_line.top_n_full_score_reduction_ratio = 0.8;
  record.trajectory_planner_stats.racing_line.top_n_preview_sort_duration_ms = 1.25;
  record.trajectory_planner_stats.racing_line.top_n_full_score_selection_duration_ms =
      2.75;
  record.trajectory_planner_stats.racing_line.dp_segment_cache_misses = 92U;
  record.trajectory_planner_stats.racing_line.candidate_segment_cache_misses = 40U;
  record.trajectory_planner_stats.racing_line.full_path_segment_cache_hits = 11U;
  record.trajectory_planner_stats.racing_line.full_path_segment_cache_misses = 12U;
  record.trajectory_planner_stats.racing_line.window_count = 2U;
  record.trajectory_planner_stats.racing_line.active_window_count = 1U;
  record.trajectory_planner_stats.racing_line.dp_states = 24U;
  record.trajectory_planner_stats.racing_line.dp_transitions = 96U;
  record.trajectory_planner_stats.racing_line.dp_coarse_states = 8U;
  record.trajectory_planner_stats.racing_line.dp_coarse_transitions = 20U;
  record.trajectory_planner_stats.racing_line.dp_fine_states = 16U;
  record.trajectory_planner_stats.racing_line.dp_fine_transitions = 76U;
  record.trajectory_planner_stats.racing_line.dp_coarse_to_fine_used = true;
  record.trajectory_planner_stats.racing_line.async_refined = true;
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
  expectJsonField(json, "\"racing_line_iterations\":4");
  expectJsonField(json, "\"racing_candidate_point_build_duration_ms\":1.25");
  expectJsonField(json, "\"racing_candidate_cost_breakdown_duration_ms\":2.25");
  expectJsonField(json, "\"racing_candidate_shape_diagnostics_duration_ms\":3.25");
  expectJsonField(json, "\"racing_candidate_speed_profile_duration_ms\":4.25");
  expectJsonField(json, "\"racing_scratch_reused_candidates\":7");
  expectJsonField(json, "\"racing_parallel_candidate_evaluation_used\":true");
  expectJsonField(json, "\"racing_parallel_workers_used\":2");
  expectJsonField(json, "\"racing_candidate_chunks\":9");
  expectJsonField(json, "\"racing_local_candidate_evaluations\":18");
  expectJsonField(json, "\"racing_local_candidate_full_score_fallbacks\":15");
  expectJsonField(json, "\"racing_local_candidate_acceptance_full_scores\":3");
  expectJsonField(json, "\"racing_top_n_full_score_candidates\":128");
  expectJsonField(json, "\"racing_top_n_full_score_selected\":96");
  expectJsonField(json, "\"racing_top_n_full_score_skipped\":384");
  expectJsonField(json, "\"racing_top_n_full_score_forced\":3");
  expectJsonField(json, "\"racing_top_n_best_full_score_local_rank\":42");
  expectJsonField(json, "\"racing_top_n_full_score_reduction_ratio\":0.8");
  expectJsonField(json, "\"racing_top_n_preview_sort_duration_ms\":1.25");
  expectJsonField(json, "\"racing_top_n_full_score_selection_duration_ms\":2.75");
  expectJsonField(json, "\"racing_line_dp_segment_cache_misses\":92");
  expectJsonField(json, "\"racing_line_candidate_segment_cache_misses\":40");
  expectJsonField(json, "\"racing_line_full_path_segment_cache_hits\":11");
  expectJsonField(json, "\"racing_line_full_path_segment_cache_misses\":12");
  expectJsonField(json, "\"racing_line_window_count\":2");
  expectJsonField(json, "\"racing_line_active_window_count\":1");
  expectJsonField(json, "\"racing_line_dp_states\":24");
  expectJsonField(json, "\"racing_line_dp_transitions\":96");
  expectJsonField(json, "\"racing_line_dp_coarse_states\":8");
  expectJsonField(json, "\"racing_line_dp_fine_transitions\":76");
  expectJsonField(json, "\"racing_line_dp_coarse_to_fine_used\":true");
  expectJsonField(json, "\"racing_line_async_refined\":true");
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
