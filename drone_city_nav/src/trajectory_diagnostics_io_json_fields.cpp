#include "trajectory_diagnostics_io_internal.hpp"

namespace drone_city_nav {

using namespace trajectory_diagnostics_io_detail;

std::string
speedProfileConstraintDiagnosticsJsonFields(const TrajectoryPlannerStats& stats) {
  return speedProfileConstraintDiagnosticsJsonFieldsImpl(stats);
}

std::string
trajectoryOptimizerDiagnosticsJsonFields(const TrajectoryPlannerStats& stats) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"trajectory_optimizer_final_estimated_time_s\":";
  writeJsonNumberOrNull(stream, stats.trajectory_optimizer.estimated_time_s);
  appendJsonNumber(stream, "trajectory_optimizer_final_min_speed_limit_mps",
                   stats.trajectory_optimizer.min_speed_limit_mps);
  appendJsonNumber(stream, "trajectory_optimizer_final_max_speed_limit_mps",
                   stats.trajectory_optimizer.max_speed_limit_mps);
  appendJsonSize(stream, "trajectory_optimizer_final_curvature_limited_samples",
                 stats.trajectory_optimizer.curvature_limited_samples);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_length_m",
                   stats.trajectory_optimizer.centerline_length_m);
  appendJsonNumber(stream, "trajectory_optimizer_final_length_m",
                   stats.trajectory_optimizer.final_length_m);
  appendJsonNumber(stream, "trajectory_optimizer_final_length_ratio",
                   stats.trajectory_optimizer.final_length_ratio);
  appendJsonNumber(stream, "trajectory_optimizer_max_abs_offset_m",
                   stats.trajectory_optimizer.max_abs_offset_m);
  appendJsonNumber(stream, "trajectory_optimizer_min_edge_margin_m",
                   stats.trajectory_optimizer.min_edge_margin_m);
  appendJsonNumber(stream, "trajectory_optimizer_mean_edge_margin_m",
                   stats.trajectory_optimizer.mean_edge_margin_m);
  appendJsonNumber(stream, "trajectory_optimizer_cost_curvature",
                   stats.trajectory_optimizer.cost_curvature);
  appendJsonNumber(stream, "trajectory_optimizer_cost_curvature_change",
                   stats.trajectory_optimizer.cost_curvature_change);
  appendJsonNumber(stream, "trajectory_optimizer_cost_radius_shortfall",
                   stats.trajectory_optimizer.cost_radius_shortfall);
  appendJsonNumber(stream, "trajectory_optimizer_cost_heading_jump",
                   stats.trajectory_optimizer.cost_heading_jump);
  appendJsonNumber(stream, "trajectory_optimizer_cost_offset_change",
                   stats.trajectory_optimizer.cost_offset_change);
  appendJsonNumber(stream, "trajectory_optimizer_cost_offset_second_change",
                   stats.trajectory_optimizer.cost_offset_second_change);
  appendJsonNumber(stream, "trajectory_optimizer_cost_offset_slope",
                   stats.trajectory_optimizer.cost_offset_slope);
  appendJsonNumber(stream, "trajectory_optimizer_cost_collision",
                   stats.trajectory_optimizer.cost_collision);
  appendJsonNumber(stream, "trajectory_optimizer_cost_outside_grid",
                   stats.trajectory_optimizer.cost_outside_grid);
  appendJsonNumber(stream, "trajectory_optimizer_best_candidate_score",
                   stats.trajectory_optimizer.best_candidate_score);
  appendJsonSize(stream, "trajectory_optimizer_regularization_iterations",
                 stats.trajectory_optimizer.regularization_iterations);
  stream << ",\"trajectory_optimizer_regularization_applied\":"
         << (stats.trajectory_optimizer.regularization_applied ? "true" : "false");
  appendJsonNumber(
      stream, "trajectory_optimizer_pre_regularization_max_curvature_jump_1pm",
      stats.trajectory_optimizer.pre_regularization_max_curvature_jump_1pm);
  appendJsonNumber(
      stream, "trajectory_optimizer_post_regularization_max_curvature_jump_1pm",
      stats.trajectory_optimizer.post_regularization_max_curvature_jump_1pm);
  appendJsonSize(stream, "trajectory_optimizer_skipped_noop_candidates",
                 stats.trajectory_optimizer.skipped_noop_candidates);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_path_evaluation_duration_ms",
                   stats.trajectory_optimizer.candidate_path_evaluation_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_score_duration_ms",
                   stats.trajectory_optimizer.candidate_score_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_point_build_duration_ms",
                   stats.trajectory_optimizer.candidate_point_build_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_sample_build_duration_ms",
                   stats.trajectory_optimizer.candidate_sample_build_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_cost_breakdown_duration_ms",
                   stats.trajectory_optimizer.candidate_cost_breakdown_duration_ms);
  appendJsonNumber(stream,
                   "trajectory_optimizer_candidate_shape_diagnostics_duration_ms",
                   stats.trajectory_optimizer.candidate_shape_diagnostics_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_regularization_duration_ms",
                   stats.trajectory_optimizer.regularization_duration_ms);
  appendJsonSize(stream, "trajectory_optimizer_scratch_reused_candidates",
                 stats.trajectory_optimizer.scratch_reused_candidates);
  appendJsonBool(stream, "trajectory_optimizer_parallel_candidate_evaluation_used",
                 stats.trajectory_optimizer.parallel_candidate_evaluation_used);
  appendJsonSize(stream, "trajectory_optimizer_parallel_workers_used",
                 stats.trajectory_optimizer.parallel_workers_used);
  appendJsonSize(stream, "trajectory_optimizer_candidate_chunks",
                 stats.trajectory_optimizer.candidate_chunks);
  appendJsonSize(stream, "trajectory_optimizer_candidate_parallel_batches",
                 stats.trajectory_optimizer.candidate_parallel_batches);
  appendJsonSize(stream, "trajectory_optimizer_candidate_threads_launched",
                 stats.trajectory_optimizer.candidate_threads_launched);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_batch_wall_duration_ms",
                   stats.trajectory_optimizer.candidate_batch_wall_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_batch_wait_duration_ms",
                   stats.trajectory_optimizer.candidate_batch_wait_duration_ms);
  appendJsonNumber(
      stream, "trajectory_optimizer_candidate_worker_buffer_prepare_duration_ms",
      stats.trajectory_optimizer.candidate_worker_buffer_prepare_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_thread_launch_duration_ms",
                   stats.trajectory_optimizer.candidate_thread_launch_duration_ms);
  appendJsonNumber(stream,
                   "trajectory_optimizer_candidate_thread_join_wait_duration_ms",
                   stats.trajectory_optimizer.candidate_thread_join_wait_duration_ms);
  appendJsonSize(stream, "trajectory_optimizer_worker_scratch_reuses",
                 stats.trajectory_optimizer.worker_scratch_reuses);
  appendJsonSize(stream, "trajectory_optimizer_candidate_snapshot_allocations_avoided",
                 stats.trajectory_optimizer.candidate_snapshot_allocations_avoided);
  appendJsonSize(stream, "trajectory_optimizer_candidate_offset_changed_samples_total",
                 stats.trajectory_optimizer.candidate_offset_changed_samples_total);
  appendJsonSize(stream, "trajectory_optimizer_candidate_offset_changed_samples_max",
                 stats.trajectory_optimizer.candidate_offset_changed_samples_max);
  appendJsonSize(
      stream, "trajectory_optimizer_candidate_offset_changed_span_samples_total",
      stats.trajectory_optimizer.candidate_offset_changed_span_samples_total);
  appendJsonSize(stream,
                 "trajectory_optimizer_candidate_offset_changed_span_samples_max",
                 stats.trajectory_optimizer.candidate_offset_changed_span_samples_max);
  appendJsonSize(stream,
                 "trajectory_optimizer_candidate_local_speed_window_samples_total",
                 stats.trajectory_optimizer.candidate_local_speed_window_samples_total);
  appendJsonSize(stream,
                 "trajectory_optimizer_candidate_local_speed_window_samples_max",
                 stats.trajectory_optimizer.candidate_local_speed_window_samples_max);
  appendJsonSize(stream, "trajectory_optimizer_local_candidate_evaluations",
                 stats.trajectory_optimizer.local_candidate_evaluations);
  appendJsonSize(stream, "trajectory_optimizer_local_candidate_full_score_fallbacks",
                 stats.trajectory_optimizer.local_candidate_full_score_fallbacks);
  appendJsonSize(stream, "trajectory_optimizer_local_candidate_full_score_required",
                 stats.trajectory_optimizer.local_candidate_full_score_required);
  appendJsonSize(
      stream, "trajectory_optimizer_local_candidate_full_score_required_invalid_input",
      stats.trajectory_optimizer.local_candidate_full_score_required_invalid_input);
  appendJsonSize(
      stream, "trajectory_optimizer_local_candidate_full_score_required_boundary",
      stats.trajectory_optimizer.local_candidate_full_score_required_boundary);
  appendJsonSize(
      stream, "trajectory_optimizer_local_candidate_full_score_required_unsafe_base",
      stats.trajectory_optimizer.local_candidate_full_score_required_unsafe_base);
  appendJsonSize(
      stream, "trajectory_optimizer_local_candidate_full_score_required_window_invalid",
      stats.trajectory_optimizer.local_candidate_full_score_required_window_invalid);
  appendJsonSize(stream, "trajectory_optimizer_local_candidate_acceptance_full_scores",
                 stats.trajectory_optimizer.local_candidate_acceptance_full_scores);
  appendJsonSize(stream, "trajectory_optimizer_local_score_false_positives",
                 stats.trajectory_optimizer.local_score_false_positives);
  appendJsonNumber(stream,
                   "trajectory_optimizer_local_candidate_point_build_duration_ms",
                   stats.trajectory_optimizer.local_candidate_point_build_duration_ms);
  appendJsonNumber(
      stream, "trajectory_optimizer_local_candidate_path_evaluation_duration_ms",
      stats.trajectory_optimizer.local_candidate_path_evaluation_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_local_candidate_score_duration_ms",
                   stats.trajectory_optimizer.local_candidate_score_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_full_candidate_score_duration_ms",
                   stats.trajectory_optimizer.full_candidate_score_duration_ms);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_evaluations",
                 stats.trajectory_optimizer.shadow_segment_score_evaluations);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_unavailable",
                 stats.trajectory_optimizer.shadow_segment_score_unavailable);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_prunable",
                 stats.trajectory_optimizer.shadow_segment_score_prunable);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_false_prunes",
                 stats.trajectory_optimizer.shadow_segment_score_false_prunes);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_winner_mismatches",
                 stats.trajectory_optimizer.shadow_segment_score_winner_mismatches);
  appendJsonSize(stream,
                 "trajectory_optimizer_shadow_segment_score_window_samples_total",
                 stats.trajectory_optimizer.shadow_segment_score_window_samples_total);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_window_samples_max",
                 stats.trajectory_optimizer.shadow_segment_score_window_samples_max);
  appendJsonNumber(stream, "trajectory_optimizer_shadow_segment_score_abs_error_sum",
                   stats.trajectory_optimizer.shadow_segment_score_abs_error_sum);
  appendJsonNumber(stream, "trajectory_optimizer_shadow_segment_score_abs_error_p95",
                   stats.trajectory_optimizer.shadow_segment_score_abs_error_p95);
  appendJsonNumber(stream, "trajectory_optimizer_shadow_segment_score_max_overestimate",
                   stats.trajectory_optimizer.shadow_segment_score_max_overestimate);
  appendJsonNumber(stream,
                   "trajectory_optimizer_shadow_segment_score_max_underestimate",
                   stats.trajectory_optimizer.shadow_segment_score_max_underestimate);
  appendJsonNumber(
      stream,
      "trajectory_optimizer_shadow_segment_score_max_false_prune_improvement_score",
      stats.trajectory_optimizer
          .shadow_segment_score_max_false_prune_improvement_score);
  appendJsonSize(stream,
                 "trajectory_optimizer_shadow_boundary_clamped_local_candidates",
                 stats.trajectory_optimizer.shadow_boundary_clamped_local_candidates);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_boundary_clamped_window_samples_total",
      stats.trajectory_optimizer.shadow_boundary_clamped_window_samples_total);
  appendJsonSize(stream,
                 "trajectory_optimizer_shadow_boundary_clamped_window_samples_max",
                 stats.trajectory_optimizer.shadow_boundary_clamped_window_samples_max);
  appendJsonSize(stream, "trajectory_optimizer_window_count",
                 stats.trajectory_optimizer.window_count);
  appendJsonSize(stream, "trajectory_optimizer_active_window_count",
                 stats.trajectory_optimizer.active_window_count);
  appendJsonSize(stream, "trajectory_optimizer_active_window_samples",
                 stats.trajectory_optimizer.active_window_samples);
  appendJsonSize(stream, "trajectory_optimizer_active_window_centerline_blocked",
                 stats.trajectory_optimizer.active_window_centerline_blocked);
  appendJsonSize(stream, "trajectory_optimizer_active_window_heading_change_samples",
                 stats.trajectory_optimizer.active_window_heading_change_samples);
  appendJsonSize(stream, "trajectory_optimizer_active_window_heading_span_samples",
                 stats.trajectory_optimizer.active_window_heading_span_samples);
  appendJsonSize(stream, "trajectory_optimizer_active_window_curvature_samples",
                 stats.trajectory_optimizer.active_window_curvature_samples);
  appendJsonSize(stream, "trajectory_optimizer_active_window_width_change_samples",
                 stats.trajectory_optimizer.active_window_width_change_samples);
  appendJsonSize(stream, "trajectory_optimizer_active_window_width_asymmetry_samples",
                 stats.trajectory_optimizer.active_window_width_asymmetry_samples);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_active_window_no_width_asymmetry_count",
      stats.trajectory_optimizer.shadow_active_window_no_width_asymmetry_count);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_active_window_no_width_asymmetry_samples",
      stats.trajectory_optimizer.shadow_active_window_no_width_asymmetry_samples);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_active_window_no_width_triggers_count",
      stats.trajectory_optimizer.shadow_active_window_no_width_triggers_count);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_active_window_no_width_triggers_samples",
      stats.trajectory_optimizer.shadow_active_window_no_width_triggers_samples);
  appendJsonSize(stream,
                 "trajectory_optimizer_shadow_active_window_no_heading_span_count",
                 stats.trajectory_optimizer.shadow_active_window_no_heading_span_count);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_active_window_no_heading_span_samples",
      stats.trajectory_optimizer.shadow_active_window_no_heading_span_samples);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_windows",
                 stats.trajectory_optimizer.centerline_blocked_windows);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_window_samples",
                 stats.trajectory_optimizer.centerline_blocked_window_samples);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_window_merged_count",
                 stats.trajectory_optimizer.centerline_blocked_window_merged_count);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_prohibited_cells",
                 stats.trajectory_optimizer.centerline_blocked_prohibited_cells);
  appendJsonSize(stream,
                 "trajectory_optimizer_centerline_blocked_outside_grid_segments",
                 stats.trajectory_optimizer.centerline_blocked_outside_grid_segments);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_segment_count",
                 stats.trajectory_optimizer.centerline_blocked_segment_count);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_span_count",
                 stats.trajectory_optimizer.centerline_blocked_span_count);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_first_segment_index",
                 stats.trajectory_optimizer.centerline_blocked_first_segment_index);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_last_segment_index",
                 stats.trajectory_optimizer.centerline_blocked_last_segment_index);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_first_s_m",
                   stats.trajectory_optimizer.centerline_blocked_first_s_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_last_s_m",
                   stats.trajectory_optimizer.centerline_blocked_last_s_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_span_length_m",
                   stats.trajectory_optimizer.centerline_blocked_span_length_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_first_x_m",
                   stats.trajectory_optimizer.centerline_blocked_first_x_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_first_y_m",
                   stats.trajectory_optimizer.centerline_blocked_first_y_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_last_x_m",
                   stats.trajectory_optimizer.centerline_blocked_last_x_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_last_y_m",
                   stats.trajectory_optimizer.centerline_blocked_last_y_m);
  appendJsonBool(stream, "trajectory_optimizer_centerline_blocked_first_outside_grid",
                 stats.trajectory_optimizer.centerline_blocked_first_outside_grid);
  appendJsonBool(stream, "trajectory_optimizer_centerline_blocked_last_outside_grid",
                 stats.trajectory_optimizer.centerline_blocked_last_outside_grid);
  appendJsonSize(stream,
                 "trajectory_optimizer_centerline_blocked_span_diagnostic_count",
                 stats.trajectory_optimizer.centerline_blocked_span_diagnostic_count);
  for (std::size_t i = 0U; i < kMaxCenterlineBlockedSpanDiagnostics; ++i) {
    const std::string prefix =
        "trajectory_optimizer_centerline_blocked_span" + std::to_string(i);
    const TrajectoryOptimizerBlockedSpanDiagnostic& span =
        stats.trajectory_optimizer.centerline_blocked_span_diagnostics.at(i);
    appendJsonSize(stream, prefix + "_begin_segment_index", span.begin_segment_index);
    appendJsonSize(stream, prefix + "_end_segment_index", span.end_segment_index);
    appendJsonNumber(stream, prefix + "_begin_s_m", span.begin_s_m);
    appendJsonNumber(stream, prefix + "_end_s_m", span.end_s_m);
    appendJsonNumber(stream, prefix + "_length_m", span.length_m);
    appendJsonNumber(stream, prefix + "_begin_x_m", span.begin_x_m);
    appendJsonNumber(stream, prefix + "_begin_y_m", span.begin_y_m);
    appendJsonNumber(stream, prefix + "_end_x_m", span.end_x_m);
    appendJsonNumber(stream, prefix + "_end_y_m", span.end_y_m);
    appendJsonSize(stream, prefix + "_prohibited_cells", span.prohibited_cells);
    appendJsonSize(stream, prefix + "_outside_grid_segments",
                   span.outside_grid_segments);
  }
  appendJsonSize(stream, "trajectory_optimizer_dp_states",
                 stats.trajectory_optimizer.dp_states);
  appendJsonSize(stream, "trajectory_optimizer_dp_transitions",
                 stats.trajectory_optimizer.dp_transitions);
  appendJsonSize(stream, "trajectory_optimizer_dp_segment_cache_hits",
                 stats.trajectory_optimizer.dp_segment_cache_hits);
  appendJsonSize(stream, "trajectory_optimizer_dp_segment_cache_misses",
                 stats.trajectory_optimizer.dp_segment_cache_misses);
  appendJsonSize(stream, "trajectory_optimizer_candidate_segment_cache_hits",
                 stats.trajectory_optimizer.candidate_segment_cache_hits);
  appendJsonSize(stream, "trajectory_optimizer_candidate_segment_cache_misses",
                 stats.trajectory_optimizer.candidate_segment_cache_misses);
  appendJsonSize(stream, "trajectory_optimizer_full_path_segment_cache_hits",
                 stats.trajectory_optimizer.full_path_segment_cache_hits);
  appendJsonSize(stream, "trajectory_optimizer_full_path_segment_cache_misses",
                 stats.trajectory_optimizer.full_path_segment_cache_misses);
  appendJsonSize(stream, "trajectory_optimizer_dp_coarse_states",
                 stats.trajectory_optimizer.dp_coarse_states);
  appendJsonSize(stream, "trajectory_optimizer_dp_coarse_transitions",
                 stats.trajectory_optimizer.dp_coarse_transitions);
  appendJsonSize(stream, "trajectory_optimizer_dp_fine_states",
                 stats.trajectory_optimizer.dp_fine_states);
  appendJsonSize(stream, "trajectory_optimizer_dp_fine_transitions",
                 stats.trajectory_optimizer.dp_fine_transitions);
  appendJsonBool(stream, "trajectory_optimizer_dp_coarse_to_fine_used",
                 stats.trajectory_optimizer.dp_coarse_to_fine_used);
  appendJsonNumber(stream, "trajectory_optimizer_window_detection_duration_ms",
                   stats.trajectory_optimizer.window_detection_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_window_eval_duration_ms",
                   stats.trajectory_optimizer.window_eval_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_dp_duration_ms",
                   stats.trajectory_optimizer.dp_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_full_final_score_duration_ms",
                   stats.trajectory_optimizer.full_final_score_duration_ms);
  appendJsonBool(stream, "trajectory_optimizer_async_refined",
                 stats.trajectory_optimizer.async_refined);
  return stream.str();
}

std::string turnSmoothingDiagnosticsJsonFields(const TrajectoryPlannerStats& stats) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"turn_smoothing_input_samples\":" << stats.turn_smoothing.input_samples;
  appendJsonSize(stream, "turn_smoothing_output_samples",
                 stats.turn_smoothing.output_samples);
  appendJsonSize(stream, "turn_smoothing_detected_corners",
                 stats.turn_smoothing.detected_corners);
  appendJsonSize(stream, "turn_smoothing_attempted_corners",
                 stats.turn_smoothing.attempted_corners);
  appendJsonSize(stream, "turn_smoothing_candidate_attempts",
                 stats.turn_smoothing.candidate_attempts);
  appendJsonSize(stream, "turn_smoothing_relaxed_candidate_attempts",
                 stats.turn_smoothing.relaxed_candidate_attempts);
  appendJsonSize(stream, "turn_smoothing_bezier_cache_hits",
                 stats.turn_smoothing.bezier_cache_hits);
  appendJsonSize(stream, "turn_smoothing_bezier_cache_misses",
                 stats.turn_smoothing.bezier_cache_misses);
  appendJsonSize(stream, "turn_smoothing_before_metrics_cache_hits",
                 stats.turn_smoothing.before_metrics_cache_hits);
  appendJsonSize(stream, "turn_smoothing_before_metrics_cache_misses",
                 stats.turn_smoothing.before_metrics_cache_misses);
  appendJsonSize(stream, "turn_smoothing_traversability_cache_hits",
                 stats.turn_smoothing.traversability_cache_hits);
  appendJsonSize(stream, "turn_smoothing_traversability_cache_misses",
                 stats.turn_smoothing.traversability_cache_misses);
  appendJsonSize(stream, "turn_smoothing_smoothed_corners",
                 stats.turn_smoothing.smoothed_corners);
  appendJsonSize(stream, "turn_smoothing_rejected_prohibited",
                 stats.turn_smoothing.rejected_prohibited);
  appendJsonSize(stream, "turn_smoothing_rejected_corridor",
                 stats.turn_smoothing.rejected_corridor);
  appendJsonSize(stream, "turn_smoothing_rejected_not_improved",
                 stats.turn_smoothing.rejected_not_improved);
  appendJsonSize(stream, "turn_smoothing_rejected_curvature_regression",
                 stats.turn_smoothing.rejected_curvature_regression);
  appendJsonSize(stream, "turn_smoothing_rejected_radius_regression",
                 stats.turn_smoothing.rejected_radius_regression);
  appendJsonNumber(stream, "turn_smoothing_heading_delta_before_rad",
                   stats.turn_smoothing.max_heading_delta_before_rad);
  appendJsonNumber(stream, "turn_smoothing_heading_delta_after_rad",
                   stats.turn_smoothing.max_heading_delta_after_rad);
  appendJsonNumber(stream, "turn_smoothing_curvature_jump_before_1pm",
                   stats.turn_smoothing.max_curvature_jump_before_1pm);
  appendJsonNumber(stream, "turn_smoothing_curvature_jump_after_1pm",
                   stats.turn_smoothing.max_curvature_jump_after_1pm);
  appendJsonNumber(stream, "turn_smoothing_min_inner_margin_m",
                   stats.turn_smoothing.min_inner_margin_m);
  appendJsonNumber(stream, "turn_smoothing_max_outer_shift_m",
                   stats.turn_smoothing.max_applied_outer_shift_m);
  appendJsonNumber(stream, "turn_smoothing_accepted_entry_distance_m",
                   stats.turn_smoothing.accepted_entry_distance_m);
  appendJsonNumber(stream, "turn_smoothing_accepted_exit_distance_m",
                   stats.turn_smoothing.accepted_exit_distance_m);
  appendJsonNumber(stream, "turn_smoothing_accepted_shift_scale",
                   stats.turn_smoothing.accepted_shift_scale);
  appendJsonNumber(stream, "turn_smoothing_accepted_relaxed_angle_deg",
                   stats.turn_smoothing.accepted_relaxed_angle_deg);
  appendJsonNumber(stream, "turn_smoothing_accepted_score",
                   stats.turn_smoothing.accepted_score);
  appendJsonNumber(stream, "turn_smoothing_accepted_min_radius_before_m",
                   stats.turn_smoothing.accepted_min_radius_before_m);
  appendJsonNumber(stream, "turn_smoothing_accepted_min_radius_after_m",
                   stats.turn_smoothing.accepted_min_radius_after_m);
  appendJsonNumber(stream, "turn_smoothing_accepted_min_speed_before_mps",
                   stats.turn_smoothing.accepted_min_speed_before_mps);
  appendJsonNumber(stream, "turn_smoothing_accepted_min_speed_after_mps",
                   stats.turn_smoothing.accepted_min_speed_after_mps);
  appendJsonNumber(stream, "turn_smoothing_accepted_local_time_before_s",
                   stats.turn_smoothing.accepted_local_time_before_s);
  appendJsonNumber(stream, "turn_smoothing_accepted_local_time_after_s",
                   stats.turn_smoothing.accepted_local_time_after_s);
  appendJsonNumber(stream, "turn_smoothing_candidate_build_duration_ms",
                   stats.turn_smoothing.candidate_build_duration_ms);
  appendJsonNumber(stream, "turn_smoothing_candidate_replace_duration_ms",
                   stats.turn_smoothing.candidate_replace_duration_ms);
  appendJsonNumber(stream, "turn_smoothing_collision_check_duration_ms",
                   stats.turn_smoothing.collision_check_duration_ms);
  appendJsonNumber(stream, "turn_smoothing_metrics_duration_ms",
                   stats.turn_smoothing.metrics_duration_ms);
  appendJsonNumber(stream, "turn_smoothing_shape_diagnostics_duration_ms",
                   stats.turn_smoothing.shape_diagnostics_duration_ms);
  appendJsonNumber(stream, "turn_smoothing_speed_profile_duration_ms",
                   stats.turn_smoothing.speed_profile_duration_ms);
  stream << ",\"turn_smoothing_corner_diagnostics\":[";
  for (std::size_t i = 0U; i < stats.turn_smoothing.corner_diagnostics.size(); ++i) {
    const TurnSmoothingCornerDiagnostic& diagnostic =
        stats.turn_smoothing.corner_diagnostics[i];
    if (i > 0U) {
      stream << ",";
    }
    stream << "{\"accepted\":" << (diagnostic.accepted ? "true" : "false");
    appendJsonString(stream, "reject_reason", diagnostic.reject_reason);
    appendJsonString(stream, "reject_detail", diagnostic.reject_detail);
    appendJsonNumber(stream, "corner_s_m", diagnostic.corner_s_m);
    appendJsonNumber(stream, "entry_distance_m", diagnostic.entry_distance_m);
    appendJsonNumber(stream, "exit_distance_m", diagnostic.exit_distance_m);
    appendJsonNumber(stream, "shift_scale", diagnostic.shift_scale);
    appendJsonNumber(stream, "relaxed_angle_deg", diagnostic.relaxed_angle_deg);
    appendJsonNumber(stream, "score", diagnostic.score);
    appendJsonNumber(stream, "min_radius_before_m", diagnostic.min_radius_before_m);
    appendJsonNumber(stream, "min_radius_after_m", diagnostic.min_radius_after_m);
    appendJsonNumber(stream, "min_speed_before_mps", diagnostic.min_speed_before_mps);
    appendJsonNumber(stream, "min_speed_after_mps", diagnostic.min_speed_after_mps);
    appendJsonNumber(stream, "local_time_before_s", diagnostic.local_time_before_s);
    appendJsonNumber(stream, "local_time_after_s", diagnostic.local_time_after_s);
    appendJsonNumber(stream, "curvature_jump_before_1pm",
                     diagnostic.curvature_jump_before_1pm);
    appendJsonNumber(stream, "curvature_jump_after_1pm",
                     diagnostic.curvature_jump_after_1pm);
    appendJsonNumber(stream, "heading_delta_before_rad",
                     diagnostic.heading_delta_before_rad);
    appendJsonNumber(stream, "heading_delta_after_rad",
                     diagnostic.heading_delta_after_rad);
    stream << "}";
  }
  stream << "]";
  return stream.str();
}

std::string trajectoryTimingDiagnosticsJsonFields(const TrajectoryPlannerStats& stats) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"trajectory_total_duration_ms\":";
  writeJsonNumberOrNull(stream, stats.total_duration_ms);
  appendJsonNumber(stream, "trajectory_corridor_duration_ms",
                   stats.corridor_duration_ms);
  appendJsonNumber(stream, "trajectory_trajectory_optimizer_duration_ms",
                   stats.trajectory_optimizer_duration_ms);
  appendJsonNumber(stream, "trajectory_turn_smoothing_duration_ms",
                   stats.turn_smoothing_duration_ms);
  appendJsonNumber(stream, "trajectory_passage_insertion_duration_ms",
                   stats.passage_insertion_duration_ms);
  appendJsonNumber(stream, "trajectory_speed_profile_duration_ms",
                   stats.speed_profile_duration_ms);
  return stream.str();
}

} // namespace drone_city_nav
