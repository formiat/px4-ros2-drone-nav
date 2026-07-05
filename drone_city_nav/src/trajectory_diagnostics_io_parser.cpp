#include "trajectory_diagnostics_io_internal.hpp"

namespace drone_city_nav {

using namespace trajectory_diagnostics_io_detail;

std::optional<TrajectoryPlannerDiagnosticsEnvelope>
parseTrajectoryPlannerDiagnosticsJson(const std::string& json) {
  TrajectoryPlannerDiagnosticsEnvelope envelope{};
  if (!parseJsonUint64(json, "planner_path_id", envelope.planner_path_id) ||
      !parseJsonUint64(json, "path_stamp_ns", envelope.path_stamp_ns)) {
    return std::nullopt;
  }

  if (const std::optional<std::string_view> status =
          jsonValueForKey(json, "trajectory_status");
      status.has_value()) {
    envelope.stats.status = parseTrajectoryPlannerStatusName(*status);
  }
  if (const std::optional<std::string_view> quality =
          jsonValueForKey(json, "trajectory_quality");
      quality.has_value()) {
    envelope.stats.quality = parseTrajectoryQualityName(*quality);
  }

  parseJsonSize(json, "trajectory_input_points", envelope.stats.input_points);
  parseJsonSize(json, "trajectory_compact_segments", envelope.stats.compact_segments);
  parseJsonSize(json, "trajectory_line_segments", envelope.stats.line_segments);
  parseJsonSize(json, "trajectory_arc_segments", envelope.stats.arc_segments);
  parseJsonSize(json, "trajectory_samples", envelope.stats.samples);
  parseJsonDouble(json, "trajectory_length_m", envelope.stats.length_m);
  parseJsonDouble(json, "curvature_min_1pm", envelope.stats.curvature_min_1pm);
  parseJsonDouble(json, "curvature_max_1pm", envelope.stats.curvature_max_1pm);
  parseJsonDouble(json, "curvature_mean_abs_1pm",
                  envelope.stats.curvature_mean_abs_1pm);
  parseJsonDouble(json, "speed_profile_min_mps", envelope.stats.speed_profile_min_mps);
  parseJsonDouble(json, "planning_speed_profile_min_mps",
                  envelope.stats.speed_profile_min_mps);
  parseJsonDouble(json, "speed_profile_max_mps", envelope.stats.speed_profile_max_mps);
  parseJsonDouble(json, "planning_speed_profile_max_mps",
                  envelope.stats.speed_profile_max_mps);
  parseJsonDouble(json, "speed_profile_mean_mps",
                  envelope.stats.speed_profile_mean_mps);
  parseJsonDouble(json, "planning_speed_profile_mean_mps",
                  envelope.stats.speed_profile_mean_mps);
  parseJsonSize(json, "speed_profile_curvature_limited_samples",
                envelope.stats.speed_profile_curvature_limited_samples);
  parseJsonSize(json, "planning_speed_profile_curvature_limited_samples",
                envelope.stats.speed_profile_curvature_limited_samples);
  (void)parseJsonUint64(json, "planning_speed_config_fingerprint",
                        envelope.stats.speed_config_fingerprint);
  std::size_t top_constraint_count = 0U;
  parseJsonSize(json, "speed_profile_top_constraint_count", top_constraint_count);
  top_constraint_count = std::min<std::size_t>(top_constraint_count, 5U);
  envelope.stats.top_speed_constraints.clear();
  envelope.stats.top_speed_constraints.reserve(top_constraint_count);
  for (std::size_t i = 0U; i < top_constraint_count; ++i) {
    const std::string prefix = speedProfileTopConstraintPrefix(i);
    SpeedProfileConstraintDiagnostic diagnostic{};
    parseJsonSize(json, prefix + "sample_index", diagnostic.sample_index);
    parseJsonDouble(json, prefix + "s_m", diagnostic.s_m);
    parseJsonDouble(json, prefix + "radius_m", diagnostic.radius_m);
    parseJsonDouble(json, prefix + "curvature_1pm", diagnostic.curvature_1pm);
    parseJsonDouble(json, prefix + "speed_limit_mps", diagnostic.speed_limit_mps);
    parseJsonDouble(json, prefix + "profiled_limit_mps", diagnostic.profiled_limit_mps);
    if (const std::optional<std::string_view> source =
            jsonValueForKey(json, prefix + "source");
        source.has_value()) {
      diagnostic.source = parseSpeedConstraintTypeName(*source);
    }
    parseJsonBool(json, prefix + "isolated_curvature_spike",
                  diagnostic.isolated_curvature_spike);
    envelope.stats.top_speed_constraints.push_back(diagnostic);
  }
  parseJsonSize(json, "isolated_curvature_spike_candidates",
                envelope.stats.isolated_curvature_spike_candidates);
  parseJsonSize(json, "isolated_curvature_spikes_smoothed_geometry",
                envelope.stats.isolated_curvature_spikes_smoothed_geometry);
  parseJsonDouble(json, "isolated_curvature_spike_max_before_1pm",
                  envelope.stats.isolated_curvature_spike_max_before_1pm);
  parseJsonDouble(json, "isolated_curvature_spike_max_after_1pm",
                  envelope.stats.isolated_curvature_spike_max_after_1pm);
  parseJsonDouble(json, "trajectory_total_duration_ms",
                  envelope.stats.total_duration_ms);
  parseJsonDouble(json, "trajectory_corridor_duration_ms",
                  envelope.stats.corridor_duration_ms);
  parseJsonDouble(json, "trajectory_trajectory_optimizer_duration_ms",
                  envelope.stats.trajectory_optimizer_duration_ms);
  parseJsonDouble(json, "trajectory_turn_smoothing_duration_ms",
                  envelope.stats.turn_smoothing_duration_ms);
  parseJsonDouble(json, "trajectory_speed_profile_duration_ms",
                  envelope.stats.speed_profile_duration_ms);

  CorridorStats& corridor = envelope.stats.corridor;
  parseJsonSize(json, "corridor_input_points", corridor.input_points);
  parseJsonSize(json, "corridor_samples", corridor.samples);
  parseJsonSize(json, "corridor_route_prohibited_samples",
                corridor.route_prohibited_samples);
  parseJsonSize(json, "corridor_center_recovered_samples",
                corridor.center_recovered_samples);
  parseJsonSize(json, "corridor_center_unrecoverable_samples",
                corridor.center_unrecoverable_samples);
  parseJsonSize(json, "corridor_outside_grid_samples", corridor.outside_grid_samples);
  parseJsonSize(json, "corridor_lateral_limited_samples",
                corridor.lateral_limited_samples);
  parseJsonDouble(json, "corridor_width_min_m", corridor.min_width_m);
  parseJsonDouble(json, "corridor_width_mean_m", corridor.mean_width_m);
  parseJsonDouble(json, "corridor_width_max_m", corridor.max_width_m);
  parseJsonDouble(json, "corridor_clearance_min_m", corridor.min_clearance_m);
  parseJsonDouble(json, "corridor_clearance_mean_m", corridor.mean_clearance_m);
  parseJsonDouble(json, "corridor_clearance_max_m", corridor.max_clearance_m);
  parseJsonDouble(json, "corridor_center_recovery_max_m",
                  corridor.max_center_recovery_m);
  parseJsonDouble(json, "corridor_lateral_reduction_max_m",
                  corridor.max_lateral_bound_reduction_m);
  parseJsonSize(json, "corridor_parallel_workers_used", corridor.parallel_workers_used);
  parseJsonBool(json, "corridor_samples_reused", corridor.samples_reused);
  parseJsonSize(json, "corridor_reused_samples", corridor.reused_samples);
  (void)parseJsonUint64(json, "corridor_route_fingerprint", corridor.route_fingerprint);
  (void)parseJsonUint64(json, "corridor_config_fingerprint",
                        corridor.config_fingerprint);
  (void)parseJsonUint64(json, "corridor_grid_cells_hash",
                        corridor.prohibited_grid_fingerprint.cells_hash);
  (void)parseJsonUint64(json, "corridor_grid_inflated_hash",
                        corridor.prohibited_grid_fingerprint.inflated_hash);
  parseJsonDouble(json, "corridor_sample_build_duration_ms",
                  corridor.sample_build_duration_ms);
  parseJsonDouble(json, "corridor_raycast_duration_ms", corridor.raycast_duration_ms);
  parseJsonDouble(json, "corridor_lateral_limit_duration_ms",
                  corridor.lateral_limit_duration_ms);
  parseJsonDouble(json, "corridor_clearance_field_build_ms",
                  corridor.clearance_field_build_duration_ms);
  parseJsonBool(json, "clearance_field_reused_by_corridor",
                corridor.clearance_field_reused);
  parseJsonBool(json, "corridor_clearance_field_cache_hit",
                corridor.clearance_field_cache_hit);

  TrajectoryOptimizerStats& optimizer = envelope.stats.trajectory_optimizer;
  parseJsonSize(json, "trajectory_optimizer_input_samples", optimizer.input_samples);
  parseJsonSize(json, "trajectory_optimizer_optimizer_samples",
                optimizer.optimizer_samples);
  parseJsonSize(json, "trajectory_optimizer_output_samples", optimizer.output_samples);
  parseJsonSize(json, "trajectory_optimizer_iterations", optimizer.iterations);
  parseJsonSize(json, "trajectory_optimizer_candidate_evaluations",
                optimizer.candidate_evaluations);
  parseJsonSize(json, "trajectory_optimizer_collision_rejections",
                optimizer.collision_rejections);
  parseJsonDouble(json, "trajectory_optimizer_cost_initial", optimizer.initial_cost);
  parseJsonDouble(json, "trajectory_optimizer_cost_final", optimizer.final_cost);
  parseJsonDouble(json, "trajectory_optimizer_centerline_length_m",
                  optimizer.centerline_length_m);
  parseJsonDouble(json, "trajectory_optimizer_final_length_m",
                  optimizer.final_length_m);
  parseJsonDouble(json, "trajectory_optimizer_final_length_ratio",
                  optimizer.final_length_ratio);
  parseJsonDouble(json, "trajectory_optimizer_cost_curvature",
                  optimizer.cost_curvature);
  parseJsonDouble(json, "trajectory_optimizer_cost_curvature_change",
                  optimizer.cost_curvature_change);
  parseJsonDouble(json, "trajectory_optimizer_cost_radius_shortfall",
                  optimizer.cost_radius_shortfall);
  parseJsonDouble(json, "trajectory_optimizer_cost_heading_jump",
                  optimizer.cost_heading_jump);
  parseJsonDouble(json, "trajectory_optimizer_cost_offset_change",
                  optimizer.cost_offset_change);
  parseJsonDouble(json, "trajectory_optimizer_cost_offset_second_change",
                  optimizer.cost_offset_second_change);
  parseJsonDouble(json, "trajectory_optimizer_cost_offset_slope",
                  optimizer.cost_offset_slope);
  parseJsonDouble(json, "trajectory_optimizer_cost_collision",
                  optimizer.cost_collision);
  parseJsonDouble(json, "trajectory_optimizer_cost_outside_grid",
                  optimizer.cost_outside_grid);
  parseJsonDouble(json, "trajectory_optimizer_final_estimated_time_s",
                  optimizer.estimated_time_s);
  parseJsonDouble(json, "trajectory_optimizer_final_min_speed_limit_mps",
                  optimizer.min_speed_limit_mps);
  parseJsonDouble(json, "trajectory_optimizer_final_max_speed_limit_mps",
                  optimizer.max_speed_limit_mps);
  parseJsonSize(json, "trajectory_optimizer_final_curvature_limited_samples",
                optimizer.curvature_limited_samples);
  parseJsonDouble(json, "trajectory_optimizer_best_candidate_score",
                  optimizer.best_candidate_score);
  parseJsonSize(json, "trajectory_optimizer_regularization_iterations",
                optimizer.regularization_iterations);
  parseJsonBool(json, "trajectory_optimizer_regularization_applied",
                optimizer.regularization_applied);
  parseJsonDouble(json,
                  "trajectory_optimizer_pre_regularization_max_curvature_jump_1pm",
                  optimizer.pre_regularization_max_curvature_jump_1pm);
  parseJsonDouble(json,
                  "trajectory_optimizer_post_regularization_max_curvature_jump_1pm",
                  optimizer.post_regularization_max_curvature_jump_1pm);
  parseJsonSize(json, "trajectory_optimizer_skipped_noop_candidates",
                optimizer.skipped_noop_candidates);
  parseJsonDouble(json, "trajectory_optimizer_candidate_path_evaluation_duration_ms",
                  optimizer.candidate_path_evaluation_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_score_duration_ms",
                  optimizer.candidate_score_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_point_build_duration_ms",
                  optimizer.candidate_point_build_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_sample_build_duration_ms",
                  optimizer.candidate_sample_build_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_cost_breakdown_duration_ms",
                  optimizer.candidate_cost_breakdown_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_shape_diagnostics_duration_ms",
                  optimizer.candidate_shape_diagnostics_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_regularization_duration_ms",
                  optimizer.regularization_duration_ms);
  parseJsonSize(json, "trajectory_optimizer_scratch_reused_candidates",
                optimizer.scratch_reused_candidates);
  parseJsonBool(json, "trajectory_optimizer_parallel_candidate_evaluation_used",
                optimizer.parallel_candidate_evaluation_used);
  parseJsonSize(json, "trajectory_optimizer_parallel_workers_used",
                optimizer.parallel_workers_used);
  parseJsonSize(json, "trajectory_optimizer_candidate_chunks",
                optimizer.candidate_chunks);
  parseJsonSize(json, "trajectory_optimizer_candidate_parallel_batches",
                optimizer.candidate_parallel_batches);
  parseJsonSize(json, "trajectory_optimizer_candidate_threads_launched",
                optimizer.candidate_threads_launched);
  parseJsonDouble(json, "trajectory_optimizer_candidate_batch_wall_duration_ms",
                  optimizer.candidate_batch_wall_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_batch_wait_duration_ms",
                  optimizer.candidate_batch_wait_duration_ms);
  parseJsonDouble(json,
                  "trajectory_optimizer_candidate_worker_buffer_prepare_duration_ms",
                  optimizer.candidate_worker_buffer_prepare_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_thread_launch_duration_ms",
                  optimizer.candidate_thread_launch_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_thread_join_wait_duration_ms",
                  optimizer.candidate_thread_join_wait_duration_ms);
  parseJsonSize(json, "trajectory_optimizer_worker_scratch_reuses",
                optimizer.worker_scratch_reuses);
  parseJsonSize(json, "trajectory_optimizer_candidate_snapshot_allocations_avoided",
                optimizer.candidate_snapshot_allocations_avoided);
  parseJsonSize(json, "trajectory_optimizer_candidate_offset_changed_samples_total",
                optimizer.candidate_offset_changed_samples_total);
  parseJsonSize(json, "trajectory_optimizer_candidate_offset_changed_samples_max",
                optimizer.candidate_offset_changed_samples_max);
  parseJsonSize(json,
                "trajectory_optimizer_candidate_offset_changed_span_samples_total",
                optimizer.candidate_offset_changed_span_samples_total);
  parseJsonSize(json, "trajectory_optimizer_candidate_offset_changed_span_samples_max",
                optimizer.candidate_offset_changed_span_samples_max);
  parseJsonSize(json, "trajectory_optimizer_candidate_local_speed_window_samples_total",
                optimizer.candidate_local_speed_window_samples_total);
  parseJsonSize(json, "trajectory_optimizer_candidate_local_speed_window_samples_max",
                optimizer.candidate_local_speed_window_samples_max);
  parseJsonSize(json, "trajectory_optimizer_local_candidate_evaluations",
                optimizer.local_candidate_evaluations);
  parseJsonSize(json, "trajectory_optimizer_local_candidate_full_score_fallbacks",
                optimizer.local_candidate_full_score_fallbacks);
  parseJsonSize(json, "trajectory_optimizer_local_candidate_full_score_required",
                optimizer.local_candidate_full_score_required);
  parseJsonSize(
      json, "trajectory_optimizer_local_candidate_full_score_required_invalid_input",
      optimizer.local_candidate_full_score_required_invalid_input);
  parseJsonSize(json,
                "trajectory_optimizer_local_candidate_full_score_required_boundary",
                optimizer.local_candidate_full_score_required_boundary);
  parseJsonSize(json,
                "trajectory_optimizer_local_candidate_full_score_required_unsafe_base",
                optimizer.local_candidate_full_score_required_unsafe_base);
  parseJsonSize(
      json, "trajectory_optimizer_local_candidate_full_score_required_window_invalid",
      optimizer.local_candidate_full_score_required_window_invalid);
  parseJsonSize(json, "trajectory_optimizer_local_candidate_acceptance_full_scores",
                optimizer.local_candidate_acceptance_full_scores);
  parseJsonSize(json, "trajectory_optimizer_local_score_false_positives",
                optimizer.local_score_false_positives);
  parseJsonDouble(json, "trajectory_optimizer_local_candidate_point_build_duration_ms",
                  optimizer.local_candidate_point_build_duration_ms);
  parseJsonDouble(json,
                  "trajectory_optimizer_local_candidate_path_evaluation_duration_ms",
                  optimizer.local_candidate_path_evaluation_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_local_candidate_score_duration_ms",
                  optimizer.local_candidate_score_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_full_candidate_score_duration_ms",
                  optimizer.full_candidate_score_duration_ms);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_evaluations",
                optimizer.shadow_segment_score_evaluations);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_unavailable",
                optimizer.shadow_segment_score_unavailable);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_prunable",
                optimizer.shadow_segment_score_prunable);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_false_prunes",
                optimizer.shadow_segment_score_false_prunes);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_winner_mismatches",
                optimizer.shadow_segment_score_winner_mismatches);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_window_samples_total",
                optimizer.shadow_segment_score_window_samples_total);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_window_samples_max",
                optimizer.shadow_segment_score_window_samples_max);
  parseJsonDouble(json, "trajectory_optimizer_shadow_segment_score_abs_error_sum",
                  optimizer.shadow_segment_score_abs_error_sum);
  parseJsonDouble(json, "trajectory_optimizer_shadow_segment_score_abs_error_p95",
                  optimizer.shadow_segment_score_abs_error_p95);
  parseJsonDouble(json, "trajectory_optimizer_shadow_segment_score_max_overestimate",
                  optimizer.shadow_segment_score_max_overestimate);
  parseJsonDouble(json, "trajectory_optimizer_shadow_segment_score_max_underestimate",
                  optimizer.shadow_segment_score_max_underestimate);
  parseJsonDouble(
      json,
      "trajectory_optimizer_shadow_segment_score_max_false_prune_improvement_score",
      optimizer.shadow_segment_score_max_false_prune_improvement_score);
  parseJsonSize(json, "trajectory_optimizer_shadow_boundary_clamped_local_candidates",
                optimizer.shadow_boundary_clamped_local_candidates);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_boundary_clamped_window_samples_total",
                optimizer.shadow_boundary_clamped_window_samples_total);
  parseJsonSize(json, "trajectory_optimizer_shadow_boundary_clamped_window_samples_max",
                optimizer.shadow_boundary_clamped_window_samples_max);
  parseJsonSize(json, "trajectory_optimizer_window_count", optimizer.window_count);
  parseJsonSize(json, "trajectory_optimizer_active_window_count",
                optimizer.active_window_count);
  parseJsonSize(json, "trajectory_optimizer_active_window_samples",
                optimizer.active_window_samples);
  parseJsonSize(json, "trajectory_optimizer_active_window_centerline_blocked",
                optimizer.active_window_centerline_blocked);
  parseJsonSize(json, "trajectory_optimizer_active_window_heading_change_samples",
                optimizer.active_window_heading_change_samples);
  parseJsonSize(json, "trajectory_optimizer_active_window_heading_span_samples",
                optimizer.active_window_heading_span_samples);
  parseJsonSize(json, "trajectory_optimizer_active_window_curvature_samples",
                optimizer.active_window_curvature_samples);
  parseJsonSize(json, "trajectory_optimizer_active_window_width_change_samples",
                optimizer.active_window_width_change_samples);
  parseJsonSize(json, "trajectory_optimizer_active_window_width_asymmetry_samples",
                optimizer.active_window_width_asymmetry_samples);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_active_window_no_width_asymmetry_count",
                optimizer.shadow_active_window_no_width_asymmetry_count);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_active_window_no_width_asymmetry_samples",
                optimizer.shadow_active_window_no_width_asymmetry_samples);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_active_window_no_width_triggers_count",
                optimizer.shadow_active_window_no_width_triggers_count);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_active_window_no_width_triggers_samples",
                optimizer.shadow_active_window_no_width_triggers_samples);
  parseJsonSize(json, "trajectory_optimizer_shadow_active_window_no_heading_span_count",
                optimizer.shadow_active_window_no_heading_span_count);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_active_window_no_heading_span_samples",
                optimizer.shadow_active_window_no_heading_span_samples);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_windows",
                optimizer.centerline_blocked_windows);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_window_samples",
                optimizer.centerline_blocked_window_samples);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_window_merged_count",
                optimizer.centerline_blocked_window_merged_count);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_prohibited_cells",
                optimizer.centerline_blocked_prohibited_cells);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_outside_grid_segments",
                optimizer.centerline_blocked_outside_grid_segments);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_segment_count",
                optimizer.centerline_blocked_segment_count);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_span_count",
                optimizer.centerline_blocked_span_count);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_first_segment_index",
                optimizer.centerline_blocked_first_segment_index);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_last_segment_index",
                optimizer.centerline_blocked_last_segment_index);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_first_s_m",
                  optimizer.centerline_blocked_first_s_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_last_s_m",
                  optimizer.centerline_blocked_last_s_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_span_length_m",
                  optimizer.centerline_blocked_span_length_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_first_x_m",
                  optimizer.centerline_blocked_first_x_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_first_y_m",
                  optimizer.centerline_blocked_first_y_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_last_x_m",
                  optimizer.centerline_blocked_last_x_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_last_y_m",
                  optimizer.centerline_blocked_last_y_m);
  parseJsonBool(json, "trajectory_optimizer_centerline_blocked_first_outside_grid",
                optimizer.centerline_blocked_first_outside_grid);
  parseJsonBool(json, "trajectory_optimizer_centerline_blocked_last_outside_grid",
                optimizer.centerline_blocked_last_outside_grid);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_span_diagnostic_count",
                optimizer.centerline_blocked_span_diagnostic_count);
  for (std::size_t i = 0U; i < kMaxCenterlineBlockedSpanDiagnostics; ++i) {
    const std::string prefix =
        "trajectory_optimizer_centerline_blocked_span" + std::to_string(i);
    TrajectoryOptimizerBlockedSpanDiagnostic& span =
        optimizer.centerline_blocked_span_diagnostics.at(i);
    parseJsonSize(json, prefix + "_begin_segment_index", span.begin_segment_index);
    parseJsonSize(json, prefix + "_end_segment_index", span.end_segment_index);
    parseJsonDouble(json, prefix + "_begin_s_m", span.begin_s_m);
    parseJsonDouble(json, prefix + "_end_s_m", span.end_s_m);
    parseJsonDouble(json, prefix + "_length_m", span.length_m);
    parseJsonDouble(json, prefix + "_begin_x_m", span.begin_x_m);
    parseJsonDouble(json, prefix + "_begin_y_m", span.begin_y_m);
    parseJsonDouble(json, prefix + "_end_x_m", span.end_x_m);
    parseJsonDouble(json, prefix + "_end_y_m", span.end_y_m);
    parseJsonSize(json, prefix + "_prohibited_cells", span.prohibited_cells);
    parseJsonSize(json, prefix + "_outside_grid_segments", span.outside_grid_segments);
  }
  parseJsonSize(json, "trajectory_optimizer_dp_states", optimizer.dp_states);
  parseJsonSize(json, "trajectory_optimizer_dp_transitions", optimizer.dp_transitions);
  parseJsonSize(json, "trajectory_optimizer_dp_segment_cache_hits",
                optimizer.dp_segment_cache_hits);
  parseJsonSize(json, "trajectory_optimizer_dp_segment_cache_misses",
                optimizer.dp_segment_cache_misses);
  parseJsonSize(json, "trajectory_optimizer_candidate_segment_cache_hits",
                optimizer.candidate_segment_cache_hits);
  parseJsonSize(json, "trajectory_optimizer_candidate_segment_cache_misses",
                optimizer.candidate_segment_cache_misses);
  parseJsonSize(json, "trajectory_optimizer_full_path_segment_cache_hits",
                optimizer.full_path_segment_cache_hits);
  parseJsonSize(json, "trajectory_optimizer_full_path_segment_cache_misses",
                optimizer.full_path_segment_cache_misses);
  parseJsonSize(json, "trajectory_optimizer_dp_coarse_states",
                optimizer.dp_coarse_states);
  parseJsonSize(json, "trajectory_optimizer_dp_coarse_transitions",
                optimizer.dp_coarse_transitions);
  parseJsonSize(json, "trajectory_optimizer_dp_fine_states", optimizer.dp_fine_states);
  parseJsonSize(json, "trajectory_optimizer_dp_fine_transitions",
                optimizer.dp_fine_transitions);
  parseJsonBool(json, "trajectory_optimizer_dp_coarse_to_fine_used",
                optimizer.dp_coarse_to_fine_used);
  parseJsonDouble(json, "trajectory_optimizer_window_detection_duration_ms",
                  optimizer.window_detection_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_window_eval_duration_ms",
                  optimizer.window_eval_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_dp_duration_ms",
                  optimizer.dp_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_full_final_score_duration_ms",
                  optimizer.full_final_score_duration_ms);
  parseJsonBool(json, "trajectory_optimizer_async_refined", optimizer.async_refined);
  parseJsonDouble(json, "trajectory_optimizer_max_abs_offset_m",
                  optimizer.max_abs_offset_m);
  parseJsonDouble(json, "trajectory_optimizer_min_edge_margin_m",
                  optimizer.min_edge_margin_m);
  parseJsonDouble(json, "trajectory_optimizer_mean_edge_margin_m",
                  optimizer.mean_edge_margin_m);

  TurnSmoothingStats& turn_smoothing = envelope.stats.turn_smoothing;
  parseJsonSize(json, "turn_smoothing_input_samples", turn_smoothing.input_samples);
  parseJsonSize(json, "turn_smoothing_output_samples", turn_smoothing.output_samples);
  parseJsonSize(json, "turn_smoothing_detected_corners",
                turn_smoothing.detected_corners);
  parseJsonSize(json, "turn_smoothing_attempted_corners",
                turn_smoothing.attempted_corners);
  parseJsonSize(json, "turn_smoothing_candidate_attempts",
                turn_smoothing.candidate_attempts);
  parseJsonSize(json, "turn_smoothing_relaxed_candidate_attempts",
                turn_smoothing.relaxed_candidate_attempts);
  parseJsonSize(json, "turn_smoothing_bezier_cache_hits",
                turn_smoothing.bezier_cache_hits);
  parseJsonSize(json, "turn_smoothing_bezier_cache_misses",
                turn_smoothing.bezier_cache_misses);
  parseJsonSize(json, "turn_smoothing_before_metrics_cache_hits",
                turn_smoothing.before_metrics_cache_hits);
  parseJsonSize(json, "turn_smoothing_before_metrics_cache_misses",
                turn_smoothing.before_metrics_cache_misses);
  parseJsonSize(json, "turn_smoothing_traversability_cache_hits",
                turn_smoothing.traversability_cache_hits);
  parseJsonSize(json, "turn_smoothing_traversability_cache_misses",
                turn_smoothing.traversability_cache_misses);
  parseJsonSize(json, "turn_smoothing_smoothed_corners",
                turn_smoothing.smoothed_corners);
  parseJsonSize(json, "turn_smoothing_rejected_prohibited",
                turn_smoothing.rejected_prohibited);
  parseJsonSize(json, "turn_smoothing_rejected_corridor",
                turn_smoothing.rejected_corridor);
  parseJsonSize(json, "turn_smoothing_rejected_not_improved",
                turn_smoothing.rejected_not_improved);
  parseJsonSize(json, "turn_smoothing_rejected_curvature_regression",
                turn_smoothing.rejected_curvature_regression);
  parseJsonSize(json, "turn_smoothing_rejected_radius_regression",
                turn_smoothing.rejected_radius_regression);
  parseJsonDouble(json, "turn_smoothing_heading_delta_before_rad",
                  turn_smoothing.max_heading_delta_before_rad);
  parseJsonDouble(json, "turn_smoothing_heading_delta_after_rad",
                  turn_smoothing.max_heading_delta_after_rad);
  parseJsonDouble(json, "turn_smoothing_curvature_jump_before_1pm",
                  turn_smoothing.max_curvature_jump_before_1pm);
  parseJsonDouble(json, "turn_smoothing_curvature_jump_after_1pm",
                  turn_smoothing.max_curvature_jump_after_1pm);
  parseJsonDouble(json, "turn_smoothing_min_inner_margin_m",
                  turn_smoothing.min_inner_margin_m);
  parseJsonDouble(json, "turn_smoothing_max_outer_shift_m",
                  turn_smoothing.max_applied_outer_shift_m);
  parseJsonDouble(json, "turn_smoothing_accepted_entry_distance_m",
                  turn_smoothing.accepted_entry_distance_m);
  parseJsonDouble(json, "turn_smoothing_accepted_exit_distance_m",
                  turn_smoothing.accepted_exit_distance_m);
  parseJsonDouble(json, "turn_smoothing_accepted_shift_scale",
                  turn_smoothing.accepted_shift_scale);
  parseJsonDouble(json, "turn_smoothing_accepted_relaxed_angle_deg",
                  turn_smoothing.accepted_relaxed_angle_deg);
  parseJsonDouble(json, "turn_smoothing_accepted_score", turn_smoothing.accepted_score);
  parseJsonDouble(json, "turn_smoothing_accepted_min_radius_before_m",
                  turn_smoothing.accepted_min_radius_before_m);
  parseJsonDouble(json, "turn_smoothing_accepted_min_radius_after_m",
                  turn_smoothing.accepted_min_radius_after_m);
  parseJsonDouble(json, "turn_smoothing_accepted_min_speed_before_mps",
                  turn_smoothing.accepted_min_speed_before_mps);
  parseJsonDouble(json, "turn_smoothing_accepted_min_speed_after_mps",
                  turn_smoothing.accepted_min_speed_after_mps);
  parseJsonDouble(json, "turn_smoothing_accepted_local_time_before_s",
                  turn_smoothing.accepted_local_time_before_s);
  parseJsonDouble(json, "turn_smoothing_accepted_local_time_after_s",
                  turn_smoothing.accepted_local_time_after_s);
  parseJsonDouble(json, "turn_smoothing_candidate_build_duration_ms",
                  turn_smoothing.candidate_build_duration_ms);
  parseJsonDouble(json, "turn_smoothing_candidate_replace_duration_ms",
                  turn_smoothing.candidate_replace_duration_ms);
  parseJsonDouble(json, "turn_smoothing_collision_check_duration_ms",
                  turn_smoothing.collision_check_duration_ms);
  parseJsonDouble(json, "turn_smoothing_metrics_duration_ms",
                  turn_smoothing.metrics_duration_ms);
  parseJsonDouble(json, "turn_smoothing_shape_diagnostics_duration_ms",
                  turn_smoothing.shape_diagnostics_duration_ms);
  parseJsonDouble(json, "turn_smoothing_speed_profile_duration_ms",
                  turn_smoothing.speed_profile_duration_ms);

  return envelope;
}

} // namespace drone_city_nav
