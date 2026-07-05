#pragma once

#include "drone_city_nav/trajectory_diagnostics_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace drone_city_nav {
namespace trajectory_diagnostics_io_test_helpers {

template<std::size_t Size>
void expectContainsAll(const std::string& text,
                       const std::array<const char*, Size>& expected_tokens) {
  for (const char* token : expected_tokens) {
    EXPECT_NE(text.find(token), std::string::npos) << token;
  }
}

[[nodiscard]] inline TrajectoryPlannerStats populatedStats() {
  TrajectoryPlannerStats stats{};
  stats.quality = TrajectoryQuality::kRefined;
  stats.trajectory_optimizer.estimated_time_s = 12.5;
  stats.trajectory_optimizer.min_speed_limit_mps = 1.0;
  stats.trajectory_optimizer.max_speed_limit_mps = 10.0;
  stats.trajectory_optimizer.curvature_limited_samples = 3U;
  stats.trajectory_optimizer.best_candidate_score = 42.0;
  stats.trajectory_optimizer.regularization_applied = true;
  stats.trajectory_optimizer.regularization_iterations = 2U;
  stats.trajectory_optimizer.pre_regularization_max_curvature_jump_1pm = 0.4;
  stats.trajectory_optimizer.post_regularization_max_curvature_jump_1pm = 0.2;
  stats.trajectory_optimizer.centerline_length_m = 100.0;
  stats.trajectory_optimizer.final_length_m = 108.0;
  stats.trajectory_optimizer.final_length_ratio = 1.08;
  stats.trajectory_optimizer.max_abs_offset_m = 3.0;
  stats.trajectory_optimizer.min_edge_margin_m = 2.5;
  stats.trajectory_optimizer.mean_edge_margin_m = 4.5;
  stats.trajectory_optimizer.candidate_path_evaluation_duration_ms = 7.25;
  stats.trajectory_optimizer.candidate_score_duration_ms = 8.5;
  stats.trajectory_optimizer.candidate_point_build_duration_ms = 1.25;
  stats.trajectory_optimizer.candidate_sample_build_duration_ms = 2.5;
  stats.trajectory_optimizer.candidate_cost_breakdown_duration_ms = 3.25;
  stats.trajectory_optimizer.candidate_shape_diagnostics_duration_ms = 1.75;
  stats.trajectory_optimizer.regularization_duration_ms = 3.75;
  stats.trajectory_optimizer.scratch_reused_candidates = 13U;
  stats.trajectory_optimizer.parallel_candidate_evaluation_used = true;
  stats.trajectory_optimizer.parallel_workers_used = 2U;
  stats.trajectory_optimizer.candidate_chunks = 31U;
  stats.trajectory_optimizer.candidate_parallel_batches = 29U;
  stats.trajectory_optimizer.candidate_threads_launched = 58U;
  stats.trajectory_optimizer.candidate_batch_wall_duration_ms = 12.25;
  stats.trajectory_optimizer.candidate_batch_wait_duration_ms = 10.5;
  stats.trajectory_optimizer.candidate_worker_buffer_prepare_duration_ms = 1.5;
  stats.trajectory_optimizer.candidate_thread_launch_duration_ms = 2.75;
  stats.trajectory_optimizer.candidate_thread_join_wait_duration_ms = 8.0;
  stats.trajectory_optimizer.worker_scratch_reuses = 62U;
  stats.trajectory_optimizer.candidate_snapshot_allocations_avoided = 60U;
  stats.trajectory_optimizer.candidate_offset_changed_samples_total = 180U;
  stats.trajectory_optimizer.candidate_offset_changed_samples_max = 7U;
  stats.trajectory_optimizer.candidate_offset_changed_span_samples_total = 220U;
  stats.trajectory_optimizer.candidate_offset_changed_span_samples_max = 9U;
  stats.trajectory_optimizer.candidate_local_speed_window_samples_total = 930U;
  stats.trajectory_optimizer.candidate_local_speed_window_samples_max = 35U;
  stats.trajectory_optimizer.local_candidate_evaluations = 61U;
  stats.trajectory_optimizer.local_candidate_full_score_fallbacks = 55U;
  stats.trajectory_optimizer.local_candidate_full_score_required = 10U;
  stats.trajectory_optimizer.local_candidate_full_score_required_invalid_input = 1U;
  stats.trajectory_optimizer.local_candidate_full_score_required_boundary = 2U;
  stats.trajectory_optimizer.local_candidate_full_score_required_unsafe_base = 3U;
  stats.trajectory_optimizer.local_candidate_full_score_required_window_invalid = 4U;
  stats.trajectory_optimizer.local_candidate_acceptance_full_scores = 7U;
  stats.trajectory_optimizer.local_score_false_positives = 1U;
  stats.trajectory_optimizer.local_candidate_point_build_duration_ms = 1.1;
  stats.trajectory_optimizer.local_candidate_path_evaluation_duration_ms = 2.2;
  stats.trajectory_optimizer.local_candidate_score_duration_ms = 4.5;
  stats.trajectory_optimizer.full_candidate_score_duration_ms = 6.75;
  stats.trajectory_optimizer.shadow_segment_score_evaluations = 52U;
  stats.trajectory_optimizer.shadow_segment_score_unavailable = 9U;
  stats.trajectory_optimizer.shadow_segment_score_prunable = 19U;
  stats.trajectory_optimizer.shadow_segment_score_false_prunes = 1U;
  stats.trajectory_optimizer.shadow_segment_score_winner_mismatches = 3U;
  stats.trajectory_optimizer.shadow_segment_score_window_samples_total = 572U;
  stats.trajectory_optimizer.shadow_segment_score_window_samples_max = 11U;
  stats.trajectory_optimizer.shadow_segment_score_abs_error_sum = 0.35;
  stats.trajectory_optimizer.shadow_segment_score_abs_error_p95 = 0.05;
  stats.trajectory_optimizer.shadow_segment_score_max_overestimate = 0.2;
  stats.trajectory_optimizer.shadow_segment_score_max_underestimate = 0.15;
  stats.trajectory_optimizer.shadow_segment_score_max_false_prune_improvement_score =
      0.75;
  stats.trajectory_optimizer.shadow_boundary_clamped_local_candidates = 11U;
  stats.trajectory_optimizer.shadow_boundary_clamped_window_samples_total = 121U;
  stats.trajectory_optimizer.shadow_boundary_clamped_window_samples_max = 13U;
  stats.trajectory_optimizer.window_count = 4U;
  stats.trajectory_optimizer.active_window_count = 3U;
  stats.trajectory_optimizer.active_window_samples = 18U;
  stats.trajectory_optimizer.active_window_centerline_blocked = 1U;
  stats.trajectory_optimizer.active_window_heading_change_samples = 5U;
  stats.trajectory_optimizer.active_window_heading_span_samples = 6U;
  stats.trajectory_optimizer.active_window_curvature_samples = 7U;
  stats.trajectory_optimizer.active_window_width_change_samples = 8U;
  stats.trajectory_optimizer.active_window_width_asymmetry_samples = 9U;
  stats.trajectory_optimizer.shadow_active_window_no_width_asymmetry_count = 2U;
  stats.trajectory_optimizer.shadow_active_window_no_width_asymmetry_samples = 16U;
  stats.trajectory_optimizer.shadow_active_window_no_width_triggers_count = 1U;
  stats.trajectory_optimizer.shadow_active_window_no_width_triggers_samples = 12U;
  stats.trajectory_optimizer.shadow_active_window_no_heading_span_count = 3U;
  stats.trajectory_optimizer.shadow_active_window_no_heading_span_samples = 14U;
  stats.trajectory_optimizer.centerline_blocked_windows = 5U;
  stats.trajectory_optimizer.centerline_blocked_window_samples = 19U;
  stats.trajectory_optimizer.centerline_blocked_window_merged_count = 3U;
  stats.trajectory_optimizer.centerline_blocked_prohibited_cells = 10U;
  stats.trajectory_optimizer.centerline_blocked_outside_grid_segments = 11U;
  stats.trajectory_optimizer.centerline_blocked_segment_count = 3U;
  stats.trajectory_optimizer.centerline_blocked_span_count = 2U;
  stats.trajectory_optimizer.centerline_blocked_first_segment_index = 12U;
  stats.trajectory_optimizer.centerline_blocked_last_segment_index = 14U;
  stats.trajectory_optimizer.centerline_blocked_first_s_m = 42.5;
  stats.trajectory_optimizer.centerline_blocked_last_s_m = 48.75;
  stats.trajectory_optimizer.centerline_blocked_span_length_m = 6.25;
  stats.trajectory_optimizer.centerline_blocked_first_x_m = 13.25;
  stats.trajectory_optimizer.centerline_blocked_first_y_m = -8.75;
  stats.trajectory_optimizer.centerline_blocked_last_x_m = 16.5;
  stats.trajectory_optimizer.centerline_blocked_last_y_m = -9.25;
  stats.trajectory_optimizer.centerline_blocked_first_outside_grid = true;
  stats.trajectory_optimizer.centerline_blocked_last_outside_grid = true;
  stats.trajectory_optimizer.centerline_blocked_span_diagnostic_count = 2U;
  stats.trajectory_optimizer.centerline_blocked_span_diagnostics[0] =
      TrajectoryOptimizerBlockedSpanDiagnostic{.begin_segment_index = 12U,
                                               .end_segment_index = 13U,
                                               .begin_s_m = 42.5,
                                               .end_s_m = 45.25,
                                               .length_m = 2.75,
                                               .begin_x_m = 13.25,
                                               .begin_y_m = -8.75,
                                               .end_x_m = 14.5,
                                               .end_y_m = -9.0,
                                               .prohibited_cells = 6U,
                                               .outside_grid_segments = 0U};
  stats.trajectory_optimizer.centerline_blocked_span_diagnostics[1] =
      TrajectoryOptimizerBlockedSpanDiagnostic{.begin_segment_index = 14U,
                                               .end_segment_index = 14U,
                                               .begin_s_m = 48.0,
                                               .end_s_m = 48.75,
                                               .length_m = 0.75,
                                               .begin_x_m = 16.0,
                                               .begin_y_m = -9.0,
                                               .end_x_m = 16.5,
                                               .end_y_m = -9.25,
                                               .prohibited_cells = 4U,
                                               .outside_grid_segments = 1U};
  stats.trajectory_optimizer.dp_states = 144U;
  stats.trajectory_optimizer.dp_transitions = 512U;
  stats.trajectory_optimizer.dp_segment_cache_hits = 10U;
  stats.trajectory_optimizer.dp_segment_cache_misses = 502U;
  stats.trajectory_optimizer.candidate_segment_cache_hits = 3U;
  stats.trajectory_optimizer.candidate_segment_cache_misses = 244U;
  stats.trajectory_optimizer.full_path_segment_cache_hits = 14U;
  stats.trajectory_optimizer.full_path_segment_cache_misses = 88U;
  stats.trajectory_optimizer.dp_coarse_states = 44U;
  stats.trajectory_optimizer.dp_coarse_transitions = 112U;
  stats.trajectory_optimizer.dp_fine_states = 100U;
  stats.trajectory_optimizer.dp_fine_transitions = 400U;
  stats.trajectory_optimizer.dp_coarse_to_fine_used = true;
  stats.trajectory_optimizer.window_detection_duration_ms = 0.75;
  stats.trajectory_optimizer.window_eval_duration_ms = 6.5;
  stats.trajectory_optimizer.dp_duration_ms = 4.25;
  stats.trajectory_optimizer.full_final_score_duration_ms = 2.75;
  stats.trajectory_optimizer.async_refined = true;
  stats.trajectory_optimizer.cost_curvature = 12.0;
  stats.trajectory_optimizer.cost_curvature_change = 3.0;
  stats.trajectory_optimizer.cost_radius_shortfall = 7.5;
  stats.trajectory_optimizer.cost_heading_jump = 5.5;
  stats.trajectory_optimizer.cost_offset_change = 1.0;
  stats.trajectory_optimizer.cost_offset_second_change = 4.0;
  stats.trajectory_optimizer.cost_offset_slope = 2.5;
  stats.trajectory_optimizer.cost_collision = 0.0;
  stats.trajectory_optimizer.cost_outside_grid = 0.0;
  stats.turn_smoothing.input_samples = 48U;
  stats.turn_smoothing.output_samples = 72U;
  stats.turn_smoothing.detected_corners = 3U;
  stats.turn_smoothing.attempted_corners = 2U;
  stats.turn_smoothing.candidate_attempts = 11U;
  stats.turn_smoothing.relaxed_candidate_attempts = 6U;
  stats.turn_smoothing.bezier_cache_hits = 21U;
  stats.turn_smoothing.bezier_cache_misses = 22U;
  stats.turn_smoothing.before_metrics_cache_hits = 23U;
  stats.turn_smoothing.before_metrics_cache_misses = 24U;
  stats.turn_smoothing.traversability_cache_hits = 25U;
  stats.turn_smoothing.traversability_cache_misses = 26U;
  stats.turn_smoothing.smoothed_corners = 1U;
  stats.turn_smoothing.rejected_prohibited = 0U;
  stats.turn_smoothing.rejected_corridor = 1U;
  stats.turn_smoothing.rejected_not_improved = 0U;
  stats.turn_smoothing.rejected_curvature_regression = 2U;
  stats.turn_smoothing.rejected_radius_regression = 3U;
  stats.turn_smoothing.max_heading_delta_before_rad = 1.2;
  stats.turn_smoothing.max_heading_delta_after_rad = 0.4;
  stats.turn_smoothing.max_curvature_jump_before_1pm = 0.5;
  stats.turn_smoothing.max_curvature_jump_after_1pm = 0.2;
  stats.turn_smoothing.min_inner_margin_m = 2.25;
  stats.turn_smoothing.max_applied_outer_shift_m = 6.5;
  stats.turn_smoothing.accepted_entry_distance_m = 30.0;
  stats.turn_smoothing.accepted_exit_distance_m = 30.0;
  stats.turn_smoothing.accepted_shift_scale = 0.5;
  stats.turn_smoothing.accepted_relaxed_angle_deg = 15.0;
  stats.turn_smoothing.accepted_score = 12.5;
  stats.turn_smoothing.accepted_min_radius_before_m = 6.0;
  stats.turn_smoothing.accepted_min_radius_after_m = 9.0;
  stats.turn_smoothing.accepted_min_speed_before_mps = 5.5;
  stats.turn_smoothing.accepted_min_speed_after_mps = 7.0;
  stats.turn_smoothing.accepted_local_time_before_s = 4.2;
  stats.turn_smoothing.accepted_local_time_after_s = 3.7;
  stats.turn_smoothing.candidate_build_duration_ms = 1.1;
  stats.turn_smoothing.candidate_replace_duration_ms = 1.2;
  stats.turn_smoothing.collision_check_duration_ms = 1.3;
  stats.turn_smoothing.metrics_duration_ms = 1.4;
  stats.turn_smoothing.shape_diagnostics_duration_ms = 1.5;
  stats.turn_smoothing.speed_profile_duration_ms = 1.6;
  stats.turn_smoothing.corner_diagnostics.push_back(
      TurnSmoothingCornerDiagnostic{.accepted = true,
                                    .reject_reason = "none",
                                    .corner_s_m = 42.0,
                                    .entry_distance_m = 30.0,
                                    .exit_distance_m = 25.0,
                                    .shift_scale = 0.5,
                                    .relaxed_angle_deg = 10.0,
                                    .score = 12.5,
                                    .min_radius_before_m = 6.0,
                                    .min_radius_after_m = 9.0,
                                    .min_speed_before_mps = 5.5,
                                    .min_speed_after_mps = 7.0,
                                    .local_time_before_s = 4.2,
                                    .local_time_after_s = 3.7,
                                    .curvature_jump_before_1pm = 0.4,
                                    .curvature_jump_after_1pm = 0.2,
                                    .heading_delta_before_rad = 0.8,
                                    .heading_delta_after_rad = 0.3});
  stats.corridor.samples = 42U;
  stats.corridor.min_width_m = 17.5;
  stats.corridor.mean_width_m = 24.25;
  stats.corridor.max_width_m = 58.75;
  stats.corridor.lateral_limited_samples = 9U;
  stats.corridor.max_center_recovery_m = 1.25;
  stats.corridor.max_lateral_bound_reduction_m = 2.5;
  stats.corridor.parallel_workers_used = 4U;
  stats.corridor.samples_reused = true;
  stats.corridor.reused_samples = 42U;
  stats.corridor.route_fingerprint = 0x1234U;
  stats.corridor.config_fingerprint = 0x2345U;
  stats.corridor.prohibited_grid_fingerprint.cells_hash = 0x5678U;
  stats.corridor.prohibited_grid_fingerprint.inflated_hash = 0x9abcU;
  stats.corridor.sample_build_duration_ms = 6.25;
  stats.corridor.raycast_duration_ms = 5.75;
  stats.corridor.lateral_limit_duration_ms = 1.5;
  stats.corridor.clearance_field_build_duration_ms = 0.0;
  stats.corridor.clearance_field_reused = true;
  stats.corridor.clearance_field_cache_hit = false;
  stats.input_points = 8U;
  stats.samples = 78U;
  stats.length_m = 412.25;
  stats.curvature_min_1pm = -0.05;
  stats.curvature_max_1pm = 0.06;
  stats.curvature_mean_abs_1pm = 0.02;
  stats.speed_profile_min_mps = 0.0;
  stats.speed_profile_mean_mps = 13.4;
  stats.speed_profile_max_mps = 19.1;
  stats.speed_profile_curvature_limited_samples = 69U;
  stats.isolated_curvature_spike_candidates = 2U;
  stats.isolated_curvature_spikes_smoothed_geometry = 1U;
  stats.isolated_curvature_spike_max_before_1pm = 0.12;
  stats.isolated_curvature_spike_max_after_1pm = 0.04;
  stats.top_speed_constraints.push_back(SpeedProfileConstraintDiagnostic{
      .sample_index = 17U,
      .s_m = 42.5,
      .radius_m = 9.25,
      .curvature_1pm = 0.108,
      .speed_limit_mps = 6.8,
      .profiled_limit_mps = 6.8,
      .source = SpeedConstraintType::kArc,
      .isolated_curvature_spike = true,
  });
  stats.total_duration_ms = 123.4;
  stats.corridor_duration_ms = 5.5;
  stats.trajectory_optimizer_duration_ms = 99.9;
  stats.turn_smoothing_duration_ms = 8.75;
  stats.speed_profile_duration_ms = 1.5;
  return stats;
}

} // namespace trajectory_diagnostics_io_test_helpers
} // namespace drone_city_nav
