#include "drone_city_nav/trajectory_diagnostics_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace drone_city_nav {
namespace {

template<std::size_t Size>
void expectContainsAll(const std::string& text,
                       const std::array<const char*, Size>& expected_tokens) {
  for (const char* token : expected_tokens) {
    EXPECT_NE(text.find(token), std::string::npos) << token;
  }
}

[[nodiscard]] TrajectoryPlannerStats populatedStats() {
  TrajectoryPlannerStats stats{};
  stats.quality = TrajectoryQuality::kRefined;
  stats.racing_line.estimated_time_s = 12.5;
  stats.racing_line.min_speed_limit_mps = 1.0;
  stats.racing_line.max_speed_limit_mps = 10.0;
  stats.racing_line.curvature_limited_samples = 3U;
  stats.racing_line.centerline_estimated_time_s = 14.0;
  stats.racing_line.centerline_min_speed_limit_mps = 2.0;
  stats.racing_line.centerline_max_speed_limit_mps = 11.0;
  stats.racing_line.centerline_curvature_limited_samples = 4U;
  stats.racing_line.best_candidate_estimated_time_s = 12.25;
  stats.racing_line.best_candidate_score = 42.0;
  stats.racing_line.best_candidate_min_speed_limit_mps = 1.5;
  stats.racing_line.best_candidate_max_speed_limit_mps = 10.5;
  stats.racing_line.best_candidate_curvature_limited_samples = 5U;
  stats.racing_line.time_gain_s = 1.5;
  stats.racing_line.regularization_applied = true;
  stats.racing_line.regularization_iterations = 2U;
  stats.racing_line.regularization_time_delta_s = 0.1;
  stats.racing_line.pre_regularization_max_curvature_jump_1pm = 0.4;
  stats.racing_line.post_regularization_max_curvature_jump_1pm = 0.2;
  stats.racing_line.centerline_length_m = 100.0;
  stats.racing_line.final_length_m = 108.0;
  stats.racing_line.final_length_ratio = 1.08;
  stats.racing_line.max_abs_offset_m = 3.0;
  stats.racing_line.min_edge_margin_m = 2.5;
  stats.racing_line.mean_edge_margin_m = 4.5;
  stats.racing_line.candidate_path_evaluation_duration_ms = 7.25;
  stats.racing_line.candidate_score_duration_ms = 8.5;
  stats.racing_line.candidate_point_build_duration_ms = 1.25;
  stats.racing_line.candidate_sample_build_duration_ms = 2.5;
  stats.racing_line.candidate_cost_breakdown_duration_ms = 3.25;
  stats.racing_line.candidate_shape_diagnostics_duration_ms = 1.75;
  stats.racing_line.candidate_speed_profile_duration_ms = 4.75;
  stats.racing_line.candidate_speed_profile_calls = 8U;
  stats.racing_line.candidate_speed_profile_samples_total = 400U;
  stats.racing_line.candidate_speed_profile_samples_max = 55U;
  stats.racing_line.regularization_duration_ms = 3.75;
  stats.racing_line.scratch_reused_candidates = 13U;
  stats.racing_line.parallel_candidate_evaluation_used = true;
  stats.racing_line.parallel_workers_used = 2U;
  stats.racing_line.candidate_chunks = 31U;
  stats.racing_line.candidate_parallel_batches = 29U;
  stats.racing_line.candidate_threads_launched = 58U;
  stats.racing_line.candidate_batch_wall_duration_ms = 12.25;
  stats.racing_line.candidate_batch_wait_duration_ms = 10.5;
  stats.racing_line.candidate_worker_buffer_prepare_duration_ms = 1.5;
  stats.racing_line.candidate_thread_launch_duration_ms = 2.75;
  stats.racing_line.candidate_thread_join_wait_duration_ms = 8.0;
  stats.racing_line.worker_scratch_reuses = 62U;
  stats.racing_line.candidate_snapshot_allocations_avoided = 60U;
  stats.racing_line.candidate_offset_changed_samples_total = 180U;
  stats.racing_line.candidate_offset_changed_samples_max = 7U;
  stats.racing_line.candidate_offset_changed_span_samples_total = 220U;
  stats.racing_line.candidate_offset_changed_span_samples_max = 9U;
  stats.racing_line.candidate_local_speed_window_samples_total = 930U;
  stats.racing_line.candidate_local_speed_window_samples_max = 35U;
  stats.racing_line.local_candidate_evaluations = 61U;
  stats.racing_line.local_candidate_full_score_fallbacks = 55U;
  stats.racing_line.local_candidate_full_score_required = 10U;
  stats.racing_line.local_candidate_full_score_required_invalid_input = 1U;
  stats.racing_line.local_candidate_full_score_required_boundary = 2U;
  stats.racing_line.local_candidate_full_score_required_unsafe_base = 3U;
  stats.racing_line.local_candidate_full_score_required_window_invalid = 4U;
  stats.racing_line.local_candidate_acceptance_full_scores = 7U;
  stats.racing_line.local_score_false_positives = 1U;
  stats.racing_line.local_candidate_point_build_duration_ms = 1.1;
  stats.racing_line.local_candidate_path_evaluation_duration_ms = 2.2;
  stats.racing_line.local_candidate_score_duration_ms = 4.5;
  stats.racing_line.local_candidate_traversal_estimate_duration_ms = 3.3;
  stats.racing_line.full_candidate_score_duration_ms = 6.75;
  stats.racing_line.shadow_lower_bound_validation_full_scores = 41U;
  stats.racing_line.shadow_lower_bound_validation_full_score_duration_ms = 5.25;
  stats.racing_line.shadow_lower_bound_evaluations = 51U;
  stats.racing_line.shadow_lower_bound_unavailable = 10U;
  stats.racing_line.shadow_lower_bound_prunable = 17U;
  stats.racing_line.shadow_lower_bound_false_prunes = 2U;
  stats.racing_line.shadow_lower_bound_winner_prunes = 1U;
  stats.racing_line.shadow_lower_bound_prunable_full_score_duration_ms = 3.5;
  stats.racing_line.shadow_lower_bound_max_overestimate_score = 0.25;
  stats.racing_line.shadow_lower_bound_max_underestimate_score = 12.5;
  stats.racing_line.shadow_lower_bound_max_false_prune_improvement_score = 1.75;
  stats.racing_line.shadow_local_speed_evaluations = 53U;
  stats.racing_line.shadow_local_speed_unavailable = 8U;
  stats.racing_line.shadow_local_speed_prunable = 21U;
  stats.racing_line.shadow_local_speed_false_prunes = 3U;
  stats.racing_line.shadow_local_speed_winner_mismatches = 2U;
  stats.racing_line.shadow_local_speed_abs_time_error_sum_s = 2.5;
  stats.racing_line.shadow_local_speed_abs_time_error_p95_s = 0.7;
  stats.racing_line.shadow_local_speed_max_time_overestimate_s = 0.6;
  stats.racing_line.shadow_local_speed_max_time_underestimate_s = 0.9;
  stats.racing_line.shadow_local_speed_abs_score_error_sum = 100.0;
  stats.racing_line.shadow_local_speed_abs_score_error_p95 = 28.0;
  stats.racing_line.shadow_local_speed_max_score_overestimate = 24.0;
  stats.racing_line.shadow_local_speed_max_score_underestimate = 36.0;
  stats.racing_line.shadow_local_speed_max_false_prune_improvement_score = 7.25;
  stats.racing_line.shadow_bounded_speed_evaluations = 54U;
  stats.racing_line.shadow_bounded_speed_unavailable = 7U;
  stats.racing_line.shadow_bounded_speed_prunable = 22U;
  stats.racing_line.shadow_bounded_speed_false_prunes = 2U;
  stats.racing_line.shadow_bounded_speed_winner_mismatches = 1U;
  stats.racing_line.shadow_bounded_speed_window_samples_total = 1430U;
  stats.racing_line.shadow_bounded_speed_window_samples_max = 55U;
  stats.racing_line.shadow_bounded_speed_duration_ms = 4.75;
  stats.racing_line.shadow_bounded_speed_abs_time_error_sum_s = 1.75;
  stats.racing_line.shadow_bounded_speed_abs_time_error_p95_s = 0.35;
  stats.racing_line.shadow_bounded_speed_max_time_overestimate_s = 0.25;
  stats.racing_line.shadow_bounded_speed_max_time_underestimate_s = 0.55;
  stats.racing_line.shadow_bounded_speed_abs_score_error_sum = 70.0;
  stats.racing_line.shadow_bounded_speed_abs_score_error_p95 = 14.0;
  stats.racing_line.shadow_bounded_speed_max_score_overestimate = 10.0;
  stats.racing_line.shadow_bounded_speed_max_score_underestimate = 22.0;
  stats.racing_line.shadow_bounded_speed_max_false_prune_improvement_score = 3.25;
  stats.racing_line.shadow_segment_score_evaluations = 52U;
  stats.racing_line.shadow_segment_score_unavailable = 9U;
  stats.racing_line.shadow_segment_score_prunable = 19U;
  stats.racing_line.shadow_segment_score_false_prunes = 1U;
  stats.racing_line.shadow_segment_score_winner_mismatches = 3U;
  stats.racing_line.shadow_segment_score_window_samples_total = 572U;
  stats.racing_line.shadow_segment_score_window_samples_max = 11U;
  stats.racing_line.shadow_segment_score_abs_error_sum = 0.35;
  stats.racing_line.shadow_segment_score_abs_error_p95 = 0.05;
  stats.racing_line.shadow_segment_score_max_overestimate = 0.2;
  stats.racing_line.shadow_segment_score_max_underestimate = 0.15;
  stats.racing_line.shadow_segment_score_max_false_prune_improvement_score = 0.75;
  stats.racing_line.window_count = 4U;
  stats.racing_line.active_window_count = 3U;
  stats.racing_line.active_window_samples = 18U;
  stats.racing_line.dp_states = 144U;
  stats.racing_line.dp_transitions = 512U;
  stats.racing_line.dp_segment_cache_hits = 10U;
  stats.racing_line.dp_segment_cache_misses = 502U;
  stats.racing_line.candidate_segment_cache_hits = 3U;
  stats.racing_line.candidate_segment_cache_misses = 244U;
  stats.racing_line.full_path_segment_cache_hits = 14U;
  stats.racing_line.full_path_segment_cache_misses = 88U;
  stats.racing_line.dp_coarse_states = 44U;
  stats.racing_line.dp_coarse_transitions = 112U;
  stats.racing_line.dp_fine_states = 100U;
  stats.racing_line.dp_fine_transitions = 400U;
  stats.racing_line.dp_coarse_to_fine_used = true;
  stats.racing_line.window_detection_duration_ms = 0.75;
  stats.racing_line.window_eval_duration_ms = 6.5;
  stats.racing_line.dp_duration_ms = 4.25;
  stats.racing_line.full_final_score_duration_ms = 2.75;
  stats.racing_line.async_refined = true;
  stats.racing_line.cost_length = 2.0;
  stats.racing_line.cost_time = 625.0;
  stats.racing_line.cost_curvature = 12.0;
  stats.racing_line.cost_curvature_change = 3.0;
  stats.racing_line.cost_heading_jump = 5.5;
  stats.racing_line.cost_offset_change = 1.0;
  stats.racing_line.cost_offset_second_change = 4.0;
  stats.racing_line.cost_offset_slope = 2.5;
  stats.racing_line.cost_collision = 0.0;
  stats.racing_line.cost_outside_grid = 0.0;
  stats.racing_line.cost_length_overrun = 0.0;
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
  stats.turn_smoothing.rejected_length = 0U;
  stats.turn_smoothing.rejected_not_improved = 0U;
  stats.turn_smoothing.rejected_curvature_regression = 2U;
  stats.turn_smoothing.rejected_radius_regression = 3U;
  stats.turn_smoothing.rejected_speed_regression = 4U;
  stats.turn_smoothing.rejected_time_regression = 5U;
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
  stats.isolated_curvature_spikes_smoothed_speed_profile = 1U;
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
  stats.racing_line_duration_ms = 99.9;
  stats.turn_smoothing_duration_ms = 8.75;
  stats.speed_profile_duration_ms = 1.5;
  return stats;
}

} // namespace

TEST(TrajectoryDiagnosticsIo, CsvHeaderAndRowContainProfiledTiming) {
  TrajectoryPointSample sample{};
  sample.s_m = 4.0;
  sample.point = Point2{1.0, 2.0};
  sample.tangent = Point2{1.0, 0.0};
  sample.curvature_1pm = 0.25;
  sample.left_bound_m = 3.0;
  sample.right_bound_m = 4.0;
  sample.racing_offset_m = -0.5;

  TrajectorySpeedSample speed_sample{};
  speed_sample.geometric_limit_mps = 8.0;
  speed_sample.profiled_limit_mps = 6.0;
  speed_sample.reason = SpeedConstraintType::kArc;
  speed_sample.constraint_s_m = 5.0;
  speed_sample.constraint_limit_mps = 4.0;

  const std::string header = finalTrajectorySamplesCsvHeader();
  const std::string row =
      finalTrajectorySamplesCsvRow(7U, sample, speed_sample, 1.25, 3.5);

  expectContainsAll(header, std::array{
                                "sample_index",
                                "s_m",
                                "x",
                                "y",
                                "curvature_1pm",
                                "speed_geometric_limit_mps",
                                "speed_profiled_limit_mps",
                                "speed_reason",
                                "speed_limit_source",
                                "constraint_s_m",
                                "constraint_limit_mps",
                                "profiled_time_from_start_s",
                                "profiled_time_to_finish_s",
                            });
  EXPECT_NE(row.find("arc"), std::string::npos);
  EXPECT_NE(row.find("1.25"), std::string::npos);
  EXPECT_NE(row.find("3.5"), std::string::npos);
  EXPECT_EQ(row.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, SummaryJsonContainsTraversalAndShapeMetrics) {
  TrajectoryShapeDiagnostics shape{};
  shape.segment_count = 9U;
  shape.max_curvature_jump_1pm = 0.2;
  shape.max_heading_delta_rad = 0.3;
  shape.max_offset_delta_m = 0.4;

  const std::string json =
      finalTrajectoryDiagnosticsSummaryJson(populatedStats(), shape);

  EXPECT_NE(json.find("\"racing_final_estimated_time_s\":12.5"), std::string::npos);
  EXPECT_NE(json.find("\"racing_final_min_speed_limit_mps\":1"), std::string::npos);
  EXPECT_NE(json.find("\"racing_final_max_speed_limit_mps\":10"), std::string::npos);
  EXPECT_NE(json.find("\"racing_centerline_estimated_time_s\":14"), std::string::npos);
  EXPECT_NE(json.find("\"racing_centerline_length_m\":100"), std::string::npos);
  EXPECT_NE(json.find("\"racing_final_length_ratio\":1.08"), std::string::npos);
  EXPECT_NE(json.find("\"racing_cost_time\":625"), std::string::npos);
  EXPECT_NE(json.find("\"racing_cost_heading_jump\":5.5"), std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_point_build_duration_ms\":1.25"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_sample_build_duration_ms\":2.5"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_cost_breakdown_duration_ms\":3.25"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_shape_diagnostics_duration_ms\":1.75"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_speed_profile_duration_ms\":4.75"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_speed_profile_calls\":8"), std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_speed_profile_samples_total\":400"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_speed_profile_samples_max\":55"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_regularization_duration_ms\":3.75"), std::string::npos);
  EXPECT_NE(json.find("\"racing_scratch_reused_candidates\":13"), std::string::npos);
  EXPECT_NE(json.find("\"racing_parallel_candidate_evaluation_used\":true"),
            std::string::npos);
  EXPECT_NE(json.find("\"corridor_parallel_workers_used\":4"), std::string::npos);
  EXPECT_NE(json.find("\"corridor_samples_reused\":true"), std::string::npos);
  EXPECT_NE(json.find("\"corridor_reused_samples\":42"), std::string::npos);
  EXPECT_NE(json.find("\"corridor_route_fingerprint\":4660"), std::string::npos);
  EXPECT_NE(json.find("\"corridor_config_fingerprint\":9029"), std::string::npos);
  EXPECT_NE(json.find("\"corridor_grid_cells_hash\":22136"), std::string::npos);
  EXPECT_NE(json.find("\"corridor_grid_inflated_hash\":39612"), std::string::npos);
  EXPECT_NE(json.find("\"corridor_sample_build_duration_ms\":6.25"), std::string::npos);
  EXPECT_NE(json.find("\"clearance_field_reused_by_corridor\":true"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_parallel_workers_used\":2"), std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_chunks\":31"), std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_parallel_batches\":29"), std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_threads_launched\":58"), std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_batch_wall_duration_ms\":12.25"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_batch_wait_duration_ms\":10.5"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_worker_buffer_prepare_duration_ms\":1.5"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_thread_launch_duration_ms\":2.75"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_thread_join_wait_duration_ms\":8"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_worker_scratch_reuses\":62"), std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_snapshot_allocations_avoided\":60"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_offset_changed_samples_total\":180"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_offset_changed_samples_max\":7"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_offset_changed_span_samples_total\":220"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_offset_changed_span_samples_max\":9"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_local_speed_window_samples_total\":930"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_local_speed_window_samples_max\":35"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_local_candidate_evaluations\":61"), std::string::npos);
  EXPECT_NE(json.find("\"racing_local_candidate_full_score_fallbacks\":55"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_local_candidate_full_score_required\":10"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_local_candidate_full_score_required_invalid_input\":1"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_local_candidate_full_score_required_boundary\":2"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_local_candidate_full_score_required_unsafe_base\":3"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"racing_local_candidate_full_score_required_window_invalid\":4"),
      std::string::npos);
  EXPECT_NE(json.find("\"racing_local_candidate_acceptance_full_scores\":7"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_local_candidate_point_build_duration_ms\":1.1"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_local_candidate_path_evaluation_duration_ms\":2.2"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_local_candidate_traversal_estimate_duration_ms\":3.3"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_lower_bound_evaluations\":51"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_lower_bound_validation_full_scores\":41"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"racing_shadow_lower_bound_validation_full_score_duration_ms\":5.25"),
      std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_lower_bound_prunable\":17"), std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_lower_bound_false_prunes\":2"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_lower_bound_winner_prunes\":1"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"racing_shadow_lower_bound_prunable_full_score_duration_ms\":3.5"),
      std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_evaluations\":53"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_unavailable\":8"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_prunable\":21"), std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_false_prunes\":3"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_winner_mismatches\":2"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_abs_time_error_sum_s\":2.5"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_abs_time_error_p95_s\":0.7"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_max_time_overestimate_s\":0.6"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_max_time_underestimate_s\":0.9"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_abs_score_error_sum\":100"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_abs_score_error_p95\":28"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_max_score_overestimate\":24"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_local_speed_max_score_underestimate\":36"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"racing_shadow_local_speed_max_false_prune_improvement_score\":7.25"),
      std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_evaluations\":54"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_unavailable\":7"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_prunable\":22"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_false_prunes\":2"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_winner_mismatches\":1"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_window_samples_total\":1430"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_window_samples_max\":55"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_duration_ms\":4.75"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_abs_time_error_sum_s\":1.75"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_abs_time_error_p95_s\":0.35"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_max_time_overestimate_s\":0.25"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_max_time_underestimate_s\":0.55"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_abs_score_error_sum\":70"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_abs_score_error_p95\":14"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_max_score_overestimate\":10"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_bounded_speed_max_score_underestimate\":22"),
            std::string::npos);
  EXPECT_NE(
      json.find(
          "\"racing_shadow_bounded_speed_max_false_prune_improvement_score\":3.25"),
      std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_segment_score_evaluations\":52"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_segment_score_unavailable\":9"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_segment_score_prunable\":19"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_segment_score_false_prunes\":1"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_segment_score_winner_mismatches\":3"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_segment_score_window_samples_total\":572"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_segment_score_window_samples_max\":11"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_segment_score_abs_error_sum\":0.35"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_segment_score_abs_error_p95\":0.05"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_segment_score_max_overestimate\":0.2"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_shadow_segment_score_max_underestimate\":0.15"),
            std::string::npos);
  EXPECT_NE(
      json.find(
          "\"racing_shadow_segment_score_max_false_prune_improvement_score\":0.75"),
      std::string::npos);
  EXPECT_NE(json.find("\"racing_line_dp_coarse_to_fine_used\":true"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_line_window_count\":4"), std::string::npos);
  EXPECT_NE(json.find("\"racing_line_active_window_count\":3"), std::string::npos);
  EXPECT_NE(json.find("\"racing_line_dp_states\":144"), std::string::npos);
  EXPECT_NE(json.find("\"racing_line_dp_coarse_states\":44"), std::string::npos);
  EXPECT_NE(json.find("\"racing_line_dp_fine_transitions\":400"), std::string::npos);
  EXPECT_NE(json.find("\"racing_line_full_path_segment_cache_hits\":14"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_line_full_path_segment_cache_misses\":88"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_quality\":\"refined\""), std::string::npos);
  EXPECT_NE(json.find("\"racing_line_async_refined\":true"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_total_duration_ms\":123.4"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_racing_line_duration_ms\":99.9"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_best_candidate_estimated_time_s\":12.25"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_regularization_applied\":true"), std::string::npos);
  EXPECT_NE(json.find("\"turn_smoothing_smoothed_corners\":1"), std::string::npos);
  EXPECT_NE(json.find("\"turn_smoothing_heading_delta_after_rad\":0.4"),
            std::string::npos);
  EXPECT_NE(json.find("\"turn_smoothing_bezier_cache_hits\":21"), std::string::npos);
  EXPECT_NE(json.find("\"turn_smoothing_collision_check_duration_ms\":1.3"),
            std::string::npos);
  EXPECT_NE(json.find("\"turn_smoothing_corner_diagnostics\""), std::string::npos);
  EXPECT_NE(json.find("\"corner_s_m\":42"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_shape_segment_count\":9"), std::string::npos);
  EXPECT_EQ(json.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, RacingLineJsonFragmentContainsBlackboxRequiredKeys) {
  const std::string fragment = racingLineDiagnosticsJsonFields(populatedStats());

  expectContainsAll(
      fragment, std::array{
                    "\"racing_final_estimated_time_s\"",
                    "\"racing_final_min_speed_limit_mps\"",
                    "\"racing_final_max_speed_limit_mps\"",
                    "\"racing_final_curvature_limited_samples\"",
                    "\"racing_centerline_length_m\"",
                    "\"racing_final_length_m\"",
                    "\"racing_final_length_ratio\"",
                    "\"racing_max_abs_offset_m\"",
                    "\"racing_min_edge_margin_m\"",
                    "\"racing_mean_edge_margin_m\"",
                    "\"racing_cost_length\"",
                    "\"racing_cost_time\"",
                    "\"racing_cost_curvature\"",
                    "\"racing_cost_curvature_change\"",
                    "\"racing_cost_heading_jump\"",
                    "\"racing_cost_offset_change\"",
                    "\"racing_cost_offset_second_change\"",
                    "\"racing_cost_offset_slope\"",
                    "\"racing_cost_collision\"",
                    "\"racing_cost_outside_grid\"",
                    "\"racing_cost_length_overrun\"",
                    "\"racing_centerline_estimated_time_s\"",
                    "\"racing_centerline_min_speed_limit_mps\"",
                    "\"racing_centerline_max_speed_limit_mps\"",
                    "\"racing_centerline_curvature_limited_samples\"",
                    "\"racing_best_candidate_estimated_time_s\"",
                    "\"racing_best_candidate_score\"",
                    "\"racing_best_candidate_min_speed_limit_mps\"",
                    "\"racing_best_candidate_max_speed_limit_mps\"",
                    "\"racing_best_candidate_curvature_limited_samples\"",
                    "\"racing_time_gain_s\"",
                    "\"racing_regularization_time_delta_s\"",
                    "\"racing_regularization_iterations\"",
                    "\"racing_regularization_applied\"",
                    "\"racing_pre_regularization_max_curvature_jump_1pm\"",
                    "\"racing_post_regularization_max_curvature_jump_1pm\"",
                    "\"racing_candidate_path_evaluation_duration_ms\"",
                    "\"racing_candidate_score_duration_ms\"",
                    "\"racing_candidate_point_build_duration_ms\"",
                    "\"racing_candidate_sample_build_duration_ms\"",
                    "\"racing_candidate_cost_breakdown_duration_ms\"",
                    "\"racing_candidate_shape_diagnostics_duration_ms\"",
                    "\"racing_candidate_speed_profile_duration_ms\"",
                    "\"racing_candidate_speed_profile_calls\"",
                    "\"racing_candidate_speed_profile_samples_total\"",
                    "\"racing_candidate_speed_profile_samples_max\"",
                    "\"racing_regularization_duration_ms\"",
                    "\"racing_scratch_reused_candidates\"",
                    "\"racing_parallel_candidate_evaluation_used\"",
                    "\"racing_parallel_workers_used\"",
                    "\"racing_candidate_chunks\"",
                    "\"racing_candidate_parallel_batches\"",
                    "\"racing_candidate_threads_launched\"",
                    "\"racing_candidate_batch_wall_duration_ms\"",
                    "\"racing_candidate_batch_wait_duration_ms\"",
                    "\"racing_candidate_worker_buffer_prepare_duration_ms\"",
                    "\"racing_candidate_thread_launch_duration_ms\"",
                    "\"racing_candidate_thread_join_wait_duration_ms\"",
                    "\"racing_worker_scratch_reuses\"",
                    "\"racing_candidate_snapshot_allocations_avoided\"",
                    "\"racing_candidate_offset_changed_samples_total\"",
                    "\"racing_candidate_offset_changed_samples_max\"",
                    "\"racing_candidate_offset_changed_span_samples_total\"",
                    "\"racing_candidate_offset_changed_span_samples_max\"",
                    "\"racing_candidate_local_speed_window_samples_total\"",
                    "\"racing_candidate_local_speed_window_samples_max\"",
                    "\"racing_local_candidate_evaluations\"",
                    "\"racing_local_candidate_full_score_fallbacks\"",
                    "\"racing_local_candidate_full_score_required\"",
                    "\"racing_local_candidate_full_score_required_invalid_input\"",
                    "\"racing_local_candidate_full_score_required_boundary\"",
                    "\"racing_local_candidate_full_score_required_unsafe_base\"",
                    "\"racing_local_candidate_full_score_required_window_invalid\"",
                    "\"racing_local_candidate_acceptance_full_scores\"",
                    "\"racing_local_score_false_positives\"",
                    "\"racing_local_candidate_point_build_duration_ms\"",
                    "\"racing_local_candidate_path_evaluation_duration_ms\"",
                    "\"racing_local_candidate_score_duration_ms\"",
                    "\"racing_local_candidate_traversal_estimate_duration_ms\"",
                    "\"racing_full_candidate_score_duration_ms\"",
                    "\"racing_shadow_lower_bound_validation_full_scores\"",
                    "\"racing_shadow_lower_bound_validation_full_score_duration_ms\"",
                    "\"racing_shadow_lower_bound_evaluations\"",
                    "\"racing_shadow_lower_bound_unavailable\"",
                    "\"racing_shadow_lower_bound_prunable\"",
                    "\"racing_shadow_lower_bound_false_prunes\"",
                    "\"racing_shadow_lower_bound_winner_prunes\"",
                    "\"racing_shadow_lower_bound_prunable_full_score_duration_ms\"",
                    "\"racing_shadow_lower_bound_max_overestimate_score\"",
                    "\"racing_shadow_lower_bound_max_underestimate_score\"",
                    "\"racing_shadow_lower_bound_max_false_prune_improvement_score\"",
                    "\"racing_shadow_local_speed_evaluations\"",
                    "\"racing_shadow_local_speed_unavailable\"",
                    "\"racing_shadow_local_speed_prunable\"",
                    "\"racing_shadow_local_speed_false_prunes\"",
                    "\"racing_shadow_local_speed_winner_mismatches\"",
                    "\"racing_shadow_local_speed_abs_time_error_sum_s\"",
                    "\"racing_shadow_local_speed_abs_time_error_p95_s\"",
                    "\"racing_shadow_local_speed_max_time_overestimate_s\"",
                    "\"racing_shadow_local_speed_max_time_underestimate_s\"",
                    "\"racing_shadow_local_speed_abs_score_error_sum\"",
                    "\"racing_shadow_local_speed_abs_score_error_p95\"",
                    "\"racing_shadow_local_speed_max_score_overestimate\"",
                    "\"racing_shadow_local_speed_max_score_underestimate\"",
                    "\"racing_shadow_local_speed_max_false_prune_improvement_score\"",
                    "\"racing_shadow_bounded_speed_evaluations\"",
                    "\"racing_shadow_bounded_speed_unavailable\"",
                    "\"racing_shadow_bounded_speed_prunable\"",
                    "\"racing_shadow_bounded_speed_false_prunes\"",
                    "\"racing_shadow_bounded_speed_winner_mismatches\"",
                    "\"racing_shadow_bounded_speed_window_samples_total\"",
                    "\"racing_shadow_bounded_speed_window_samples_max\"",
                    "\"racing_shadow_bounded_speed_duration_ms\"",
                    "\"racing_shadow_bounded_speed_abs_time_error_sum_s\"",
                    "\"racing_shadow_bounded_speed_abs_time_error_p95_s\"",
                    "\"racing_shadow_bounded_speed_max_time_overestimate_s\"",
                    "\"racing_shadow_bounded_speed_max_time_underestimate_s\"",
                    "\"racing_shadow_bounded_speed_abs_score_error_sum\"",
                    "\"racing_shadow_bounded_speed_abs_score_error_p95\"",
                    "\"racing_shadow_bounded_speed_max_score_overestimate\"",
                    "\"racing_shadow_bounded_speed_max_score_underestimate\"",
                    "\"racing_shadow_bounded_speed_max_false_prune_improvement_score\"",
                    "\"racing_shadow_segment_score_evaluations\"",
                    "\"racing_shadow_segment_score_unavailable\"",
                    "\"racing_shadow_segment_score_prunable\"",
                    "\"racing_shadow_segment_score_false_prunes\"",
                    "\"racing_shadow_segment_score_winner_mismatches\"",
                    "\"racing_shadow_segment_score_window_samples_total\"",
                    "\"racing_shadow_segment_score_window_samples_max\"",
                    "\"racing_shadow_segment_score_abs_error_sum\"",
                    "\"racing_shadow_segment_score_abs_error_p95\"",
                    "\"racing_shadow_segment_score_max_overestimate\"",
                    "\"racing_shadow_segment_score_max_underestimate\"",
                    "\"racing_shadow_segment_score_max_false_prune_improvement_score\"",
                    "\"racing_line_window_count\"",
                    "\"racing_line_active_window_count\"",
                    "\"racing_line_active_window_samples\"",
                    "\"racing_line_dp_states\"",
                    "\"racing_line_dp_transitions\"",
                    "\"racing_line_dp_segment_cache_hits\"",
                    "\"racing_line_dp_segment_cache_misses\"",
                    "\"racing_line_candidate_segment_cache_hits\"",
                    "\"racing_line_candidate_segment_cache_misses\"",
                    "\"racing_line_full_path_segment_cache_hits\"",
                    "\"racing_line_full_path_segment_cache_misses\"",
                    "\"racing_line_dp_coarse_states\"",
                    "\"racing_line_dp_coarse_transitions\"",
                    "\"racing_line_dp_fine_states\"",
                    "\"racing_line_dp_fine_transitions\"",
                    "\"racing_line_dp_coarse_to_fine_used\"",
                    "\"racing_line_window_detection_duration_ms\"",
                    "\"racing_line_window_eval_duration_ms\"",
                    "\"racing_line_dp_duration_ms\"",
                    "\"racing_line_full_final_score_duration_ms\"",
                    "\"racing_line_async_refined\"",
                });
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, TurnSmoothingJsonFragmentContainsBlackboxRequiredKeys) {
  const std::string fragment = turnSmoothingDiagnosticsJsonFields(populatedStats());

  expectContainsAll(fragment, std::array{
                                  "\"turn_smoothing_input_samples\"",
                                  "\"turn_smoothing_output_samples\"",
                                  "\"turn_smoothing_detected_corners\"",
                                  "\"turn_smoothing_attempted_corners\"",
                                  "\"turn_smoothing_candidate_attempts\"",
                                  "\"turn_smoothing_relaxed_candidate_attempts\"",
                                  "\"turn_smoothing_bezier_cache_hits\"",
                                  "\"turn_smoothing_bezier_cache_misses\"",
                                  "\"turn_smoothing_before_metrics_cache_hits\"",
                                  "\"turn_smoothing_before_metrics_cache_misses\"",
                                  "\"turn_smoothing_traversability_cache_hits\"",
                                  "\"turn_smoothing_traversability_cache_misses\"",
                                  "\"turn_smoothing_smoothed_corners\"",
                                  "\"turn_smoothing_rejected_prohibited\"",
                                  "\"turn_smoothing_rejected_corridor\"",
                                  "\"turn_smoothing_rejected_length\"",
                                  "\"turn_smoothing_rejected_not_improved\"",
                                  "\"turn_smoothing_rejected_curvature_regression\"",
                                  "\"turn_smoothing_rejected_radius_regression\"",
                                  "\"turn_smoothing_rejected_speed_regression\"",
                                  "\"turn_smoothing_rejected_time_regression\"",
                                  "\"turn_smoothing_heading_delta_before_rad\"",
                                  "\"turn_smoothing_heading_delta_after_rad\"",
                                  "\"turn_smoothing_curvature_jump_before_1pm\"",
                                  "\"turn_smoothing_curvature_jump_after_1pm\"",
                                  "\"turn_smoothing_min_inner_margin_m\"",
                                  "\"turn_smoothing_max_outer_shift_m\"",
                                  "\"turn_smoothing_accepted_entry_distance_m\"",
                                  "\"turn_smoothing_accepted_exit_distance_m\"",
                                  "\"turn_smoothing_accepted_shift_scale\"",
                                  "\"turn_smoothing_accepted_relaxed_angle_deg\"",
                                  "\"turn_smoothing_accepted_score\"",
                                  "\"turn_smoothing_accepted_min_radius_before_m\"",
                                  "\"turn_smoothing_accepted_min_radius_after_m\"",
                                  "\"turn_smoothing_accepted_min_speed_before_mps\"",
                                  "\"turn_smoothing_accepted_min_speed_after_mps\"",
                                  "\"turn_smoothing_accepted_local_time_before_s\"",
                                  "\"turn_smoothing_accepted_local_time_after_s\"",
                                  "\"turn_smoothing_candidate_build_duration_ms\"",
                                  "\"turn_smoothing_candidate_replace_duration_ms\"",
                                  "\"turn_smoothing_collision_check_duration_ms\"",
                                  "\"turn_smoothing_metrics_duration_ms\"",
                                  "\"turn_smoothing_shape_diagnostics_duration_ms\"",
                                  "\"turn_smoothing_speed_profile_duration_ms\"",
                                  "\"turn_smoothing_corner_diagnostics\"",
                              });
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo,
     SpeedProfileConstraintJsonFragmentContainsBlackboxRequiredKeys) {
  const std::string fragment =
      speedProfileConstraintDiagnosticsJsonFields(populatedStats());

  expectContainsAll(fragment,
                    std::array{
                        "\"speed_profile_top_constraint_count\"",
                        "\"speed_profile_top1_sample_index\"",
                        "\"speed_profile_top1_s_m\"",
                        "\"speed_profile_top1_radius_m\"",
                        "\"speed_profile_top1_curvature_1pm\"",
                        "\"speed_profile_top1_speed_limit_mps\"",
                        "\"speed_profile_top1_profiled_limit_mps\"",
                        "\"speed_profile_top1_source\"",
                        "\"speed_profile_top1_isolated_curvature_spike\"",
                        "\"isolated_curvature_spike_candidates\"",
                        "\"isolated_curvature_spikes_smoothed_geometry\"",
                        "\"isolated_curvature_spikes_smoothed_speed_profile\"",
                        "\"isolated_curvature_spike_max_before_1pm\"",
                        "\"isolated_curvature_spike_max_after_1pm\"",
                    });
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, TimingJsonFragmentContainsBlackboxRequiredKeys) {
  const std::string fragment = trajectoryTimingDiagnosticsJsonFields(populatedStats());

  expectContainsAll(fragment, std::array{
                                  "\"trajectory_total_duration_ms\"",
                                  "\"trajectory_corridor_duration_ms\"",
                                  "\"trajectory_racing_line_duration_ms\"",
                                  "\"trajectory_turn_smoothing_duration_ms\"",
                                  "\"trajectory_speed_profile_duration_ms\"",
                              });
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, RacingLineJsonFragmentWritesNullForNonFiniteMetrics) {
  const std::string fragment =
      racingLineDiagnosticsJsonFields(TrajectoryPlannerStats{});

  EXPECT_NE(fragment.find("\"racing_final_estimated_time_s\":null"), std::string::npos);
  EXPECT_NE(fragment.find("\"racing_centerline_estimated_time_s\":null"),
            std::string::npos);
  EXPECT_NE(fragment.find("\"racing_best_candidate_score\":null"), std::string::npos);
  EXPECT_NE(fragment.find("\"racing_final_length_ratio\":null"), std::string::npos);
  EXPECT_NE(fragment.find("\"racing_cost_time\":null"), std::string::npos);
  EXPECT_NE(fragment.find("\"racing_cost_heading_jump\":null"), std::string::npos);
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, PlannerDiagnosticsJsonRoundTripsRuntimeStats) {
  const std::uint64_t planner_path_id = 42U;
  const std::uint64_t path_stamp_ns = 1'782'477'871'305'471'587ULL;
  const std::string json = trajectoryPlannerDiagnosticsJson(
      planner_path_id, path_stamp_ns, populatedStats());
  EXPECT_NE(json.find("\"trajectory_quality\":\"refined\""), std::string::npos);

  const std::optional<TrajectoryPlannerDiagnosticsEnvelope> parsed =
      parseTrajectoryPlannerDiagnosticsJson(json);

  ASSERT_TRUE(parsed.has_value());
  const TrajectoryPlannerDiagnosticsEnvelope parsed_value =
      parsed.value_or(TrajectoryPlannerDiagnosticsEnvelope{});
  EXPECT_EQ(parsed_value.planner_path_id, planner_path_id);
  EXPECT_EQ(parsed_value.path_stamp_ns, path_stamp_ns);
  EXPECT_EQ(parsed_value.stats.quality, TrajectoryQuality::kRefined);
  EXPECT_EQ(parsed_value.stats.samples, 78U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.length_m, 412.25);
  EXPECT_EQ(parsed_value.stats.corridor.samples, 42U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor.min_width_m, 17.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor.mean_width_m, 24.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor.max_width_m, 58.75);
  EXPECT_EQ(parsed_value.stats.corridor.lateral_limited_samples, 9U);
  EXPECT_EQ(parsed_value.stats.corridor.parallel_workers_used, 4U);
  EXPECT_TRUE(parsed_value.stats.corridor.samples_reused);
  EXPECT_EQ(parsed_value.stats.corridor.reused_samples, 42U);
  EXPECT_EQ(parsed_value.stats.corridor.route_fingerprint, 0x1234U);
  EXPECT_EQ(parsed_value.stats.corridor.config_fingerprint, 0x2345U);
  EXPECT_EQ(parsed_value.stats.corridor.prohibited_grid_fingerprint.cells_hash,
            0x5678U);
  EXPECT_EQ(parsed_value.stats.corridor.prohibited_grid_fingerprint.inflated_hash,
            0x9abcU);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor.sample_build_duration_ms, 6.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor.raycast_duration_ms, 5.75);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor.lateral_limit_duration_ms, 1.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor.clearance_field_build_duration_ms, 0.0);
  EXPECT_TRUE(parsed_value.stats.corridor.clearance_field_reused);
  EXPECT_FALSE(parsed_value.stats.corridor.clearance_field_cache_hit);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.final_length_m, 108.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.final_length_ratio, 1.08);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.time_gain_s, 1.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.min_edge_margin_m, 2.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.cost_offset_slope, 2.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_path_evaluation_duration_ms,
                   7.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_score_duration_ms, 8.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_point_build_duration_ms,
                   1.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_sample_build_duration_ms,
                   2.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_cost_breakdown_duration_ms,
                   3.25);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.candidate_shape_diagnostics_duration_ms, 1.75);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_speed_profile_duration_ms,
                   4.75);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_speed_profile_calls, 8U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_speed_profile_samples_total, 400U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_speed_profile_samples_max, 55U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.regularization_duration_ms, 3.75);
  EXPECT_EQ(parsed_value.stats.racing_line.scratch_reused_candidates, 13U);
  EXPECT_TRUE(parsed_value.stats.racing_line.parallel_candidate_evaluation_used);
  EXPECT_EQ(parsed_value.stats.racing_line.parallel_workers_used, 2U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_chunks, 31U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_parallel_batches, 29U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_threads_launched, 58U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_batch_wall_duration_ms,
                   12.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_batch_wait_duration_ms,
                   10.5);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.candidate_worker_buffer_prepare_duration_ms, 1.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_thread_launch_duration_ms,
                   2.75);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.candidate_thread_join_wait_duration_ms, 8.0);
  EXPECT_EQ(parsed_value.stats.racing_line.worker_scratch_reuses, 62U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_snapshot_allocations_avoided, 60U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_offset_changed_samples_total,
            180U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_offset_changed_samples_max, 7U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_offset_changed_span_samples_total,
            220U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_offset_changed_span_samples_max,
            9U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_local_speed_window_samples_total,
            930U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_local_speed_window_samples_max,
            35U);
  EXPECT_EQ(parsed_value.stats.racing_line.local_candidate_evaluations, 61U);
  EXPECT_EQ(parsed_value.stats.racing_line.local_candidate_full_score_fallbacks, 55U);
  EXPECT_EQ(parsed_value.stats.racing_line.local_candidate_full_score_required, 10U);
  EXPECT_EQ(
      parsed_value.stats.racing_line.local_candidate_full_score_required_invalid_input,
      1U);
  EXPECT_EQ(parsed_value.stats.racing_line.local_candidate_full_score_required_boundary,
            2U);
  EXPECT_EQ(
      parsed_value.stats.racing_line.local_candidate_full_score_required_unsafe_base,
      3U);
  EXPECT_EQ(
      parsed_value.stats.racing_line.local_candidate_full_score_required_window_invalid,
      4U);
  EXPECT_EQ(parsed_value.stats.racing_line.local_candidate_acceptance_full_scores, 7U);
  EXPECT_EQ(parsed_value.stats.racing_line.local_score_false_positives, 1U);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.local_candidate_point_build_duration_ms, 1.1);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.local_candidate_path_evaluation_duration_ms, 2.2);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.local_candidate_score_duration_ms,
                   4.5);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.local_candidate_traversal_estimate_duration_ms,
      3.3);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.full_candidate_score_duration_ms,
                   6.75);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_lower_bound_validation_full_scores,
            41U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line
                       .shadow_lower_bound_validation_full_score_duration_ms,
                   5.25);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_lower_bound_evaluations, 51U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_lower_bound_unavailable, 10U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_lower_bound_prunable, 17U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_lower_bound_false_prunes, 2U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_lower_bound_winner_prunes, 1U);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_lower_bound_prunable_full_score_duration_ms,
      3.5);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_lower_bound_max_overestimate_score, 0.25);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_lower_bound_max_underestimate_score, 12.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line
                       .shadow_lower_bound_max_false_prune_improvement_score,
                   1.75);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_local_speed_evaluations, 53U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_local_speed_unavailable, 8U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_local_speed_prunable, 21U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_local_speed_false_prunes, 3U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_local_speed_winner_mismatches, 2U);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_local_speed_abs_time_error_sum_s, 2.5);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_local_speed_abs_time_error_p95_s, 0.7);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_local_speed_max_time_overestimate_s, 0.6);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_local_speed_max_time_underestimate_s, 0.9);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_local_speed_abs_score_error_sum, 100.0);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_local_speed_abs_score_error_p95, 28.0);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_local_speed_max_score_overestimate, 24.0);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_local_speed_max_score_underestimate, 36.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line
                       .shadow_local_speed_max_false_prune_improvement_score,
                   7.25);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_bounded_speed_evaluations, 54U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_bounded_speed_unavailable, 7U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_bounded_speed_prunable, 22U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_bounded_speed_false_prunes, 2U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_bounded_speed_winner_mismatches, 1U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_bounded_speed_window_samples_total,
            1430U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_bounded_speed_window_samples_max,
            55U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.shadow_bounded_speed_duration_ms,
                   4.75);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_bounded_speed_abs_time_error_sum_s, 1.75);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_bounded_speed_abs_time_error_p95_s, 0.35);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_bounded_speed_max_time_overestimate_s,
      0.25);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_bounded_speed_max_time_underestimate_s,
      0.55);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_bounded_speed_abs_score_error_sum, 70.0);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_bounded_speed_abs_score_error_p95, 14.0);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_bounded_speed_max_score_overestimate, 10.0);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_bounded_speed_max_score_underestimate,
      22.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line
                       .shadow_bounded_speed_max_false_prune_improvement_score,
                   3.25);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_segment_score_evaluations, 52U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_segment_score_unavailable, 9U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_segment_score_prunable, 19U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_segment_score_false_prunes, 1U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_segment_score_winner_mismatches, 3U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_segment_score_window_samples_total,
            572U);
  EXPECT_EQ(parsed_value.stats.racing_line.shadow_segment_score_window_samples_max,
            11U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.shadow_segment_score_abs_error_sum,
                   0.35);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.shadow_segment_score_abs_error_p95,
                   0.05);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.shadow_segment_score_max_overestimate,
                   0.2);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.racing_line.shadow_segment_score_max_underestimate, 0.15);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line
                       .shadow_segment_score_max_false_prune_improvement_score,
                   0.75);
  EXPECT_EQ(parsed_value.stats.racing_line.full_path_segment_cache_hits, 14U);
  EXPECT_EQ(parsed_value.stats.racing_line.full_path_segment_cache_misses, 88U);
  EXPECT_EQ(parsed_value.stats.racing_line.window_count, 4U);
  EXPECT_EQ(parsed_value.stats.racing_line.active_window_count, 3U);
  EXPECT_EQ(parsed_value.stats.racing_line.active_window_samples, 18U);
  EXPECT_EQ(parsed_value.stats.racing_line.dp_states, 144U);
  EXPECT_EQ(parsed_value.stats.racing_line.dp_transitions, 512U);
  EXPECT_EQ(parsed_value.stats.racing_line.dp_segment_cache_hits, 10U);
  EXPECT_EQ(parsed_value.stats.racing_line.dp_segment_cache_misses, 502U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_segment_cache_hits, 3U);
  EXPECT_EQ(parsed_value.stats.racing_line.candidate_segment_cache_misses, 244U);
  EXPECT_EQ(parsed_value.stats.racing_line.dp_coarse_states, 44U);
  EXPECT_EQ(parsed_value.stats.racing_line.dp_coarse_transitions, 112U);
  EXPECT_EQ(parsed_value.stats.racing_line.dp_fine_states, 100U);
  EXPECT_EQ(parsed_value.stats.racing_line.dp_fine_transitions, 400U);
  EXPECT_TRUE(parsed_value.stats.racing_line.dp_coarse_to_fine_used);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.window_detection_duration_ms, 0.75);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.window_eval_duration_ms, 6.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.dp_duration_ms, 4.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.full_final_score_duration_ms, 2.75);
  EXPECT_TRUE(parsed_value.stats.racing_line.async_refined);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.input_samples, 48U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.output_samples, 72U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.candidate_attempts, 11U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.relaxed_candidate_attempts, 6U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.bezier_cache_hits, 21U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.bezier_cache_misses, 22U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.before_metrics_cache_hits, 23U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.before_metrics_cache_misses, 24U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.traversability_cache_hits, 25U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.traversability_cache_misses, 26U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.smoothed_corners, 1U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.rejected_curvature_regression, 2U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.rejected_radius_regression, 3U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.rejected_speed_regression, 4U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.rejected_time_regression, 5U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.max_heading_delta_before_rad, 1.2);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.max_heading_delta_after_rad, 0.4);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.min_inner_margin_m, 2.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.max_applied_outer_shift_m, 6.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_entry_distance_m, 30.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_exit_distance_m, 30.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_shift_scale, 0.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_relaxed_angle_deg, 15.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_score, 12.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_min_radius_before_m, 6.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_min_radius_after_m, 9.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_min_speed_before_mps,
                   5.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_min_speed_after_mps, 7.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_local_time_before_s, 4.2);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_local_time_after_s, 3.7);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.candidate_build_duration_ms, 1.1);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.candidate_replace_duration_ms,
                   1.2);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.collision_check_duration_ms, 1.3);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.metrics_duration_ms, 1.4);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.shape_diagnostics_duration_ms,
                   1.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.speed_profile_duration_ms, 1.6);
  EXPECT_DOUBLE_EQ(parsed_value.stats.speed_profile_mean_mps, 13.4);
  EXPECT_EQ(parsed_value.stats.speed_profile_curvature_limited_samples, 69U);
  EXPECT_EQ(parsed_value.stats.isolated_curvature_spike_candidates, 2U);
  EXPECT_EQ(parsed_value.stats.isolated_curvature_spikes_smoothed_geometry, 1U);
  EXPECT_EQ(parsed_value.stats.isolated_curvature_spikes_smoothed_speed_profile, 1U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.isolated_curvature_spike_max_before_1pm, 0.12);
  EXPECT_DOUBLE_EQ(parsed_value.stats.isolated_curvature_spike_max_after_1pm, 0.04);
  ASSERT_EQ(parsed_value.stats.top_speed_constraints.size(), 1U);
  EXPECT_EQ(parsed_value.stats.top_speed_constraints.front().sample_index, 17U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.top_speed_constraints.front().s_m, 42.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.top_speed_constraints.front().radius_m, 9.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.top_speed_constraints.front().curvature_1pm,
                   0.108);
  EXPECT_DOUBLE_EQ(parsed_value.stats.top_speed_constraints.front().speed_limit_mps,
                   6.8);
  EXPECT_EQ(parsed_value.stats.top_speed_constraints.front().source,
            SpeedConstraintType::kArc);
  EXPECT_TRUE(
      parsed_value.stats.top_speed_constraints.front().isolated_curvature_spike);
  EXPECT_DOUBLE_EQ(parsed_value.stats.total_duration_ms, 123.4);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor_duration_ms, 5.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line_duration_ms, 99.9);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing_duration_ms, 8.75);
  EXPECT_DOUBLE_EQ(parsed_value.stats.speed_profile_duration_ms, 1.5);
}

TEST(TrajectoryDiagnosticsIo, PlannerDiagnosticsJsonExposesBaselineQuality) {
  TrajectoryPlannerStats stats{};
  stats.quality = TrajectoryQuality::kBaseline;

  const std::string json = trajectoryPlannerDiagnosticsJson(1U, 2U, stats);
  const std::optional<TrajectoryPlannerDiagnosticsEnvelope> parsed =
      parseTrajectoryPlannerDiagnosticsJson(json);

  EXPECT_NE(json.find("\"trajectory_quality\":\"baseline\""), std::string::npos);
  ASSERT_TRUE(parsed.has_value());
  const TrajectoryPlannerDiagnosticsEnvelope parsed_value =
      parsed.value_or(TrajectoryPlannerDiagnosticsEnvelope{});
  EXPECT_EQ(parsed_value.stats.quality, TrajectoryQuality::kBaseline);
}

} // namespace drone_city_nav
