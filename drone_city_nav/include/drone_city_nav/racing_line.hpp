#pragma once

#include "drone_city_nav/corridor.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_speed_planner.hpp"

#include <cstddef>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

namespace drone_city_nav {

struct RacingLineConfig {
  double optimizer_sample_step_m{0.0};
  std::size_t max_iterations{80U};
  double initial_offset_step_m{2.0};
  double min_offset_step_m{0.1};
  double cooling_ratio{0.5};
  double weight_length{0.02};
  double weight_curvature{300.0};
  double weight_curvature_change{130.0};
  double weight_offset_change{0.5};
  double weight_offset_second_change{6.5};
  double weight_offset_slope{100.0};
  double max_offset_slope_per_m{0.32};
  double weight_time{40.0};
  double max_length_ratio{1.6};
  std::size_t regularization_iterations{2U};
  double regularization_max_time_regression_s{0.5};
  std::size_t parallel_workers{0U};
  double window_pre_margin_m{25.0};
  double window_post_margin_m{25.0};
  double window_heading_threshold_rad{10.0 * std::numbers::pi / 180.0};
  double window_width_change_threshold_m{2.0};
  double window_min_heading_span_rad{10.0 * std::numbers::pi / 180.0};
  double window_min_curvature_1pm{0.01};
  double window_min_width_asymmetry_m{1.0};
  double dp_offset_step_m{1.5};
  double dp_coarse_offset_step_m{2.0};
  double dp_fine_offset_step_m{0.75};
  double dp_fine_radius_m{1.5};
  std::size_t async_refinement_workers{1U};
};

struct RacingLineStats {
  std::size_t input_samples{0U};
  std::size_t optimizer_samples{0U};
  std::size_t output_samples{0U};
  std::size_t iterations{0U};
  std::size_t candidate_evaluations{0U};
  std::size_t skipped_noop_candidates{0U};
  std::size_t collision_rejections{0U};
  double candidate_path_evaluation_duration_ms{0.0};
  double candidate_score_duration_ms{0.0};
  double candidate_point_build_duration_ms{0.0};
  double candidate_sample_build_duration_ms{0.0};
  double candidate_cost_breakdown_duration_ms{0.0};
  double candidate_shape_diagnostics_duration_ms{0.0};
  double candidate_speed_profile_duration_ms{0.0};
  std::size_t candidate_speed_profile_calls{0U};
  std::size_t candidate_speed_profile_samples_total{0U};
  std::size_t candidate_speed_profile_samples_max{0U};
  double regularization_duration_ms{0.0};
  std::size_t scratch_reused_candidates{0U};
  bool parallel_candidate_evaluation_used{false};
  std::size_t parallel_workers_used{0U};
  std::size_t candidate_chunks{0U};
  std::size_t candidate_parallel_batches{0U};
  std::size_t candidate_threads_launched{0U};
  double candidate_batch_wall_duration_ms{0.0};
  double candidate_batch_wait_duration_ms{0.0};
  double candidate_worker_buffer_prepare_duration_ms{0.0};
  double candidate_thread_launch_duration_ms{0.0};
  double candidate_thread_join_wait_duration_ms{0.0};
  std::size_t worker_scratch_reuses{0U};
  std::size_t candidate_snapshot_allocations_avoided{0U};
  std::size_t candidate_offset_changed_samples_total{0U};
  std::size_t candidate_offset_changed_samples_max{0U};
  std::size_t candidate_offset_changed_span_samples_total{0U};
  std::size_t candidate_offset_changed_span_samples_max{0U};
  std::size_t candidate_local_speed_window_samples_total{0U};
  std::size_t candidate_local_speed_window_samples_max{0U};
  std::size_t local_candidate_evaluations{0U};
  std::size_t local_candidate_full_score_fallbacks{0U};
  std::size_t local_candidate_full_score_required{0U};
  std::size_t local_candidate_full_score_required_invalid_input{0U};
  std::size_t local_candidate_full_score_required_boundary{0U};
  std::size_t local_candidate_full_score_required_unsafe_base{0U};
  std::size_t local_candidate_full_score_required_window_invalid{0U};
  std::size_t local_candidate_acceptance_full_scores{0U};
  std::size_t local_score_false_positives{0U};
  double local_candidate_point_build_duration_ms{0.0};
  double local_candidate_path_evaluation_duration_ms{0.0};
  double local_candidate_score_duration_ms{0.0};
  double local_candidate_traversal_estimate_duration_ms{0.0};
  double full_candidate_score_duration_ms{0.0};
  std::size_t shadow_lower_bound_validation_full_scores{0U};
  double shadow_lower_bound_validation_full_score_duration_ms{0.0};
  std::size_t shadow_lower_bound_evaluations{0U};
  std::size_t shadow_lower_bound_unavailable{0U};
  std::size_t shadow_lower_bound_prunable{0U};
  std::size_t shadow_lower_bound_false_prunes{0U};
  std::size_t shadow_lower_bound_winner_prunes{0U};
  double shadow_lower_bound_prunable_full_score_duration_ms{0.0};
  double shadow_lower_bound_max_overestimate_score{0.0};
  double shadow_lower_bound_max_underestimate_score{0.0};
  double shadow_lower_bound_max_false_prune_improvement_score{0.0};
  std::size_t shadow_local_speed_evaluations{0U};
  std::size_t shadow_local_speed_unavailable{0U};
  std::size_t shadow_local_speed_prunable{0U};
  std::size_t shadow_local_speed_false_prunes{0U};
  std::size_t shadow_local_speed_winner_mismatches{0U};
  double shadow_local_speed_abs_time_error_sum_s{0.0};
  double shadow_local_speed_abs_time_error_p95_s{0.0};
  double shadow_local_speed_max_time_overestimate_s{0.0};
  double shadow_local_speed_max_time_underestimate_s{0.0};
  double shadow_local_speed_abs_score_error_sum{0.0};
  double shadow_local_speed_abs_score_error_p95{0.0};
  double shadow_local_speed_max_score_overestimate{0.0};
  double shadow_local_speed_max_score_underestimate{0.0};
  double shadow_local_speed_max_false_prune_improvement_score{0.0};
  std::size_t shadow_bounded_speed_evaluations{0U};
  std::size_t shadow_bounded_speed_unavailable{0U};
  std::size_t shadow_bounded_speed_prunable{0U};
  std::size_t shadow_bounded_speed_false_prunes{0U};
  std::size_t shadow_bounded_speed_winner_mismatches{0U};
  std::size_t shadow_bounded_speed_window_samples_total{0U};
  std::size_t shadow_bounded_speed_window_samples_max{0U};
  double shadow_bounded_speed_duration_ms{0.0};
  double shadow_bounded_speed_abs_time_error_sum_s{0.0};
  double shadow_bounded_speed_abs_time_error_p95_s{0.0};
  double shadow_bounded_speed_max_time_overestimate_s{0.0};
  double shadow_bounded_speed_max_time_underestimate_s{0.0};
  double shadow_bounded_speed_abs_score_error_sum{0.0};
  double shadow_bounded_speed_abs_score_error_p95{0.0};
  double shadow_bounded_speed_max_score_overestimate{0.0};
  double shadow_bounded_speed_max_score_underestimate{0.0};
  double shadow_bounded_speed_max_false_prune_improvement_score{0.0};
  std::size_t shadow_segment_score_evaluations{0U};
  std::size_t shadow_segment_score_unavailable{0U};
  std::size_t shadow_segment_score_prunable{0U};
  std::size_t shadow_segment_score_false_prunes{0U};
  std::size_t shadow_segment_score_winner_mismatches{0U};
  std::size_t shadow_segment_score_window_samples_total{0U};
  std::size_t shadow_segment_score_window_samples_max{0U};
  double shadow_segment_score_abs_error_sum{0.0};
  double shadow_segment_score_abs_error_p95{0.0};
  double shadow_segment_score_max_overestimate{0.0};
  double shadow_segment_score_max_underestimate{0.0};
  double shadow_segment_score_max_false_prune_improvement_score{0.0};
  std::size_t window_count{0U};
  std::size_t active_window_count{0U};
  std::size_t active_window_samples{0U};
  std::size_t dp_states{0U};
  std::size_t dp_transitions{0U};
  std::size_t dp_segment_cache_hits{0U};
  std::size_t dp_segment_cache_misses{0U};
  std::size_t candidate_segment_cache_hits{0U};
  std::size_t candidate_segment_cache_misses{0U};
  std::size_t full_path_segment_cache_hits{0U};
  std::size_t full_path_segment_cache_misses{0U};
  std::size_t dp_coarse_states{0U};
  std::size_t dp_coarse_transitions{0U};
  std::size_t dp_fine_states{0U};
  std::size_t dp_fine_transitions{0U};
  bool dp_coarse_to_fine_used{false};
  double window_detection_duration_ms{0.0};
  double window_eval_duration_ms{0.0};
  double dp_duration_ms{0.0};
  double full_final_score_duration_ms{0.0};
  bool async_refined{false};
  double initial_cost{0.0};
  double final_cost{0.0};
  double centerline_length_m{0.0};
  double final_length_m{0.0};
  double final_length_ratio{std::numeric_limits<double>::quiet_NaN()};
  double cost_length{std::numeric_limits<double>::quiet_NaN()};
  double cost_time{std::numeric_limits<double>::quiet_NaN()};
  double cost_curvature{std::numeric_limits<double>::quiet_NaN()};
  double cost_curvature_change{std::numeric_limits<double>::quiet_NaN()};
  double cost_heading_jump{std::numeric_limits<double>::quiet_NaN()};
  double cost_offset_change{std::numeric_limits<double>::quiet_NaN()};
  double cost_offset_second_change{std::numeric_limits<double>::quiet_NaN()};
  double cost_offset_slope{std::numeric_limits<double>::quiet_NaN()};
  double cost_collision{std::numeric_limits<double>::quiet_NaN()};
  double cost_outside_grid{std::numeric_limits<double>::quiet_NaN()};
  double cost_length_overrun{std::numeric_limits<double>::quiet_NaN()};
  double estimated_time_s{std::numeric_limits<double>::quiet_NaN()};
  double min_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double max_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  std::size_t curvature_limited_samples{0U};
  double centerline_estimated_time_s{std::numeric_limits<double>::quiet_NaN()};
  double centerline_min_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double centerline_max_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  std::size_t centerline_curvature_limited_samples{0U};
  double best_candidate_estimated_time_s{std::numeric_limits<double>::quiet_NaN()};
  double best_candidate_score{std::numeric_limits<double>::quiet_NaN()};
  double best_candidate_min_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double best_candidate_max_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  std::size_t best_candidate_curvature_limited_samples{0U};
  double time_gain_s{std::numeric_limits<double>::quiet_NaN()};
  double regularization_time_delta_s{std::numeric_limits<double>::quiet_NaN()};
  bool regularization_applied{false};
  std::size_t regularization_iterations{0U};
  double pre_regularization_max_curvature_jump_1pm{
      std::numeric_limits<double>::quiet_NaN()};
  double post_regularization_max_curvature_jump_1pm{
      std::numeric_limits<double>::quiet_NaN()};
  double max_abs_offset_m{0.0};
  double min_edge_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double mean_edge_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double max_abs_curvature_1pm{0.0};
  double mean_abs_curvature_1pm{0.0};
};

struct RacingLineWindowMetadata {
  std::size_t id{0U};
  double begin_s_m{0.0};
  double end_s_m{0.0};
};

struct RacingLineResult {
  std::vector<TrajectoryPointSample> samples;
  std::vector<RacingLineWindowMetadata> active_windows;
  RacingLineStats stats{};
  bool valid{false};
};

[[nodiscard]] RacingLineResult
optimizeRacingLine(std::span<const CorridorSample> corridor_samples,
                   const OccupancyGrid2D& prohibited_grid,
                   const RacingLineConfig& config,
                   const VelocityFollowerConfig& speed_config);

} // namespace drone_city_nav
