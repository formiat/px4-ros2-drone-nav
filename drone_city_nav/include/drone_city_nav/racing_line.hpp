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
  double weight_curvature{250.0};
  double weight_curvature_change{100.0};
  double weight_offset_change{0.5};
  double weight_offset_second_change{5.0};
  double weight_time{50.0};
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
  double regularization_duration_ms{0.0};
  std::size_t scratch_reused_candidates{0U};
  bool parallel_candidate_evaluation_used{false};
  std::size_t parallel_workers_used{0U};
  std::size_t candidate_chunks{0U};
  std::size_t worker_scratch_reuses{0U};
  std::size_t candidate_snapshot_allocations_avoided{0U};
  std::size_t local_candidate_evaluations{0U};
  std::size_t local_candidate_full_score_fallbacks{0U};
  std::size_t local_candidate_acceptance_full_scores{0U};
  std::size_t local_score_false_positives{0U};
  double local_candidate_score_duration_ms{0.0};
  double full_candidate_score_duration_ms{0.0};
  std::size_t window_count{0U};
  std::size_t active_window_count{0U};
  std::size_t active_window_samples{0U};
  std::size_t dp_states{0U};
  std::size_t dp_transitions{0U};
  std::size_t dp_segment_cache_hits{0U};
  std::size_t dp_segment_cache_misses{0U};
  std::size_t candidate_segment_cache_hits{0U};
  std::size_t candidate_segment_cache_misses{0U};
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
