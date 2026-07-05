#include <set>
#include <string>

#include "trajectory_diagnostics_io_test_helpers.hpp"

namespace drone_city_nav {

using trajectory_diagnostics_io_test_helpers::expectContainsAll;
using trajectory_diagnostics_io_test_helpers::populatedStats;

namespace {

[[nodiscard]] std::size_t findJsonStringEnd(const std::string& json,
                                            const std::size_t string_start) {
  for (std::size_t index = string_start + 1U; index < json.size(); ++index) {
    if (json[index] == '\\') {
      ++index;
      continue;
    }
    if (json[index] == '"') {
      return index;
    }
  }
  return std::string::npos;
}

void expectUniqueTopLevelJsonObjectKeys(const std::string& json) {
  std::set<std::string> keys;
  int object_depth = 0;
  int array_depth = 0;
  for (std::size_t index = 0U; index < json.size(); ++index) {
    const char token = json[index];
    if (token == '"') {
      const std::size_t key_end = findJsonStringEnd(json, index);
      if (key_end == std::string::npos) {
        ADD_FAILURE() << "unterminated JSON string at " << index;
        return;
      }
      const bool top_level_object_key = object_depth == 1 && array_depth == 0;
      if (top_level_object_key) {
        const std::size_t previous_token_index =
            index == 0U ? std::string::npos
                        : json.find_last_not_of(" \n\r\t", index - 1U);
        const char previous_token = previous_token_index == std::string::npos
                                        ? '\0'
                                        : json[previous_token_index];
        const std::size_t separator = json.find_first_not_of(" \n\r\t", key_end + 1U);
        if (separator == std::string::npos) {
          ADD_FAILURE() << "missing JSON key separator after " << key_end;
          return;
        }
        if ((previous_token == '{' || previous_token == ',') &&
            json[separator] == ':') {
          const std::string key = json.substr(index + 1U, key_end - index - 1U);
          EXPECT_TRUE(keys.insert(key).second) << key;
        }
      }
      index = key_end;
      continue;
    }
    if (token == '{') {
      ++object_depth;
    } else if (token == '}') {
      --object_depth;
    } else if (token == '[') {
      ++array_depth;
    } else if (token == ']') {
      --array_depth;
    }
  }
}

} // namespace

TEST(TrajectoryDiagnosticsIo, SummaryJsonContainsTraversalAndShapeMetrics) {
  TrajectoryShapeDiagnostics shape{};
  shape.segment_count = 9U;
  shape.max_curvature_jump_1pm = 0.2;
  shape.max_heading_delta_rad = 0.3;
  shape.max_offset_delta_m = 0.4;

  const std::string json =
      finalTrajectoryDiagnosticsSummaryJson(populatedStats(), shape);

  EXPECT_NE(json.find("\"trajectory_optimizer_final_estimated_time_s\":12.5"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_final_min_speed_limit_mps\":1"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_final_max_speed_limit_mps\":10"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_length_m\":100"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_final_length_ratio\":1.08"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_cost_radius_shortfall\":7.5"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_cost_heading_jump\":5.5"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_candidate_point_build_duration_ms\":1.25"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_candidate_sample_build_duration_ms\":2.5"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_candidate_cost_breakdown_duration_ms\":3.25"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_candidate_shape_diagnostics_duration_ms\":1.75"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_regularization_duration_ms\":3.75"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_scratch_reused_candidates\":13"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_parallel_candidate_evaluation_used\":true"),
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
  EXPECT_NE(json.find("\"trajectory_optimizer_parallel_workers_used\":2"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_candidate_chunks\":31"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_candidate_parallel_batches\":29"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_candidate_threads_launched\":58"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_candidate_batch_wall_duration_ms\":12.25"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_candidate_batch_wait_duration_ms\":10.5"),
            std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_candidate_worker_buffer_prepare_duration_ms\":1.5"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_candidate_thread_launch_duration_ms\":2.75"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_candidate_thread_join_wait_duration_ms\":8"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_worker_scratch_reuses\":62"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_candidate_snapshot_allocations_avoided\":60"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_candidate_offset_changed_samples_total\":180"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_candidate_offset_changed_samples_max\":7"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_candidate_offset_changed_span_samples_total\":220"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_candidate_offset_changed_span_samples_max\":9"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_candidate_local_speed_window_samples_total\":930"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_candidate_local_speed_window_samples_max\":35"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_local_candidate_evaluations\":61"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_local_candidate_full_score_fallbacks\":55"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_local_candidate_full_score_required\":10"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_local_candidate_full_score_required_"
                      "invalid_input\":1"),
            std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_local_candidate_full_score_required_boundary\":2"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_local_candidate_full_score_required_unsafe_base\":3"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_local_candidate_full_score_required_"
                      "window_invalid\":4"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_local_candidate_acceptance_full_scores\":7"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_local_candidate_point_build_duration_ms\":1.1"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_local_candidate_path_evaluation_duration_ms\":2.2"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_shadow_segment_score_evaluations\":52"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_shadow_segment_score_unavailable\":9"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_shadow_segment_score_prunable\":19"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_shadow_segment_score_false_prunes\":1"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_shadow_segment_score_winner_mismatches\":3"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_shadow_segment_score_window_samples_total\":572"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_shadow_segment_score_window_samples_max\":11"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_shadow_segment_score_abs_error_sum\":0.35"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_shadow_segment_score_abs_error_p95\":0.05"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_shadow_segment_score_max_overestimate\":0.2"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_shadow_segment_score_max_underestimate\":0.15"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_shadow_segment_score_max_false_prune_"
                      "improvement_score\":0.75"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_shadow_boundary_clamped_local_candidates\":11"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_shadow_boundary_clamped_window_samples_total\":121"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_shadow_boundary_clamped_window_samples_max\":13"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_dp_coarse_to_fine_used\":true"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_window_count\":4"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_active_window_count\":3"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_active_window_centerline_blocked\":1"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_active_window_heading_change_samples\":5"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_active_window_heading_span_samples\":6"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_active_window_curvature_samples\":7"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_active_window_width_change_samples\":8"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_active_window_width_asymmetry_samples\":9"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_shadow_active_window_no_width_asymmetry_count\":2"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_shadow_active_window_no_width_asymmetry_"
                      "samples\":16"),
            std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_shadow_active_window_no_width_triggers_count\":1"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_shadow_active_window_no_width_triggers_samples\":12"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_shadow_active_window_no_heading_span_count\":3"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_shadow_active_window_no_heading_span_samples\":14"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_windows\":5"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_window_samples\":19"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_window_merged_count\":3"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_prohibited_cells\":10"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_outside_grid_segments\":11"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_segment_count\":3"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_span_count\":2"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_first_segment_index\":12"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_last_segment_index\":14"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_first_s_m\":42.5"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_last_s_m\":48.75"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_span_length_m\":6.25"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_first_x_m\":13.25"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_first_y_m\":-8.75"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_last_x_m\":16.5"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_last_y_m\":-9.25"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_first_outside_grid\":true"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_last_outside_grid\":true"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_span_diagnostic_count\":2"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_centerline_blocked_span0_begin_segment_index\":12"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_centerline_blocked_span0_end_segment_index\":13"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_span0_begin_s_m\":42.5"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_span0_end_s_m\":45.25"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_span0_length_m\":2.75"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_span0_begin_x_m\":13.25"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_span0_begin_y_m\":-8.75"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_span0_end_x_m\":14.5"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_span0_end_y_m\":-9"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_span0_prohibited_cells\":6"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_centerline_blocked_span0_outside_grid_segments\":0"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_centerline_blocked_span1_begin_segment_index\":14"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_centerline_blocked_span1_end_segment_index\":14"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_centerline_blocked_span1_begin_s_m\":48"),
            std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_span1_end_s_m\":48.75"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_span1_length_m\":0.75"),
      std::string::npos);
  EXPECT_NE(
      json.find("\"trajectory_optimizer_centerline_blocked_span1_prohibited_cells\":4"),
      std::string::npos);
  EXPECT_NE(
      json.find(
          "\"trajectory_optimizer_centerline_blocked_span1_outside_grid_segments\":1"),
      std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_dp_states\":144"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_dp_coarse_states\":44"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_dp_fine_transitions\":400"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_full_path_segment_cache_hits\":14"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_full_path_segment_cache_misses\":88"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_quality\":\"refined\""), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_async_refined\":true"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_total_duration_ms\":123.4"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_trajectory_optimizer_duration_ms\":99.9"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_optimizer_regularization_applied\":true"),
            std::string::npos);
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

TEST(TrajectoryDiagnosticsIo,
     TrajectoryOptimizerJsonFragmentContainsBlackboxRequiredKeys) {
  const std::string fragment =
      trajectoryOptimizerDiagnosticsJsonFields(populatedStats());

  expectContainsAll(
      fragment,
      std::array{
          "\"trajectory_optimizer_final_estimated_time_s\"",
          "\"trajectory_optimizer_final_min_speed_limit_mps\"",
          "\"trajectory_optimizer_final_max_speed_limit_mps\"",
          "\"trajectory_optimizer_final_curvature_limited_samples\"",
          "\"trajectory_optimizer_centerline_length_m\"",
          "\"trajectory_optimizer_final_length_m\"",
          "\"trajectory_optimizer_final_length_ratio\"",
          "\"trajectory_optimizer_max_abs_offset_m\"",
          "\"trajectory_optimizer_min_edge_margin_m\"",
          "\"trajectory_optimizer_mean_edge_margin_m\"",
          "\"trajectory_optimizer_cost_curvature\"",
          "\"trajectory_optimizer_cost_curvature_change\"",
          "\"trajectory_optimizer_cost_radius_shortfall\"",
          "\"trajectory_optimizer_cost_heading_jump\"",
          "\"trajectory_optimizer_cost_offset_change\"",
          "\"trajectory_optimizer_cost_offset_second_change\"",
          "\"trajectory_optimizer_cost_offset_slope\"",
          "\"trajectory_optimizer_cost_collision\"",
          "\"trajectory_optimizer_cost_outside_grid\"",
          "\"trajectory_optimizer_best_candidate_score\"",
          "\"trajectory_optimizer_regularization_iterations\"",
          "\"trajectory_optimizer_regularization_applied\"",
          "\"trajectory_optimizer_pre_regularization_max_curvature_jump_1pm\"",
          "\"trajectory_optimizer_post_regularization_max_curvature_jump_1pm\"",
          "\"trajectory_optimizer_candidate_path_evaluation_duration_ms\"",
          "\"trajectory_optimizer_candidate_score_duration_ms\"",
          "\"trajectory_optimizer_candidate_point_build_duration_ms\"",
          "\"trajectory_optimizer_candidate_sample_build_duration_ms\"",
          "\"trajectory_optimizer_candidate_cost_breakdown_duration_ms\"",
          "\"trajectory_optimizer_candidate_shape_diagnostics_duration_ms\"",
          "\"trajectory_optimizer_regularization_duration_ms\"",
          "\"trajectory_optimizer_scratch_reused_candidates\"",
          "\"trajectory_optimizer_parallel_candidate_evaluation_used\"",
          "\"trajectory_optimizer_parallel_workers_used\"",
          "\"trajectory_optimizer_candidate_chunks\"",
          "\"trajectory_optimizer_candidate_parallel_batches\"",
          "\"trajectory_optimizer_candidate_threads_launched\"",
          "\"trajectory_optimizer_candidate_batch_wall_duration_ms\"",
          "\"trajectory_optimizer_candidate_batch_wait_duration_ms\"",
          "\"trajectory_optimizer_candidate_worker_buffer_prepare_duration_ms\"",
          "\"trajectory_optimizer_candidate_thread_launch_duration_ms\"",
          "\"trajectory_optimizer_candidate_thread_join_wait_duration_ms\"",
          "\"trajectory_optimizer_worker_scratch_reuses\"",
          "\"trajectory_optimizer_candidate_snapshot_allocations_avoided\"",
          "\"trajectory_optimizer_candidate_offset_changed_samples_total\"",
          "\"trajectory_optimizer_candidate_offset_changed_samples_max\"",
          "\"trajectory_optimizer_candidate_offset_changed_span_samples_total\"",
          "\"trajectory_optimizer_candidate_offset_changed_span_samples_max\"",
          "\"trajectory_optimizer_candidate_local_speed_window_samples_total\"",
          "\"trajectory_optimizer_candidate_local_speed_window_samples_max\"",
          "\"trajectory_optimizer_local_candidate_evaluations\"",
          "\"trajectory_optimizer_local_candidate_full_score_fallbacks\"",
          "\"trajectory_optimizer_local_candidate_full_score_required\"",
          "\"trajectory_optimizer_local_candidate_full_score_required_invalid_input\"",
          "\"trajectory_optimizer_local_candidate_full_score_required_boundary\"",
          "\"trajectory_optimizer_local_candidate_full_score_required_unsafe_base\"",
          "\"trajectory_optimizer_local_candidate_full_score_required_window_invalid\"",
          "\"trajectory_optimizer_local_candidate_acceptance_full_scores\"",
          "\"trajectory_optimizer_local_score_false_positives\"",
          "\"trajectory_optimizer_local_candidate_point_build_duration_ms\"",
          "\"trajectory_optimizer_local_candidate_path_evaluation_duration_ms\"",
          "\"trajectory_optimizer_local_candidate_score_duration_ms\"",
          "\"trajectory_optimizer_full_candidate_score_duration_ms\"",
          "\"trajectory_optimizer_shadow_segment_score_evaluations\"",
          "\"trajectory_optimizer_shadow_segment_score_unavailable\"",
          "\"trajectory_optimizer_shadow_segment_score_prunable\"",
          "\"trajectory_optimizer_shadow_segment_score_false_prunes\"",
          "\"trajectory_optimizer_shadow_segment_score_winner_mismatches\"",
          "\"trajectory_optimizer_shadow_segment_score_window_samples_total\"",
          "\"trajectory_optimizer_shadow_segment_score_window_samples_max\"",
          "\"trajectory_optimizer_shadow_segment_score_abs_error_sum\"",
          "\"trajectory_optimizer_shadow_segment_score_abs_error_p95\"",
          "\"trajectory_optimizer_shadow_segment_score_max_overestimate\"",
          "\"trajectory_optimizer_shadow_segment_score_max_underestimate\"",
          "\"trajectory_optimizer_shadow_segment_score_max_false_prune_improvement_"
          "score\"",
          "\"trajectory_optimizer_shadow_boundary_clamped_local_candidates\"",
          "\"trajectory_optimizer_shadow_boundary_clamped_window_samples_total\"",
          "\"trajectory_optimizer_shadow_boundary_clamped_window_samples_max\"",
          "\"trajectory_optimizer_window_count\"",
          "\"trajectory_optimizer_active_window_count\"",
          "\"trajectory_optimizer_active_window_samples\"",
          "\"trajectory_optimizer_active_window_centerline_blocked\"",
          "\"trajectory_optimizer_active_window_heading_change_samples\"",
          "\"trajectory_optimizer_active_window_heading_span_samples\"",
          "\"trajectory_optimizer_active_window_curvature_samples\"",
          "\"trajectory_optimizer_active_window_width_change_samples\"",
          "\"trajectory_optimizer_active_window_width_asymmetry_samples\"",
          "\"trajectory_optimizer_shadow_active_window_no_width_asymmetry_count\"",
          "\"trajectory_optimizer_shadow_active_window_no_width_asymmetry_samples\"",
          "\"trajectory_optimizer_shadow_active_window_no_width_triggers_count\"",
          "\"trajectory_optimizer_shadow_active_window_no_width_triggers_samples\"",
          "\"trajectory_optimizer_shadow_active_window_no_heading_span_count\"",
          "\"trajectory_optimizer_shadow_active_window_no_heading_span_samples\"",
          "\"trajectory_optimizer_centerline_blocked_windows\"",
          "\"trajectory_optimizer_centerline_blocked_window_samples\"",
          "\"trajectory_optimizer_centerline_blocked_window_merged_count\"",
          "\"trajectory_optimizer_centerline_blocked_prohibited_cells\"",
          "\"trajectory_optimizer_centerline_blocked_outside_grid_segments\"",
          "\"trajectory_optimizer_centerline_blocked_segment_count\"",
          "\"trajectory_optimizer_centerline_blocked_span_count\"",
          "\"trajectory_optimizer_centerline_blocked_first_segment_index\"",
          "\"trajectory_optimizer_centerline_blocked_last_segment_index\"",
          "\"trajectory_optimizer_centerline_blocked_first_s_m\"",
          "\"trajectory_optimizer_centerline_blocked_last_s_m\"",
          "\"trajectory_optimizer_centerline_blocked_span_length_m\"",
          "\"trajectory_optimizer_centerline_blocked_first_x_m\"",
          "\"trajectory_optimizer_centerline_blocked_first_y_m\"",
          "\"trajectory_optimizer_centerline_blocked_last_x_m\"",
          "\"trajectory_optimizer_centerline_blocked_last_y_m\"",
          "\"trajectory_optimizer_centerline_blocked_first_outside_grid\"",
          "\"trajectory_optimizer_centerline_blocked_last_outside_grid\"",
          "\"trajectory_optimizer_centerline_blocked_span_diagnostic_count\"",
          "\"trajectory_optimizer_centerline_blocked_span0_begin_segment_index\"",
          "\"trajectory_optimizer_centerline_blocked_span0_end_segment_index\"",
          "\"trajectory_optimizer_centerline_blocked_span0_begin_s_m\"",
          "\"trajectory_optimizer_centerline_blocked_span0_end_s_m\"",
          "\"trajectory_optimizer_centerline_blocked_span0_length_m\"",
          "\"trajectory_optimizer_centerline_blocked_span0_begin_x_m\"",
          "\"trajectory_optimizer_centerline_blocked_span0_begin_y_m\"",
          "\"trajectory_optimizer_centerline_blocked_span0_end_x_m\"",
          "\"trajectory_optimizer_centerline_blocked_span0_end_y_m\"",
          "\"trajectory_optimizer_centerline_blocked_span0_prohibited_cells\"",
          "\"trajectory_optimizer_centerline_blocked_span0_outside_grid_segments\"",
          "\"trajectory_optimizer_dp_states\"",
          "\"trajectory_optimizer_dp_transitions\"",
          "\"trajectory_optimizer_dp_segment_cache_hits\"",
          "\"trajectory_optimizer_dp_segment_cache_misses\"",
          "\"trajectory_optimizer_candidate_segment_cache_hits\"",
          "\"trajectory_optimizer_candidate_segment_cache_misses\"",
          "\"trajectory_optimizer_full_path_segment_cache_hits\"",
          "\"trajectory_optimizer_full_path_segment_cache_misses\"",
          "\"trajectory_optimizer_dp_coarse_states\"",
          "\"trajectory_optimizer_dp_coarse_transitions\"",
          "\"trajectory_optimizer_dp_fine_states\"",
          "\"trajectory_optimizer_dp_fine_transitions\"",
          "\"trajectory_optimizer_dp_coarse_to_fine_used\"",
          "\"trajectory_optimizer_window_detection_duration_ms\"",
          "\"trajectory_optimizer_window_eval_duration_ms\"",
          "\"trajectory_optimizer_dp_duration_ms\"",
          "\"trajectory_optimizer_full_final_score_duration_ms\"",
          "\"trajectory_optimizer_async_refined\"",
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
                                  "\"turn_smoothing_rejected_not_improved\"",
                                  "\"turn_smoothing_rejected_curvature_regression\"",
                                  "\"turn_smoothing_rejected_radius_regression\"",
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

  expectContainsAll(fragment, std::array{
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
                                  "\"isolated_curvature_spike_max_before_1pm\"",
                                  "\"isolated_curvature_spike_max_after_1pm\"",
                              });
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, PlannerDiagnosticsJsonHasUniqueKeys) {
  const std::string json = trajectoryPlannerDiagnosticsJson(1U, 2U, populatedStats());

  expectUniqueTopLevelJsonObjectKeys(json);
}

TEST(TrajectoryDiagnosticsIo, TimingJsonFragmentContainsBlackboxRequiredKeys) {
  const std::string fragment = trajectoryTimingDiagnosticsJsonFields(populatedStats());

  expectContainsAll(fragment, std::array{
                                  "\"trajectory_total_duration_ms\"",
                                  "\"trajectory_corridor_duration_ms\"",
                                  "\"trajectory_trajectory_optimizer_duration_ms\"",
                                  "\"trajectory_turn_smoothing_duration_ms\"",
                                  "\"trajectory_speed_profile_duration_ms\"",
                              });
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo,
     TrajectoryOptimizerJsonFragmentWritesNullForNonFiniteMetrics) {
  const std::string fragment =
      trajectoryOptimizerDiagnosticsJsonFields(TrajectoryPlannerStats{});

  EXPECT_NE(fragment.find("\"trajectory_optimizer_final_estimated_time_s\":null"),
            std::string::npos);
  EXPECT_NE(fragment.find("\"trajectory_optimizer_best_candidate_score\":null"),
            std::string::npos);
  EXPECT_NE(fragment.find("\"trajectory_optimizer_final_length_ratio\":null"),
            std::string::npos);
  EXPECT_NE(fragment.find("\"trajectory_optimizer_cost_radius_shortfall\":null"),
            std::string::npos);
  EXPECT_NE(fragment.find("\"trajectory_optimizer_cost_heading_jump\":null"),
            std::string::npos);
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

} // namespace drone_city_nav
