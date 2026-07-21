#include "trajectory_diagnostics_io_test_helpers.hpp"

namespace drone_city_nav {

using trajectory_diagnostics_io_test_helpers::populatedStats;

TEST(TrajectoryDiagnosticsIo, PlannerDiagnosticsJsonRoundTripsRuntimeStats) {
  const std::uint64_t planner_path_id = 42U;
  const std::uint64_t path_stamp_ns = 1'782'477'871'305'471'587ULL;
  const TrajectoryDeliveryDiagnostics delivery{
      .generation = 17U,
      .blocker_detected_stamp_ns = 1'000'000'000U,
      .trajectory_build_started_stamp_ns = 1'100'000'000U,
      .path_published_stamp_ns = 1'900'000'000U,
      .blocked_path_id = 41U,
      .truncation_generation = 9U,
      .temporary_prefix_fingerprint = 123456U,
      .replan_triggered = true,
      .truncation_suffix = true,
      .truncation_immediate_hold = true,
      .blocker_position = Point2{12.5, 30.25},
      .blocker_detection_position = Point2{5.0, 6.0},
      .blocker_detection_velocity = Point2{10.0, -2.0},
      .blocker_detection_velocity_valid = true,
      .candidate_start_position = Point2{5.5, 6.5},
      .planning_start_position = Point2{6.0, 7.0},
      .planning_start_velocity = Point2{11.0, -1.5},
      .planning_start_velocity_valid = true,
      .predicted_publication_position = Point2{14.8, 5.8},
      .predicted_publication_position_valid = true,
      .actual_publication_position = Point2{14.1, 6.2},
      .actual_publication_position_valid = true,
      .blocker_to_build_start_ms = 100.0,
      .build_start_to_publish_ms = 800.0,
      .blocker_to_publish_ms = 900.0,
      .publication_prediction_error_m = 0.806,
  };
  const std::string json = trajectoryPlannerDiagnosticsJson(
      planner_path_id, path_stamp_ns, populatedStats(), delivery);
  EXPECT_NE(json.find("\"trajectory_quality\":\"refined\""), std::string::npos);

  const std::optional<TrajectoryPlannerDiagnosticsEnvelope> parsed =
      parseTrajectoryPlannerDiagnosticsJson(json);

  ASSERT_TRUE(parsed.has_value());
  const TrajectoryPlannerDiagnosticsEnvelope parsed_value =
      parsed.value_or(TrajectoryPlannerDiagnosticsEnvelope{});
  EXPECT_EQ(parsed_value.planner_path_id, planner_path_id);
  EXPECT_EQ(parsed_value.path_stamp_ns, path_stamp_ns);
  EXPECT_EQ(parsed_value.delivery.generation, 17U);
  EXPECT_TRUE(parsed_value.delivery.replan_triggered);
  EXPECT_TRUE(parsed_value.delivery.truncation_suffix);
  EXPECT_TRUE(parsed_value.delivery.truncation_immediate_hold);
  EXPECT_EQ(parsed_value.delivery.blocked_path_id, 41U);
  EXPECT_EQ(parsed_value.delivery.truncation_generation, 9U);
  EXPECT_EQ(parsed_value.delivery.temporary_prefix_fingerprint, 123456U);
  EXPECT_EQ(parsed_value.delivery.blocker_detected_stamp_ns, 1'000'000'000U);
  EXPECT_EQ(parsed_value.delivery.trajectory_build_started_stamp_ns, 1'100'000'000U);
  EXPECT_EQ(parsed_value.delivery.path_published_stamp_ns, 1'900'000'000U);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.blocker_position.x, 12.5);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.blocker_detection_position.y, 6.0);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.blocker_detection_velocity.x, 10.0);
  EXPECT_TRUE(parsed_value.delivery.blocker_detection_velocity_valid);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.candidate_start_position.x, 5.5);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.candidate_start_position.y, 6.5);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.planning_start_position.x, 6.0);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.planning_start_velocity.y, -1.5);
  EXPECT_TRUE(parsed_value.delivery.planning_start_velocity_valid);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.predicted_publication_position.x, 14.8);
  EXPECT_TRUE(parsed_value.delivery.predicted_publication_position_valid);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.actual_publication_position.y, 6.2);
  EXPECT_TRUE(parsed_value.delivery.actual_publication_position_valid);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.blocker_to_build_start_ms, 100.0);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.build_start_to_publish_ms, 800.0);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.blocker_to_publish_ms, 900.0);
  EXPECT_DOUBLE_EQ(parsed_value.delivery.publication_prediction_error_m, 0.806);
  EXPECT_EQ(parsed_value.stats.quality, TrajectoryQuality::kRefined);
  EXPECT_EQ(parsed_value.stats.samples, 78U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.length_m, 412.25);
  EXPECT_EQ(parsed_value.stats.speed_profile_construction_config_fingerprint, 0x3456U);
  EXPECT_EQ(parsed_value.stats.runtime_speed_policy_config_fingerprint, 0x4567U);
  EXPECT_EQ(parsed_value.stats.runtime_velocity_control_config_fingerprint, 0x5678U);
  EXPECT_TRUE(parsed_value.stats.known_passage_validation.enabled);
  EXPECT_FALSE(parsed_value.stats.known_passage_validation.valid);
  EXPECT_EQ(parsed_value.stats.known_passage_validation.structures_checked, 2U);
  EXPECT_EQ(parsed_value.stats.known_passage_validation.structures_intersected, 1U);
  EXPECT_EQ(parsed_value.stats.known_passage_validation.opening_matches, 1U);
  EXPECT_EQ(parsed_value.stats.known_passage_validation.violations, 1U);
  EXPECT_EQ(parsed_value.stats.known_passage_validation.worst_reason,
            KnownPassageValidationReason::kOpeningVolumeMiss);
  ASSERT_EQ(parsed_value.stats.known_passage_validation.diagnostics.size(), 2U);
  const KnownPassageValidationSpan& passage0 =
      parsed_value.stats.known_passage_validation.diagnostics[0];
  EXPECT_EQ(passage0.structure_id, "arch_01");
  EXPECT_EQ(passage0.opening_id, "main");
  EXPECT_DOUBLE_EQ(passage0.entry_s_m, 42.5);
  EXPECT_DOUBLE_EQ(passage0.exit_s_m, 48.75);
  EXPECT_DOUBLE_EQ(passage0.overlap_m, 3.25);
  EXPECT_DOUBLE_EQ(passage0.clearance_m, 1.5);
  EXPECT_EQ(passage0.reason, KnownPassageValidationReason::kMatchedOpening);
  EXPECT_TRUE(passage0.valid);
  const KnownPassageValidationSpan& passage1 =
      parsed_value.stats.known_passage_validation.diagnostics[1];
  EXPECT_EQ(passage1.structure_id, "arch_02");
  EXPECT_TRUE(passage1.opening_id.empty());
  EXPECT_DOUBLE_EQ(passage1.clearance_m, -1.25);
  EXPECT_EQ(passage1.reason, KnownPassageValidationReason::kOpeningVolumeMiss);
  EXPECT_FALSE(passage1.valid);
  EXPECT_TRUE(parsed_value.stats.vertical_profile.enabled);
  EXPECT_TRUE(parsed_value.stats.vertical_profile.active);
  EXPECT_TRUE(parsed_value.stats.vertical_profile.applied);
  EXPECT_TRUE(parsed_value.stats.vertical_profile.valid);
  EXPECT_EQ(parsed_value.stats.vertical_profile.passages_matched, 1U);
  EXPECT_EQ(parsed_value.stats.vertical_profile.passages_profiled, 1U);
  EXPECT_EQ(parsed_value.stats.vertical_profile.infeasible_count, 0U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.vertical_profile.min_z_m, 10.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.vertical_profile.max_z_m, 18.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.vertical_profile.max_abs_dz_ds, 0.18);
  EXPECT_DOUBLE_EQ(parsed_value.stats.vertical_profile.max_abs_d2z_ds2, 0.04);
  EXPECT_DOUBLE_EQ(parsed_value.stats.vertical_profile.max_abs_d3z_ds3, 0.02);
  EXPECT_DOUBLE_EQ(parsed_value.stats.vertical_profile.min_vertical_speed_cap_mps, 7.5);
  ASSERT_EQ(parsed_value.stats.vertical_profile.diagnostics.size(), 1U);
  const VerticalProfilePassageDiagnostic& vertical0 =
      parsed_value.stats.vertical_profile.diagnostics.front();
  EXPECT_EQ(vertical0.structure_id, "arch_01");
  EXPECT_EQ(vertical0.opening_id, "main");
  EXPECT_DOUBLE_EQ(vertical0.entry_s_m, 42.5);
  EXPECT_DOUBLE_EQ(vertical0.exit_s_m, 48.75);
  EXPECT_DOUBLE_EQ(vertical0.approach_start_s_m, 30.0);
  EXPECT_DOUBLE_EQ(vertical0.gate_hold_start_s_m, 38.0);
  EXPECT_DOUBLE_EQ(vertical0.exit_end_s_m, 62.0);
  EXPECT_DOUBLE_EQ(vertical0.gate_z_m, 10.5);
  EXPECT_DOUBLE_EQ(vertical0.safe_min_z_m, 8.5);
  EXPECT_DOUBLE_EQ(vertical0.safe_max_z_m, 11.5);
  EXPECT_DOUBLE_EQ(vertical0.transition_required_m, 24.0);
  EXPECT_DOUBLE_EQ(vertical0.transition_available_m, 30.0);
  EXPECT_DOUBLE_EQ(vertical0.desired_gate_hold_m, 15.0);
  EXPECT_DOUBLE_EQ(vertical0.actual_gate_hold_m, 8.0);
  EXPECT_EQ(vertical0.reason, "profiled");
  EXPECT_TRUE(vertical0.valid);
  EXPECT_TRUE(parsed_value.stats.passage_insertion.enabled);
  EXPECT_TRUE(parsed_value.stats.passage_insertion.applied);
  EXPECT_EQ(parsed_value.stats.passage_insertion.candidates, 3U);
  EXPECT_EQ(parsed_value.stats.passage_insertion.inserted_count, 1U);
  EXPECT_EQ(parsed_value.stats.passage_insertion.rejected_join, 1U);
  EXPECT_EQ(parsed_value.stats.passage_insertion.rejected_traversability, 1U);
  EXPECT_EQ(parsed_value.stats.passage_insertion.final_reason,
            PassageInsertionRejectReason::kNone);
  ASSERT_EQ(parsed_value.stats.passage_insertion.diagnostics.size(), 1U);
  const PassageInsertionDiagnostic& insertion0 =
      parsed_value.stats.passage_insertion.diagnostics.front();
  EXPECT_EQ(insertion0.structure_id, "arch_02");
  EXPECT_EQ(insertion0.opening_id, "main");
  EXPECT_DOUBLE_EQ(insertion0.anchor_s_m, 70.0);
  EXPECT_DOUBLE_EQ(insertion0.reconnect_s_m, 116.0);
  EXPECT_DOUBLE_EQ(insertion0.lateral_miss_before_m, 2.25);
  EXPECT_DOUBLE_EQ(insertion0.lateral_miss_after_m, 0.0);
  EXPECT_EQ(insertion0.reason, PassageInsertionRejectReason::kNone);
  EXPECT_TRUE(insertion0.accepted);
  EXPECT_DOUBLE_EQ(parsed_value.stats.passage_insertion_duration_ms, 2.25);
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
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.final_length_m, 108.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.final_length_ratio, 1.08);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.min_edge_margin_m, 2.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.cost_offset_slope, 2.5);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_path_evaluation_duration_ms,
      7.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.candidate_score_duration_ms,
                   8.5);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_point_build_duration_ms, 1.25);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_sample_build_duration_ms, 2.5);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_cost_breakdown_duration_ms,
      3.25);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_shape_diagnostics_duration_ms,
      1.75);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.regularization_duration_ms,
                   3.75);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.scratch_reused_candidates, 13U);
  EXPECT_TRUE(
      parsed_value.stats.trajectory_optimizer.parallel_candidate_evaluation_used);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.parallel_workers_used, 2U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.candidate_chunks, 31U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.candidate_parallel_batches, 29U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.candidate_threads_launched, 58U);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_batch_wall_duration_ms, 12.25);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_batch_wait_duration_ms, 10.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer
                       .candidate_worker_buffer_prepare_duration_ms,
                   1.5);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_thread_launch_duration_ms,
      2.75);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_thread_join_wait_duration_ms,
      8.0);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.worker_scratch_reuses, 62U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_snapshot_allocations_avoided,
      60U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_offset_changed_samples_total,
      180U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_offset_changed_samples_max, 7U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .candidate_offset_changed_span_samples_total,
            220U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_offset_changed_span_samples_max,
      9U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .candidate_local_speed_window_samples_total,
            930U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.candidate_local_speed_window_samples_max,
      35U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.local_candidate_evaluations, 61U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.local_candidate_full_score_fallbacks,
      55U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.local_candidate_full_score_required,
            10U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .local_candidate_full_score_required_invalid_input,
            1U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .local_candidate_full_score_required_boundary,
            2U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .local_candidate_full_score_required_unsafe_base,
            3U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .local_candidate_full_score_required_window_invalid,
            4U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.local_candidate_acceptance_full_scores,
      7U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.local_score_false_positives, 1U);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.local_candidate_point_build_duration_ms,
      1.1);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer
                       .local_candidate_path_evaluation_duration_ms,
                   2.2);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.local_candidate_score_duration_ms, 4.5);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.full_candidate_score_duration_ms, 6.75);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.shadow_segment_score_evaluations,
            52U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.shadow_segment_score_unavailable,
            9U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.shadow_segment_score_prunable, 19U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.shadow_segment_score_false_prunes,
            1U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.shadow_segment_score_winner_mismatches,
      3U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.shadow_segment_score_window_samples_total,
      572U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.shadow_segment_score_window_samples_max,
      11U);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.shadow_segment_score_abs_error_sum, 0.35);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.shadow_segment_score_abs_error_p95, 0.05);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.shadow_segment_score_max_overestimate,
      0.2);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.shadow_segment_score_max_underestimate,
      0.15);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer
                       .shadow_segment_score_max_false_prune_improvement_score,
                   0.75);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.shadow_boundary_clamped_local_candidates,
      11U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .shadow_boundary_clamped_window_samples_total,
            121U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .shadow_boundary_clamped_window_samples_max,
            13U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.full_path_segment_cache_hits, 14U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.full_path_segment_cache_misses,
            88U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.window_count, 4U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.active_window_count, 3U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.active_window_samples, 18U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.active_window_centerline_blocked,
            1U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.active_window_heading_change_samples, 5U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.active_window_heading_span_samples,
            6U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.active_window_curvature_samples,
            7U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.active_window_width_change_samples,
            8U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.active_window_width_asymmetry_samples,
      9U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .shadow_active_window_no_width_asymmetry_count,
            2U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .shadow_active_window_no_width_asymmetry_samples,
            16U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .shadow_active_window_no_width_triggers_count,
            1U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .shadow_active_window_no_width_triggers_samples,
            12U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .shadow_active_window_no_heading_span_count,
            3U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer
                .shadow_active_window_no_heading_span_samples,
            14U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.centerline_blocked_windows, 5U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.centerline_blocked_window_samples,
            19U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.centerline_blocked_window_merged_count,
      3U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.centerline_blocked_prohibited_cells,
            10U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.centerline_blocked_outside_grid_segments,
      11U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.centerline_blocked_segment_count,
            3U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.centerline_blocked_span_count, 2U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.centerline_blocked_first_segment_index,
      12U);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.centerline_blocked_last_segment_index,
      14U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.centerline_blocked_first_s_m,
                   42.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.centerline_blocked_last_s_m,
                   48.75);
  EXPECT_DOUBLE_EQ(
      parsed_value.stats.trajectory_optimizer.centerline_blocked_span_length_m, 6.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.centerline_blocked_first_x_m,
                   13.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.centerline_blocked_first_y_m,
                   -8.75);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.centerline_blocked_last_x_m,
                   16.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.centerline_blocked_last_y_m,
                   -9.25);
  EXPECT_TRUE(
      parsed_value.stats.trajectory_optimizer.centerline_blocked_first_outside_grid);
  EXPECT_TRUE(
      parsed_value.stats.trajectory_optimizer.centerline_blocked_last_outside_grid);
  EXPECT_EQ(
      parsed_value.stats.trajectory_optimizer.centerline_blocked_span_diagnostic_count,
      2U);
  const TrajectoryOptimizerBlockedSpanDiagnostic& span0 =
      parsed_value.stats.trajectory_optimizer.centerline_blocked_span_diagnostics[0];
  EXPECT_EQ(span0.begin_segment_index, 12U);
  EXPECT_EQ(span0.end_segment_index, 13U);
  EXPECT_DOUBLE_EQ(span0.begin_s_m, 42.5);
  EXPECT_DOUBLE_EQ(span0.end_s_m, 45.25);
  EXPECT_DOUBLE_EQ(span0.length_m, 2.75);
  EXPECT_DOUBLE_EQ(span0.begin_x_m, 13.25);
  EXPECT_DOUBLE_EQ(span0.begin_y_m, -8.75);
  EXPECT_DOUBLE_EQ(span0.end_x_m, 14.5);
  EXPECT_DOUBLE_EQ(span0.end_y_m, -9.0);
  EXPECT_EQ(span0.prohibited_cells, 6U);
  EXPECT_EQ(span0.outside_grid_segments, 0U);
  const TrajectoryOptimizerBlockedSpanDiagnostic& span1 =
      parsed_value.stats.trajectory_optimizer.centerline_blocked_span_diagnostics[1];
  EXPECT_EQ(span1.begin_segment_index, 14U);
  EXPECT_EQ(span1.end_segment_index, 14U);
  EXPECT_DOUBLE_EQ(span1.begin_s_m, 48.0);
  EXPECT_DOUBLE_EQ(span1.end_s_m, 48.75);
  EXPECT_DOUBLE_EQ(span1.length_m, 0.75);
  EXPECT_DOUBLE_EQ(span1.begin_x_m, 16.0);
  EXPECT_DOUBLE_EQ(span1.begin_y_m, -9.0);
  EXPECT_DOUBLE_EQ(span1.end_x_m, 16.5);
  EXPECT_DOUBLE_EQ(span1.end_y_m, -9.25);
  EXPECT_EQ(span1.prohibited_cells, 4U);
  EXPECT_EQ(span1.outside_grid_segments, 1U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.dp_states, 144U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.dp_transitions, 512U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.dp_segment_cache_hits, 10U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.dp_segment_cache_misses, 502U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.candidate_segment_cache_hits, 3U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.candidate_segment_cache_misses,
            244U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.dp_coarse_states, 44U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.dp_coarse_transitions, 112U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.dp_fine_states, 100U);
  EXPECT_EQ(parsed_value.stats.trajectory_optimizer.dp_fine_transitions, 400U);
  EXPECT_TRUE(parsed_value.stats.trajectory_optimizer.dp_coarse_to_fine_used);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.window_detection_duration_ms,
                   0.75);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.window_eval_duration_ms,
                   6.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.dp_duration_ms, 4.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer.full_final_score_duration_ms,
                   2.75);
  EXPECT_TRUE(parsed_value.stats.trajectory_optimizer.async_refined);
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
  EXPECT_DOUBLE_EQ(parsed_value.stats.trajectory_optimizer_duration_ms, 99.9);
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

TEST(TrajectoryDiagnosticsIo, PlannerDiagnosticsJsonEscapesKnownPassageIds) {
  TrajectoryPlannerStats stats = populatedStats();
  ASSERT_FALSE(stats.known_passage_validation.diagnostics.empty());
  stats.known_passage_validation.diagnostics[0].structure_id = "arch_\"01\\north";
  stats.known_passage_validation.diagnostics[0].opening_id = "main\\gate\"east";

  const std::string json = trajectoryPlannerDiagnosticsJson(42U, 100U, stats);

  EXPECT_NE(json.find("\"known_passage_diag0_structure_id\":\"arch_\\\"01\\\\north\""),
            std::string::npos);
  EXPECT_NE(json.find("\"known_passage_diag0_opening_id\":\"main\\\\gate\\\"east\""),
            std::string::npos);

  const std::optional<TrajectoryPlannerDiagnosticsEnvelope> parsed =
      parseTrajectoryPlannerDiagnosticsJson(json);

  ASSERT_TRUE(parsed.has_value());
  const TrajectoryPlannerDiagnosticsEnvelope parsed_value =
      parsed.value_or(TrajectoryPlannerDiagnosticsEnvelope{});
  ASSERT_FALSE(parsed_value.stats.known_passage_validation.diagnostics.empty());
  EXPECT_EQ(parsed_value.stats.known_passage_validation.diagnostics[0].structure_id,
            "arch_\"01\\north");
  EXPECT_EQ(parsed_value.stats.known_passage_validation.diagnostics[0].opening_id,
            "main\\gate\"east");
}

} // namespace drone_city_nav
