#include <algorithm>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include "planner_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool pathTraversableForGrid(const std::span<const Point2> points,
                                          const void* context) {
  const auto& grid = *static_cast<const OccupancyGrid2D*>(context);
  return pathIsTraversable(grid, points);
}

[[nodiscard]] std::string
centerlineBlockedSpansSummary(const TrajectoryOptimizerStats& stats) {
  if (stats.centerline_blocked_span_diagnostic_count == 0U) {
    return "none";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2);
  const std::size_t count = std::min(stats.centerline_blocked_span_diagnostic_count,
                                     kMaxCenterlineBlockedSpanDiagnostics);
  for (std::size_t i = 0U; i < count; ++i) {
    if (i > 0U) {
      stream << "; ";
    }
    const TrajectoryOptimizerBlockedSpanDiagnostic& span =
        stats.centerline_blocked_span_diagnostics.at(i);
    stream << "#" << i << " seg=[" << span.begin_segment_index << ".."
           << span.end_segment_index << "] s=[" << span.begin_s_m << ".."
           << span.end_s_m << "] len=" << span.length_m << " p=(" << span.begin_x_m
           << "," << span.begin_y_m << ")->(" << span.end_x_m << "," << span.end_y_m
           << ") cells=" << span.prohibited_cells
           << " outside=" << span.outside_grid_segments;
  }
  if (stats.centerline_blocked_span_count > count) {
    stream << "; ...";
  }
  return stream.str();
}

} // namespace

bool PlannerNode::publishPathFromPathCells(
    const OccupancyGrid2D& route_grid, const OccupancyGrid2D& runtime_grid,
    const std::vector<GridIndex>& raw_cells,
    const std::vector<GridIndex>& smoothed_cells, const char* source_label,
    const ClearanceField2D* route_clearance_field,
    const bool route_clearance_field_cache_hit) {
  struct CandidatePath {
    std::vector<Point2> points;
    const char* source_kind{""};
    std::size_t input_cells{0U};
    std::size_t pre_collapse_points{0U};
    std::size_t collapsed_points{0U};
    bool collapse_reverted{false};
  };

  const auto build_candidate =
      [&](const std::vector<GridIndex>& cells,
          const char* source_kind) -> std::optional<CandidatePath> {
    if (cells.empty()) {
      return std::nullopt;
    }

    std::vector<Point2> path_points = cellsToPoints(route_grid, cells);
    if (!connectRouteToCurrentPose(route_grid, path_points, source_label)) {
      return std::nullopt;
    }

    const RouteCandidateDecision route_decision =
        selectRouteCandidate(path_points, kPublishedPathCollinearityToleranceM,
                             pathTraversableForGrid, &route_grid);
    if (route_decision.status == RouteCandidateStatus::kRejectedNonTraversable) {
      logRejectedUnsafeRoute(route_grid, path_points, source_label,
                             "pre-collapse path contains a non-traversable segment");
      return std::nullopt;
    }
    if (route_decision.status == RouteCandidateStatus::kEmptyInput) {
      return std::nullopt;
    }
    CandidatePath candidate{route_decision.points,
                            source_kind,
                            cells.size(),
                            route_decision.pre_collapse_points,
                            route_decision.collapsed_points,
                            route_decision.collapse_reverted};
    if (candidate.collapse_reverted) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "%s path restored pre-collapse waypoints because collinear collapse "
          "would create a non-traversable segment: source=%s before=%zu "
          "collapsed=%zu",
          source_label, source_kind, candidate.pre_collapse_points,
          candidate.collapsed_points);
    }

    return candidate;
  };

  const std::vector<GridIndex>* selected_cells = &smoothed_cells;
  const char* selected_source_kind = "smoothed";
  bool used_raw_fallback = false;
  if (selected_cells->empty()) {
    RCLCPP_WARN(get_logger(),
                "%s path has empty smoothed cells; falling back to raw A* cells: "
                "raw_cells=%zu",
                source_label, raw_cells.size());
    selected_cells = &raw_cells;
    selected_source_kind = "raw";
    used_raw_fallback = true;
  }

  std::optional<CandidatePath> candidate =
      build_candidate(*selected_cells, selected_source_kind);
  if (!candidate.has_value() && selected_cells != &raw_cells && !raw_cells.empty()) {
    RCLCPP_WARN(get_logger(),
                "%s path postprocess rejected smoothed cells; falling back to raw "
                "A* cells: smoothed_cells=%zu raw_cells=%zu",
                source_label, smoothed_cells.size(), raw_cells.size());
    selected_cells = &raw_cells;
    selected_source_kind = "raw";
    used_raw_fallback = true;
    candidate = build_candidate(*selected_cells, selected_source_kind);
  }
  if (!candidate.has_value()) {
    RCLCPP_WARN(get_logger(),
                "%s path postprocess could not build a traversable route seed: "
                "smoothed_cells=%zu raw_cells=%zu",
                source_label, smoothed_cells.size(), raw_cells.size());
    if (keepCurrentPathAfterInvalidReplacement(source_label, "route_seed_invalid")) {
      return false;
    }
    publishPath({}, PathPublicationReason::kHoldInvalidPath);
    return false;
  }

  const std::vector<Point2> route_points = std::move(candidate->points);
  RCLCPP_INFO(get_logger(),
              "%s path postprocess: selected_source=%s raw_cells=%zu "
              "smoothed_cells=%zu selected_cells=%zu pre_collapse_points=%zu "
              "collapsed_points=%zu route_points=%zu route_segments=%zu "
              "raw_fallback=%s collapse_reverted=%s",
              source_label, candidate->source_kind, raw_cells.size(),
              smoothed_cells.size(), candidate->input_cells,
              candidate->pre_collapse_points, candidate->collapsed_points,
              route_points.size(),
              !route_points.empty() ? route_points.size() - 1U : 0U,
              used_raw_fallback ? "true" : "false",
              candidate->collapse_reverted ? "true" : "false");
  if (route_points.size() != candidate->pre_collapse_points) {
    RCLCPP_INFO(get_logger(),
                "%s path collinear waypoints collapsed: before=%zu after=%zu "
                "tolerance=%.2fm",
                source_label, candidate->pre_collapse_points, route_points.size(),
                kPublishedPathCollinearityToleranceM);
  }

  const std::uint64_t generation = ++trajectory_generation_;
  const auto started_at = std::chrono::steady_clock::now();
  const TrajectoryPlannerInput trajectory_input{
      std::span<const Point2>{route_points.data(), route_points.size()},
      &route_grid,
      route_clearance_field,
      route_clearance_field_cache_hit,
      std::span<const CorridorSample>{},
      nullptr};
  const bool async_refinement_enabled =
      trajectory_planner_config_.trajectory_optimizer.async_refinement_workers > 0U;
  TrajectoryPlannerResult trajectory_result =
      async_refinement_enabled
          ? planBaselineTrajectory(trajectory_input, trajectory_planner_config_)
          : planOptimizedTrajectory(trajectory_input, trajectory_planner_config_);
  const double duration_ms =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - started_at)
                              .count()) /
      1000.0;
  std::uint64_t published_path_id = 0U;
  if (!publishTrajectoryResult(runtime_grid, trajectory_result, route_points,
                               source_label, duration_ms, &published_path_id)) {
    return false;
  }
  if (async_refinement_enabled) {
    startAsyncTrajectoryRefinement(
        route_grid, route_points, generation, published_path_id, trajectory_result,
        source_label, route_clearance_field, route_clearance_field_cache_hit);
  }
  return true;
}

bool PlannerNode::keepCurrentPathAfterInvalidReplacement(
    const char* source_label, const char* invalid_reason) const {
  if (last_valid_path_points_.size() < 2U) {
    return false;
  }

  RCLCPP_WARN(get_logger(),
              "%s replacement path rejected; keeping current valid path instead of "
              "publishing an empty hold path: reason=%s last_published_path_id=%" PRIu64
              " last_waypoints=%zu",
              source_label, invalid_reason, last_published_path_id_,
              last_valid_path_points_.size());
  return true;
}

bool PlannerNode::publishTrajectoryResult(
    const OccupancyGrid2D& validation_grid,
    const TrajectoryPlannerResult& trajectory_result,
    const std::span<const Point2> route_points, const char* source_label,
    const double duration_ms, std::uint64_t* published_path_id) {
  writeCorridorSamplesDump(trajectory_result, source_label, next_path_id_);
  writeTrajectoryCandidateDumps(trajectory_result, source_label, next_path_id_);
  if (!trajectory_result.valid) {
    RCLCPP_WARN(
        get_logger(),
        "%s trajectory build failed; rough A* route will not be published as "
        "runtime path: status=%.*s route_points=%zu duration_ms=%.1f "
        "trajectory_quality=%.*s "
        "timing[total=%.1f corridor=%.1f trajectory_optimizer=%.1f "
        "turn_smoothing=%.1f speed_profile=%.1f] "
        "corridor[samples=%zu samples_reused=%s reused_samples=%zu "
        "route_fp=%" PRIu64 " grid_cells=%" PRIu64 " grid_inflated=%" PRIu64
        " width_min=%.2f width_mean=%.2f] "
        "trajectory_optimizer[iterations=%zu evals=%zu collision_rejections=%zu]",
        source_label,
        static_cast<int>(
            trajectoryPlannerStatusName(trajectory_result.stats.status).size()),
        trajectoryPlannerStatusName(trajectory_result.stats.status).data(),
        route_points.size(), duration_ms,
        static_cast<int>(trajectoryQualityName(trajectory_result.stats.quality).size()),
        trajectoryQualityName(trajectory_result.stats.quality).data(),
        trajectory_result.stats.total_duration_ms,
        trajectory_result.stats.corridor_duration_ms,
        trajectory_result.stats.trajectory_optimizer_duration_ms,
        trajectory_result.stats.turn_smoothing_duration_ms,
        trajectory_result.stats.speed_profile_duration_ms,
        trajectory_result.stats.corridor.samples,
        trajectory_result.stats.corridor.samples_reused ? "true" : "false",
        trajectory_result.stats.corridor.reused_samples,
        trajectory_result.stats.corridor.route_fingerprint,
        trajectory_result.stats.corridor.prohibited_grid_fingerprint.cells_hash,
        trajectory_result.stats.corridor.prohibited_grid_fingerprint.inflated_hash,
        trajectory_result.stats.corridor.min_width_m,
        trajectory_result.stats.corridor.mean_width_m,
        trajectory_result.stats.trajectory_optimizer.iterations,
        trajectory_result.stats.trajectory_optimizer.candidate_evaluations,
        trajectory_result.stats.trajectory_optimizer.collision_rejections);
    if (keepCurrentPathAfterInvalidReplacement(source_label,
                                               "trajectory_build_failed")) {
      return false;
    }
    publishPath({}, PathPublicationReason::kHoldInvalidPath);
    return false;
  }

  std::vector<Point2> trajectory_points =
      trajectorySamplePoints(trajectory_result.samples);
  if (trajectory_points.size() < 2U ||
      !pathIsTraversable(validation_grid, trajectory_points)) {
    RCLCPP_WARN(
        get_logger(),
        "%s trajectory build produced a non-traversable runtime trajectory; "
        "holding instead of publishing rough A* route: route_points=%zu "
        "trajectory_points=%zu duration_ms=%.1f status=%.*s "
        "trajectory_quality=%.*s "
        "timing[total=%.1f corridor=%.1f trajectory_optimizer=%.1f "
        "turn_smoothing=%.1f speed_profile=%.1f]",
        source_label, route_points.size(), trajectory_points.size(), duration_ms,
        static_cast<int>(
            trajectoryPlannerStatusName(trajectory_result.stats.status).size()),
        trajectoryPlannerStatusName(trajectory_result.stats.status).data(),
        static_cast<int>(trajectoryQualityName(trajectory_result.stats.quality).size()),
        trajectoryQualityName(trajectory_result.stats.quality).data(),
        trajectory_result.stats.total_duration_ms,
        trajectory_result.stats.corridor_duration_ms,
        trajectory_result.stats.trajectory_optimizer_duration_ms,
        trajectory_result.stats.turn_smoothing_duration_ms,
        trajectory_result.stats.speed_profile_duration_ms);
    if (keepCurrentPathAfterInvalidReplacement(source_label,
                                               "trajectory_non_traversable")) {
      return false;
    }
    publishPath({}, PathPublicationReason::kHoldInvalidPath);
    return false;
  }

  const SpeedProfileConstraintDiagnostic* top_speed_constraint =
      trajectory_result.stats.top_speed_constraints.empty()
          ? nullptr
          : &trajectory_result.stats.top_speed_constraints.front();
  const double top_speed_constraint_s = top_speed_constraint != nullptr
                                            ? top_speed_constraint->s_m
                                            : std::numeric_limits<double>::quiet_NaN();
  const double top_speed_constraint_radius =
      top_speed_constraint != nullptr ? top_speed_constraint->radius_m
                                      : std::numeric_limits<double>::quiet_NaN();
  const double top_speed_constraint_curvature =
      top_speed_constraint != nullptr ? top_speed_constraint->curvature_1pm
                                      : std::numeric_limits<double>::quiet_NaN();
  const double top_speed_constraint_limit =
      top_speed_constraint != nullptr ? top_speed_constraint->speed_limit_mps
                                      : std::numeric_limits<double>::quiet_NaN();
  const char* top_speed_constraint_source =
      top_speed_constraint != nullptr
          ? speedConstraintTypeName(top_speed_constraint->source)
          : speedConstraintTypeName(SpeedConstraintType::kNone);
  const bool top_speed_constraint_isolated =
      top_speed_constraint != nullptr && top_speed_constraint->isolated_curvature_spike;
  const std::string centerline_blocked_spans =
      centerlineBlockedSpansSummary(trajectory_result.stats.trajectory_optimizer);

  RCLCPP_INFO(
      get_logger(),
      "%s final trajectory: route_points=%zu trajectory_points=%zu "
      "duration_ms=%.1f status=%.*s "
      "trajectory_quality=%.*s "
      "timing[total=%.1f corridor=%.1f trajectory_optimizer=%.1f "
      "turn_smoothing=%.1f speed_profile=%.1f] "
      "length=%.2f samples=%zu "
      "corridor[samples=%zu samples_reused=%s reused_samples=%zu "
      "route_fp=%" PRIu64 " grid_cells=%" PRIu64 " grid_inflated=%" PRIu64
      " width_min=%.2f width_mean=%.2f width_max=%.2f "
      "lateral_limited=%zu workers=%zu sample_build=%.1fms "
      "raycast=%.1fms lateral_limit=%.1fms clearance_build=%.1fms "
      "clearance_reused=%s clearance_cache_hit=%s config_fp=%" PRIu64 "] "
      "trajectory_optimizer[iterations=%zu evals=%zu skipped_noop=%zu "
      "eval_time=%.1fms score_time=%.1fms point_build=%.1fms "
      "sample_build=%.1fms cost=%.1fms shape=%.1fms "
      "regularization=%.1fms scratch_reused=%zu "
      "parallel=%s workers=%zu chunks=%zu parallel_batches=%zu threads=%zu "
      "worker_reuses=%zu batch_wall=%.1fms batch_wait=%.1fms "
      "buffer_prepare=%.1fms thread_launch=%.1fms thread_shutdown=%.1fms "
      "allocations_avoided=%zu local_evals=%zu local_full_fallbacks=%zu "
      "offset_changes(samples_total=%zu samples_max=%zu span_total=%zu "
      "span_max=%zu local_speed_window_total=%zu local_speed_window_max=%zu) "
      "local_required=%zu "
      "local_required_reasons(invalid=%zu boundary=%zu unsafe_base=%zu "
      "window_invalid=%zu) "
      "local_accept_full_scores=%zu local_false_positives=%zu "
      "local_timing(point=%.1fms path=%.1fms total=%.1fms) "
      "full_candidate_score=%.1fms "
      "shadow_segment_score(evals=%zu unavailable=%zu prunable=%zu "
      "false_prunes=%zu winner_mismatches=%zu window_total=%zu window_max=%zu "
      "abs_err_sum=%.6f abs_err_p95=%.6f max_over=%.6f max_under=%.6f "
      "max_false_improve=%.6f) "
      "shadow_boundary_clamped(candidates=%zu window_total=%zu window_max=%zu) "
      "cost_initial=%.3f cost_final=%.3f "
      "length_initial=%.2f length_final=%.2f length_ratio=%.3f "
      "max_offset=%.2f edge_margin_min=%.2f offset_slope_cost=%.3f "
      "time_final=%.2f "
      "speed_limit_min=%.2f "
      "speed_limit_max=%.2f curvature_limited=%zu "
      "windows=%zu active_windows=%zu active_samples=%zu "
      "window_triggers(centerline_blocked=%zu heading_change=%zu "
      "heading_span=%zu curvature=%zu width_change=%zu width_asymmetry=%zu) "
      "shadow_windows(no_width_asym=%zu/%zu no_width=%zu/%zu "
      "no_heading_span=%zu/%zu) "
      "centerline_blocked_windows(raw=%zu merged=%zu samples=%zu) "
      "centerline_blocked_detail(prohibited=%zu outside=%zu segments=%zu spans=%zu "
      "first_segment=%zu last_segment=%zu s=[%.2f,%.2f] span_len=%.2f "
      "first_point=(%.2f,%.2f) last_point=(%.2f,%.2f) "
      "first_outside=%s last_outside=%s) "
      "centerline_blocked_spans{%s} "
      "dp_states=%zu dp_transitions=%zu dp_cache_hits=%zu dp_cache_misses=%zu "
      "candidate_cache_hits=%zu candidate_cache_misses=%zu "
      "full_path_cache_hits=%zu full_path_cache_misses=%zu "
      "dp_coarse_states=%zu dp_coarse_transitions=%zu "
      "dp_fine_states=%zu dp_fine_transitions=%zu coarse_to_fine=%s "
      "window_detect=%.1fms "
      "window_eval=%.1fms dp=%.1fms final_score=%.1fms async_refined=%s] "
      "turn_smoothing[detected=%zu attempted=%zu candidate_attempts=%zu "
      "relaxed_attempts=%zu "
      "bezier_cache=%zu/%zu before_metrics_cache=%zu/%zu "
      "traversability_cache=%zu/%zu "
      "timing(build=%.1fms replace=%.1fms collision=%.1fms "
      "metrics=%.1fms shape=%.1fms speed=%.1fms) "
      "smoothed=%zu "
      "rejected(prohibited=%zu corridor=%zu not_improved=%zu "
      "curvature=%zu radius=%zu) "
      "heading_before=%.1fdeg heading_after=%.1fdeg "
      "curvature_jump_before=%.3f curvature_jump_after=%.3f "
      "min_inner_margin=%.2f max_outer_shift=%.2f "
      "accepted(entry=%.2fm exit=%.2fm shift_scale=%.2f "
      "relaxed_angle=%.1fdeg score=%.3f radius=%.2f->%.2f "
      "speed=%.2f->%.2f time=%.2f->%.2f)] "
      "speed_profile[min=%.2f mean=%.2f max=%.2f curvature_limited=%zu "
      "top_constraints=%zu top1(s=%.2f radius=%.2f curvature=%.4f "
      "limit=%.2f source=%s isolated=%s) "
      "isolated_spikes(candidates=%zu geometry_smoothed=%zu "
      "max_before=%.4f max_after=%.4f)]",
      source_label, route_points.size(), trajectory_points.size(), duration_ms,
      static_cast<int>(
          trajectoryPlannerStatusName(trajectory_result.stats.status).size()),
      trajectoryPlannerStatusName(trajectory_result.stats.status).data(),
      static_cast<int>(trajectoryQualityName(trajectory_result.stats.quality).size()),
      trajectoryQualityName(trajectory_result.stats.quality).data(),
      trajectory_result.stats.total_duration_ms,
      trajectory_result.stats.corridor_duration_ms,
      trajectory_result.stats.trajectory_optimizer_duration_ms,
      trajectory_result.stats.turn_smoothing_duration_ms,
      trajectory_result.stats.speed_profile_duration_ms,
      trajectory_result.stats.length_m, trajectory_result.stats.samples,
      trajectory_result.stats.corridor.samples,
      trajectory_result.stats.corridor.samples_reused ? "true" : "false",
      trajectory_result.stats.corridor.reused_samples,
      trajectory_result.stats.corridor.route_fingerprint,
      trajectory_result.stats.corridor.prohibited_grid_fingerprint.cells_hash,
      trajectory_result.stats.corridor.prohibited_grid_fingerprint.inflated_hash,
      trajectory_result.stats.corridor.min_width_m,
      trajectory_result.stats.corridor.mean_width_m,
      trajectory_result.stats.corridor.max_width_m,
      trajectory_result.stats.corridor.lateral_limited_samples,
      trajectory_result.stats.corridor.parallel_workers_used,
      trajectory_result.stats.corridor.sample_build_duration_ms,
      trajectory_result.stats.corridor.raycast_duration_ms,
      trajectory_result.stats.corridor.lateral_limit_duration_ms,
      trajectory_result.stats.corridor.clearance_field_build_duration_ms,
      trajectory_result.stats.corridor.clearance_field_reused ? "true" : "false",
      trajectory_result.stats.corridor.clearance_field_cache_hit ? "true" : "false",
      trajectory_result.stats.corridor.config_fingerprint,
      trajectory_result.stats.trajectory_optimizer.iterations,
      trajectory_result.stats.trajectory_optimizer.candidate_evaluations,
      trajectory_result.stats.trajectory_optimizer.skipped_noop_candidates,
      trajectory_result.stats.trajectory_optimizer
          .candidate_path_evaluation_duration_ms,
      trajectory_result.stats.trajectory_optimizer.candidate_score_duration_ms,
      trajectory_result.stats.trajectory_optimizer.candidate_point_build_duration_ms,
      trajectory_result.stats.trajectory_optimizer.candidate_sample_build_duration_ms,
      trajectory_result.stats.trajectory_optimizer.candidate_cost_breakdown_duration_ms,
      trajectory_result.stats.trajectory_optimizer
          .candidate_shape_diagnostics_duration_ms,
      trajectory_result.stats.trajectory_optimizer.regularization_duration_ms,
      trajectory_result.stats.trajectory_optimizer.scratch_reused_candidates,
      trajectory_result.stats.trajectory_optimizer.parallel_candidate_evaluation_used
          ? "true"
          : "false",
      trajectory_result.stats.trajectory_optimizer.parallel_workers_used,
      trajectory_result.stats.trajectory_optimizer.candidate_chunks,
      trajectory_result.stats.trajectory_optimizer.candidate_parallel_batches,
      trajectory_result.stats.trajectory_optimizer.candidate_threads_launched,
      trajectory_result.stats.trajectory_optimizer.worker_scratch_reuses,
      trajectory_result.stats.trajectory_optimizer.candidate_batch_wall_duration_ms,
      trajectory_result.stats.trajectory_optimizer.candidate_batch_wait_duration_ms,
      trajectory_result.stats.trajectory_optimizer
          .candidate_worker_buffer_prepare_duration_ms,
      trajectory_result.stats.trajectory_optimizer.candidate_thread_launch_duration_ms,
      trajectory_result.stats.trajectory_optimizer
          .candidate_thread_join_wait_duration_ms,
      trajectory_result.stats.trajectory_optimizer
          .candidate_snapshot_allocations_avoided,
      trajectory_result.stats.trajectory_optimizer.local_candidate_evaluations,
      trajectory_result.stats.trajectory_optimizer.local_candidate_full_score_fallbacks,
      trajectory_result.stats.trajectory_optimizer
          .candidate_offset_changed_samples_total,
      trajectory_result.stats.trajectory_optimizer.candidate_offset_changed_samples_max,
      trajectory_result.stats.trajectory_optimizer
          .candidate_offset_changed_span_samples_total,
      trajectory_result.stats.trajectory_optimizer
          .candidate_offset_changed_span_samples_max,
      trajectory_result.stats.trajectory_optimizer
          .candidate_local_speed_window_samples_total,
      trajectory_result.stats.trajectory_optimizer
          .candidate_local_speed_window_samples_max,
      trajectory_result.stats.trajectory_optimizer.local_candidate_full_score_required,
      trajectory_result.stats.trajectory_optimizer
          .local_candidate_full_score_required_invalid_input,
      trajectory_result.stats.trajectory_optimizer
          .local_candidate_full_score_required_boundary,
      trajectory_result.stats.trajectory_optimizer
          .local_candidate_full_score_required_unsafe_base,
      trajectory_result.stats.trajectory_optimizer
          .local_candidate_full_score_required_window_invalid,
      trajectory_result.stats.trajectory_optimizer
          .local_candidate_acceptance_full_scores,
      trajectory_result.stats.trajectory_optimizer.local_score_false_positives,
      trajectory_result.stats.trajectory_optimizer
          .local_candidate_point_build_duration_ms,
      trajectory_result.stats.trajectory_optimizer
          .local_candidate_path_evaluation_duration_ms,
      trajectory_result.stats.trajectory_optimizer.local_candidate_score_duration_ms,
      trajectory_result.stats.trajectory_optimizer.full_candidate_score_duration_ms,
      trajectory_result.stats.trajectory_optimizer.shadow_segment_score_evaluations,
      trajectory_result.stats.trajectory_optimizer.shadow_segment_score_unavailable,
      trajectory_result.stats.trajectory_optimizer.shadow_segment_score_prunable,
      trajectory_result.stats.trajectory_optimizer.shadow_segment_score_false_prunes,
      trajectory_result.stats.trajectory_optimizer
          .shadow_segment_score_winner_mismatches,
      trajectory_result.stats.trajectory_optimizer
          .shadow_segment_score_window_samples_total,
      trajectory_result.stats.trajectory_optimizer
          .shadow_segment_score_window_samples_max,
      trajectory_result.stats.trajectory_optimizer.shadow_segment_score_abs_error_sum,
      trajectory_result.stats.trajectory_optimizer.shadow_segment_score_abs_error_p95,
      trajectory_result.stats.trajectory_optimizer
          .shadow_segment_score_max_overestimate,
      trajectory_result.stats.trajectory_optimizer
          .shadow_segment_score_max_underestimate,
      trajectory_result.stats.trajectory_optimizer
          .shadow_segment_score_max_false_prune_improvement_score,
      trajectory_result.stats.trajectory_optimizer
          .shadow_boundary_clamped_local_candidates,
      trajectory_result.stats.trajectory_optimizer
          .shadow_boundary_clamped_window_samples_total,
      trajectory_result.stats.trajectory_optimizer
          .shadow_boundary_clamped_window_samples_max,
      trajectory_result.stats.trajectory_optimizer.initial_cost,
      trajectory_result.stats.trajectory_optimizer.final_cost,
      trajectory_result.stats.trajectory_optimizer.centerline_length_m,
      trajectory_result.stats.trajectory_optimizer.final_length_m,
      trajectory_result.stats.trajectory_optimizer.final_length_ratio,
      trajectory_result.stats.trajectory_optimizer.max_abs_offset_m,
      trajectory_result.stats.trajectory_optimizer.min_edge_margin_m,
      trajectory_result.stats.trajectory_optimizer.cost_offset_slope,
      trajectory_result.stats.trajectory_optimizer.estimated_time_s,
      trajectory_result.stats.trajectory_optimizer.min_speed_limit_mps,
      trajectory_result.stats.trajectory_optimizer.max_speed_limit_mps,
      trajectory_result.stats.trajectory_optimizer.curvature_limited_samples,
      trajectory_result.stats.trajectory_optimizer.window_count,
      trajectory_result.stats.trajectory_optimizer.active_window_count,
      trajectory_result.stats.trajectory_optimizer.active_window_samples,
      trajectory_result.stats.trajectory_optimizer.active_window_centerline_blocked,
      trajectory_result.stats.trajectory_optimizer.active_window_heading_change_samples,
      trajectory_result.stats.trajectory_optimizer.active_window_heading_span_samples,
      trajectory_result.stats.trajectory_optimizer.active_window_curvature_samples,
      trajectory_result.stats.trajectory_optimizer.active_window_width_change_samples,
      trajectory_result.stats.trajectory_optimizer
          .active_window_width_asymmetry_samples,
      trajectory_result.stats.trajectory_optimizer
          .shadow_active_window_no_width_asymmetry_count,
      trajectory_result.stats.trajectory_optimizer
          .shadow_active_window_no_width_asymmetry_samples,
      trajectory_result.stats.trajectory_optimizer
          .shadow_active_window_no_width_triggers_count,
      trajectory_result.stats.trajectory_optimizer
          .shadow_active_window_no_width_triggers_samples,
      trajectory_result.stats.trajectory_optimizer
          .shadow_active_window_no_heading_span_count,
      trajectory_result.stats.trajectory_optimizer
          .shadow_active_window_no_heading_span_samples,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_windows,
      trajectory_result.stats.trajectory_optimizer
          .centerline_blocked_window_merged_count,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_window_samples,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_prohibited_cells,
      trajectory_result.stats.trajectory_optimizer
          .centerline_blocked_outside_grid_segments,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_segment_count,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_span_count,
      trajectory_result.stats.trajectory_optimizer
          .centerline_blocked_first_segment_index,
      trajectory_result.stats.trajectory_optimizer
          .centerline_blocked_last_segment_index,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_first_s_m,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_last_s_m,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_span_length_m,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_first_x_m,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_first_y_m,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_last_x_m,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_last_y_m,
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_first_outside_grid
          ? "true"
          : "false",
      trajectory_result.stats.trajectory_optimizer.centerline_blocked_last_outside_grid
          ? "true"
          : "false",
      centerline_blocked_spans.c_str(),
      trajectory_result.stats.trajectory_optimizer.dp_states,
      trajectory_result.stats.trajectory_optimizer.dp_transitions,
      trajectory_result.stats.trajectory_optimizer.dp_segment_cache_hits,
      trajectory_result.stats.trajectory_optimizer.dp_segment_cache_misses,
      trajectory_result.stats.trajectory_optimizer.candidate_segment_cache_hits,
      trajectory_result.stats.trajectory_optimizer.candidate_segment_cache_misses,
      trajectory_result.stats.trajectory_optimizer.full_path_segment_cache_hits,
      trajectory_result.stats.trajectory_optimizer.full_path_segment_cache_misses,
      trajectory_result.stats.trajectory_optimizer.dp_coarse_states,
      trajectory_result.stats.trajectory_optimizer.dp_coarse_transitions,
      trajectory_result.stats.trajectory_optimizer.dp_fine_states,
      trajectory_result.stats.trajectory_optimizer.dp_fine_transitions,
      trajectory_result.stats.trajectory_optimizer.dp_coarse_to_fine_used ? "true"
                                                                          : "false",
      trajectory_result.stats.trajectory_optimizer.window_detection_duration_ms,
      trajectory_result.stats.trajectory_optimizer.window_eval_duration_ms,
      trajectory_result.stats.trajectory_optimizer.dp_duration_ms,
      trajectory_result.stats.trajectory_optimizer.full_final_score_duration_ms,
      trajectory_result.stats.trajectory_optimizer.async_refined ? "true" : "false",
      trajectory_result.stats.turn_smoothing.detected_corners,
      trajectory_result.stats.turn_smoothing.attempted_corners,
      trajectory_result.stats.turn_smoothing.candidate_attempts,
      trajectory_result.stats.turn_smoothing.relaxed_candidate_attempts,
      trajectory_result.stats.turn_smoothing.bezier_cache_hits,
      trajectory_result.stats.turn_smoothing.bezier_cache_misses,
      trajectory_result.stats.turn_smoothing.before_metrics_cache_hits,
      trajectory_result.stats.turn_smoothing.before_metrics_cache_misses,
      trajectory_result.stats.turn_smoothing.traversability_cache_hits,
      trajectory_result.stats.turn_smoothing.traversability_cache_misses,
      trajectory_result.stats.turn_smoothing.candidate_build_duration_ms,
      trajectory_result.stats.turn_smoothing.candidate_replace_duration_ms,
      trajectory_result.stats.turn_smoothing.collision_check_duration_ms,
      trajectory_result.stats.turn_smoothing.metrics_duration_ms,
      trajectory_result.stats.turn_smoothing.shape_diagnostics_duration_ms,
      trajectory_result.stats.turn_smoothing.speed_profile_duration_ms,
      trajectory_result.stats.turn_smoothing.smoothed_corners,
      trajectory_result.stats.turn_smoothing.rejected_prohibited,
      trajectory_result.stats.turn_smoothing.rejected_corridor,
      trajectory_result.stats.turn_smoothing.rejected_not_improved,
      trajectory_result.stats.turn_smoothing.rejected_curvature_regression,
      trajectory_result.stats.turn_smoothing.rejected_radius_regression,
      radiansToDegrees(
          trajectory_result.stats.turn_smoothing.max_heading_delta_before_rad),
      radiansToDegrees(
          trajectory_result.stats.turn_smoothing.max_heading_delta_after_rad),
      trajectory_result.stats.turn_smoothing.max_curvature_jump_before_1pm,
      trajectory_result.stats.turn_smoothing.max_curvature_jump_after_1pm,
      trajectory_result.stats.turn_smoothing.min_inner_margin_m,
      trajectory_result.stats.turn_smoothing.max_applied_outer_shift_m,
      trajectory_result.stats.turn_smoothing.accepted_entry_distance_m,
      trajectory_result.stats.turn_smoothing.accepted_exit_distance_m,
      trajectory_result.stats.turn_smoothing.accepted_shift_scale,
      trajectory_result.stats.turn_smoothing.accepted_relaxed_angle_deg,
      trajectory_result.stats.turn_smoothing.accepted_score,
      trajectory_result.stats.turn_smoothing.accepted_min_radius_before_m,
      trajectory_result.stats.turn_smoothing.accepted_min_radius_after_m,
      trajectory_result.stats.turn_smoothing.accepted_min_speed_before_mps,
      trajectory_result.stats.turn_smoothing.accepted_min_speed_after_mps,
      trajectory_result.stats.turn_smoothing.accepted_local_time_before_s,
      trajectory_result.stats.turn_smoothing.accepted_local_time_after_s,
      trajectory_result.stats.speed_profile_min_mps,
      trajectory_result.stats.speed_profile_mean_mps,
      trajectory_result.stats.speed_profile_max_mps,
      trajectory_result.stats.speed_profile_curvature_limited_samples,
      trajectory_result.stats.top_speed_constraints.size(), top_speed_constraint_s,
      top_speed_constraint_radius, top_speed_constraint_curvature,
      top_speed_constraint_limit, top_speed_constraint_source,
      top_speed_constraint_isolated ? "true" : "false",
      trajectory_result.stats.isolated_curvature_spike_candidates,
      trajectory_result.stats.isolated_curvature_spikes_smoothed_geometry,
      trajectory_result.stats.isolated_curvature_spike_max_before_1pm,
      trajectory_result.stats.isolated_curvature_spike_max_after_1pm);

  for (std::size_t i = 0U;
       i < trajectory_result.stats.turn_smoothing.corner_diagnostics.size(); ++i) {
    const TurnSmoothingCornerDiagnostic& diagnostic =
        trajectory_result.stats.turn_smoothing.corner_diagnostics[i];
    RCLCPP_INFO(get_logger(),
                "%s turn_smoothing corner[%zu]: accepted=%s reason=%s corner_s=%.2f "
                "entry=%.2fm exit=%.2fm shift_scale=%.2f relaxed_angle=%.1fdeg "
                "score=%.3f radius=%.2f->%.2f speed=%.2f->%.2f "
                "time=%.2f->%.2f curvature_jump=%.3f->%.3f "
                "heading_delta=%.1fdeg->%.1fdeg",
                source_label, i, diagnostic.accepted ? "true" : "false",
                diagnostic.reject_reason.c_str(), diagnostic.corner_s_m,
                diagnostic.entry_distance_m, diagnostic.exit_distance_m,
                diagnostic.shift_scale, diagnostic.relaxed_angle_deg, diagnostic.score,
                diagnostic.min_radius_before_m, diagnostic.min_radius_after_m,
                diagnostic.min_speed_before_mps, diagnostic.min_speed_after_mps,
                diagnostic.local_time_before_s, diagnostic.local_time_after_s,
                diagnostic.curvature_jump_before_1pm,
                diagnostic.curvature_jump_after_1pm,
                radiansToDegrees(diagnostic.heading_delta_before_rad),
                radiansToDegrees(diagnostic.heading_delta_after_rad));
  }

  last_valid_path_points_ = trajectory_points;
  logPublishedPathSafety(validation_grid, trajectory_points, "final_trajectory");
  const std::uint64_t path_id = publishTrajectoryPath(
      trajectory_result.samples, PathPublicationReason::kComputedPath,
      &trajectory_result.stats);
  if (published_path_id != nullptr) {
    *published_path_id = path_id;
  }
  return true;
}

} // namespace drone_city_nav
