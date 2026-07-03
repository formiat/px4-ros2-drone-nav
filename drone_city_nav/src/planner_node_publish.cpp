#include <algorithm>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include "planner_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool segmentTraversableForGrid(const Point2 start, const Point2 end,
                                             const void* context) {
  const auto& grid = *static_cast<const OccupancyGrid2D*>(context);
  return pathSegmentIsTraversable(grid, start, end);
}

[[nodiscard]] bool segmentAllowedForGrid(const Point2 start, const Point2 end,
                                         const void* context) {
  const auto& grid = *static_cast<const OccupancyGrid2D*>(context);
  return pathSegmentIsAllowed(grid, start, end);
}

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
  TrajectoryPlannerResult trajectory_result = planBaselineTrajectory(
      TrajectoryPlannerInput{
          std::span<const Point2>{route_points.data(), route_points.size()},
          &route_grid, route_clearance_field, route_clearance_field_cache_hit,
          std::span<const CorridorSample>{}, nullptr},
      trajectory_planner_config_);
  const double duration_ms =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - started_at)
                              .count()) /
      1000.0;
  std::uint64_t baseline_path_id = 0U;
  if (!publishTrajectoryResult(runtime_grid, trajectory_result, route_points,
                               source_label, duration_ms, &baseline_path_id)) {
    return false;
  }
  startAsyncTrajectoryRefinement(route_grid, route_points, generation, baseline_path_id,
                                 trajectory_result, source_label, route_clearance_field,
                                 route_clearance_field_cache_hit);
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
      "shadow_lb(evals=%zu unavailable=%zu prunable=%zu false_prunes=%zu "
      "winner_prunes=%zu validation_full_scores=%zu "
      "validation_full_score=%.1fms prunable_full_score=%.1fms "
      "max_over=%.3f max_under=%.3f max_false_improve=%.3f) "
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
      "curvature=%zu radius=%zu speed=%zu) "
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
      "speed_profile_smoothed=%zu max_before=%.4f max_after=%.4f)]",
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
      trajectory_result.stats.trajectory_optimizer.shadow_lower_bound_evaluations,
      trajectory_result.stats.trajectory_optimizer.shadow_lower_bound_unavailable,
      trajectory_result.stats.trajectory_optimizer.shadow_lower_bound_prunable,
      trajectory_result.stats.trajectory_optimizer.shadow_lower_bound_false_prunes,
      trajectory_result.stats.trajectory_optimizer.shadow_lower_bound_winner_prunes,
      trajectory_result.stats.trajectory_optimizer
          .shadow_lower_bound_validation_full_scores,
      trajectory_result.stats.trajectory_optimizer
          .shadow_lower_bound_validation_full_score_duration_ms,
      trajectory_result.stats.trajectory_optimizer
          .shadow_lower_bound_prunable_full_score_duration_ms,
      trajectory_result.stats.trajectory_optimizer
          .shadow_lower_bound_max_overestimate_score,
      trajectory_result.stats.trajectory_optimizer
          .shadow_lower_bound_max_underestimate_score,
      trajectory_result.stats.trajectory_optimizer
          .shadow_lower_bound_max_false_prune_improvement_score,
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
      trajectory_result.stats.turn_smoothing.rejected_speed_regression,
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
      trajectory_result.stats.isolated_curvature_spikes_smoothed_speed_profile,
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
  const std::uint64_t path_id =
      publishPath(trajectory_points, PathPublicationReason::kComputedPath,
                  &trajectory_result.stats);
  if (published_path_id != nullptr) {
    *published_path_id = path_id;
  }
  return true;
}

void PlannerNode::startAsyncTrajectoryRefinement(
    const OccupancyGrid2D& grid, const std::span<const Point2> route_points,
    const std::uint64_t generation, const std::uint64_t baseline_path_id,
    const TrajectoryPlannerResult& baseline, const char* source_label,
    const ClearanceField2D* prohibited_clearance_field,
    const bool prohibited_clearance_field_cache_hit) {
  if (route_points.size() < 2U || !baseline.valid) {
    return;
  }

  const TrajectoryRefinementJob job{generation, baseline_path_id};
  const TrajectoryRefinementScheduleDecision schedule =
      refinement_scheduler_.submit(job);
  if (schedule.action == TrajectoryRefinementScheduleAction::kDisabled) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%s refined trajectory async build disabled: generation=%" PRIu64
        " baseline_path_id=%" PRIu64 " async_workers=%zu",
        source_label, generation, baseline_path_id,
        refinement_scheduler_.workerCount());
    return;
  }

  TrajectoryRefinementRequest request{
      .generation = generation,
      .baseline_path_id = baseline_path_id,
      .route_start = route_points.front(),
      .goal = route_points.back(),
      .baseline_estimated_time_s = baseline.stats.trajectory_optimizer.estimated_time_s,
      .baseline_length_m = baseline.stats.length_m,
      .route_points = std::vector<Point2>{route_points.begin(), route_points.end()},
      .source_label = source_label,
      .grid = grid,
      .prohibited_clearance_field =
          prohibited_clearance_field != nullptr
              ? std::optional<ClearanceField2D>{*prohibited_clearance_field}
              : std::nullopt,
      .prohibited_clearance_field_cache_hit = prohibited_clearance_field_cache_hit,
      .corridor_samples = baseline.corridor_samples,
      .corridor_stats = baseline.stats.corridor,
      .config = trajectory_planner_config_};
  if (schedule.action == TrajectoryRefinementScheduleAction::kQueuedLatest ||
      schedule.action == TrajectoryRefinementScheduleAction::kReplacedQueuedLatest) {
    queued_refinement_ = std::move(request);
    const std::uint64_t active_generation =
        schedule.active_job.has_value() ? schedule.active_job->generation : 0U;
    RCLCPP_INFO(get_logger(),
                "%s refined trajectory async build queued: action=%s "
                "active_generation=%" PRIu64 " queued_generation=%" PRIu64
                " baseline_path_id=%" PRIu64,
                source_label,
                schedule.action ==
                        TrajectoryRefinementScheduleAction::kReplacedQueuedLatest
                    ? "replaced_latest"
                    : "queued_latest",
                active_generation, generation, baseline_path_id);
    return;
  }

  launchScheduledTrajectoryRefinement(std::move(request));
}

void PlannerNode::launchScheduledTrajectoryRefinement(
    TrajectoryRefinementRequest request) {
  std::vector<Point2> route_for_build = request.route_points;
  const bool clearance_snapshot_available =
      request.prohibited_clearance_field.has_value();
  const std::size_t corridor_sample_count = request.corridor_samples.size();
  std::future<TrajectoryPlannerResult> future = std::async(
      std::launch::async,
      [grid = std::move(request.grid), route = std::move(route_for_build),
       clearance_field = std::move(request.prohibited_clearance_field),
       clearance_cache_hit = request.prohibited_clearance_field_cache_hit,
       corridor_samples = std::move(request.corridor_samples),
       corridor_stats = request.corridor_stats, config = request.config]() mutable {
        const ClearanceField2D* clearance_field_ptr =
            clearance_field.has_value() ? &*clearance_field : nullptr;
        const CorridorStats* corridor_stats_ptr =
            corridor_samples.size() >= 2U ? &corridor_stats : nullptr;
        TrajectoryPlannerResult refined = planOptimizedTrajectoryFromSnapshots(
            std::span<const Point2>{route.data(), route.size()}, grid,
            clearance_field_ptr, clearance_cache_hit,
            std::span<const CorridorSample>{corridor_samples.data(),
                                            corridor_samples.size()},
            corridor_stats_ptr, config);
        refined.stats.quality = TrajectoryQuality::kRefined;
        refined.stats.trajectory_optimizer.async_refined = true;
        return refined;
      });

  PendingTrajectoryRefinement pending{};
  pending.generation = request.generation;
  pending.baseline_path_id = request.baseline_path_id;
  pending.route_start = request.route_start;
  pending.goal = request.goal;
  pending.baseline_estimated_time_s = request.baseline_estimated_time_s;
  pending.baseline_length_m = request.baseline_length_m;
  pending.route_points = std::move(request.route_points);
  pending.source_label = std::move(request.source_label);
  pending.future = std::move(future);
  pending_refinement_ = std::move(pending);
  RCLCPP_INFO(get_logger(),
              "%s refined trajectory async build started: generation=%" PRIu64
              " baseline_path_id=%" PRIu64
              " route_points=%zu baseline_time=%.2fs baseline_length=%.2fm "
              "clearance_snapshot=%s corridor_samples=%zu",
              pending_refinement_->source_label.c_str(),
              pending_refinement_->generation, pending_refinement_->baseline_path_id,
              pending_refinement_->route_points.size(),
              pending_refinement_->baseline_estimated_time_s,
              pending_refinement_->baseline_length_m,
              clearance_snapshot_available ? "true" : "false", corridor_sample_count);
}

void PlannerNode::launchQueuedTrajectoryRefinement(
    const std::optional<TrajectoryRefinementJob> expected_job) {
  if (!expected_job.has_value()) {
    return;
  }
  if (!queued_refinement_.has_value()) {
    RCLCPP_WARN(get_logger(),
                "Refined trajectory scheduler expected queued job but no queued "
                "request snapshot is available: generation=%" PRIu64
                " baseline_path_id=%" PRIu64,
                expected_job->generation, expected_job->baseline_path_id);
    return;
  }
  if (queued_refinement_->generation != expected_job->generation ||
      queued_refinement_->baseline_path_id != expected_job->baseline_path_id) {
    RCLCPP_WARN(
        get_logger(),
        "Refined trajectory queued request mismatch: expected_generation=%" PRIu64
        " queued_generation=%" PRIu64 " expected_baseline_path_id=%" PRIu64
        " queued_baseline_path_id=%" PRIu64,
        expected_job->generation, queued_refinement_->generation,
        expected_job->baseline_path_id, queued_refinement_->baseline_path_id);
    queued_refinement_.reset();
    return;
  }

  TrajectoryRefinementRequest request = std::move(*queued_refinement_);
  queued_refinement_.reset();
  launchScheduledTrajectoryRefinement(std::move(request));
}

bool PlannerNode::pollPendingTrajectoryRefinement(
    const OccupancyGrid2D& validation_grid) {
  if (!pending_refinement_.has_value() || !pending_refinement_->future.valid()) {
    return false;
  }
  if (pending_refinement_->future.wait_for(std::chrono::seconds{0}) !=
      std::future_status::ready) {
    return false;
  }

  PendingTrajectoryRefinement pending = std::move(*pending_refinement_);
  pending_refinement_.reset();
  const std::optional<TrajectoryRefinementJob> queued_job =
      refinement_scheduler_.completeActive(
          TrajectoryRefinementJob{pending.generation, pending.baseline_path_id});
  TrajectoryPlannerResult refined{};
  try {
    refined = pending.future.get();
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(),
                "%s refined trajectory async build failed with exception: "
                "generation=%" PRIu64 " baseline_path_id=%" PRIu64 " error='%s'",
                pending.source_label.c_str(), pending.generation,
                pending.baseline_path_id, error.what());
    launchQueuedTrajectoryRefinement(queued_job);
    return false;
  }

  if (last_published_path_id_ != pending.baseline_path_id) {
    RCLCPP_WARN(
        get_logger(),
        "%s refined trajectory rejected: reason=path_id_mismatch generation=%" PRIu64
        " baseline_path_id=%" PRIu64 " last_published_path_id=%" PRIu64,
        pending.source_label.c_str(), pending.generation, pending.baseline_path_id,
        last_published_path_id_);
    launchQueuedTrajectoryRefinement(queued_job);
    return false;
  }

  const std::vector<Point2> refined_points = trajectorySamplePoints(refined.samples);
  const TrajectoryRefinementDecision decision =
      evaluateTrajectoryRefinement(TrajectoryRefinementDecisionInput{
          .current_generation = trajectory_generation_,
          .snapshot_generation = pending.generation,
          .expected_start = pending.route_start,
          .expected_goal = pending.goal,
          .endpoint_tolerance_m = stable_path_goal_tolerance_m_,
          .max_time_regression_s = 0.5,
          .baseline_estimated_time_s = pending.baseline_estimated_time_s,
          .refined = &refined,
          .refined_points =
              std::span<const Point2>{refined_points.data(), refined_points.size()},
          .validation_grid = &validation_grid,
      });
  if (!decision.accepted) {
    RCLCPP_WARN(
        get_logger(),
        "%s refined trajectory rejected: reason=%.*s generation=%" PRIu64
        " current_generation=%" PRIu64 " baseline_path_id=%" PRIu64
        " route_points=%zu refined_points=%zu baseline_time=%.2fs refined_time=%.2fs "
        "baseline_length=%.2fm refined_length=%.2fm",
        pending.source_label.c_str(),
        static_cast<int>(refinementDecisionReasonName(decision.reason).size()),
        refinementDecisionReasonName(decision.reason).data(), pending.generation,
        trajectory_generation_, pending.baseline_path_id, pending.route_points.size(),
        refined_points.size(), pending.baseline_estimated_time_s,
        refined.stats.trajectory_optimizer.estimated_time_s, pending.baseline_length_m,
        refined.stats.length_m);
    launchQueuedTrajectoryRefinement(queued_job);
    return false;
  }

  RCLCPP_INFO(
      get_logger(),
      "%s refined trajectory accepted: generation=%" PRIu64 " baseline_path_id=%" PRIu64
      " refined_points=%zu baseline_time=%.2fs refined_time=%.2fs "
      "baseline_length=%.2fm refined_length=%.2fm",
      pending.source_label.c_str(), pending.generation, pending.baseline_path_id,
      refined_points.size(), pending.baseline_estimated_time_s,
      refined.stats.trajectory_optimizer.estimated_time_s, pending.baseline_length_m,
      refined.stats.length_m);
  const bool published = publishTrajectoryResult(
      validation_grid, refined, pending.route_points, pending.source_label.c_str(),
      refined.stats.total_duration_ms);
  launchQueuedTrajectoryRefinement(queued_job);
  return published;
}

[[nodiscard]] PublishedPathSafetySummary PlannerNode::summarizePublishedPathSafety(
    const OccupancyGrid2D& grid, const std::span<const Point2> path_points) const {
  return summarizePathSafety(path_points, segmentTraversableForGrid,
                             segmentAllowedForGrid, &grid);
}

void PlannerNode::logPublishedPathSafety(const OccupancyGrid2D& grid,
                                         const std::span<const Point2> path_points,
                                         const char* source_label) const {
  const PublishedPathSafetySummary summary =
      summarizePublishedPathSafety(grid, path_points);
  const bool unsafe_path = summary.non_traversable_segments > 0U;
  if (unsafe_path) {
    RCLCPP_WARN(
        get_logger(),
        "%s published path traversability: segments=%zu "
        "non_traversable_segments=%zu escape_segments=%zu traversable=false "
        "first_non_traversable_segment=%zu "
        "segment_start=(%.2f, %.2f) segment_end=(%.2f, %.2f)",
        source_label, summary.segments, summary.non_traversable_segments,
        summary.escape_segments, summary.first_non_traversable_segment,
        summary.first_non_traversable_start.x, summary.first_non_traversable_start.y,
        summary.first_non_traversable_end.x, summary.first_non_traversable_end.y);
    return;
  }

  if (summary.escape_segments > 0U) {
    RCLCPP_WARN(get_logger(),
                "%s published path traversability: segments=%zu "
                "non_traversable_segments=0 escape_segments=%zu traversable=true",
                source_label, summary.segments, summary.escape_segments);
    return;
  }

  RCLCPP_INFO(get_logger(),
              "%s published path traversability: segments=%zu "
              "non_traversable_segments=0 escape_segments=0 traversable=true",
              source_label, summary.segments);
}

[[nodiscard]] bool
PlannerNode::connectRouteToCurrentPose(const OccupancyGrid2D& grid,
                                       std::vector<Point2>& path_points,
                                       const char* source_label) const {
  if (path_points.empty()) {
    return false;
  }

  const Point2 current_position = current_pose_.position;
  const Point2 first_path_point = path_points.front();
  const double distance_to_first_m = distance(current_position, first_path_point);
  if (distance_to_first_m < 1.0e-6) {
    path_points.front() = current_position;
    return true;
  }

  if (distance_to_first_m < grid.resolution() && path_points.size() >= 2U &&
      pathSegmentIsTraversable(grid, current_position, path_points[1])) {
    path_points.front() = current_position;
    return true;
  }

  if (pathSegmentIsTraversable(grid, current_position, first_path_point)) {
    path_points.insert(path_points.begin(), current_position);
    return true;
  }

  RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "%s route candidate rejected because current pose cannot connect to the "
      "planned "
      "start with a traversable segment: current=(%.2f, %.2f) first=(%.2f, %.2f) "
      "distance=%.2fm",
      source_label, current_position.x, current_position.y, first_path_point.x,
      first_path_point.y, distance_to_first_m);
  return false;
}

void PlannerNode::logRejectedUnsafeRoute(const OccupancyGrid2D& grid,
                                         const std::span<const Point2> path_points,
                                         const char* source_label,
                                         const char* reason) const {
  const PublishedPathSafetySummary summary =
      summarizePublishedPathSafety(grid, path_points);
  RCLCPP_WARN(
      get_logger(),
      "%s route rejected before optimized trajectory build: reason='%s' segments=%zu "
      "non_traversable_segments=%zu escape_segments=%zu "
      "first_non_traversable_segment=%zu "
      "segment_start=(%.2f, %.2f) "
      "segment_end=(%.2f, %.2f)",
      source_label, reason, summary.segments, summary.non_traversable_segments,
      summary.escape_segments, summary.first_non_traversable_segment,
      summary.first_non_traversable_start.x, summary.first_non_traversable_start.y,
      summary.first_non_traversable_end.x, summary.first_non_traversable_end.y);
}

[[nodiscard]] double PlannerNode::currentLidarRangeMax() const {
  return std::min(static_cast<double>(last_scan_.range_max), max_lidar_range_m_);
}

[[nodiscard]] double PlannerNode::currentLidarPoseReceiveLagSeconds(
    const std::int64_t scan_receive_ns, const std::int64_t pose_receive_ns) const {
  if (scan_receive_ns > 0 && pose_receive_ns > 0 && scan_receive_ns > pose_receive_ns) {
    return static_cast<double>(scan_receive_ns - pose_receive_ns) / 1.0e9;
  }
  return 0.0;
}

[[nodiscard]] LidarProjectionPose PlannerNode::currentLidarProjectionPose() const {
  return LidarProjectionPose{current_pose_.position,
                             current_altitude_m_,
                             use_px4_heading_for_scan_ ? current_pose_.yaw_rad
                                                       : initial_heading_rad_,
                             current_attitude_.roll_rad,
                             current_attitude_.pitch_rad,
                             altitude_valid_,
                             attitude_valid_};
}

[[nodiscard]] LidarProjectionConfig PlannerNode::currentLidarProjectionConfig() const {
  return LidarProjectionConfig{max_lidar_range_m_,
                               range_hit_epsilon_m_,
                               scan_yaw_offset_rad_,
                               lidar_z_offset_m_,
                               min_projected_lidar_altitude_m_,
                               max_projected_lidar_altitude_m_,
                               compensate_lidar_attitude_,
                               lidar_mount_roll_rad_,
                               lidar_mount_pitch_rad_,
                               lidar_mount_yaw_rad_};
}

CurrentLidarOverlayStats
PlannerNode::overlayCurrentLidarHits(OccupancyGrid2D& grid,
                                     const std::int64_t now_ns) const {
  CurrentLidarOverlayStats stats{};
  stats.enabled = true;

  stats.fresh =
      timestampIsFresh(last_scan_update_ns_, now_ns, max_current_lidar_staleness_ns_);
  if (!scan_seen_ || !stats.fresh) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner current lidar overlay is waiting for a fresh scan: seen=%s "
        "fresh=%s age_s=%.2f",
        scan_seen_ ? "true" : "false", stats.fresh ? "true" : "false",
        scanAgeSeconds(now_ns));
    return stats;
  }
  if (!last_scan_projection_pose_valid_) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner current lidar overlay is waiting for a valid scan projection pose");
    return stats;
  }

  const double scan_range_max = currentLidarRangeMax();
  if (!(scan_range_max > 0.0) || last_scan_.angle_increment == 0.0F) {
    return stats;
  }

  const CurrentLidarOverlayStats overlay_stats =
      drone_city_nav::overlayCurrentLidarHits(
          grid,
          LidarScanView{std::span<const float>{last_scan_.ranges.data(),
                                               last_scan_.ranges.size()},
                        static_cast<double>(last_scan_.range_min), scan_range_max,
                        static_cast<double>(last_scan_.angle_min),
                        static_cast<double>(last_scan_.angle_increment)},
          last_scan_projection_pose_, currentLidarProjectionConfig());
  stats.used = overlay_stats.used;
  stats.processed_beams = overlay_stats.processed_beams;
  stats.hit_beams = overlay_stats.hit_beams;
  stats.altitude_rejected_beams = overlay_stats.altitude_rejected_beams;
  stats.occupied_cells = overlay_stats.occupied_cells;
  stats.outside_hits = overlay_stats.outside_hits;
  return stats;
}

[[nodiscard]] std_msgs::msg::Header PlannerNode::makePlannerHeader() const {
  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = frame_id_;
  return header;
}

void PlannerNode::publishStaticMapDebug(const OccupancyGrid2D& grid,
                                        const bool log_publication) {
  const StaticMapDebugConfig config{makePlannerHeader(),
                                    static_cast<float>(kGroundDebugZ)};
  static_map_pub_->publish(staticMapGridMessage(grid, config));
  static_map_points_pub_->publish(staticMapPointCloud(grid, config));
  if (log_publication) {
    RCLCPP_INFO(get_logger(),
                "Published static map grid: cells=%zu occupied=%zu "
                "points_topic='%s' republish_period=%.2fs",
                grid.cellCount(), static_map_occupied_cells_,
                static_map_points_pub_->get_topic_name(),
                static_map_debug_publish_period_s_);
  }
}

void PlannerNode::republishStaticMapDebug() {
  if (!static_grid_.has_value()) {
    return;
  }

  publishStaticMapDebug(*static_grid_, false);
}

void PlannerNode::publishProhibitedGrid(const OccupancyGrid2D& grid) {
  prohibited_grid_pub_->publish(
      prohibitedGridToRos(grid, ProhibitedGridToRosConfig{makePlannerHeader()}));
}

std::uint64_t PlannerNode::publishPath(const std::vector<Point2>& points,
                                       const PathPublicationReason reason,
                                       const TrajectoryPlannerStats* trajectory_stats) {
  recordPathPublication(reason, points.empty());
  const std::uint64_t path_id = next_path_id_++;
  last_published_path_id_ = path_id;

  if (points.empty()) {
    last_valid_path_points_.clear();
  }

  const PathMetrics metrics = pointPathMetrics(points);
  const std_msgs::msg::Header header = makePlannerHeader();
  const std::uint64_t path_stamp_ns = stampNanoseconds(header.stamp);
  const nav_msgs::msg::Path path = pathToRos(
      std::span<const Point2>{points.data(), points.size()}, header, kGroundDebugZ);

  std_msgs::msg::UInt64 path_id_msg;
  path_id_msg.data = path_id;
  path_id_pub_->publish(path_id_msg);
  if (trajectory_stats != nullptr && !points.empty()) {
    publishTrajectoryDiagnostics(path_id, path_stamp_ns, *trajectory_stats);
  }
  path_pub_->publish(path);
  if (!path.poses.empty()) {
    waypoint_pub_->publish(path.poses.front());
  }

  logPathUpdate(path, metrics, reason, path_id);
  logPlannerCountersThrottled();
  return path_id;
}

void PlannerNode::publishTrajectoryDiagnostics(
    const std::uint64_t path_id, const std::uint64_t path_stamp_ns,
    const TrajectoryPlannerStats& stats) const {
  if (!trajectory_diagnostics_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = trajectoryPlannerDiagnosticsJson(path_id, path_stamp_ns, stats);
  trajectory_diagnostics_pub_->publish(msg);
}

[[nodiscard]] std::filesystem::path PlannerNode::corridorSamplesDirectory() {
  return std::filesystem::path{"log"} / "corridor_samples";
}

[[nodiscard]] std::filesystem::path PlannerNode::trajectoryCandidatesDirectory() {
  return std::filesystem::path{"log"} / "trajectory_candidates";
}

bool PlannerNode::writeCorridorSamplesCsvFile(
    const std::filesystem::path& path, const TrajectoryPlannerResult& result,
    const char* source_label, const std::uint64_t candidate_path_id) const {
  return drone_city_nav::writeCorridorSamplesCsvFile(path, result, source_label,
                                                     candidate_path_id);
}

void PlannerNode::writeCorridorSamplesDump(
    const TrajectoryPlannerResult& result, const char* source_label,
    const std::uint64_t candidate_path_id) const {
  if (result.corridor_samples.empty()) {
    return;
  }

  const std::filesystem::path directory = corridorSamplesDirectory();
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    RCLCPP_WARN(get_logger(), "Failed to create corridor samples directory '%s': %s",
                directory.string().c_str(), error.message().c_str());
    return;
  }

  const std::int64_t stamp_ns = get_clock()->now().nanoseconds();
  const std::filesystem::path latest_path = directory / "latest.csv";
  const std::filesystem::path history_path =
      directory / ("path_" + std::to_string(candidate_path_id) + "_" +
                   std::to_string(stamp_ns) + ".csv");
  const bool wrote_latest =
      writeCorridorSamplesCsvFile(latest_path, result, source_label, candidate_path_id);
  const bool wrote_history = writeCorridorSamplesCsvFile(
      history_path, result, source_label, candidate_path_id);
  if (!wrote_latest || !wrote_history) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to write corridor samples dump: latest='%s' history='%s'",
        latest_path.string().c_str(), history_path.string().c_str());
  }
}

bool PlannerNode::writeTrajectoryOptimizerCandidatesCsvFile(
    const std::filesystem::path& path, const TrajectoryPlannerResult& result,
    const char* source_label, const std::uint64_t candidate_path_id) const {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }

  stream << std::setprecision(9);
  stream << "# source=" << source_label << " candidate_path_id=" << candidate_path_id
         << " status=" << trajectoryPlannerStatusName(result.stats.status)
         << " valid=" << (result.valid ? "true" : "false") << "\n";
  stream << "phase,decision,local_full_score_reason,iteration,order,center_index,"
            "center_s_m,step_m,delta_m,score,incumbent_score,length_m,noop,"
            "traversable,local_evaluated,requires_full_score,full_score_used,"
            "prohibited_cells,outside_grid_segments,changed_samples,"
            "changed_span_samples,cost_curvature,cost_curvature_change,"
            "cost_radius_shortfall,cost_heading_jump,"
            "cost_offset_change,cost_offset_second_change,cost_offset_slope,"
            "cost_collision,cost_outside_grid,point_build_ms,path_eval_ms,score_ms,"
            "full_score_ms\n";
  for (const TrajectoryOptimizerCandidateDiagnostic& diagnostic :
       result.stats.trajectory_optimizer.candidate_diagnostics) {
    stream << diagnostic.phase << "," << diagnostic.decision << ","
           << diagnostic.local_full_score_reason << "," << diagnostic.iteration << ","
           << diagnostic.order << "," << diagnostic.center_index << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.center_s_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.step_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.delta_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.score);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.incumbent_score);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.length_m);
    stream << "," << (diagnostic.noop ? "true" : "false") << ","
           << (diagnostic.traversable ? "true" : "false") << ","
           << (diagnostic.local_evaluated ? "true" : "false") << ","
           << (diagnostic.requires_full_score ? "true" : "false") << ","
           << (diagnostic.full_score_used ? "true" : "false") << ","
           << diagnostic.prohibited_cells << "," << diagnostic.outside_grid_segments
           << "," << diagnostic.changed_samples << ","
           << diagnostic.changed_span_samples << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_curvature);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_curvature_change);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_radius_shortfall);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_heading_jump);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_offset_change);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_offset_second_change);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_offset_slope);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_collision);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_outside_grid);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.point_build_duration_ms);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.path_evaluation_duration_ms);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.score_duration_ms);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.full_score_duration_ms);
    stream << "\n";
  }
  return stream.good();
}

bool PlannerNode::writeTurnSmoothingCandidatesCsvFile(
    const std::filesystem::path& path, const TrajectoryPlannerResult& result,
    const char* source_label, const std::uint64_t candidate_path_id) const {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }

  stream << std::setprecision(9);
  stream << "# source=" << source_label << " candidate_path_id=" << candidate_path_id
         << " status=" << trajectoryPlannerStatusName(result.stats.status)
         << " valid=" << (result.valid ? "true" : "false") << "\n";
  stream << "decision,reject_reason,reject_detail,pass,attempt_index,corner_index,"
            "corner_s_m,"
            "entry_distance_m,exit_distance_m,shift_scale,applied_shift_m,"
            "relaxed_angle_deg,score,min_radius_before_m,min_radius_after_m,"
            "min_speed_before_mps,min_speed_after_mps,local_time_before_s,"
            "local_time_after_s,curvature_jump_before_1pm,"
            "curvature_jump_after_1pm,heading_delta_before_rad,"
            "heading_delta_after_rad\n";
  for (const TurnSmoothingCandidateDiagnostic& diagnostic :
       result.stats.turn_smoothing.candidate_diagnostics) {
    stream << diagnostic.decision << "," << diagnostic.reject_reason << ","
           << diagnostic.reject_detail << "," << diagnostic.pass << ","
           << diagnostic.attempt_index << "," << diagnostic.corner_index << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.corner_s_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.entry_distance_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.exit_distance_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.shift_scale);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.applied_shift_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.relaxed_angle_deg);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.score);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.min_radius_before_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.min_radius_after_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.min_speed_before_mps);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.min_speed_after_mps);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.local_time_before_s);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.local_time_after_s);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.curvature_jump_before_1pm);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.curvature_jump_after_1pm);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.heading_delta_before_rad);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.heading_delta_after_rad);
    stream << "\n";
  }
  return stream.good();
}

void PlannerNode::writeTrajectoryCandidateDumps(
    const TrajectoryPlannerResult& result, const char* source_label,
    const std::uint64_t candidate_path_id) const {
  const bool has_optimizer_candidates =
      !result.stats.trajectory_optimizer.candidate_diagnostics.empty();
  const bool has_turn_smoothing_candidates =
      !result.stats.turn_smoothing.candidate_diagnostics.empty();
  if (!has_optimizer_candidates && !has_turn_smoothing_candidates) {
    return;
  }

  const std::filesystem::path directory = trajectoryCandidatesDirectory();
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    RCLCPP_WARN(get_logger(),
                "Failed to create trajectory candidates directory '%s': %s",
                directory.string().c_str(), error.message().c_str());
    return;
  }

  const std::int64_t stamp_ns = get_clock()->now().nanoseconds();
  bool wrote_all = true;
  if (has_optimizer_candidates) {
    const std::filesystem::path latest_path =
        directory / "latest_optimizer_candidates.csv";
    const std::filesystem::path history_path =
        directory / ("path_" + std::to_string(candidate_path_id) + "_" +
                     std::to_string(stamp_ns) + "_optimizer_candidates.csv");
    wrote_all = writeTrajectoryOptimizerCandidatesCsvFile(
                    latest_path, result, source_label, candidate_path_id) &&
                wrote_all;
    wrote_all = writeTrajectoryOptimizerCandidatesCsvFile(
                    history_path, result, source_label, candidate_path_id) &&
                wrote_all;
  }
  if (has_turn_smoothing_candidates) {
    const std::filesystem::path latest_path =
        directory / "latest_turn_smoothing_candidates.csv";
    const std::filesystem::path history_path =
        directory / ("path_" + std::to_string(candidate_path_id) + "_" +
                     std::to_string(stamp_ns) + "_turn_smoothing_candidates.csv");
    wrote_all = writeTurnSmoothingCandidatesCsvFile(latest_path, result, source_label,
                                                    candidate_path_id) &&
                wrote_all;
    wrote_all = writeTurnSmoothingCandidatesCsvFile(history_path, result, source_label,
                                                    candidate_path_id) &&
                wrote_all;
  }
  if (!wrote_all) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to write one or more trajectory candidate dump files in '%s'",
        directory.string().c_str());
  }
}

void PlannerNode::publishPlanningFailureHold() {
  if (!last_valid_path_points_.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Clearing path after replanning failure; holding position instead of reusing "
        "stale waypoints");
  }
  publishPath({}, PathPublicationReason::kHoldAfterPlanningFailure);
}

} // namespace drone_city_nav
