#include "drone_city_nav/trajectory_horizontal_handover.hpp"

#include <algorithm>
#include <exception>
#include <iomanip>
#include <limits>
#include <ranges>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include "planner_node.hpp"

namespace drone_city_nav {
namespace {

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

[[nodiscard]] std::string
verticalProfileDiagnosticsSummary(const VerticalProfileStats& stats) {
  if (!stats.applied) {
    return "not_applied";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2);
  stream << "valid=" << (stats.valid ? "true" : "false")
         << " active=" << (stats.active ? "true" : "false")
         << " matched=" << stats.passages_matched
         << " profiled=" << stats.passages_profiled
         << " infeasible=" << stats.infeasible_count;
  const std::size_t count = std::min<std::size_t>(stats.diagnostics.size(), 3U);
  for (std::size_t i = 0U; i < count; ++i) {
    const VerticalProfilePassageDiagnostic& diagnostic = stats.diagnostics.at(i);
    stream << " diag" << i << "[opening="
           << (diagnostic.opening_id.empty() ? "<none>" : diagnostic.opening_id)
           << " reason=" << (diagnostic.reason.empty() ? "<none>" : diagnostic.reason)
           << " valid=" << (diagnostic.valid ? "true" : "false") << " s=["
           << diagnostic.entry_s_m << ".." << diagnostic.exit_s_m << "]"
           << " approach=" << diagnostic.approach_start_s_m
           << " hold_start=" << diagnostic.gate_hold_start_s_m
           << " gate_z=" << diagnostic.gate_z_m
           << " transition_required=" << diagnostic.transition_required_m
           << " transition_available=" << diagnostic.transition_available_m
           << " hold_desired=" << diagnostic.desired_gate_hold_m
           << " hold_actual=" << diagnostic.actual_gate_hold_m << "]";
  }
  return stream.str();
}

[[nodiscard]] std::string
knownPassageValidationDiagnosticsSummary(const KnownPassageValidationSummary& summary) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2);
  stream << "enabled=" << (summary.enabled ? "true" : "false")
         << " valid=" << (summary.valid ? "true" : "false")
         << " intersected=" << summary.structures_intersected
         << " matches=" << summary.opening_matches
         << " violations=" << summary.violations
         << " reason=" << knownPassageValidationReasonName(summary.worst_reason);
  for (std::size_t i = 0U; i < summary.diagnostics.size(); ++i) {
    const KnownPassageValidationSpan& diagnostic = summary.diagnostics.at(i);
    stream << " diag" << i << "[structure=" << diagnostic.structure_id << " opening="
           << (diagnostic.opening_id.empty() ? "<none>" : diagnostic.opening_id)
           << " s=[" << diagnostic.entry_s_m << ".." << diagnostic.exit_s_m << "]"
           << " overlap=" << diagnostic.overlap_m
           << " clearance=" << diagnostic.clearance_m
           << " reason=" << knownPassageValidationReasonName(diagnostic.reason) << "]";
  }
  return stream.str();
}

[[nodiscard]] std::string
passageInsertionDiagnosticsSummary(const PassageInsertionStats& stats) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2);
  stream << "enabled=" << (stats.enabled ? "true" : "false")
         << " applied=" << (stats.applied ? "true" : "false")
         << " candidates=" << stats.candidates << " inserted=" << stats.inserted_count
         << " rejected(join=" << stats.rejected_join
         << " traversability=" << stats.rejected_traversability
         << " validation=" << stats.rejected_validation
         << " geometry=" << stats.rejected_geometry << ")"
         << " dropped=" << stats.diagnostics_dropped
         << " reason=" << passageInsertionRejectReasonName(stats.final_reason);
  for (std::size_t i = 0U; i < stats.diagnostics.size(); ++i) {
    const PassageInsertionDiagnostic& diagnostic = stats.diagnostics.at(i);
    stream << " diag" << i << "[structure=" << diagnostic.structure_id << " opening="
           << (diagnostic.opening_id.empty() ? "<none>" : diagnostic.opening_id)
           << " s=[" << diagnostic.entry_s_m << ".." << diagnostic.exit_s_m << "]"
           << " anchor=" << diagnostic.anchor_s_m
           << " reconnect=" << diagnostic.reconnect_s_m
           << " miss=" << diagnostic.lateral_miss_before_m << "->"
           << diagnostic.lateral_miss_after_m
           << " tangent=" << diagnostic.join_tangent_delta_before_rad << ","
           << diagnostic.join_tangent_delta_after_rad
           << " curvature=" << diagnostic.join_curvature_jump_before_1pm << ","
           << diagnostic.join_curvature_jump_after_1pm
           << " radius=" << diagnostic.min_inserted_radius_m
           << " reason=" << passageInsertionRejectReasonName(diagnostic.reason)
           << " accepted=" << (diagnostic.accepted ? "true" : "false") << "]";
  }
  return stream.str();
}

} // namespace

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
    const TrajectoryPlannerResult& trajectory_result,
    const std::span<const Point2> route_points, const char* source_label,
    const double duration_ms, TrajectoryDeliveryDiagnostics delivery,
    std::string astar_grid_name, std::string route_grid_name,
    std::uint64_t* published_path_id) {
  const NavigationStateSnapshot fresh_navigation = navigationStateSnapshot();
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (!fresh_navigation.pose_valid ||
      !timestampIsFresh(fresh_navigation.stamp_ns, now_ns, max_pose_staleness_ns_)) {
    RCLCPP_WARN(get_logger(),
                "%s trajectory candidate discarded before publication: "
                "reason=fresh_pose_unavailable generation=%" PRIu64,
                source_label, delivery.generation);
    requestPlanningCycle();
    return false;
  }
  applyNavigationStateSnapshot(fresh_navigation);
  applyPendingMemorySnapshot(now_ns);
  applyLatestLidarInputSnapshot();
  std::optional<PlanningGridBuildResult> latest_planning_result =
      buildPlanningGrid(now_ns);
  if (!latest_planning_result.has_value() ||
      !latest_planning_result->grid.has_value() ||
      !latest_planning_result->planning_grid.has_value()) {
    RCLCPP_WARN(get_logger(),
                "%s trajectory candidate discarded before publication: "
                "reason=latest_validation_grid_unavailable generation=%" PRIu64,
                source_label, delivery.generation);
    requestPlanningCycle();
    return false;
  }
  OccupancyGrid2D latest_prohibited_grid = std::move(*latest_planning_result->grid);
  OccupancyGrid2D latest_planning_grid =
      std::move(*latest_planning_result->planning_grid);
  const LocalInflationRelaxationStats latest_runtime_relaxation =
      latest_prohibited_grid.clearInflationWithinRadius(
          fresh_navigation.pose.position, local_inflation_relaxation_radius_m_);
  const LocalInflationRelaxationStats latest_planning_relaxation =
      latest_planning_grid.clearInflationWithinRadius(
          fresh_navigation.pose.position, local_inflation_relaxation_radius_m_);
  publishProhibitedGrid(latest_prohibited_grid);
  const std::vector<TrajectoryGridCandidate> latest_grid_candidates{
      TrajectoryGridCandidate{"planning_clearance", &latest_planning_grid, nullptr,
                              false},
      TrajectoryGridCandidate{"runtime_prohibited", &latest_prohibited_grid, nullptr,
                              false},
  };
  RCLCPP_INFO(get_logger(),
              "LOCAL_INFLATION_RELAXATION stage=fresh_validation center=(%.2f,%.2f) "
              "radius_m=%.2f runtime_cleared=%zu planning_cleared=%zu "
              "runtime_occupied_preserved=%zu planning_occupied_preserved=%zu",
              fresh_navigation.pose.position.x, fresh_navigation.pose.position.y,
              local_inflation_relaxation_radius_m_,
              latest_runtime_relaxation.inflated_cells_cleared,
              latest_planning_relaxation.inflated_cells_cleared,
              latest_runtime_relaxation.occupied_cells_preserved,
              latest_planning_relaxation.occupied_cells_preserved);

  std::string handover_grid_name{"not_required"};
  if (!delivery.truncation_suffix && trajectory_result.valid &&
      trajectorySamplesAreUsable(last_valid_trajectory_samples_) &&
      trajectorySamplesAreUsable(trajectory_result.samples)) {
    const std::optional<TrajectoryProjection> candidate_projection =
        projectOnTrajectorySamples(trajectory_result.samples,
                                   fresh_navigation.pose.position);
    const double candidate_cross_track_m =
        candidate_projection.has_value() ? std::sqrt(candidate_projection->distance_sq)
                                         : std::numeric_limits<double>::infinity();
    if (!(candidate_cross_track_m <= stable_path_goal_tolerance_m_)) {
      HorizontalTrajectoryHandoverResult preflight{};
      for (const TrajectoryGridCandidate& candidate : latest_grid_candidates) {
        preflight = buildHorizontalTrajectoryHandover(
            last_valid_trajectory_samples_, trajectory_result.samples,
            HorizontalTrajectoryHandoverState{
                .current_position = fresh_navigation.pose.position,
                .current_horizontal_speed_mps = fresh_navigation.speed_mps,
                .current_position_valid = true,
                .current_horizontal_speed_valid = fresh_navigation.velocity_valid,
            },
            HorizontalTrajectoryHandoverConfig{}, candidate.grid);
        if (preflight.applied ||
            std::string_view{preflight.reason} == "already_compatible") {
          handover_grid_name = std::string{candidate.name};
          break;
        }
      }
      if (!preflight.applied &&
          std::string_view{preflight.reason} != "already_compatible") {
        RCLCPP_WARN(get_logger(),
                    "%s trajectory candidate discarded before publication: "
                    "reason=handover_not_executable generation=%" PRIu64
                    " candidate_cross_track_m=%.2f handover_reason=%s "
                    "actual=(%.2f, %.2f) candidate_start=(%.2f, %.2f)",
                    source_label, delivery.generation, candidate_cross_track_m,
                    preflight.reason, fresh_navigation.pose.position.x,
                    fresh_navigation.pose.position.y,
                    trajectory_result.samples.front().point.x,
                    trajectory_result.samples.front().point.y);
        requestPlanningCycle();
        return false;
      }
      RCLCPP_INFO(get_logger(),
                  "%s trajectory candidate pre-publication handover accepted: "
                  "generation=%" PRIu64 " candidate_cross_track_m=%.2f "
                  "handover_reason=%s grid=%s old_join_s=%.2f "
                  "candidate_join_s=%.2f",
                  source_label, delivery.generation, candidate_cross_track_m,
                  preflight.reason, handover_grid_name.c_str(), preflight.old_join_s_m,
                  preflight.candidate_join_s_m);
    }
  }
  writeCorridorSamplesDump(trajectory_result, source_label, next_path_id_);
  writeTrajectoryCandidateDumps(trajectory_result, source_label, next_path_id_);
  TrajectoryPlannerStats stats = trajectory_result.stats;
  if (!trajectory_result.valid) {
    const std::string vertical_profile_summary =
        verticalProfileDiagnosticsSummary(stats.vertical_profile);
    const std::string passage_validation_summary =
        knownPassageValidationDiagnosticsSummary(stats.known_passage_validation);
    const std::string passage_insertion_summary =
        passageInsertionDiagnosticsSummary(stats.passage_insertion);
    RCLCPP_WARN(
        get_logger(),
        "%s trajectory build failed; rough A* route will not be published as "
        "runtime path: status=%.*s route_points=%zu duration_ms=%.1f "
        "trajectory_quality=%.*s "
        "timing[total=%.1f corridor=%.1f trajectory_optimizer=%.1f "
        "turn_smoothing=%.1f passage_insertion=%.1f speed_profile=%.1f] "
        "corridor[samples=%zu samples_reused=%s reused_samples=%zu "
        "route_fp=%" PRIu64 " grid_cells=%" PRIu64 " grid_inflated=%" PRIu64
        " width_min=%.2f width_mean=%.2f] "
        "trajectory_optimizer[iterations=%zu evals=%zu collision_rejections=%zu] "
        "vertical_profile[%s] "
        "known_passage_validation[%s] "
        "passage_insertion_details[%s] "
        "grid_attempts[corridor=%s(%zu) optimizer=%s(%zu) "
        "turn_smoothing=%s(%zu) trajectory_validation=%s(%zu) "
        "shape_cleanup=%s(%zu) passage_insertion=%s(%zu)]",
        source_label,
        static_cast<int>(trajectoryPlannerStatusName(stats.status).size()),
        trajectoryPlannerStatusName(stats.status).data(), route_points.size(),
        duration_ms, static_cast<int>(trajectoryQualityName(stats.quality).size()),
        trajectoryQualityName(stats.quality).data(), stats.total_duration_ms,
        stats.corridor_duration_ms, stats.trajectory_optimizer_duration_ms,
        stats.turn_smoothing_duration_ms, stats.passage_insertion_duration_ms,
        stats.speed_profile_duration_ms, stats.corridor.samples,
        stats.corridor.samples_reused ? "true" : "false", stats.corridor.reused_samples,
        stats.corridor.route_fingerprint,
        stats.corridor.prohibited_grid_fingerprint.cells_hash,
        stats.corridor.prohibited_grid_fingerprint.inflated_hash,
        stats.corridor.min_width_m, stats.corridor.mean_width_m,
        stats.trajectory_optimizer.iterations,
        stats.trajectory_optimizer.candidate_evaluations,
        stats.trajectory_optimizer.collision_rejections,
        vertical_profile_summary.c_str(), passage_validation_summary.c_str(),
        passage_insertion_summary.c_str(), stats.grid_stages.corridor.c_str(),
        stats.grid_stages.corridor_attempts, stats.grid_stages.optimizer.c_str(),
        stats.grid_stages.optimizer_attempts, stats.grid_stages.turn_smoothing.c_str(),
        stats.grid_stages.turn_smoothing_attempts,
        stats.grid_stages.trajectory_validation.c_str(),
        stats.grid_stages.trajectory_validation_attempts,
        stats.grid_stages.shape_cleanup.c_str(),
        stats.grid_stages.shape_cleanup_attempts,
        stats.grid_stages.passage_insertion.c_str(),
        stats.grid_stages.passage_insertion_attempts);
    if (keepCurrentPathAfterInvalidReplacement(source_label,
                                               "trajectory_build_failed")) {
      return false;
    }
    publishPath({}, PathPublicationReason::kHoldInvalidPath);
    return false;
  }

  std::vector<Point2> trajectory_points =
      trajectorySamplePoints(trajectory_result.samples);
  std::string final_validation_grid_name{"none"};
  const OccupancyGrid2D* final_validation_grid = nullptr;
  if (trajectory_points.size() >= 2U) {
    for (const TrajectoryGridCandidate& candidate : latest_grid_candidates) {
      if (candidate.grid != nullptr &&
          pathIsTraversable(*candidate.grid, trajectory_points)) {
        final_validation_grid_name = std::string{candidate.name};
        final_validation_grid = candidate.grid;
        break;
      }
    }
  }
  if (final_validation_grid == nullptr) {
    RCLCPP_WARN(get_logger(),
                "%s trajectory build produced a non-traversable runtime trajectory; "
                "holding instead of publishing rough A* route: route_points=%zu "
                "trajectory_points=%zu duration_ms=%.1f status=%.*s "
                "trajectory_quality=%.*s "
                "timing[total=%.1f corridor=%.1f trajectory_optimizer=%.1f "
                "turn_smoothing=%.1f passage_insertion=%.1f speed_profile=%.1f]",
                source_label, route_points.size(), trajectory_points.size(),
                duration_ms,
                static_cast<int>(trajectoryPlannerStatusName(stats.status).size()),
                trajectoryPlannerStatusName(stats.status).data(),
                static_cast<int>(trajectoryQualityName(stats.quality).size()),
                trajectoryQualityName(stats.quality).data(), stats.total_duration_ms,
                stats.corridor_duration_ms, stats.trajectory_optimizer_duration_ms,
                stats.turn_smoothing_duration_ms, stats.passage_insertion_duration_ms,
                stats.speed_profile_duration_ms);
    if (keepCurrentPathAfterInvalidReplacement(source_label,
                                               "trajectory_non_traversable")) {
      return false;
    }
    publishPath({}, PathPublicationReason::kHoldInvalidPath);
    return false;
  }

  const TrajectoryGridStageSelections& grid_stages = stats.grid_stages;
  RCLCPP_INFO(
      get_logger(),
      "GRID_ATTEMPT_SELECTION astar=%s route_connection=%s "
      "corridor=%s(%zu) optimizer=%s(%zu) turn_smoothing=%s(%zu) "
      "trajectory_validation=%s(%zu) shape_cleanup=%s(%zu) "
      "passage_insertion=%s(%zu) handover=%s final_validation=%s",
      astar_grid_name.c_str(), route_grid_name.c_str(), grid_stages.corridor.c_str(),
      grid_stages.corridor_attempts, grid_stages.optimizer.c_str(),
      grid_stages.optimizer_attempts, grid_stages.turn_smoothing.c_str(),
      grid_stages.turn_smoothing_attempts, grid_stages.trajectory_validation.c_str(),
      grid_stages.trajectory_validation_attempts, grid_stages.shape_cleanup.c_str(),
      grid_stages.shape_cleanup_attempts, grid_stages.passage_insertion.c_str(),
      grid_stages.passage_insertion_attempts, handover_grid_name.c_str(),
      final_validation_grid_name.c_str());

  stats.known_passage_validation = trajectory_result.stats.known_passage_validation;
  const KnownPassageValidationSummary& passage_validation =
      stats.known_passage_validation;
  const KnownPassageValidationSpan* first_passage_diagnostic =
      passage_validation.diagnostics.empty() ? nullptr
                                             : &passage_validation.diagnostics.front();
  const char* first_passage_structure =
      first_passage_diagnostic != nullptr
          ? first_passage_diagnostic->structure_id.c_str()
          : "<none>";
  const char* first_passage_opening =
      first_passage_diagnostic != nullptr &&
              !first_passage_diagnostic->opening_id.empty()
          ? first_passage_diagnostic->opening_id.c_str()
          : "<none>";
  const char* first_passage_reason =
      first_passage_diagnostic != nullptr
          ? knownPassageValidationReasonName(first_passage_diagnostic->reason)
          : knownPassageValidationReasonName(
                KnownPassageValidationReason::kNoStructureIntersection);
  const double first_passage_entry_s = first_passage_diagnostic != nullptr
                                           ? first_passage_diagnostic->entry_s_m
                                           : std::numeric_limits<double>::quiet_NaN();
  const double first_passage_exit_s = first_passage_diagnostic != nullptr
                                          ? first_passage_diagnostic->exit_s_m
                                          : std::numeric_limits<double>::quiet_NaN();
  const double first_passage_overlap = first_passage_diagnostic != nullptr
                                           ? first_passage_diagnostic->overlap_m
                                           : std::numeric_limits<double>::quiet_NaN();
  const double first_passage_clearance = first_passage_diagnostic != nullptr
                                             ? first_passage_diagnostic->clearance_m
                                             : std::numeric_limits<double>::quiet_NaN();
  const PassageInsertionDiagnostic* first_insertion_diagnostic =
      stats.passage_insertion.diagnostics.empty()
          ? nullptr
          : &stats.passage_insertion.diagnostics.front();
  const char* first_insertion_structure =
      first_insertion_diagnostic != nullptr
          ? first_insertion_diagnostic->structure_id.c_str()
          : "<none>";
  const char* first_insertion_opening =
      first_insertion_diagnostic != nullptr &&
              !first_insertion_diagnostic->opening_id.empty()
          ? first_insertion_diagnostic->opening_id.c_str()
          : "<none>";
  const char* first_insertion_reason =
      first_insertion_diagnostic != nullptr
          ? passageInsertionRejectReasonName(first_insertion_diagnostic->reason)
          : passageInsertionRejectReasonName(stats.passage_insertion.final_reason);
  const double first_insertion_anchor_s =
      first_insertion_diagnostic != nullptr ? first_insertion_diagnostic->anchor_s_m
                                            : std::numeric_limits<double>::quiet_NaN();
  const double first_insertion_reconnect_s =
      first_insertion_diagnostic != nullptr ? first_insertion_diagnostic->reconnect_s_m
                                            : std::numeric_limits<double>::quiet_NaN();
  const double first_insertion_miss_before =
      first_insertion_diagnostic != nullptr
          ? first_insertion_diagnostic->lateral_miss_before_m
          : std::numeric_limits<double>::quiet_NaN();
  const double first_insertion_miss_after =
      first_insertion_diagnostic != nullptr
          ? first_insertion_diagnostic->lateral_miss_after_m
          : std::numeric_limits<double>::quiet_NaN();
  const SpeedProfileConstraintDiagnostic* top_speed_constraint =
      stats.top_speed_constraints.empty() ? nullptr
                                          : &stats.top_speed_constraints.front();
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
      centerlineBlockedSpansSummary(stats.trajectory_optimizer);

  RCLCPP_INFO(
      get_logger(),
      "%s final trajectory: route_points=%zu trajectory_points=%zu "
      "duration_ms=%.1f status=%.*s "
      "trajectory_quality=%.*s "
      "timing[total=%.1f corridor=%.1f trajectory_optimizer=%.1f "
      "turn_smoothing=%.1f passage_insertion=%.1f speed_profile=%.1f] "
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
      "known_passage_validation[enabled=%s valid=%s checked=%zu "
      "intersected=%zu matches=%zu violations=%zu reason=%s "
      "first(structure=%s opening=%s s=[%.2f,%.2f] overlap=%.2f "
      "clearance=%.2f reason=%s)] "
      "vertical_profile[enabled=%s active=%s applied=%s valid=%s "
      "matched=%zu profiled=%zu infeasible=%zu z=[%.2f,%.2f] "
      "max_slope=%.4f min_cap=%.2f] "
      "passage_insertion[enabled=%s applied=%s candidates=%zu inserted=%zu "
      "rejected(join=%zu traversability=%zu validation=%zu geometry=%zu) "
      "reason=%s first(structure=%s opening=%s s=[%.2f,%.2f] "
      "miss=%.2f->%.2f reason=%s accepted=%s)] "
      "speed_profile[min=%.2f mean=%.2f max=%.2f curvature_limited=%zu "
      "top_constraints=%zu top1(s=%.2f radius=%.2f curvature=%.4f "
      "limit=%.2f source=%s isolated=%s) "
      "isolated_spikes(candidates=%zu geometry_smoothed=%zu "
      "max_before=%.4f max_after=%.4f)]",
      source_label, route_points.size(), trajectory_points.size(), duration_ms,
      static_cast<int>(trajectoryPlannerStatusName(stats.status).size()),
      trajectoryPlannerStatusName(stats.status).data(),
      static_cast<int>(trajectoryQualityName(stats.quality).size()),
      trajectoryQualityName(stats.quality).data(), stats.total_duration_ms,
      stats.corridor_duration_ms, stats.trajectory_optimizer_duration_ms,
      stats.turn_smoothing_duration_ms, stats.passage_insertion_duration_ms,
      stats.speed_profile_duration_ms, stats.length_m, stats.samples,
      stats.corridor.samples, stats.corridor.samples_reused ? "true" : "false",
      stats.corridor.reused_samples, stats.corridor.route_fingerprint,
      stats.corridor.prohibited_grid_fingerprint.cells_hash,
      stats.corridor.prohibited_grid_fingerprint.inflated_hash,
      stats.corridor.min_width_m, stats.corridor.mean_width_m,
      stats.corridor.max_width_m, stats.corridor.lateral_limited_samples,
      stats.corridor.parallel_workers_used, stats.corridor.sample_build_duration_ms,
      stats.corridor.raycast_duration_ms, stats.corridor.lateral_limit_duration_ms,
      stats.corridor.clearance_field_build_duration_ms,
      stats.corridor.clearance_field_reused ? "true" : "false",
      stats.corridor.clearance_field_cache_hit ? "true" : "false",
      stats.corridor.config_fingerprint, stats.trajectory_optimizer.iterations,
      stats.trajectory_optimizer.candidate_evaluations,
      stats.trajectory_optimizer.skipped_noop_candidates,
      stats.trajectory_optimizer.candidate_path_evaluation_duration_ms,
      stats.trajectory_optimizer.candidate_score_duration_ms,
      stats.trajectory_optimizer.candidate_point_build_duration_ms,
      stats.trajectory_optimizer.candidate_sample_build_duration_ms,
      stats.trajectory_optimizer.candidate_cost_breakdown_duration_ms,
      stats.trajectory_optimizer.candidate_shape_diagnostics_duration_ms,
      stats.trajectory_optimizer.regularization_duration_ms,
      stats.trajectory_optimizer.scratch_reused_candidates,
      stats.trajectory_optimizer.parallel_candidate_evaluation_used ? "true" : "false",
      stats.trajectory_optimizer.parallel_workers_used,
      stats.trajectory_optimizer.candidate_chunks,
      stats.trajectory_optimizer.candidate_parallel_batches,
      stats.trajectory_optimizer.candidate_threads_launched,
      stats.trajectory_optimizer.worker_scratch_reuses,
      stats.trajectory_optimizer.candidate_batch_wall_duration_ms,
      stats.trajectory_optimizer.candidate_batch_wait_duration_ms,
      stats.trajectory_optimizer.candidate_worker_buffer_prepare_duration_ms,
      stats.trajectory_optimizer.candidate_thread_launch_duration_ms,
      stats.trajectory_optimizer.candidate_thread_join_wait_duration_ms,
      stats.trajectory_optimizer.candidate_snapshot_allocations_avoided,
      stats.trajectory_optimizer.local_candidate_evaluations,
      stats.trajectory_optimizer.local_candidate_full_score_fallbacks,
      stats.trajectory_optimizer.candidate_offset_changed_samples_total,
      stats.trajectory_optimizer.candidate_offset_changed_samples_max,
      stats.trajectory_optimizer.candidate_offset_changed_span_samples_total,
      stats.trajectory_optimizer.candidate_offset_changed_span_samples_max,
      stats.trajectory_optimizer.candidate_local_speed_window_samples_total,
      stats.trajectory_optimizer.candidate_local_speed_window_samples_max,
      stats.trajectory_optimizer.local_candidate_full_score_required,
      stats.trajectory_optimizer.local_candidate_full_score_required_invalid_input,
      stats.trajectory_optimizer.local_candidate_full_score_required_boundary,
      stats.trajectory_optimizer.local_candidate_full_score_required_unsafe_base,
      stats.trajectory_optimizer.local_candidate_full_score_required_window_invalid,
      stats.trajectory_optimizer.local_candidate_acceptance_full_scores,
      stats.trajectory_optimizer.local_score_false_positives,
      stats.trajectory_optimizer.local_candidate_point_build_duration_ms,
      stats.trajectory_optimizer.local_candidate_path_evaluation_duration_ms,
      stats.trajectory_optimizer.local_candidate_score_duration_ms,
      stats.trajectory_optimizer.full_candidate_score_duration_ms,
      stats.trajectory_optimizer.shadow_segment_score_evaluations,
      stats.trajectory_optimizer.shadow_segment_score_unavailable,
      stats.trajectory_optimizer.shadow_segment_score_prunable,
      stats.trajectory_optimizer.shadow_segment_score_false_prunes,
      stats.trajectory_optimizer.shadow_segment_score_winner_mismatches,
      stats.trajectory_optimizer.shadow_segment_score_window_samples_total,
      stats.trajectory_optimizer.shadow_segment_score_window_samples_max,
      stats.trajectory_optimizer.shadow_segment_score_abs_error_sum,
      stats.trajectory_optimizer.shadow_segment_score_abs_error_p95,
      stats.trajectory_optimizer.shadow_segment_score_max_overestimate,
      stats.trajectory_optimizer.shadow_segment_score_max_underestimate,
      stats.trajectory_optimizer.shadow_segment_score_max_false_prune_improvement_score,
      stats.trajectory_optimizer.shadow_boundary_clamped_local_candidates,
      stats.trajectory_optimizer.shadow_boundary_clamped_window_samples_total,
      stats.trajectory_optimizer.shadow_boundary_clamped_window_samples_max,
      stats.trajectory_optimizer.initial_cost, stats.trajectory_optimizer.final_cost,
      stats.trajectory_optimizer.centerline_length_m,
      stats.trajectory_optimizer.final_length_m,
      stats.trajectory_optimizer.final_length_ratio,
      stats.trajectory_optimizer.max_abs_offset_m,
      stats.trajectory_optimizer.min_edge_margin_m,
      stats.trajectory_optimizer.cost_offset_slope,
      stats.trajectory_optimizer.estimated_time_s,
      stats.trajectory_optimizer.min_speed_limit_mps,
      stats.trajectory_optimizer.max_speed_limit_mps,
      stats.trajectory_optimizer.curvature_limited_samples,
      stats.trajectory_optimizer.window_count,
      stats.trajectory_optimizer.active_window_count,
      stats.trajectory_optimizer.active_window_samples,
      stats.trajectory_optimizer.active_window_centerline_blocked,
      stats.trajectory_optimizer.active_window_heading_change_samples,
      stats.trajectory_optimizer.active_window_heading_span_samples,
      stats.trajectory_optimizer.active_window_curvature_samples,
      stats.trajectory_optimizer.active_window_width_change_samples,
      stats.trajectory_optimizer.active_window_width_asymmetry_samples,
      stats.trajectory_optimizer.shadow_active_window_no_width_asymmetry_count,
      stats.trajectory_optimizer.shadow_active_window_no_width_asymmetry_samples,
      stats.trajectory_optimizer.shadow_active_window_no_width_triggers_count,
      stats.trajectory_optimizer.shadow_active_window_no_width_triggers_samples,
      stats.trajectory_optimizer.shadow_active_window_no_heading_span_count,
      stats.trajectory_optimizer.shadow_active_window_no_heading_span_samples,
      stats.trajectory_optimizer.centerline_blocked_windows,
      stats.trajectory_optimizer.centerline_blocked_window_merged_count,
      stats.trajectory_optimizer.centerline_blocked_window_samples,
      stats.trajectory_optimizer.centerline_blocked_prohibited_cells,
      stats.trajectory_optimizer.centerline_blocked_outside_grid_segments,
      stats.trajectory_optimizer.centerline_blocked_segment_count,
      stats.trajectory_optimizer.centerline_blocked_span_count,
      stats.trajectory_optimizer.centerline_blocked_first_segment_index,
      stats.trajectory_optimizer.centerline_blocked_last_segment_index,
      stats.trajectory_optimizer.centerline_blocked_first_s_m,
      stats.trajectory_optimizer.centerline_blocked_last_s_m,
      stats.trajectory_optimizer.centerline_blocked_span_length_m,
      stats.trajectory_optimizer.centerline_blocked_first_x_m,
      stats.trajectory_optimizer.centerline_blocked_first_y_m,
      stats.trajectory_optimizer.centerline_blocked_last_x_m,
      stats.trajectory_optimizer.centerline_blocked_last_y_m,
      stats.trajectory_optimizer.centerline_blocked_first_outside_grid ? "true"
                                                                       : "false",
      stats.trajectory_optimizer.centerline_blocked_last_outside_grid ? "true"
                                                                      : "false",
      centerline_blocked_spans.c_str(), stats.trajectory_optimizer.dp_states,
      stats.trajectory_optimizer.dp_transitions,
      stats.trajectory_optimizer.dp_segment_cache_hits,
      stats.trajectory_optimizer.dp_segment_cache_misses,
      stats.trajectory_optimizer.candidate_segment_cache_hits,
      stats.trajectory_optimizer.candidate_segment_cache_misses,
      stats.trajectory_optimizer.full_path_segment_cache_hits,
      stats.trajectory_optimizer.full_path_segment_cache_misses,
      stats.trajectory_optimizer.dp_coarse_states,
      stats.trajectory_optimizer.dp_coarse_transitions,
      stats.trajectory_optimizer.dp_fine_states,
      stats.trajectory_optimizer.dp_fine_transitions,
      stats.trajectory_optimizer.dp_coarse_to_fine_used ? "true" : "false",
      stats.trajectory_optimizer.window_detection_duration_ms,
      stats.trajectory_optimizer.window_eval_duration_ms,
      stats.trajectory_optimizer.dp_duration_ms,
      stats.trajectory_optimizer.full_final_score_duration_ms,
      stats.trajectory_optimizer.async_refined ? "true" : "false",
      stats.turn_smoothing.detected_corners, stats.turn_smoothing.attempted_corners,
      stats.turn_smoothing.candidate_attempts,
      stats.turn_smoothing.relaxed_candidate_attempts,
      stats.turn_smoothing.bezier_cache_hits, stats.turn_smoothing.bezier_cache_misses,
      stats.turn_smoothing.before_metrics_cache_hits,
      stats.turn_smoothing.before_metrics_cache_misses,
      stats.turn_smoothing.traversability_cache_hits,
      stats.turn_smoothing.traversability_cache_misses,
      stats.turn_smoothing.candidate_build_duration_ms,
      stats.turn_smoothing.candidate_replace_duration_ms,
      stats.turn_smoothing.collision_check_duration_ms,
      stats.turn_smoothing.metrics_duration_ms,
      stats.turn_smoothing.shape_diagnostics_duration_ms,
      stats.turn_smoothing.speed_profile_duration_ms,
      stats.turn_smoothing.smoothed_corners, stats.turn_smoothing.rejected_prohibited,
      stats.turn_smoothing.rejected_corridor,
      stats.turn_smoothing.rejected_not_improved,
      stats.turn_smoothing.rejected_curvature_regression,
      stats.turn_smoothing.rejected_radius_regression,
      radiansToDegrees(stats.turn_smoothing.max_heading_delta_before_rad),
      radiansToDegrees(stats.turn_smoothing.max_heading_delta_after_rad),
      stats.turn_smoothing.max_curvature_jump_before_1pm,
      stats.turn_smoothing.max_curvature_jump_after_1pm,
      stats.turn_smoothing.min_inner_margin_m,
      stats.turn_smoothing.max_applied_outer_shift_m,
      stats.turn_smoothing.accepted_entry_distance_m,
      stats.turn_smoothing.accepted_exit_distance_m,
      stats.turn_smoothing.accepted_shift_scale,
      stats.turn_smoothing.accepted_relaxed_angle_deg,
      stats.turn_smoothing.accepted_score,
      stats.turn_smoothing.accepted_min_radius_before_m,
      stats.turn_smoothing.accepted_min_radius_after_m,
      stats.turn_smoothing.accepted_min_speed_before_mps,
      stats.turn_smoothing.accepted_min_speed_after_mps,
      stats.turn_smoothing.accepted_local_time_before_s,
      stats.turn_smoothing.accepted_local_time_after_s,
      passage_validation.enabled ? "true" : "false",
      passage_validation.valid ? "true" : "false",
      passage_validation.structures_checked, passage_validation.structures_intersected,
      passage_validation.opening_matches, passage_validation.violations,
      knownPassageValidationReasonName(passage_validation.worst_reason),
      first_passage_structure, first_passage_opening, first_passage_entry_s,
      first_passage_exit_s, first_passage_overlap, first_passage_clearance,
      first_passage_reason, stats.vertical_profile.enabled ? "true" : "false",
      stats.vertical_profile.active ? "true" : "false",
      stats.vertical_profile.applied ? "true" : "false",
      stats.vertical_profile.valid ? "true" : "false",
      stats.vertical_profile.passages_matched, stats.vertical_profile.passages_profiled,
      stats.vertical_profile.infeasible_count, stats.vertical_profile.min_z_m,
      stats.vertical_profile.max_z_m, stats.vertical_profile.max_abs_dz_ds,
      stats.vertical_profile.min_vertical_speed_cap_mps,
      stats.passage_insertion.enabled ? "true" : "false",
      stats.passage_insertion.applied ? "true" : "false",
      stats.passage_insertion.candidates, stats.passage_insertion.inserted_count,
      stats.passage_insertion.rejected_join,
      stats.passage_insertion.rejected_traversability,
      stats.passage_insertion.rejected_validation,
      stats.passage_insertion.rejected_geometry,
      passageInsertionRejectReasonName(stats.passage_insertion.final_reason),
      first_insertion_structure, first_insertion_opening, first_insertion_anchor_s,
      first_insertion_reconnect_s, first_insertion_miss_before,
      first_insertion_miss_after, first_insertion_reason,
      first_insertion_diagnostic != nullptr && first_insertion_diagnostic->accepted
          ? "true"
          : "false",
      stats.speed_profile_min_mps, stats.speed_profile_mean_mps,
      stats.speed_profile_max_mps, stats.speed_profile_curvature_limited_samples,
      stats.top_speed_constraints.size(), top_speed_constraint_s,
      top_speed_constraint_radius, top_speed_constraint_curvature,
      top_speed_constraint_limit, top_speed_constraint_source,
      top_speed_constraint_isolated ? "true" : "false",
      stats.isolated_curvature_spike_candidates,
      stats.isolated_curvature_spikes_smoothed_geometry,
      stats.isolated_curvature_spike_max_before_1pm,
      stats.isolated_curvature_spike_max_after_1pm);

  for (std::size_t i = 0U; i < stats.turn_smoothing.corner_diagnostics.size(); ++i) {
    const TurnSmoothingCornerDiagnostic& diagnostic =
        stats.turn_smoothing.corner_diagnostics[i];
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

  logPublishedPathSafety(*final_validation_grid, trajectory_points, "final_trajectory");
  const std::uint64_t path_id = publishTrajectoryPath(
      trajectory_result.samples, PathPublicationReason::kComputedPath, &stats, delivery,
      source_label);
  if (path_id == 0U) {
    return false;
  }
  if (published_path_id != nullptr) {
    *published_path_id = path_id;
  }
  return true;
}

} // namespace drone_city_nav
