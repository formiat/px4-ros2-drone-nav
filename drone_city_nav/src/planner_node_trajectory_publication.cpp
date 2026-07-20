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

[[nodiscard]] bool pathTraversableForGrid(const std::span<const Point2> points,
                                          const void* context) {
  const auto& grid = *static_cast<const OccupancyGrid2D*>(context);
  return pathIsTraversable(grid, points);
}

struct RouteSeedDiagnostics {
  Point2 grid_seed{};
  double connection_distance_m{0.0};
  std::size_t connection_cells{0U};
  std::size_t connection_prohibited_prefix_cells{0U};
  bool planning_start_occupied{false};
  bool planning_start_inflated{false};
  bool planning_start_prohibited{false};
  bool connection_traversable{false};
  bool connection_reentered_prohibited{false};
};

[[nodiscard]] RouteSeedDiagnostics routeSeedDiagnostics(const OccupancyGrid2D& grid,
                                                        const Point2 planning_start,
                                                        const Point2 grid_seed) {
  RouteSeedDiagnostics diagnostics{};
  diagnostics.grid_seed = grid_seed;
  diagnostics.connection_distance_m = distance(planning_start, grid_seed);
  const std::optional<GridIndex> planning_start_cell = grid.worldToCell(planning_start);
  const std::optional<GridIndex> grid_seed_cell = grid.worldToCell(grid_seed);
  if (!planning_start_cell.has_value() || !grid_seed_cell.has_value()) {
    return diagnostics;
  }

  diagnostics.planning_start_occupied = grid.isOccupied(*planning_start_cell);
  diagnostics.planning_start_inflated = grid.isInflated(*planning_start_cell);
  diagnostics.planning_start_prohibited = grid.isProhibited(*planning_start_cell);
  const std::vector<GridIndex> connection_cells =
      grid.cellsOnLine(*planning_start_cell, *grid_seed_cell);
  diagnostics.connection_cells = connection_cells.size();
  bool reached_free_cell = false;
  for (const GridIndex cell : connection_cells) {
    if (grid.isProhibited(cell)) {
      if (reached_free_cell) {
        diagnostics.connection_reentered_prohibited = true;
      } else {
        ++diagnostics.connection_prohibited_prefix_cells;
      }
    } else {
      reached_free_cell = true;
    }
  }
  diagnostics.connection_traversable =
      pathSegmentIsTraversable(grid, planning_start, grid_seed);
  return diagnostics;
}

void logTrajectoryCorridorDiagnostics(
    const rclcpp::Logger& logger, const OccupancyGrid2D& grid,
    const std::string_view grid_name, const char* source_label, const char* source_kind,
    const Point2 planning_start, const RouteSeedDiagnostics& route_seed,
    const TrajectoryPlannerResult& trajectory_result) {
  const std::string_view status_name =
      trajectoryPlannerStatusName(trajectory_result.stats.status);
  const CorridorSample* critical_sample = nullptr;
  std::size_t critical_index = 0U;
  bool critical_route_prohibited = false;
  for (std::size_t index = 0U; index < trajectory_result.corridor_samples.size();
       ++index) {
    const CorridorSample& sample = trajectory_result.corridor_samples[index];
    const std::optional<GridIndex> route_cell = grid.worldToCell(sample.route_center);
    const bool route_prohibited =
        !route_cell.has_value() || grid.isProhibited(*route_cell);
    const double width_m = sample.left_bound_m + sample.right_bound_m;
    if ((route_prohibited && !critical_route_prohibited) ||
        (critical_sample == nullptr && !route_prohibited) ||
        (!critical_route_prohibited && !route_prohibited &&
         width_m < critical_sample->left_bound_m + critical_sample->right_bound_m)) {
      critical_sample = &sample;
      critical_index = index;
      critical_route_prohibited = route_prohibited;
    }
  }

  if (critical_sample == nullptr) {
    RCLCPP_WARN(logger,
                "TRAJECTORY_CORRIDOR_DIAGNOSTIC source=%s route_source=%s grid=%.*s "
                "status=%s "
                "valid=%s planning_start=(%.2f,%.2f) start[occupied=%s inflated=%s "
                "prohibited=%s] grid_seed=(%.2f,%.2f) prefix[distance=%.2f cells=%zu "
                "prohibited_prefix=%zu reentered_prohibited=%s traversable=%s] "
                "corridor_samples=0",
                source_label, source_kind, static_cast<int>(grid_name.size()),
                grid_name.data(), status_name.data(),
                trajectory_result.valid ? "true" : "false", planning_start.x,
                planning_start.y, route_seed.planning_start_occupied ? "true" : "false",
                route_seed.planning_start_inflated ? "true" : "false",
                route_seed.planning_start_prohibited ? "true" : "false",
                route_seed.grid_seed.x, route_seed.grid_seed.y,
                route_seed.connection_distance_m, route_seed.connection_cells,
                route_seed.connection_prohibited_prefix_cells,
                route_seed.connection_reentered_prohibited ? "true" : "false",
                route_seed.connection_traversable ? "true" : "false");
    return;
  }

  const std::optional<GridIndex> route_cell =
      grid.worldToCell(critical_sample->route_center);
  const std::optional<GridIndex> center_cell =
      grid.worldToCell(critical_sample->center);
  RCLCPP_WARN(
      logger,
      "TRAJECTORY_CORRIDOR_DIAGNOSTIC source=%s route_source=%s grid=%.*s status=%s "
      "valid=%s "
      "planning_start=(%.2f,%.2f) start[occupied=%s inflated=%s prohibited=%s] "
      "grid_seed=(%.2f,%.2f) prefix[distance=%.2f cells=%zu prohibited_prefix=%zu "
      "reentered_prohibited=%s traversable=%s] critical[index=%zu s=%.2f "
      "route_prohibited=%s route=(%.2f,%.2f) route_cell=(%d,%d) "
      "center=(%.2f,%.2f) center_cell=(%d,%d) recovery=%.2f left=%.2f right=%.2f "
      "width=%.2f clearance=%.2f] corridor[route_prohibited=%zu recovered=%zu "
      "unrecoverable=%zu width_min=%.2f]",
      source_label, source_kind, static_cast<int>(grid_name.size()), grid_name.data(),
      status_name.data(), trajectory_result.valid ? "true" : "false", planning_start.x,
      planning_start.y, route_seed.planning_start_occupied ? "true" : "false",
      route_seed.planning_start_inflated ? "true" : "false",
      route_seed.planning_start_prohibited ? "true" : "false", route_seed.grid_seed.x,
      route_seed.grid_seed.y, route_seed.connection_distance_m,
      route_seed.connection_cells, route_seed.connection_prohibited_prefix_cells,
      route_seed.connection_reentered_prohibited ? "true" : "false",
      route_seed.connection_traversable ? "true" : "false", critical_index,
      critical_sample->s_m, critical_route_prohibited ? "true" : "false",
      critical_sample->route_center.x, critical_sample->route_center.y,
      route_cell.has_value() ? route_cell->x : -1,
      route_cell.has_value() ? route_cell->y : -1, critical_sample->center.x,
      critical_sample->center.y, center_cell.has_value() ? center_cell->x : -1,
      center_cell.has_value() ? center_cell->y : -1, critical_sample->center_recovery_m,
      critical_sample->left_bound_m, critical_sample->right_bound_m,
      critical_sample->left_bound_m + critical_sample->right_bound_m,
      critical_sample->clearance_m,
      trajectory_result.stats.corridor.route_prohibited_samples,
      trajectory_result.stats.corridor.center_recovered_samples,
      trajectory_result.stats.corridor.center_unrecoverable_samples,
      trajectory_result.stats.corridor.min_width_m);
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

} // namespace

TrajectoryPlannerConfig PlannerNode::trajectoryPlannerConfigForCurrentAltitude() const {
  TrajectoryPlannerConfig config = trajectory_planner_config_;
  const bool has_accepted_trajectory = last_valid_trajectory_samples_.size() >= 2U;
  const bool airborne_altitude_valid =
      altitude_valid_ && std::isfinite(current_altitude_m_) &&
      (has_accepted_trajectory || current_altitude_m_ > 1.0);
  config.initial_altitude_m =
      airborne_altitude_valid ? current_altitude_m_ : initial_altitude_m_;
  return config;
}

bool PlannerNode::publishPathFromPathCells(
    const std::span<const TrajectoryGridCandidate> grid_candidates,
    const std::size_t astar_grid_index, const std::vector<GridIndex>& raw_cells,
    const std::vector<GridIndex>& smoothed_cells, const char* source_label,
    const Point2 planning_start) {
  if (grid_candidates.empty() || astar_grid_index >= grid_candidates.size() ||
      grid_candidates[astar_grid_index].grid == nullptr) {
    publishPath({}, PathPublicationReason::kHoldInvalidPath);
    return false;
  }
  const OccupancyGrid2D& route_geometry_grid = *grid_candidates[astar_grid_index].grid;
  TrajectoryDeliveryDiagnostics delivery =
      pending_replan_delivery_.value_or(TrajectoryDeliveryDiagnostics{});
  pending_replan_delivery_.reset();

  struct CandidatePath {
    std::vector<Point2> points;
    RouteSeedDiagnostics seed_diagnostics{};
    const char* source_kind{""};
    std::size_t input_cells{0U};
    std::size_t pre_collapse_points{0U};
    std::size_t collapsed_points{0U};
    std::size_t grid_index{0U};
    bool collapse_reverted{false};
  };

  const auto build_candidate =
      [&](const std::vector<GridIndex>& cells,
          const char* source_kind) -> std::optional<CandidatePath> {
    if (cells.empty()) {
      return std::nullopt;
    }

    const std::vector<Point2> cell_points = cellsToPoints(route_geometry_grid, cells);
    for (std::size_t grid_index = 0U; grid_index < grid_candidates.size();
         ++grid_index) {
      const TrajectoryGridCandidate& grid_candidate = grid_candidates[grid_index];
      if (grid_candidate.grid == nullptr ||
          !haveSameGridGeometry(route_geometry_grid, *grid_candidate.grid)) {
        continue;
      }
      const OccupancyGrid2D& route_grid = *grid_candidate.grid;
      std::vector<Point2> path_points = cell_points;
      const RouteSeedDiagnostics seed_diagnostics =
          routeSeedDiagnostics(route_grid, planning_start, path_points.front());
      if (!connectRouteToCurrentPose(route_grid, path_points, source_label,
                                     planning_start)) {
        continue;
      }

      const RouteCandidateDecision route_decision =
          selectRouteCandidate(path_points, kPublishedPathCollinearityToleranceM,
                               pathTraversableForGrid, &route_grid);
      if (route_decision.status == RouteCandidateStatus::kRejectedNonTraversable) {
        logRejectedUnsafeRoute(route_grid, path_points, source_label,
                               "pre-collapse path contains a non-traversable segment");
        continue;
      }
      if (route_decision.status == RouteCandidateStatus::kEmptyInput) {
        continue;
      }
      CandidatePath candidate{route_decision.points,
                              seed_diagnostics,
                              source_kind,
                              cells.size(),
                              route_decision.pre_collapse_points,
                              route_decision.collapsed_points,
                              grid_index,
                              route_decision.collapse_reverted};
      if (candidate.collapse_reverted) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "%s path restored pre-collapse waypoints because collinear collapse "
            "would create a non-traversable segment: source=%s grid=%.*s "
            "before=%zu collapsed=%zu",
            source_label, source_kind, static_cast<int>(grid_candidate.name.size()),
            grid_candidate.name.data(), candidate.pre_collapse_points,
            candidate.collapsed_points);
      }
      return candidate;
    }
    return std::nullopt;
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
  const TrajectoryGridCandidate& route_grid_candidate =
      grid_candidates[candidate->grid_index];
  const OccupancyGrid2D& route_grid = *route_grid_candidate.grid;
  const std::string route_grid_name{route_grid_candidate.name};
  RCLCPP_INFO(
      get_logger(),
      "GRID_STAGE_SELECTED stage=route_connection grid=%s attempt=%zu candidates=%zu",
      route_grid_name.c_str(), candidate->grid_index + 1U, grid_candidates.size());
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
  const std::int64_t build_started_stamp_ns = get_clock()->now().nanoseconds();
  delivery.generation = generation;
  delivery.trajectory_build_started_stamp_ns =
      build_started_stamp_ns > 0 ? static_cast<std::uint64_t>(build_started_stamp_ns)
                                 : 0U;
  delivery.candidate_start_position = route_points.front();
  delivery.planning_start_position = planning_start;
  delivery.planning_start_velocity = current_velocity_;
  delivery.planning_start_velocity_valid = current_velocity_valid_;
  delivery.predicted_publication_position = planning_start;
  delivery.predicted_publication_position_valid = finite2D(planning_start);
  if (delivery.replan_triggered && delivery.blocker_detected_stamp_ns > 0U &&
      delivery.trajectory_build_started_stamp_ns >=
          delivery.blocker_detected_stamp_ns) {
    delivery.blocker_to_build_start_ms =
        1.0e-6 * static_cast<double>(delivery.trajectory_build_started_stamp_ns -
                                     delivery.blocker_detected_stamp_ns);
  }
  RCLCPP_INFO(get_logger(),
              "REPLAN_DELIVERY event=trajectory_build_started generation=%" PRIu64
              " replan_triggered=%s blocker_stamp_ns=%" PRIu64
              " build_stamp_ns=%" PRIu64 " blocker_to_build_ms=%.1f "
              "candidate_start=(%.2f, %.2f) planning_start=(%.2f, %.2f) "
              "velocity=(%.2f, %.2f) velocity_valid=%s",
              delivery.generation, delivery.replan_triggered ? "true" : "false",
              delivery.blocker_detected_stamp_ns,
              delivery.trajectory_build_started_stamp_ns,
              delivery.blocker_to_build_start_ms, delivery.candidate_start_position.x,
              delivery.candidate_start_position.y, delivery.planning_start_position.x,
              delivery.planning_start_position.y, delivery.planning_start_velocity.x,
              delivery.planning_start_velocity.y,
              delivery.planning_start_velocity_valid ? "true" : "false");
  const TrajectoryPlannerConfig trajectory_config =
      trajectoryPlannerConfigForCurrentAltitude();
  const auto started_at = std::chrono::steady_clock::now();
  const TrajectoryPlannerInput trajectory_input{
      std::span<const Point2>{route_points.data(), route_points.size()},
      &route_grid,
      route_grid_candidate.clearance_field,
      route_grid_candidate.clearance_field_cache_hit,
      std::span<const CorridorSample>{},
      nullptr,
      known_passages_ ? &*known_passages_ : nullptr,
      grid_candidates};
  TrajectoryPlannerResult trajectory_result =
      planOptimizedTrajectory(trajectory_input, trajectory_config);
  if (candidate->seed_diagnostics.planning_start_prohibited ||
      !trajectory_result.valid) {
    const OccupancyGrid2D* diagnostic_grid = &route_grid;
    std::string_view diagnostic_grid_name = route_grid_candidate.name;
    bool corridor_grid_selected = false;
    for (const TrajectoryGridCandidate& grid_candidate : grid_candidates) {
      if (grid_candidate.grid != nullptr &&
          grid_candidate.name == trajectory_result.stats.grid_stages.corridor) {
        diagnostic_grid = grid_candidate.grid;
        diagnostic_grid_name = grid_candidate.name;
        corridor_grid_selected = true;
        break;
      }
    }
    if (!corridor_grid_selected) {
      for (const TrajectoryGridCandidate& grid_candidate :
           grid_candidates | std::views::reverse) {
        if (grid_candidate.grid != nullptr) {
          diagnostic_grid = grid_candidate.grid;
          diagnostic_grid_name = grid_candidate.name;
          break;
        }
      }
    }
    const RouteSeedDiagnostics diagnostic_seed = routeSeedDiagnostics(
        *diagnostic_grid, planning_start, candidate->seed_diagnostics.grid_seed);
    logTrajectoryCorridorDiagnostics(
        get_logger(), *diagnostic_grid, diagnostic_grid_name, source_label,
        candidate->source_kind, planning_start, diagnostic_seed, trajectory_result);
  }
  const double duration_ms =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - started_at)
                              .count()) /
      1000.0;
  const NavigationStateSnapshot fresh_navigation = navigationStateSnapshot();
  applyNavigationStateSnapshot(fresh_navigation);
  return publishTrajectoryResult(
      trajectory_result, route_points, source_label, duration_ms, delivery,
      std::string{grid_candidates[astar_grid_index].name}, route_grid_name);
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
  if (trajectory_result.valid &&
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
        vertical_profile_summary.c_str(), stats.grid_stages.corridor.c_str(),
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

  last_valid_path_points_ = trajectory_points;
  last_valid_trajectory_samples_ = trajectory_result.samples;
  logPublishedPathSafety(*final_validation_grid, trajectory_points, "final_trajectory");
  const std::uint64_t path_id =
      publishTrajectoryPath(trajectory_result.samples,
                            PathPublicationReason::kComputedPath, &stats, delivery);
  if (published_path_id != nullptr) {
    *published_path_id = path_id;
  }
  return true;
}

} // namespace drone_city_nav
