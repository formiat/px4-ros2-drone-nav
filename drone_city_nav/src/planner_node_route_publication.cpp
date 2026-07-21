#include <algorithm>
#include <chrono>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

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
} // namespace

TrajectoryPlannerConfig PlannerNode::trajectoryPlannerConfigForCurrentAltitude(
    const std::optional<double> initial_altitude_m) const {
  TrajectoryPlannerConfig config = trajectory_planner_config_;
  if (initial_altitude_m.has_value() && std::isfinite(*initial_altitude_m)) {
    config.initial_altitude_m = *initial_altitude_m;
    return config;
  }
  const bool has_accepted_trajectory = last_valid_trajectory_samples_.size() >= 2U;
  const bool airborne_altitude_valid =
      altitude_valid_ && std::isfinite(current_altitude_m_) &&
      (has_accepted_trajectory || current_altitude_m_ > 1.0);
  config.initial_altitude_m =
      airborne_altitude_valid ? current_altitude_m_ : initial_altitude_m_;
  return config;
}

bool PlannerNode::publishPathFromPathCells(
    const PlanningGridBuildResult& planning_result,
    const std::span<const TrajectoryGridCandidate> grid_candidates,
    const std::size_t astar_grid_index, const std::vector<GridIndex>& raw_cells,
    const std::vector<GridIndex>& smoothed_cells, const char* source_label,
    const Point2 planning_start, const TruncationReplanState* truncation_replan) {
  if (grid_candidates.empty() || astar_grid_index >= grid_candidates.size() ||
      grid_candidates[astar_grid_index].grid == nullptr) {
    publishPath({}, PathPublicationReason::kHoldInvalidPath);
    return false;
  }
  const OccupancyGrid2D& route_geometry_grid = *grid_candidates[astar_grid_index].grid;
  TrajectoryDeliveryDiagnostics delivery =
      pending_replan_delivery_.value_or(TrajectoryDeliveryDiagnostics{});
  pending_replan_delivery_.reset();
  if (truncation_replan != nullptr) {
    delivery.blocked_path_id = truncation_replan->blocked_path_id;
    delivery.truncation_generation = truncation_replan->generation;
    delivery.temporary_prefix_fingerprint =
        truncation_replan->temporary_prefix_fingerprint;
    delivery.truncation_suffix = true;
    delivery.truncation_immediate_hold = truncation_replan->immediate_hold;
  }

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
      trajectoryPlannerConfigForCurrentAltitude(
          truncation_replan != nullptr
              ? std::optional<double>{truncation_replan->altitude_m}
              : std::nullopt);
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
  const std::vector<PassageInsertionGridAttempt>& insertion_grid_attempts =
      trajectory_result.stats.passage_insertion_grid_attempts;
  for (std::size_t index = 0U; index < insertion_grid_attempts.size(); ++index) {
    const PassageInsertionGridAttempt& attempt = insertion_grid_attempts[index];
    const char* fallback_grid =
        !attempt.accepted && index + 1U < insertion_grid_attempts.size()
            ? insertion_grid_attempts[index + 1U].grid_name.c_str()
            : "none";
    RCLCPP_INFO(
        get_logger(),
        "PASSAGE_INSERTION_GRID_ATTEMPT grid=%s valid=%s repair_required=%s "
        "repair_satisfied=%s applied=%s trajectory_invariants_hold=%s accepted=%s "
        "reason=%s fallback=%s",
        attempt.grid_name.c_str(), attempt.valid ? "true" : "false",
        attempt.repair_required ? "true" : "false",
        attempt.repair_satisfied ? "true" : "false", attempt.applied ? "true" : "false",
        attempt.trajectory_invariants_hold ? "true" : "false",
        attempt.accepted ? "true" : "false",
        passageInsertionRejectReasonName(attempt.reason), fallback_grid);
  }
  const PassageInsertionStats& insertion_stats =
      trajectory_result.stats.passage_insertion;
  for (const PassageInsertionDiagnostic& diagnostic : insertion_stats.diagnostics) {
    const PassageInsertionBlockedSegmentDiagnostic& blocked =
        diagnostic.blocked_segment;
    if (diagnostic.reason != PassageInsertionRejectReason::kNonTraversable ||
        !blocked.available) {
      continue;
    }
    const TrajectoryGridCandidate* diagnostic_grid_candidate = &route_grid_candidate;
    for (const TrajectoryGridCandidate& grid_candidate : grid_candidates) {
      if (grid_candidate.grid != nullptr &&
          grid_candidate.name == diagnostic.grid_name) {
        diagnostic_grid_candidate = &grid_candidate;
        break;
      }
    }
    const OccupancyGrid2D& diagnostic_grid = *diagnostic_grid_candidate->grid;
    PathProhibitedIntersection intersection{};
    intersection.segment_index = blocked.segment_index;
    intersection.line_cell_index = blocked.line_cell_index;
    intersection.line_cell_count = blocked.line_cell_count;
    intersection.cell = blocked.blocked_cell;
    intersection.cell_center = blocked.blocked_cell_center;
    intersection.path_distance_m = blocked.start_s_m;
    intersection.occupied = blocked.occupied;
    intersection.inflated = blocked.inflated;
    const double source_search_radius_m =
        inflation_radius_m_ +
        (diagnostic_grid_candidate->name == "planning_clearance" ? planning_clearance_m_
                                                                 : 0.0) +
        std::max(0.0, diagnostic_grid.resolution());
    const std::string source_diagnostic =
        blocked.blocked_cell_available
            ? describeProhibitedIntersectionSource(diagnostic_grid, intersection,
                                                   planning_result,
                                                   source_search_radius_m)
            : std::string{"raw_sources[unavailable reason=outside_grid]"};
    RCLCPP_WARN(
        get_logger(),
        "PASSAGE_INSERTION_BLOCKED_SEGMENT grid=%.*s structure=%s opening=%s "
        "segment=%zu s=[%.3f..%.3f] endpoints=(%.3f, %.3f)->(%.3f, %.3f) "
        "start_cell=(%d,%d) start_cell_available=%s end_cell=(%d,%d) "
        "end_cell_available=%s blocked_cell=(%d,%d) blocked_cell_available=%s "
        "line_cell=%zu/%zu center=(%.3f, %.3f) occupied=%s inflated=%s %s",
        static_cast<int>(diagnostic_grid_candidate->name.size()),
        diagnostic_grid_candidate->name.data(), diagnostic.structure_id.c_str(),
        diagnostic.opening_id.c_str(), blocked.segment_index, blocked.start_s_m,
        blocked.end_s_m, blocked.start_point.x, blocked.start_point.y,
        blocked.end_point.x, blocked.end_point.y, blocked.start_cell.x,
        blocked.start_cell.y, blocked.start_cell_available ? "true" : "false",
        blocked.end_cell.x, blocked.end_cell.y,
        blocked.end_cell_available ? "true" : "false", blocked.blocked_cell.x,
        blocked.blocked_cell.y, blocked.blocked_cell_available ? "true" : "false",
        blocked.line_cell_index, blocked.line_cell_count, blocked.blocked_cell_center.x,
        blocked.blocked_cell_center.y, blocked.occupied ? "true" : "false",
        blocked.inflated ? "true" : "false", source_diagnostic.c_str());
  }
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

} // namespace drone_city_nav
