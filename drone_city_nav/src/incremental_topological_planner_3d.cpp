#include "drone_city_nav/incremental_topological_planner_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <ranges>
#include <tuple>
#include <unordered_map>
#include <utility>

#include "incremental_topological_planner_3d_detail.hpp"

namespace drone_city_nav {

IncrementalTopologicalPlan3D IncrementalTopologicalPlanner3D::planImpl(
    const IncrementalTopologyGraph3DSnapshot& graph,
    const ObservedOccupancyGrid3D* const occupancy,
    const SensorObservabilityConfig* const observability, const Point3& start,
    const Point3& mission_goal, const TopologicalExplorationMemory3D& memory,
    const std::optional<ObservationFrontier> active_frontier,
    const std::optional<std::chrono::steady_clock::time_point> deadline,
    const IncrementalTopologyConnector3D* const supplied_start_connector) const {
  IncrementalTopologicalPlan3D result;
  result.planned_on_revision = graph.revision();
  result.mission_target = mission_goal;
  if (graph.revision() == 0U || !finitePoint(start) || !finitePoint(mission_goal)) {
    return result;
  }
  const SweptFootprintConfig footprint =
      observability != nullptr ? observability->footprint : SweptFootprintConfig{};
  const auto supplied_start_connector_valid = [&]() {
    if (supplied_start_connector == nullptr || occupancy == nullptr ||
        supplied_start_connector->polyline.empty() ||
        distance3D(supplied_start_connector->polyline.front(), start) > 1.0e-6 ||
        graph.findNode(supplied_start_connector->node) == nullptr ||
        supplied_start_connector->evidence.support_segment_count == 0U ||
        supplied_start_connector->evidence.validated_through_revision >
            graph.revision()) {
      return false;
    }
    return !config_.require_known_free_space ||
           !supplied_start_connector->evidence.unknown_exposure;
  }();
  auto stage_started = std::chrono::steady_clock::now();
  const std::optional<IncrementalTopologyConnector3D> start_connector =
      supplied_start_connector_valid
          ? std::optional<IncrementalTopologyConnector3D>{*supplied_start_connector}
          : connectPointToGraph(graph, occupancy, start,
                                config_.maximum_start_anchor_distance_m, footprint,
                                config_.require_known_free_space, deadline);
  result.timing.start_connector_ms = elapsedMilliseconds(stage_started);
  if (deadline.has_value() && std::chrono::steady_clock::now() >= *deadline) {
    result.status = IncrementalTopologicalPlanStatus3D::kDeadlineExceeded;
    return result;
  }
  if (!start_connector.has_value()) {
    result.status = IncrementalTopologicalPlanStatus3D::kStartNotRepresented;
    return result;
  }
  result.start_node = start_connector->node;
  const std::optional<GridIndex3D> goal_cell =
      occupancy != nullptr ? occupancy->worldToCell(mission_goal) : std::nullopt;
  const bool goal_anchor_can_satisfy_policy =
      occupancy == nullptr || !config_.require_known_free_space ||
      (goal_cell.has_value() && occupancy->isKnownFree(*goal_cell));
  // A connector validated under the known-free policy must contain the goal
  // centre. Avoid spending the bounded search budget testing graph samples
  // when that necessary condition is already known to be false.
  stage_started = std::chrono::steady_clock::now();
  const std::optional<IncrementalTopologyConnector3D> goal_connector =
      goal_anchor_can_satisfy_policy
          ? connectPointToGraph(graph, occupancy, mission_goal,
                                config_.maximum_goal_anchor_distance_m, footprint,
                                config_.require_known_free_space, deadline)
          : std::nullopt;
  result.timing.goal_connector_ms = elapsedMilliseconds(stage_started);
  if (deadline.has_value() && std::chrono::steady_clock::now() >= *deadline) {
    result.status = IncrementalTopologicalPlanStatus3D::kDeadlineExceeded;
    return result;
  }
  if (goal_connector.has_value()) {
    result.goal_node = goal_connector->node;
  }
  std::vector<IncrementalTopologyNodeId> anchors{result.start_node};
  if (result.goal_node && *result.goal_node != result.start_node) {
    anchors.push_back(*result.goal_node);
  }
  stage_started = std::chrono::steady_clock::now();
  const RegionalTopologyGraph3D regional = buildRegionalTopologyGraph3D(graph, anchors);
  const RegionalAdjacency adjacency = buildRegionalAdjacency(regional);
  const SourceEdges source_edges = indexSourceEdges(graph);
  result.timing.regional_graph_ms = elapsedMilliseconds(stage_started);
  bool deadline_exceeded{false};
  if (result.goal_node && goal_connector.has_value()) {
    // A mission-mode shortest-path tree is only useful when the mission goal
    // is represented in this graph. Building the full tree for an unanchored
    // distant goal would consume the budget before exploration can select a
    // safe continuation toward it.
    stage_started = std::chrono::steady_clock::now();
    const SearchRecords mission_records =
        runDijkstra(regional, adjacency, result.start_node, SearchMode::kMission, graph,
                    source_edges, memory, config_, deadline, deadline_exceeded);
    result.timing.mission_search_ms = elapsedMilliseconds(stage_started);
    if (deadline_exceeded) {
      result.status = IncrementalTopologicalPlanStatus3D::kDeadlineExceeded;
      return result;
    }
    if (const std::optional<std::vector<PathStep>> path =
            reconstructPath(result.start_node, *result.goal_node, mission_records)) {
      result.status = IncrementalTopologicalPlanStatus3D::kMissionRoute;
      result.purpose = IncrementalTopologicalRoutePurpose3D::kMissionTransit;
      result.target_node = *result.goal_node;
      result.reaches_mission_goal = true;
      materializePath(result, *path, *start_connector, *goal_connector, graph,
                      source_edges, memory);
      result.selection_score = result.route_length_m;
      result.goal_progress_m = distance3D(start, mission_goal);
      return result;
    }
  }

  std::size_t reachable_mission_continuations = 0U;
  double maximum_mission_continuation_goal_progress_m = 0.0;
  stage_started = std::chrono::steady_clock::now();
  const SearchRecords exploration_records =
      runDijkstra(regional, adjacency, result.start_node, SearchMode::kExploration,
                  graph, source_edges, memory, config_, deadline, deadline_exceeded);
  result.timing.exploration_search_ms = elapsedMilliseconds(stage_started);
  const bool exploration_deadline_exceeded = deadline_exceeded;
  stage_started = std::chrono::steady_clock::now();
  const std::optional<TopologicalDeadEndConclusion3D> start_dead_end =
      deadEndConclusion(graph, result.start_node, memory);
  const std::optional<MissionContinuationCandidate> mission_continuation =
      start_dead_end.has_value()
          ? std::nullopt
          : selectMissionContinuation(regional, exploration_records, result.start_node,
                                      start, mission_goal, config_,
                                      reachable_mission_continuations,
                                      maximum_mission_continuation_goal_progress_m);
  result.timing.mission_selection_ms = elapsedMilliseconds(stage_started);
  result.reachable_mission_continuation_count = reachable_mission_continuations;
  result.maximum_reachable_mission_continuation_goal_progress_m =
      maximum_mission_continuation_goal_progress_m;
  const bool mission_incumbent_available =
      mission_continuation.has_value() && mission_continuation->node != nullptr;
  // Dijkstra records are complete predecessor chains even when the queue was
  // not exhausted. Such a route is a safe, merely suboptimal anytime result.
  if (exploration_deadline_exceeded && !mission_incumbent_available) {
    result.status = IncrementalTopologicalPlanStatus3D::kDeadlineExceeded;
    return result;
  }

  std::size_t reachable_frontiers = 0U;
  FrontierSelectionDiagnostics frontier_diagnostics;
  std::optional<FrontierCandidate> frontier;
  ObservationFrontierDiscovery fresh_discovery;
  if (!exploration_deadline_exceeded && occupancy != nullptr &&
      observability != nullptr) {
    stage_started = std::chrono::steady_clock::now();
    frontier = selectFreshFrontier(graph, exploration_records, result.start_node, start,
                                   mission_goal, source_edges, memory, config_,
                                   *occupancy, *observability, fresh_discovery,
                                   reachable_frontiers, active_frontier,
                                   frontier_diagnostics, deadline, deadline_exceeded);
    result.timing.frontier_selection_ms = elapsedMilliseconds(stage_started);
    if (deadline_exceeded && !mission_incumbent_available) {
      result.status = IncrementalTopologicalPlanStatus3D::kDeadlineExceeded;
      return result;
    }
    if (deadline_exceeded) {
      // Fresh-frontier discovery is an optional refinement. Preserve the
      // already validated mission-continuation incumbent when that refinement
      // consumes the remaining planning budget.
      frontier.reset();
    }
  }
  result.reachable_frontier_count = reachable_frontiers;
  result.goal_directed_reachable_frontier_count =
      frontier_diagnostics.goal_directed_count;
  result.maximum_goal_progress_frontier_id =
      frontier_diagnostics.maximum_goal_progress_id;
  result.maximum_reachable_frontier_goal_progress_m =
      std::isfinite(frontier_diagnostics.maximum_goal_progress_m)
          ? frontier_diagnostics.maximum_goal_progress_m
          : 0.0;
  result.fresh_frontier_candidate_count = fresh_discovery.boundary_candidates;
  result.fresh_frontier_evaluated_count = fresh_discovery.evaluated_candidates;
  result.fresh_frontier_discovered_count = fresh_discovery.frontiers.size();
  result.fresh_frontier_sample_fingerprint =
      fresh_discovery.evaluation_sample_fingerprint;
  result.fresh_frontier_status_counts = fresh_discovery.evaluation_status_counts;
  result.fresh_frontier_budget_exhausted =
      fresh_discovery.evaluation_budget_exhausted || fresh_discovery.deadline_exhausted;
  const bool select_mission_continuation =
      mission_continuation.has_value() && mission_continuation->node != nullptr &&
      (!frontier.has_value() || mission_continuation->score + 1.0e-9 < frontier->score);
  if (select_mission_continuation) {
    result.status = IncrementalTopologicalPlanStatus3D::kMissionContinuationRoute;
    result.purpose = IncrementalTopologicalRoutePurpose3D::kMissionTransit;
    result.target_node = mission_continuation->node->source_node_id;
    result.selection_score = mission_continuation->score;
    result.goal_progress_m = mission_continuation->goal_progress_m;
    const IncrementalTopologyConnector3D target_connector = makeObservedFreeConnector(
        result.target_node, {mission_continuation->node->position},
        mission_continuation->node->validated_through_revision,
        mission_continuation->node->complete_through_revision);
    materializePath(result, mission_continuation->path, *start_connector,
                    target_connector, graph, source_edges, memory);
    return result;
  }
  if (frontier.has_value() && frontier->node != nullptr) {
    result.status = IncrementalTopologicalPlanStatus3D::kFrontierRoute;
    result.purpose = IncrementalTopologicalRoutePurpose3D::kObservationFrontier;
    result.target_node = frontier->node->id;
    result.selected_frontier = frontier->frontier;
    result.selection_score = frontier->score;
    result.goal_progress_m = frontier->goal_progress_m;
    result.coverage_penalty = frontier->coverage_penalty;
    result.selected_frontier_selection_count = frontier->frontier_selection_count;
    result.selected_frontier_completion_count = frontier->frontier_completion_count;
    materializePath(result, frontier->path, *start_connector, frontier->connector,
                    graph, source_edges, memory);
    return result;
  }

  TopologicalBacktrackReason3D backtrack_reason{TopologicalBacktrackReason3D::kNone};
  const std::optional<IncrementalTopologyNodeId> backtrack_target =
      selectBacktrackTarget(graph, regional, memory, backtrack_reason);
  if (!backtrack_target || *backtrack_target == result.start_node) {
    result.status = IncrementalTopologicalPlanStatus3D::kNoRoute;
    result.backtrack_reason = backtrack_reason;
    return result;
  }
  stage_started = std::chrono::steady_clock::now();
  const SearchRecords backtrack_records =
      runDijkstra(regional, adjacency, result.start_node, SearchMode::kBacktrack, graph,
                  source_edges, memory, config_, deadline, deadline_exceeded);
  result.timing.backtrack_search_ms = elapsedMilliseconds(stage_started);
  if (deadline_exceeded) {
    result.status = IncrementalTopologicalPlanStatus3D::kDeadlineExceeded;
    return result;
  }
  const std::optional<std::vector<PathStep>> backtrack_path =
      reconstructPath(result.start_node, *backtrack_target, backtrack_records);
  const RegionalTopologyNode3D* target_node = regional.findNode(*backtrack_target);
  if (!backtrack_path || target_node == nullptr) {
    result.status = IncrementalTopologicalPlanStatus3D::kNoRoute;
    result.backtrack_reason = backtrack_reason;
    return result;
  }
  result.status = IncrementalTopologicalPlanStatus3D::kBacktrackRoute;
  result.purpose = IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack;
  result.backtrack_reason = backtrack_reason;
  result.target_node = *backtrack_target;
  result.dead_end_conclusion = start_dead_end;
  if (result.dead_end_conclusion) {
    result.backtrack_reason = TopologicalBacktrackReason3D::kConfirmedTerminal;
  }
  const IncrementalTopologyConnector3D target_connector = makeObservedFreeConnector(
      *backtrack_target, {target_node->position},
      std::min(graph.revision(), target_node->validated_through_revision),
      target_node->complete_through_revision);
  materializePath(result, *backtrack_path, *start_connector, target_connector, graph,
                  source_edges, memory);
  result.selection_score = result.route_length_m;
  return result;
}

} // namespace drone_city_nav
