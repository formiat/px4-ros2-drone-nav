#include "drone_city_nav/incremental_topological_planner_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <ranges>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace drone_city_nav {
namespace {

struct ContractedAdjacencyEntry {
  IncrementalTopologyNodeId neighbor{};
  const ContractedTopologyEdge3D* edge{nullptr};
};

using ContractedAdjacency =
    std::unordered_map<IncrementalTopologyNodeId, std::vector<ContractedAdjacencyEntry>,
                       IncrementalTopologyNodeIdHash>;
using SourceEdges =
    std::unordered_map<IncrementalTopologyEdgeId, const IncrementalTopologyEdge3D*,
                       IncrementalTopologyEdgeIdHash>;

enum class SearchMode : std::uint8_t {
  kMission,
  kExploration,
  kBacktrack,
};

struct PathStep {
  const ContractedTopologyEdge3D* edge{nullptr};
  IncrementalTopologyNodeId from{};
  IncrementalTopologyNodeId to{};
};

struct SearchRecord {
  double cost{std::numeric_limits<double>::infinity()};
  IncrementalTopologyNodeId parent{};
  const ContractedTopologyEdge3D* incoming_edge{nullptr};
  bool has_parent{false};
};

using SearchRecords = std::unordered_map<IncrementalTopologyNodeId, SearchRecord,
                                         IncrementalTopologyNodeIdHash>;

struct QueueEntry {
  double cost{0.0};
  std::uint64_t sequence{0U};
  IncrementalTopologyNodeId node{};
};

struct QueueGreater {
  [[nodiscard]] bool operator()(const QueueEntry& first,
                                const QueueEntry& second) const noexcept {
    return first.cost == second.cost ? first.sequence > second.sequence
                                     : first.cost > second.cost;
  }
};

struct DirectedEdgeMetrics {
  std::vector<DirectedTopologyEdge3D> edges;
  std::size_t traversal_count{0U};
  double repeated_distance_m{0.0};
  std::uint64_t supporting_revision{0U};
  bool active_dead_end{false};
};

struct PathMetrics {
  std::size_t traversal_count{0U};
  double repeated_distance_m{0.0};
};

struct FrontierCandidate {
  const IncrementalTopologyNode3D* node{nullptr};
  std::vector<PathStep> path;
  double score{std::numeric_limits<double>::infinity()};
  double path_length_m{0.0};
  double goal_progress_m{0.0};
  double coverage_penalty{0.0};
  PathMetrics history{};
};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] ContractedAdjacency
buildContractedAdjacency(const ContractedTopologyGraph3D& graph) {
  ContractedAdjacency result;
  result.reserve(graph.nodes().size());
  for (const ContractedTopologyNode3D& node : graph.nodes()) {
    result.try_emplace(node.source_node_id);
  }
  for (const ContractedTopologyEdge3D& edge : graph.edges()) {
    result[edge.first].push_back({.neighbor = edge.second, .edge = &edge});
    result[edge.second].push_back({.neighbor = edge.first, .edge = &edge});
  }
  for (auto& [node_id, entries] : result) {
    static_cast<void>(node_id);
    std::ranges::sort(entries, [](const ContractedAdjacencyEntry& first,
                                  const ContractedAdjacencyEntry& second) {
      return std::tuple{first.neighbor.value, first.edge->id.value} <
             std::tuple{second.neighbor.value, second.edge->id.value};
    });
  }
  return result;
}

[[nodiscard]] SourceEdges
indexSourceEdges(const IncrementalTopologyGraph3DSnapshot& graph) {
  SourceEdges result;
  result.reserve(graph.edges().size());
  for (const IncrementalTopologyEdge3D& edge : graph.edges()) {
    result.emplace(edge.id, &edge);
  }
  return result;
}

[[nodiscard]] std::uint64_t
supportingRevision(const IncrementalTopologyGraph3DSnapshot& graph,
                   const IncrementalTopologyEdge3D& edge) noexcept {
  std::uint64_t revision = edge.supporting_revision;
  if (const IncrementalTopologyNode3D* first = graph.findNode(edge.first)) {
    revision = std::max(revision, first->geometry_revision);
  }
  if (const IncrementalTopologyNode3D* second = graph.findNode(edge.second)) {
    revision = std::max(revision, second->geometry_revision);
  }
  return revision;
}

[[nodiscard]] DirectedEdgeMetrics directedEdgeMetrics(
    const ContractedTopologyEdge3D& edge, const IncrementalTopologyNodeId from,
    const IncrementalTopologyGraph3DSnapshot& source_graph,
    const SourceEdges& source_edges, const TopologicalExplorationMemory3D& memory) {
  DirectedEdgeMetrics result;
  if (edge.source_nodes.size() != edge.source_edges.size() + 1U ||
      (from != edge.first && from != edge.second)) {
    return result;
  }
  result.edges.reserve(edge.source_edges.size());
  const bool forward = from == edge.first;
  for (std::size_t offset = 0U; offset < edge.source_edges.size(); ++offset) {
    const std::size_t edge_index =
        forward ? offset : edge.source_edges.size() - 1U - offset;
    const std::size_t from_index =
        forward ? offset : edge.source_nodes.size() - 1U - offset;
    const std::size_t to_index =
        forward ? offset + 1U : edge.source_nodes.size() - 2U - offset;
    const DirectedTopologyEdge3D directed{
        .edge_id = edge.source_edges[edge_index],
        .from = edge.source_nodes[from_index],
        .to = edge.source_nodes[to_index],
    };
    result.edges.push_back(directed);
    const auto source_found = source_edges.find(directed.edge_id);
    if (source_found == source_edges.end() || source_found->second == nullptr) {
      continue;
    }
    const IncrementalTopologyEdge3D& source_edge = *source_found->second;
    const std::uint64_t support = supportingRevision(source_graph, source_edge);
    result.supporting_revision = std::max(result.supporting_revision, support);
    const DirectedTopologyEdgeEvidence3D evidence = memory.evidence(directed, support);
    result.traversal_count += evidence.traversal_count;
    if (evidence.traversal_count > 0U) {
      result.repeated_distance_m += source_edge.length_m;
    }
    result.active_dead_end =
        result.active_dead_end ||
        evidence.result == TopologicalExplorationResult3D::kDeadEnd;
  }
  return result;
}

[[nodiscard]] double transitionCost(
    const ContractedTopologyEdge3D& edge, const IncrementalTopologyNodeId from,
    const SearchMode mode, const IncrementalTopologyGraph3DSnapshot& source_graph,
    const SourceEdges& source_edges, const TopologicalExplorationMemory3D& memory,
    const IncrementalTopologicalPlanner3DConfig& config) {
  const DirectedEdgeMetrics history =
      directedEdgeMetrics(edge, from, source_graph, source_edges, memory);
  if (mode == SearchMode::kExploration && history.active_dead_end) {
    return std::numeric_limits<double>::infinity();
  }
  double cost = edge.length_m;
  if (mode == SearchMode::kExploration) {
    cost += config.directed_traversal_penalty *
            static_cast<double>(history.traversal_count);
    cost += config.repeated_distance_penalty * history.repeated_distance_m;
  }
  return cost;
}

[[nodiscard]] SearchRecords runDijkstra(
    const ContractedTopologyGraph3D& graph, const ContractedAdjacency& adjacency,
    const IncrementalTopologyNodeId start, const SearchMode mode,
    const IncrementalTopologyGraph3DSnapshot& source_graph,
    const SourceEdges& source_edges, const TopologicalExplorationMemory3D& memory,
    const IncrementalTopologicalPlanner3DConfig& config) {
  SearchRecords records;
  records.reserve(graph.nodes().size());
  records[start] = SearchRecord{.cost = 0.0};
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueGreater> pending;
  std::uint64_t sequence = 0U;
  pending.push({.cost = 0.0, .sequence = sequence++, .node = start});
  while (!pending.empty()) {
    const QueueEntry current = pending.top();
    pending.pop();
    const auto record_found = records.find(current.node);
    if (record_found == records.end() ||
        current.cost > record_found->second.cost + 1.0e-9) {
      continue;
    }
    const auto adjacency_found = adjacency.find(current.node);
    if (adjacency_found == adjacency.end()) {
      continue;
    }
    for (const ContractedAdjacencyEntry& next : adjacency_found->second) {
      if (next.edge == nullptr || next.neighbor == current.node) {
        continue;
      }
      const double edge_cost = transitionCost(
          *next.edge, current.node, mode, source_graph, source_edges, memory, config);
      if (!std::isfinite(edge_cost)) {
        continue;
      }
      const double candidate_cost = current.cost + edge_cost;
      SearchRecord& candidate = records[next.neighbor];
      const bool strictly_better = candidate_cost + 1.0e-9 < candidate.cost;
      const bool deterministic_tie =
          std::abs(candidate_cost - candidate.cost) <= 1.0e-9 &&
          (!candidate.has_parent ||
           std::tuple{current.node.value, next.edge->id.value} <
               std::tuple{candidate.parent.value, candidate.incoming_edge->id.value});
      if (!strictly_better && !deterministic_tie) {
        continue;
      }
      candidate = SearchRecord{.cost = candidate_cost,
                               .parent = current.node,
                               .incoming_edge = next.edge,
                               .has_parent = true};
      pending.push(
          {.cost = candidate_cost, .sequence = sequence++, .node = next.neighbor});
    }
  }
  return records;
}

[[nodiscard]] std::optional<std::vector<PathStep>>
reconstructPath(const IncrementalTopologyNodeId start,
                const IncrementalTopologyNodeId target, const SearchRecords& records) {
  const auto target_found = records.find(target);
  if (target_found == records.end()) {
    return std::nullopt;
  }
  std::vector<PathStep> reverse_path;
  IncrementalTopologyNodeId current = target;
  while (current != start) {
    const auto found = records.find(current);
    if (found == records.end() || !found->second.has_parent ||
        found->second.incoming_edge == nullptr) {
      return std::nullopt;
    }
    reverse_path.push_back(PathStep{.edge = found->second.incoming_edge,
                                    .from = found->second.parent,
                                    .to = current});
    current = found->second.parent;
  }
  std::ranges::reverse(reverse_path);
  return reverse_path;
}

[[nodiscard]] double pathLength(const std::span<const PathStep> path) noexcept {
  double result = 0.0;
  for (const PathStep& step : path) {
    if (step.edge != nullptr) {
      result += step.edge->length_m;
    }
  }
  return result;
}

[[nodiscard]] PathMetrics
pathHistory(const std::span<const PathStep> path,
            const IncrementalTopologyGraph3DSnapshot& source_graph,
            const SourceEdges& source_edges,
            const TopologicalExplorationMemory3D& memory) {
  PathMetrics result;
  for (const PathStep& step : path) {
    if (step.edge == nullptr) {
      continue;
    }
    const DirectedEdgeMetrics edge =
        directedEdgeMetrics(*step.edge, step.from, source_graph, source_edges, memory);
    result.traversal_count += edge.traversal_count;
    result.repeated_distance_m += edge.repeated_distance_m;
  }
  return result;
}

void appendUnique(std::vector<Point3>& points, const Point3& point) {
  if (points.empty() || distance3D(points.back(), point) > 1.0e-9) {
    points.push_back(point);
  }
}

void appendOrientedPolyline(std::vector<Point3>& output,
                            const ContractedTopologyEdge3D& edge,
                            const IncrementalTopologyNodeId from) {
  if (from == edge.first) {
    for (const Point3& point : edge.polyline) {
      appendUnique(output, point);
    }
    return;
  }
  for (const Point3& point : std::views::reverse(edge.polyline)) {
    appendUnique(output, point);
  }
}

void materializePath(IncrementalTopologicalPlan3D& result,
                     const std::span<const PathStep> path, const Point3& start,
                     const Point3& target,
                     const IncrementalTopologyGraph3DSnapshot& source_graph,
                     const SourceEdges& source_edges,
                     const TopologicalExplorationMemory3D& memory) {
  result.guidance_points.clear();
  result.route_nodes.clear();
  result.route_steps.clear();
  appendUnique(result.guidance_points, start);
  result.route_nodes.push_back(result.start_node);
  for (const PathStep& path_step : path) {
    if (path_step.edge == nullptr) {
      continue;
    }
    const DirectedEdgeMetrics history = directedEdgeMetrics(
        *path_step.edge, path_step.from, source_graph, source_edges, memory);
    result.route_steps.push_back(TopologicalRouteStep3D{
        .contracted_edge_id = path_step.edge->id,
        .from = path_step.from,
        .to = path_step.to,
        .directed_source_edges = history.edges,
        .length_m = path_step.edge->length_m,
        .repeated_distance_m = history.repeated_distance_m,
        .traversal_count = history.traversal_count,
        .supporting_revision = history.supporting_revision,
    });
    result.directed_traversal_count += history.traversal_count;
    result.repeated_edge_distance_m += history.repeated_distance_m;
    appendOrientedPolyline(result.guidance_points, *path_step.edge, path_step.from);
    const bool forward = path_step.from == path_step.edge->first;
    if (path_step.edge->source_nodes.size() >= 2U) {
      if (forward) {
        for (const IncrementalTopologyNodeId node :
             std::span{path_step.edge->source_nodes}.subspan(1U)) {
          if (result.route_nodes.empty() || result.route_nodes.back() != node) {
            result.route_nodes.push_back(node);
          }
        }
      } else {
        for (const IncrementalTopologyNodeId node :
             std::views::reverse(path_step.edge->source_nodes)) {
          if (node == path_step.from) {
            continue;
          }
          if (result.route_nodes.empty() || result.route_nodes.back() != node) {
            result.route_nodes.push_back(node);
          }
        }
      }
    }
  }
  appendUnique(result.guidance_points, target);
  result.route_length_m = 0.0;
  for (std::size_t index = 1U; index < result.guidance_points.size(); ++index) {
    result.route_length_m +=
        distance3D(result.guidance_points[index - 1U], result.guidance_points[index]);
  }
}

[[nodiscard]] bool betterFrontier(const FrontierCandidate& candidate,
                                  const FrontierCandidate& current) noexcept {
  if (candidate.score + 1.0e-9 < current.score) {
    return true;
  }
  if (std::abs(candidate.score - current.score) > 1.0e-9 || candidate.node == nullptr ||
      current.node == nullptr || !candidate.node->observation_frontier ||
      !current.node->observation_frontier) {
    return false;
  }
  return candidate.node->observation_frontier->id <
         current.node->observation_frontier->id;
}

[[nodiscard]] std::optional<FrontierCandidate> selectFrontier(
    const IncrementalTopologyGraph3DSnapshot& source_graph,
    const ContractedTopologyGraph3D& contracted, const SearchRecords& records,
    const IncrementalTopologyNodeId start_node, const Point3& start,
    const Point3& mission_goal, const SourceEdges& source_edges,
    const TopologicalExplorationMemory3D& memory,
    const IncrementalTopologicalPlanner3DConfig& config, std::size_t& reachable_count) {
  std::optional<FrontierCandidate> best;
  for (const IncrementalTopologyNode3D& node : source_graph.nodes()) {
    if (!node.observation_frontier || contracted.findNode(node.id) == nullptr) {
      continue;
    }
    const std::optional<std::vector<PathStep>> path =
        reconstructPath(start_node, node.id, records);
    if (!path) {
      continue;
    }
    ++reachable_count;
    const ObservationFrontier& frontier = *node.observation_frontier;
    const double physical_path_m = pathLength(*path);
    const PathMetrics history = pathHistory(*path, source_graph, source_edges, memory);
    const double goal_progress_m = distance3D(start, mission_goal) -
                                   distance3D(frontier.observation_pose, mission_goal);
    const double coverage_penalty =
        memory.softCoveragePenalty(frontier.observation_pose, source_graph.revision());
    const double score =
        config.path_cost_weight * physical_path_m +
        config.directed_traversal_penalty *
            static_cast<double>(history.traversal_count) +
        config.repeated_distance_penalty * history.repeated_distance_m +
        config.frontier_selection_penalty *
            static_cast<double>(memory.frontierSelectionCount(frontier.id)) +
        config.coverage_penalty_weight * coverage_penalty -
        config.information_gain_reward *
            std::log1p(static_cast<double>(frontier.information_gain_voxels)) -
        config.clearance_reward * frontier.minimum_known_free_ray_m -
        config.goal_progress_reward * goal_progress_m;
    FrontierCandidate candidate{.node = &node,
                                .path = *path,
                                .score = score,
                                .path_length_m = physical_path_m,
                                .goal_progress_m = goal_progress_m,
                                .coverage_penalty = coverage_penalty,
                                .history = history};
    if (!best || betterFrontier(candidate, *best)) {
      best = std::move(candidate);
    }
  }
  return best;
}

[[nodiscard]] std::optional<IncrementalTopologyEdge3D>
edgeBetween(const IncrementalTopologyGraph3DSnapshot& graph,
            const IncrementalTopologyNodeId first,
            const IncrementalTopologyNodeId second) noexcept {
  for (const IncrementalTopologyEdge3D& edge : graph.edges()) {
    if ((edge.first == first && edge.second == second) ||
        (edge.first == second && edge.second == first)) {
      return edge;
    }
  }
  return std::nullopt;
}

[[nodiscard]] bool
hasUnexploredAlternative(const IncrementalTopologyGraph3DSnapshot& graph,
                         const IncrementalTopologyNodeId node,
                         const IncrementalTopologyNodeId excluded_neighbor,
                         const TopologicalExplorationMemory3D& memory) {
  for (const IncrementalTopologyEdge3D& edge : graph.edges()) {
    IncrementalTopologyNodeId neighbor{};
    if (edge.first == node) {
      neighbor = edge.second;
    } else if (edge.second == node) {
      neighbor = edge.first;
    } else {
      continue;
    }
    if (neighbor == excluded_neighbor) {
      continue;
    }
    const DirectedTopologyEdge3D directed{
        .edge_id = edge.id, .from = node, .to = neighbor};
    const DirectedTopologyEdgeEvidence3D evidence =
        memory.evidence(directed, supportingRevision(graph, edge));
    if (evidence.result == TopologicalExplorationResult3D::kUnknown &&
        evidence.traversal_count == 0U) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] std::optional<IncrementalTopologyNodeId>
selectBacktrackTarget(const IncrementalTopologyGraph3DSnapshot& source_graph,
                      const ContractedTopologyGraph3D& contracted,
                      const TopologicalExplorationMemory3D& memory,
                      TopologicalBacktrackReason3D& reason) {
  const std::span<const IncrementalTopologyNodeId> trail = memory.trail();
  if (trail.size() < 2U) {
    return std::nullopt;
  }
  for (std::size_t index = trail.size() - 1U; index > 0U; --index) {
    const IncrementalTopologyNodeId candidate = trail[index - 1U];
    if (contracted.findNode(candidate) == nullptr) {
      continue;
    }
    if (hasUnexploredAlternative(source_graph, candidate, trail[index], memory)) {
      reason = TopologicalBacktrackReason3D::kNoReachableFrontier;
      return candidate;
    }
  }
  for (const IncrementalTopologyNodeId candidate : trail) {
    if (contracted.findNode(candidate) != nullptr) {
      reason = TopologicalBacktrackReason3D::kAllReachableBranchesExplored;
      return candidate;
    }
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<TopologicalDeadEndConclusion3D>
deadEndConclusion(const IncrementalTopologyGraph3DSnapshot& graph,
                  const IncrementalTopologyNodeId start_node,
                  const TopologicalExplorationMemory3D& memory) {
  const IncrementalTopologyNode3D* node = graph.findNode(start_node);
  const std::span<const IncrementalTopologyNodeId> trail = memory.trail();
  if (node == nullptr || !node->traits.terminal || trail.size() < 2U ||
      trail.back() != start_node) {
    return std::nullopt;
  }
  const IncrementalTopologyNodeId from = trail[trail.size() - 2U];
  const std::optional<IncrementalTopologyEdge3D> edge =
      edgeBetween(graph, from, start_node);
  if (!edge.has_value()) {
    return std::nullopt;
  }
  return TopologicalDeadEndConclusion3D{
      .attempted_direction =
          DirectedTopologyEdge3D{.edge_id = edge->id, .from = from, .to = start_node},
      .supporting_revision = supportingRevision(graph, *edge),
  };
}

} // namespace

bool IncrementalTopologicalPlan3D::executableTargetSelected() const noexcept {
  return status == IncrementalTopologicalPlanStatus3D::kMissionRoute ||
         status == IncrementalTopologicalPlanStatus3D::kFrontierRoute ||
         status == IncrementalTopologicalPlanStatus3D::kBacktrackRoute;
}

bool incrementalTopologicalPlanner3DConfigIsValid(
    const IncrementalTopologicalPlanner3DConfig& config) noexcept {
  const auto valid_nonnegative = [](const double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  return valid_nonnegative(config.maximum_start_anchor_distance_m) &&
         valid_nonnegative(config.maximum_goal_anchor_distance_m) &&
         valid_nonnegative(config.path_cost_weight) &&
         valid_nonnegative(config.information_gain_reward) &&
         valid_nonnegative(config.clearance_reward) &&
         valid_nonnegative(config.goal_progress_reward) &&
         valid_nonnegative(config.directed_traversal_penalty) &&
         valid_nonnegative(config.repeated_distance_penalty) &&
         valid_nonnegative(config.frontier_selection_penalty) &&
         valid_nonnegative(config.coverage_penalty_weight);
}

IncrementalTopologicalPlanner3D::IncrementalTopologicalPlanner3D(
    const IncrementalTopologicalPlanner3DConfig& config)
    : config_{config} {
  if (!incrementalTopologicalPlanner3DConfigIsValid(config_)) {
    throw std::invalid_argument{
        "invalid incremental topological planner configuration"};
  }
}

IncrementalTopologicalPlan3D IncrementalTopologicalPlanner3D::plan(
    const IncrementalTopologyGraph3DSnapshot& graph, const Point3& start,
    const Point3& mission_goal, const TopologicalExplorationMemory3D& memory) const {
  IncrementalTopologicalPlan3D result;
  result.graph_revision = graph.revision();
  if (graph.revision() == 0U || !finitePoint(start) || !finitePoint(mission_goal)) {
    return result;
  }
  const std::optional<IncrementalTopologyNodeId> start_node =
      graph.nearestNode(start, config_.maximum_start_anchor_distance_m);
  if (!start_node) {
    result.status = IncrementalTopologicalPlanStatus3D::kStartNotRepresented;
    return result;
  }
  result.start_node = *start_node;
  result.goal_node =
      graph.nearestNode(mission_goal, config_.maximum_goal_anchor_distance_m);
  std::vector<IncrementalTopologyNodeId> anchors{*start_node};
  if (result.goal_node && *result.goal_node != *start_node) {
    anchors.push_back(*result.goal_node);
  }
  const ContractedTopologyGraph3D contracted =
      contractIncrementalTopologyGraph3D(graph, anchors);
  const ContractedAdjacency adjacency = buildContractedAdjacency(contracted);
  const SourceEdges source_edges = indexSourceEdges(graph);

  if (result.goal_node) {
    const SearchRecords mission_records =
        runDijkstra(contracted, adjacency, *start_node, SearchMode::kMission, graph,
                    source_edges, memory, config_);
    if (const std::optional<std::vector<PathStep>> path =
            reconstructPath(*start_node, *result.goal_node, mission_records)) {
      result.status = IncrementalTopologicalPlanStatus3D::kMissionRoute;
      result.purpose = IncrementalTopologicalRoutePurpose3D::kMissionTransit;
      result.target_node = *result.goal_node;
      result.reaches_mission_goal = true;
      materializePath(result, *path, start, mission_goal, graph, source_edges, memory);
      result.selection_score = result.route_length_m;
      result.goal_progress_m = distance3D(start, mission_goal);
      return result;
    }
  }

  const SearchRecords exploration_records =
      runDijkstra(contracted, adjacency, *start_node, SearchMode::kExploration, graph,
                  source_edges, memory, config_);
  std::size_t reachable_frontiers = 0U;
  const std::optional<FrontierCandidate> frontier =
      selectFrontier(graph, contracted, exploration_records, *start_node, start,
                     mission_goal, source_edges, memory, config_, reachable_frontiers);
  result.reachable_frontier_count = reachable_frontiers;
  if (frontier.has_value() && frontier->node != nullptr &&
      frontier->node->observation_frontier.has_value()) {
    result.status = IncrementalTopologicalPlanStatus3D::kFrontierRoute;
    result.purpose = IncrementalTopologicalRoutePurpose3D::kObservationFrontier;
    result.target_node = frontier->node->id;
    result.selected_frontier = frontier->node->observation_frontier;
    result.selection_score = frontier->score;
    result.goal_progress_m = frontier->goal_progress_m;
    result.coverage_penalty = frontier->coverage_penalty;
    materializePath(result, frontier->path, start,
                    frontier->node->observation_frontier->observation_pose, graph,
                    source_edges, memory);
    return result;
  }

  TopologicalBacktrackReason3D backtrack_reason{TopologicalBacktrackReason3D::kNone};
  const std::optional<IncrementalTopologyNodeId> backtrack_target =
      selectBacktrackTarget(graph, contracted, memory, backtrack_reason);
  if (!backtrack_target || *backtrack_target == *start_node) {
    result.status = IncrementalTopologicalPlanStatus3D::kNoRoute;
    result.backtrack_reason = backtrack_reason;
    return result;
  }
  const SearchRecords backtrack_records =
      runDijkstra(contracted, adjacency, *start_node, SearchMode::kBacktrack, graph,
                  source_edges, memory, config_);
  const std::optional<std::vector<PathStep>> backtrack_path =
      reconstructPath(*start_node, *backtrack_target, backtrack_records);
  const ContractedTopologyNode3D* target_node = contracted.findNode(*backtrack_target);
  if (!backtrack_path || target_node == nullptr) {
    result.status = IncrementalTopologicalPlanStatus3D::kNoRoute;
    result.backtrack_reason = backtrack_reason;
    return result;
  }
  result.status = IncrementalTopologicalPlanStatus3D::kBacktrackRoute;
  result.purpose = IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack;
  result.backtrack_reason = backtrack_reason;
  result.target_node = *backtrack_target;
  result.dead_end_conclusion = deadEndConclusion(graph, *start_node, memory);
  if (result.dead_end_conclusion) {
    result.backtrack_reason = TopologicalBacktrackReason3D::kConfirmedTerminal;
  }
  materializePath(result, *backtrack_path, start, target_node->position, graph,
                  source_edges, memory);
  result.selection_score = result.route_length_m;
  return result;
}

const IncrementalTopologicalPlanner3DConfig&
IncrementalTopologicalPlanner3D::config() const noexcept {
  return config_;
}

const char* incrementalTopologicalPlanStatus3DName(
    const IncrementalTopologicalPlanStatus3D status) noexcept {
  switch (status) {
    case IncrementalTopologicalPlanStatus3D::kInvalidInput:
      return "invalid_input";
    case IncrementalTopologicalPlanStatus3D::kStartNotRepresented:
      return "start_not_represented";
    case IncrementalTopologicalPlanStatus3D::kMissionRoute:
      return "mission_route";
    case IncrementalTopologicalPlanStatus3D::kFrontierRoute:
      return "frontier_route";
    case IncrementalTopologicalPlanStatus3D::kBacktrackRoute:
      return "backtrack_route";
    case IncrementalTopologicalPlanStatus3D::kNoRoute:
      return "no_route";
  }
  return "unknown";
}

const char* incrementalTopologicalRoutePurpose3DName(
    const IncrementalTopologicalRoutePurpose3D purpose) noexcept {
  switch (purpose) {
    case IncrementalTopologicalRoutePurpose3D::kMissionTransit:
      return "mission_transit";
    case IncrementalTopologicalRoutePurpose3D::kObservationFrontier:
      return "observation_frontier";
    case IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack:
      return "topological_backtrack";
  }
  return "unknown";
}

const char*
topologicalBacktrackReason3DName(const TopologicalBacktrackReason3D reason) noexcept {
  switch (reason) {
    case TopologicalBacktrackReason3D::kNone:
      return "none";
    case TopologicalBacktrackReason3D::kConfirmedTerminal:
      return "confirmed_terminal";
    case TopologicalBacktrackReason3D::kNoReachableFrontier:
      return "no_reachable_frontier";
    case TopologicalBacktrackReason3D::kAllReachableBranchesExplored:
      return "all_reachable_branches_explored";
  }
  return "unknown";
}

} // namespace drone_city_nav
