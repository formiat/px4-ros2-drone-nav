#include "drone_city_nav/incremental_topological_navigation_3d.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <queue>
#include <ranges>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::optional<GridIndex3D> worldToCell(const GridBounds3D& bounds,
                                                     const Point3& point) noexcept {
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
      !(bounds.resolution_m > 0.0)) {
    return std::nullopt;
  }
  const GridIndex3D cell{
      static_cast<int>(std::floor((point.x - bounds.origin_x) / bounds.resolution_m)),
      static_cast<int>(std::floor((point.y - bounds.origin_y) / bounds.resolution_m)),
      static_cast<int>(std::floor((point.z - bounds.origin_z) / bounds.resolution_m)),
  };
  return cell.x >= 0 && cell.y >= 0 && cell.z >= 0 && cell.x < bounds.width_cells &&
                 cell.y < bounds.height_cells && cell.z < bounds.depth_cells
             ? std::optional<GridIndex3D>{cell}
             : std::nullopt;
}

[[nodiscard]] std::optional<std::vector<IncrementalTopologyNodeId>>
activeRouteSegment(const IncrementalTopologicalPlan3D& plan,
                   const IncrementalTopologyNodeId from,
                   const IncrementalTopologyNodeId to) {
  const auto from_found = std::ranges::find(plan.route_nodes, from);
  const auto to_found = std::ranges::find(plan.route_nodes, to);
  if (from_found == plan.route_nodes.end() || to_found == plan.route_nodes.end()) {
    return std::nullopt;
  }
  const std::size_t from_index =
      static_cast<std::size_t>(std::distance(plan.route_nodes.begin(), from_found));
  const std::size_t to_index =
      static_cast<std::size_t>(std::distance(plan.route_nodes.begin(), to_found));
  std::vector<IncrementalTopologyNodeId> segment;
  segment.reserve(from_index > to_index ? from_index - to_index + 1U
                                        : to_index - from_index + 1U);
  if (from_index <= to_index) {
    segment.insert(segment.end(),
                   plan.route_nodes.begin() + static_cast<std::ptrdiff_t>(from_index),
                   plan.route_nodes.begin() +
                       static_cast<std::ptrdiff_t>(to_index + 1U));
  } else {
    for (std::size_t index = from_index;; --index) {
      segment.push_back(plan.route_nodes[index]);
      if (index == to_index) {
        break;
      }
    }
  }
  return segment;
}

[[nodiscard]] std::optional<std::vector<IncrementalTopologyNodeId>>
localObservedTransition(const IncrementalTopologyGraph3DSnapshot& graph,
                        const IncrementalTopologyNodeId from,
                        const IncrementalTopologyNodeId to,
                        const double maximum_length_m) {
  const IncrementalTopologyNode3D* const from_node = graph.findNode(from);
  const IncrementalTopologyNode3D* const to_node = graph.findNode(to);
  if (from_node == nullptr || to_node == nullptr ||
      distance3D(from_node->representative, to_node->representative) >
          maximum_length_m) {
    return std::nullopt;
  }

  struct QueueEntry {
    double distance_m{0.0};
    IncrementalTopologyNodeId node{};

    [[nodiscard]] bool operator>(const QueueEntry& other) const noexcept {
      return distance_m > other.distance_m;
    }
  };

  std::unordered_map<IncrementalTopologyNodeId,
                     std::vector<const IncrementalTopologyEdge3D*>,
                     IncrementalTopologyNodeIdHash>
      adjacency;
  for (const IncrementalTopologyEdge3D& edge : graph.edges()) {
    if (!std::isfinite(edge.length_m) || edge.length_m <= 0.0 ||
        edge.length_m > maximum_length_m) {
      continue;
    }
    adjacency[edge.first].push_back(&edge);
    adjacency[edge.second].push_back(&edge);
  }

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> queue;
  std::unordered_map<IncrementalTopologyNodeId, double, IncrementalTopologyNodeIdHash>
      distances;
  std::unordered_map<IncrementalTopologyNodeId, IncrementalTopologyNodeId,
                     IncrementalTopologyNodeIdHash>
      predecessors;
  distances.emplace(from, 0.0);
  queue.push({.distance_m = 0.0, .node = from});
  while (!queue.empty()) {
    const QueueEntry current = queue.top();
    queue.pop();
    const auto distance = distances.find(current.node);
    if (distance == distances.end() || current.distance_m > distance->second + 1.0e-9) {
      continue;
    }
    if (current.node == to) {
      break;
    }
    for (const IncrementalTopologyEdge3D* const edge : adjacency[current.node]) {
      const IncrementalTopologyNodeId next =
          edge->first == current.node ? edge->second : edge->first;
      const double candidate_distance_m = current.distance_m + edge->length_m;
      if (candidate_distance_m > maximum_length_m) {
        continue;
      }
      const auto found = distances.find(next);
      if (found != distances.end() && found->second <= candidate_distance_m + 1.0e-9) {
        continue;
      }
      distances.insert_or_assign(next, candidate_distance_m);
      predecessors.insert_or_assign(next, current.node);
      queue.push({.distance_m = candidate_distance_m, .node = next});
    }
  }
  if (!distances.contains(to)) {
    return std::nullopt;
  }
  std::vector<IncrementalTopologyNodeId> reverse_path{to};
  while (reverse_path.back() != from) {
    const auto predecessor = predecessors.find(reverse_path.back());
    if (predecessor == predecessors.end()) {
      return std::nullopt;
    }
    reverse_path.push_back(predecessor->second);
  }
  std::ranges::reverse(reverse_path);
  return reverse_path;
}

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  return std::abs(first.origin_x - second.origin_x) <= 1.0e-9 &&
         std::abs(first.origin_y - second.origin_y) <= 1.0e-9 &&
         std::abs(first.origin_z - second.origin_z) <= 1.0e-9 &&
         std::abs(first.resolution_m - second.resolution_m) <= 1.0e-9 &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

} // namespace

IncrementalTopologicalWorldUpdate3D IncrementalTopologicalNavigation3D::updateObserved(
    const ObservedOccupancyGrid3D& occupancy, const std::uint64_t producer_instance_id,
    const std::uint64_t revision,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks,
    const bool complete_snapshot,
    const std::optional<IncrementalTopologyBuildPriority3D> priority,
    const std::optional<std::chrono::steady_clock::time_point> deadline) {
  bool producer_changed = false;
  IncrementalTopologicalWorldUpdate3D result;
  {
    const std::scoped_lock lock{graph_mutex_};
    if (!observed_producer_instance_id_.has_value() ||
        *observed_producer_instance_id_ != producer_instance_id) {
      graph_ = IncrementalTopologyGraph3D{graph_.config()};
      observed_producer_instance_id_ = producer_instance_id;
      producer_changed = true;
    }
    result.graph = graph_.update(occupancy, revision, dirty_chunks, complete_snapshot,
                                 priority, deadline);
    result.snapshot =
        std::make_shared<const IncrementalTopologyGraph3DSnapshot>(graph_.snapshot());
    snapshot_ = result.snapshot;
  }
  if (producer_changed) {
    const std::scoped_lock lock{memory_mutex_};
    current_node_.reset();
    strategic_route_manager_.reset();
    memory_.clear();
  }
  return result;
}

IncrementalTopologicalWorldUpdate3D
IncrementalTopologicalNavigation3D::resetStatic(const OccupancyGrid3D& occupancy,
                                                const std::uint64_t revision) {
  IncrementalTopologicalWorldUpdate3D result;
  {
    const std::scoped_lock lock{graph_mutex_};
    result.graph = graph_.reset(occupancy, revision);
    result.snapshot =
        std::make_shared<const IncrementalTopologyGraph3DSnapshot>(graph_.snapshot());
    snapshot_ = result.snapshot;
    observed_producer_instance_id_.reset();
  }
  {
    const std::scoped_lock lock{memory_mutex_};
    current_node_.reset();
    strategic_route_manager_.reset();
    memory_.clear();
  }
  return result;
}

std::size_t IncrementalTopologicalNavigation3D::recordTransitionPath(
    const IncrementalTopologyGraph3DSnapshot& graph,
    const IncrementalTopologyNodeId from, const IncrementalTopologyNodeId to) {
  std::vector<IncrementalTopologyNodeId> transition_nodes;
  const StrategicRoutePlan3D* const strategic_route = strategic_route_manager_.active();
  if (strategic_route != nullptr) {
    if (const auto active_segment =
            activeRouteSegment(strategic_route->plan, from, to)) {
      transition_nodes = *active_segment;
    }
  }
  if (transition_nodes.size() < 2U) {
    if (const auto local_transition = localObservedTransition(
            graph, from, to, memory_.config().maximum_observed_transition_m)) {
      transition_nodes = *local_transition;
    }
  }
  if (transition_nodes.size() < 2U) {
    memory_.resetTrail(to);
    return 0U;
  }

  std::size_t traversed_edges = 0U;
  for (std::size_t index = 1U; index < transition_nodes.size(); ++index) {
    const IncrementalTopologyNodeId edge_from = transition_nodes[index - 1U];
    const IncrementalTopologyNodeId edge_to = transition_nodes[index];
    const auto edge = std::ranges::find_if(
        graph.edges(),
        [edge_from, edge_to](const IncrementalTopologyEdge3D& candidate) {
          return (candidate.first == edge_from && candidate.second == edge_to) ||
                 (candidate.first == edge_to && candidate.second == edge_from);
        });
    if (edge == graph.edges().end()) {
      memory_.resetTrail(to);
      return traversed_edges;
    }
    memory_.recordTraversal(
        DirectedTopologyEdge3D{.edge_id = edge->id, .from = edge_from, .to = edge_to},
        edge->evidence.validated_through_revision, edge->length_m);
    ++traversed_edges;
  }
  return traversed_edges;
}

IncrementalTopologicalNavigationObservation3D
IncrementalTopologicalNavigation3D::observePosition(
    const std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>& graph,
    const Point3& position, const ObservedOccupancyGrid3D* const occupancy) {
  IncrementalTopologicalNavigationObservation3D result;
  if (!graph) {
    return result;
  }
  result.position = position;
  std::optional<IncrementalTopologyNodeId> observed_node;
  if (occupancy != nullptr && sameBounds(graph->bounds(), occupancy->bounds())) {
    result.current_anchor = graph->connectObserved(
        *occupancy, position, planner_.config().maximum_start_anchor_distance_m,
        observability_.footprint,
        planner_.config().require_known_free_space
            ? ObservedSpaceValidationPolicy::kRequireKnownFree
            : ObservedSpaceValidationPolicy::kAllowUnknown);
    if (result.current_anchor.has_value()) {
      observed_node = result.current_anchor->node;
    }
  } else if (const std::optional<GridIndex3D> cell =
                 worldToCell(graph->bounds(), position)) {
    observed_node = graph->nodeForSampleCell(*cell);
  }
  if (!observed_node.has_value() && occupancy == nullptr) {
    observed_node =
        graph->nearestNode(position, planner_.config().maximum_start_anchor_distance_m);
  }

  const std::scoped_lock lock{memory_mutex_};
  result.graph_revision = graph->revision();
  result.previous_node = current_node_;
  memory_.recordVisited(position, graph->revision());
  memory_.recordObserved(position, graph->revision());
  result.coverage_cells = memory_.coverageCellCount();
  result.current_node = observed_node;
  if (!result.current_node.has_value()) {
    current_node_.reset();
    return result;
  }
  if (!current_node_.has_value() || graph->findNode(*current_node_) == nullptr) {
    memory_.resetTrail(*result.current_node);
    result.trail_reset = true;
  } else if (*current_node_ != *result.current_node) {
    result.traversed_edges =
        recordTransitionPath(*graph, *current_node_, *result.current_node);
    result.trail_reset = result.traversed_edges == 0U;
  }
  current_node_ = result.current_node;
  return result;
}

std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>
IncrementalTopologicalNavigation3D::snapshot() const {
  const std::scoped_lock lock{graph_mutex_};
  return snapshot_;
}

} // namespace drone_city_nav
