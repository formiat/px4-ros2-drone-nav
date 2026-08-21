#include "drone_city_nav/incremental_topological_navigation_3d.hpp"

#include <algorithm>
#include <ranges>
#include <vector>

namespace drone_city_nav {
namespace {

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

} // namespace

IncrementalTopologicalNavigation3D::IncrementalTopologicalNavigation3D(
    const IncrementalTopologyGraph3DConfig& graph_config,
    const IncrementalTopologicalPlanner3DConfig& planner_config,
    const TopologicalExplorationMemory3DConfig& memory_config)
    : graph_{graph_config},
      planner_{planner_config},
      memory_{memory_config} {
}

IncrementalTopologicalWorldUpdate3D IncrementalTopologicalNavigation3D::updateObserved(
    const ObservedOccupancyGrid3D& occupancy, const std::uint64_t revision,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks, const bool full_reset) {
  const std::scoped_lock lock{mutex_};
  IncrementalTopologicalWorldUpdate3D result;
  result.graph = graph_.update(occupancy, revision, dirty_chunks, full_reset);
  result.snapshot =
      std::make_shared<const IncrementalTopologyGraph3DSnapshot>(graph_.snapshot());
  snapshot_ = result.snapshot;
  return result;
}

IncrementalTopologicalWorldUpdate3D
IncrementalTopologicalNavigation3D::resetStatic(const OccupancyGrid3D& occupancy,
                                                const std::uint64_t revision) {
  const std::scoped_lock lock{mutex_};
  IncrementalTopologicalWorldUpdate3D result;
  result.graph = graph_.reset(occupancy, revision);
  result.snapshot =
      std::make_shared<const IncrementalTopologyGraph3DSnapshot>(graph_.snapshot());
  snapshot_ = result.snapshot;
  current_node_.reset();
  active_plan_.reset();
  memory_.clear();
  return result;
}

IncrementalTopologicalPlan3D IncrementalTopologicalNavigation3D::plan(
    const std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>& graph,
    const Point3& start, const Point3& mission_goal) const {
  if (!graph) {
    return {};
  }
  const std::scoped_lock lock{mutex_};
  return planner_.plan(*graph, start, mission_goal, memory_);
}

std::size_t IncrementalTopologicalNavigation3D::recordTransitionPath(
    const IncrementalTopologyGraph3DSnapshot& graph,
    const IncrementalTopologyNodeId from, const IncrementalTopologyNodeId to) {
  std::vector<IncrementalTopologyNodeId> transition_nodes;
  if (active_plan_) {
    if (const auto active_segment = activeRouteSegment(*active_plan_, from, to)) {
      transition_nodes = *active_segment;
    }
  }
  if (transition_nodes.empty() && edgeBetween(graph, from, to).has_value()) {
    transition_nodes = {from, to};
  }
  if (transition_nodes.size() < 2U) {
    memory_.resetTrail(to);
    return 0U;
  }

  std::size_t traversed_edges = 0U;
  for (std::size_t index = 1U; index < transition_nodes.size(); ++index) {
    const IncrementalTopologyNodeId edge_from = transition_nodes[index - 1U];
    const IncrementalTopologyNodeId edge_to = transition_nodes[index];
    const std::optional<IncrementalTopologyEdge3D> edge =
        edgeBetween(graph, edge_from, edge_to);
    if (!edge.has_value()) {
      memory_.resetTrail(to);
      return traversed_edges;
    }
    memory_.recordTraversal(
        DirectedTopologyEdge3D{.edge_id = edge->id, .from = edge_from, .to = edge_to},
        edge->supporting_revision, edge->length_m);
    ++traversed_edges;
  }
  return traversed_edges;
}

IncrementalTopologicalNavigationObservation3D
IncrementalTopologicalNavigation3D::observePosition(
    const std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>& graph,
    const Point3& position) {
  IncrementalTopologicalNavigationObservation3D result;
  if (!graph) {
    return result;
  }
  const std::scoped_lock lock{mutex_};
  result.graph_revision = graph->revision();
  result.previous_node = current_node_;
  memory_.recordVisited(position, graph->revision());
  memory_.recordObserved(position, graph->revision());
  result.coverage_cells = memory_.coverageCellCount();
  result.current_node =
      graph->nearestNode(position, planner_.config().maximum_start_anchor_distance_m);
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

IncrementalTopologicalPlanCommit3D
IncrementalTopologicalNavigation3D::commitAcceptedPlan(
    const IncrementalTopologicalPlan3D& plan) {
  IncrementalTopologicalPlanCommit3D result{.graph_revision = plan.graph_revision};
  if (!plan.executableTargetSelected()) {
    return result;
  }
  const std::scoped_lock lock{mutex_};
  if (plan.selected_frontier.has_value()) {
    memory_.recordFrontierSelection(plan.selected_frontier->id);
    result.frontier_selection_recorded = true;
  }
  if (plan.dead_end_conclusion.has_value()) {
    memory_.recordDeadEnd(plan.dead_end_conclusion->attempted_direction,
                          plan.dead_end_conclusion->supporting_revision);
    result.dead_end_recorded = true;
  }
  active_plan_ = plan;
  result.active_route_nodes = plan.route_nodes.size();
  result.accepted = true;
  return result;
}

std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>
IncrementalTopologicalNavigation3D::snapshot() const {
  const std::scoped_lock lock{mutex_};
  return snapshot_;
}

} // namespace drone_city_nav
