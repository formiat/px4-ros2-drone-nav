#include "drone_city_nav/contracted_topology_graph_3d.hpp"

#include <algorithm>
#include <cstdint>
#include <deque>
#include <limits>
#include <ranges>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

struct AdjacencyEntry {
  IncrementalTopologyNodeId neighbor{};
  const IncrementalTopologyEdge3D* edge{nullptr};
};

using Adjacency =
    std::unordered_map<IncrementalTopologyNodeId, std::vector<AdjacencyEntry>,
                       IncrementalTopologyNodeIdHash>;

void hashUnsigned(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t shift = 0U; shift < sizeof(value); ++shift) {
    hash ^= static_cast<std::uint8_t>(value >> (shift * 8U));
    hash *= kFnvPrime;
  }
}

[[nodiscard]] bool isTopologicalEvent(const IncrementalTopologyNode3D& node) noexcept {
  return node.degree != 2U || node.traits.junction || node.traits.turn ||
         node.traits.vertical_connector || node.traits.frontier || node.traits.terminal;
}

[[nodiscard]] Adjacency
buildAdjacency(const IncrementalTopologyGraph3DSnapshot& graph) {
  Adjacency result;
  result.reserve(graph.nodes().size());
  for (const IncrementalTopologyNode3D& node : graph.nodes()) {
    result.try_emplace(node.id);
  }
  for (const IncrementalTopologyEdge3D& edge : graph.edges()) {
    result[edge.first].push_back({.neighbor = edge.second, .edge = &edge});
    result[edge.second].push_back({.neighbor = edge.first, .edge = &edge});
  }
  for (auto& [node_id, entries] : result) {
    static_cast<void>(node_id);
    std::ranges::sort(entries,
                      [](const AdjacencyEntry& first, const AdjacencyEntry& second) {
                        return std::tuple{first.neighbor.value, first.edge->id.value} <
                               std::tuple{second.neighbor.value, second.edge->id.value};
                      });
  }
  return result;
}

void ensureCycleAnchors(const IncrementalTopologyGraph3DSnapshot& graph,
                        const Adjacency& adjacency,
                        std::unordered_set<IncrementalTopologyNodeId,
                                           IncrementalTopologyNodeIdHash>& events) {
  std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash> visited;
  for (const IncrementalTopologyNode3D& seed : graph.nodes()) {
    if (visited.contains(seed.id)) {
      continue;
    }
    std::deque<IncrementalTopologyNodeId> pending{seed.id};
    visited.insert(seed.id);
    IncrementalTopologyNodeId minimum = seed.id;
    bool component_has_event = false;
    while (!pending.empty()) {
      const IncrementalTopologyNodeId current = pending.front();
      pending.pop_front();
      minimum = std::min(minimum, current);
      component_has_event = component_has_event || events.contains(current);
      const auto found = adjacency.find(current);
      if (found == adjacency.end()) {
        continue;
      }
      for (const AdjacencyEntry& entry : found->second) {
        if (visited.insert(entry.neighbor).second) {
          pending.push_back(entry.neighbor);
        }
      }
    }
    if (!component_has_event) {
      events.insert(minimum);
    }
  }
}

void appendUnique(std::vector<Point3>& output, const Point3& point) {
  if (output.empty() || distance3D(output.back(), point) > 1.0e-9) {
    output.push_back(point);
  }
}

void appendOrientedEdgeGeometry(std::vector<Point3>& output,
                                const IncrementalTopologyEdge3D& edge,
                                const IncrementalTopologyNodeId from,
                                const IncrementalTopologyNodeId to,
                                const IncrementalTopologyGraph3DSnapshot& graph) {
  const IncrementalTopologyNode3D* from_node = graph.findNode(from);
  const IncrementalTopologyNode3D* to_node = graph.findNode(to);
  if (from_node == nullptr || to_node == nullptr) {
    return;
  }
  appendUnique(output, from_node->representative);
  if (edge.first == from && edge.second == to) {
    appendUnique(output, edge.first_contact);
    appendUnique(output, edge.second_contact);
  } else {
    appendUnique(output, edge.second_contact);
    appendUnique(output, edge.first_contact);
  }
  appendUnique(output, to_node->representative);
}

[[nodiscard]] ContractedTopologyEdgeId3D
makeContractedEdgeId(const IncrementalTopologyNodeId first,
                     const IncrementalTopologyNodeId second,
                     const std::span<const IncrementalTopologyEdgeId> source_edges) {
  std::vector<std::uint64_t> forward;
  forward.reserve(source_edges.size());
  for (const IncrementalTopologyEdgeId edge : source_edges) {
    forward.push_back(edge.value);
  }
  std::vector<std::uint64_t> reverse = forward;
  std::ranges::reverse(reverse);
  const std::vector<std::uint64_t>& canonical =
      std::ranges::lexicographical_compare(reverse, forward) ? reverse : forward;
  std::uint64_t hash{kFnvOffset};
  hashUnsigned(hash, std::min(first, second).value);
  hashUnsigned(hash, std::max(first, second).value);
  hashUnsigned(hash, static_cast<std::uint64_t>(canonical.size()));
  for (const std::uint64_t edge : canonical) {
    hashUnsigned(hash, edge);
  }
  return ContractedTopologyEdgeId3D{hash == 0U ? 1U : hash};
}

[[nodiscard]] std::vector<IncrementalTopologyNodeId>
sortedEvents(const std::unordered_set<IncrementalTopologyNodeId,
                                      IncrementalTopologyNodeIdHash>& events) {
  std::vector<IncrementalTopologyNodeId> result{events.begin(), events.end()};
  std::ranges::sort(result);
  return result;
}

[[nodiscard]] const AdjacencyEntry*
continuation(const std::vector<AdjacencyEntry>& entries,
             const IncrementalTopologyEdgeId previous_edge) noexcept {
  for (const AdjacencyEntry& entry : entries) {
    if (entry.edge != nullptr && entry.edge->id != previous_edge) {
      return &entry;
    }
  }
  return nullptr;
}

} // namespace

ContractedTopologyGraph3D contractIncrementalTopologyGraph3D(
    const IncrementalTopologyGraph3DSnapshot& graph,
    const std::span<const IncrementalTopologyNodeId> anchors) {
  ContractedTopologyGraph3D result;
  result.source_revision_ = graph.revision();
  if (graph.nodes().empty()) {
    return result;
  }

  const Adjacency adjacency = buildAdjacency(graph);
  std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash> events;
  for (const IncrementalTopologyNode3D& node : graph.nodes()) {
    if (isTopologicalEvent(node)) {
      events.insert(node.id);
    }
  }
  for (const IncrementalTopologyNodeId anchor : anchors) {
    if (graph.findNode(anchor) != nullptr) {
      events.insert(anchor);
    }
  }
  ensureCycleAnchors(graph, adjacency, events);

  std::unordered_set<IncrementalTopologyEdgeId, IncrementalTopologyEdgeIdHash>
      consumed_edges;
  for (const IncrementalTopologyNodeId start : sortedEvents(events)) {
    const auto start_adjacency = adjacency.find(start);
    if (start_adjacency == adjacency.end()) {
      continue;
    }
    for (const AdjacencyEntry& first_step : start_adjacency->second) {
      if (first_step.edge == nullptr || consumed_edges.contains(first_step.edge->id)) {
        continue;
      }
      ContractedTopologyEdge3D contracted;
      contracted.first = start;
      contracted.source_nodes.push_back(start);
      IncrementalTopologyNodeId current = start;
      const AdjacencyEntry* step = &first_step;
      while (step != nullptr && step->edge != nullptr &&
             !consumed_edges.contains(step->edge->id)) {
        consumed_edges.insert(step->edge->id);
        contracted.source_edges.push_back(step->edge->id);
        contracted.length_m += step->edge->length_m;
        contracted.supporting_revision =
            std::max(contracted.supporting_revision, step->edge->supporting_revision);
        appendOrientedEdgeGeometry(contracted.polyline, *step->edge, current,
                                   step->neighbor, graph);
        current = step->neighbor;
        contracted.source_nodes.push_back(current);
        if (events.contains(current)) {
          break;
        }
        const auto next_adjacency = adjacency.find(current);
        if (next_adjacency == adjacency.end()) {
          break;
        }
        step = continuation(next_adjacency->second, step->edge->id);
      }
      contracted.second = current;
      contracted.id = makeContractedEdgeId(contracted.first, contracted.second,
                                           contracted.source_edges);
      for (const IncrementalTopologyNodeId node_id : contracted.source_nodes) {
        if (const IncrementalTopologyNode3D* node = graph.findNode(node_id)) {
          contracted.supporting_revision =
              std::max(contracted.supporting_revision, node->geometry_revision);
        }
      }
      result.edges_.push_back(std::move(contracted));
    }
  }

  result.nodes_.reserve(events.size());
  for (const IncrementalTopologyNodeId event : sortedEvents(events)) {
    if (const IncrementalTopologyNode3D* node = graph.findNode(event)) {
      result.nodes_.push_back(ContractedTopologyNode3D{
          .source_node_id = event,
          .position = node->representative,
          .traits = node->traits,
      });
    }
  }
  for (ContractedTopologyNode3D& node : result.nodes_) {
    for (const ContractedTopologyEdge3D& edge : result.edges_) {
      if (edge.first == node.source_node_id) {
        ++node.degree;
      }
      if (edge.second == node.source_node_id) {
        ++node.degree;
      }
    }
  }
  std::ranges::sort(result.edges_, {}, &ContractedTopologyEdge3D::id);
  return result;
}

std::uint64_t ContractedTopologyGraph3D::sourceRevision() const noexcept {
  return source_revision_;
}

std::span<const ContractedTopologyNode3D>
ContractedTopologyGraph3D::nodes() const noexcept {
  return nodes_;
}

std::span<const ContractedTopologyEdge3D>
ContractedTopologyGraph3D::edges() const noexcept {
  return edges_;
}

const ContractedTopologyNode3D*
ContractedTopologyGraph3D::findNode(const IncrementalTopologyNodeId id) const noexcept {
  const auto found =
      std::ranges::find(nodes_, id, &ContractedTopologyNode3D::source_node_id);
  return found == nodes_.end() ? nullptr : &*found;
}

const ContractedTopologyEdge3D* ContractedTopologyGraph3D::findEdge(
    const ContractedTopologyEdgeId3D id) const noexcept {
  const auto found = std::ranges::find(edges_, id, &ContractedTopologyEdge3D::id);
  return found == edges_.end() ? nullptr : &*found;
}

} // namespace drone_city_nav
