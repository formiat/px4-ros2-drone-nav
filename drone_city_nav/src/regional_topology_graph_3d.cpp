#include "drone_city_nav/regional_topology_graph_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
#include <ranges>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

void hashUnsigned(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t shift = 0U; shift < sizeof(value); ++shift) {
    hash ^= static_cast<std::uint8_t>(value >> (shift * 8U));
    hash *= kFnvPrime;
  }
}

[[nodiscard]] RegionalTopologyEdgeId3D makeRegionalEdgeId(
    const IncrementalTopologyNodeId first, const IncrementalTopologyNodeId second,
    const std::span<const IncrementalTopologyEdgeId> source_edges) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashUnsigned(hash, first.value);
  hashUnsigned(hash, second.value);
  for (const IncrementalTopologyEdgeId source_edge : source_edges) {
    hashUnsigned(hash, source_edge.value);
  }
  return RegionalTopologyEdgeId3D{hash == 0U ? 1U : hash};
}

struct SourceAdjacencyEntry {
  IncrementalTopologyNodeId neighbor{};
  const IncrementalTopologyEdge3D* edge{nullptr};
};

using SourceAdjacency =
    std::unordered_map<IncrementalTopologyNodeId, std::vector<SourceAdjacencyEntry>,
                       IncrementalTopologyNodeIdHash>;
using SourceNodes =
    std::unordered_map<IncrementalTopologyNodeId, const IncrementalTopologyNode3D*,
                       IncrementalTopologyNodeIdHash>;

[[nodiscard]] SourceNodes
indexSourceNodes(const IncrementalTopologyGraph3DSnapshot& graph) {
  SourceNodes result;
  result.reserve(graph.nodes().size());
  for (const IncrementalTopologyNode3D& node : graph.nodes()) {
    result.emplace(node.id, &node);
  }
  return result;
}

[[nodiscard]] SourceAdjacency
buildSourceAdjacency(const IncrementalTopologyGraph3DSnapshot& graph) {
  SourceAdjacency result;
  result.reserve(graph.nodes().size());
  for (const IncrementalTopologyNode3D& node : graph.nodes()) {
    result.try_emplace(node.id);
  }
  for (const IncrementalTopologyEdge3D& edge : graph.edges()) {
    if (edge.first == edge.second || edge.polyline.size() < 2U ||
        !result.contains(edge.first) || !result.contains(edge.second)) {
      continue;
    }
    result[edge.first].push_back(SourceAdjacencyEntry{edge.second, &edge});
    result[edge.second].push_back(SourceAdjacencyEntry{edge.first, &edge});
  }
  for (auto& [node, entries] : result) {
    static_cast<void>(node);
    std::ranges::sort(entries, [](const SourceAdjacencyEntry& first,
                                  const SourceAdjacencyEntry& second) {
      return std::tie(first.neighbor.value, first.edge->id.value) <
             std::tie(second.neighbor.value, second.edge->id.value);
    });
  }
  return result;
}

[[nodiscard]] bool hasStrategicTrait(const IncrementalTopologyNode3D& node) noexcept {
  return node.traits.junction || node.traits.turn || node.traits.vertical_connector ||
         node.traits.frontier || node.traits.terminal;
}

void preserveCycleAnchors(
    const SourceAdjacency& adjacency, const SourceNodes& nodes,
    std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash>&
        preserved) {
  std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash> visited;
  for (const auto& [seed, unused] : adjacency) {
    static_cast<void>(unused);
    if (!visited.insert(seed).second) {
      continue;
    }
    std::vector<IncrementalTopologyNodeId> component;
    std::vector<IncrementalTopologyNodeId> pending{seed};
    while (!pending.empty()) {
      const IncrementalTopologyNodeId current = pending.back();
      pending.pop_back();
      component.push_back(current);
      for (const SourceAdjacencyEntry& edge : adjacency.at(current)) {
        if (visited.insert(edge.neighbor).second) {
          pending.push_back(edge.neighbor);
        }
      }
    }
    std::ranges::sort(component);
    std::vector<IncrementalTopologyNodeId> component_anchors;
    std::ranges::copy_if(
        component, std::back_inserter(component_anchors),
        [&](const IncrementalTopologyNodeId node) { return preserved.contains(node); });
    if (component_anchors.empty() && !component.empty()) {
      preserved.insert(component.front());
      component_anchors.push_back(component.front());
    }
    if (component.size() <= 1U || component_anchors.size() >= 2U) {
      continue;
    }
    const IncrementalTopologyNode3D* const anchor = nodes.at(component_anchors.front());
    std::optional<IncrementalTopologyNodeId> farthest;
    double farthest_distance_m{-1.0};
    for (const IncrementalTopologyNodeId candidate : component) {
      if (candidate == anchor->id) {
        continue;
      }
      const double distance_m =
          distance3D(anchor->representative, nodes.at(candidate)->representative);
      if (distance_m > farthest_distance_m + 1.0e-9 ||
          (std::abs(distance_m - farthest_distance_m) <= 1.0e-9 &&
           (!farthest.has_value() || candidate < *farthest))) {
        farthest = candidate;
        farthest_distance_m = distance_m;
      }
    }
    if (farthest.has_value()) {
      preserved.insert(*farthest);
    }
  }
}

void appendUnique(std::vector<Point3>& output, const Point3& point) {
  if (output.empty() || distance3D(output.back(), point) > 1.0e-9) {
    output.push_back(point);
  }
}

void appendOrientedEdge(std::vector<Point3>& output,
                        const IncrementalTopologyEdge3D& edge,
                        const IncrementalTopologyNodeId from,
                        const IncrementalTopologyNodeId to) {
  if (edge.first == from && edge.second == to) {
    for (const Point3& point : edge.polyline) {
      appendUnique(output, point);
    }
    return;
  }
  if (edge.second == from && edge.first == to) {
    for (const Point3& point : edge.polyline | std::views::reverse) {
      appendUnique(output, point);
    }
  }
}

[[nodiscard]] double polylineLength(const std::span<const Point3> polyline) noexcept {
  double result = 0.0;
  for (std::size_t index = 1U; index < polyline.size(); ++index) {
    result += distance3D(polyline[index - 1U], polyline[index]);
  }
  return result;
}

} // namespace

RegionalTopologyGraph3D
buildRegionalTopologyGraph3D(const IncrementalTopologyGraph3DSnapshot& graph,
                             const std::span<const IncrementalTopologyNodeId> anchors) {
  RegionalTopologyGraph3D result;
  result.source_revision_ = graph.revision();
  const SourceNodes nodes = indexSourceNodes(graph);
  const SourceAdjacency adjacency = buildSourceAdjacency(graph);
  std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash>
      preserved;
  preserved.reserve(graph.nodes().size());
  for (const IncrementalTopologyNodeId anchor : anchors) {
    if (nodes.contains(anchor)) {
      preserved.insert(anchor);
    }
  }
  for (const IncrementalTopologyNode3D& node : graph.nodes()) {
    const auto adjacent = adjacency.find(node.id);
    const std::size_t degree =
        adjacent == adjacency.end() ? 0U : adjacent->second.size();
    // A degree-two voxel is route geometry, not a strategic decision. Its full
    // certified polyline remains on the contracted edge, so preserving every
    // grid-staircase bend or observation-boundary cell only defeats regional
    // contraction without adding connectivity information.
    if (degree != 2U || hasStrategicTrait(node)) {
      preserved.insert(node.id);
    }
  }
  preserveCycleAnchors(adjacency, nodes, preserved);

  std::vector<IncrementalTopologyNodeId> ordered_nodes{preserved.begin(),
                                                       preserved.end()};
  std::ranges::sort(ordered_nodes);
  result.nodes_.reserve(ordered_nodes.size());
  for (const IncrementalTopologyNodeId node_id : ordered_nodes) {
    const IncrementalTopologyNode3D& node = *nodes.at(node_id);
    result.nodes_.push_back(RegionalTopologyNode3D{
        .source_node_id = node.id,
        .position = node.representative,
        .traits =
            IncrementalTopologyNodeTraits3D{
                .junction = node.traits.junction,
                .turn = node.traits.turn,
                .vertical_connector = node.traits.vertical_connector,
                .frontier = node.traits.frontier,
                .terminal = node.traits.terminal,
            },
        .created_on_revision = node.created_on_revision,
        .validated_through_revision = node.validated_through_revision,
        .complete_through_revision = node.complete_through_revision,
        .generation = node.generation,
        .lineage_event = node.lineage_event,
        .predecessors = node.predecessors,
        .unknown_boundary_exposure = node.unknown_boundary_exposure,
    });
  }

  std::unordered_set<IncrementalTopologyEdgeId, IncrementalTopologyEdgeIdHash>
      consumed_edges;
  consumed_edges.reserve(graph.edges().size());
  result.edges_.reserve(graph.edges().size());
  for (const IncrementalTopologyNodeId start : ordered_nodes) {
    for (const SourceAdjacencyEntry& initial : adjacency.at(start)) {
      if (initial.edge == nullptr || consumed_edges.contains(initial.edge->id)) {
        continue;
      }
      std::vector<IncrementalTopologyNodeId> source_nodes{start};
      std::vector<IncrementalTopologyEdgeId> source_edges;
      std::vector<Point3> polyline;
      std::uint64_t validated_through = std::numeric_limits<std::uint64_t>::max();
      std::uint64_t complete_through = std::numeric_limits<std::uint64_t>::max();
      std::uint64_t transition_lineage{kFnvOffset};
      std::size_t support_segment_count = 0U;
      bool unknown_exposure = false;
      std::uint64_t created_on = 0U;
      IncrementalTopologyNodeId current = start;
      const IncrementalTopologyEdge3D* edge = initial.edge;
      bool complete = false;
      for (std::size_t guard = 0U; guard <= graph.edges().size(); ++guard) {
        if (edge == nullptr || consumed_edges.contains(edge->id)) {
          break;
        }
        IncrementalTopologyNodeId next;
        if (edge->first == current) {
          next = edge->second;
        } else if (edge->second == current) {
          next = edge->first;
        }
        if (next.value == 0U) {
          break;
        }
        consumed_edges.insert(edge->id);
        source_edges.push_back(edge->id);
        source_nodes.push_back(next);
        appendOrientedEdge(polyline, *edge, current, next);
        validated_through =
            std::min(validated_through, edge->evidence.validated_through_revision);
        complete_through =
            std::min(complete_through, edge->evidence.complete_through_revision);
        support_segment_count += edge->evidence.support_segment_count;
        unknown_exposure = unknown_exposure || edge->evidence.unknown_exposure;
        hashUnsigned(transition_lineage, edge->evidence.lineage_id);
        created_on = std::max(created_on, edge->created_on_revision);
        if (preserved.contains(next)) {
          complete = true;
          break;
        }
        const auto next_adjacency = adjacency.find(next);
        if (next_adjacency == adjacency.end() || next_adjacency->second.size() != 2U) {
          break;
        }
        const auto continuation = std::ranges::find_if(
            next_adjacency->second, [&](const SourceAdjacencyEntry& candidate) {
              return candidate.edge != nullptr && candidate.edge->id != edge->id;
            });
        if (continuation == next_adjacency->second.end()) {
          break;
        }
        current = next;
        edge = continuation->edge;
      }
      if (!complete || source_edges.empty() || polyline.size() < 2U) {
        continue;
      }
      IncrementalTopologyNodeId first = source_nodes.front();
      IncrementalTopologyNodeId second = source_nodes.back();
      if (second < first) {
        std::swap(first, second);
        std::ranges::reverse(source_nodes);
        std::ranges::reverse(source_edges);
        std::ranges::reverse(polyline);
      }
      const double length_m = polylineLength(polyline);
      result.edges_.push_back(RegionalTopologyEdge3D{
          .id = makeRegionalEdgeId(first, second, source_edges),
          .first = first,
          .second = second,
          .source_nodes = std::move(source_nodes),
          .source_edges = std::move(source_edges),
          .polyline = std::move(polyline),
          .length_m = length_m,
          .created_on_revision = created_on,
          .evidence =
              IncrementalTopologyTransitionEvidence3D{
                  .kind = unknown_exposure
                              ? IncrementalTopologyTransitionKind3D::kOptimisticUnknown
                              : IncrementalTopologyTransitionKind3D::kObservedFree,
                  .support_segment_count = support_segment_count,
                  .validated_through_revision =
                      validated_through == std::numeric_limits<std::uint64_t>::max()
                          ? 0U
                          : validated_through,
                  .complete_through_revision =
                      complete_through == std::numeric_limits<std::uint64_t>::max()
                          ? 0U
                          : complete_through,
                  .lineage_id = transition_lineage == 0U ? 1U : transition_lineage,
                  .unknown_exposure = unknown_exposure,
              },
      });
    }
  }
  std::unordered_map<IncrementalTopologyNodeId, std::size_t,
                     IncrementalTopologyNodeIdHash>
      regional_degree;
  for (const RegionalTopologyEdge3D& edge : result.edges_) {
    ++regional_degree[edge.first];
    ++regional_degree[edge.second];
  }
  for (RegionalTopologyNode3D& node : result.nodes_) {
    node.degree = regional_degree[node.source_node_id];
    node.traits.junction = node.degree >= 3U;
    node.traits.terminal = node.degree <= 1U && !node.unknown_boundary_exposure;
  }
  std::ranges::sort(result.nodes_, {}, &RegionalTopologyNode3D::source_node_id);
  std::ranges::sort(result.edges_, {}, &RegionalTopologyEdge3D::id);
  return result;
}

std::uint64_t RegionalTopologyGraph3D::sourceRevision() const noexcept {
  return source_revision_;
}

std::span<const RegionalTopologyNode3D>
RegionalTopologyGraph3D::nodes() const noexcept {
  return nodes_;
}

std::span<const RegionalTopologyEdge3D>
RegionalTopologyGraph3D::edges() const noexcept {
  return edges_;
}

const RegionalTopologyNode3D*
RegionalTopologyGraph3D::findNode(const IncrementalTopologyNodeId id) const noexcept {
  const auto found =
      std::ranges::find(nodes_, id, &RegionalTopologyNode3D::source_node_id);
  return found == nodes_.end() ? nullptr : &*found;
}

const RegionalTopologyEdge3D*
RegionalTopologyGraph3D::findEdge(const RegionalTopologyEdgeId3D id) const noexcept {
  const auto found = std::ranges::find(edges_, id, &RegionalTopologyEdge3D::id);
  return found == edges_.end() ? nullptr : &*found;
}

} // namespace drone_city_nav
