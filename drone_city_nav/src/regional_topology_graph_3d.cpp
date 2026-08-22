#include "drone_city_nav/regional_topology_graph_3d.hpp"

#include <algorithm>
#include <cstdint>
#include <ranges>

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

[[nodiscard]] RegionalTopologyEdgeId3D
makeRegionalEdgeId(const IncrementalTopologyEdgeId source_edge) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashUnsigned(hash, source_edge.value);
  return RegionalTopologyEdgeId3D{hash == 0U ? 1U : hash};
}

} // namespace

RegionalTopologyGraph3D
buildRegionalTopologyGraph3D(const IncrementalTopologyGraph3DSnapshot& graph,
                             const std::span<const IncrementalTopologyNodeId> anchors) {
  static_cast<void>(anchors);
  RegionalTopologyGraph3D result;
  result.source_revision_ = graph.revision();
  result.nodes_.reserve(graph.nodes().size());
  for (const IncrementalTopologyNode3D& node : graph.nodes()) {
    result.nodes_.push_back(RegionalTopologyNode3D{
        .source_node_id = node.id,
        .position = node.representative,
        .traits = node.traits,
        .degree = node.degree,
    });
  }

  result.edges_.reserve(graph.edges().size());
  for (const IncrementalTopologyEdge3D& edge : graph.edges()) {
    if (edge.polyline.size() < 2U) {
      continue;
    }
    result.edges_.push_back(RegionalTopologyEdge3D{
        .id = makeRegionalEdgeId(edge.id),
        .first = edge.first,
        .second = edge.second,
        .source_nodes = {edge.first, edge.second},
        .source_edges = {edge.id},
        .polyline = edge.polyline,
        .length_m = edge.length_m,
        .validated_through_revision = edge.validated_through_revision,
    });
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
