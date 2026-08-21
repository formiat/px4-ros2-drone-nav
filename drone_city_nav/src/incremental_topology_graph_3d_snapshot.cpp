#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <cmath>
#include <limits>

#include "incremental_topology_graph_3d_internal.hpp"

namespace drone_city_nav {

std::uint64_t IncrementalTopologyGraph3DSnapshot::revision() const noexcept {
  return revision_;
}

const GridBounds3D& IncrementalTopologyGraph3DSnapshot::bounds() const noexcept {
  return bounds_;
}

std::span<const IncrementalTopologyNode3D>
IncrementalTopologyGraph3DSnapshot::nodes() const noexcept {
  return nodes_;
}

std::span<const IncrementalTopologyEdge3D>
IncrementalTopologyGraph3DSnapshot::edges() const noexcept {
  return edges_;
}

const IncrementalTopologyNode3D* IncrementalTopologyGraph3DSnapshot::findNode(
    const IncrementalTopologyNodeId id) const noexcept {
  const auto found = node_indices_.find(id);
  return found == node_indices_.end() ? nullptr : &nodes_[found->second];
}

std::optional<IncrementalTopologyNodeId>
IncrementalTopologyGraph3DSnapshot::nodeForSampleCell(
    const GridIndex3D cell) const noexcept {
  const auto found = sample_cell_nodes_.find(
      incremental_topology_detail::sampleCellKey(bounds_, cell));
  return found == sample_cell_nodes_.end()
             ? std::nullopt
             : std::optional<IncrementalTopologyNodeId>{found->second};
}

std::optional<IncrementalTopologyNodeId>
IncrementalTopologyGraph3DSnapshot::nearestNode(
    const Point3& position, const double maximum_distance_m) const noexcept {
  if (!(maximum_distance_m >= 0.0) || !std::isfinite(maximum_distance_m)) {
    return std::nullopt;
  }
  std::optional<IncrementalTopologyNodeId> best;
  double best_distance = maximum_distance_m;
  for (const IncrementalTopologyNode3D& node : nodes_) {
    const double candidate_distance = distance3D(position, node.representative);
    if (candidate_distance + 1.0e-9 < best_distance ||
        (std::abs(candidate_distance - best_distance) <= 1.0e-9 &&
         (!best.has_value() || node.id < *best))) {
      best = node.id;
      best_distance = candidate_distance;
    }
  }
  return best;
}

} // namespace drone_city_nav
