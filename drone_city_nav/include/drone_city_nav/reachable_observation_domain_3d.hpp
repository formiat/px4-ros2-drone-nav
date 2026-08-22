#pragma once

#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone_city_nav {

struct ReachableObservationDomain3DConfig {
  double maximum_fresh_extension_m{20.0};
  SweptFootprintConfig footprint{};
  ObservedSpaceValidationPolicy validation_policy{
      ObservedSpaceValidationPolicy::kAllowUnknown};
};

class ReachableObservationDomain3D {
public:
  [[nodiscard]] std::span<const GridIndex3D> candidateCells() const noexcept;
  [[nodiscard]] std::optional<IncrementalTopologyConnector3D>
  connect(const Point3& position, double maximum_distance_m) const;

private:
  friend ReachableObservationDomain3D
  buildReachableObservationDomain3D(const IncrementalTopologyGraph3DSnapshot&,
                                    const ObservedOccupancyGrid3D&,
                                    std::span<const IncrementalTopologyNodeId>,
                                    const ReachableObservationDomain3DConfig&);

  struct ExtensionCell {
    GridIndex3D cell{};
    std::uint64_t parent_key{0U};
    std::uint64_t root_key{0U};
    IncrementalTopologyNodeId root_node{};
    double distance_from_graph_m{0.0};
  };

  const IncrementalTopologyGraph3DSnapshot* graph_{nullptr};
  const ObservedOccupancyGrid3D* occupancy_{nullptr};
  ReachableObservationDomain3DConfig config_{};
  std::vector<GridIndex3D> candidate_cells_;
  std::unordered_map<std::uint64_t, ExtensionCell> extension_cells_;
  std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash>
      reachable_nodes_;
};

[[nodiscard]] ReachableObservationDomain3D buildReachableObservationDomain3D(
    const IncrementalTopologyGraph3DSnapshot& graph,
    const ObservedOccupancyGrid3D& occupancy,
    std::span<const IncrementalTopologyNodeId> reachable_nodes,
    const ReachableObservationDomain3DConfig& config);

[[nodiscard]] bool reachableObservationDomain3DConfigIsValid(
    const ReachableObservationDomain3DConfig& config) noexcept;

} // namespace drone_city_nav
