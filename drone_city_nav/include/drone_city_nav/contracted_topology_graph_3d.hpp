#pragma once

#include "drone_city_nav/incremental_topology_graph_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <compare>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

struct ContractedTopologyEdgeId3D {
  std::uint64_t value{0U};

  [[nodiscard]] auto
  operator<=>(const ContractedTopologyEdgeId3D&) const noexcept = default;
};

struct ContractedTopologyNode3D {
  IncrementalTopologyNodeId source_node_id{};
  Point3 position{};
  IncrementalTopologyNodeTraits3D traits{};
  std::size_t degree{0U};
};

struct ContractedTopologyEdge3D {
  ContractedTopologyEdgeId3D id{};
  IncrementalTopologyNodeId first{};
  IncrementalTopologyNodeId second{};
  std::vector<IncrementalTopologyNodeId> source_nodes;
  std::vector<IncrementalTopologyEdgeId> source_edges;
  std::vector<Point3> polyline;
  double length_m{0.0};
  std::uint64_t supporting_revision{0U};
};

class ContractedTopologyGraph3D {
public:
  [[nodiscard]] std::uint64_t sourceRevision() const noexcept;
  [[nodiscard]] std::span<const ContractedTopologyNode3D> nodes() const noexcept;
  [[nodiscard]] std::span<const ContractedTopologyEdge3D> edges() const noexcept;
  [[nodiscard]] const ContractedTopologyNode3D*
  findNode(IncrementalTopologyNodeId id) const noexcept;
  [[nodiscard]] const ContractedTopologyEdge3D*
  findEdge(ContractedTopologyEdgeId3D id) const noexcept;

private:
  friend ContractedTopologyGraph3D
  contractIncrementalTopologyGraph3D(const IncrementalTopologyGraph3DSnapshot&,
                                     std::span<const IncrementalTopologyNodeId>);

  std::uint64_t source_revision_{0U};
  std::vector<ContractedTopologyNode3D> nodes_;
  std::vector<ContractedTopologyEdge3D> edges_;
};

[[nodiscard]] ContractedTopologyGraph3D contractIncrementalTopologyGraph3D(
    const IncrementalTopologyGraph3DSnapshot& graph,
    std::span<const IncrementalTopologyNodeId> anchors = {});

} // namespace drone_city_nav
