#pragma once

#include "drone_city_nav/incremental_topology_graph_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <compare>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

struct RegionalTopologyEdgeId3D {
  std::uint64_t value{0U};

  [[nodiscard]] auto
  operator<=>(const RegionalTopologyEdgeId3D&) const noexcept = default;
};

struct RegionalTopologyNode3D {
  IncrementalTopologyNodeId source_node_id{};
  Point3 position{};
  IncrementalTopologyNodeTraits3D traits{};
  std::size_t degree{0U};
  std::uint64_t created_on_revision{0U};
  std::uint64_t validated_through_revision{0U};
  std::uint64_t complete_through_revision{0U};
  std::uint64_t generation{0U};
  IncrementalTopologyLineageEvent3D lineage_event{
      IncrementalTopologyLineageEvent3D::kCreated};
  std::vector<IncrementalTopologyNodeId> predecessors;
  bool unknown_boundary_exposure{false};
};

struct RegionalTopologyEdge3D {
  RegionalTopologyEdgeId3D id{};
  IncrementalTopologyNodeId first{};
  IncrementalTopologyNodeId second{};
  std::vector<IncrementalTopologyNodeId> source_nodes;
  std::vector<IncrementalTopologyEdgeId> source_edges;
  std::vector<Point3> polyline;
  double length_m{0.0};
  std::uint64_t created_on_revision{0U};
  IncrementalTopologyTransitionEvidence3D evidence{};
};

class RegionalTopologyGraph3D {
public:
  [[nodiscard]] std::uint64_t sourceRevision() const noexcept;
  [[nodiscard]] std::span<const RegionalTopologyNode3D> nodes() const noexcept;
  [[nodiscard]] std::span<const RegionalTopologyEdge3D> edges() const noexcept;
  [[nodiscard]] const RegionalTopologyNode3D*
  findNode(IncrementalTopologyNodeId id) const noexcept;
  [[nodiscard]] const RegionalTopologyEdge3D*
  findEdge(RegionalTopologyEdgeId3D id) const noexcept;

private:
  friend RegionalTopologyGraph3D
  buildRegionalTopologyGraph3D(const IncrementalTopologyGraph3DSnapshot&,
                               std::span<const IncrementalTopologyNodeId>);

  std::uint64_t source_revision_{0U};
  std::vector<RegionalTopologyNode3D> nodes_;
  std::vector<RegionalTopologyEdge3D> edges_;
};

[[nodiscard]] RegionalTopologyGraph3D
buildRegionalTopologyGraph3D(const IncrementalTopologyGraph3DSnapshot& graph,
                             std::span<const IncrementalTopologyNodeId> anchors = {});

} // namespace drone_city_nav
