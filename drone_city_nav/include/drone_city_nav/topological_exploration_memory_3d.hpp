#pragma once

#include "drone_city_nav/incremental_topology_graph_3d.hpp"
#include "drone_city_nav/observation_frontier.hpp"
#include "drone_city_nav/types.hpp"

#include <compare>
#include <cstddef>
#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {

struct DirectedTopologyEdge3D {
  IncrementalTopologyEdgeId edge_id{};
  IncrementalTopologyNodeId from{};
  IncrementalTopologyNodeId to{};

  [[nodiscard]] auto
  operator<=>(const DirectedTopologyEdge3D&) const noexcept = default;
};

struct DirectedTopologyEdge3DHash {
  [[nodiscard]] std::size_t
  operator()(const DirectedTopologyEdge3D& edge) const noexcept;
};

enum class TopologicalExplorationResult3D : std::uint8_t {
  kUnknown,
  kTraversed,
  kDeadEnd,
};

struct DirectedTopologyEdgeEvidence3D {
  std::size_t traversal_count{0U};
  double traversed_distance_m{0.0};
  std::uint64_t last_traversal_revision{0U};
  std::uint64_t conclusion_revision{0U};
  TopologicalExplorationResult3D result{TopologicalExplorationResult3D::kUnknown};
};

struct SparseCoverageCell3D {
  std::size_t visit_count{0U};
  std::size_t observation_count{0U};
  std::uint64_t last_visit_revision{0U};
  std::uint64_t last_observation_revision{0U};
};

struct TopologicalExplorationMemory3DConfig {
  double coverage_resolution_m{2.0};
  double visit_penalty_weight{2.0};
  double observation_penalty_weight{0.25};
  double revision_decay{0.02};
  std::size_t maximum_trail_nodes{4096U};
};

class TopologicalExplorationMemory3D {
public:
  explicit TopologicalExplorationMemory3D(
      const TopologicalExplorationMemory3DConfig& config = {});

  void recordTraversal(const DirectedTopologyEdge3D& edge,
                       std::uint64_t supporting_revision, double distance_m);
  void recordDeadEnd(const DirectedTopologyEdge3D& edge,
                     std::uint64_t supporting_revision);
  [[nodiscard]] DirectedTopologyEdgeEvidence3D
  evidence(const DirectedTopologyEdge3D& edge,
           std::uint64_t current_supporting_revision) const noexcept;

  void recordVisited(const Point3& position, std::uint64_t revision);
  void recordObserved(const Point3& position, std::uint64_t revision);
  void recordVisitedPath(std::span<const Point3> points, std::uint64_t revision,
                         double sample_spacing_m);
  [[nodiscard]] double
  softCoveragePenalty(const Point3& position,
                      std::uint64_t current_revision) const noexcept;
  [[nodiscard]] std::size_t coverageCellCount() const noexcept;

  void recordFrontierSelection(ObservationFrontierId frontier_id);
  [[nodiscard]] std::size_t
  frontierSelectionCount(ObservationFrontierId frontier_id) const noexcept;

  void resetTrail(IncrementalTopologyNodeId node);
  void recordTrailTransition(IncrementalTopologyNodeId from,
                             IncrementalTopologyNodeId to);
  [[nodiscard]] std::span<const IncrementalTopologyNodeId> trail() const noexcept;

  void clear();
  [[nodiscard]] const TopologicalExplorationMemory3DConfig& config() const noexcept;

private:
  struct SparseCoverageIndex3D {
    int x{0};
    int y{0};
    int z{0};

    [[nodiscard]] auto
    operator<=>(const SparseCoverageIndex3D&) const noexcept = default;
  };

  struct SparseCoverageIndex3DHash {
    [[nodiscard]] std::size_t
    operator()(const SparseCoverageIndex3D& index) const noexcept;
  };

  [[nodiscard]] SparseCoverageIndex3D
  coverageIndex(const Point3& position) const noexcept;

  TopologicalExplorationMemory3DConfig config_{};
  std::unordered_map<DirectedTopologyEdge3D, DirectedTopologyEdgeEvidence3D,
                     DirectedTopologyEdge3DHash>
      edge_evidence_;
  std::unordered_map<SparseCoverageIndex3D, SparseCoverageCell3D,
                     SparseCoverageIndex3DHash>
      coverage_;
  std::unordered_map<std::uint64_t, std::size_t> frontier_selection_counts_;
  std::vector<IncrementalTopologyNodeId> trail_;
};

[[nodiscard]] bool topologicalExplorationMemory3DConfigIsValid(
    const TopologicalExplorationMemory3DConfig& config) noexcept;

[[nodiscard]] const char*
topologicalExplorationResult3DName(TopologicalExplorationResult3D result) noexcept;

} // namespace drone_city_nav
