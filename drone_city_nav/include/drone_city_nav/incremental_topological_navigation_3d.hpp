#pragma once

#include "drone_city_nav/incremental_topological_planner_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <span>

namespace drone_city_nav {

struct IncrementalTopologicalWorldUpdate3D {
  IncrementalTopologyGraph3DUpdate graph{};
  std::shared_ptr<const IncrementalTopologyGraph3DSnapshot> snapshot;
};

struct IncrementalTopologicalNavigationObservation3D {
  std::uint64_t graph_revision{0U};
  std::optional<IncrementalTopologyNodeId> previous_node;
  std::optional<IncrementalTopologyNodeId> current_node;
  std::size_t traversed_edges{0U};
  std::size_t coverage_cells{0U};
  bool trail_reset{false};
};

struct IncrementalTopologicalPlanCommit3D {
  std::uint64_t graph_revision{0U};
  std::size_t active_route_nodes{0U};
  bool accepted{false};
  bool frontier_selection_recorded{false};
  bool dead_end_recorded{false};
};

class IncrementalTopologicalNavigation3D {
public:
  IncrementalTopologicalNavigation3D(
      const IncrementalTopologyGraph3DConfig& graph_config = {},
      const IncrementalTopologicalPlanner3DConfig& planner_config = {},
      const TopologicalExplorationMemory3DConfig& memory_config = {});

  [[nodiscard]] IncrementalTopologicalWorldUpdate3D
  updateObserved(const ObservedOccupancyGrid3D& occupancy, std::uint64_t revision,
                 std::span<const OccupancyChunkIndex3D> dirty_chunks, bool full_reset);
  [[nodiscard]] IncrementalTopologicalWorldUpdate3D
  resetStatic(const OccupancyGrid3D& occupancy, std::uint64_t revision);

  [[nodiscard]] IncrementalTopologicalPlan3D
  plan(const std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>& graph,
       const Point3& start, const Point3& mission_goal) const;
  [[nodiscard]] IncrementalTopologicalNavigationObservation3D observePosition(
      const std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>& graph,
      const Point3& position);
  [[nodiscard]] IncrementalTopologicalPlanCommit3D
  commitAcceptedPlan(const IncrementalTopologicalPlan3D& plan);

  [[nodiscard]] std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>
  snapshot() const;

private:
  [[nodiscard]] std::size_t
  recordTransitionPath(const IncrementalTopologyGraph3DSnapshot& graph,
                       IncrementalTopologyNodeId from, IncrementalTopologyNodeId to);

  mutable std::mutex mutex_;
  IncrementalTopologyGraph3D graph_;
  IncrementalTopologicalPlanner3D planner_;
  TopologicalExplorationMemory3D memory_;
  std::shared_ptr<const IncrementalTopologyGraph3DSnapshot> snapshot_;
  std::optional<IncrementalTopologyNodeId> current_node_;
  std::optional<IncrementalTopologicalPlan3D> active_plan_;
};

} // namespace drone_city_nav
