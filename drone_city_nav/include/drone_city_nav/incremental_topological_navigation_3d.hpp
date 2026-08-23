#pragma once

#include "drone_city_nav/incremental_topological_lattice_adapter_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/strategic_route_manager_3d.hpp"

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
  std::uint64_t strategic_plan_id{0U};
  std::uint64_t graph_revision{0U};
  std::size_t active_route_nodes{0U};
  bool accepted{false};
  bool replaced_frontier_coverage_recorded{false};
  bool frontier_selection_recorded{false};
  bool dead_end_recorded{false};
};

struct IncrementalTopologicalNavigation3DConfig {
  double active_route_completion_tolerance_m{2.0};
  double maximum_active_route_cross_track_m{20.0};
};

[[nodiscard]] bool incrementalTopologicalNavigation3DConfigIsValid(
    const IncrementalTopologicalNavigation3DConfig& config) noexcept;

class IncrementalTopologicalNavigation3D {
public:
  IncrementalTopologicalNavigation3D(
      const IncrementalTopologyGraph3DConfig& graph_config = {},
      const IncrementalTopologicalPlanner3DConfig& planner_config = {},
      const TopologicalExplorationMemory3DConfig& memory_config = {},
      const SensorObservabilityConfig& observability = {},
      const IncrementalTopologicalNavigation3DConfig& navigation_config = {});

  [[nodiscard]] IncrementalTopologicalWorldUpdate3D updateObserved(
      const ObservedOccupancyGrid3D& occupancy, std::uint64_t producer_instance_id,
      std::uint64_t revision, std::span<const OccupancyChunkIndex3D> dirty_chunks,
      bool complete_snapshot,
      std::optional<IncrementalTopologyBuildPriority3D> priority = std::nullopt);
  [[nodiscard]] IncrementalTopologicalWorldUpdate3D
  resetStatic(const OccupancyGrid3D& occupancy, std::uint64_t revision);

  [[nodiscard]] IncrementalTopologicalPlan3D
  plan(const std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>& graph,
       const Point3& start, const Point3& mission_goal);
  [[nodiscard]] IncrementalTopologicalPlan3D
  planObserved(const std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>& graph,
               const ObservedOccupancyGrid3D& occupancy, const Point3& start,
               const Point3& mission_goal);
  [[nodiscard]] std::optional<IncrementalTopologicalLatticeDirective3D>
  makeLatticeDirective(const IncrementalTopologicalPlan3D& plan, const Point3& position,
                       const IncrementalTopologicalLatticeAdapter3DConfig& config = {});
  [[nodiscard]] IncrementalTopologicalNavigationObservation3D observePosition(
      const std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>& graph,
      const Point3& position, const ObservedOccupancyGrid3D* occupancy = nullptr);
  [[nodiscard]] IncrementalTopologicalPlanCommit3D
  commitAcceptedPlan(const IncrementalTopologicalPlan3D& plan);
  [[nodiscard]] bool invalidateAcceptedPlan(const IncrementalTopologicalPlan3D& plan);
  [[nodiscard]] bool supersedeAcceptedPlan();
  void rejectObservationFrontier(ObservationFrontierId frontier_id);
  void completeObservationFrontier(const ObservationFrontier& frontier,
                                   std::uint64_t revision);
  void beginMissionLeg();

  [[nodiscard]] std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>
  snapshot() const;

private:
  [[nodiscard]] std::size_t
  recordTransitionPath(const IncrementalTopologyGraph3DSnapshot& graph,
                       IncrementalTopologyNodeId from, IncrementalTopologyNodeId to);
  [[nodiscard]] std::optional<IncrementalTopologicalPlan3D>
  continueAcceptedObservedPlan(const Point3& start, const Point3& mission_goal,
                               const ObservedOccupancyGrid3D& occupancy,
                               std::uint64_t current_revision);

  mutable std::mutex graph_mutex_;
  mutable std::mutex memory_mutex_;
  IncrementalTopologyGraph3D graph_;
  IncrementalTopologicalPlanner3D planner_;
  TopologicalExplorationMemory3D memory_;
  SensorObservabilityConfig observability_;
  IncrementalTopologicalNavigation3DConfig navigation_config_;
  std::shared_ptr<const IncrementalTopologyGraph3DSnapshot> snapshot_;
  std::optional<std::uint64_t> observed_producer_instance_id_;
  std::optional<IncrementalTopologyNodeId> current_node_;
  StrategicRouteManager3D strategic_route_manager_;
};

} // namespace drone_city_nav
