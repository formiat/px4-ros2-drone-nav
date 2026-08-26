#pragma once

#include "drone_city_nav/incremental_topology_graph_3d.hpp"
#include "drone_city_nav/observation_frontier.hpp"
#include "drone_city_nav/regional_topology_graph_3d.hpp"
#include "drone_city_nav/topological_exploration_memory_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace drone_city_nav {

class IncrementalTopologicalNavigation3D;

enum class IncrementalTopologicalPlanStatus3D : std::uint8_t {
  kInvalidInput,
  kStartNotRepresented,
  kMissionRoute,
  kMissionContinuationRoute,
  kFrontierRoute,
  kBacktrackRoute,
  kDeadlineExceeded,
  kNoRoute,
};

enum class IncrementalTopologicalRoutePurpose3D : std::uint8_t {
  kMissionTransit,
  kObservationFrontier,
  kTopologicalBacktrack,
};

enum class TopologicalBacktrackReason3D : std::uint8_t {
  kNone,
  kConfirmedTerminal,
  kNoReachableFrontier,
  kAllReachableBranchesExplored,
};

struct TopologicalRouteStep3D {
  RegionalTopologyEdgeId3D regional_edge_id{};
  IncrementalTopologyNodeId from{};
  IncrementalTopologyNodeId to{};
  std::vector<DirectedTopologyEdge3D> directed_source_edges;
  double length_m{0.0};
  double repeated_distance_m{0.0};
  std::size_t traversal_count{0U};
  IncrementalTopologyTransitionEvidence3D evidence{};
};

struct TopologicalDeadEndConclusion3D {
  DirectedTopologyEdge3D attempted_direction{};
  std::uint64_t validated_through_revision{0U};
};

struct IncrementalTopologicalPlanningTiming3D {
  double memory_snapshot_ms{0.0};
  double start_connector_ms{0.0};
  double goal_connector_ms{0.0};
  double regional_graph_ms{0.0};
  double mission_search_ms{0.0};
  double exploration_search_ms{0.0};
  double mission_selection_ms{0.0};
  double frontier_selection_ms{0.0};
  double backtrack_search_ms{0.0};
};

struct IncrementalTopologicalPlan3D {
  IncrementalTopologicalPlanStatus3D status{
      IncrementalTopologicalPlanStatus3D::kInvalidInput};
  IncrementalTopologicalRoutePurpose3D purpose{
      IncrementalTopologicalRoutePurpose3D::kMissionTransit};
  TopologicalBacktrackReason3D backtrack_reason{TopologicalBacktrackReason3D::kNone};
  std::uint64_t strategic_plan_id{0U};
  std::uint64_t planned_on_revision{0U};
  std::uint64_t validated_through_revision{0U};
  std::uint64_t complete_through_revision{0U};
  std::uint64_t topology_lineage_id{0U};
  Point3 mission_target{};
  IncrementalTopologyNodeId start_node{};
  IncrementalTopologyNodeId target_node{};
  std::optional<IncrementalTopologyNodeId> goal_node;
  std::optional<ObservationFrontier> selected_frontier;
  std::optional<TopologicalDeadEndConclusion3D> dead_end_conclusion;
  std::vector<IncrementalTopologyNodeId> route_nodes;
  std::vector<TopologicalRouteStep3D> route_steps;
  std::vector<Point3> guidance_points;
  double route_length_m{0.0};
  double selection_score{0.0};
  double goal_progress_m{0.0};
  double coverage_penalty{0.0};
  std::size_t selected_frontier_selection_count{0U};
  std::size_t selected_frontier_completion_count{0U};
  double repeated_edge_distance_m{0.0};
  std::size_t directed_traversal_count{0U};
  std::size_t transition_support_segment_count{0U};
  std::size_t reachable_mission_continuation_count{0U};
  double maximum_reachable_mission_continuation_goal_progress_m{0.0};
  std::size_t reachable_frontier_count{0U};
  std::size_t goal_directed_reachable_frontier_count{0U};
  ObservationFrontierId maximum_goal_progress_frontier_id{};
  double maximum_reachable_frontier_goal_progress_m{0.0};
  std::size_t fresh_frontier_candidate_count{0U};
  std::size_t fresh_frontier_evaluated_count{0U};
  std::size_t fresh_frontier_discovered_count{0U};
  std::uint64_t fresh_frontier_sample_fingerprint{0U};
  std::array<std::size_t, 7U> fresh_frontier_status_counts{};
  bool fresh_frontier_budget_exhausted{false};
  bool reaches_mission_goal{false};
  bool unknown_exposure{false};
  bool continued_from_active_plan{false};
  IncrementalTopologicalPlanningTiming3D timing{};

  [[nodiscard]] bool executableTargetSelected() const noexcept;
};

struct IncrementalTopologicalPlanner3DConfig {
  double maximum_start_anchor_distance_m{20.0};
  bool require_known_free_space{false};
  double maximum_goal_anchor_distance_m{8.0};
  double minimum_mission_continuation_goal_progress_m{8.0};
  double path_cost_weight{1.0};
  double information_gain_reward{1.0};
  double clearance_reward{0.5};
  double goal_progress_reward{3.0};
  double directed_traversal_penalty{6.0};
  double repeated_distance_penalty{0.25};
  double dead_end_penalty{100.0};
  double frontier_selection_penalty{8.0};
  // Completion is stronger evidence than merely committing a candidate. It
  // remains a soft preference: a completed frontier can still be selected if
  // it is the only viable way to expose new free space.
  double frontier_completion_penalty{48.0};
  double coverage_penalty_weight{1.0};
  double maximum_fresh_frontier_anchor_distance_m{20.0};
  // A frontier destination must extend beyond the terminal-control
  // neighbourhood of the current vehicle state. This preserves finite-path
  // stopping semantics without turning already-reached observations into
  // repeated sub-metre missions.
  double minimum_observation_target_displacement_m{2.0};
  std::size_t maximum_fresh_frontier_evaluations{128U};
  // Ray evaluation is deadline-bounded, but an evaluated frontier still needs
  // graph reconnection, raw validation, scoring, and route materialization.
  // Preserve an explicit tail budget for those mandatory completion stages.
  double fresh_frontier_materialization_reserve_ms{20.0};
};

class IncrementalTopologicalPlanner3D {
public:
  explicit IncrementalTopologicalPlanner3D(
      const IncrementalTopologicalPlanner3DConfig& config = {});

  [[nodiscard]] IncrementalTopologicalPlan3D
  plan(const IncrementalTopologyGraph3DSnapshot& graph, const Point3& start,
       const Point3& mission_goal, const TopologicalExplorationMemory3D& memory,
       std::optional<std::chrono::steady_clock::time_point> deadline =
           std::nullopt) const;

  [[nodiscard]] IncrementalTopologicalPlan3D
  planObserved(const IncrementalTopologyGraph3DSnapshot& graph,
               const ObservedOccupancyGrid3D& occupancy,
               const SensorObservabilityConfig& observability, const Point3& start,
               const Point3& mission_goal, const TopologicalExplorationMemory3D& memory,
               std::optional<ObservationFrontier> active_frontier = std::nullopt,
               std::optional<std::chrono::steady_clock::time_point> deadline =
                   std::nullopt) const;

  [[nodiscard]] const IncrementalTopologicalPlanner3DConfig& config() const noexcept;

private:
  friend class IncrementalTopologicalNavigation3D;

  [[nodiscard]] IncrementalTopologicalPlan3D planObservedFromStartConnector(
      const IncrementalTopologyGraph3DSnapshot& graph,
      const ObservedOccupancyGrid3D& occupancy,
      const SensorObservabilityConfig& observability, const Point3& start,
      const Point3& mission_goal, const TopologicalExplorationMemory3D& memory,
      const IncrementalTopologyConnector3D& start_connector,
      std::optional<ObservationFrontier> active_frontier,
      std::optional<std::chrono::steady_clock::time_point> deadline) const;

  [[nodiscard]] IncrementalTopologicalPlan3D
  planImpl(const IncrementalTopologyGraph3DSnapshot& graph,
           const ObservedOccupancyGrid3D* occupancy,
           const SensorObservabilityConfig* observability, const Point3& start,
           const Point3& mission_goal, const TopologicalExplorationMemory3D& memory,
           std::optional<ObservationFrontier> active_frontier,
           std::optional<std::chrono::steady_clock::time_point> deadline,
           const IncrementalTopologyConnector3D* start_connector) const;

  IncrementalTopologicalPlanner3DConfig config_{};
};

[[nodiscard]] bool incrementalTopologicalPlanner3DConfigIsValid(
    const IncrementalTopologicalPlanner3DConfig& config) noexcept;

[[nodiscard]] bool
isExplicitTopologicalBacktrack3D(const IncrementalTopologicalPlan3D& plan) noexcept;

[[nodiscard]] const char* incrementalTopologicalPlanStatus3DName(
    IncrementalTopologicalPlanStatus3D status) noexcept;
[[nodiscard]] const char* incrementalTopologicalRoutePurpose3DName(
    IncrementalTopologicalRoutePurpose3D purpose) noexcept;
[[nodiscard]] const char*
topologicalBacktrackReason3DName(TopologicalBacktrackReason3D reason) noexcept;

} // namespace drone_city_nav
