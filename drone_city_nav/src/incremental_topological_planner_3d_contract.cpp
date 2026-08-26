#include "drone_city_nav/incremental_topological_planner_3d.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_city_nav {

bool IncrementalTopologicalPlan3D::executableTargetSelected() const noexcept {
  return status == IncrementalTopologicalPlanStatus3D::kMissionRoute ||
         status == IncrementalTopologicalPlanStatus3D::kMissionContinuationRoute ||
         status == IncrementalTopologicalPlanStatus3D::kFrontierRoute ||
         status == IncrementalTopologicalPlanStatus3D::kBacktrackRoute;
}

bool incrementalTopologicalPlanner3DConfigIsValid(
    const IncrementalTopologicalPlanner3DConfig& config) noexcept {
  const auto valid_nonnegative = [](const double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  return valid_nonnegative(config.maximum_start_anchor_distance_m) &&
         valid_nonnegative(config.maximum_goal_anchor_distance_m) &&
         valid_nonnegative(config.minimum_mission_continuation_goal_progress_m) &&
         valid_nonnegative(config.path_cost_weight) &&
         valid_nonnegative(config.information_gain_reward) &&
         valid_nonnegative(config.clearance_reward) &&
         valid_nonnegative(config.goal_progress_reward) &&
         valid_nonnegative(config.directed_traversal_penalty) &&
         valid_nonnegative(config.repeated_distance_penalty) &&
         valid_nonnegative(config.dead_end_penalty) &&
         valid_nonnegative(config.frontier_selection_penalty) &&
         valid_nonnegative(config.frontier_completion_penalty) &&
         valid_nonnegative(config.coverage_penalty_weight) &&
         std::isfinite(config.maximum_fresh_frontier_anchor_distance_m) &&
         config.maximum_fresh_frontier_anchor_distance_m > 0.0 &&
         std::isfinite(config.minimum_observation_target_displacement_m) &&
         config.minimum_observation_target_displacement_m > 0.0 &&
         config.maximum_fresh_frontier_evaluations > 0U;
}

IncrementalTopologicalPlanner3D::IncrementalTopologicalPlanner3D(
    const IncrementalTopologicalPlanner3DConfig& config)
    : config_{config} {
  if (!incrementalTopologicalPlanner3DConfigIsValid(config_)) {
    throw std::invalid_argument{
        "invalid incremental topological planner configuration"};
  }
}

IncrementalTopologicalPlan3D IncrementalTopologicalPlanner3D::plan(
    const IncrementalTopologyGraph3DSnapshot& graph, const Point3& start,
    const Point3& mission_goal, const TopologicalExplorationMemory3D& memory,
    const std::optional<std::chrono::steady_clock::time_point> deadline) const {
  return planImpl(graph, nullptr, nullptr, start, mission_goal, memory, std::nullopt,
                  deadline, nullptr);
}

IncrementalTopologicalPlan3D IncrementalTopologicalPlanner3D::planObserved(
    const IncrementalTopologyGraph3DSnapshot& graph,
    const ObservedOccupancyGrid3D& occupancy,
    const SensorObservabilityConfig& observability, const Point3& start,
    const Point3& mission_goal, const TopologicalExplorationMemory3D& memory,
    const std::optional<ObservationFrontier> active_frontier,
    const std::optional<std::chrono::steady_clock::time_point> deadline) const {
  return planImpl(graph, &occupancy, &observability, start, mission_goal, memory,
                  active_frontier, deadline, nullptr);
}

IncrementalTopologicalPlan3D
IncrementalTopologicalPlanner3D::planObservedFromStartConnector(
    const IncrementalTopologyGraph3DSnapshot& graph,
    const ObservedOccupancyGrid3D& occupancy,
    const SensorObservabilityConfig& observability, const Point3& start,
    const Point3& mission_goal, const TopologicalExplorationMemory3D& memory,
    const IncrementalTopologyConnector3D& start_connector,
    const std::optional<ObservationFrontier> active_frontier,
    const std::optional<std::chrono::steady_clock::time_point> deadline) const {
  return planImpl(graph, &occupancy, &observability, start, mission_goal, memory,
                  active_frontier, deadline, &start_connector);
}

const IncrementalTopologicalPlanner3DConfig&
IncrementalTopologicalPlanner3D::config() const noexcept {
  return config_;
}

bool isExplicitTopologicalBacktrack3D(
    const IncrementalTopologicalPlan3D& plan) noexcept {
  return plan.status == IncrementalTopologicalPlanStatus3D::kBacktrackRoute ||
         plan.purpose == IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack;
}

const char* incrementalTopologicalPlanStatus3DName(
    const IncrementalTopologicalPlanStatus3D status) noexcept {
  switch (status) {
    case IncrementalTopologicalPlanStatus3D::kInvalidInput:
      return "invalid_input";
    case IncrementalTopologicalPlanStatus3D::kStartNotRepresented:
      return "start_not_represented";
    case IncrementalTopologicalPlanStatus3D::kMissionRoute:
      return "mission_route";
    case IncrementalTopologicalPlanStatus3D::kMissionContinuationRoute:
      return "mission_continuation_route";
    case IncrementalTopologicalPlanStatus3D::kFrontierRoute:
      return "frontier_route";
    case IncrementalTopologicalPlanStatus3D::kBacktrackRoute:
      return "backtrack_route";
    case IncrementalTopologicalPlanStatus3D::kDeadlineExceeded:
      return "deadline_exceeded";
    case IncrementalTopologicalPlanStatus3D::kNoRoute:
      return "no_route";
  }
  return "unknown";
}

const char* incrementalTopologicalRoutePurpose3DName(
    const IncrementalTopologicalRoutePurpose3D purpose) noexcept {
  switch (purpose) {
    case IncrementalTopologicalRoutePurpose3D::kMissionTransit:
      return "mission_transit";
    case IncrementalTopologicalRoutePurpose3D::kObservationFrontier:
      return "observation_frontier";
    case IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack:
      return "topological_backtrack";
  }
  return "unknown";
}

const char*
topologicalBacktrackReason3DName(const TopologicalBacktrackReason3D reason) noexcept {
  switch (reason) {
    case TopologicalBacktrackReason3D::kNone:
      return "none";
    case TopologicalBacktrackReason3D::kConfirmedTerminal:
      return "confirmed_terminal";
    case TopologicalBacktrackReason3D::kNoReachableFrontier:
      return "no_reachable_frontier";
    case TopologicalBacktrackReason3D::kAllReachableBranchesExplored:
      return "all_reachable_branches_explored";
  }
  return "unknown";
}

} // namespace drone_city_nav
