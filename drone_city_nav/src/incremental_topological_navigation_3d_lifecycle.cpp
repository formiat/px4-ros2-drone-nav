#include "drone_city_nav/incremental_topological_navigation_3d.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace drone_city_nav {
namespace {

constexpr double kMissionTargetIdentityToleranceM{1.0e-6};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  return std::abs(first.origin_x - second.origin_x) <= 1.0e-9 &&
         std::abs(first.origin_y - second.origin_y) <= 1.0e-9 &&
         std::abs(first.origin_z - second.origin_z) <= 1.0e-9 &&
         std::abs(first.resolution_m - second.resolution_m) <= 1.0e-9 &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

[[nodiscard]] bool samePlanIdentity(const IncrementalTopologicalPlan3D& first,
                                    const IncrementalTopologicalPlan3D& second) {
  if (first.status != second.status || first.purpose != second.purpose ||
      first.planned_on_revision != second.planned_on_revision ||
      first.target_node != second.target_node ||
      distance3D(first.mission_target, second.mission_target) >
          kMissionTargetIdentityToleranceM ||
      first.route_nodes != second.route_nodes ||
      first.guidance_points.size() != second.guidance_points.size()) {
    return false;
  }
  if (first.selected_frontier.has_value() != second.selected_frontier.has_value() ||
      (first.selected_frontier.has_value() &&
       first.selected_frontier->id != second.selected_frontier->id)) {
    return false;
  }
  return std::ranges::equal(first.guidance_points, second.guidance_points,
                            [](const Point3& lhs, const Point3& rhs) {
                              return distance3D(lhs, rhs) <=
                                     kMissionTargetIdentityToleranceM;
                            });
}

} // namespace

bool incrementalTopologicalNavigation3DConfigIsValid(
    const IncrementalTopologicalNavigation3DConfig& config) noexcept {
  return std::isfinite(config.active_route_completion_tolerance_m) &&
         config.active_route_completion_tolerance_m >= 0.0;
}

IncrementalTopologicalNavigation3D::IncrementalTopologicalNavigation3D(
    const IncrementalTopologyGraph3DConfig& graph_config,
    const IncrementalTopologicalPlanner3DConfig& planner_config,
    const TopologicalExplorationMemory3DConfig& memory_config,
    const SensorObservabilityConfig& observability,
    const IncrementalTopologicalNavigation3DConfig& navigation_config)
    : graph_{graph_config},
      planner_{planner_config},
      memory_{memory_config},
      observability_{observability},
      navigation_config_{navigation_config} {
  if (!incrementalTopologicalNavigation3DConfigIsValid(navigation_config_)) {
    throw std::invalid_argument{
        "invalid incremental topological navigation configuration"};
  }
}

std::optional<IncrementalTopologicalPlan3D>
IncrementalTopologicalNavigation3D::continueAcceptedObservedPlan(
    const Point3& start, const Point3& mission_goal,
    const ObservedOccupancyGrid3D& occupancy, const std::uint64_t current_revision) {
  std::optional<IncrementalTopologicalPlan3D> candidate;
  {
    const std::scoped_lock lock{memory_mutex_};
    candidate = active_plan_;
  }
  if (!candidate.has_value()) {
    return std::nullopt;
  }

  bool continuable = candidate->executableTargetSelected() && finitePoint(start) &&
                     finitePoint(mission_goal) &&
                     distance3D(candidate->mission_target, mission_goal) <=
                         kMissionTargetIdentityToleranceM &&
                     candidate->guidance_points.size() >= 2U &&
                     distance3D(start, candidate->guidance_points.back()) >
                         navigation_config_.active_route_completion_tolerance_m;
  if (continuable && candidate->selected_frontier.has_value()) {
    const ObservationFrontierSetEvaluation frontier_evaluation =
        evaluateObservationFrontiers(
            occupancy, candidate->selected_frontier->observation_pose, current_revision,
            observability_,
            planner_.config().require_known_free_space
                ? ObservedSpaceValidationPolicy::kRequireKnownFree
                : ObservedSpaceValidationPolicy::kAllowUnknown);
    continuable = frontier_evaluation.accepted();
  }
  if (continuable) {
    candidate->continued_from_active_plan = true;
    return candidate;
  }

  const std::scoped_lock lock{memory_mutex_};
  if (active_plan_.has_value() && samePlanIdentity(*active_plan_, *candidate)) {
    active_plan_.reset();
  }
  return std::nullopt;
}

IncrementalTopologicalPlan3D IncrementalTopologicalNavigation3D::plan(
    const std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>& graph,
    const Point3& start, const Point3& mission_goal) {
  if (!graph) {
    return {};
  }
  TopologicalExplorationMemory3D memory_snapshot;
  {
    const std::scoped_lock lock{memory_mutex_};
    memory_snapshot = memory_;
  }
  return planner_.plan(*graph, start, mission_goal, memory_snapshot);
}

IncrementalTopologicalPlan3D IncrementalTopologicalNavigation3D::planObserved(
    const std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>& graph,
    const ObservedOccupancyGrid3D& occupancy, const Point3& start,
    const Point3& mission_goal) {
  if (!graph || !sameBounds(graph->bounds(), occupancy.bounds())) {
    return {};
  }
  if (const auto continued = continueAcceptedObservedPlan(
          start, mission_goal, occupancy, graph->revision())) {
    return *continued;
  }
  TopologicalExplorationMemory3D memory_snapshot;
  {
    const std::scoped_lock lock{memory_mutex_};
    memory_snapshot = memory_;
  }
  return planner_.planObserved(*graph, occupancy, observability_, start, mission_goal,
                               memory_snapshot, std::nullopt);
}

IncrementalTopologicalPlanCommit3D
IncrementalTopologicalNavigation3D::commitAcceptedPlan(
    const IncrementalTopologicalPlan3D& plan) {
  IncrementalTopologicalPlanCommit3D result{.graph_revision = plan.planned_on_revision};
  if (!plan.executableTargetSelected()) {
    return result;
  }
  const std::scoped_lock lock{memory_mutex_};
  const std::optional<Point3> replaced_frontier_observation_pose = [&]() {
    if (!active_plan_.has_value() || !active_plan_->selected_frontier.has_value() ||
        !plan.selected_frontier.has_value()) {
      return std::optional<Point3>{};
    }
    if (active_plan_->selected_frontier->id != plan.selected_frontier->id) {
      return std::optional<Point3>{active_plan_->selected_frontier->observation_pose};
    }
    if (distance3D(active_plan_->selected_frontier->observation_pose,
                   plan.selected_frontier->observation_pose) <=
        memory_.config().coverage_resolution_m) {
      return std::optional<Point3>{};
    }
    return std::optional<Point3>{active_plan_->selected_frontier->observation_pose};
  }();
  if (replaced_frontier_observation_pose.has_value()) {
    memory_.recordObserved(*replaced_frontier_observation_pose,
                           plan.validated_through_revision);
    result.replaced_frontier_coverage_recorded = true;
  }
  const bool selected_frontier_already_active = [&]() {
    if (!active_plan_.has_value() || !active_plan_->selected_frontier.has_value() ||
        !plan.selected_frontier.has_value() ||
        active_plan_->selected_frontier->id != plan.selected_frontier->id) {
      return false;
    }
    return distance3D(active_plan_->selected_frontier->observation_pose,
                      plan.selected_frontier->observation_pose) <=
           memory_.config().coverage_resolution_m;
  }();
  if (plan.selected_frontier.has_value() && !selected_frontier_already_active) {
    memory_.recordFrontierSelection(plan.selected_frontier->id);
    result.frontier_selection_recorded = true;
  }
  const bool dead_end_already_active =
      active_plan_.has_value() && active_plan_->dead_end_conclusion.has_value() &&
      plan.dead_end_conclusion.has_value() &&
      active_plan_->dead_end_conclusion->attempted_direction ==
          plan.dead_end_conclusion->attempted_direction &&
      active_plan_->dead_end_conclusion->validated_through_revision ==
          plan.dead_end_conclusion->validated_through_revision;
  if (plan.dead_end_conclusion.has_value() && !dead_end_already_active) {
    memory_.recordDeadEnd(plan.dead_end_conclusion->attempted_direction,
                          plan.dead_end_conclusion->validated_through_revision);
    result.dead_end_recorded = true;
  }
  active_plan_ = plan;
  result.active_route_nodes = plan.route_nodes.size();
  result.accepted = true;
  return result;
}

bool IncrementalTopologicalNavigation3D::invalidateAcceptedPlan(
    const IncrementalTopologicalPlan3D& plan) {
  const std::scoped_lock lock{memory_mutex_};
  if (!active_plan_.has_value() || !samePlanIdentity(*active_plan_, plan)) {
    return false;
  }
  active_plan_.reset();
  return true;
}

void IncrementalTopologicalNavigation3D::rejectObservationFrontier(
    const ObservationFrontierId frontier_id) {
  if (frontier_id.value == 0U) {
    return;
  }
  const std::scoped_lock lock{memory_mutex_};
  // Rejection remains evidence about this proposal, not a prohibited region.
  memory_.recordFrontierSelection(frontier_id);
  if (active_plan_.has_value() && active_plan_->selected_frontier.has_value() &&
      active_plan_->selected_frontier->id == frontier_id) {
    active_plan_.reset();
  }
}

void IncrementalTopologicalNavigation3D::completeObservationFrontier(
    const ObservationFrontier& frontier, const std::uint64_t revision) {
  if (frontier.id.value == 0U) {
    return;
  }
  const std::scoped_lock lock{memory_mutex_};
  memory_.recordObserved(frontier.observation_pose, revision);
  memory_.recordFrontierSelection(frontier.id);
  memory_.recordFrontierCompletion(frontier.id);
  if (active_plan_.has_value() && active_plan_->selected_frontier.has_value() &&
      active_plan_->selected_frontier->id == frontier.id) {
    active_plan_.reset();
  }
}

void IncrementalTopologicalNavigation3D::beginMissionLeg() {
  const std::scoped_lock lock{memory_mutex_};
  memory_.beginMissionLeg();
  if (current_node_.has_value()) {
    memory_.resetTrail(*current_node_);
  }
  active_plan_.reset();
}

} // namespace drone_city_nav
