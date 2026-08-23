#include "drone_city_nav/incremental_topological_lattice_adapter_3d.hpp"
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

} // namespace

bool incrementalTopologicalNavigation3DConfigIsValid(
    const IncrementalTopologicalNavigation3DConfig& config) noexcept {
  return std::isfinite(config.active_route_completion_tolerance_m) &&
         config.active_route_completion_tolerance_m >= 0.0 &&
         std::isfinite(config.maximum_active_route_cross_track_m) &&
         config.maximum_active_route_cross_track_m > 0.0;
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
  double minimum_station_m{0.0};
  {
    const std::scoped_lock lock{memory_mutex_};
    const StrategicRoutePlan3D* const active = strategic_route_manager_.active();
    if (active != nullptr) {
      candidate = active->plan;
      minimum_station_m = active->cursor.station_m;
    }
  }
  if (!candidate.has_value()) {
    return std::nullopt;
  }

  const std::optional<TopologicalPolylineProjection3D> projection =
      projectOntoTopologicalPolyline3D(candidate->guidance_points, start,
                                       minimum_station_m);
  bool continuable =
      candidate->executableTargetSelected() && finitePoint(start) &&
      finitePoint(mission_goal) && projection.has_value() &&
      distance3D(candidate->mission_target, mission_goal) <=
          kMissionTargetIdentityToleranceM &&
      projection->remaining_m >
          navigation_config_.active_route_completion_tolerance_m &&
      projection->distance_m <= navigation_config_.maximum_active_route_cross_track_m;
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
    const std::scoped_lock lock{memory_mutex_};
    if (!strategic_route_manager_.advance(*candidate, projection->station_m,
                                          projection->segment_index)) {
      return std::nullopt;
    }
    candidate->continued_from_active_plan = true;
    return candidate;
  }

  const std::scoped_lock lock{memory_mutex_};
  static_cast<void>(strategic_route_manager_.invalidate(*candidate));
  return std::nullopt;
}

std::optional<IncrementalTopologicalLatticeDirective3D>
IncrementalTopologicalNavigation3D::makeLatticeDirective(
    const IncrementalTopologicalPlan3D& plan, const Point3& position,
    const IncrementalTopologicalLatticeAdapter3DConfig& config) {
  double minimum_station_m{0.0};
  {
    const std::scoped_lock lock{memory_mutex_};
    const StrategicRoutePlan3D* const active = strategic_route_manager_.active();
    if (active != nullptr && sameStrategicRoutePlan3D(active->plan, plan)) {
      minimum_station_m = active->cursor.station_m;
    }
  }

  std::optional<IncrementalTopologicalLatticeDirective3D> directive =
      makeIncrementalTopologicalLatticeDirective3D(plan, position, config,
                                                   minimum_station_m);
  if (!directive.has_value()) {
    return std::nullopt;
  }

  const std::scoped_lock lock{memory_mutex_};
  static_cast<void>(strategic_route_manager_.advance(plan, directive->source_station_m,
                                                     directive->source_segment_index));
  return directive;
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
  IncrementalTopologicalPlan3D result =
      planner_.plan(*graph, start, mission_goal, memory_snapshot);
  {
    const std::scoped_lock lock{memory_mutex_};
    result.strategic_plan_id = strategic_route_manager_.previewPlanId(result);
  }
  return result;
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
  IncrementalTopologicalPlan3D result =
      planner_.planObserved(*graph, occupancy, observability_, start, mission_goal,
                            memory_snapshot, std::nullopt);
  {
    const std::scoped_lock lock{memory_mutex_};
    result.strategic_plan_id = strategic_route_manager_.previewPlanId(result);
  }
  return result;
}

IncrementalTopologicalPlanCommit3D
IncrementalTopologicalNavigation3D::commitAcceptedPlan(
    const IncrementalTopologicalPlan3D& plan) {
  IncrementalTopologicalPlanCommit3D result{.graph_revision = plan.planned_on_revision};
  if (!plan.executableTargetSelected()) {
    return result;
  }
  const std::scoped_lock lock{memory_mutex_};
  const StrategicRoutePlan3D* const active_route = strategic_route_manager_.active();
  const IncrementalTopologicalPlan3D* const active_plan =
      active_route != nullptr ? std::addressof(active_route->plan) : nullptr;
  const std::optional<Point3> replaced_frontier_observation_pose = [&]() {
    if (active_plan == nullptr || !active_plan->selected_frontier.has_value() ||
        !plan.selected_frontier.has_value()) {
      return std::optional<Point3>{};
    }
    if (active_plan->selected_frontier->id != plan.selected_frontier->id) {
      return std::optional<Point3>{active_plan->selected_frontier->observation_pose};
    }
    if (distance3D(active_plan->selected_frontier->observation_pose,
                   plan.selected_frontier->observation_pose) <=
        memory_.config().coverage_resolution_m) {
      return std::optional<Point3>{};
    }
    return std::optional<Point3>{active_plan->selected_frontier->observation_pose};
  }();
  if (replaced_frontier_observation_pose.has_value()) {
    memory_.recordObserved(*replaced_frontier_observation_pose,
                           plan.validated_through_revision);
    result.replaced_frontier_coverage_recorded = true;
  }
  const bool selected_frontier_already_active = [&]() {
    if (active_plan == nullptr || !active_plan->selected_frontier.has_value() ||
        !plan.selected_frontier.has_value() ||
        active_plan->selected_frontier->id != plan.selected_frontier->id) {
      return false;
    }
    return distance3D(active_plan->selected_frontier->observation_pose,
                      plan.selected_frontier->observation_pose) <=
           memory_.config().coverage_resolution_m;
  }();
  if (plan.selected_frontier.has_value() && !selected_frontier_already_active) {
    memory_.recordFrontierSelection(plan.selected_frontier->id);
    result.frontier_selection_recorded = true;
  }
  const bool dead_end_already_active =
      active_plan != nullptr && active_plan->dead_end_conclusion.has_value() &&
      plan.dead_end_conclusion.has_value() &&
      active_plan->dead_end_conclusion->attempted_direction ==
          plan.dead_end_conclusion->attempted_direction &&
      active_plan->dead_end_conclusion->validated_through_revision ==
          plan.dead_end_conclusion->validated_through_revision;
  if (plan.dead_end_conclusion.has_value() && !dead_end_already_active) {
    memory_.recordDeadEnd(plan.dead_end_conclusion->attempted_direction,
                          plan.dead_end_conclusion->validated_through_revision);
    result.dead_end_recorded = true;
  }
  const StrategicRouteCommit3D strategic_commit = strategic_route_manager_.accept(plan);
  result.strategic_plan_id = strategic_commit.plan_id;
  result.active_route_nodes = plan.route_nodes.size();
  result.accepted = strategic_commit.accepted;
  return result;
}

bool IncrementalTopologicalNavigation3D::invalidateAcceptedPlan(
    const IncrementalTopologicalPlan3D& plan) {
  const std::scoped_lock lock{memory_mutex_};
  return strategic_route_manager_.invalidate(plan);
}

bool IncrementalTopologicalNavigation3D::supersedeAcceptedPlan() {
  const std::scoped_lock lock{memory_mutex_};
  return strategic_route_manager_.supersede();
}

void IncrementalTopologicalNavigation3D::rejectObservationFrontier(
    const ObservationFrontierId frontier_id) {
  if (frontier_id.value == 0U) {
    return;
  }
  const std::scoped_lock lock{memory_mutex_};
  // Rejection remains evidence about this proposal, not a prohibited region.
  memory_.recordFrontierSelection(frontier_id);
  const StrategicRoutePlan3D* const active = strategic_route_manager_.active();
  if (active != nullptr && active->plan.selected_frontier.has_value() &&
      active->plan.selected_frontier->id == frontier_id) {
    static_cast<void>(strategic_route_manager_.supersede());
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
  const StrategicRoutePlan3D* const active = strategic_route_manager_.active();
  if (active != nullptr && active->plan.selected_frontier.has_value() &&
      active->plan.selected_frontier->id == frontier.id) {
    static_cast<void>(strategic_route_manager_.supersede());
  }
}

void IncrementalTopologicalNavigation3D::beginMissionLeg() {
  const std::scoped_lock lock{memory_mutex_};
  memory_.beginMissionLeg();
  if (current_node_.has_value()) {
    memory_.resetTrail(*current_node_);
  }
  strategic_route_manager_.reset();
}

} // namespace drone_city_nav
