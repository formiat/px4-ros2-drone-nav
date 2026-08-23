#include "production_mppi_route_materialization.hpp"

#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"

#include <chrono>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "production_mppi_route_helpers.hpp"

namespace drone_city_nav {

ProductionRouteMaterialization3D ProductionMppiNode::materializeRouteCandidate3D(
    const ProductionMppiPreparedEsdf& world, const ProductionMppiNavigation& navigation,
    const Point3& mission_goal, const ProductionRouteSearchCandidate3D& candidate,
    const std::uint64_t candidate_generation,
    const bool active_observation_segment_completed) {
  const Point3 search_start{navigation.state.x, navigation.state.y, navigation.state.z};
  const RiskAwareLattice3DResult& lattice = candidate.lattice;
  ProductionRouteMaterialization3D result;
  ProductionMppiPreparedEsdf& prepared = result.prepared;
  prepared = world;
  prepared.passage_traversals.reset();
  prepared.route_intent = candidate.intent;
  prepared.route_segment_evidence = candidate.evidence;
  prepared.planning_search_kind = ProductionPlanningSearchKind::kLattice3D;
  prepared.planning_search_start = search_start;
  prepared.planning_search_goal = lattice.planning_goal;
  prepared.planning_candidate_endpoint =
      lattice.points.empty() ? search_start : lattice.points.back();
  prepared.planning_search_direction = candidate.directive.preferred_direction;
  prepared.planning_candidate_points = lattice.points.size();
  prepared.planning_candidate_samples = lattice.route.size();
  prepared.lattice_search_performed = true;
  prepared.lattice_executable =
      lattice.status == Lattice3DStatus::kReachedPlanningGoal ||
      lattice.status == Lattice3DStatus::kViableFrontier;
  prepared.global_guide_expansions = lattice.expansions;
  prepared.lattice_3d_status = lattice.status;
  prepared.lattice_3d_risk_stage = lattice.risk_stage;
  prepared.lattice_3d_termination = lattice.termination;
  prepared.lattice_3d_route_purpose = lattice.route_purpose;
  prepared.lattice_3d_observation_frontier = lattice.observation_frontier;
  prepared.lattice_3d_minimum_clearance_m = lattice.minimum_clearance_m;
  prepared.lattice_3d_successor_diagnostics = lattice.successor_diagnostics;
  prepared.lattice_3d_successor_profiling = lattice.successor_profiling;
  prepared.lattice_search_session_complete =
      lattice.status != Lattice3DStatus::kSearchIncomplete;
  prepared.lattice_search_revision = world.revision;
  prepared.lattice_validation_revision = world.revision;
  prepared.lattice_planning_goal_reached =
      lattice.status == Lattice3DStatus::kReachedPlanningGoal;
  prepared.lattice_achieved_progress_m = lattice.achieved_progress_m;
  prepared.lattice_guide_length_m = lattice.route_length_m;
  prepared.lattice_remaining_goal_distance_m =
      distance3D(prepared.planning_candidate_endpoint, lattice.planning_goal);
  prepared.lattice_terminal_successor_count = lattice.terminal_successor_count;
  prepared.lattice_stale_queue_pops = lattice.stale_queue_pops;
  prepared.lattice_open_peak = lattice.open_peak;
  prepared.lattice_records_peak = lattice.records_peak;
  prepared.lattice_continuation_reachable_states =
      lattice.continuation_reachable_states;
  prepared.lattice_reachable_depth_m = lattice.continuation_reachable_depth_m;
  prepared.lattice_frontier_endpoint_displacement_m =
      lattice.frontier_endpoint_displacement_m;
  prepared.lattice_frontier_selection_score = lattice.frontier_selection_score;
  prepared.lattice_frontier_candidates_considered =
      lattice.frontier_candidates_considered;
  prepared.lattice_frontier_sampled_free_voxels = lattice.frontier_sampled_free_voxels;
  prepared.lattice_frontier_boundary_candidates = lattice.frontier_boundary_candidates;
  prepared.lattice_frontier_evaluated_candidates =
      lattice.frontier_evaluated_candidates;
  prepared.lattice_frontier_searches = lattice.frontier_searches;
  prepared.lattice_frontier_evaluation_budget_exhausted =
      lattice.frontier_evaluation_budget_exhausted;
  prepared.global_guide_cost = lattice.objective_cost;
  prepared.global_guide_reaches_mission_goal = lattice.reached_mission_goal;
  prepared.topology_candidates = lattice.topology_candidates;
  prepared.topology_objective_cost = lattice.objective_cost;
  prepared.topology_route_length_m = lattice.route_length_m;
  prepared.topology_travel_time_s = lattice.estimated_travel_time_s;
  prepared.topology_vertical_alignment_time_s = lattice.vertical_alignment_time_s;
  prepared.topology_planning_exposure_m = lattice.planning_exposure_m;
  prepared.topology_critical_exposure_m = lattice.critical_exposure_m;
  prepared.continuation_validation_ms = lattice.continuation_validation_ms;
  prepared.route_fingerprint = lattice.route_fingerprint;

  std::vector<SelectedPassageTraversal> route_traversals =
      lattice.selected_passage_traversals;
  if (world.static_route_replan_request) {
    result.replacement_policy = StaticRouteReplacementPolicy::kAllowSafetyReplan;
  } else if (world.static_route_extension_request ||
             lattice.route_purpose != world.lattice_3d_route_purpose ||
             lattice.route_purpose == Lattice3DRoutePurpose::kObservationFrontier) {
    result.replacement_policy = StaticRouteReplacementPolicy::kAllowTopologicalProgress;
  }

  bool active_observation_frontier_still_valid = false;
  if (world.observed_occupancy && world.lattice_3d_observation_frontier) {
    const ObservationFrontierSetEvaluation active_evaluation =
        evaluateObservationFrontiers(
            *world.observed_occupancy,
            world.lattice_3d_observation_frontier->observation_pose, world.revision,
            lattice_3d_config_.sensor_observability,
            lattice_3d_config_.require_known_free_space
                ? ObservedSpaceValidationPolicy::kRequireKnownFree
                : ObservedSpaceValidationPolicy::kAllowUnknown);
    active_observation_frontier_still_valid = !active_evaluation.frontiers.empty();
  }
  if (lattice.route_purpose == Lattice3DRoutePurpose::kObservationFrontier) {
    const bool active_observation_frontier_reached =
        world.lattice_3d_observation_frontier.has_value() &&
        distance3D(search_start,
                   world.lattice_3d_observation_frontier->observation_pose) <=
            lattice_3d_config_.goal_tolerance_m;
    double observation_endpoint_improvement_m = 0.0;
    if (world.lattice_3d_observation_frontier && lattice.observation_frontier) {
      observation_endpoint_improvement_m =
          distance3D(world.lattice_3d_observation_frontier->observation_pose,
                     mission_goal) -
          distance3D(lattice.observation_frontier->observation_pose, mission_goal);
    }
    result.observation_replacement =
        evaluateObservationRouteReplacement(ObservationRouteReplacementObservation{
            .active_frontier = world.lattice_3d_observation_frontier,
            .candidate_frontier = lattice.observation_frontier,
            .active_score = world.lattice_frontier_selection_score,
            .candidate_score = lattice.frontier_selection_score,
            .minimum_score_improvement =
                lattice_3d_config_
                    .observation_frontier_replacement_minimum_score_improvement,
            .endpoint_improvement_m = observation_endpoint_improvement_m,
            .minimum_endpoint_improvement_m =
                lattice_3d_config_
                    .observation_frontier_replacement_minimum_endpoint_improvement_m,
            .active_frontier_still_valid = active_observation_frontier_still_valid,
            .active_frontier_reached = active_observation_frontier_reached,
            .active_route_exhausted = active_observation_segment_completed,
            .route_extension_requested = world.static_route_extension_request,
        });
  }
  prepared.observation_route_replacement_status = result.observation_replacement.status;

  result.validation =
      StaticRouteCandidateValidation{.status = StaticRouteCandidateStatus::kEmpty};
  if (!prepared.lattice_executable) {
    return result;
  }

  const auto validation_started = std::chrono::steady_clock::now();
  auto mutable_route = std::make_shared<std::vector<RouteSample3D>>(lattice.route);
  std::vector<ConstrainedRouteSpan> initial_spans = makeConstrainedRouteSpans(
      *mutable_route, route_traversals, candidate_generation, route_envelope_config_);
  const auto smoothing_started = std::chrono::steady_clock::now();
  StaticRouteGeometryResult geometry = optimizeStaticRouteGeometry(
      *mutable_route, initial_spans, world.grid, *world.distances_m,
      SweptFootprintConfig{
          .radius_m = lattice_3d_config_.physical_footprint_radius_m,
          .lower_extent_m = lattice_3d_config_.physical_footprint_lower_extent_m,
          .upper_extent_m = lattice_3d_config_.physical_footprint_upper_extent_m,
          .perimeter_samples = physical_footprint_config_.perimeter_samples,
          .radial_rings = physical_footprint_config_.radial_rings,
          .axial_samples = physical_footprint_config_.axial_samples,
          .sweep_step_m = physical_footprint_config_.sweep_step_m},
      static_route_geometry_config_, route_envelope_config_,
      planning_worker_pool_.get());
  prepared.route_smoothing_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                smoothing_started)
          .count();
  prepared.route_shortcuts_applied = geometry.shortcuts_applied;
  prepared.route_corners_smoothed = geometry.corners_smoothed;
  prepared.route_shortcut_candidates = geometry.shortcut_candidates;
  prepared.route_parallel_shortcut_candidates = geometry.parallel_shortcut_candidates;
  prepared.route_corner_candidates = geometry.corner_candidates;
  prepared.route_parallel_corner_candidates = geometry.parallel_corner_candidates;
  prepared.route_shortcut_validation_ms = geometry.shortcut_validation_ms;
  prepared.route_corner_validation_ms = geometry.corner_validation_ms;
  if (geometry.route.size() >= 2U) {
    *mutable_route = std::move(geometry.route);
  }

  const SweptFootprintConfig footprint_config{
      .radius_m = lattice_3d_config_.physical_footprint_radius_m,
      .lower_extent_m = lattice_3d_config_.physical_footprint_lower_extent_m,
      .upper_extent_m = lattice_3d_config_.physical_footprint_upper_extent_m,
      .perimeter_samples = physical_footprint_config_.perimeter_samples,
      .radial_rings = physical_footprint_config_.radial_rings,
      .axial_samples = physical_footprint_config_.axial_samples,
      .sweep_step_m = physical_footprint_config_.sweep_step_m};
  const RouteRiskTierAssignmentResult optimized_risk_assignment = assignRouteRiskTiers(
      *mutable_route, world.grid, *world.distances_m,
      mppi_config_.risk.critical_distance_m, mppi_config_.risk.preferred_distance_m,
      lattice_3d_config_.require_known_free_space);
  if (!optimized_risk_assignment.accepted()) {
    *mutable_route = lattice.route;
    geometry.constrained_spans = initial_spans;
    prepared.route_shortcuts_applied = 0U;
    prepared.route_corners_smoothed = 0U;
    const std::string_view optimization_failure =
        routeRiskTierAssignmentStatusName(optimized_risk_assignment.status);
    RCLCPP_INFO(get_logger(),
                "STATIC_ROUTE_GEOMETRY status=fallback_to_lattice reason=%.*s "
                "failure=(%.2f,%.2f,%.2f)",
                static_cast<int>(optimization_failure.size()),
                optimization_failure.data(), optimized_risk_assignment.failure_point.x,
                optimized_risk_assignment.failure_point.y,
                optimized_risk_assignment.failure_point.z);
  }
  for (ConstrainedRouteSpan& span : geometry.constrained_spans) {
    span.route_generation = candidate_generation;
  }

  std::vector<CooperativePassageAssignment> passage_assignments;
  std::shared_ptr<const std::vector<PassageVolume>> passage_volumes;
  bool cooperative_route_valid = true;
  const bool passage_geometry_required =
      static_occupancy_3d_.has_value() && !geometry.constrained_spans.empty() &&
      (cooperative_traffic_enabled_ || static_free_space_topology_router_ != nullptr);
  if (passage_geometry_required) {
    const auto passage_started = std::chrono::steady_clock::now();
    PassageVolumeResource volume_resource = acquireDerivedPassageVolumes(
        *mutable_route, geometry.constrained_spans, *static_occupancy_3d_,
        cooperative_passage_volume_config_);
    prepared.passage_volume_build_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  passage_started)
            .count();
    prepared.passage_volume_resource_reused = volume_resource.shared_resource_reused;
    passage_volumes = std::move(volume_resource.volumes);
    const std::span<const PassageVolume> volumes =
        passage_volumes ? std::span<const PassageVolume>{*passage_volumes}
                        : std::span<const PassageVolume>{};
    static_cast<void>(
        projectPassageVolumeEnvelopes(geometry.constrained_spans, volumes,
                                      cooperative_passage_volume_config_.footprint));
    if (cooperative_traffic_enabled_) {
      CooperativePassageRouteResult cooperative_route =
          applyCooperativePassageCorridors(*mutable_route, geometry.constrained_spans,
                                           volumes, *static_occupancy_3d_,
                                           cooperative_passage_route_config_);
      cooperative_route_valid = cooperative_route.valid;
      if (cooperative_route.valid) {
        *mutable_route = std::move(cooperative_route.route);
        geometry.constrained_spans = std::move(cooperative_route.constrained_spans);
      }
      passage_assignments = std::move(cooperative_route.assignments);
    }
  }

  auto mutable_spans = std::make_shared<std::vector<ConstrainedRouteSpan>>(
      std::move(geometry.constrained_spans));
  if (!cooperative_route_valid) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
  } else if (const RouteRiskTierAssignmentResult risk_assignment =
                 assignRouteRiskTiers(*mutable_route, world.grid, *world.distances_m,
                                      mppi_config_.risk.critical_distance_m,
                                      mppi_config_.risk.preferred_distance_m,
                                      lattice_3d_config_.require_known_free_space);
             risk_assignment.accepted()) {
    result.validation = validateStaticRouteCandidate(
        world.route_3d ? std::span<const RouteSample3D>{*world.route_3d}
                       : std::span<const RouteSample3D>{},
        *mutable_route, world.grid, *world.distances_m, mission_goal,
        static_route_extension_config_.minimum_endpoint_improvement_m,
        lattice.reached_mission_goal, lattice_3d_config_.flight_envelope,
        result.replacement_policy, footprint_config);
  } else {
    StaticRouteCandidateStatus candidate_status =
        StaticRouteCandidateStatus::kInvalidEsdf;
    if (risk_assignment.status == RouteRiskTierAssignmentStatus::kRawCollision) {
      candidate_status = StaticRouteCandidateStatus::kRawCollision;
    } else if (risk_assignment.status == RouteRiskTierAssignmentStatus::kOutsideGrid ||
               risk_assignment.status == RouteRiskTierAssignmentStatus::kUnknownSpace) {
      candidate_status = StaticRouteCandidateStatus::kOutsideEsdf;
    }
    result.validation = StaticRouteCandidateValidation{
        .status = candidate_status,
        .failure_segment_index = risk_assignment.failure_sample_index,
        .failure_point = risk_assignment.failure_point};
  }

  const std::shared_ptr<const std::vector<RouteSample3D>> route = mutable_route;
  const std::shared_ptr<const std::vector<ConstrainedRouteSpan>> spans = mutable_spans;
  if (result.validation.accepted && spans->size() != initial_spans.size()) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
  }
  if (result.validation.accepted &&
      lattice.route_purpose == Lattice3DRoutePurpose::kObservationFrontier &&
      !result.observation_replacement.accepted) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kNoExplorationProgress,
        .endpoint_improvement_m = result.validation.endpoint_improvement_m,
    };
  }
  if (result.validation.accepted &&
      !validateConstrainedRouteSpans(*route, *spans, world.grid, *world.distances_m)) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
  }
  const bool protected_suffix =
      world.route_3d && world.constrained_spans &&
      staticRouteReplacementProtected(
          *world.route_3d, *world.constrained_spans, search_start,
          world.route_objective, world.search_objective,
          static_route_extension_config_.protected_departure_m);
  if (result.validation.accepted && protected_suffix) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kProtectedConstrainedSuffix};
  }

  std::vector<PassageTraversalId> selected_passage_traversal_ids;
  selected_passage_traversal_ids.reserve(route_traversals.size());
  for (const SelectedPassageTraversal& traversal : route_traversals) {
    selected_passage_traversal_ids.push_back(traversal.passage_traversal_id);
  }
  prepared.route_3d = route;
  prepared.route_2d_projection = projectRouteTo2D(*route);
  prepared.constrained_spans = spans;
  prepared.passage_volumes = std::move(passage_volumes);
  prepared.cooperative_passage_assignments =
      std::make_shared<const std::vector<CooperativePassageAssignment>>(
          std::move(passage_assignments));
  prepared.selected_passage_traversal_ids =
      std::make_shared<const std::vector<PassageTraversalId>>(
          std::move(selected_passage_traversal_ids));
  prepared.mppi_route =
      makeMppiRoute3D(*route, *spans, speed_policy_config_.cruise_speed_mps,
                      constrained_route_speed_limit_mps_, speed_policy_config_);
  prepared.global_guide_projection = projectOntoGlobalGuide(
      *prepared.route_2d_projection, Point2{navigation.state.x, navigation.state.y});
  prepared.route_fingerprint = routeFingerprint(*route, route_traversals);
  prepared.candidate_validation_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                validation_started)
          .count();
  return result;
}

} // namespace drone_city_nav
