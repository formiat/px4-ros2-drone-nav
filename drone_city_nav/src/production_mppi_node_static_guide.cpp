#include "drone_city_nav/mppi/static_route_handoff.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "production_mppi_node.hpp"
#include "production_mppi_route_helpers.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {

void ProductionMppiNode::processGuideSearch3D(
    const ProductionMppiPreparedEsdf& world,
    const ProductionMppiNavigation& navigation) {
  const Point3 mission_goal =
      world.search_objective.available ? world.search_objective.goal : mission_goal_;
  const std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world_3d =
      latest_raw_world_3d_.load(std::memory_order_acquire);
  const ObservedOccupancyGrid3D* const latest_observed_occupancy =
      latest_raw_world_3d && latest_raw_world_3d->occupancy
          ? latest_raw_world_3d->occupancy.get()
          : nullptr;
  const Point3 search_start{navigation.state.x, navigation.state.y, navigation.state.z};
  const double completed_observation_tolerance_m =
      std::min(lattice_3d_config_.goal_tolerance_m,
               0.5 * std::min(lattice_3d_config_.horizontal_step_m,
                              lattice_3d_config_.vertical_step_m));
  if (topological_navigation_3d_ &&
      world.lattice_3d_route_purpose == Lattice3DRoutePurpose::kObservationFrontier &&
      world.lattice_3d_observation_frontier && world.route_3d &&
      !world.route_3d->empty() &&
      distance3D(search_start, world.route_3d->back().position) <=
          completed_observation_tolerance_m) {
    if (world.route_intent.valid && world.route_intent.segment_reaches_intent_target) {
      topological_navigation_3d_->completeObservationFrontier(
          *world.lattice_3d_observation_frontier, world.revision);
      RCLCPP_INFO(get_logger(),
                  "INCREMENTAL_TOPOLOGY3D_FRONTIER_COMPLETED frontier_id=%" PRIu64
                  " route_endpoint_reached=true intent_id=%" PRIu64,
                  world.lattice_3d_observation_frontier->id.value,
                  world.route_intent.id);
    } else {
      RCLCPP_INFO(get_logger(),
                  "INCREMENTAL_TOPOLOGY3D_SEGMENT_COMPLETED frontier_id=%" PRIu64
                  " intent_id=%" PRIu64 " intent_target_reached=false",
                  world.lattice_3d_observation_frontier->id.value,
                  world.route_intent.id);
    }
    requestGuideRelease(GlobalGuideReleaseReason::kExhausted,
                        world.global_guide_generation);
  }

  ProductionRouteCandidateSelection3D route_selection =
      selectRouteCandidate3D(world, navigation, mission_goal, latest_raw_world_3d);
  RiskAwareLattice3DResult& lattice = route_selection.lattice;
  ProductionIncrementalTopologySearch3D& topology = route_selection.topology;
  const std::optional<Lattice3DStrategicDirective>& strategic_directive =
      route_selection.directive;
  const Vec3 preferred_direction = route_selection.preferred_direction;
  const bool topology_route_used = route_selection.topology_route_used;
  const double search_ms = route_selection.search_ms;
  ProductionMppiPreparedEsdf prepared = world;
  prepared.passage_traversals.reset();
  prepared.route_intent = route_selection.intent;
  prepared.route_segment_evidence = route_selection.evidence;
  prepared.route_proposal_selection_reason = route_selection.proposal_selection.reason;
  prepared.route_proposal_candidate_count =
      route_selection.proposal_selection.considered_candidates;
  prepared.route_proposal_eligible_count =
      route_selection.proposal_selection.eligible_candidates;
  prepared.global_guide_search_ms = search_ms;
  prepared.planning_search_kind = ProductionPlanningSearchKind::kLattice3D;
  prepared.planning_search_start = search_start;
  prepared.planning_search_goal = lattice.planning_goal;
  prepared.planning_candidate_endpoint =
      lattice.points.empty() ? search_start : lattice.points.back();
  prepared.planning_search_direction = preferred_direction;
  prepared.planning_candidate_points = lattice.points.size();
  prepared.planning_candidate_samples = lattice.route.size();
  prepared.lattice_search_performed = strategic_directive.has_value();
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
  StaticRouteReplacementPolicy replacement_policy =
      StaticRouteReplacementPolicy::kRequireEndpointImprovement;
  if (world.static_route_replan_request) {
    replacement_policy = StaticRouteReplacementPolicy::kAllowSafetyReplan;
  } else if (strategic_directive.has_value() &&
             (world.static_route_extension_request ||
              lattice.route_purpose != world.lattice_3d_route_purpose ||
              lattice.route_purpose == Lattice3DRoutePurpose::kObservationFrontier)) {
    replacement_policy = StaticRouteReplacementPolicy::kAllowTopologicalProgress;
  }
  bool active_observation_frontier_still_valid = false;
  if (world.observed_occupancy && world.lattice_3d_observation_frontier) {
    const ObservationFrontierSetEvaluation active_evaluation =
        evaluateObservationFrontiers(
            *world.observed_occupancy,
            world.lattice_3d_observation_frontier->observation_pose, world.revision,
            lattice_3d_config_.sensor_observability);
    // A boundary voxel can change identity while the observation pose remains
    // known-free and still reveals unknown space. Keep executing that useful
    // finite route until it is reached or ceases to expose any frontier.
    active_observation_frontier_still_valid = !active_evaluation.frontiers.empty();
  }
  ObservationRouteReplacementDecision observation_replacement;
  if (lattice.route_purpose == Lattice3DRoutePurpose::kObservationFrontier) {
    const double active_route_completion_tolerance_m =
        std::min(lattice_3d_config_.goal_tolerance_m,
                 0.5 * std::min(lattice_3d_config_.horizontal_step_m,
                                lattice_3d_config_.vertical_step_m));
    const bool active_observation_frontier_reached =
        world.lattice_3d_observation_frontier.has_value() &&
        distance3D(search_start,
                   world.lattice_3d_observation_frontier->observation_pose) <=
            lattice_3d_config_.goal_tolerance_m;
    const bool active_observation_route_exhausted =
        world.lattice_3d_route_purpose == Lattice3DRoutePurpose::kObservationFrontier &&
        world.route_3d && !world.route_3d->empty() &&
        distance3D(search_start, world.route_3d->back().position) <=
            active_route_completion_tolerance_m;
    double observation_endpoint_improvement_m = 0.0;
    if (world.lattice_3d_observation_frontier && lattice.observation_frontier) {
      observation_endpoint_improvement_m =
          distance3D(world.lattice_3d_observation_frontier->observation_pose,
                     mission_goal) -
          distance3D(lattice.observation_frontier->observation_pose, mission_goal);
    }
    observation_replacement =
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
            .active_route_exhausted = active_observation_route_exhausted,
            .route_extension_requested = world.static_route_extension_request,
        });
  }
  prepared.observation_route_replacement_status = observation_replacement.status;

  StaticRouteCandidateValidation validation{.status =
                                                StaticRouteCandidateStatus::kEmpty};
  if (prepared.lattice_executable) {
    const auto validation_started = std::chrono::steady_clock::now();
    auto mutable_route = std::make_shared<std::vector<RouteSample3D>>(lattice.route);
    const std::uint64_t candidate_generation = static_route_generation_ + 1U;
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

    // Geometry optimization is optional. It must not discard an executable
    // lattice route merely because a shortcut becomes unsafe after a fresh
    // observed-world update.
    const SweptFootprintConfig footprint_config{
        .radius_m = lattice_3d_config_.physical_footprint_radius_m,
        .lower_extent_m = lattice_3d_config_.physical_footprint_lower_extent_m,
        .upper_extent_m = lattice_3d_config_.physical_footprint_upper_extent_m,
        .perimeter_samples = physical_footprint_config_.perimeter_samples,
        .radial_rings = physical_footprint_config_.radial_rings,
        .axial_samples = physical_footprint_config_.axial_samples,
        .sweep_step_m = physical_footprint_config_.sweep_step_m};
    const auto optimized_risk_assignment = assignRouteRiskTiers(
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
                  optimization_failure.data(),
                  optimized_risk_assignment.failure_point.x,
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
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
    } else if (const RouteRiskTierAssignmentResult risk_assignment =
                   assignRouteRiskTiers(*mutable_route, world.grid, *world.distances_m,
                                        mppi_config_.risk.critical_distance_m,
                                        mppi_config_.risk.preferred_distance_m,
                                        lattice_3d_config_.require_known_free_space);
               risk_assignment.accepted()) {
      validation = validateStaticRouteCandidate(
          world.route_3d ? std::span<const RouteSample3D>{*world.route_3d}
                         : std::span<const RouteSample3D>{},
          *mutable_route, world.grid, *world.distances_m, mission_goal,
          static_route_extension_config_.minimum_endpoint_improvement_m,
          lattice.reached_mission_goal, lattice_3d_config_.flight_envelope,
          replacement_policy, footprint_config);
    } else {
      validation = StaticRouteCandidateValidation{
          .status =
              risk_assignment.status == RouteRiskTierAssignmentStatus::kRawCollision
                  ? StaticRouteCandidateStatus::kRawCollision
              : risk_assignment.status == RouteRiskTierAssignmentStatus::kOutsideGrid ||
                      risk_assignment.status ==
                          RouteRiskTierAssignmentStatus::kUnknownSpace
                  ? StaticRouteCandidateStatus::kOutsideEsdf
                  : StaticRouteCandidateStatus::kInvalidEsdf,
          .failure_segment_index = risk_assignment.failure_sample_index,
          .failure_point = risk_assignment.failure_point};
    }
    if (validation.accepted && latest_observed_occupancy != nullptr) {
      if (const std::optional<StaticRouteCandidateValidation> latest_raw_validation =
              validateRouteAgainstLatestObservedRawOccupancy(
                  *mutable_route, *latest_observed_occupancy, footprint_config,
                  world.proprioceptive_free_space_seed
                      ? std::addressof(*world.proprioceptive_free_space_seed)
                      : nullptr,
                  world.launch_support_contact
                      ? std::addressof(*world.launch_support_contact)
                      : nullptr);
          latest_raw_validation.has_value()) {
        validation = *latest_raw_validation;
      }
    }
    const std::shared_ptr<const std::vector<RouteSample3D>> route = mutable_route;
    const std::shared_ptr<const std::vector<ConstrainedRouteSpan>> spans =
        mutable_spans;
    if (validation.accepted && spans->size() != initial_spans.size()) {
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
    }
    if (validation.accepted &&
        lattice.route_purpose == Lattice3DRoutePurpose::kObservationFrontier &&
        !observation_replacement.accepted) {
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kNoExplorationProgress,
          .endpoint_improvement_m = validation.endpoint_improvement_m,
      };
    }
    if (validation.accepted && !validateConstrainedRouteSpans(
                                   *route, *spans, world.grid, *world.distances_m)) {
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
    }
    const bool protected_suffix =
        world.route_3d && world.constrained_spans &&
        staticRouteReplacementProtected(
            *world.route_3d, *world.constrained_spans, search_start,
            world.route_objective, world.search_objective,
            static_route_extension_config_.protected_departure_m);
    if (validation.accepted && protected_suffix) {
      validation = StaticRouteCandidateValidation{
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
  }

  bool observed_world_rebased = false;
  std::optional<ProductionMppiPreparedEsdf> latest_observed_world;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    if (world.observed_occupancy && prepared_esdf_ &&
        prepared_esdf_->observed_occupancy && prepared_esdf_->distances_m &&
        prepared_esdf_->revision != prepared.revision) {
      latest_observed_world = *prepared_esdf_;
    }
  }
  if (validation.accepted && latest_observed_world && prepared.route_3d &&
      prepared.constrained_spans) {
    auto current_route =
        std::make_shared<std::vector<RouteSample3D>>(*prepared.route_3d);
    const RouteRiskTierAssignmentResult rebase_risk_assignment = assignRouteRiskTiers(
        *current_route, latest_observed_world->grid,
        *latest_observed_world->distances_m, mppi_config_.risk.critical_distance_m,
        mppi_config_.risk.preferred_distance_m,
        lattice_3d_config_.require_known_free_space);
    if (!rebase_risk_assignment.accepted()) {
      validation = StaticRouteCandidateValidation{
          .status = rebase_risk_assignment.status ==
                            RouteRiskTierAssignmentStatus::kRawCollision
                        ? StaticRouteCandidateStatus::kRawCollision
                    : rebase_risk_assignment.status ==
                                RouteRiskTierAssignmentStatus::kOutsideGrid ||
                            rebase_risk_assignment.status ==
                                RouteRiskTierAssignmentStatus::kUnknownSpace
                        ? StaticRouteCandidateStatus::kOutsideEsdf
                        : StaticRouteCandidateStatus::kInvalidEsdf,
          .failure_segment_index = rebase_risk_assignment.failure_sample_index,
          .failure_point = rebase_risk_assignment.failure_point};
    } else {
      validation = validateStaticRouteCandidate(
          latest_observed_world->route_3d
              ? std::span<const RouteSample3D>{*latest_observed_world->route_3d}
              : std::span<const RouteSample3D>{},
          *current_route, latest_observed_world->grid,
          *latest_observed_world->distances_m, mission_goal,
          static_route_extension_config_.minimum_endpoint_improvement_m,
          lattice.reached_mission_goal, lattice_3d_config_.flight_envelope,
          replacement_policy,
          SweptFootprintConfig{
              .radius_m = lattice_3d_config_.physical_footprint_radius_m,
              .lower_extent_m = lattice_3d_config_.physical_footprint_lower_extent_m,
              .upper_extent_m = lattice_3d_config_.physical_footprint_upper_extent_m,
              .perimeter_samples = physical_footprint_config_.perimeter_samples,
              .radial_rings = physical_footprint_config_.radial_rings,
              .axial_samples = physical_footprint_config_.axial_samples,
              .sweep_step_m = physical_footprint_config_.sweep_step_m});
    }
    if (validation.accepted &&
        !validateConstrainedRouteSpans(*current_route, *prepared.constrained_spans,
                                       latest_observed_world->grid,
                                       *latest_observed_world->distances_m)) {
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
    }
    if (validation.accepted) {
      adoptWorldResources(prepared, *latest_observed_world);
      prepared.route_3d = current_route;
      prepared.route_2d_projection = projectRouteTo2D(*current_route);
      prepared.mppi_route =
          makeMppiRoute3D(*current_route, *prepared.constrained_spans,
                          speed_policy_config_.cruise_speed_mps,
                          constrained_route_speed_limit_mps_, speed_policy_config_);
      prepared.global_guide_projection =
          projectOntoGlobalGuide(*prepared.route_2d_projection,
                                 Point2{navigation.state.x, navigation.state.y});
      observed_world_rebased = true;
    }
  }
  prepared.static_route_candidate_status = validation.status;
  StaticRouteActivationStatus activation_status =
      prepared.lattice_executable
          ? StaticRouteActivationStatus::kCandidateValidationRejected
          : StaticRouteActivationStatus::kCandidateNotExecutable;
  bool revision_matches = false;
  bool generation_matches = false;
  const std::shared_ptr<const ProductionNavigationObjective> activation_objective =
      navigationObjective();
  const std::uint64_t required_objective_epoch =
      minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
  const std::uint64_t required_objective_sample =
      activation_objective &&
              activation_objective->mission_epoch == required_objective_epoch
          ? minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire)
          : 0U;
  const bool objective_matches =
      activation_objective &&
      staticRouteObjectiveMatches(
          world.search_objective, makeStaticRouteObjective(*activation_objective),
          required_objective_sample, std::numeric_limits<double>::infinity());
  const StaticRouteCandidate route_candidate{
      .search_revision = prepared.revision,
      .base_route_generation = world.static_route_replan_request
                                   ? world.static_route_replan_base_generation
                                   : world.static_route_extension_base_generation,
      .candidate_route_generation = static_route_generation_ + 1U,
      .fingerprint = prepared.route_fingerprint,
      .executable = prepared.lattice_executable,
      .reaches_mission_goal = prepared.global_guide_reaches_mission_goal,
      .validation = validation,
      .route = prepared.route_3d,
      .constrained_spans = prepared.constrained_spans,
  };
  ProductionMppiNavigation handoff_navigation;
  ProductionMppiAppliedControl handoff_applied_control;
  {
    const std::scoped_lock input_lock{input_mutex_};
    handoff_navigation = navigation_;
    handoff_applied_control = applied_control_;
  }
  const std::int64_t handoff_now_ns = get_clock()->now().nanoseconds();
  const bool handoff_control_fresh =
      handoff_applied_control.valid && handoff_applied_control.receive_stamp_ns > 0 &&
      handoff_now_ns >= handoff_applied_control.receive_stamp_ns &&
      static_cast<double>(handoff_now_ns - handoff_applied_control.receive_stamp_ns) *
              1.0e-6 <=
          maximum_control_feedback_age_ms_;
  mppi::StaticRouteHandoffResult handoff;
  if (route_candidate.executable && route_candidate.validation.accepted &&
      prepared.mppi_route && handoff_navigation.valid) {
    handoff = mppi::validateStaticRouteHandoff(
        handoff_navigation.state,
        handoff_control_fresh ? handoff_applied_control.control : mppi::Control{},
        *prepared.mppi_route, static_cast<float>(speed_policy_config_.cruise_speed_mps),
        static_cast<float>(active_guide_config_.maximum_cross_track_m), mppi_config_,
        prepared.grid, *prepared.distances_m);
  }

  bool activated = false;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    const std::uint64_t required_base_generation =
        world.static_route_replan_request
            ? world.static_route_replan_base_generation
            : world.static_route_extension_base_generation;
    generation_matches =
        (!world.static_route_extension_request && !world.static_route_replan_request) ||
        (prepared_esdf_ &&
         prepared_esdf_->global_guide_generation == required_base_generation);
    revision_matches = prepared_esdf_ && prepared_esdf_->revision == prepared.revision;
    if (route_candidate.executable && route_candidate.validation.accepted &&
        handoff.accepted && route_candidate.route &&
        route_candidate.constrained_spans && revision_matches && generation_matches &&
        objective_matches) {
      activation_status = StaticRouteActivationStatus::kActivated;
      prepared.static_route_activation_status = activation_status;
      prepared.static_route_revision_matches = true;
      prepared.static_route_generation_matches = true;
      prepared.global_guide_generation = ++static_route_generation_;
      prepared.route_objective = world.search_objective;
      prepared.global_guide_release_reason = GlobalGuideReleaseReason::kNone;
      prepared.static_route_extension_request = false;
      prepared.static_route_extension_base_generation = 0U;
      prepared.static_route_replan_request = false;
      prepared.static_route_replan_base_generation = 0U;
      prepared.static_route_replan_reason = GlobalGuideReleaseReason::kNone;
      prepared_esdf_ = prepared;
      activated = true;
    } else if (route_candidate.validation.accepted && !revision_matches) {
      activation_status = StaticRouteActivationStatus::kStaleWorldRevision;
    } else if (route_candidate.validation.accepted && !generation_matches) {
      activation_status = StaticRouteActivationStatus::kStaleRouteGeneration;
    } else if (route_candidate.validation.accepted && !objective_matches) {
      activation_status = StaticRouteActivationStatus::kStaleObjective;
    } else if (route_candidate.validation.accepted && !handoff.accepted) {
      activation_status = StaticRouteActivationStatus::kDynamicHandoffRejected;
    }
  }
  if (activated && !use_static_map_) {
    const std::uint64_t blocked_raw_revision =
        observed_route_blocked_raw_revision_.load(std::memory_order_acquire);
    if (blocked_raw_revision != 0U &&
        blocked_raw_revision <= prepared.source_raw_revision) {
      observed_route_blocked_raw_revision_.store(0U, std::memory_order_release);
    }
  }
  if (activated && topology_route_used) {
    commitIncrementalTopologyRoute3D(topology);
  } else if (topology_route_used &&
             validation.status == StaticRouteCandidateStatus::kRawCollision &&
             topology.plan.selected_frontier.has_value() &&
             topological_navigation_3d_) {
    topological_navigation_3d_->rejectObservationFrontier(
        topology.plan.selected_frontier->id);
    RCLCPP_INFO(get_logger(),
                "INCREMENTAL_TOPOLOGY3D_FRONTIER_REJECTED reason=raw_collision "
                "frontier_id=%" PRIu64,
                topology.plan.selected_frontier->id.value);
  }
  if (activated && prepared.cooperative_passage_assignments) {
    for (const CooperativePassageAssignment& assignment :
         *prepared.cooperative_passage_assignments) {
      RCLCPP_INFO(
          get_logger(),
          "COOPERATIVE_PASSAGE_ROUTE route_generation=%" PRIu64
          " span_index=%zu passage='%s' offset_interval_m=[%.2f,%.2f] "
          "secondary_interval_m=[%.2f,%.2f] cross_sections=%zu "
          "raw_volume=%s requested_offset_m=%.2f applied_offset_m=%.2f "
          "status=%s volume_build_ms=%.2f volume_resource_reused=%s",
          assignment.route_generation, assignment.span_index,
          assignment.passage_traversal_id.c_str(), assignment.minimum_lateral_offset_m,
          assignment.maximum_lateral_offset_m, assignment.minimum_secondary_offset_m,
          assignment.maximum_secondary_offset_m, assignment.passage_cross_section_count,
          assignment.passage_volume_raw_validated ? "true" : "false",
          assignment.requested_lateral_offset_m, assignment.applied_lateral_offset_m,
          cooperativePassageRouteStatusName(assignment.status),
          prepared.passage_volume_build_ms,
          prepared.passage_volume_resource_reused ? "true" : "false");
    }
  }
  const char* const route_space =
      world.observed_occupancy ? "observed_occupancy_3d" : "static_occupancy_3d";
  const char* const topology_acceleration =
      world.topological_graph ? "incremental_topological_graph" : "unavailable";
  logIncrementalTopologyRoute3D(topology, lattice, validation, activation_status,
                                activated);
  const ObservationFrontier* const observation_frontier =
      lattice.observation_frontier ? &*lattice.observation_frontier : nullptr;
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_GUIDE3D revision=%" PRIu64
      " activated=%s activation_status=%.*s revision_matches=%s "
      "generation_matches=%s objective_matches=%s route_generation=%" PRIu64
      " route_space=%s observed_world_rebased=%s route_purpose=%s "
      "topology_acceleration=%s "
      "observation_frontier_id=%" PRIu64 " observation_frontier_revision=%" PRIu64
      " observation_frontier_rays=%zu observation_frontier_gain=%zu "
      "observation_frontier_score=%.3f frontier_candidates=%zu "
      "frontier_sampled_free=%zu frontier_boundary=%zu frontier_evaluated=%zu "
      "frontier_searches=%zu frontier_evaluation_budget_exhausted=%s "
      "observation_replacement=%.*s observation_score_improvement=%.3f "
      "extension=%s replan=%s "
      "base_generation=%" PRIu64 " replan_reason=%s"
      " replacement_policy=%.*s validation=%.*s endpoint_improvement_m=%.2f "
      "validation_failure_segment=%zu validation_failure=(%.2f,%.2f,%.2f) "
      "handoff=%s handoff_cross_track_m=%.2f handoff_minimum_clearance_m=%.2f "
      "handoff_planning_exposure_m=%.2f handoff_critical_exposure_m=%.2f "
      "status=%s termination=%s "
      "points=%zu samples=%zu spans=%zu expansions=%zu expansion_limit=%zu "
      "deadline_ms=%.2f "
      "risk_stage=%s start=(%.2f,%.2f,%.2f) planning_goal=(%.2f,%.2f,%.2f) "
      "endpoint=(%.2f,%.2f,%.2f) direction=(%.3f,%.3f,%.3f) "
      "achieved_progress_m=%.2f minimum_clearance_m=%.2f stale_pops=%zu "
      "open_peak=%zu records_peak=%zu terminal_successors=%zu "
      "continuation_states=%zu continuation_depth_m=%.2f "
      "lattice_successor_generated=%zu lattice_successor_accepted=%zu "
      "lattice_successor_reject_edge=%zu lattice_successor_reject_zero=%zu "
      "lattice_successor_reject_outside_roi=%zu "
      "lattice_successor_reject_unknown_space=%zu "
      "lattice_successor_reject_envelope=%zu "
      "lattice_successor_reject_invalid=%zu "
      "lattice_successor_reject_collision=%zu lattice_successor_reject_risk=%zu "
      "lattice_successor_reject_cost=%zu "
      "passage_successor_generated=%zu passage_successor_accepted=%zu "
      "passage_successor_rejected=%zu passage_successor_reject_connection=%zu "
      "passage_successor_reject_outside_roi=%zu "
      "passage_successor_reject_unknown_space=%zu "
      "passage_successor_reject_envelope=%zu "
      "passage_successor_reject_invalid=%zu "
      "passage_successor_reject_collision=%zu passage_successor_reject_risk=%zu "
      "passage_successor_reject_cost=%zu "
      "successor_search_batches=%zu successor_search_candidates=%zu "
      "successor_search_batch_max=%zu successor_search_worker_ms=%.3f "
      "successor_continuation_batches=%zu "
      "successor_continuation_candidates=%zu "
      "successor_continuation_batch_max=%zu "
      "successor_continuation_worker_ms=%.3f "
      "objective=%.3f route_length_m=%.2f travel_time_s=%.2f "
      "vertical_alignment_time_s=%.2f planning_exposure_m=%.2f "
      "critical_exposure_m=%.2f selected_passage_traversals=%zu search_ms=%.2f "
      "topology_searches=%zu parallel_topology_searches=%zu "
      "topology_search_worker_ms=%.2f "
      "continuation_ms=%.2f validation_ms=%.2f smoothing_ms=%.2f "
      "shortcut_validation_ms=%.2f corner_validation_ms=%.2f "
      "shortcut_candidates=%zu parallel_shortcut_candidates=%zu "
      "corner_candidates=%zu parallel_corner_candidates=%zu "
      "shortcuts=%zu smoothed_corners=%zu route_fingerprint=%" PRIu64,
      prepared.revision, activated ? "true" : "false",
      static_cast<int>(staticRouteActivationStatusName(activation_status).size()),
      staticRouteActivationStatusName(activation_status).data(),
      revision_matches ? "true" : "false", generation_matches ? "true" : "false",
      objective_matches ? "true" : "false", prepared.global_guide_generation,
      route_space, observed_world_rebased ? "true" : "false",
      lattice3DRoutePurposeName(lattice.route_purpose), topology_acceleration,
      observation_frontier ? observation_frontier->id.value : 0U,
      observation_frontier ? observation_frontier->supporting_map_revision : 0U,
      observation_frontier ? observation_frontier->supporting_rays : 0U,
      observation_frontier ? observation_frontier->information_gain_voxels : 0U,
      lattice.frontier_selection_score, lattice.frontier_candidates_considered,
      lattice.frontier_sampled_free_voxels, lattice.frontier_boundary_candidates,
      lattice.frontier_evaluated_candidates, lattice.frontier_searches,
      lattice.frontier_evaluation_budget_exhausted ? "true" : "false",
      static_cast<int>(
          observationRouteReplacementStatusName(observation_replacement.status).size()),
      observationRouteReplacementStatusName(observation_replacement.status).data(),
      observation_replacement.score_improvement,
      world.static_route_extension_request ? "true" : "false",
      world.static_route_replan_request ? "true" : "false",
      world.static_route_replan_request ? world.static_route_replan_base_generation
                                        : world.static_route_extension_base_generation,
      globalGuideReleaseReasonName(world.static_route_replan_reason),
      static_cast<int>(staticRouteReplacementPolicyName(replacement_policy).size()),
      staticRouteReplacementPolicyName(replacement_policy).data(),
      static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
      staticRouteCandidateStatusName(validation.status).data(),
      validation.endpoint_improvement_m, validation.failure_segment_index,
      validation.failure_point.x, validation.failure_point.y,
      validation.failure_point.z, mppi::staticRouteHandoffStatusName(handoff.status),
      handoff.cross_track_m, handoff.minimum_clearance_m, handoff.planning_exposure_m,
      handoff.critical_exposure_m, lattice3DStatusName(lattice.status),
      lattice3DSearchTerminationName(lattice.termination), lattice.points.size(),
      lattice.route.size(),
      prepared.constrained_spans ? prepared.constrained_spans->size() : 0U,
      lattice.expansions, lattice_3d_config_.maximum_expansions,
      lattice_3d_config_.maximum_search_time_ms,
      lattice3DRiskStageName(lattice.risk_stage), search_start.x, search_start.y,
      search_start.z, lattice.planning_goal.x, lattice.planning_goal.y,
      lattice.planning_goal.z, prepared.planning_candidate_endpoint.x,
      prepared.planning_candidate_endpoint.y, prepared.planning_candidate_endpoint.z,
      preferred_direction.x, preferred_direction.y, preferred_direction.z,
      lattice.achieved_progress_m, lattice.minimum_clearance_m,
      lattice.stale_queue_pops, lattice.open_peak, lattice.records_peak,
      lattice.terminal_successor_count, lattice.continuation_reachable_states,
      lattice.continuation_reachable_depth_m,
      lattice.successor_diagnostics.lattice_generated,
      lattice.successor_diagnostics.lattice_accepted,
      lattice.successor_diagnostics.lattice_rejected_edge,
      lattice.successor_diagnostics.lattice_rejected_zero_length,
      lattice.successor_diagnostics.lattice_rejected_outside_grid,
      lattice.successor_diagnostics.lattice_rejected_unknown_space,
      lattice.successor_diagnostics.lattice_rejected_flight_envelope,
      lattice.successor_diagnostics.lattice_rejected_invalid_esdf,
      lattice.successor_diagnostics.lattice_rejected_raw_collision,
      lattice.successor_diagnostics.lattice_rejected_risk_stage,
      lattice.successor_diagnostics.lattice_rejected_no_cost_improvement,
      lattice.successor_diagnostics.passage_generated,
      lattice.successor_diagnostics.passage_accepted,
      lattice.successor_diagnostics.passage_rejected,
      lattice.successor_diagnostics.passage_rejected_connection_distance,
      lattice.successor_diagnostics.passage_rejected_outside_grid,
      lattice.successor_diagnostics.passage_rejected_unknown_space,
      lattice.successor_diagnostics.passage_rejected_flight_envelope,
      lattice.successor_diagnostics.passage_rejected_invalid_esdf,
      lattice.successor_diagnostics.passage_rejected_raw_collision,
      lattice.successor_diagnostics.passage_rejected_risk_stage,
      lattice.successor_diagnostics.passage_rejected_no_cost_improvement,
      lattice.successor_profiling.search.collection_calls,
      lattice.successor_profiling.search.candidates,
      lattice.successor_profiling.search.maximum_candidates,
      lattice.successor_profiling.search.worker_ms,
      lattice.successor_profiling.continuation.collection_calls,
      lattice.successor_profiling.continuation.candidates,
      lattice.successor_profiling.continuation.maximum_candidates,
      lattice.successor_profiling.continuation.worker_ms, lattice.objective_cost,
      lattice.route_length_m, lattice.estimated_travel_time_s,
      lattice.vertical_alignment_time_s, lattice.planning_exposure_m,
      lattice.critical_exposure_m, route_traversals.size(), search_ms,
      lattice.topology_searches, lattice.parallel_topology_searches,
      lattice.topology_search_worker_ms, prepared.continuation_validation_ms,
      prepared.candidate_validation_ms, prepared.route_smoothing_ms,
      prepared.route_shortcut_validation_ms, prepared.route_corner_validation_ms,
      prepared.route_shortcut_candidates, prepared.route_parallel_shortcut_candidates,
      prepared.route_corner_candidates, prepared.route_parallel_corner_candidates,
      prepared.route_shortcuts_applied, prepared.route_corners_smoothed,
      prepared.route_fingerprint);
  for (const Lattice3DTopologyCandidate& candidate : lattice.topology_candidates) {
    RCLCPP_INFO(get_logger(),
                "PRODUCTION_MPPI_TOPOLOGY_CANDIDATE revision=%" PRIu64
                " topology=%s risk_stage=%s status=%s termination=%s "
                "achieved_progress_m=%.2f minimum_clearance_m=%.2f "
                "expansions=%zu stale_pops=%zu open_peak=%zu records_peak=%zu "
                "terminal_successors=%zu continuation_states=%zu "
                "continuation_depth_m=%.2f objective=%.3f "
                "route_length_m=%.2f travel_time_s=%.2f "
                "vertical_alignment_time_s=%.2f planning_exposure_m=%.2f "
                "critical_exposure_m=%.2f turn_cost=%.3f rank=%zu selected=%s "
                "reason=%s",
                prepared.revision, candidate.topology.c_str(),
                lattice3DRiskStageName(candidate.risk_stage),
                lattice3DStatusName(candidate.status),
                lattice3DSearchTerminationName(candidate.termination),
                candidate.achieved_progress_m, candidate.minimum_clearance_m,
                candidate.expansions, candidate.stale_queue_pops, candidate.open_peak,
                candidate.records_peak, candidate.terminal_successor_count,
                candidate.continuation_reachable_states,
                candidate.continuation_reachable_depth_m, candidate.objective_cost,
                candidate.route_length_m, candidate.estimated_travel_time_s,
                candidate.vertical_alignment_time_s, candidate.planning_exposure_m,
                candidate.critical_exposure_m, candidate.turn_cost,
                candidate.candidate_rank, candidate.selected ? "true" : "false",
                candidate.decision_reason.c_str());
  }
  if (world.static_route_extension_request) {
    finishStaticRouteExtension(world.static_route_extension_base_generation, activated);
  }
  const bool initial_route_search = !world.static_route_extension_request &&
                                    !world.static_route_replan_request &&
                                    world.global_guide_generation == 0U;
  if (world.static_route_replan_request || initial_route_search) {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    if (activated) {
      static_route_failed_search_latch_.clear();
    } else if (initial_route_search ||
               (revision_matches && generation_matches && objective_matches &&
                !(lattice.route_purpose ==
                      Lattice3DRoutePurpose::kObservationFrontier &&
                  observation_replacement.status ==
                      ObservationRouteReplacementStatus::kSameFrontierRetained))) {
      const std::uint64_t failed_generation =
          world.static_route_replan_request ? world.static_route_replan_base_generation
                                            : 0U;
      static_route_failed_search_latch_.recordFailure(StaticRouteSearchContext{
          .base_route_generation = failed_generation,
          .search_start = search_start,
          .objective = world.search_objective,
          .minimum_tracking_sample_sequence = required_objective_sample,
          .stamp_ns = get_clock()->now().nanoseconds(),
      });
      RCLCPP_INFO(
          get_logger(),
          "STATIC_ROUTE_SEARCH_OUTCOME status=failed_latched "
          "generation=%" PRIu64 " initial=%s activation_status=%.*s "
          "candidate_status=%.*s start=(%.2f,%.2f,%.2f)",
          failed_generation, initial_route_search ? "true" : "false",
          static_cast<int>(staticRouteActivationStatusName(activation_status).size()),
          staticRouteActivationStatusName(activation_status).data(),
          static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
          staticRouteCandidateStatusName(validation.status).data(), search_start.x,
          search_start.y, search_start.z);
    }
    if (world.static_route_replan_request) {
      static_route_replan_gate_.finish(world.static_route_replan_base_generation);
    }
  } else if (activated) {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    static_route_failed_search_latch_.clear();
  }
  const std::shared_ptr<const ProductionNavigationObjective> current_objective =
      navigationObjective();
  if (current_objective && current_objective->continuous_tracking) {
    const std::uint64_t required_epoch =
        minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
    const std::uint64_t required_sample =
        current_objective->mission_epoch == required_epoch
            ? minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire)
            : 0U;
    StaticRouteObjective resident_route_objective;
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      if (prepared_esdf_) {
        resident_route_objective = prepared_esdf_->route_objective;
      }
    }
    if (!staticRouteObjectiveMatches(
            resident_route_objective, makeStaticRouteObjective(*current_objective),
            required_sample, std::numeric_limits<double>::infinity())) {
      RCLCPP_INFO(get_logger(),
                  "STATIC_ROUTE_SHADOW status=followup_required "
                  "required_epoch=%" PRIu64 " required_sample=%" PRIu64
                  " resident_epoch=%" PRIu64 " resident_sample=%" PRIu64,
                  required_epoch, required_sample,
                  resident_route_objective.mission_epoch,
                  resident_route_objective.sample_sequence);
      requestGuideRelease(GlobalGuideReleaseReason::kObjectiveChanged);
    }
  }
}

} // namespace drone_city_nav
