#include "production_mppi_route_materialization.hpp"

#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/route_compiler_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <ranges>
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
    const CertifiedRouteSuffix3D* const active_route,
    const ProductionMppiRawWorld3D* const activation_raw_world) {
  const Point3 current_position{navigation.state.x, navigation.state.y,
                                navigation.state.z};
  const Point3 search_start = candidate.search_start;
  const PersistentPlannerResult3D& plan = candidate.plan;
  ProductionRouteMaterialization3D result;
  ProductionMppiPreparedEsdf& prepared = result.prepared;
  prepared = world;
  prepared.mppi_route.reset();
  prepared.route_3d.reset();
  prepared.compiled_route_geometry.reset();
  prepared.route_compilation_validation = {};
  prepared.route_stop_turn_count = 0U;
  prepared.route_2d_projection.reset();
  prepared.constrained_spans.reset();
  prepared.passage_volumes.reset();
  prepared.cooperative_passage_assignments.reset();
  prepared.selected_passage_traversal_ids.reset();
  prepared.passage_traversals.reset();
  prepared.route_intent = candidate.intent;
  prepared.route_segment_evidence = candidate.evidence;
  prepared.planning_search_kind = ProductionPlanningSearchKind::kPersistentDStarLite3D;
  prepared.planning_search_base_route_instance_id =
      candidate.search_base_route_instance_id;
  prepared.planning_search_base_stitch_station_m =
      candidate.search_base_stitch_station_m;
  prepared.required_splice_base_route_instance_id =
      candidate.search_base_route_instance_id;
  prepared.planning_search_start = search_start;
  prepared.planning_search_goal = mission_goal;
  prepared.planning_candidate_endpoint =
      plan.points.empty() ? search_start : plan.points.back();
  prepared.planning_search_direction = candidate.search_velocity;
  prepared.planning_candidate_points = plan.points.size();
  prepared.planning_candidate_samples = candidate.route.size();
  prepared.lattice_search_performed = true;
  prepared.lattice_executable = plan.executable();
  prepared.global_guide_expansions = plan.expansions;
  prepared.lattice_3d_status = Lattice3DStatus::kReachedPlanningGoal;
  prepared.lattice_3d_risk_stage = Lattice3DRiskStage::kPreferredOnly;
  prepared.lattice_3d_termination = Lattice3DSearchTermination::kPlanningGoalReached;
  prepared.lattice_3d_route_purpose = Lattice3DRoutePurpose::kMissionTransit;
  prepared.lattice_3d_observation_frontier.reset();
  prepared.lattice_search_session_complete = plan.search_complete;
  prepared.lattice_search_revision = plan.planned_on_revision;
  prepared.lattice_validation_revision = plan.planned_on_revision;
  prepared.lattice_planning_goal_reached = plan.executable();
  prepared.lattice_achieved_progress_m = plan.path_length_m;
  prepared.lattice_guide_length_m = plan.path_length_m;
  prepared.lattice_remaining_goal_distance_m =
      distance3D(prepared.planning_candidate_endpoint, mission_goal);
  prepared.lattice_open_peak = plan.open_entries;
  prepared.lattice_records_peak = plan.records;
  prepared.lattice_frontier_candidates_considered = 0U;
  prepared.lattice_frontier_sampled_free_voxels = 0U;
  prepared.lattice_frontier_boundary_candidates = 0U;
  prepared.lattice_frontier_evaluated_candidates = 0U;
  prepared.lattice_frontier_searches = 0U;
  prepared.lattice_frontier_evaluation_budget_exhausted = false;
  prepared.global_guide_cost = plan.estimated_execution_time_s;
  prepared.global_guide_reaches_mission_goal = plan.executable();
  prepared.topology_candidates.clear();
  prepared.topology_objective_cost = plan.estimated_execution_time_s;
  prepared.topology_route_length_m = plan.path_length_m;
  prepared.topology_travel_time_s = plan.estimated_execution_time_s;
  prepared.topology_vertical_alignment_time_s = 0.0;
  prepared.topology_planning_exposure_m = 0.0;
  prepared.topology_critical_exposure_m = 0.0;
  prepared.continuation_validation_ms = 0.0;
  prepared.route_fingerprint = routeFingerprint(candidate.route);
  prepared.bound_route_instance_id = {};

  const std::vector<SelectedPassageTraversal> route_traversals;
  if (world.static_route_replan_request) {
    result.replacement_policy = StaticRouteReplacementPolicy::kAllowSafetyReplan;
  } else if (world.static_route_extension_request) {
    result.replacement_policy = StaticRouteReplacementPolicy::kAllowTopologicalProgress;
  } else if (!optional_constraints_.route_replacement_progress_enabled) {
    result.replacement_policy =
        StaticRouteReplacementPolicy::kAllowAnyValidatedReplacement;
  }

  prepared.observation_route_replacement_status =
      ObservationRouteReplacementStatus::kInvalidCandidate;

  result.validation =
      StaticRouteCandidateValidation{.status = StaticRouteCandidateStatus::kEmpty};
  if (!prepared.lattice_executable) {
    return result;
  }

  const auto validation_started = std::chrono::steady_clock::now();
  auto mutable_route = std::make_shared<std::vector<RouteSample3D>>(candidate.route);
  std::vector<ConstrainedRouteSpan> initial_spans = makeConstrainedRouteSpans(
      *mutable_route, route_traversals, candidate_generation, route_envelope_config_);
  std::optional<FrozenRoutePrefix3D> frozen_prefix;
  const bool overlap_search = candidate.search_base_route_instance_id.valid();
  if ((world.static_route_extension_request || world.static_route_replan_request) &&
      overlap_search) {
    if (!candidate.search_base_stitch_station_m.has_value() ||
        active_route == nullptr ||
        active_route->route_instance_id != candidate.search_base_route_instance_id ||
        !active_route->valid() || !active_route->geometry ||
        !active_route->geometry->route || !active_route->geometry->constrained_spans) {
      result.validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
      return result;
    }
    frozen_prefix = materializeFrozenRoutePrefixAtStation3D(
        *active_route->geometry->route, candidate.route, current_position,
        *candidate.search_base_stitch_station_m);
    if (!frozen_prefix.has_value()) {
      std::optional<FrozenRoutePrefix3D> connected_prefix =
          materializeTangentContinuousRoutePrefixAtStation3D(
              *active_route->geometry->route, candidate.route, current_position,
              *candidate.search_base_stitch_station_m, future_route_connector_config_);
      if (connected_prefix.has_value()) {
        const double successor_join_station_m =
            connected_prefix.value().successor_stitch_station_m;
        const bool connector_replaces_constrained_geometry = std::ranges::any_of(
            initial_spans,
            [successor_join_station_m](const ConstrainedRouteSpan& span) {
              return span.begin_station_m < successor_join_station_m;
            });
        if (!connector_replaces_constrained_geometry) {
          frozen_prefix = std::move(connected_prefix);
        }
      }
    }
    if (!frozen_prefix.has_value()) {
      result.validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
      return result;
    }
    const FrozenRoutePrefix3D& materialized_prefix = frozen_prefix.value();
    const std::vector<ConstrainedRouteSpan> active_prefix_spans =
        clipConstrainedRouteSpans(*active_route->geometry->constrained_spans,
                                  materialized_prefix.active_begin_station_m,
                                  materialized_prefix.stitch_station_m);
    const std::vector<ConstrainedRouteSpan> successor_suffix_spans =
        clipConstrainedRouteSpans(initial_spans,
                                  materialized_prefix.successor_stitch_station_m,
                                  std::numeric_limits<double>::infinity());
    initial_spans =
        remapConstrainedRouteSpans(*active_route->geometry->route, active_prefix_spans,
                                   materialized_prefix.route, route_envelope_config_);
    const std::vector<ConstrainedRouteSpan> remapped_successor_spans =
        remapConstrainedRouteSpans(candidate.route, successor_suffix_spans,
                                   materialized_prefix.route, route_envelope_config_);
    initial_spans.insert(initial_spans.end(), remapped_successor_spans.begin(),
                         remapped_successor_spans.end());
    mergeAdjacentConstrainedRouteSpans(initial_spans);
    *mutable_route = materialized_prefix.route;
  }
  const std::size_t expected_span_count = initial_spans.size();
  const std::vector<RouteSample3D> canonical_route = *mutable_route;
  const auto smoothing_started = std::chrono::steady_clock::now();
  StaticRouteGeometryConfig geometry_config = static_route_geometry_config_;
  if (frozen_prefix.has_value()) {
    const FrozenRoutePrefix3D& materialized_prefix = frozen_prefix.value();
    geometry_config.frozen_prefix_end_station_m =
        materialized_prefix.stitch_station_m -
        materialized_prefix.active_begin_station_m;
  }
  const StaticRouteGeometryRawValidation raw_geometry_validation{
      .occupancy =
          activation_raw_world != nullptr && activation_raw_world->occupancy != nullptr
              ? activation_raw_world->occupancy.get()
              : nullptr,
      .static_occupancy = static_occupancy_3d_.get(),
      .proprioceptive_free_space_seed =
          world.proprioceptive_free_space_seed
              ? std::addressof(*world.proprioceptive_free_space_seed)
              : nullptr,
      .launch_support_contact = world.launch_support_contact
                                    ? std::addressof(*world.launch_support_contact)
                                    : nullptr,
      .policy = ObservedSpaceValidationPolicy::kAllowUnknown,
  };
  StaticRouteGeometryResult geometry = optimizeStaticRouteGeometry(
      *mutable_route, initial_spans, world.grid, *world.distances_m,
      SweptFootprintConfig{.radius_m = physical_footprint_config_.radius_m,
                           .lower_extent_m = physical_footprint_config_.lower_extent_m,
                           .upper_extent_m = physical_footprint_config_.upper_extent_m,
                           .perimeter_samples =
                               physical_footprint_config_.perimeter_samples,
                           .radial_rings = physical_footprint_config_.radial_rings,
                           .axial_samples = physical_footprint_config_.axial_samples,
                           .sweep_step_m = physical_footprint_config_.sweep_step_m},
      geometry_config, route_envelope_config_, planning_worker_pool_.get(),
      raw_geometry_validation.occupancy != nullptr ||
              raw_geometry_validation.static_occupancy != nullptr
          ? &raw_geometry_validation
          : nullptr);
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
      .radius_m = physical_footprint_config_.radius_m,
      .lower_extent_m = physical_footprint_config_.lower_extent_m,
      .upper_extent_m = physical_footprint_config_.upper_extent_m,
      .perimeter_samples = physical_footprint_config_.perimeter_samples,
      .radial_rings = physical_footprint_config_.radial_rings,
      .axial_samples = physical_footprint_config_.axial_samples,
      .sweep_step_m = physical_footprint_config_.sweep_step_m};
  const RouteRiskTierAssignmentResult optimized_risk_assignment =
      assignRouteRiskTiers(*mutable_route, world.grid, *world.distances_m,
                           mppi_config_.risk.critical_distance_m,
                           mppi_config_.risk.preferred_distance_m, false);
  if (!optimized_risk_assignment.accepted()) {
    *mutable_route = canonical_route;
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
  const OccupancyGrid3D* passage_occupancy = nullptr;
  std::uint64_t passage_occupancy_content_fingerprint{0U};
  if (static_occupancy_3d_ != nullptr) {
    passage_occupancy = &*static_occupancy_3d_;
    passage_occupancy_content_fingerprint = static_occupancy_3d_->contentFingerprint();
  }
  const bool passage_geometry_required = !geometry.constrained_spans.empty();
  if (passage_geometry_required && passage_occupancy != nullptr) {
    const auto passage_started = std::chrono::steady_clock::now();
    PassageVolumeResource volume_resource = acquireDerivedPassageVolumes(
        *mutable_route, geometry.constrained_spans, *passage_occupancy,
        passage_occupancy_content_fingerprint, cooperative_passage_volume_config_);
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
                                           volumes, *passage_occupancy,
                                           cooperative_passage_route_config_);
      cooperative_route_valid = cooperative_route.valid;
      if (cooperative_route.valid) {
        *mutable_route = std::move(cooperative_route.route);
        geometry.constrained_spans = std::move(cooperative_route.constrained_spans);
      }
      passage_assignments = std::move(cooperative_route.assignments);
    }
    if (cooperative_route_valid) {
      PassageVolumeResource final_volume_resource = acquireDerivedPassageVolumes(
          *mutable_route, geometry.constrained_spans, *passage_occupancy,
          passage_occupancy_content_fingerprint, cooperative_passage_volume_config_);
      prepared.passage_volume_resource_reused =
          prepared.passage_volume_resource_reused ||
          final_volume_resource.shared_resource_reused;
      passage_volumes = std::move(final_volume_resource.volumes);
      const std::span<const PassageVolume> final_volumes =
          passage_volumes ? std::span<const PassageVolume>{*passage_volumes}
                          : std::span<const PassageVolume>{};
      cooperative_route_valid =
          final_volumes.size() == geometry.constrained_spans.size() &&
          (passage_assignments.empty() ||
           passage_assignments.size() == geometry.constrained_spans.size()) &&
          std::ranges::all_of(final_volumes, &PassageVolume::raw_validated) &&
          projectPassageVolumeEnvelopes(geometry.constrained_spans, final_volumes,
                                        cooperative_passage_volume_config_.footprint) ==
              geometry.constrained_spans.size();
      for (std::size_t index = 0U;
           cooperative_route_valid && index < passage_assignments.size(); ++index) {
        CooperativePassageAssignment& assignment = passage_assignments[index];
        const PassageVolume& volume = final_volumes[index];
        const ConstrainedRouteSpan& span = geometry.constrained_spans[index];
        const double first_lateral_bound_m =
            volume.minimum_lateral_offset_m * static_cast<double>(span.direction_sign) +
            assignment.applied_lateral_offset_m;
        const double second_lateral_bound_m =
            volume.maximum_lateral_offset_m * static_cast<double>(span.direction_sign) +
            assignment.applied_lateral_offset_m;
        assignment.physical_width_m = volume.minimum_physical_width_m;
        assignment.minimum_lateral_offset_m =
            std::min(first_lateral_bound_m, second_lateral_bound_m);
        assignment.maximum_lateral_offset_m =
            std::max(first_lateral_bound_m, second_lateral_bound_m);
        assignment.minimum_secondary_offset_m = volume.minimum_secondary_offset_m;
        assignment.maximum_secondary_offset_m = volume.maximum_secondary_offset_m;
        assignment.passage_cross_section_count = volume.cross_sections.size();
        assignment.passage_volume_raw_validated = volume.raw_validated;
      }
    }
    prepared.passage_volume_build_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  passage_started)
            .count();
  }
  if (passage_geometry_required && (passage_occupancy == nullptr || !passage_volumes)) {
    cooperative_route_valid = false;
  } else if (!passage_volumes) {
    passage_volumes = std::make_shared<const std::vector<PassageVolume>>();
  }

  auto mutable_spans = std::make_shared<std::vector<ConstrainedRouteSpan>>(
      std::move(geometry.constrained_spans));
  if (!cooperative_route_valid) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
  } else if (const RouteRiskTierAssignmentResult risk_assignment =
                 assignRouteRiskTiers(*mutable_route, world.grid, *world.distances_m,
                                      mppi_config_.risk.critical_distance_m,
                                      mppi_config_.risk.preferred_distance_m, false);
             risk_assignment.accepted()) {
    result.validation = validateStaticRouteCandidate(
        world.route_3d ? std::span<const RouteSample3D>{*world.route_3d}
                       : std::span<const RouteSample3D>{},
        *mutable_route, world.grid, *world.distances_m, mission_goal,
        static_route_extension_config_.minimum_endpoint_improvement_m,
        plan.executable(), lattice_3d_config_.flight_envelope,
        result.replacement_policy, footprint_config, false, true);
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
  if (result.validation.accepted && spans->size() != expected_span_count) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
  }
  if (result.validation.accepted &&
      !validateConstrainedRouteSpans(*route, *spans, world.grid, *world.distances_m)) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
  }
  const bool protected_suffix =
      world.route_3d && world.constrained_spans &&
      staticRouteReplacementProtected(
          *world.route_3d, *world.constrained_spans, current_position,
          world.route_objective, world.search_objective,
          static_route_extension_config_.protected_departure_m);
  if (result.validation.accepted && protected_suffix) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kProtectedConstrainedSuffix};
  }

  std::vector<PassageTraversalId> selected_passage_traversal_ids;
  selected_passage_traversal_ids.reserve(spans->size());
  for (const ConstrainedRouteSpan& span : *spans) {
    if (std::ranges::find(selected_passage_traversal_ids, span.passage_traversal_id) ==
        selected_passage_traversal_ids.end()) {
      selected_passage_traversal_ids.push_back(span.passage_traversal_id);
    }
  }
  const RouteEndpointSemantics3D endpoint_semantics = routeEndpointSemantics3D(
      prepared.route_intent, prepared.route_segment_evidence.reaches_intent_target,
      prepared.global_guide_reaches_mission_goal,
      !world.search_objective.continuous_tracking);
  prepared.route_fingerprint = routeFingerprint(*route, route_traversals);
  RouteCompilationResult3D compilation = compileExecutionRoute3D(RouteCompilerInput3D{
      .route = *route,
      .constrained_spans = *spans,
      .passage_volumes = *passage_volumes,
      .cooperative_passage_assignments = std::move(passage_assignments),
      .selected_passage_traversal_ids = std::move(selected_passage_traversal_ids),
      .passage_volume_config = cooperative_passage_volume_config_,
      .route_purpose = prepared.lattice_3d_route_purpose,
      .observation_frontier = prepared.lattice_3d_observation_frontier,
      .endpoint_semantics = endpoint_semantics,
      .materialized_route_fingerprint = prepared.route_fingerprint,
      .tracking_world = trackingErrorTubeWorld3D(world),
      .config = routeCompilerConfig3D(),
  });
  prepared.route_compilation_validation = compilation.validation;
  prepared.route_stop_turn_count = compilation.stop_turn_count;
  const bool route_compiled = compilation.compiled();
  prepared.compiled_route_geometry = std::move(compilation.geometry);
  if (prepared.compiled_route_geometry != nullptr) {
    prepared.mppi_route = prepared.compiled_route_geometry->mppi_route;
    prepared.route_3d = prepared.compiled_route_geometry->route;
    prepared.route_2d_projection =
        prepared.compiled_route_geometry->route_2d_projection;
    prepared.constrained_spans = prepared.compiled_route_geometry->constrained_spans;
    prepared.passage_volumes = prepared.compiled_route_geometry->passage_volumes;
    prepared.cooperative_passage_assignments =
        prepared.compiled_route_geometry->cooperative_passage_assignments;
    prepared.selected_passage_traversal_ids =
        prepared.compiled_route_geometry->selected_passage_traversal_ids;
  }
  if (result.validation.accepted && !route_compiled) {
    prepared.lattice_executable = false;
  }
  if (prepared.route_2d_projection == nullptr) {
    prepared.route_2d_projection = projectRouteTo2D(*route);
  }
  prepared.global_guide_projection = projectOntoGlobalGuide(
      *prepared.route_2d_projection, Point2{navigation.state.x, navigation.state.y});
  prepared.candidate_validation_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                validation_started)
          .count();
  return result;
}

} // namespace drone_city_nav
