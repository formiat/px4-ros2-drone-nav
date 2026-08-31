#include "route_materializer_3d.hpp"

#include "drone_city_nav/mppi/route_risk_adapter_3d.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/passage_traversal_selection_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <ranges>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "production_mppi_route_helpers.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool
routeMaterializerConfigValid(const RouteMaterializerConfig3D& config) noexcept {
  return futureRouteConnectorConfig3DValid(config.future_route_connector) &&
         staticRouteExtensionConfigValid(config.route_extension) &&
         passageVolumeConfigIsValid(config.passage_volume) &&
         std::isfinite(config.critical_distance_m) &&
         config.critical_distance_m >= 0.0 &&
         std::isfinite(config.preferred_distance_m) &&
         config.preferred_distance_m >= config.critical_distance_m &&
         std::isfinite(config.route_geometry.maximum_shortcut_length_m) &&
         config.route_geometry.maximum_shortcut_length_m > 0.0 &&
         std::isfinite(config.route_geometry.sparse_deviation_tolerance_m) &&
         config.route_geometry.sparse_deviation_tolerance_m >= 0.0 &&
         std::isfinite(config.route_geometry.maximum_shortcut_turn_increase_rad) &&
         config.route_geometry.maximum_shortcut_turn_increase_rad >= 0.0 &&
         config.route_geometry.shortcut_validation_batch_size != 0U &&
         std::isfinite(config.route_geometry.corner_smoothing_distance_m) &&
         config.route_geometry.corner_smoothing_distance_m >= 0.0 &&
         config.route_geometry.corner_curve_samples >= 2U;
}

} // namespace

RouteMaterializer3D::RouteMaterializer3D(const RouteMaterializerConfig3D& config)
    : config_{config} {
  if (!routeMaterializerConfigValid(config_)) {
    throw std::invalid_argument{"invalid route materializer configuration"};
  }
}

ProductionRouteMaterialization3D
RouteMaterializer3D::materialize(RouteMaterializationRequest3D request) const {
  if (!request.valid()) {
    return ProductionRouteMaterialization3D{
        .route = {},
        .telemetry = {},
        .validation =
            StaticRouteCandidateValidation{
                .status = StaticRouteCandidateStatus::kInvalidInput},
        .replacement_policy = StaticRouteReplacementPolicy::kRequireEndpointImprovement,
        .geometry_optimization_fallback = std::nullopt,
    };
  }
  const PlannerSearchTransaction3D& transaction = *request.transaction;
  const ProductionWorldBuildTelemetry3D& world_telemetry = request.world_telemetry;
  const Point3& current_position = request.current_position;
  const Point3& mission_goal = transaction.objective.goal;
  const RouteSearchCandidate3D& candidate = request.candidate;
  const std::uint64_t candidate_generation = request.candidate_generation;
  const CertifiedRouteSuffix3D* const active_route = request.active_route.get();
  const ProductionMppiRawWorld3D* const activation_raw_world =
      request.activation_raw_world.get();
  const Point3 search_start = candidate.search_start;
  const PlannerTelemetry3D& plan = candidate.planner_telemetry;
  const SpatialRouteCandidate3D& spatial_route = candidate.spatial_route;
  ProductionRouteMaterialization3D result;
  MaterializedRoute3D& route = result.route;
  ProductionRoutePipelineTelemetry3D& telemetry = result.telemetry;
  ProductionRouteMaterializationTelemetry3D& materialization =
      telemetry.materialization;
  ProductionRouteSearchProvenance3D& provenance = route.provenance;
  route.world = transaction.world;
  route.objective = transaction.objective;
  route.candidate_generation = candidate_generation;
  route.intent = candidate.intent;
  route.segment_evidence = candidate.evidence;
  provenance.kind = ProductionPlanningSearchKind::kPersistentDStarLite3D;
  provenance.base_route_instance_id = candidate.search_base_route_instance_id;
  provenance.base_stitch_station_m = candidate.search_base_stitch_station_m;
  provenance.required_splice_base_route_instance_id =
      candidate.search_base_route_instance_id;
  provenance.start = search_start;
  provenance.goal = mission_goal;
  provenance.candidate_endpoint =
      spatial_route.points.empty() ? search_start : spatial_route.points.back();
  provenance.direction = candidate.search_velocity;
  provenance.candidate_points = spatial_route.points.size();
  provenance.candidate_samples = candidate.route.size();
  telemetry.world_build = world_telemetry;
  telemetry.planner = ProductionPersistentPlannerTelemetry3D{
      .input_status = candidate.planner_input_status,
      .progress = candidate.planner_progress,
      .mission_epoch = plan.mission_epoch,
      .planned_on_revision = plan.planned_on_revision,
      .occupied_fingerprint = plan.occupied_fingerprint,
      .search_generation = plan.search_generation,
      .repair_generation = plan.repair_generation,
      .expansions = plan.expansions,
      .changed_occupied_voxels = plan.changed_occupied_voxels,
      .affected_lattice_states = plan.affected_lattice_states,
      .repair_lattice_states_processed = plan.repair_lattice_states_processed,
      .repair_lattice_states_pending = plan.repair_lattice_states_pending,
      .feasibility_expansions = plan.feasibility_expansions,
      .records = plan.records,
      .open_entries = plan.open_entries,
      .shortcut_checks = plan.shortcut_checks,
      .shortcuts_applied = plan.shortcuts_applied,
      .lattice_edge_queries = plan.lattice_edge_queries,
      .raw_edge_validation_checks = plan.raw_edge_validation_checks,
      .adaptive_edge_queries = plan.adaptive_edge_queries,
      .adaptive_edges_in_extracted_path = plan.adaptive_edges_in_extracted_path,
      .maximum_queried_lattice_level = plan.maximum_queried_lattice_level,
      .execution_time_search_expansions = plan.execution_time_search_expansions,
      .execution_time_search_records = plan.execution_time_search_records,
      .execution_time_search_open_entries = plan.execution_time_search_open_entries,
      .path_length_m = spatial_route.path_length_m,
      .remaining_goal_distance_m =
          distance3D(provenance.candidate_endpoint, mission_goal),
      .execution_time_search_objective_s = plan.execution_time_search_objective_s,
      .estimated_execution_time_s = spatial_route.estimated_execution_time_s,
      .estimated_translation_time_s = spatial_route.estimated_translation_time_s,
      .estimated_stationary_turn_time_s =
          spatial_route.estimated_stationary_turn_time_s,
      .world_update_ms = plan.world_update_ms,
      .search_ms = plan.search_ms,
      .invoked = true,
      .executable = spatial_route.valid(),
      .search_state_reused = plan.search_state_reused,
      .occupied_world_unchanged = plan.occupied_world_unchanged,
      .incumbent_retained = plan.incumbent_retained,
      .repair_pending = plan.repair_pending,
      .feasibility_attempted = plan.feasibility_attempted,
      .feasibility_route_found = plan.feasibility_route_found,
      .execution_time_search_complete = plan.execution_time_search_complete,
      .incumbent_available = plan.incumbent_available,
  };
  route.reaches_mission_goal = spatial_route.valid();
  route.planner_executable = telemetry.planner.executable;
  materialization.continuation_validation_ms = 0.0;
  route.fingerprint = routeFingerprint(candidate.route);

  const std::span<const PassageTraversalEdge> topology_traversals =
      transaction.world->topology_passage_traversals
          ? std::span<const PassageTraversalEdge>{*transaction.world
                                                       ->topology_passage_traversals}
          : std::span<const PassageTraversalEdge>{};
  const std::vector<SelectedPassageTraversal> route_traversals =
      selectRoutePassageTraversals3D(candidate.route, topology_traversals);
  if (transaction.replacement()) {
    result.replacement_policy = StaticRouteReplacementPolicy::kAllowSafetyReplan;
  } else if (transaction.extension()) {
    result.replacement_policy = StaticRouteReplacementPolicy::kAllowSuccessorProgress;
  }

  result.validation =
      StaticRouteCandidateValidation{.status = StaticRouteCandidateStatus::kEmpty};
  if (!route.planner_executable) {
    return result;
  }

  const auto validation_started = std::chrono::steady_clock::now();
  auto mutable_route = std::make_shared<std::vector<RouteSample3D>>(candidate.route);
  std::vector<ConstrainedRouteSpan> initial_spans = makeConstrainedRouteSpans(
      *mutable_route, route_traversals, candidate_generation, config_.route_envelope);
  std::optional<FrozenRoutePrefix3D> frozen_prefix;
  const bool overlap_search = candidate.search_base_route_instance_id.valid();
  if ((transaction.extension() || transaction.replacement()) && overlap_search) {
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
              *candidate.search_base_stitch_station_m, config_.future_route_connector);
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
                                   materialized_prefix.route, config_.route_envelope);
    const std::vector<ConstrainedRouteSpan> remapped_successor_spans =
        remapConstrainedRouteSpans(candidate.route, successor_suffix_spans,
                                   materialized_prefix.route, config_.route_envelope);
    initial_spans.insert(initial_spans.end(), remapped_successor_spans.begin(),
                         remapped_successor_spans.end());
    mergeAdjacentConstrainedRouteSpans(initial_spans);
    *mutable_route = materialized_prefix.route;
  }
  const std::size_t expected_span_count = initial_spans.size();
  const std::vector<RouteSample3D> canonical_route = *mutable_route;
  const auto smoothing_started = std::chrono::steady_clock::now();
  StaticRouteGeometryConfig geometry_config = config_.route_geometry;
  if (frozen_prefix.has_value()) {
    const FrozenRoutePrefix3D& materialized_prefix = frozen_prefix.value();
    geometry_config.frozen_prefix_end_station_m =
        materialized_prefix.stitch_station_m -
        materialized_prefix.active_begin_station_m;
  }
  const OccupiedCollisionWorld3D geometry_collision_world{
      .observed_occupancy =
          activation_raw_world != nullptr && activation_raw_world->occupancy != nullptr
              ? activation_raw_world->occupancy.get()
              : nullptr,
      .static_occupancy = transaction.world->static_occupancy.get(),
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact =
          transaction.world->launch_support_contact
              ? std::addressof(*transaction.world->launch_support_contact)
              : nullptr,
      .footprint = config_.physical_footprint,
      .flight_envelope = config_.flight_envelope,
  };
  StaticRouteGeometryResult geometry = optimizeStaticRouteGeometry(
      *mutable_route, initial_spans, geometry_collision_world, geometry_config,
      config_.route_envelope, config_.worker_pool);
  materialization.route_smoothing_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                smoothing_started)
          .count();
  materialization.route_shortcuts_applied = geometry.shortcuts_applied;
  materialization.route_corners_smoothed = geometry.corners_smoothed;
  materialization.route_shortcut_candidates = geometry.shortcut_candidates;
  materialization.route_parallel_shortcut_candidates =
      geometry.parallel_shortcut_candidates;
  materialization.route_corner_candidates = geometry.corner_candidates;
  materialization.route_parallel_corner_candidates =
      geometry.parallel_corner_candidates;
  materialization.route_shortcut_validation_ms = geometry.shortcut_validation_ms;
  materialization.route_corner_validation_ms = geometry.corner_validation_ms;
  if (geometry.route.size() >= 2U) {
    *mutable_route = std::move(geometry.route);
  }

  const RouteRiskTierAssignmentResult3D optimized_risk_assignment =
      assignRouteRiskTiersFromMppiEsdf3D(
          *mutable_route, transaction.world->grid, *transaction.world->distances_m,
          config_.critical_distance_m, config_.preferred_distance_m);
  if (!optimized_risk_assignment.accepted()) {
    *mutable_route = canonical_route;
    geometry.constrained_spans = initial_spans;
    materialization.route_shortcuts_applied = 0U;
    materialization.route_corners_smoothed = 0U;
    result.geometry_optimization_fallback = optimized_risk_assignment;
  }
  for (ConstrainedRouteSpan& span : geometry.constrained_spans) {
    span.route_generation = candidate_generation;
  }

  std::vector<CooperativePassageAssignment> passage_assignments;
  std::shared_ptr<const std::vector<PassageVolume>> passage_volumes;
  bool cooperative_route_valid = true;
  const OccupancyGrid3D* passage_occupancy = nullptr;
  std::uint64_t passage_occupancy_content_fingerprint{0U};
  if (transaction.world->static_occupancy != nullptr) {
    passage_occupancy = transaction.world->static_occupancy.get();
    passage_occupancy_content_fingerprint =
        transaction.world->static_occupancy->contentFingerprint();
  }
  const bool passage_geometry_required = !geometry.constrained_spans.empty();
  if (passage_geometry_required && passage_occupancy != nullptr) {
    const auto passage_started = std::chrono::steady_clock::now();
    PassageVolumeResource volume_resource = acquireDerivedPassageVolumes(
        *mutable_route, geometry.constrained_spans, *passage_occupancy,
        passage_occupancy_content_fingerprint, config_.passage_volume);
    materialization.passage_volume_resource_reused =
        volume_resource.shared_resource_reused;
    passage_volumes = std::move(volume_resource.volumes);
    const std::span<const PassageVolume> volumes =
        passage_volumes ? std::span<const PassageVolume>{*passage_volumes}
                        : std::span<const PassageVolume>{};
    static_cast<void>(projectPassageVolumeEnvelopes(geometry.constrained_spans, volumes,
                                                    config_.passage_volume.footprint));
    if (config_.cooperative_traffic_enabled) {
      CooperativePassageRouteResult cooperative_route =
          applyCooperativePassageCorridors(*mutable_route, geometry.constrained_spans,
                                           volumes, *passage_occupancy,
                                           config_.cooperative_passage_route);
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
          passage_occupancy_content_fingerprint, config_.passage_volume);
      materialization.passage_volume_resource_reused =
          materialization.passage_volume_resource_reused ||
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
                                        config_.passage_volume.footprint) ==
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
    materialization.passage_volume_build_ms =
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
  } else if (const RouteRiskTierAssignmentResult3D risk_assignment =
                 assignRouteRiskTiersFromMppiEsdf3D(
                     *mutable_route, transaction.world->grid,
                     *transaction.world->distances_m, config_.critical_distance_m,
                     config_.preferred_distance_m);
             risk_assignment.accepted()) {
    result.validation = validateStaticRouteCandidate(
        active_route != nullptr && active_route->geometry != nullptr &&
                active_route->geometry->route != nullptr
            ? std::span<const RouteSample3D>{*active_route->geometry->route}
            : std::span<const RouteSample3D>{},
        *mutable_route, mission_goal,
        config_.route_extension.minimum_endpoint_improvement_m, spatial_route.valid(),
        config_.flight_envelope, result.replacement_policy);
  } else {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kInvalidInput,
        .failure_segment_index = risk_assignment.failure_sample_index,
        .failure_point = risk_assignment.failure_point};
  }

  const std::shared_ptr<const std::vector<RouteSample3D>> materialized_route =
      mutable_route;
  const std::shared_ptr<const std::vector<ConstrainedRouteSpan>> spans = mutable_spans;
  if (result.validation.accepted && spans->size() != expected_span_count) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
  }
  if (result.validation.accepted &&
      !validateConstrainedRouteSpans(*materialized_route, *spans)) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
  }
  const bool protected_suffix =
      active_route != nullptr && active_route->geometry != nullptr &&
      active_route->geometry->route != nullptr &&
      active_route->geometry->constrained_spans != nullptr &&
      staticRouteReplacementProtected(
          *active_route->geometry->route, *active_route->geometry->constrained_spans,
          current_position, active_route->identity.proposal.objective,
          transaction.objective, config_.route_extension.protected_departure_m);
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
  route.fingerprint = routeFingerprint(*materialized_route, route_traversals);
  route.route = materialized_route;
  route.constrained_spans = spans;
  route.passage_volumes = passage_volumes;
  route.cooperative_passage_assignments =
      std::make_shared<const std::vector<CooperativePassageAssignment>>(
          std::move(passage_assignments));
  route.selected_passage_traversal_ids =
      std::make_shared<const std::vector<PassageTraversalId>>(
          std::move(selected_passage_traversal_ids));
  route.initial_projection =
      projectOntoRouteProgress3D(*materialized_route, current_position);
  materialization.candidate_validation_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                validation_started)
          .count();
  return result;
}

} // namespace drone_city_nav
