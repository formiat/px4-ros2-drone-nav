#include "drone_city_nav/route_risk_annotation_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"

#include <algorithm>
#include <cinttypes>
#include <format>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {

void ProductionMppiNode::handleRoutePlanningRejection3D(
    const RoutePlanningRejection3D& rejection) {
  const std::shared_ptr<const PlannerSearchTransaction3D>& transaction =
      rejection.request.transaction;
  if (transaction == nullptr) {
    return;
  }
  switch (rejection.reason) {
    case RoutePlanningRejectionReason3D::kInvalidWorldGeneration: {
      const std::string_view status_name =
          productionWorldGenerationStatusName(rejection.world_status);
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ROUTE rejected local_world_generation=%" PRIu64
                   " reason=%.*s",
                   transaction->world->local_world_generation.generation,
                   static_cast<int>(status_name.size()), status_name.data());
      break;
    }
    case RoutePlanningRejectionReason3D::kFullThreeDimensionalWorldRequired:
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ROUTE rejected local_world_generation=%" PRIu64
                   " reason=full_3d_world_required depth=%d",
                   transaction->world->local_world_generation.generation,
                   rejection.world_depth);
      break;
    case RoutePlanningRejectionReason3D::kSupersededRouteGeneration: {
      const StaticRouteSearchRequestIdentity& request = transaction->request;
      const StaticRouteSearchCurrencyAssessment& currency = rejection.currency;
      RCLCPP_INFO(
          get_logger(),
          "STATIC_ROUTE_SEARCH_REQUEST status=%.*s kind=%.*s "
          "request_generation=%" PRIu64 " resident_generation=%" PRIu64,
          static_cast<int>(staticRouteSearchCurrencyStatusName(currency.status).size()),
          staticRouteSearchCurrencyStatusName(currency.status).data(),
          static_cast<int>(staticRouteSearchRequestKindName(request.kind).size()),
          staticRouteSearchRequestKindName(request.kind).data(),
          request.base_route_generation, currency.resident_route_generation);
      break;
    }
    case RoutePlanningRejectionReason3D::kVehicleStateUnavailable:
      break;
    case RoutePlanningRejectionReason3D::kProcessingFailed:
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ROUTE rejected local_world_generation=%" PRIu64
                   " reason=planning_coordinator_failure",
                   transaction->world->local_world_generation.generation);
      break;
    case RoutePlanningRejectionReason3D::kUpdateHandlerFailed:
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ROUTE rejected local_world_generation=%" PRIu64
                   " reason=planning_update_handler_failure",
                   transaction->world->local_world_generation.generation);
      break;
  }
}

void ProductionMppiNode::processRouteSearch3D(RouteLifecycleUpdate3D update) {
  const std::shared_ptr<const PlannerSearchTransaction3D>& transaction =
      update.request.transaction;
  if (transaction == nullptr || !update.valid()) {
    return;
  }
  const RoutePlannerUpdate3D& planner_update = update.planner_update;
  const Point3 search_start = update.search_start;

  if (planner_update.status == RoutePlannerUpdateStatus3D::kStitchBaseUnavailable) {
    RCLCPP_INFO(get_logger(),
                "PERSISTENT_PLANNER3D stage=deferred "
                "reason=stitch_base_unavailable revision=%" PRIu64,
                transaction->world->revision);
  } else if (planner_update.status ==
             RoutePlannerUpdateStatus3D::kStitchBeyondCertifiedRoute) {
    RCLCPP_INFO(get_logger(),
                "PERSISTENT_PLANNER3D stage=deferred "
                "reason=future_stitch_beyond_certified_route revision=%" PRIu64
                " stitch_station_m=%.3f route_end_station_m=%.3f",
                transaction->world->revision, planner_update.attempted_stitch_station_m,
                planner_update.certified_route_end_station_m);
  }
  if (planner_update.planner_invoked) {
    const PlannerTelemetry3D& planner_telemetry = planner_update.planner_telemetry;
    RCLCPP_INFO(
        get_logger(),
        "PERSISTENT_PLANNER3D stage=complete raw_revision=%" PRIu64
        " mission_epoch=%" PRIu64 " input=%s input_failure=%s progress=%s "
        "publishable=%s "
        "source=%s reused=%s occupied_unchanged=%s incumbent_retained=%s "
        "time_search_complete=%s points=%zu expansions=%zu time_expansions=%zu "
        "changed_occupied=%zu affected_states=%zu repair_processed=%zu "
        "repair_pending=%zu repair_in_progress=%s feasibility_attempted=%s "
        "feasibility_anchor_closed=%s "
        "feasibility_found=%s feasibility_expansions=%zu "
        "feasibility_exhausted=%s feasibility_explored=%zu "
        "feasibility_closest_goal_m=%.1f feasibility_restarts=%zu "
        "feasibility_invalidated_labels=%zu feasibility_adopted_labels=%zu "
        "feasibility_connector_sweeps=%zu "
        "feasibility_invalid_segment=%zu "
        "feasibility_anchor=(%.1f,%.1f,%.1f) records=%zu open=%zu "
        "time_records=%zu time_open=%zu shortcuts=%zu/%zu edge_queries=%zu "
        "raw_edge_checks=%zu adaptive_edge_queries=%zu adaptive_path_edges=%zu "
        "maximum_adaptive_level=%zu time_objective_s=%.3f eta_s=%.3f "
        "translation_s=%.3f turn_s=%.3f world_update_ms=%.3f search_ms=%.3f "
        "feasibility_ms=%.1f repair_ms=%.1f spatial_search_ms=%.1f "
        "refinement_ms=%.1f world_diff_ms=%.1f world_install_ms=%.1f "
        "schedule_ms=%.1f schedule_ranking_ms=%.1f edges_forgotten=%zu "
        "clearances_tightened=%zu clearances_rederived=%zu "
        "clearance_derivation_ms=%.1f raw_sweep_ms=%.1f "
        "departure_waypoints=%zu anchor_skip=%zu goal_refined=%s escape_attempted=%s "
        "escape_found=%s "
        "escape_active=%s escape_exhausted=%s escape_probes=%zu escape_cells=%zu "
        "escape_ms=%.1f departure_nodes=%zu/%zu departure_rejected_legs=%zu "
        "departure_probes=%zu/%zu departure_failure=%s(%.2f,%.2f,%.2f) "
        "seed_distance_m=%.2f seed_tolerance_m=%.3f",
        planner_telemetry.planned_on_revision, planner_telemetry.mission_epoch,
        plannerInputStatus3DName(planner_update.planner_input_status),
        planner_telemetry.input_failure,
        searchProgress3DName(planner_update.planner_progress),
        update.candidate.available ? "true" : "false",
        update.candidate.available
            ? spatialRouteCandidateSource3DName(update.candidate.source)
            : "none",
        planner_telemetry.search_state_reused ? "true" : "false",
        planner_telemetry.occupied_world_unchanged ? "true" : "false",
        planner_telemetry.incumbent_retained ? "true" : "false",
        planner_telemetry.execution_time_search_complete ? "true" : "false",
        update.candidate.point_count, planner_telemetry.expansions,
        planner_telemetry.execution_time_search_expansions,
        planner_telemetry.changed_occupied_voxels,
        planner_telemetry.affected_lattice_states,
        planner_telemetry.repair_lattice_states_processed,
        planner_telemetry.repair_lattice_states_pending,
        planner_telemetry.repair_pending ? "true" : "false",
        planner_telemetry.feasibility_attempted ? "true" : "false",
        planner_telemetry.feasibility_anchor_in_closed_component ? "true" : "false",
        planner_telemetry.feasibility_route_found ? "true" : "false",
        planner_telemetry.feasibility_expansions,
        planner_telemetry.feasibility_frontier_exhausted ? "true" : "false",
        planner_telemetry.feasibility_explored_nodes,
        planner_telemetry.feasibility_closest_goal_distance_m,
        planner_telemetry.feasibility_restarts,
        planner_telemetry.feasibility_invalidated_labels,
        planner_telemetry.feasibility_adopted_labels,
        planner_telemetry.feasibility_connector_sweeps,
        planner_telemetry.feasibility_last_invalid_segment,
        planner_telemetry.feasibility_anchor.x, planner_telemetry.feasibility_anchor.y,
        planner_telemetry.feasibility_anchor.z, planner_telemetry.records,
        planner_telemetry.open_entries, planner_telemetry.execution_time_search_records,
        planner_telemetry.execution_time_search_open_entries,
        planner_telemetry.shortcuts_applied, planner_telemetry.shortcut_checks,
        planner_telemetry.lattice_edge_queries,
        planner_telemetry.raw_edge_validation_checks,
        planner_telemetry.adaptive_edge_queries,
        planner_telemetry.adaptive_edges_in_extracted_path,
        planner_telemetry.maximum_queried_lattice_level,
        planner_telemetry.execution_time_search_objective_s,
        update.candidate.estimated_execution_time_s,
        update.candidate.estimated_translation_time_s,
        update.candidate.estimated_stationary_turn_time_s,
        planner_telemetry.world_update_ms, planner_telemetry.search_ms,
        planner_telemetry.feasibility_ms, planner_telemetry.repair_ms,
        planner_telemetry.spatial_search_ms, planner_telemetry.refinement_ms,
        planner_telemetry.world_diff_ms, planner_telemetry.world_install_ms,
        planner_telemetry.schedule_ms, planner_telemetry.schedule_ranking_ms,
        planner_telemetry.schedule_edges_forgotten,
        planner_telemetry.schedule_clearances_tightened,
        planner_telemetry.schedule_clearances_rederived,
        planner_telemetry.clearance_derivation_ms, planner_telemetry.raw_sweep_ms,
        planner_telemetry.departure_waypoint_count,
        planner_telemetry.departure_anchor_skip,
        planner_telemetry.goal_refined ? "true" : "false",
        planner_telemetry.escape_search_attempted ? "true" : "false",
        planner_telemetry.escape_search_found ? "true" : "false",
        planner_telemetry.escape_connection_active ? "true" : "false",
        planner_telemetry.escape_search_exhausted ? "true" : "false",
        planner_telemetry.escape_search_probes,
        planner_telemetry.escape_search_explored_cells,
        planner_telemetry.escape_search_ms, planner_telemetry.departure_valid_nodes,
        planner_telemetry.departure_candidate_nodes,
        planner_telemetry.departure_rejected_legs,
        planner_telemetry.departure_refinement_reachable,
        planner_telemetry.departure_refinement_probes,
        planner_telemetry.departure_first_failure_available ? "" : "none",
        planner_telemetry.departure_first_failure.x,
        planner_telemetry.departure_first_failure.y,
        planner_telemetry.departure_first_failure.z,
        planner_telemetry.departure_seed_distance_m,
        planner_telemetry.departure_seed_contact_tolerance_m);
  }

  if (update.status == RouteLifecycleAdvanceStatus3D::kContinuationQueued) {
    RCLCPP_INFO(
        get_logger(),
        "PERSISTENT_PLANNER3D stage=continuation "
        "queued=%s raw_revision=%" PRIu64 " search_generation=%" PRIu64
        " repair_generation=%" PRIu64 " repair_processed=%zu "
        "repair_pending=%zu feasibility_attempted=%s "
        "feasibility_found=%s feasibility_expansions=%zu "
        "route_planning_ms=%.3f",
        update.continuation_queued ? "true"
        : update.search_retired    ? "retired"
                                   : "not_queued",
        planner_update.planner_telemetry.planned_on_revision,
        planner_update.planner_telemetry.search_generation,
        planner_update.planner_telemetry.repair_generation,
        planner_update.planner_telemetry.repair_lattice_states_processed,
        planner_update.planner_telemetry.repair_lattice_states_pending,
        planner_update.planner_telemetry.feasibility_attempted ? "true" : "false",
        planner_update.planner_telemetry.feasibility_route_found ? "true" : "false",
        planner_update.planner_telemetry.feasibility_expansions,
        update.route_planning_ms);
    return;
  }

  if (update.geometry_optimization_fallback.has_value()) {
    const RouteRiskAnnotationResult3D& fallback =
        *update.geometry_optimization_fallback;
    const std::string_view reason = routeRiskAnnotationStatus3DName(fallback.status);
    RCLCPP_INFO(get_logger(),
                "STATIC_ROUTE_GEOMETRY status=fallback_to_lattice reason=%.*s "
                "failure=(%.2f,%.2f,%.2f)",
                static_cast<int>(reason.size()), reason.data(),
                fallback.failure_point.x, fallback.failure_point.y,
                fallback.failure_point.z);
  }

  const ProductionRouteActivationResult3D& activation = update.activation;
  const MaterializedRoute3D& candidate = activation.materialized;
  const RouteAdmissionReport3D& activation_report = activation.admission;
  if (candidate.candidate_generation != 0U &&
      (!activation_report.trajectory_validation.valid() ||
       activation.proposal.trajectory == nullptr)) {
    const std::size_t failed_sample =
        activation_report.trajectory_validation.sample_index;
    const Point3 failed_position =
        candidate.route != nullptr && failed_sample < candidate.route->size()
            ? (*candidate.route)[failed_sample].position
            : Point3{};
    RCLCPP_WARN(get_logger(),
                "COMPILED_TRAJECTORY valid=false reason=%s sample_index=%zu "
                "sample=(%.2f,%.2f,%.2f) route_generation=%" PRIu64,
                compiledTrajectoryFailureReason3DName(
                    activation_report.trajectory_validation.reason),
                failed_sample, failed_position.x, failed_position.y, failed_position.z,
                candidate.candidate_generation);
  }

  if (update.search_superseded_by_activation) {
    const char* const reason =
        activation_report.activation_status ==
                StaticRouteActivationStatus::kActivationSnapshotSuperseded
            ? "activation_snapshot_superseded"
            : "activation_world_raw_collision";
    RCLCPP_WARN(get_logger(),
                "PERSISTENT_PLANNER3D stage=continuation_cancelled "
                "reason=%s disposition=%s attempts=%zu search_raw_revision=%" PRIu64
                " activation_raw_revision=%" PRIu64 " search_generation=%" PRIu64,
                reason, routeCandidateDisposition3DName(update.candidate_disposition),
                update.activation_attempts,
                planner_update.planner_telemetry.planned_on_revision,
                activation_report.snapshot_raw_revision,
                planner_update.planner_telemetry.search_generation);
    // A retired initial search leaves the vehicle with no route at all. The
    // lifecycle gate replays replacement and extension searches itself; an
    // initial search has no base generation to replay from, so it is requested
    // again here against the current resident world instead of waiting for the
    // next observed-world publication.
    if (update.request.transaction != nullptr &&
        update.request.transaction->initial()) {
      const std::shared_ptr<const ExecutionPlan3D> resident_execution =
          execution_supervisor_.plan();
      const bool route_still_missing =
          resident_execution == nullptr ||
          resident_execution->routeGenerationHighWater() == 0U;
      if (route_still_missing) {
        WorldPipeline3D::ResidentLease resident = world_pipeline_->lockResident();
        bool replaced_pending = false;
        const bool queued = requestInitialRouteSearch3D(
            resident.world(), resident.telemetry(), replaced_pending);
        RCLCPP_INFO(get_logger(),
                    "PRODUCTION_MPPI_ROUTE3D status=initial_search_replayed "
                    "queued=%s replaced_pending=%s",
                    queued ? "true" : "false", replaced_pending ? "true" : "false");
      }
    }
  } else if (update.search_running) {
    RCLCPP_INFO(get_logger(),
                "PERSISTENT_PLANNER3D stage=continuation_after_incumbent "
                "queued=%s raw_revision=%" PRIu64 " search_generation=%" PRIu64,
                update.continuation_queued ? "true"
                : update.search_retired    ? "retired"
                                           : "not_queued",
                planner_update.planner_telemetry.planned_on_revision,
                planner_update.planner_telemetry.search_generation);
  }

  const PlannerTelemetry3D& plan = planner_update.planner_telemetry;
  const MaterializedRoute3D& materialized = activation.materialized;
  const ProductionRoutePipelineTelemetry3D& telemetry = activation.telemetry;
  const ProductionRouteMaterializationTelemetry3D& materialization_telemetry =
      telemetry.materialization;
  const RouteAdmissionReport3D& admission = activation.admission;
  const StaticRouteCandidateValidation& validation = admission.candidate_validation;
  const DynamicHandoffResult3D& handoff = admission.handoff;

  struct RouteAltitudeSpan {
    double minimum_z_m{0.0};
    double maximum_z_m{0.0};
    double first_z_m{0.0};
  } route_altitude_span;

  if (materialized.route != nullptr && !materialized.route->empty()) {
    route_altitude_span.first_z_m = materialized.route->front().position.z;
    route_altitude_span.minimum_z_m = route_altitude_span.first_z_m;
    route_altitude_span.maximum_z_m = route_altitude_span.first_z_m;
    for (const RouteSample3D& sample : *materialized.route) {
      route_altitude_span.minimum_z_m =
          std::min(route_altitude_span.minimum_z_m, sample.position.z);
      route_altitude_span.maximum_z_m =
          std::max(route_altitude_span.maximum_z_m, sample.position.z);
    }
  }
  const TrackingErrorTubeProfile3D* const tracking_profile =
      activation.trajectory != nullptr
          ? activation.trajectory->tracking_error_tube.get()
          : nullptr;
  const char* const planner_input =
      planner_update.planner_invoked
          ? plannerInputStatus3DName(planner_update.planner_input_status)
          : "not_invoked";
  const char* const planner_progress =
      planner_update.planner_invoked
          ? searchProgress3DName(planner_update.planner_progress)
          : "not_invoked";
  // Where along the candidate the raw world refused it, against the departure
  // the vehicle is committed to: a refusal beyond it is a block ahead the
  // search repairs, one within it a route the vehicle cannot enter.
  const double refusal_station_m =
      validation.status == StaticRouteCandidateStatus::kRawCollision &&
              materialized.route != nullptr &&
              validation.failure_segment_index < materialized.route->size()
          ? (*materialized.route)[validation.failure_segment_index].station_m
          : -1.0;
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_ROUTE3D planner=persistent_dstar_lite "
      "raw_revision=%" PRIu64 " mission_epoch=%" PRIu64
      " activation_raw_revision=%" PRIu64
      " input=%s progress=%s search_state_reused=%s "
      "time_search_complete=%s incumbent_retained=%s certified_pending=%s "
      "activation_status=%.*s route_certified=%s certification=%.*s "
      "activation_currentness=(resident=%s,objective=%s,raw=%s,execution=%s,"
      "candidate=%s,certification_execution=%s) "
      "trajectory_compile=%s/%s tracking_source_occupied=%" PRIu64
      " tracking_activation_occupied=%" PRIu64 " tracking_min_speed_mps=%.3f "
      "publication_status=%.*s "
      "validation=%.*s handoff=%s handoff_cross_track_m=%.2f "
      "handoff_min_clearance_m=%.2f handoff_critical_exposure_m=%.2f "
      "handoff_terminal_cross_track_m=%.2f route_z=[%.1f,%.1f] route_first_z=%.1f "
      "splice=%.*s "
      "certified_reserve=%.*s reserve_available_m=%.3f "
      "reserve_required_m=%.3f reserve_shortfall_m=%.3f "
      "route_reaches_mission_goal=%s route_generation=%" PRIu64
      " base_route_instance_id=%" PRIu64 " stitch_station_m=%.3f "
      "stitch_prefix=%s stitch_connector=%s "
      "points=%zu samples=%zu expansions=%zu time_expansions=%zu "
      "changed_occupied=%zu affected_states=%zu repair_processed=%zu "
      "repair_pending=%zu feasibility_attempted=%s feasibility_found=%s "
      "feasibility_expansions=%zu records=%zu open=%zu "
      "time_records=%zu time_open=%zu shortcuts=%zu/%zu "
      "clearance_centering=%zu/%zu departure_waypoint=%s departure_hull=%s "
      "edge_queries=%zu raw_edge_checks=%zu adaptive_edge_queries=%zu "
      "adaptive_path_edges=%zu maximum_adaptive_level=%zu "
      "path_length_m=%.3f refusal_station_m=%.2f departure_end_station_m=%.2f "
      "time_objective_s=%.3f eta_s=%.3f "
      "translation_s=%.3f turn_s=%.3f "
      "search_ms=%.3f route_planning_ms=%.3f validation_ms=%.3f "
      "smoothing_ms=%.3f raw_connector_validated=%s "
      "raw_suffix_validated=%s route_fingerprint=%" PRIu64,
      plan.planned_on_revision, plan.mission_epoch, admission.snapshot_raw_revision,
      planner_input, planner_progress, plan.search_state_reused ? "true" : "false",
      plan.execution_time_search_complete ? "true" : "false",
      plan.incumbent_retained ? "true" : "false",
      admission.certified_pending ? "true" : "false",
      static_cast<int>(
          staticRouteActivationStatusName(admission.activation_status).size()),
      staticRouteActivationStatusName(admission.activation_status).data(),
      admission.route_certified ? "true" : "false",
      static_cast<int>(
          routeCertificationStatus3DName(admission.route_certification).size()),
      routeCertificationStatus3DName(admission.route_certification).data(),
      admission.resident_world_snapshot_current ? "true" : "false",
      admission.objective_snapshot_current ? "true" : "false",
      admission.raw_snapshot_current ? "true" : "false",
      admission.execution_base_snapshot_current ? "true" : "false",
      admission.candidate_world_coherent ? "true" : "false",
      admission.certification_execution_base_current ? "true" : "false",
      admission.trajectory_compile_attempted ? "attempted" : "not_needed",
      admission.trajectory_compiled ? "compiled" : "not_compiled",
      admission.tracking_profile_source_occupied_fingerprint,
      admission.tracking_profile_activation_occupied_fingerprint,
      tracking_profile != nullptr ? tracking_profile->minimum_speed_limit_mps : -1.0,
      static_cast<int>(
          routePublicationStatus3DName(admission.assessment.publication.status).size()),
      routePublicationStatus3DName(admission.assessment.publication.status).data(),
      static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
      staticRouteCandidateStatusName(validation.status).data(),
      dynamicHandoffStatus3DName(handoff.status), handoff.cross_track_m,
      handoff.minimum_clearance_m, handoff.critical_exposure_m,
      handoff.terminal_cross_track_m, route_altitude_span.minimum_z_m,
      route_altitude_span.maximum_z_m, route_altitude_span.first_z_m,
      static_cast<int>(
          routeSpliceCertificationStatus3DName(admission.splice.status).size()),
      routeSpliceCertificationStatus3DName(admission.splice.status).data(),
      static_cast<int>(
          certifiedRouteReserveStatus3DName(admission.certified_reserve.status).size()),
      certifiedRouteReserveStatus3DName(admission.certified_reserve.status).data(),
      admission.certified_reserve.available_m, admission.certified_reserve.required_m,
      admission.certified_reserve.shortfall_m,
      materialized.reaches_mission_goal ? "true" : "false",
      materialized.candidate_generation,
      materialized.provenance.base_route_instance_id.value,
      materialized.provenance.base_stitch_station_m.value_or(-1.0),
      frozenRoutePrefixStatus3DName(materialized.provenance.stitch_prefix_status),
      frozenRoutePrefixStatus3DName(materialized.provenance.stitch_connector_status),
      materialized.provenance.candidate_points,
      materialized.route ? materialized.route->size() : 0U, plan.expansions,
      plan.execution_time_search_expansions, plan.changed_occupied_voxels,
      plan.affected_lattice_states, plan.repair_lattice_states_processed,
      plan.repair_lattice_states_pending, plan.feasibility_attempted ? "true" : "false",
      plan.feasibility_route_found ? "true" : "false", plan.feasibility_expansions,
      plan.records, plan.open_entries, plan.execution_time_search_records,
      plan.execution_time_search_open_entries, plan.shortcuts_applied,
      plan.shortcut_checks, plan.clearance_centering_moves,
      plan.clearance_centering_queries, plan.departure_waypoint_used ? "true" : "false",
      plan.departure_hull_fallback ? "true" : "false", plan.lattice_edge_queries,
      plan.raw_edge_validation_checks, plan.adaptive_edge_queries,
      plan.adaptive_edges_in_extracted_path, plan.maximum_queried_lattice_level,
      telemetry.planner.path_length_m, refusal_station_m,
      materialized.departure_end_station_m, plan.execution_time_search_objective_s,
      telemetry.planner.estimated_execution_time_s,
      telemetry.planner.estimated_translation_time_s,
      telemetry.planner.estimated_stationary_turn_time_s, planner_update.search_ms,
      update.route_planning_ms, materialization_telemetry.candidate_validation_ms,
      materialization_telemetry.route_smoothing_ms,
      admission.assessment.raw_validation.connector_validated ? "true" : "false",
      admission.assessment.raw_validation.suffix_validated ? "true" : "false",
      materialized.fingerprint);
  if (admission.certified_pending && activation.trajectory != nullptr &&
      activation.trajectory->route != nullptr &&
      activation.trajectory->route->size() >= 2U) {
    // The geometry the vehicle is about to fly. Without it a run's log records
    // that a route changed and how long it is, but not where it goes, and a
    // reversal between two passages cannot be told from a local adjustment
    // after the fact.
    constexpr std::size_t kMaximumLoggedRoutePoints{24U};
    const std::vector<RouteSample3D>& route = *activation.trajectory->route;
    const std::size_t stride =
        std::max<std::size_t>(1U, (route.size() + kMaximumLoggedRoutePoints - 1U) /
                                      kMaximumLoggedRoutePoints);
    std::string points;
    for (std::size_t index = 0U; index < route.size(); index += stride) {
      points += (points.empty() ? "" : " ");
      points += std::format("({:.1f},{:.1f},{:.1f})", route[index].position.x,
                            route[index].position.y, route[index].position.z);
    }
    if ((route.size() - 1U) % stride != 0U) {
      points += std::format(" ({:.1f},{:.1f},{:.1f})", route.back().position.x,
                            route.back().position.y, route.back().position.z);
    }
    RCLCPP_INFO(
        get_logger(),
        "ROUTE_GEOMETRY route_generation=%" PRIu64 " samples=%zu stride=%zu points=%s",
        materialized.candidate_generation, route.size(), stride, points.c_str());
  }
  if (activation.trajectory != nullptr && tracking_profile != nullptr &&
      tracking_profile->constrained_segment_count > 0U &&
      activation.trajectory->route != nullptr) {
    // Names where the route runs close to raw occupied evidence: every
    // constrained range caps the executable speed and shrinks the tube the
    // finite path has to stay inside.
    const std::string ranges = describeTrackingErrorTubeConstraints3D(
        *activation.trajectory->route, *tracking_profile, 8U);
    RCLCPP_INFO(get_logger(),
                "TRACKING_TUBE_PROFILE route_generation=%" PRIu64
                " constrained_segments=%zu minimum_speed_limit_mps=%.2f "
                "maximum_error_m=%.2f ranges=%s",
                materialized.candidate_generation,
                tracking_profile->constrained_segment_count,
                tracking_profile->minimum_speed_limit_mps,
                tracking_profile->maximum_tracking_error_m, ranges.c_str());
  }

  if (admission.successor_improvement_required ||
      admission.successor_compared_to_pending) {
    const std::string_view improvement_status =
        routeSuccessorImprovementStatus3DName(admission.successor_improvement.status);
    RCLCPP_INFO(get_logger(),
                "ROUTE_SUCCESSOR_IMPROVEMENT status=%.*s required=%s "
                "resident_pending=%s resident_remaining_s=%.3f "
                "candidate_remaining_s=%.3f absolute_improvement_s=%.3f "
                "relative_improvement=%.6f",
                static_cast<int>(improvement_status.size()), improvement_status.data(),
                admission.successor_improvement_required ? "true" : "false",
                admission.successor_compared_to_pending ? "true" : "false",
                admission.successor_improvement.resident_remaining_time_s,
                admission.successor_improvement.candidate_remaining_time_s,
                admission.successor_improvement.absolute_improvement_s,
                admission.successor_improvement.relative_improvement);
  }

  if (admission.certified_pending &&
      materialized.cooperative_passage_assignments != nullptr) {
    for (const CooperativePassageAssignment& assignment :
         *materialized.cooperative_passage_assignments) {
      RCLCPP_INFO(
          get_logger(),
          "COOPERATIVE_PASSAGE_ROUTE route_generation=%" PRIu64
          " span_index=%zu passage='%s' offset_interval_m=[%.2f,%.2f] "
          "secondary_interval_m=[%.2f,%.2f] raw_volume=%s status=%s",
          assignment.route_generation, assignment.span_index,
          assignment.passage_traversal_id.c_str(), assignment.minimum_lateral_offset_m,
          assignment.maximum_lateral_offset_m, assignment.minimum_secondary_offset_m,
          assignment.maximum_secondary_offset_m,
          assignment.passage_volume_raw_validated ? "true" : "false",
          cooperativePassageRouteStatusName(assignment.status));
    }
  }

  if (update.failed_search_latched) {
    const std::uint64_t failed_generation =
        transaction->replacement() ? transaction->request.base_route_generation : 0U;
    RCLCPP_INFO(
        get_logger(),
        "STATIC_ROUTE_SEARCH_OUTCOME status=failed_latched "
        "generation=%" PRIu64 " initial=%s planner_input=%s "
        "planner_progress=%s activation_status=%.*s candidate_status=%.*s "
        "start=(%.2f,%.2f,%.2f)",
        failed_generation, transaction->initial() ? "true" : "false", planner_input,
        planner_progress,
        static_cast<int>(
            staticRouteActivationStatusName(admission.activation_status).size()),
        staticRouteActivationStatusName(admission.activation_status).data(),
        static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
        staticRouteCandidateStatusName(validation.status).data(), search_start.x,
        search_start.y, search_start.z);
  }
}

bool ProductionMppiNode::requestInitialRouteSearch3D(
    const std::shared_ptr<const WorldSnapshot3D>& world,
    const ProductionWorldBuildTelemetry3D& world_telemetry, bool& replaced_pending) {
  replaced_pending = false;
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  if (world == nullptr || objective == nullptr) {
    return false;
  }
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(world, captureResidentPlannerWorld3D(*world),
                                     makeStaticRouteObjective(*objective),
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kInitial,
                                     },
                                     std::nullopt, RouteReleaseReason3D::kNone,
                                     get_clock()->now().nanoseconds());
  if (transaction == nullptr) {
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_ROUTE3D status=invalid_initial_transaction "
                 "raw_revision=%" PRIu64,
                 world->source_raw_revision);
    return false;
  }
  // A vehicle without a route asks for a search once per observed world; the
  // search's own continuation is already queued while it runs, and displacing
  // it with a fresh request would drop the world the continuation absorbs.
  const RoutePlanningEnqueueResult3D enqueue = route_lifecycle_coordinator_->enqueue(
      RoutePlanningRequest3D{
          .transaction = transaction,
          .world_telemetry = world_telemetry,
          .continuation_session = nullptr,
      },
      RoutePlanningQueuePolicy3D::kKeepPending);
  replaced_pending = enqueue.displaced.has_value();
  return enqueue.queued();
}

} // namespace drone_city_nav
