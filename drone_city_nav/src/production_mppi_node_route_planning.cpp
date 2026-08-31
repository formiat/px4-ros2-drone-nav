#include "drone_city_nav/route_risk_annotation_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"

#include <chrono>
#include <cinttypes>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_activation.hpp"
#include "production_mppi_route_world.hpp"
#include "route_materializer_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point started) noexcept {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                   started)
      .count();
}

} // namespace

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
  finishStaticRouteSearch(*transaction);
}

void ProductionMppiNode::processRouteSearch3D(RoutePlanningUpdateEvent3D event) {
  const std::shared_ptr<const PlannerSearchTransaction3D>& transaction =
      event.request.transaction;
  if (transaction == nullptr) {
    return;
  }
  const ProductionWorldBuildTelemetry3D& world_telemetry =
      event.request.world_telemetry;
  const RoutePlannerVehicleState3D& vehicle_state = event.vehicle_state;
  RoutePlannerUpdate3D planner_update = std::move(event.update);
  const auto planning_started = std::chrono::steady_clock::now();
  const Point3 mission_goal = transaction->objective.goal;
  const NavigationWorldCertificate3D planned_world_certificate =
      navigationWorldCertificate3D(*transaction->world);

  const auto observe_recovery_episode = [this, &transaction] {
    const RouteExecutionManagerSnapshot3D execution_state =
        route_execution_manager_.snapshot();
    const std::shared_ptr<const ExecutionPlan3D> execution_plan =
        execution_state.plan();
    const bool recovery_active =
        (execution_plan == nullptr || execution_plan->route() == nullptr) &&
        execution_state.pending == nullptr;
    static_cast<void>(navigation_recovery_episodes_.observe(
        transaction->objective.mission_epoch, recovery_active));
  };
  const Point3 search_start = planner_update.planner_session
                                  ? planner_update.planner_session->search_start
                                  : vehicle_state.position;

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
    const SpatialRouteCandidate3D* const improved =
        planner_update.improved_incumbent
            ? std::addressof(planner_update.improved_incumbent->spatial_route)
            : nullptr;
    RCLCPP_INFO(
        get_logger(),
        "PERSISTENT_PLANNER3D stage=complete raw_revision=%" PRIu64
        " mission_epoch=%" PRIu64 " input=%s progress=%s publishable=%s "
        "source=%s reused=%s occupied_unchanged=%s incumbent_retained=%s "
        "time_search_complete=%s points=%zu expansions=%zu time_expansions=%zu "
        "changed_occupied=%zu affected_states=%zu repair_processed=%zu "
        "repair_pending=%zu repair_in_progress=%s feasibility_attempted=%s "
        "feasibility_found=%s feasibility_expansions=%zu records=%zu open=%zu "
        "time_records=%zu time_open=%zu shortcuts=%zu/%zu edge_queries=%zu "
        "raw_edge_checks=%zu adaptive_edge_queries=%zu adaptive_path_edges=%zu "
        "maximum_adaptive_level=%zu time_objective_s=%.3f eta_s=%.3f "
        "translation_s=%.3f turn_s=%.3f world_update_ms=%.3f search_ms=%.3f",
        planner_telemetry.planned_on_revision, planner_telemetry.mission_epoch,
        plannerInputStatus3DName(planner_update.planner_input_status),
        searchProgress3DName(planner_update.planner_progress),
        improved != nullptr ? "true" : "false",
        improved != nullptr ? spatialRouteCandidateSource3DName(improved->source)
                            : "none",
        planner_telemetry.search_state_reused ? "true" : "false",
        planner_telemetry.occupied_world_unchanged ? "true" : "false",
        planner_telemetry.incumbent_retained ? "true" : "false",
        planner_telemetry.execution_time_search_complete ? "true" : "false",
        improved != nullptr ? improved->points.size() : 0U,
        planner_telemetry.expansions,
        planner_telemetry.execution_time_search_expansions,
        planner_telemetry.changed_occupied_voxels,
        planner_telemetry.affected_lattice_states,
        planner_telemetry.repair_lattice_states_processed,
        planner_telemetry.repair_lattice_states_pending,
        planner_telemetry.repair_pending ? "true" : "false",
        planner_telemetry.feasibility_attempted ? "true" : "false",
        planner_telemetry.feasibility_route_found ? "true" : "false",
        planner_telemetry.feasibility_expansions, planner_telemetry.records,
        planner_telemetry.open_entries, planner_telemetry.execution_time_search_records,
        planner_telemetry.execution_time_search_open_entries,
        planner_telemetry.shortcuts_applied, planner_telemetry.shortcut_checks,
        planner_telemetry.lattice_edge_queries,
        planner_telemetry.raw_edge_validation_checks,
        planner_telemetry.adaptive_edge_queries,
        planner_telemetry.adaptive_edges_in_extracted_path,
        planner_telemetry.maximum_queried_lattice_level,
        planner_telemetry.execution_time_search_objective_s,
        improved != nullptr ? improved->estimated_execution_time_s : 0.0,
        improved != nullptr ? improved->estimated_translation_time_s : 0.0,
        improved != nullptr ? improved->estimated_stationary_turn_time_s : 0.0,
        planner_telemetry.world_update_ms, planner_telemetry.search_ms);
  }

  const bool search_running =
      planner_update.planner_invoked && planner_update.dispatch.continue_search;
  const bool improved_incumbent_available =
      planner_update.improved_incumbent.has_value();
  const auto queue_continuation = [this, &planner_update, &transaction,
                                   &world_telemetry]() {
    if (!planner_update.planner_session) {
      return false;
    }
    return route_planning_coordinator_
        ->enqueue(RoutePlanningRequest3D{
            .transaction = transaction,
            .world_telemetry = world_telemetry,
            .continuation_session = planner_update.planner_session,
        })
        .queued();
  };
  bool continuation_queued{false};
  if (search_running && !improved_incumbent_available) {
    continuation_queued = queue_continuation();
    const double continuation_planning_ms = elapsedMilliseconds(planning_started);
    RCLCPP_INFO(
        get_logger(),
        "PERSISTENT_PLANNER3D stage=continuation "
        "queued=%s raw_revision=%" PRIu64 " search_generation=%" PRIu64
        " repair_generation=%" PRIu64 " repair_processed=%zu "
        "repair_pending=%zu feasibility_attempted=%s "
        "feasibility_found=%s feasibility_expansions=%zu "
        "route_planning_ms=%.3f",
        continuation_queued ? "true" : "newer_world_pending",
        planner_update.planner_telemetry.planned_on_revision,
        planner_update.planner_telemetry.search_generation,
        planner_update.planner_telemetry.repair_generation,
        planner_update.planner_telemetry.repair_lattice_states_processed,
        planner_update.planner_telemetry.repair_lattice_states_pending,
        planner_update.planner_telemetry.feasibility_attempted ? "true" : "false",
        planner_update.planner_telemetry.feasibility_route_found ? "true" : "false",
        planner_update.planner_telemetry.feasibility_expansions,
        continuation_planning_ms);
    if (!continuation_queued) {
      // A newer world owns the single queue slot. Close this request's gate;
      // the newer request will continue against its own immutable snapshot.
      finishStaticRouteSearch(*transaction);
    }
    {
      const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
      static_route_planning_latency_tracker_.record(continuation_planning_ms,
                                                    world_telemetry.build_ms);
    }
    observe_recovery_episode();
    return;
  }

  const std::uint64_t candidate_generation =
      improved_incumbent_available ? nextRouteGeneration3D() : 0U;
  const ProductionRouteActivationSnapshot3D materialization_snapshot =
      captureRouteActivationSnapshot3D();
  const std::shared_ptr<const ExecutionPlan3D> materialization_execution =
      materialization_snapshot.execution_authority != nullptr
          ? materialization_snapshot.execution_authority->plan()
          : nullptr;
  const CertifiedRouteSuffix3D* const activation_active_route =
      materialization_execution != nullptr ? materialization_execution->route()
                                           : nullptr;
  const std::shared_ptr<const CertifiedRouteSuffix3D> materialization_active_route =
      activation_active_route != nullptr
          ? std::make_shared<const CertifiedRouteSuffix3D>(*activation_active_route)
          : nullptr;

  ProductionRouteMaterialization3D materialization;
  materialization.route.world = transaction->world;
  materialization.route.objective = transaction->objective;
  materialization.route.candidate_generation = candidate_generation;
  materialization.telemetry.world_build = world_telemetry;
  materialization.telemetry.route_search_ms = planner_update.search_ms;
  ProductionRouteActivationResult3D activation;
  activation.materialized = materialization.route;
  activation.telemetry = materialization.telemetry;
  if (improved_incumbent_available && candidate_generation != 0U) {
    materialization = route_materializer_->materialize(RouteMaterializationRequest3D{
        .transaction = transaction,
        .world_telemetry = world_telemetry,
        .current_position = vehicle_state.position,
        .candidate = std::move(*planner_update.improved_incumbent),
        .candidate_generation = candidate_generation,
        .active_route = materialization_active_route,
        .activation_raw_world = materialization_snapshot.raw_world,
    });
    materialization.telemetry.route_search_ms = planner_update.search_ms;
    if (materialization.geometry_optimization_fallback.has_value()) {
      const RouteRiskAnnotationResult3D fallback =
          materialization.geometry_optimization_fallback.value();
      const std::string_view reason = routeRiskAnnotationStatus3DName(fallback.status);
      RCLCPP_INFO(get_logger(),
                  "STATIC_ROUTE_GEOMETRY status=fallback_to_lattice reason=%.*s "
                  "failure=(%.2f,%.2f,%.2f)",
                  static_cast<int>(reason.size()), reason.data(),
                  fallback.failure_point.x, fallback.failure_point.y,
                  fallback.failure_point.z);
    }
  }
  // Materialization owns the expensive spatial validation. Capture the
  // transaction base afterwards so it is not stale before activation begins.
  const ProductionRouteActivationSnapshot3D activation_snapshot =
      captureRouteActivationSnapshot3D();
  if (improved_incumbent_available && candidate_generation != 0U) {
    const StaticRouteCandidateValidation materialization_validation =
        materialization.validation;
    const StaticRouteReplacementPolicy replacement_policy =
        materialization.replacement_policy;
    activation = prepareRouteActivation3D(
        *transaction, std::move(materialization), planned_world_certificate,
        materialization_validation, replacement_policy, mission_goal,
        candidate_generation, activation_snapshot);
    commitRouteActivation3D(*transaction, activation_snapshot, candidate_generation,
                            activation);
  }
  activation.telemetry.route_search_ms = planner_update.search_ms;

  if (search_running) {
    continuation_queued = queue_continuation();
    RCLCPP_INFO(get_logger(),
                "PERSISTENT_PLANNER3D stage=continuation_after_incumbent "
                "queued=%s raw_revision=%" PRIu64 " search_generation=%" PRIu64,
                continuation_queued ? "true" : "newer_world_pending",
                planner_update.planner_telemetry.planned_on_revision,
                planner_update.planner_telemetry.search_generation);
  }

  observe_recovery_episode();

  const double route_planning_ms = elapsedMilliseconds(planning_started);
  {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    static_route_planning_latency_tracker_.record(route_planning_ms,
                                                  world_telemetry.build_ms);
  }

  const PlannerTelemetry3D& plan = planner_update.planner_telemetry;
  const MaterializedRoute3D& materialized = activation.materialized;
  const ProductionRoutePipelineTelemetry3D& telemetry = activation.telemetry;
  const ProductionRouteMaterializationTelemetry3D& materialization_telemetry =
      telemetry.materialization;
  const RouteAdmissionReport3D& admission = activation.admission;
  const StaticRouteCandidateValidation& validation = admission.candidate_validation;
  const mppi::StaticRouteHandoffResult& handoff = admission.handoff;
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
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_ROUTE3D planner=persistent_dstar_lite "
      "raw_revision=%" PRIu64 " mission_epoch=%" PRIu64
      " activation_raw_revision=%" PRIu64
      " input=%s progress=%s search_state_reused=%s "
      "time_search_complete=%s incumbent_retained=%s certified_pending=%s "
      "activation_status=%.*s route_certified=%s "
      "activation_currentness=(resident=%s,objective=%s,raw=%s,execution=%s,"
      "candidate=%s,certification_execution=%s) "
      "trajectory_compile=%s/%s tracking_source_occupied=%" PRIu64
      " tracking_activation_occupied=%" PRIu64 " tracking_min_speed_mps=%.3f "
      "publication_status=%.*s "
      "validation=%.*s handoff=%s splice=%.*s "
      "certified_reserve=%.*s reserve_available_m=%.3f "
      "reserve_required_m=%.3f reserve_shortfall_m=%.3f "
      "route_reaches_mission_goal=%s route_generation=%" PRIu64
      " base_route_instance_id=%" PRIu64 " stitch_station_m=%.3f "
      "points=%zu samples=%zu expansions=%zu time_expansions=%zu "
      "changed_occupied=%zu affected_states=%zu repair_processed=%zu "
      "repair_pending=%zu feasibility_attempted=%s feasibility_found=%s "
      "feasibility_expansions=%zu records=%zu open=%zu "
      "time_records=%zu time_open=%zu shortcuts=%zu/%zu "
      "edge_queries=%zu raw_edge_checks=%zu adaptive_edge_queries=%zu "
      "adaptive_path_edges=%zu maximum_adaptive_level=%zu "
      "path_length_m=%.3f time_objective_s=%.3f eta_s=%.3f "
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
      mppi::staticRouteHandoffStatusName(handoff.status),
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
      materialized.provenance.candidate_points,
      materialized.route ? materialized.route->size() : 0U, plan.expansions,
      plan.execution_time_search_expansions, plan.changed_occupied_voxels,
      plan.affected_lattice_states, plan.repair_lattice_states_processed,
      plan.repair_lattice_states_pending, plan.feasibility_attempted ? "true" : "false",
      plan.feasibility_route_found ? "true" : "false", plan.feasibility_expansions,
      plan.records, plan.open_entries, plan.execution_time_search_records,
      plan.execution_time_search_open_entries, plan.shortcuts_applied,
      plan.shortcut_checks, plan.lattice_edge_queries, plan.raw_edge_validation_checks,
      plan.adaptive_edge_queries, plan.adaptive_edges_in_extracted_path,
      plan.maximum_queried_lattice_level, telemetry.planner.path_length_m,
      plan.execution_time_search_objective_s,
      telemetry.planner.estimated_execution_time_s,
      telemetry.planner.estimated_translation_time_s,
      telemetry.planner.estimated_stationary_turn_time_s, planner_update.search_ms,
      route_planning_ms, materialization_telemetry.candidate_validation_ms,
      materialization_telemetry.route_smoothing_ms,
      admission.assessment.raw_validation.connector_validated ? "true" : "false",
      admission.assessment.raw_validation.suffix_validated ? "true" : "false",
      materialized.fingerprint);

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

  const bool initial_route_search = transaction->initial();
  if (transaction->replacement() || initial_route_search) {
    const StaticRouteSearchRequestIdentity& search_request = transaction->request;
    const std::shared_ptr<const ExecutionPlan3D> resident_execution =
        route_execution_manager_.plan();
    const std::uint64_t resident_route_generation =
        resident_execution != nullptr ? resident_execution->routeGenerationHighWater()
                                      : 0U;
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    if (admission.certified_pending) {
      static_route_failed_search_latch_.clear();
    } else if (!search_running && staticRouteSearchFailureLatchEligible(
                                      search_request, resident_route_generation)) {
      const std::uint64_t failed_generation =
          transaction->replacement() ? transaction->request.base_route_generation : 0U;
      static_route_failed_search_latch_.recordFailure(StaticRouteSearchContext{
          .base_route_generation = failed_generation,
          .search_start = search_start,
          .objective = transaction->objective,
          .minimum_tracking_sample_sequence = admission.required_objective_sample,
          .stamp_ns = get_clock()->now().nanoseconds(),
      });
      RCLCPP_INFO(
          get_logger(),
          "STATIC_ROUTE_SEARCH_OUTCOME status=failed_latched "
          "generation=%" PRIu64 " initial=%s planner_input=%s "
          "planner_progress=%s "
          "activation_status=%.*s candidate_status=%.*s "
          "start=(%.2f,%.2f,%.2f)",
          failed_generation, initial_route_search ? "true" : "false", planner_input,
          planner_progress,
          static_cast<int>(
              staticRouteActivationStatusName(admission.activation_status).size()),
          staticRouteActivationStatusName(admission.activation_status).data(),
          static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
          staticRouteCandidateStatusName(validation.status).data(), search_start.x,
          search_start.y, search_start.z);
    }
  } else if (admission.certified_pending) {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    static_route_failed_search_latch_.clear();
  }

  if (!continuation_queued) {
    finishStaticRouteSearch(*transaction, admission.certified_pending);
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
    const std::shared_ptr<const ExecutionPlan3D> resident_execution =
        route_execution_manager_.plan();
    if (resident_execution != nullptr && resident_execution->route() != nullptr) {
      resident_route_objective =
          resident_execution->route()->identity.proposal.objective;
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
      requestRouteRelease(RouteReleaseReason3D::kObjectiveChanged);
    }
  }
}

} // namespace drone_city_nav
