#include "drone_city_nav/static_route_extension.hpp"

#include <chrono>
#include <cinttypes>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_activation.hpp"
#include "production_mppi_route_materialization.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point started) noexcept {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                   started)
      .count();
}

} // namespace

void ProductionMppiNode::processRouteSearch3D(
    const ProductionMppiPreparedEsdf& world,
    const ProductionMppiNavigation& navigation) {
  const auto planning_started = std::chrono::steady_clock::now();
  const Point3 mission_goal =
      world.search_objective.available ? world.search_objective.goal : mission_goal_;
  const NavigationWorldCertificate3D planned_world_certificate =
      navigationWorldCertificate3D(world);
  const Point3 search_start{navigation.state.x, navigation.state.y, navigation.state.z};

  const std::shared_ptr<const ExecutionRouteSnapshot3D> search_execution_snapshot =
      execution_route_store_.snapshot();
  const CertifiedRouteSuffix3D* const search_active_route =
      search_execution_snapshot && search_execution_snapshot->route.has_value()
          ? search_execution_snapshot->route.operator->()
          : nullptr;
  const auto observe_recovery_episode = [this, &world] {
    const std::shared_ptr<const ExecutionRouteSnapshot3D> current_execution =
        execution_route_store_.snapshot();
    const std::shared_ptr<const PendingCertifiedRoute3D> current_pending =
        pending_certified_route_mailbox_.snapshot();
    const bool recovery_active =
        (current_execution == nullptr || !current_execution->route.has_value()) &&
        current_pending == nullptr;
    static_cast<void>(navigation_recovery_episodes_.observe(
        world.search_objective.mission_epoch, recovery_active));
  };
  ProductionRouteCandidateSet3D candidate_set =
      generateRouteCandidates3D(world, navigation, mission_goal, search_active_route);

  if (candidate_set.planner_invoked &&
      candidate_set.planner_result.status ==
          PersistentPlannerStatus3D::kSearchInProgress) {
    bool continuation_queued{false};
    {
      const std::scoped_lock lock{route_planning_queue_mutex_};
      if (!pending_route_planning_world_) {
        pending_route_planning_world_ =
            std::make_shared<const ProductionMppiPreparedEsdf>(world);
        continuation_queued = true;
      }
    }
    if (continuation_queued) {
      route_planning_queue_condition_.notify_all();
    } else {
      // A newer world already owns the single planning queue slot, so this
      // request can no longer deliver its continuation. Close its lifecycle
      // gate now; the recurring physical/no-route source will rebind the
      // request to the newer resident world instead of leaving a phantom
      // in-flight generation behind.
      finishStaticRouteSearch(world);
    }
    const double route_planning_ms = elapsedMilliseconds(planning_started);
    {
      const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
      static_route_planning_latency_tracker_.record(route_planning_ms, world.build_ms);
    }
    observe_recovery_episode();
    RCLCPP_INFO(get_logger(),
                "PERSISTENT_PLANNER3D stage=continuation "
                "queued=%s raw_revision=%" PRIu64 " search_generation=%" PRIu64
                " repair_generation=%" PRIu64 " repair_processed=%zu "
                "repair_pending=%zu feasibility_attempted=%s "
                "feasibility_found=%s feasibility_expansions=%zu "
                "route_planning_ms=%.3f",
                continuation_queued ? "true" : "newer_world_pending",
                candidate_set.planner_result.planned_on_revision,
                candidate_set.planner_result.search_generation,
                candidate_set.planner_result.repair_generation,
                candidate_set.planner_result.repair_lattice_states_processed,
                candidate_set.planner_result.repair_lattice_states_pending,
                candidate_set.planner_result.feasibility_attempted ? "true" : "false",
                candidate_set.planner_result.feasibility_route_found ? "true" : "false",
                candidate_set.planner_result.feasibility_expansions, route_planning_ms);
    return;
  }

  const std::uint64_t candidate_generation =
      candidate_set.candidates.empty() ? 0U : nextRouteGeneration3D();
  const ProductionRouteActivationSnapshot3D materialization_snapshot =
      captureRouteActivationSnapshot3D();
  const CertifiedRouteSuffix3D* const activation_active_route =
      materialization_snapshot.execution_snapshot &&
              materialization_snapshot.execution_snapshot->route.has_value()
          ? materialization_snapshot.execution_snapshot->route.operator->()
          : nullptr;

  ProductionRouteMaterialization3D materialization{.prepared = world};
  ProductionRouteActivationResult3D activation{.prepared = world};
  if (!candidate_set.candidates.empty() && candidate_generation != 0U) {
    const ProductionRouteSearchCandidate3D& candidate =
        candidate_set.candidates.front();
    materialization = materializeRouteCandidate3D(
        world, navigation, mission_goal, candidate, candidate_generation,
        activation_active_route, materialization_snapshot.raw_world.get());
    materialization.prepared.route_search_ms = candidate_set.search_ms;
  }
  // Materialization owns the expensive spatial validation. Capture the
  // transaction base afterwards so it is not stale before activation begins.
  const ProductionRouteActivationSnapshot3D activation_snapshot =
      captureRouteActivationSnapshot3D();
  if (!candidate_set.candidates.empty() && candidate_generation != 0U) {
    activation = prepareRouteActivation3D(
        world, std::move(materialization.prepared), planned_world_certificate,
        materialization.validation, materialization.replacement_policy, mission_goal,
        candidate_generation, activation_snapshot);
    commitRouteActivation3D(world, activation_snapshot, candidate_generation,
                            activation);
  }
  activation.prepared.route_search_ms = candidate_set.search_ms;

  observe_recovery_episode();

  const double route_planning_ms = elapsedMilliseconds(planning_started);
  {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    static_route_planning_latency_tracker_.record(route_planning_ms, world.build_ms);
  }

  const PersistentPlannerResult3D& plan = candidate_set.planner_result;
  const ProductionMppiPreparedEsdf& prepared = activation.prepared;
  const StaticRouteCandidateValidation& validation = activation.validation;
  const mppi::StaticRouteHandoffResult& handoff = activation.handoff;
  const TrackingErrorTubeProfile3D* const tracking_profile =
      prepared.compiled_route_geometry != nullptr
          ? prepared.compiled_route_geometry->tracking_error_tube.get()
          : nullptr;
  const char* const planner_status = candidate_set.planner_invoked
                                         ? persistentPlannerStatus3DName(plan.status)
                                         : "not_invoked";
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_ROUTE3D planner=persistent_dstar_lite "
      "raw_revision=%" PRIu64 " mission_epoch=%" PRIu64
      " activation_raw_revision=%" PRIu64
      " status=%s search_complete=%s search_state_reused=%s "
      "time_search_complete=%s incumbent_retained=%s certified_pending=%s "
      "activation_status=%.*s route_certified=%s "
      "activation_currentness=(resident=%s,objective=%s,raw=%s,execution=%s,"
      "candidate=%s,certification_execution=%s) "
      "tracking_recompile=%s/%s tracking_source_occupied=%" PRIu64
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
      plan.planned_on_revision, plan.mission_epoch, activation.snapshot_raw_revision,
      planner_status, plan.search_complete ? "true" : "false",
      plan.search_state_reused ? "true" : "false",
      plan.execution_time_search_complete ? "true" : "false",
      plan.incumbent_retained ? "true" : "false",
      activation.certified_pending ? "true" : "false",
      static_cast<int>(
          staticRouteActivationStatusName(activation.activation_status).size()),
      staticRouteActivationStatusName(activation.activation_status).data(),
      activation.route_certified ? "true" : "false",
      activation.resident_world_snapshot_current ? "true" : "false",
      activation.objective_snapshot_current ? "true" : "false",
      activation.raw_snapshot_current ? "true" : "false",
      activation.execution_base_snapshot_current ? "true" : "false",
      activation.candidate_world_coherent ? "true" : "false",
      activation.certification_execution_base_current ? "true" : "false",
      activation.tracking_geometry_recompile_attempted ? "attempted" : "not_needed",
      activation.tracking_geometry_recompiled ? "compiled" : "not_compiled",
      activation.tracking_geometry_source_occupied_fingerprint,
      activation.tracking_geometry_activation_occupied_fingerprint,
      tracking_profile != nullptr ? tracking_profile->minimum_speed_limit_mps : -1.0,
      static_cast<int>(
          routePublicationStatus3DName(activation.assessment.publication.status)
              .size()),
      routePublicationStatus3DName(activation.assessment.publication.status).data(),
      static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
      staticRouteCandidateStatusName(validation.status).data(),
      mppi::staticRouteHandoffStatusName(handoff.status),
      static_cast<int>(
          routeSpliceCertificationStatus3DName(activation.splice.status).size()),
      routeSpliceCertificationStatus3DName(activation.splice.status).data(),
      static_cast<int>(
          certifiedRouteReserveStatus3DName(prepared.certified_route_reserve_status)
              .size()),
      certifiedRouteReserveStatus3DName(prepared.certified_route_reserve_status).data(),
      prepared.certified_route_reserve_available_m,
      prepared.certified_route_reserve_required_m,
      prepared.certified_route_reserve_shortfall_m,
      prepared.route_reaches_mission_goal ? "true" : "false", prepared.route_generation,
      prepared.planning_search_base_route_instance_id.value,
      prepared.planning_search_base_stitch_station_m.value_or(-1.0), plan.points.size(),
      prepared.route_3d ? prepared.route_3d->size() : 0U, plan.expansions,
      plan.execution_time_search_expansions, plan.changed_occupied_voxels,
      plan.affected_lattice_states, plan.repair_lattice_states_processed,
      plan.repair_lattice_states_pending, plan.feasibility_attempted ? "true" : "false",
      plan.feasibility_route_found ? "true" : "false", plan.feasibility_expansions,
      plan.records, plan.open_entries, plan.execution_time_search_records,
      plan.execution_time_search_open_entries, plan.shortcuts_applied,
      plan.shortcut_checks, plan.lattice_edge_queries, plan.raw_edge_validation_checks,
      plan.adaptive_edge_queries, plan.adaptive_edges_in_extracted_path,
      plan.maximum_queried_lattice_level, plan.path_length_m,
      plan.execution_time_search_objective_s, plan.estimated_execution_time_s,
      plan.estimated_translation_time_s, plan.estimated_stationary_turn_time_s,
      candidate_set.search_ms, route_planning_ms, prepared.candidate_validation_ms,
      prepared.route_smoothing_ms,
      activation.assessment.raw_validation.connector_validated ? "true" : "false",
      activation.assessment.raw_validation.suffix_validated ? "true" : "false",
      prepared.route_fingerprint);

  if (activation.certified_pending &&
      prepared.cooperative_passage_assignments != nullptr) {
    for (const CooperativePassageAssignment& assignment :
         *prepared.cooperative_passage_assignments) {
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

  const bool initial_route_search = !world.static_route_extension_request &&
                                    !world.static_route_replan_request &&
                                    world.route_generation == 0U;
  if (world.static_route_replan_request || initial_route_search) {
    const StaticRouteSearchRequestIdentity search_request =
        identifyStaticRouteSearchRequest(world.route_generation,
                                         world.static_route_extension_request,
                                         world.static_route_extension_base_generation,
                                         world.static_route_replan_request,
                                         world.static_route_replan_base_generation);
    const std::shared_ptr<const ExecutionRouteSnapshot3D> resident_execution =
        execution_route_store_.snapshot();
    const std::uint64_t resident_route_generation =
        resident_execution != nullptr ? resident_execution->routeGenerationHighWater()
                                      : 0U;
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    if (activation.certified_pending) {
      static_route_failed_search_latch_.clear();
    } else if (staticRouteSearchFailureLatchEligible(search_request,
                                                     resident_route_generation)) {
      const std::uint64_t failed_generation =
          world.static_route_replan_request ? world.static_route_replan_base_generation
                                            : 0U;
      static_route_failed_search_latch_.recordFailure(StaticRouteSearchContext{
          .base_route_generation = failed_generation,
          .search_start = search_start,
          .objective = world.search_objective,
          .minimum_tracking_sample_sequence = activation.required_objective_sample,
          .stamp_ns = get_clock()->now().nanoseconds(),
      });
      RCLCPP_INFO(
          get_logger(),
          "STATIC_ROUTE_SEARCH_OUTCOME status=failed_latched "
          "generation=%" PRIu64 " initial=%s planner_status=%s "
          "activation_status=%.*s candidate_status=%.*s "
          "start=(%.2f,%.2f,%.2f)",
          failed_generation, initial_route_search ? "true" : "false", planner_status,
          static_cast<int>(
              staticRouteActivationStatusName(activation.activation_status).size()),
          staticRouteActivationStatusName(activation.activation_status).data(),
          static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
          staticRouteCandidateStatusName(validation.status).data(), search_start.x,
          search_start.y, search_start.z);
    }
  } else if (activation.certified_pending) {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    static_route_failed_search_latch_.clear();
  }

  finishStaticRouteSearch(world, activation.certified_pending);
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
      const std::scoped_lock lock{world_generation_publication_mutex_,
                                  esdf_state_mutex_};
      if (prepared_esdf_ && productionWorldGenerationCoherent(*prepared_esdf_)) {
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
      requestRouteRelease(RouteReleaseReason3D::kObjectiveChanged);
    }
  }
}

} // namespace drone_city_nav
