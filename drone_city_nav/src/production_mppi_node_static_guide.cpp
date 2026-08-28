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

void ProductionMppiNode::processGuideSearch3D(
    const ProductionMppiPreparedEsdf& world,
    const ProductionMppiNavigation& navigation) {
  const auto planning_started = std::chrono::steady_clock::now();
  const Point3 mission_goal =
      world.search_objective.available ? world.search_objective.goal : mission_goal_;
  const NavigationWorldCertificate3D planned_world_certificate =
      navigationWorldCertificate3D(world);
  const std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world =
      latest_raw_world_3d_.load(std::memory_order_acquire);
  const Point3 search_start{navigation.state.x, navigation.state.y, navigation.state.z};

  const std::shared_ptr<const ExecutionRouteSnapshot3D> search_execution_snapshot =
      execution_route_store_.snapshot();
  const CertifiedRouteSuffix3D* const search_active_route =
      search_execution_snapshot && search_execution_snapshot->route.has_value()
          ? search_execution_snapshot->route.operator->()
          : nullptr;
  ProductionRouteCandidateSet3D candidate_set = generateRouteCandidates3D(
      world, navigation, mission_goal, latest_raw_world, search_active_route);

  if (candidate_set.planner_invoked &&
      candidate_set.planner_result.status ==
          PersistentPlannerStatus3D::kSearchInProgress) {
    bool continuation_queued{false};
    {
      const std::scoped_lock lock{guide_queue_mutex_};
      if (!pending_guide_world_) {
        pending_guide_world_ =
            std::make_shared<const ProductionMppiPreparedEsdf>(world);
        continuation_queued = true;
      }
    }
    if (continuation_queued) {
      guide_queue_condition_.notify_all();
    }
    const double route_planning_ms = elapsedMilliseconds(planning_started);
    {
      const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
      static_route_planning_latency_tracker_.record(route_planning_ms, world.build_ms);
    }
    RCLCPP_INFO(get_logger(),
                "PERSISTENT_PLANNER3D stage=continuation "
                "queued=%s raw_revision=%" PRIu64 " search_generation=%" PRIu64
                " repair_generation=%" PRIu64 " route_planning_ms=%.3f",
                continuation_queued ? "true" : "newer_world_pending",
                candidate_set.planner_result.planned_on_revision,
                candidate_set.planner_result.search_generation,
                candidate_set.planner_result.repair_generation, route_planning_ms);
    return;
  }

  const std::uint64_t candidate_generation =
      candidate_set.candidates.empty() ? 0U : nextRouteGeneration3D();
  const ProductionRouteActivationSnapshot3D activation_snapshot =
      captureRouteActivationSnapshot3D();
  const CertifiedRouteSuffix3D* const activation_active_route =
      activation_snapshot.execution_snapshot &&
              activation_snapshot.execution_snapshot->route.has_value()
          ? activation_snapshot.execution_snapshot->route.operator->()
          : nullptr;

  ProductionRouteMaterialization3D materialization{.prepared = world};
  ProductionRouteActivationResult3D activation{.prepared = world};
  if (!candidate_set.candidates.empty() && candidate_generation != 0U) {
    const ProductionRouteSearchCandidate3D& candidate =
        candidate_set.candidates.front();
    materialization = materializeRouteCandidate3D(
        world, navigation, mission_goal, candidate, candidate_generation,
        activation_active_route, activation_snapshot.raw_world.get());
    materialization.prepared.route_search_ms = candidate_set.search_ms;
    activation = prepareRouteActivation3D(
        world, std::move(materialization.prepared), planned_world_certificate,
        materialization.validation, materialization.replacement_policy, mission_goal,
        candidate_generation, activation_snapshot);
    commitRouteActivation3D(world, activation_snapshot, candidate_generation,
                            activation);
  }
  activation.prepared.route_search_ms = candidate_set.search_ms;

  const bool recovery_without_active_route =
      (activation_snapshot.execution_snapshot == nullptr ||
       !activation_snapshot.execution_snapshot->route.has_value()) &&
      !activation.certified_pending;
  if (recovery_without_active_route) {
    std::uint64_t current =
        navigation_recovery_sequence_.load(std::memory_order_relaxed);
    while (current != std::numeric_limits<std::uint64_t>::max() &&
           !navigation_recovery_sequence_.compare_exchange_weak(
               current, current + 1U, std::memory_order_release,
               std::memory_order_relaxed)) {
    }
  }

  const double route_planning_ms = elapsedMilliseconds(planning_started);
  {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    static_route_planning_latency_tracker_.record(route_planning_ms, world.build_ms);
  }

  const PersistentPlannerResult3D& plan = candidate_set.planner_result;
  const ProductionMppiPreparedEsdf& prepared = activation.prepared;
  const StaticRouteCandidateValidation& validation = activation.validation;
  const mppi::StaticRouteHandoffResult& handoff = activation.handoff;
  const char* const planner_status = candidate_set.planner_invoked
                                         ? persistentPlannerStatus3DName(plan.status)
                                         : "not_invoked";
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_GUIDE3D planner=persistent_dstar_lite "
      "raw_revision=%" PRIu64 " mission_epoch=%" PRIu64
      " status=%s search_complete=%s search_state_reused=%s "
      "incumbent_retained=%s certified_pending=%s "
      "activation_status=%.*s publication_status=%.*s "
      "validation=%.*s handoff=%s splice=%.*s "
      "certified_reserve=%.*s reserve_available_m=%.3f "
      "reserve_required_m=%.3f reserve_shortfall_m=%.3f "
      "route_reaches_mission_goal=%s route_generation=%" PRIu64
      " base_route_instance_id=%" PRIu64 " stitch_station_m=%.3f "
      "points=%zu samples=%zu expansions=%zu changed_occupied=%zu "
      "affected_states=%zu records=%zu open=%zu shortcuts=%zu/%zu "
      "path_length_m=%.3f eta_s=%.3f translation_s=%.3f turn_s=%.3f "
      "search_ms=%.3f route_planning_ms=%.3f validation_ms=%.3f "
      "smoothing_ms=%.3f raw_connector_validated=%s "
      "raw_suffix_validated=%s route_fingerprint=%" PRIu64,
      plan.planned_on_revision, plan.mission_epoch, planner_status,
      plan.search_complete ? "true" : "false",
      plan.search_state_reused ? "true" : "false",
      plan.incumbent_retained ? "true" : "false",
      activation.certified_pending ? "true" : "false",
      static_cast<int>(
          staticRouteActivationStatusName(activation.activation_status).size()),
      staticRouteActivationStatusName(activation.activation_status).data(),
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
      plan.changed_occupied_voxels, plan.affected_lattice_states, plan.records,
      plan.open_entries, plan.shortcuts_applied, plan.shortcut_checks,
      plan.path_length_m, plan.estimated_execution_time_s,
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
    } else if (staticRouteSearchFailureLatchEligible(
                   search_request, resident_route_generation,
                   activation.world_compatible, activation.objective_matches)) {
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
      requestGuideRelease(GlobalGuideReleaseReason::kObjectiveChanged);
    }
  }
}

} // namespace drone_city_nav
