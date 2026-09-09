#include "drone_city_nav/trajectory_compiler_3d.hpp"

#include <cinttypes>
#include <memory>
#include <stdexcept>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::maybeRequestStaticRouteExtensionFromExecution(
    const std::shared_ptr<const WorldSnapshot3D>& world,
    const ProductionWorldBuildTelemetry3D& world_build,
    const ProductionRouteExecutionSelection3D& route_execution,
    const ProductionMppiNavigation& navigation, const std::int64_t now_ns) {
  if (route_execution.source_snapshot == nullptr) {
    return;
  }
  const ExecutionPlan3D& source = *route_execution.source_snapshot;
  const CertifiedRouteSuffix3D* const active_route = source.route();
  if (route_execution.physical_trajectory_invalidated && active_route != nullptr &&
      executionRouteAcceptsCertifiedReplacement3D(source)) {
    // A physical invalidation is latched for the resident generation. Search
    // must outlive its finite braking tail: a rejected candidate is retried by
    // the failed-search latch while the safe resident owner brakes or holds.
    requestStaticRouteReplan(RouteReleaseReason3D::kBlocked,
                             active_route->identity.generation);
    return;
  }
  // A stop owns the vehicle while its route stays resident, so a successor can
  // hand off from it. The invalidation that produced the stop is latched for
  // the tick it was observed on, and nothing asked for the successor again
  // while the vehicle braked: the request above stopped, the deferred replan
  // was consumed, and a failed search then sat latched with nothing to
  // re-evaluate it. One recorded flight braked and rested for two and a half
  // seconds with the planner idle, a quarter of that run's no-route time. The
  // request is repeated for as long as the stop owns the vehicle; the
  // failed-search latch decides how often a search actually runs.
  if (source.stopExecution() != nullptr && active_route != nullptr &&
      executionRouteAcceptsCertifiedReplacement3D(source)) {
    requestStaticRouteReplan(RouteReleaseReason3D::kBlocked,
                             active_route->identity.generation);
    return;
  }
  const bool suspended_route =
      source.phase() == ExecutionRoutePhase3D::kAwaitingSuccessor &&
      source.route() != nullptr && source.finiteExecution() == nullptr &&
      source.brakingFallback() == nullptr;
  if (source.phase() != ExecutionRoutePhase3D::kFollowing && !suspended_route) {
    return;
  }
  if (active_route == nullptr) {
    return;
  }
  const RouteProjection3D projection = projectOntoRoute3DWithinStationWindow(
      *active_route->geometry->route,
      Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      active_route->progress.station_m, active_route->endStationM());
  if (!projection.valid ||
      (config_.planning.optional_constraints.route_cross_track_constraints_enabled &&
       projection.distance_m >
           config_.planning.route_tracking_policy.maximum_cross_track_m)) {
    return;
  }
  const std::shared_ptr<const CertifiedRouteSuffix3D> active_route_snapshot{
      route_execution.source_snapshot, active_route};
  maybeRequestStaticRouteExtension(
      world, world_build, active_route_snapshot, navigation,
      RouteProgressProjection3D{
          .valid = true,
          .station_m = projection.station_m,
          .total_length_m = active_route->endStationM(),
          .remaining_m = projection.remaining_m,
          .cross_track_m = projection.distance_m,
          .point = {projection.point.x, projection.point.y},
      },
      now_ns);
}

void ProductionMppiNode::maybeRequestStaticRouteExtension(
    const std::shared_ptr<const WorldSnapshot3D>& world,
    const ProductionWorldBuildTelemetry3D& world_build,
    const std::shared_ptr<const CertifiedRouteSuffix3D>& active_route,
    const ProductionMppiNavigation& navigation,
    const RouteProgressProjection3D& route_projection, const std::int64_t now_ns) {
  if (active_route == nullptr) {
    return;
  }
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  const RouteLifecycleExtensionOutcome3D outcome =
      route_lifecycle_coordinator_->requestExtension(RouteLifecycleExtensionRequest3D{
          .world = world,
          .world_telemetry = world_build,
          .active_route = active_route,
          .navigation = navigation,
          .objective = objective,
          .route_projection = route_projection,
          .stamp_ns = now_ns,
          .pending_successor = execution_supervisor_.pending() != nullptr,
      });
  if (outcome.status == RouteLifecycleExtensionStatus3D::kInvalidTransaction) {
    RCLCPP_ERROR(get_logger(),
                 "STATIC_ROUTE_EXTENSION_REQUEST status=rejected_invalid_transaction "
                 "generation=%" PRIu64,
                 outcome.route_generation);
    return;
  }
  if (outcome.status == RouteLifecycleExtensionStatus3D::kRouteQueueBusy) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "STATIC_ROUTE_EXTENSION_REQUEST status=deferred_route_queue_busy "
        "generation=%" PRIu64 " station_m=%.2f remaining_m=%.2f",
        outcome.route_generation, outcome.station_m, outcome.remaining_m);
    return;
  }
  if (outcome.status == RouteLifecycleExtensionStatus3D::kWorldRefreshUnavailable) {
    RCLCPP_ERROR(get_logger(),
                 "STATIC_ROUTE_EXTENSION_REQUEST "
                 "status=rejected_world_refresh_unavailable generation=%" PRIu64,
                 outcome.route_generation);
    return;
  }
  if (!outcome.queued()) {
    return;
  }

  const StaticRouteExtensionDecision& decision = outcome.decision;
  const StaticRoutePlanningLatencyStats& latency = outcome.planning_latency;
  RCLCPP_INFO(get_logger(),
              "STATIC_ROUTE_EXTENSION_REQUEST status=queued generation=%" PRIu64
              " station_m=%.2f remaining_m=%.2f mode=%s extension_trigger_m=%.2f "
              "p95_trigger_m=%.2f roi_trigger_m=%.2f braking_path_m=%.2f "
              "horizontal_braking_m=%.2f vertical_braking_m=%.2f "
              "required_overlap_m=%.2f roi_request_sequence=%" PRIu64
              " latency_samples=%zu planning_p95_ms=%.2f planning_p99_ms=%.2f "
              "build_and_planning_p99_ms=%.2f",
              outcome.route_generation, outcome.station_m, outcome.remaining_m,
              outcome.observed_world
                  ? "observed_resident_esdf"
                  : (decision.request_roi_refresh ? "roi_refresh" : "resident_esdf"),
              decision.extension_trigger_remaining_m,
              decision.planning_p95_trigger_remaining_m,
              decision.roi_refresh_trigger_remaining_m, decision.braking_path_m,
              decision.horizontal_braking_path_m, decision.vertical_braking_path_m,
              decision.required_certified_overlap_m, outcome.world_refresh_sequence,
              latency.sample_count, latency.planning_p95_ms, latency.planning_p99_ms,
              latency.build_and_planning_p99_ms);
}

void ProductionMppiNode::requestStaticRouteReplan(
    const RouteReleaseReason3D reason, const std::uint64_t route_generation) {
  static_cast<void>(
      route_lifecycle_coordinator_->requestReplan(reason, route_generation));
}

void ProductionMppiNode::logRouteLifecycleReplanOutcome3D(
    const RouteLifecycleReplanOutcome3D& outcome) {
  switch (outcome.origin) {
    case RouteLifecycleReplanOrigin3D::kRequested:
      break;
    case RouteLifecycleReplanOrigin3D::kDeferredExtensionReplay:
      RCLCPP_INFO(get_logger(),
                  "STATIC_ROUTE_REPLAN_REQUEST status=replaying_deferred_request "
                  "generation=%" PRIu64 " reason=%s",
                  outcome.requested_route_generation,
                  routeReleaseReason3DName(outcome.reason));
      break;
    case RouteLifecycleReplanOrigin3D::kDeferredReplanReplay:
      RCLCPP_INFO(get_logger(),
                  "STATIC_ROUTE_REPLAN_REQUEST status=replaying_deferred_replan "
                  "completed_generation=%" PRIu64 " reason=%s",
                  outcome.replay_completed_generation,
                  routeReleaseReason3DName(outcome.reason));
      break;
  }

  if (outcome.cleared_gate_generation.has_value()) {
    RCLCPP_INFO(get_logger(),
                "STATIC_ROUTE_REPLAN_REQUEST status=cleared_superseded_gate "
                "gate_generation=%" PRIu64 " resident_generation=%" PRIu64,
                *outcome.cleared_gate_generation, outcome.committed_route_generation);
  }

  if (outcome.raw_search_overlay_used) {
    RCLCPP_INFO(get_logger(),
                "OBSERVED_ROUTE_REPLAN status=using_raw_search_overlay "
                "raw_revision=%" PRIu64 " esdf_source_raw_revision=%" PRIu64
                " blocked_raw_revision=%" PRIu64 " generation=%" PRIu64 " reason=%s",
                outcome.latest_raw_revision, outcome.resident_source_raw_revision,
                outcome.blocked_raw_revision, outcome.search_generation,
                routeReleaseReason3DName(outcome.reason));
  }

  switch (outcome.status) {
    case RouteLifecycleReplanStatus3D::kObjectiveUnavailable:
      return;
    case RouteLifecycleReplanStatus3D::kDeferredDuringExtension:
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=deferred_active_request "
          "in_flight_generation=%" PRIu64 " requested_generation=%" PRIu64
          " deferred_generation=%" PRIu64 " reason=%s",
          outcome.in_flight_generation, outcome.requested_route_generation,
          outcome.deferred_route_generation, routeReleaseReason3DName(outcome.reason));
      return;
    case RouteLifecycleReplanStatus3D::kDeferredReplanInFlight:
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=deferred_replan_in_flight "
          "in_flight_generation=%" PRIu64 " requested_generation=%" PRIu64 " reason=%s",
          outcome.in_flight_generation, outcome.requested_route_generation,
          routeReleaseReason3DName(outcome.reason));
      return;
    case RouteLifecycleReplanStatus3D::kInvalidResidentWorld:
    case RouteLifecycleReplanStatus3D::kGenerationMismatch:
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=rejected_generation_mismatch "
          "resident_generation=%" PRIu64 " requested_generation=%" PRIu64 " reason=%s",
          outcome.search_generation != 0U ? outcome.search_generation
                                          : outcome.committed_route_generation,
          outcome.requested_route_generation, routeReleaseReason3DName(outcome.reason));
      return;
    case RouteLifecycleReplanStatus3D::kWaitingInitialSearch:
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "STATIC_ROUTE_REPLAN_REQUEST status=waiting_initial_search "
                           "requested_generation=%" PRIu64 " reason=%s",
                           outcome.requested_route_generation,
                           routeReleaseReason3DName(outcome.reason));
      return;
    case RouteLifecycleReplanStatus3D::kWaitingRawSnapshot:
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "OBSERVED_ROUTE_REPLAN status=deferred_waiting_for_raw_snapshot "
          "blocked_raw_revision=%" PRIu64 " latest_raw_revision=%" PRIu64
          " esdf_source_raw_revision=%" PRIu64 " generation=%" PRIu64 " reason=%s",
          outcome.blocked_raw_revision, outcome.latest_raw_revision,
          outcome.resident_source_raw_revision, outcome.search_generation,
          routeReleaseReason3DName(outcome.reason));
      return;
    case RouteLifecycleReplanStatus3D::kInvalidStart:
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=suppressed_invalid_start "
          "generation=%" PRIu64 " start=(%.2f,%.2f,%.2f) reason=%s",
          outcome.search_generation, outcome.search_start.x, outcome.search_start.y,
          outcome.search_start.z, routeReleaseReason3DName(outcome.reason));
      return;
    case RouteLifecycleReplanStatus3D::kSuppressedFailedSearch:
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=suppressed_failed_search "
          "generation=%" PRIu64
          " pose_change_m=%.2f objective_change_m=%.2f elapsed_s=%.2f "
          "raw_world_changed=%s reason=%s",
          outcome.search_generation, outcome.retry.pose_change_m,
          outcome.retry.objective_change_m, outcome.retry.elapsed_s,
          outcome.retry.raw_world_changed ? "true" : "false",
          routeReleaseReason3DName(outcome.reason));
      return;
    case RouteLifecycleReplanStatus3D::kInvalidTransaction:
      RCLCPP_ERROR(get_logger(),
                   "STATIC_ROUTE_REPLAN_REQUEST status=rejected_invalid_transaction "
                   "generation=%" PRIu64 " reason=%s",
                   outcome.search_generation, routeReleaseReason3DName(outcome.reason));
      return;
    case RouteLifecycleReplanStatus3D::kRouteQueueBusy:
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=deferred_route_queue_busy "
          "generation=%" PRIu64 " reason=%s",
          outcome.search_generation, routeReleaseReason3DName(outcome.reason));
      return;
    case RouteLifecycleReplanStatus3D::kQueued:
      if (outcome.dispatched_raw_revision != 0U) {
        std::uint64_t previous_dispatched =
            observed_route_replan_dispatched_raw_revision_.load(
                std::memory_order_relaxed);
        while (previous_dispatched < outcome.dispatched_raw_revision &&
               !observed_route_replan_dispatched_raw_revision_.compare_exchange_weak(
                   previous_dispatched, outcome.dispatched_raw_revision,
                   std::memory_order_release, std::memory_order_relaxed)) {
        }
      }
      RCLCPP_INFO(get_logger(),
                  "STATIC_ROUTE_REPLAN_REQUEST status=queued generation=%" PRIu64
                  " resident_esdf_revision=%" PRIu64
                  " retry_trigger=%.*s reason=%s stitch_limit_m=%.2f",
                  outcome.search_generation, outcome.resident_world_revision,
                  static_cast<int>(
                      staticRouteSearchRetryTriggerName(outcome.retry.trigger).size()),
                  staticRouteSearchRetryTriggerName(outcome.retry.trigger).data(),
                  routeReleaseReason3DName(outcome.reason),
                  outcome.stitch_limit_station_m.value_or(-1.0));
      return;
  }
}

void ProductionMppiNode::maybeRequestStaticTrackingWorldRefresh(
    const std::shared_ptr<const WorldSnapshot3D>& world,
    const ProductionMppiNavigation& navigation,
    const std::shared_ptr<const ProductionNavigationObjective>& objective,
    const std::int64_t now_ns) {
  const RouteLifecycleTrackingRefreshOutcome3D outcome =
      route_lifecycle_coordinator_->requestTrackingWorldRefresh(
          RouteLifecycleTrackingRefreshRequest3D{
              .world = world,
              .navigation = navigation,
              .objective = objective,
              .stamp_ns = now_ns,
          });
  if (outcome.status ==
      RouteLifecycleTrackingRefreshStatus3D::kWorldRefreshUnavailable) {
    RCLCPP_ERROR(get_logger(),
                 "STATIC_TRACKING_ROI_REFRESH "
                 "status=rejected_world_refresh_unavailable base_generation=%" PRIu64,
                 outcome.base_route_generation);
    return;
  }
  if (outcome.status != RouteLifecycleTrackingRefreshStatus3D::kQueued) {
    return;
  }
  RCLCPP_INFO(get_logger(),
              "STATIC_TRACKING_ROI_REFRESH status=queued sequence=%" PRIu64
              " base_generation=%" PRIu64 " objective_epoch=%" PRIu64
              " objective_sample=%" PRIu64 " margin_m=%.2f now_ns=%" PRId64,
              outcome.sequence, outcome.base_route_generation,
              outcome.objective_mission_epoch, outcome.objective_sample_sequence,
              config_.planning.static_tracking_esdf_refresh_margin_m, outcome.stamp_ns);
}

} // namespace drone_city_nav
