#include "drone_city_nav/trajectory_compiler_3d.hpp"

#include <cinttypes>
#include <memory>
#include <stdexcept>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::configureStaticRouteExtension(
    const double maximum_horizontal_acceleration_mps2) {
  static_route_extension_config_.minimum_remaining_m =
      route_tracking_policy_.minimum_remaining_m;
  static_route_extension_config_.required_certified_overlap_m =
      declare_parameter<double>("route_required_certified_overlap_m", 8.0);
  static_route_extension_config_.latency_margin_s =
      declare_parameter<double>("route_extension_latency_margin_s", 0.5);
  static_route_extension_config_.maximum_latency_s =
      declare_parameter<double>("route_extension_maximum_latency_s", 8.0);
  static_route_extension_config_.maximum_horizontal_acceleration_mps2 =
      maximum_horizontal_acceleration_mps2;
  static_route_extension_config_.maximum_vertical_acceleration_mps2 =
      mppi_config_.dynamics.maximum_vertical_acceleration_mps2;
  static_route_extension_config_.maximum_control_jerk_mps3 =
      mppi_config_.dynamics.maximum_control_jerk_mps3;
  static_route_extension_config_.stopping_capability =
      speed_policy_config_.stopping_capability;
  static_route_extension_config_.minimum_retry_progress_m =
      declare_parameter<double>("route_extension_retry_progress_m", 15.0);
  static_route_extension_config_.minimum_retry_interval_s =
      declare_parameter<double>("route_extension_retry_interval_s", 1.0);
  static_route_extension_config_.minimum_endpoint_improvement_m =
      declare_parameter<double>("route_extension_minimum_endpoint_improvement_m", 5.0);
  route_successor_improvement_config_.minimum_absolute_improvement_s =
      declare_parameter<double>("route_successor_minimum_time_improvement_s", 1.0);
  route_successor_improvement_config_.minimum_relative_improvement =
      declare_parameter<double>("route_successor_minimum_time_improvement_ratio", 0.05);
  future_route_connector_config_.tangent_departure_length_m =
      declare_parameter<double>("route_connector_departure_m", 0.5);
  future_route_connector_config_.successor_join_station_m =
      declare_parameter<double>("route_connector_join_m", 2.0);
  future_route_connector_config_.curve_control_distance_m =
      declare_parameter<double>("route_connector_control_m", 0.75);
  const auto connector_curve_samples =
      declare_parameter<std::int64_t>("route_connector_curve_samples", 12);
  if (connector_curve_samples >= 0) {
    future_route_connector_config_.curve_samples =
        static_cast<std::size_t>(connector_curve_samples);
  } else {
    future_route_connector_config_.curve_samples = 0U;
  }
  future_route_connector_config_.minimum_continuous_turn_alignment =
      declare_parameter<double>(
          "route_connector_minimum_continuous_turn_alignment",
          TrajectoryCompilerConfig3D{}.minimum_continuous_turn_alignment);
  certified_route_splice_config_.required_overlap_m =
      static_route_extension_config_.required_certified_overlap_m;
  certified_route_splice_config_.sample_step_m =
      declare_parameter<double>("route_splice_sample_step_m", route_sampling_step_m_);
  certified_route_splice_config_.maximum_position_separation_m =
      declare_parameter<double>("route_splice_maximum_position_separation_m", 0.05);
  certified_route_splice_config_.minimum_tangent_alignment =
      declare_parameter<double>("route_splice_minimum_tangent_alignment", 0.995);
  certified_route_splice_config_.activation_station_tolerance_m =
      declare_parameter<double>("route_splice_activation_station_tolerance_m", 1.0);
  if (!staticRouteExtensionConfigValid(static_route_extension_config_) ||
      !route_successor_improvement_config_.valid() ||
      !futureRouteConnectorConfig3DValid(future_route_connector_config_) ||
      !certifiedRouteSpliceConfig3DValid(certified_route_splice_config_)) {
    throw std::invalid_argument{
        "invalid route extension, successor hysteresis, connector, or certified "
        "splice configuration"};
  }
}

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
      (optional_constraints_.route_cross_track_constraints_enabled &&
       projection.distance_m > route_tracking_policy_.maximum_cross_track_m)) {
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
          " pose_change_m=%.2f objective_change_m=%.2f elapsed_s=%.2f reason=%s",
          outcome.search_generation, outcome.retry.pose_change_m,
          outcome.retry.objective_change_m, outcome.retry.elapsed_s,
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
                  " resident_esdf_revision=%" PRIu64 " retry_trigger=%.*s reason=%s",
                  outcome.search_generation, outcome.resident_world_revision,
                  static_cast<int>(
                      staticRouteSearchRetryTriggerName(outcome.retry.trigger).size()),
                  staticRouteSearchRetryTriggerName(outcome.retry.trigger).data(),
                  routeReleaseReason3DName(outcome.reason));
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
              static_tracking_esdf_refresh_margin_m_, outcome.stamp_ns);
}

} // namespace drone_city_nav
