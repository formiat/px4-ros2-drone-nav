#include "drone_city_nav/trajectory_compiler_3d.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"
#include "world_pipeline_3d.hpp"

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
      !futureRouteConnectorConfig3DValid(future_route_connector_config_) ||
      !certifiedRouteSpliceConfig3DValid(certified_route_splice_config_)) {
    throw std::invalid_argument{
        "invalid static route extension, connector, or certified splice configuration"};
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
  maybeRequestStaticRouteExtension(
      world, world_build, *active_route, navigation,
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
    const CertifiedRouteSuffix3D& active_route,
    const ProductionMppiNavigation& navigation,
    const RouteProgressProjection3D& route_projection, const std::int64_t now_ns) {
  if (!active_route.valid() || active_route.geometry == nullptr ||
      active_route.geometry->route == nullptr ||
      active_route.geometry->route->size() < 2U) {
    return;
  }
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  if (!objective) {
    return;
  }
  const Point3 mission_goal = objective->goal;
  const Point3 current{navigation.state.x, navigation.state.y, navigation.state.z};
  const Point3 next_planning_goal =
      staticRoutePlanningGoal(current, mission_goal, static_esdf_route_lookahead_m_);
  const bool observed_world = !use_static_map_;
  const bool pending_successor = route_execution_manager_.pending() != nullptr;
  const ProductionMppiForwardAcceleration3D forward_acceleration =
      productionMppiForwardAcceleration3D(navigation);

  std::scoped_lock extension_lock{static_route_extension_mutex_};
  StaticRoutePlanningLatencyStats latency =
      static_route_planning_latency_tracker_.stats();
  if (latency.sample_count == 0U) {
    const std::shared_ptr<const ProductionRouteActivationResult3D> latest_route_event =
        latest_route_pipeline_event_.load(std::memory_order_acquire);
    latency.planning_p95_ms =
        latest_route_event != nullptr
            ? std::max(0.0, latest_route_event->telemetry.route_search_ms)
            : 0.0;
    latency.planning_p99_ms = latency.planning_p95_ms;
    latency.build_and_planning_p99_ms =
        latency.planning_p99_ms + std::max(0.0, world_build.build_ms);
  }
  const StaticRouteExtensionDecision decision = evaluateStaticRouteExtension(
      static_route_extension_config_,
      StaticRouteExtensionObservation{
          .route_generation = active_route.identity.generation,
          .route_station_m = route_projection.station_m,
          .route_remaining_m = route_projection.remaining_m,
          .horizontal_speed_mps = std::hypot(navigation.state.vx, navigation.state.vy),
          .forward_acceleration_mps2 = forward_acceleration.horizontal_mps2,
          .vertical_speed_mps = std::abs(navigation.state.vz),
          .forward_vertical_acceleration_mps2 = forward_acceleration.vertical_mps2,
          .planning_latency_p95_ms = latency.planning_p95_ms,
          .planning_latency_p99_ms = latency.planning_p99_ms,
          .build_and_planning_latency_p99_ms = latency.build_and_planning_p99_ms,
          .route_reaches_mission_goal =
              active_route.identity.proposal.reaches_mission_goal,
          .next_planning_goal_inside_esdf =
              observed_world ||
              staticRoutePointInsideEsdf(world->grid, next_planning_goal),
          .request_in_flight = static_route_extension_request_in_flight_ ||
                               static_route_replan_gate_.inFlight() ||
                               pending_successor,
          .last_request_generation = static_route_extension_last_request_generation_,
          .last_request_station_m = static_route_extension_last_request_station_m_,
          .request_stamp_ns = now_ns,
          .last_request_stamp_ns = static_route_extension_last_request_stamp_ns_,
      });
  if (!decision.request_extension && !decision.request_roi_refresh) {
    return;
  }

  std::uint64_t roi_refresh_sequence = 0U;
  if (decision.request_extension) {
    const StaticRouteSearchRequestIdentity request_identity{
        .kind = StaticRouteSearchRequestKind::kExtension,
        .base_route_generation = active_route.identity.generation,
    };
    const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
        makePlannerSearchTransaction3D(
            world, captureResidentPlannerWorld3D(*world),
            makeStaticRouteObjective(*objective), request_identity,
            PlannerSearchContinuityBase3D{
                .route = std::make_shared<const CertifiedRouteSuffix3D>(active_route),
                .request_projection = route_projection,
            });
    if (transaction == nullptr) {
      RCLCPP_ERROR(get_logger(),
                   "STATIC_ROUTE_EXTENSION_REQUEST status=rejected_invalid_transaction "
                   "generation=%" PRIu64,
                   active_route.identity.generation);
      return;
    }
    {
      const std::scoped_lock queue_lock{route_planning_queue_mutex_};
      if (pending_route_planning_work_) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "STATIC_ROUTE_EXTENSION_REQUEST status=deferred_route_queue_busy "
            "generation=%" PRIu64 " station_m=%.2f remaining_m=%.2f",
            active_route.identity.generation, route_projection.station_m,
            route_projection.remaining_m);
        return;
      }
      pending_route_planning_work_ = ProductionRoutePlanningWork3D{
          .transaction = transaction,
          .world_telemetry = world_build,
          .continuation_session = nullptr,
      };
    }
    route_planning_queue_condition_.notify_all();
  } else {
    const StaticWorldRefreshRequest3D refresh = world_pipeline_->requestStaticRefresh(
        active_route.identity.generation, StaticWorldRefreshPurpose3D::kRouteExtension);
    if (!refresh.valid()) {
      RCLCPP_ERROR(get_logger(),
                   "STATIC_ROUTE_EXTENSION_REQUEST "
                   "status=rejected_world_refresh_unavailable generation=%" PRIu64,
                   active_route.identity.generation);
      return;
    }
    roi_refresh_sequence = refresh.sequence;
  }

  static_route_extension_request_in_flight_ = true;
  static_route_extension_in_flight_generation_ = active_route.identity.generation;
  static_route_extension_last_request_generation_ = active_route.identity.generation;
  static_route_extension_last_request_station_m_ = route_projection.station_m;
  static_route_extension_last_request_stamp_ns_ = now_ns;
  RCLCPP_INFO(
      get_logger(),
      "STATIC_ROUTE_EXTENSION_REQUEST status=queued generation=%" PRIu64
      " station_m=%.2f remaining_m=%.2f mode=%s extension_trigger_m=%.2f "
      "p95_trigger_m=%.2f roi_trigger_m=%.2f braking_path_m=%.2f "
      "horizontal_braking_m=%.2f vertical_braking_m=%.2f "
      "required_overlap_m=%.2f roi_request_sequence=%" PRIu64
      " latency_samples=%zu planning_p95_ms=%.2f planning_p99_ms=%.2f "
      "build_and_planning_p99_ms=%.2f",
      active_route.identity.generation, route_projection.station_m,
      route_projection.remaining_m,
      observed_world ? "observed_resident_esdf"
                     : (decision.request_roi_refresh ? "roi_refresh" : "resident_esdf"),
      decision.extension_trigger_remaining_m, decision.planning_p95_trigger_remaining_m,
      decision.roi_refresh_trigger_remaining_m, decision.braking_path_m,
      decision.horizontal_braking_path_m, decision.vertical_braking_path_m,
      decision.required_certified_overlap_m, roi_refresh_sequence, latency.sample_count,
      latency.planning_p95_ms, latency.planning_p99_ms,
      latency.build_and_planning_p99_ms);
}

void ProductionMppiNode::finishStaticRouteExtension(const std::uint64_t base_generation,
                                                    const bool extension_activated) {
  std::optional<StaticRouteDeferredReplan> deferred_replan;
  {
    const std::scoped_lock lock{static_route_extension_mutex_};
    if (static_route_extension_request_in_flight_ &&
        static_route_extension_in_flight_generation_ == base_generation) {
      static_route_extension_request_in_flight_ = false;
      static_route_extension_in_flight_generation_ = 0U;
    }
    deferred_replan = static_route_deferred_replan_latch_.finishExtension(
        base_generation, extension_activated);
  }
  if (deferred_replan.has_value()) {
    RCLCPP_INFO(get_logger(),
                "STATIC_ROUTE_REPLAN_REQUEST status=replaying_deferred_request "
                "generation=%" PRIu64 " reason=%s",
                deferred_replan->route_generation,
                routeReleaseReason3DName(deferred_replan->reason));
    requestStaticRouteReplan(deferred_replan->reason,
                             deferred_replan->route_generation);
  }
}

void ProductionMppiNode::requestStaticRouteReplan(
    const RouteReleaseReason3D reason, const std::uint64_t route_generation) {
  ProductionMppiNavigation navigation;
  {
    const std::scoped_lock input_lock{input_mutex_};
    navigation = navigation_;
  }
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  if (!objective) {
    return;
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const std::shared_ptr<const ExecutionPlan3D> execution_snapshot =
      route_execution_manager_.plan();
  const std::uint64_t committed_route_generation =
      execution_snapshot != nullptr ? execution_snapshot->routeGenerationHighWater()
                                    : 0U;

  std::shared_ptr<const WorldSnapshot3D> world_snapshot;
  std::shared_ptr<const PersistentPlannerWorld3D> planner_world;
  ProductionWorldBuildTelemetry3D world_telemetry;
  std::uint64_t search_generation{0U};
  std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
  const std::optional<std::uint64_t> superseded_gate =
      static_route_replan_gate_.finishIfSupersededBy(committed_route_generation);
  if (superseded_gate.has_value()) {
    RCLCPP_INFO(get_logger(),
                "STATIC_ROUTE_REPLAN_REQUEST status=cleared_superseded_gate "
                "gate_generation=%" PRIu64 " resident_generation=%" PRIu64,
                *superseded_gate, committed_route_generation);
  }
  const bool replan_in_flight = static_route_replan_gate_.inFlight();
  if (deferStaticRouteReleaseDuringExtension(static_route_extension_request_in_flight_,
                                             reason)) {
    const std::uint64_t deferred_generation =
        route_generation != 0U ? route_generation
                               : static_route_extension_in_flight_generation_;
    static_route_deferred_replan_latch_.defer(StaticRouteDeferredReplan{
        .reason = reason, .route_generation = deferred_generation});
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STATIC_ROUTE_REPLAN_REQUEST status=deferred_active_request "
                         "in_flight_generation=%" PRIu64
                         " requested_generation=%" PRIu64
                         " deferred_generation=%" PRIu64 " reason=%s",
                         static_route_extension_in_flight_generation_, route_generation,
                         deferred_generation, routeReleaseReason3DName(reason));
    return;
  }
  if (replan_in_flight) {
    const std::uint64_t deferred_generation =
        route_generation != 0U ? route_generation
                               : static_route_replan_gate_.generation();
    static_route_deferred_replan_latch_.defer(StaticRouteDeferredReplan{
        .reason = reason, .route_generation = deferred_generation});
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STATIC_ROUTE_REPLAN_REQUEST status=deferred_replan_in_flight "
                         "in_flight_generation=%" PRIu64
                         " requested_generation=%" PRIu64 " reason=%s",
                         static_route_replan_gate_.generation(), route_generation,
                         routeReleaseReason3DName(reason));
    return;
  }
  {
    WorldPipeline3D::ResidentLease resident = world_pipeline_->lockResident();
    if (!resident.world() || !productionWorldGenerationCoherent(*resident.world())) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=rejected_generation_mismatch "
          "resident_generation=%" PRIu64 " requested_generation=%" PRIu64 " reason=%s",
          committed_route_generation, route_generation,
          routeReleaseReason3DName(reason));
      return;
    }
    constexpr bool snapshot_owned_execution{true};
    search_generation = staticRouteSearchGeneration(snapshot_owned_execution,
                                                    committed_route_generation,
                                                    committed_route_generation);
    if (committed_route_generation == 0U &&
        !static_route_failed_search_latch_.latched()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "STATIC_ROUTE_REPLAN_REQUEST status=waiting_initial_search "
                           "requested_generation=%" PRIu64 " reason=%s",
                           route_generation, routeReleaseReason3DName(reason));
      return;
    }
    if (route_generation != 0U && search_generation != route_generation) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=rejected_generation_mismatch "
          "resident_generation=%" PRIu64 " requested_generation=%" PRIu64 " reason=%s",
          search_generation, route_generation, routeReleaseReason3DName(reason));
      return;
    }
    world_snapshot = resident.world();
    world_telemetry = resident.telemetry();
    planner_world = captureResidentPlannerWorld3D(*world_snapshot);
  }

  std::uint64_t dispatched_raw_revision{0U};
  if (!use_static_map_) {
    const std::uint64_t blocked_raw_revision =
        observed_route_blocked_raw_revision_.load(std::memory_order_acquire);
    const bool latest_raw_overlay_required =
        routeSearchRequiresLatestRawOverlay3D(reason) ||
        blocked_raw_revision > world_snapshot->source_raw_revision;
    if (latest_raw_overlay_required) {
      const std::uint64_t minimum_search_raw_revision =
          std::max(blocked_raw_revision, world_snapshot->source_raw_revision);
      const std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world =
          world_pipeline_->latestRawWorld();
      const std::shared_ptr<const PersistentPlannerWorld3D> raw_overlay =
          latest_raw_world != nullptr
              ? captureObservedRouteSearchWorld3D(
                    *latest_raw_world, world_snapshot->proprioceptive_free_space_seed,
                    world_snapshot->launch_support_contact)
              : nullptr;
      if (raw_overlay == nullptr ||
          raw_overlay->producer_instance_id != world_snapshot->producer_instance_id ||
          raw_overlay->revision < minimum_search_raw_revision) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "OBSERVED_ROUTE_REPLAN status=deferred_waiting_for_raw_snapshot "
            "blocked_raw_revision=%" PRIu64 " latest_raw_revision=%" PRIu64
            " esdf_source_raw_revision=%" PRIu64 " generation=%" PRIu64 " reason=%s",
            blocked_raw_revision,
            latest_raw_world != nullptr ? latest_raw_world->version.revision : 0U,
            world_snapshot->source_raw_revision, search_generation,
            routeReleaseReason3DName(reason));
        return;
      }
      planner_world = raw_overlay;
      if (blocked_raw_revision != 0U) {
        dispatched_raw_revision = raw_overlay->revision;
      }
      RCLCPP_INFO(get_logger(),
                  "OBSERVED_ROUTE_REPLAN status=using_raw_search_overlay "
                  "raw_revision=%" PRIu64 " esdf_source_raw_revision=%" PRIu64
                  " blocked_raw_revision=%" PRIu64 " generation=%" PRIu64 " reason=%s",
                  raw_overlay->revision, world_snapshot->source_raw_revision,
                  blocked_raw_revision, search_generation,
                  routeReleaseReason3DName(reason));
    }
  }

  const std::uint64_t required_epoch =
      minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
  const std::uint64_t required_sample =
      objective->mission_epoch == required_epoch
          ? minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire)
          : 0U;
  const StaticRouteSearchContext retry_context{
      .base_route_generation = search_generation,
      .search_start =
          Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      .objective = makeStaticRouteObjective(*objective),
      .minimum_tracking_sample_sequence = required_sample,
      .stamp_ns = now_ns,
  };
  if (!navigation.valid ||
      (static_route_failed_search_latch_.latched() &&
       !insideFlightEnvelope(retry_context.search_start, flight_envelope_config_))) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STATIC_ROUTE_REPLAN_REQUEST status=suppressed_invalid_start "
                         "generation=%" PRIu64 " start=(%.2f,%.2f,%.2f) reason=%s",
                         retry_context.base_route_generation,
                         retry_context.search_start.x, retry_context.search_start.y,
                         retry_context.search_start.z,
                         routeReleaseReason3DName(reason));
    return;
  }
  const StaticRouteSearchRetryDecision retry =
      static_route_failed_search_latch_.evaluate(static_route_search_retry_config_,
                                                 retry_context);
  if (!retry.allow) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "STATIC_ROUTE_REPLAN_REQUEST status=suppressed_failed_search "
        "generation=%" PRIu64
        " pose_change_m=%.2f objective_change_m=%.2f elapsed_s=%.2f reason=%s",
        retry_context.base_route_generation, retry.pose_change_m,
        retry.objective_change_m, retry.elapsed_s, routeReleaseReason3DName(reason));
    return;
  }

  const StaticRouteSearchRequestIdentity request_identity{
      .kind = search_generation == 0U ? StaticRouteSearchRequestKind::kInitialRetry
                                      : StaticRouteSearchRequestKind::kReplan,
      .base_route_generation = search_generation,
  };
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(world_snapshot, std::move(planner_world),
                                     makeStaticRouteObjective(*objective),
                                     request_identity, std::nullopt, reason);
  if (transaction == nullptr) {
    RCLCPP_ERROR(get_logger(),
                 "STATIC_ROUTE_REPLAN_REQUEST status=rejected_invalid_transaction "
                 "generation=%" PRIu64 " reason=%s",
                 search_generation, routeReleaseReason3DName(reason));
    return;
  }

  {
    const std::scoped_lock queue_lock{route_planning_queue_mutex_};
    if (pending_route_planning_work_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=deferred_route_queue_busy "
          "generation=%" PRIu64 " reason=%s",
          search_generation, routeReleaseReason3DName(reason));
      return;
    }
    if (!static_route_replan_gate_.tryBegin(search_generation)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "STATIC_ROUTE_REPLAN_REQUEST status=coalesced_gate_rejected "
                           "generation=%" PRIu64 " in_flight_generation=%" PRIu64
                           " reason=%s",
                           search_generation, static_route_replan_gate_.generation(),
                           routeReleaseReason3DName(reason));
      return;
    }
    pending_route_planning_work_ = ProductionRoutePlanningWork3D{
        .transaction = transaction,
        .world_telemetry = world_telemetry,
        .continuation_session = nullptr,
    };
  }
  if (dispatched_raw_revision != 0U) {
    std::uint64_t previous_dispatched =
        observed_route_replan_dispatched_raw_revision_.load(std::memory_order_relaxed);
    while (previous_dispatched < dispatched_raw_revision &&
           !observed_route_replan_dispatched_raw_revision_.compare_exchange_weak(
               previous_dispatched, dispatched_raw_revision, std::memory_order_release,
               std::memory_order_relaxed)) {
    }
  }
  route_planning_queue_condition_.notify_all();
  RCLCPP_INFO(get_logger(),
              "STATIC_ROUTE_REPLAN_REQUEST status=queued generation=%" PRIu64
              " resident_esdf_revision=%" PRIu64 " retry_trigger=%.*s reason=%s",
              search_generation, world_snapshot->revision,
              static_cast<int>(staticRouteSearchRetryTriggerName(retry.trigger).size()),
              staticRouteSearchRetryTriggerName(retry.trigger).data(),
              routeReleaseReason3DName(reason));
}

void ProductionMppiNode::maybeRequestStaticTrackingWorldRefresh(
    const std::shared_ptr<const WorldSnapshot3D>& world,
    const ProductionMppiNavigation& navigation,
    const ProductionNavigationObjective& objective, const std::int64_t now_ns) {
  if (!navigation.valid) {
    return;
  }
  const std::shared_ptr<const ExecutionPlan3D> execution_snapshot =
      route_execution_manager_.plan();
  if (execution_snapshot == nullptr) {
    return;
  }
  const CertifiedRouteSuffix3D* const route = execution_snapshot->route();
  if (route == nullptr) {
    return;
  }
  const CertifiedRouteSuffix3D& active_route = *route;
  if (!active_route.valid()) {
    return;
  }
  const std::uint64_t active_generation = active_route.identity.generation;
  const Point3 current{navigation.state.x, navigation.state.y, navigation.state.z};
  const Point3 planning_goal =
      staticRoutePlanningGoal(current, objective.goal, static_esdf_route_lookahead_m_);
  if (staticRoutePointInsideEsdf(world->grid, current,
                                 static_tracking_esdf_refresh_margin_m_) &&
      staticRoutePointInsideEsdf(world->grid, planning_goal,
                                 static_tracking_esdf_refresh_margin_m_)) {
    return;
  }

  std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
  if (static_route_extension_request_in_flight_ ||
      static_route_replan_gate_.inFlight() ||
      !static_route_replan_gate_.tryBegin(active_generation)) {
    return;
  }
  const StaticWorldRefreshRequest3D request = world_pipeline_->requestStaticRefresh(
      active_generation, StaticWorldRefreshPurpose3D::kTrackingObjective);
  if (!request.valid()) {
    static_route_replan_gate_.finish(active_generation);
    RCLCPP_ERROR(get_logger(),
                 "STATIC_TRACKING_ROI_REFRESH "
                 "status=rejected_world_refresh_unavailable base_generation=%" PRIu64,
                 active_generation);
    return;
  }
  RCLCPP_INFO(get_logger(),
              "STATIC_TRACKING_ROI_REFRESH status=queued sequence=%" PRIu64
              " base_generation=%" PRIu64 " objective_epoch=%" PRIu64
              " objective_sample=%" PRIu64 " margin_m=%.2f now_ns=%" PRId64,
              request.sequence, request.base_route_generation, objective.mission_epoch,
              objective.sample_sequence, static_tracking_esdf_refresh_margin_m_,
              now_ns);
}

void ProductionMppiNode::finishStaticRouteReplan(const std::uint64_t base_generation,
                                                 const bool route_activated) {
  std::optional<StaticRouteDeferredReplan> deferred_replan;
  {
    const std::scoped_lock lock{static_route_extension_mutex_};
    static_route_replan_gate_.finish(base_generation);
    deferred_replan = static_route_deferred_replan_latch_.finishReplan(base_generation,
                                                                       route_activated);
  }
  if (!deferred_replan.has_value()) {
    return;
  }
  RCLCPP_INFO(get_logger(),
              "STATIC_ROUTE_REPLAN_REQUEST status=replaying_deferred_replan "
              "completed_generation=%" PRIu64 " reason=%s",
              base_generation, routeReleaseReason3DName(deferred_replan->reason));
  // A completed search may have installed a new route generation. Resolve the
  // replay against the resident route rather than rejecting its old generation.
  requestStaticRouteReplan(deferred_replan->reason, 0U);
}

void ProductionMppiNode::finishStaticRouteSearch(
    const PlannerSearchTransaction3D& transaction, const bool route_activated) {
  if (transaction.extension()) {
    finishStaticRouteExtension(transaction.request.base_route_generation,
                               route_activated);
  }
  if (transaction.replacement()) {
    finishStaticRouteReplan(transaction.request.base_route_generation, route_activated);
  }
}

} // namespace drone_city_nav
