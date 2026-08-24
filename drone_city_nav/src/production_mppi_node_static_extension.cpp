#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double
forwardHorizontalAcceleration(const ProductionMppiNavigation& navigation) noexcept {
  if (!navigation.measured_acceleration_valid) {
    return 0.0;
  }
  const double speed_mps = std::hypot(navigation.state.vx, navigation.state.vy);
  if (!(speed_mps > 1.0e-6)) {
    return 0.0;
  }
  return (static_cast<double>(navigation.state.vx) *
              navigation.measured_equivalent_control.ax +
          static_cast<double>(navigation.state.vy) *
              navigation.measured_equivalent_control.ay) /
         speed_mps;
}

} // namespace

void ProductionMppiNode::configureStaticRouteExtension(
    const double maximum_horizontal_acceleration_mps2) {
  static_route_extension_config_.minimum_remaining_m =
      active_guide_config_.minimum_remaining_m;
  static_route_extension_config_.required_certified_overlap_m =
      declare_parameter<double>("static_global_guide_required_certified_overlap_m",
                                8.0);
  static_route_extension_config_.latency_margin_s =
      declare_parameter<double>("static_global_guide_extension_latency_margin_s", 0.5);
  static_route_extension_config_.maximum_latency_s =
      declare_parameter<double>("static_global_guide_extension_maximum_latency_s", 8.0);
  static_route_extension_config_.maximum_horizontal_acceleration_mps2 =
      maximum_horizontal_acceleration_mps2;
  static_route_extension_config_.maximum_control_jerk_mps3 =
      mppi_config_.dynamics.maximum_control_jerk_mps3;
  static_route_extension_config_.stopping_capability =
      speed_policy_config_.stopping_capability;
  static_route_extension_config_.minimum_retry_progress_m =
      declare_parameter<double>("static_global_guide_extension_retry_progress_m", 15.0);
  static_route_extension_config_.minimum_retry_interval_s =
      declare_parameter<double>("static_global_guide_extension_retry_interval_s", 1.0);
  static_route_extension_config_.minimum_endpoint_improvement_m =
      declare_parameter<double>(
          "static_global_guide_extension_minimum_endpoint_improvement_m", 5.0);
  certified_route_splice_config_.required_overlap_m =
      static_route_extension_config_.required_certified_overlap_m;
  certified_route_splice_config_.sample_step_m = declare_parameter<double>(
      "static_global_guide_splice_sample_step_m", lattice_3d_config_.sample_step_m);
  certified_route_splice_config_.maximum_position_separation_m =
      declare_parameter<double>(
          "static_global_guide_splice_maximum_position_separation_m", 2.0);
  certified_route_splice_config_.minimum_tangent_alignment = declare_parameter<double>(
      "static_global_guide_splice_minimum_tangent_alignment", 0.5);
  certified_route_splice_config_.activation_station_tolerance_m =
      declare_parameter<double>(
          "static_global_guide_splice_activation_station_tolerance_m", 1.0);
  if (!staticRouteExtensionConfigValid(static_route_extension_config_) ||
      !certifiedRouteSpliceConfig3DValid(certified_route_splice_config_)) {
    throw std::invalid_argument{
        "invalid static route extension or certified splice configuration"};
  }
}

void ProductionMppiNode::bindStaticRouteRequestToExecution(
    ProductionMppiPreparedEsdf& request, const CertifiedRouteSuffix3D& active_route,
    const GlobalGuideProjection& projection) {
  const ExecutionRouteGeometry3D& geometry = *active_route.geometry;
  request.global_guide_generation = active_route.identity.generation;
  request.global_guide_reaches_mission_goal =
      active_route.identity.proposal.reaches_mission_goal;
  request.global_guide_projection = projection;
  request.route_fingerprint = active_route.identity.proposal.route_fingerprint;
  request.route_intent = active_route.identity.proposal.intent;
  request.route_segment_evidence = active_route.identity.proposal.evidence;
  request.route_objective = active_route.identity.proposal.objective;
  request.mppi_route = geometry.mppi_route;
  request.route_3d = geometry.route;
  request.route_2d_projection = geometry.route_2d_projection;
  request.constrained_spans = geometry.constrained_spans;
  request.passage_volumes = geometry.passage_volumes;
  request.cooperative_passage_assignments = geometry.cooperative_passage_assignments;
  request.selected_passage_traversal_ids = geometry.selected_passage_traversal_ids;
  request.lattice_3d_route_purpose = geometry.route_purpose;
  request.lattice_3d_observation_frontier = geometry.observation_frontier;
}

void ProductionMppiNode::maybeRequestStaticRouteExtensionFromExecution(
    const ProductionMppiPreparedEsdf& esdf,
    const ProductionRouteExecutionSelection3D& route_execution,
    const ProductionMppiNavigation& navigation, const std::int64_t now_ns) {
  if (route_execution.source_snapshot == nullptr ||
      route_execution.source_snapshot->phase != ExecutionRoutePhase3D::kFollowing ||
      !route_execution.source_snapshot->route.has_value()) {
    return;
  }
  const CertifiedRouteSuffix3D& active_route = *route_execution.source_snapshot->route;
  const RouteProjection3D projection = projectOntoRoute3DWithinStationWindow(
      *active_route.geometry->route,
      Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      active_route.progress.station_m, active_route.endStationM());
  if (!projection.valid ||
      projection.distance_m > active_guide_config_.maximum_cross_track_m) {
    return;
  }
  maybeRequestStaticRouteExtension(
      esdf, active_route, navigation,
      GlobalGuideProjection{
          .valid = true,
          .station_m = projection.station_m,
          .total_length_m = active_route.endStationM(),
          .remaining_m = projection.remaining_m,
          .cross_track_m = projection.distance_m,
          .point = {projection.point.x, projection.point.y},
      },
      now_ns);
}

void ProductionMppiNode::maybeRequestStaticRouteExtension(
    const ProductionMppiPreparedEsdf& esdf, const CertifiedRouteSuffix3D& active_route,
    const ProductionMppiNavigation& navigation,
    const GlobalGuideProjection& route_projection, const std::int64_t now_ns) {
  if (!active_route.valid() || active_route.geometry == nullptr ||
      active_route.geometry->route == nullptr ||
      active_route.geometry->route->size() < 2U ||
      active_route.geometry->route_purpose == Lattice3DRoutePurpose::kLaunchDeparture) {
    return;
  }
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
  const Point3 current{navigation.state.x, navigation.state.y, navigation.state.z};
  const Point3 next_planning_goal = staticRoutePlanningGoal(
      current, mission_goal, lattice_3d_config_.planning_goal_distance_m);
  const bool observed_world =
      !use_static_map_ &&
      no_static_world_model_ == ProductionNoStaticWorldModel::kObservedOccupancy3D;
  const bool pending_successor = pending_certified_route_mailbox_.snapshot() != nullptr;

  std::scoped_lock extension_lock{static_route_extension_mutex_};
  StaticRoutePlanningLatencyStats latency =
      static_route_planning_latency_tracker_.stats();
  if (latency.sample_count == 0U) {
    latency.planning_p95_ms = std::max(0.0, esdf.global_guide_search_ms);
    latency.planning_p99_ms = latency.planning_p95_ms;
    latency.build_and_planning_p99_ms =
        latency.planning_p99_ms + std::max(0.0, esdf.build_ms);
  }
  const StaticRouteExtensionDecision decision = evaluateStaticRouteExtension(
      static_route_extension_config_,
      StaticRouteExtensionObservation{
          .route_generation = active_route.identity.generation,
          .route_station_m = route_projection.station_m,
          .route_remaining_m = route_projection.remaining_m,
          .horizontal_speed_mps = std::hypot(navigation.state.vx, navigation.state.vy),
          .forward_acceleration_mps2 = forwardHorizontalAcceleration(navigation),
          .planning_latency_p95_ms = latency.planning_p95_ms,
          .planning_latency_p99_ms = latency.planning_p99_ms,
          .build_and_planning_latency_p99_ms = latency.build_and_planning_p99_ms,
          .route_reaches_mission_goal =
              active_route.identity.proposal.reaches_mission_goal,
          .next_planning_goal_inside_esdf =
              observed_world ||
              staticRoutePointInsideEsdf(esdf.grid, next_planning_goal),
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
    auto request = std::make_shared<ProductionMppiPreparedEsdf>(esdf);
    bindStaticRouteRequestToExecution(*request, active_route, route_projection);
    if (objective) {
      request->search_objective = makeStaticRouteObjective(*objective);
    }
    request->static_route_extension_request = true;
    request->static_route_extension_base_generation = active_route.identity.generation;
    {
      const std::scoped_lock queue_lock{guide_queue_mutex_};
      if (pending_guide_world_) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "STATIC_ROUTE_EXTENSION_REQUEST status=deferred_guide_queue_busy "
            "generation=%" PRIu64 " station_m=%.2f remaining_m=%.2f",
            active_route.identity.generation, route_projection.station_m,
            route_projection.remaining_m);
        return;
      }
      pending_guide_world_ = std::move(request);
    }
    guide_queue_condition_.notify_all();
  } else {
    roi_refresh_sequence =
        static_roi_refresh_lifecycle_.queue(active_route.identity.generation).sequence;
    requestStaticEsdfWork(true);
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
      "required_overlap_m=%.2f roi_request_sequence=%" PRIu64
      " latency_samples=%zu planning_p95_ms=%.2f planning_p99_ms=%.2f "
      "build_and_planning_p99_ms=%.2f",
      active_route.identity.generation, route_projection.station_m,
      route_projection.remaining_m,
      observed_world ? "observed_resident_esdf"
                     : (decision.request_roi_refresh ? "roi_refresh" : "resident_esdf"),
      decision.extension_trigger_remaining_m, decision.planning_p95_trigger_remaining_m,
      decision.roi_refresh_trigger_remaining_m, decision.braking_path_m,
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
                globalGuideReleaseReasonName(deferred_replan->reason));
    requestStaticRouteReplan(deferred_replan->reason,
                             deferred_replan->route_generation);
  }
}

void ProductionMppiNode::requestStaticRouteReplan(
    const GlobalGuideReleaseReason reason, const std::uint64_t guide_generation) {
  ProductionMppiNavigation navigation;
  {
    const std::scoped_lock input_lock{input_mutex_};
    navigation = navigation_;
  }
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  const std::int64_t now_ns = get_clock()->now().nanoseconds();

  std::shared_ptr<ProductionMppiPreparedEsdf> request;
  std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
  const bool replan_in_flight = static_route_replan_gate_.inFlight();
  if (deferStaticRouteReleaseDuringExtension(static_route_extension_request_in_flight_,
                                             reason)) {
    const std::uint64_t deferred_generation =
        guide_generation != 0U ? guide_generation
                               : static_route_extension_in_flight_generation_;
    static_route_deferred_replan_latch_.defer(StaticRouteDeferredReplan{
        .reason = reason, .route_generation = deferred_generation});
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STATIC_ROUTE_REPLAN_REQUEST status=deferred_active_request "
                         "in_flight_generation=%" PRIu64
                         " requested_generation=%" PRIu64
                         " deferred_generation=%" PRIu64 " reason=%s",
                         static_route_extension_in_flight_generation_, guide_generation,
                         deferred_generation, globalGuideReleaseReasonName(reason));
    return;
  }
  if (replan_in_flight) {
    const std::uint64_t deferred_generation =
        guide_generation != 0U ? guide_generation
                               : static_route_replan_gate_.generation();
    static_route_deferred_replan_latch_.defer(StaticRouteDeferredReplan{
        .reason = reason, .route_generation = deferred_generation});
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STATIC_ROUTE_REPLAN_REQUEST status=deferred_replan_in_flight "
                         "in_flight_generation=%" PRIu64
                         " requested_generation=%" PRIu64 " reason=%s",
                         static_route_replan_gate_.generation(), guide_generation,
                         globalGuideReleaseReasonName(reason));
    return;
  }
  {
    const std::scoped_lock esdf_lock{esdf_state_mutex_};
    if (!prepared_esdf_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=rejected_generation_mismatch "
          "resident_generation=%" PRIu64 " requested_generation=%" PRIu64 " reason=%s",
          prepared_esdf_ ? prepared_esdf_->global_guide_generation : 0U,
          guide_generation, globalGuideReleaseReasonName(reason));
      return;
    }
    const std::uint64_t resident_generation = prepared_esdf_->global_guide_generation;
    if (resident_generation == 0U && !static_route_failed_search_latch_.latched()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "STATIC_ROUTE_REPLAN_REQUEST status=waiting_initial_search "
                           "requested_generation=%" PRIu64 " reason=%s",
                           guide_generation, globalGuideReleaseReasonName(reason));
      return;
    }
    if (guide_generation != 0U && resident_generation != guide_generation) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=rejected_generation_mismatch "
          "resident_generation=%" PRIu64 " requested_generation=%" PRIu64 " reason=%s",
          resident_generation, guide_generation, globalGuideReleaseReasonName(reason));
      return;
    }
    request = std::make_shared<ProductionMppiPreparedEsdf>(*prepared_esdf_);
    if (objective) {
      request->search_objective = makeStaticRouteObjective(*objective);
    }
    request->global_guide_release_reason = reason;
    request->static_route_replan_request = true;
    request->static_route_replan_base_generation = resident_generation;
    request->static_route_replan_reason = reason;
  }

  if (!use_static_map_ &&
      no_static_world_model_ == ProductionNoStaticWorldModel::kObservedOccupancy3D) {
    const std::uint64_t blocked_raw_revision =
        observed_route_blocked_raw_revision_.load(std::memory_order_acquire);
    if (blocked_raw_revision > request->source_raw_revision) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "OBSERVED_ROUTE_REPLAN status=deferred_waiting_for_esdf raw_revision=%" PRIu64
          " esdf_source_raw_revision=%" PRIu64 " generation=%" PRIu64,
          blocked_raw_revision, request->source_raw_revision,
          request->static_route_replan_base_generation);
      return;
    }
  }

  const std::uint64_t required_epoch =
      minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
  const std::uint64_t required_sample =
      objective && objective->mission_epoch == required_epoch
          ? minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire)
          : 0U;
  const StaticRouteSearchContext retry_context{
      .base_route_generation = request->static_route_replan_base_generation,
      .search_start =
          Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      .objective = request->search_objective,
      .minimum_tracking_sample_sequence = required_sample,
      .stamp_ns = now_ns,
  };
  if (!navigation.valid ||
      (static_route_failed_search_latch_.latched() &&
       !insideFlightEnvelope(retry_context.search_start,
                             lattice_3d_config_.flight_envelope))) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STATIC_ROUTE_REPLAN_REQUEST status=suppressed_invalid_start "
                         "generation=%" PRIu64 " start=(%.2f,%.2f,%.2f) reason=%s",
                         retry_context.base_route_generation,
                         retry_context.search_start.x, retry_context.search_start.y,
                         retry_context.search_start.z,
                         globalGuideReleaseReasonName(reason));
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
        retry.objective_change_m, retry.elapsed_s,
        globalGuideReleaseReasonName(reason));
    return;
  }

  {
    const std::scoped_lock queue_lock{guide_queue_mutex_};
    if (pending_guide_world_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=deferred_guide_queue_busy "
          "generation=%" PRIu64 " reason=%s",
          request->static_route_replan_base_generation,
          globalGuideReleaseReasonName(reason));
      return;
    }
    if (!static_route_replan_gate_.tryBegin(
            request->static_route_replan_base_generation)) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=coalesced_gate_rejected "
          "generation=%" PRIu64 " in_flight_generation=%" PRIu64 " reason=%s",
          request->static_route_replan_base_generation,
          static_route_replan_gate_.generation(), globalGuideReleaseReasonName(reason));
      return;
    }
    pending_guide_world_ = request;
  }
  guide_queue_condition_.notify_all();
  RCLCPP_INFO(get_logger(),
              "STATIC_ROUTE_REPLAN_REQUEST status=queued generation=%" PRIu64
              " resident_esdf_revision=%" PRIu64 " retry_trigger=%.*s reason=%s",
              request->static_route_replan_base_generation, request->revision,
              static_cast<int>(staticRouteSearchRetryTriggerName(retry.trigger).size()),
              staticRouteSearchRetryTriggerName(retry.trigger).data(),
              globalGuideReleaseReasonName(reason));
}

void ProductionMppiNode::maybeRequestStaticTrackingWorldRefresh(
    const ProductionMppiPreparedEsdf& esdf, const ProductionMppiNavigation& navigation,
    const ProductionNavigationObjective& objective, const std::int64_t now_ns) {
  if (!navigation.valid) {
    return;
  }
  const std::shared_ptr<const ExecutionRouteSnapshot3D> execution_snapshot =
      execution_route_store_.snapshot();
  if (execution_snapshot == nullptr || !execution_snapshot->route.has_value() ||
      !execution_snapshot->route->valid()) {
    return;
  }
  const std::uint64_t active_generation =
      execution_snapshot->route->identity.generation;
  const Point3 current{navigation.state.x, navigation.state.y, navigation.state.z};
  const Point3 planning_goal = staticRoutePlanningGoal(
      current, objective.goal, lattice_3d_config_.planning_goal_distance_m);
  if (staticRoutePointInsideEsdf(esdf.grid, current,
                                 static_tracking_esdf_refresh_margin_m_) &&
      staticRoutePointInsideEsdf(esdf.grid, planning_goal,
                                 static_tracking_esdf_refresh_margin_m_)) {
    return;
  }

  std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
  if (static_route_extension_request_in_flight_ ||
      static_route_replan_gate_.inFlight() ||
      !static_route_replan_gate_.tryBegin(active_generation)) {
    return;
  }
  const StaticRouteRoiRefreshRequest request = static_roi_refresh_lifecycle_.queue(
      active_generation, StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective);
  requestStaticEsdfWork(true);
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
              base_generation, globalGuideReleaseReasonName(deferred_replan->reason));
  // A completed search may have installed a new guide generation. Resolve the
  // replay against the resident route rather than rejecting its old generation.
  requestStaticRouteReplan(deferred_replan->reason, 0U);
}

void ProductionMppiNode::finishStaticRouteSearch(
    const ProductionMppiPreparedEsdf& world, const bool route_activated) {
  if (world.static_route_extension_request) {
    finishStaticRouteExtension(world.static_route_extension_base_generation,
                               route_activated);
  }
  if (world.static_route_replan_request) {
    finishStaticRouteReplan(world.static_route_replan_base_generation, route_activated);
  }
}

} // namespace drone_city_nav
