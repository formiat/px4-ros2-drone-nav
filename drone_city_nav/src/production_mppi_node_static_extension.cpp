#include <cinttypes>
#include <cmath>
#include <memory>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::maybeRequestStaticRouteExtension(
    const ProductionMppiPreparedEsdf& esdf, const ProductionMppiNavigation& navigation,
    const GlobalGuideProjection& route_projection, const std::int64_t now_ns) {
  if (!esdf.route_3d || esdf.route_3d->size() < 2U ||
      esdf.global_guide_generation == 0U) {
    return;
  }
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
  const Point3 current{navigation.state.x, navigation.state.y, navigation.state.z};
  const Point3 next_planning_goal = staticRoutePlanningGoal(
      current, mission_goal, lattice_3d_config_.planning_goal_distance_m);

  std::scoped_lock extension_lock{static_route_extension_mutex_};
  const StaticRouteExtensionDecision decision = evaluateStaticRouteExtension(
      static_route_extension_config_,
      StaticRouteExtensionObservation{
          .route_generation = esdf.global_guide_generation,
          .route_station_m = route_projection.station_m,
          .route_remaining_m = route_projection.remaining_m,
          .horizontal_speed_mps = std::hypot(navigation.state.vx, navigation.state.vy),
          .guide_search_latency_ms = esdf.global_guide_search_ms,
          .esdf_build_latency_ms = esdf.build_ms,
          .route_reaches_mission_goal = esdf.global_guide_reaches_mission_goal,
          .next_planning_goal_inside_esdf =
              staticRoutePointInsideEsdf(esdf.grid, next_planning_goal),
          .request_in_flight = static_route_extension_request_in_flight_ ||
                               static_route_replan_gate_.inFlight(),
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
    if (objective) {
      request->search_objective = makeStaticRouteObjective(*objective);
    }
    request->static_route_extension_request = true;
    request->static_route_extension_base_generation = esdf.global_guide_generation;
    {
      const std::scoped_lock queue_lock{guide_queue_mutex_};
      if (pending_guide_world_) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "STATIC_ROUTE_EXTENSION_REQUEST status=deferred_guide_queue_busy "
            "generation=%" PRIu64 " station_m=%.2f remaining_m=%.2f",
            esdf.global_guide_generation, route_projection.station_m,
            route_projection.remaining_m);
        return;
      }
      pending_guide_world_ = std::move(request);
    }
    guide_queue_condition_.notify_all();
  } else {
    roi_refresh_sequence =
        static_roi_refresh_lifecycle_.queue(esdf.global_guide_generation).sequence;
    requestStaticEsdfWork(true);
  }

  static_route_extension_request_in_flight_ = true;
  static_route_extension_in_flight_generation_ = esdf.global_guide_generation;
  static_route_extension_last_request_generation_ = esdf.global_guide_generation;
  static_route_extension_last_request_station_m_ = route_projection.station_m;
  static_route_extension_last_request_stamp_ns_ = now_ns;
  RCLCPP_INFO(get_logger(),
              "STATIC_ROUTE_EXTENSION_REQUEST status=queued generation=%" PRIu64
              " station_m=%.2f remaining_m=%.2f mode=%s extension_trigger_m=%.2f "
              "roi_trigger_m=%.2f roi_request_sequence=%" PRIu64
              " search_latency_ms=%.2f build_latency_ms=%.2f",
              esdf.global_guide_generation, route_projection.station_m,
              route_projection.remaining_m,
              decision.request_roi_refresh ? "roi_refresh" : "resident_esdf",
              decision.extension_trigger_remaining_m,
              decision.roi_refresh_trigger_remaining_m, roi_refresh_sequence,
              esdf.global_guide_search_ms, esdf.build_ms);
}

void ProductionMppiNode::finishStaticRouteExtension(
    const std::uint64_t base_generation) noexcept {
  const std::scoped_lock lock{static_route_extension_mutex_};
  if (static_route_extension_request_in_flight_ &&
      static_route_extension_in_flight_generation_ == base_generation) {
    static_route_extension_request_in_flight_ = false;
    static_route_extension_in_flight_generation_ = 0U;
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
  if (deferStaticRouteReleaseDuringExtension(
          static_route_extension_request_in_flight_ || replan_in_flight, reason)) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STATIC_ROUTE_REPLAN_REQUEST status=deferred_active_request "
                         "in_flight_generation=%" PRIu64
                         " requested_generation=%" PRIu64 " reason=%s",
                         static_route_replan_gate_.generation(), guide_generation,
                         globalGuideReleaseReasonName(reason));
    return;
  }
  if (replan_in_flight) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "STATIC_ROUTE_REPLAN_REQUEST status=coalesced_replan_in_flight "
        "in_flight_generation=%" PRIu64 " requested_generation=%" PRIu64 " reason=%s",
        static_route_replan_gate_.generation(), guide_generation,
        globalGuideReleaseReasonName(reason));
    return;
  }
  {
    const std::scoped_lock esdf_lock{esdf_state_mutex_};
    if (!prepared_esdf_ || prepared_esdf_->global_guide_generation == 0U ||
        (guide_generation != 0U &&
         prepared_esdf_->global_guide_generation != guide_generation)) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STATIC_ROUTE_REPLAN_REQUEST status=rejected_generation_mismatch "
          "resident_generation=%" PRIu64 " requested_generation=%" PRIu64 " reason=%s",
          prepared_esdf_ ? prepared_esdf_->global_guide_generation : 0U,
          guide_generation, globalGuideReleaseReasonName(reason));
      return;
    }
    request = std::make_shared<ProductionMppiPreparedEsdf>(*prepared_esdf_);
    if (objective) {
      request->search_objective = makeStaticRouteObjective(*objective);
    }
    request->global_guide_release_reason = reason;
    request->static_route_replan_request = true;
    request->static_route_replan_base_generation =
        prepared_esdf_->global_guide_generation;
    request->static_route_replan_reason = reason;
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
  if (esdf.global_guide_generation == 0U || !navigation.valid) {
    return;
  }
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
      !static_route_replan_gate_.tryBegin(esdf.global_guide_generation)) {
    return;
  }
  const StaticRouteRoiRefreshRequest request = static_roi_refresh_lifecycle_.queue(
      esdf.global_guide_generation,
      StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective);
  requestStaticEsdfWork(true);
  RCLCPP_INFO(get_logger(),
              "STATIC_TRACKING_ROI_REFRESH status=queued sequence=%" PRIu64
              " base_generation=%" PRIu64 " objective_epoch=%" PRIu64
              " objective_sample=%" PRIu64 " margin_m=%.2f now_ns=%" PRId64,
              request.sequence, request.base_route_generation, objective.mission_epoch,
              objective.sample_sequence, static_tracking_esdf_refresh_margin_m_,
              now_ns);
}

void ProductionMppiNode::finishStaticRouteReplan(
    const std::uint64_t base_generation) noexcept {
  const std::scoped_lock lock{static_route_extension_mutex_};
  static_route_replan_gate_.finish(base_generation);
}

} // namespace drone_city_nav
