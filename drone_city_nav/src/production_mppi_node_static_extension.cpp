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
  const Point3 current{navigation.state.x, navigation.state.y, navigation.state.z};
  const Point3 next_planning_goal = staticRoutePlanningGoal(
      current, mission_goal_, lattice_3d_config_.planning_goal_distance_m);

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

void ProductionMppiNode::finishStaticRouteReplan(
    const std::uint64_t base_generation) noexcept {
  const std::scoped_lock lock{static_route_extension_mutex_};
  static_route_replan_gate_.finish(base_generation);
}

} // namespace drone_city_nav
