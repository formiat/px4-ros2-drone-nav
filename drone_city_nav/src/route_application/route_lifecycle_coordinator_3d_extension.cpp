#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

#include "production_mppi_route_world.hpp"
#include "route_lifecycle_coordinator_3d.hpp"

namespace drone_city_nav {

// Extension and refresh requests: the search that extends an active route,
// the tracking world refreshed under it, and what a finished search settles.

RouteLifecycleExtensionOutcome3D RouteLifecycleCoordinator3D::requestExtension(
    RouteLifecycleExtensionRequest3D request) {
  RouteLifecycleExtensionOutcome3D outcome;
  outcome.observed_world = config_.observed_world;
  if (request.world == nullptr || request.active_route == nullptr ||
      request.objective == nullptr || !request.navigation.valid ||
      !request.route_projection.valid || !request.active_route->valid() ||
      request.active_route->geometry == nullptr ||
      request.active_route->geometry->route == nullptr ||
      request.active_route->geometry->route->size() < 2U) {
    outcome.status = RouteLifecycleExtensionStatus3D::kInvalidRequest;
    return outcome;
  }

  const CertifiedRouteSuffix3D& active_route = *request.active_route;
  outcome.route_generation = active_route.identity.generation;
  outcome.station_m = request.route_projection.station_m;
  outcome.remaining_m = request.route_projection.remaining_m;
  const Point3 current{request.navigation.state.x, request.navigation.state.y,
                       request.navigation.state.z};
  const Point3 next_planning_goal = staticRoutePlanningGoal(
      current, request.objective->goal, config_.static_route_lookahead_m);
  const ProductionMppiForwardAcceleration3D forward_acceleration =
      productionMppiForwardAcceleration3D(request.navigation);

  bool previous_in_flight{false};
  std::uint64_t previous_in_flight_generation{0U};
  std::uint64_t previous_last_generation{0U};
  double previous_last_station_m{0.0};
  std::int64_t previous_last_stamp_ns{0};
  {
    const std::scoped_lock lock{lifecycle_mutex_};
    StaticRoutePlanningLatencyStats latency = planning_latency_tracker_.stats();
    if (latency.sample_count == 0U) {
      latency.planning_p95_ms = std::max(0.0, latest_route_search_ms_);
      latency.planning_p99_ms = latency.planning_p95_ms;
      latency.build_and_planning_p99_ms =
          latency.planning_p99_ms + std::max(0.0, request.world_telemetry.build_ms);
    }
    outcome.planning_latency = latency;
    outcome.decision = evaluateStaticRouteExtension(
        config_.extension,
        StaticRouteExtensionObservation{
            .route_generation = active_route.identity.generation,
            .route_station_m = request.route_projection.station_m,
            .route_remaining_m = request.route_projection.remaining_m,
            .horizontal_speed_mps =
                std::hypot(request.navigation.state.vx, request.navigation.state.vy),
            .forward_acceleration_mps2 = forward_acceleration.horizontal_mps2,
            .vertical_speed_mps = std::abs(request.navigation.state.vz),
            .forward_vertical_acceleration_mps2 = forward_acceleration.vertical_mps2,
            .planning_latency_p95_ms = latency.planning_p95_ms,
            .planning_latency_p99_ms = latency.planning_p99_ms,
            .build_and_planning_latency_p99_ms = latency.build_and_planning_p99_ms,
            .route_reaches_mission_goal =
                active_route.identity.proposal.reaches_mission_goal,
            .next_planning_goal_inside_esdf =
                config_.observed_world ||
                staticRoutePointInsideEsdf(request.world->grid, next_planning_goal),
            .request_in_flight = extension_request_in_flight_ ||
                                 replan_gate_.inFlight() || request.pending_successor,
            .last_request_generation = extension_last_request_generation_,
            .last_request_station_m = extension_last_request_station_m_,
            .request_stamp_ns = request.stamp_ns,
            .last_request_stamp_ns = extension_last_request_stamp_ns_,
        });
    if (!outcome.decision.request_extension && !outcome.decision.request_roi_refresh) {
      return outcome;
    }
    previous_in_flight = extension_request_in_flight_;
    previous_in_flight_generation = extension_in_flight_generation_;
    previous_last_generation = extension_last_request_generation_;
    previous_last_station_m = extension_last_request_station_m_;
    previous_last_stamp_ns = extension_last_request_stamp_ns_;
    extension_request_in_flight_ = true;
    extension_in_flight_generation_ = active_route.identity.generation;
    extension_last_request_generation_ = active_route.identity.generation;
    extension_last_request_station_m_ = request.route_projection.station_m;
    extension_last_request_stamp_ns_ = request.stamp_ns;
  }

  const auto rollback_reservation = [this, &outcome, previous_in_flight,
                                     previous_in_flight_generation,
                                     previous_last_generation, previous_last_station_m,
                                     previous_last_stamp_ns]() {
    std::optional<StaticRouteDeferredReplan> replay;
    {
      const std::scoped_lock lock{lifecycle_mutex_};
      if (extension_request_in_flight_ &&
          extension_in_flight_generation_ == outcome.route_generation) {
        extension_request_in_flight_ = previous_in_flight;
        extension_in_flight_generation_ = previous_in_flight_generation;
        extension_last_request_generation_ = previous_last_generation;
        extension_last_request_station_m_ = previous_last_station_m;
        extension_last_request_stamp_ns_ = previous_last_stamp_ns;
        if (!previous_in_flight) {
          replay =
              deferred_replan_latch_.finishExtension(outcome.route_generation, false);
        }
      }
    }
    if (replay.has_value()) {
      replayDeferredReplan(*replay,
                           RouteLifecycleReplanOrigin3D::kDeferredExtensionReplay,
                           outcome.route_generation);
    }
  };

  try {
    if (outcome.decision.request_extension) {
      const StaticRouteSearchRequestIdentity identity{
          .kind = StaticRouteSearchRequestKind::kExtension,
          .base_route_generation = active_route.identity.generation,
      };
      const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
          makePlannerSearchTransaction3D(
              request.world, captureResidentPlannerWorld3D(*request.world),
              makeStaticRouteObjective(*request.objective), identity,
              PlannerSearchContinuityBase3D{
                  .route = std::move(request.active_route),
                  .request_projection = request.route_projection,
              },
              RouteReleaseReason3D::kNone, config_.stamp_provider());
      if (transaction == nullptr) {
        rollback_reservation();
        outcome.status = RouteLifecycleExtensionStatus3D::kInvalidTransaction;
        return outcome;
      }
      const RoutePlanningEnqueueResult3D enqueue_result =
          enqueue(RoutePlanningRequest3D{
              .transaction = transaction,
              .world_telemetry = request.world_telemetry,
              .continuation_session = nullptr,
          });
      outcome.enqueue_status = enqueue_result.status;
      if (!enqueue_result.queued()) {
        rollback_reservation();
        outcome.status = RouteLifecycleExtensionStatus3D::kRouteQueueBusy;
        return outcome;
      }
      outcome.status = RouteLifecycleExtensionStatus3D::kSearchQueued;
      return outcome;
    }

    const RouteLifecycleWorldRefreshResult3D refresh = config_.world_refresh_requester(
        active_route.identity.generation,
        RouteLifecycleWorldRefreshPurpose3D::kRouteExtension);
    if (!refresh.valid()) {
      rollback_reservation();
      outcome.status = RouteLifecycleExtensionStatus3D::kWorldRefreshUnavailable;
      return outcome;
    }
    outcome.world_refresh_sequence = refresh.sequence;
    outcome.status = RouteLifecycleExtensionStatus3D::kWorldRefreshQueued;
    return outcome;
  } catch (...) {
    rollback_reservation();
    throw;
  }
}

RouteLifecycleTrackingRefreshOutcome3D
RouteLifecycleCoordinator3D::requestTrackingWorldRefresh(
    RouteLifecycleTrackingRefreshRequest3D request) {
  RouteLifecycleTrackingRefreshOutcome3D outcome{
      .objective_mission_epoch =
          request.objective != nullptr ? request.objective->mission_epoch : 0U,
      .objective_sample_sequence =
          request.objective != nullptr ? request.objective->sample_sequence : 0U,
      .stamp_ns = request.stamp_ns,
  };
  if (request.world == nullptr || request.objective == nullptr ||
      !request.navigation.valid) {
    outcome.status = RouteLifecycleTrackingRefreshStatus3D::kInvalidRequest;
    return outcome;
  }
  const std::shared_ptr<const ExecutionPlan3D> execution = execution_supervisor_.plan();
  const CertifiedRouteSuffix3D* const active_route =
      execution != nullptr ? execution->route() : nullptr;
  if (active_route == nullptr || !active_route->valid()) {
    outcome.status = RouteLifecycleTrackingRefreshStatus3D::kInvalidRequest;
    return outcome;
  }
  outcome.base_route_generation = active_route->identity.generation;
  const Point3 current{request.navigation.state.x, request.navigation.state.y,
                       request.navigation.state.z};
  const Point3 planning_goal = staticRoutePlanningGoal(
      current, request.objective->goal, config_.static_route_lookahead_m);
  if (staticRoutePointInsideEsdf(request.world->grid, current,
                                 config_.tracking_world_refresh_margin_m) &&
      staticRoutePointInsideEsdf(request.world->grid, planning_goal,
                                 config_.tracking_world_refresh_margin_m)) {
    return outcome;
  }

  {
    const std::scoped_lock lock{lifecycle_mutex_};
    if (extension_request_in_flight_ || replan_gate_.inFlight() ||
        !replan_gate_.tryBegin(outcome.base_route_generation)) {
      outcome.status = RouteLifecycleTrackingRefreshStatus3D::kLifecycleBusy;
      return outcome;
    }
    replan_in_flight_transaction_.reset();
    replan_in_flight_mission_epoch_ =
        request.objective != nullptr ? request.objective->mission_epoch : 0U;
  }
  RouteLifecycleWorldRefreshResult3D refresh;
  try {
    refresh = config_.world_refresh_requester(
        outcome.base_route_generation,
        RouteLifecycleWorldRefreshPurpose3D::kTrackingObjective);
  } catch (...) {
    finishReplan(outcome.base_route_generation, false);
    throw;
  }
  if (!refresh.valid()) {
    finishReplan(outcome.base_route_generation, false);
    outcome.status = RouteLifecycleTrackingRefreshStatus3D::kWorldRefreshUnavailable;
    return outcome;
  }
  outcome.sequence = refresh.sequence;
  outcome.status = RouteLifecycleTrackingRefreshStatus3D::kQueued;
  return outcome;
}

void RouteLifecycleCoordinator3D::finishWorldRefresh(
    const std::uint64_t base_generation,
    const RouteLifecycleWorldRefreshPurpose3D purpose, const bool route_activated) {
  if (purpose == RouteLifecycleWorldRefreshPurpose3D::kTrackingObjective) {
    finishReplan(base_generation, route_activated);
  } else {
    finishExtension(base_generation, route_activated);
  }
}

void RouteLifecycleCoordinator3D::finishSearch(
    const PlannerSearchTransaction3D& transaction, const bool route_activated) {
  if (transaction.extension()) {
    finishExtension(transaction.request.base_route_generation, route_activated);
  }
  if (transaction.replacement()) {
    finishReplan(transaction.request.base_route_generation, route_activated);
  }
}

} // namespace drone_city_nav
