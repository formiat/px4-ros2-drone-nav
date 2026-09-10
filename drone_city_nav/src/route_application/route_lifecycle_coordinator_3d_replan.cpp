#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "production_mppi_node_types.hpp"
#include "production_mppi_route_world.hpp"
#include "route_lifecycle_coordinator_3d.hpp"

namespace drone_city_nav {

// Replan requests: the gate that serialises searches per route generation,
// the deferral of releases raised while a search is in flight, and the
// world a replan searches on.
RouteLifecycleReplanOutcome3D
RouteLifecycleCoordinator3D::requestReplan(const RouteReleaseReason3D reason,
                                           const std::uint64_t route_generation) {
  return requestReplanImpl(reason, route_generation,
                           RouteLifecycleReplanOrigin3D::kRequested);
}

RouteLifecycleReplanOutcome3D RouteLifecycleCoordinator3D::requestReplanImpl(
    const RouteReleaseReason3D reason, const std::uint64_t route_generation,
    const RouteLifecycleReplanOrigin3D origin,
    const std::uint64_t replay_completed_generation) {
  const RouteLifecycleReplanSnapshot3D snapshot = config_.replan_snapshot_provider();
  RouteLifecycleReplanOutcome3D outcome;
  outcome.origin = origin;
  outcome.reason = reason;
  outcome.requested_route_generation = route_generation;
  outcome.replay_completed_generation = replay_completed_generation;
  outcome.committed_route_generation = snapshot.committed_route_generation;
  outcome.resident_world_revision =
      snapshot.resident_world != nullptr ? snapshot.resident_world->revision : 0U;
  outcome.resident_source_raw_revision =
      snapshot.resident_world != nullptr ? snapshot.resident_world->source_raw_revision
                                         : 0U;
  outcome.latest_raw_revision = snapshot.latest_raw_world != nullptr
                                    ? snapshot.latest_raw_world->version().revision
                                    : 0U;
  outcome.blocked_raw_revision = snapshot.blocked_raw_revision;
  const auto complete = [this](const RouteLifecycleReplanOutcome3D& value) {
    notifyReplanOutcome(value);
    return value;
  };
  if (snapshot.objective == nullptr) {
    return complete(outcome);
  }

  {
    // A mission that moved to its next leg superseded the objective an
    // in-flight search serves. Waiting for that search to converge would defer
    // the new leg behind a route nobody will execute, so the search is retired
    // here: a queued continuation is dropped, its gate is released, and what
    // it had deferred is discarded in favour of this request. A continuation
    // already running returns to advance(), which discards its result by
    // objective epoch. A refresh of the same objective (a tracking sample)
    // keeps deferring behind the in-flight search as before.
    bool retire_in_flight_search{false};
    {
      const std::scoped_lock lock{lifecycle_mutex_};
      const bool objective_superseded =
          reason == RouteReleaseReason3D::kObjectiveChanged &&
          replan_in_flight_mission_epoch_ != 0U &&
          replan_in_flight_mission_epoch_ != snapshot.objective->mission_epoch;
      // A route released as blocked or diverged, or a vehicle without any
      // route, needs a fresh search from where it is now; an in-flight search
      // that has already delivered its candidate only runs for improvements
      // of a route the executor just gave up on.
      const bool release_needs_fresh_search =
          replan_in_flight_published_ &&
          (reason == RouteReleaseReason3D::kBlocked ||
           reason == RouteReleaseReason3D::kDiverged ||
           reason == RouteReleaseReason3D::kNoActiveRoute);
      // A search that has delivered nothing at all is not improving anything:
      // it is what the vehicle is waiting on, and it keeps every later
      // request deferred behind it. Its labels are seeded on the world and
      // the pose it opened with, and a stationary vehicle never moves them,
      // so a session that has produced nothing for a whole retry interval
      // will most likely produce nothing on the next one either. One
      // recorded flight stood for five and a half seconds while such a
      // session ran on, and the fresh session that replaced it found a route
      // in its first update.
      const double in_flight_age_s =
          replan_in_flight_stamp_ns_ != 0 &&
                  snapshot.stamp_ns > replan_in_flight_stamp_ns_
              ? static_cast<double>(snapshot.stamp_ns - replan_in_flight_stamp_ns_) /
                    1.0e9
              : 0.0;
      // Measured in the search's own units: five planner updates are what a
      // first-found route takes wherever one exists, and a session that has
      // spent them without publishing is not about to. The retry interval
      // bounds it from above, so a planner whose updates are long is not
      // restarted faster than the latch would allow anyway.
      constexpr double kUnpublishedSearchRetirementUpdates{5.0};
      const StaticRoutePlanningLatencyStats update_latency =
          planner_update_latency_tracker_.stats();
      const double unpublished_budget_s =
          update_latency.sample_count > 0U
              ? std::min(std::max(0.0, config_.search_retry.minimum_retry_interval_s),
                         kUnpublishedSearchRetirementUpdates * 0.001 *
                             update_latency.planning_p95_ms)
              : std::max(0.0, config_.search_retry.minimum_retry_interval_s);
      const bool unpublished_search_outlived_its_retry =
          !replan_in_flight_published_ &&
          reason == RouteReleaseReason3D::kNoActiveRoute &&
          in_flight_age_s >= unpublished_budget_s;
      retire_in_flight_search = replan_gate_.inFlight() &&
                                (objective_superseded || release_needs_fresh_search ||
                                 unpublished_search_outlived_its_retry);
      if (retire_in_flight_search) {
        const std::uint64_t in_flight_generation = replan_gate_.generation();
        replan_gate_.finish(in_flight_generation);
        static_cast<void>(
            deferred_replan_latch_.finishReplan(in_flight_generation, false));
        outcome.cleared_gate_generation = in_flight_generation;
      }
    }
    if (retire_in_flight_search) {
      static_cast<void>(route_planning_coordinator_->cancelPending());
    }
  }
  {
    const std::scoped_lock lock{lifecycle_mutex_};
    // A gate this request already retired above is reported as retired; the
    // superseded check speaks only for a gate that is still in flight.
    if (const std::optional<std::uint64_t> superseded =
            replan_gate_.finishIfSupersededBy(snapshot.committed_route_generation)) {
      outcome.cleared_gate_generation = superseded;
    }
    if (deferStaticRouteReleaseDuringExtension(extension_request_in_flight_, reason)) {
      outcome.deferred_route_generation =
          route_generation != 0U ? route_generation : extension_in_flight_generation_;
      outcome.in_flight_generation = extension_in_flight_generation_;
      deferred_replan_latch_.defer(StaticRouteDeferredReplan{
          .reason = reason,
          .route_generation = outcome.deferred_route_generation,
      });
      outcome.status = RouteLifecycleReplanStatus3D::kDeferredDuringExtension;
    } else if (replan_gate_.inFlight()) {
      outcome.in_flight_generation = replan_gate_.generation();
      outcome.deferred_route_generation =
          route_generation != 0U ? route_generation : outcome.in_flight_generation;
      deferred_replan_latch_.defer(StaticRouteDeferredReplan{
          .reason = reason,
          .route_generation = outcome.deferred_route_generation,
      });
      outcome.status = RouteLifecycleReplanStatus3D::kDeferredReplanInFlight;
    }
  }
  if (outcome.status == RouteLifecycleReplanStatus3D::kDeferredDuringExtension ||
      outcome.status == RouteLifecycleReplanStatus3D::kDeferredReplanInFlight) {
    return complete(outcome);
  }

  if (snapshot.resident_world == nullptr ||
      !productionWorldGenerationCoherent(*snapshot.resident_world) ||
      snapshot.resident_planner_world == nullptr) {
    outcome.status = RouteLifecycleReplanStatus3D::kInvalidResidentWorld;
    return complete(outcome);
  }
  outcome.search_generation = staticRouteSearchGeneration(
      true, snapshot.committed_route_generation, snapshot.committed_route_generation);
  bool failed_search_latched{false};
  {
    const std::scoped_lock lock{lifecycle_mutex_};
    failed_search_latched = failed_search_latch_.latched();
  }
  if (snapshot.committed_route_generation == 0U && !failed_search_latched) {
    outcome.status = RouteLifecycleReplanStatus3D::kWaitingInitialSearch;
    return complete(outcome);
  }
  if (route_generation != 0U && outcome.search_generation != route_generation) {
    outcome.status = RouteLifecycleReplanStatus3D::kGenerationMismatch;
    return complete(outcome);
  }

  std::shared_ptr<const PersistentPlannerWorld3D> planner_world =
      snapshot.resident_planner_world;
  if (config_.observed_world) {
    const bool latest_overlay_required =
        routeSearchRequiresLatestRawOverlay3D(reason) ||
        snapshot.blocked_raw_revision > snapshot.resident_world->source_raw_revision;
    const std::shared_ptr<const PersistentPlannerWorld3D> raw_overlay =
        snapshot.latest_raw_world != nullptr
            ? captureObservedRouteSearchWorld3D(
                  *snapshot.latest_raw_world,
                  snapshot.resident_world->proprioceptive_free_space_seed,
                  snapshot.resident_world->launch_support_contact)
            : nullptr;
    const bool raw_overlay_coherent =
        raw_overlay != nullptr && raw_overlay->producer_instance_id ==
                                      snapshot.resident_world->producer_instance_id;
    if (latest_overlay_required) {
      const std::uint64_t minimum_raw_revision = std::max(
          snapshot.blocked_raw_revision, snapshot.resident_world->source_raw_revision);
      if (!raw_overlay_coherent || raw_overlay->revision < minimum_raw_revision) {
        outcome.status = RouteLifecycleReplanStatus3D::kWaitingRawSnapshot;
        return complete(outcome);
      }
    }
    // Every search starts on the newest coherent occupied evidence; a
    // continuation keeps following it, so an initial request must not lag
    // behind the planner's resident world.
    if (raw_overlay_coherent &&
        (planner_world == nullptr || raw_overlay->revision > planner_world->revision)) {
      planner_world = raw_overlay;
      outcome.raw_search_overlay_used = true;
      if (snapshot.blocked_raw_revision != 0U) {
        outcome.dispatched_raw_revision = raw_overlay->revision;
      }
    }
  }

  const std::uint64_t required_sample =
      snapshot.objective->mission_epoch == snapshot.minimum_route_mission_epoch
          ? snapshot.minimum_route_sample_sequence
          : 0U;
  const StaticRouteSearchContext retry_context{
      .base_route_generation = outcome.search_generation,
      .search_start = Point3{snapshot.navigation.state.x, snapshot.navigation.state.y,
                             snapshot.navigation.state.z},
      .objective = makeStaticRouteObjective(*snapshot.objective),
      .minimum_tracking_sample_sequence = required_sample,
      .stamp_ns = snapshot.stamp_ns,
      .raw_revision = planner_world != nullptr ? planner_world->revision : 0U,
  };
  outcome.search_start = retry_context.search_start;
  if (!snapshot.navigation.valid ||
      (failed_search_latched &&
       !insideFlightEnvelope(retry_context.search_start, config_.flight_envelope))) {
    outcome.status = RouteLifecycleReplanStatus3D::kInvalidStart;
    return complete(outcome);
  }
  {
    const std::scoped_lock lock{lifecycle_mutex_};
    outcome.retry = failed_search_latch_.evaluate(config_.search_retry, retry_context);
  }
  if (!outcome.retry.allow) {
    outcome.status = RouteLifecycleReplanStatus3D::kSuppressedFailedSearch;
    return complete(outcome);
  }

  const StaticRouteSearchRequestIdentity request_identity{
      .kind = outcome.search_generation == 0U
                  ? StaticRouteSearchRequestKind::kInitialRetry
                  : StaticRouteSearchRequestKind::kReplan,
      .base_route_generation = outcome.search_generation,
  };
  // A route blocked far enough ahead is replaced onto its own certified
  // prefix: the successor stitches one overlap ahead of the vehicle and at
  // least one overlap short of the block, so the geometry the vehicle is
  // flying now stays, the executor splices instead of turning the vehicle
  // around, and the successor is searched from where the incumbent already
  // is. A block inside that reach leaves nothing certified worth keeping.
  std::optional<PlannerSearchContinuityBase3D> continuity_base;
  if (request_identity.kind == StaticRouteSearchRequestKind::kReplan &&
      reason == RouteReleaseReason3D::kBlocked && snapshot.active_route != nullptr &&
      snapshot.active_route->valid() &&
      snapshot.active_route->identity.generation == outcome.search_generation &&
      snapshot.route_projection.valid && snapshot.blocked_station_m.has_value() &&
      std::isfinite(*snapshot.blocked_station_m)) {
    // Where the stitch may lie is decided by the vehicle's own motion, not by
    // the certified overlap. Ahead of it, the prefix has to still be in front
    // of the vehicle when the successor arrives: the distance covered while
    // the search runs, plus the path it would need to stop. Short of the
    // block, the stitch has to leave the vehicle room to turn onto the
    // successor: the same stopping path. The overlap is what the executor
    // keeps certified ahead of itself while it flies, and charging it at both
    // ends here asked for a block seen sixteen and a half metres ahead --
    // measured over four urban flights, blocked cells are found four point
    // nine metres ahead at the median and ten point eight at the ninetieth
    // percentile, so the stitch never applied and every blocked route was
    // replaced by a search from the vehicle, which stopped for it.
    const StaticRoutePlanningLatencyStats latency = planning_latency_tracker_.stats();
    const double latency_s =
        0.001 * (latency.sample_count > 0U ? latency.planning_p99_ms
                                           : std::max(0.0, latest_route_search_ms_));
    const double horizontal_speed_mps =
        std::hypot(snapshot.navigation.state.vx, snapshot.navigation.state.vy);
    const double vertical_speed_mps = std::abs(snapshot.navigation.state.vz);
    const double speed_mps = std::hypot(horizontal_speed_mps, vertical_speed_mps);
    const ProductionMppiForwardAcceleration3D forward_acceleration =
        productionMppiForwardAcceleration3D(snapshot.navigation);
    const JerkLimitedStoppingDistance3D stopping = jerkLimitedStoppingDistance3D(
        horizontal_speed_mps, forward_acceleration.horizontal_mps2, vertical_speed_mps,
        forward_acceleration.vertical_mps2, config_.extension);
    const double stopping_path_m = stopping.valid()
                                       ? stopping.route_station_m
                                       : config_.extension.required_certified_overlap_m;
    const double minimum_stitch_m =
        snapshot.route_projection.station_m + stopping_path_m + speed_mps * latency_s;
    const double stitch_limit_m = *snapshot.blocked_station_m - stopping_path_m;
    if (std::isfinite(minimum_stitch_m) && stitch_limit_m >= minimum_stitch_m) {
      continuity_base = PlannerSearchContinuityBase3D{
          .route = snapshot.active_route,
          .request_projection = snapshot.route_projection,
          .stitch_limit_station_m = stitch_limit_m,
          .minimum_stitch_station_m = minimum_stitch_m,
      };
      outcome.stitch_limit_station_m = stitch_limit_m;
    }
  }
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(snapshot.resident_world, std::move(planner_world),
                                     makeStaticRouteObjective(*snapshot.objective),
                                     request_identity, std::move(continuity_base),
                                     reason, config_.stamp_provider());
  if (transaction == nullptr) {
    outcome.status = RouteLifecycleReplanStatus3D::kInvalidTransaction;
    return complete(outcome);
  }

  {
    const std::scoped_lock lock{lifecycle_mutex_};
    if (deferStaticRouteReleaseDuringExtension(extension_request_in_flight_, reason)) {
      outcome.deferred_route_generation =
          route_generation != 0U ? route_generation : extension_in_flight_generation_;
      outcome.in_flight_generation = extension_in_flight_generation_;
      deferred_replan_latch_.defer(StaticRouteDeferredReplan{
          .reason = reason,
          .route_generation = outcome.deferred_route_generation,
      });
      outcome.status = RouteLifecycleReplanStatus3D::kDeferredDuringExtension;
    } else if (!replan_gate_.tryBegin(outcome.search_generation)) {
      outcome.in_flight_generation = replan_gate_.generation();
      outcome.deferred_route_generation =
          route_generation != 0U ? route_generation : outcome.in_flight_generation;
      deferred_replan_latch_.defer(StaticRouteDeferredReplan{
          .reason = reason,
          .route_generation = outcome.deferred_route_generation,
      });
      outcome.status = RouteLifecycleReplanStatus3D::kDeferredReplanInFlight;
    } else {
      replan_in_flight_transaction_.reset();
      replan_in_flight_mission_epoch_ = snapshot.objective->mission_epoch;
      replan_in_flight_published_ = false;
      replan_in_flight_stamp_ns_ = snapshot.stamp_ns;
    }
  }
  if (outcome.status == RouteLifecycleReplanStatus3D::kDeferredDuringExtension ||
      outcome.status == RouteLifecycleReplanStatus3D::kDeferredReplanInFlight) {
    return complete(outcome);
  }
  RoutePlanningEnqueueResult3D enqueue_result;
  try {
    // A vehicle without a route asks for a search; a search already queued
    // for it is that search, so the request waits instead of displacing it
    // and restarting the planner session every tick.
    enqueue_result = enqueue(
        RoutePlanningRequest3D{
            .transaction = transaction,
            .world_telemetry = snapshot.world_telemetry,
            .continuation_session = nullptr,
        },
        reason == RouteReleaseReason3D::kNoActiveRoute
            ? RoutePlanningQueuePolicy3D::kKeepPending
            : RoutePlanningQueuePolicy3D::kReplacePending);
  } catch (...) {
    finishReplan(outcome.search_generation, false);
    throw;
  }
  outcome.enqueue_status = enqueue_result.status;
  if (!enqueue_result.queued()) {
    outcome.status = RouteLifecycleReplanStatus3D::kRouteQueueBusy;
    notifyReplanOutcome(outcome);
    finishReplan(outcome.search_generation, false);
    return outcome;
  }
  outcome.status = RouteLifecycleReplanStatus3D::kQueued;
  return complete(outcome);
}

void RouteLifecycleCoordinator3D::replayDeferredReplan(
    const StaticRouteDeferredReplan& replay, const RouteLifecycleReplanOrigin3D origin,
    const std::uint64_t completed_generation) noexcept {
  if (origin == RouteLifecycleReplanOrigin3D::kRequested) {
    return;
  }
  const std::uint64_t requested_generation =
      origin == RouteLifecycleReplanOrigin3D::kDeferredExtensionReplay
          ? replay.route_generation
          : 0U;
  try {
    static_cast<void>(requestReplanImpl(replay.reason, requested_generation, origin,
                                        completed_generation));
  } catch (...) {
    handleFailure(std::current_exception());
  }
}

void RouteLifecycleCoordinator3D::notifyReplanOutcome(
    const RouteLifecycleReplanOutcome3D& outcome) const {
  if (!config_.replan_outcome_handler) {
    return;
  }
  try {
    config_.replan_outcome_handler(outcome);
  } catch (...) {
    handleFailure(std::current_exception());
  }
}

} // namespace drone_city_nav
