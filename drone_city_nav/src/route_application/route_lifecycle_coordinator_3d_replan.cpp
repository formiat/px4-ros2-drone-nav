#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

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
      retire_in_flight_search = replan_gate_.inFlight() &&
                                (objective_superseded || release_needs_fresh_search);
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
    outcome.cleared_gate_generation =
        replan_gate_.finishIfSupersededBy(snapshot.committed_route_generation);
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
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(snapshot.resident_world, std::move(planner_world),
                                     makeStaticRouteObjective(*snapshot.objective),
                                     request_identity, std::nullopt, reason,
                                     config_.stamp_provider());
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
      replan_in_flight_mission_epoch_ = snapshot.objective->mission_epoch;
      replan_in_flight_published_ = false;
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
