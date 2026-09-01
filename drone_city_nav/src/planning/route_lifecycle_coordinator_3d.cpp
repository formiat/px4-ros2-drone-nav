#include "route_lifecycle_coordinator_3d.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point started) noexcept {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                   started)
      .count();
}

[[nodiscard]] RouteLifecycleCandidateSummary3D
summarizeCandidate(const std::optional<RouteSearchCandidate3D>& candidate) noexcept {
  if (!candidate.has_value()) {
    return {};
  }
  const SpatialRouteCandidate3D& spatial = candidate->spatial_route;
  return RouteLifecycleCandidateSummary3D{
      .source = spatial.source,
      .point_count = spatial.points.size(),
      .estimated_execution_time_s = spatial.estimated_execution_time_s,
      .estimated_translation_time_s = spatial.estimated_translation_time_s,
      .estimated_stationary_turn_time_s = spatial.estimated_stationary_turn_time_s,
      .available = true,
  };
}

} // namespace

std::string_view routeLifecycleExtensionStatus3DName(
    const RouteLifecycleExtensionStatus3D status) noexcept {
  switch (status) {
    case RouteLifecycleExtensionStatus3D::kNotRequired:
      return "not_required";
    case RouteLifecycleExtensionStatus3D::kInvalidRequest:
      return "invalid_request";
    case RouteLifecycleExtensionStatus3D::kInvalidTransaction:
      return "invalid_transaction";
    case RouteLifecycleExtensionStatus3D::kRouteQueueBusy:
      return "route_queue_busy";
    case RouteLifecycleExtensionStatus3D::kWorldRefreshUnavailable:
      return "world_refresh_unavailable";
    case RouteLifecycleExtensionStatus3D::kSearchQueued:
      return "search_queued";
    case RouteLifecycleExtensionStatus3D::kWorldRefreshQueued:
      return "world_refresh_queued";
  }
  return "unknown";
}

std::string_view
routeLifecycleReplanStatus3DName(const RouteLifecycleReplanStatus3D status) noexcept {
  switch (status) {
    case RouteLifecycleReplanStatus3D::kObjectiveUnavailable:
      return "objective_unavailable";
    case RouteLifecycleReplanStatus3D::kDeferredDuringExtension:
      return "deferred_during_extension";
    case RouteLifecycleReplanStatus3D::kDeferredReplanInFlight:
      return "deferred_replan_in_flight";
    case RouteLifecycleReplanStatus3D::kInvalidResidentWorld:
      return "invalid_resident_world";
    case RouteLifecycleReplanStatus3D::kWaitingInitialSearch:
      return "waiting_initial_search";
    case RouteLifecycleReplanStatus3D::kGenerationMismatch:
      return "generation_mismatch";
    case RouteLifecycleReplanStatus3D::kWaitingRawSnapshot:
      return "waiting_raw_snapshot";
    case RouteLifecycleReplanStatus3D::kInvalidStart:
      return "invalid_start";
    case RouteLifecycleReplanStatus3D::kSuppressedFailedSearch:
      return "suppressed_failed_search";
    case RouteLifecycleReplanStatus3D::kInvalidTransaction:
      return "invalid_transaction";
    case RouteLifecycleReplanStatus3D::kRouteQueueBusy:
      return "route_queue_busy";
    case RouteLifecycleReplanStatus3D::kQueued:
      return "queued";
  }
  return "unknown";
}

std::string_view routeLifecycleTrackingRefreshStatus3DName(
    const RouteLifecycleTrackingRefreshStatus3D status) noexcept {
  switch (status) {
    case RouteLifecycleTrackingRefreshStatus3D::kNotRequired:
      return "not_required";
    case RouteLifecycleTrackingRefreshStatus3D::kInvalidRequest:
      return "invalid_request";
    case RouteLifecycleTrackingRefreshStatus3D::kLifecycleBusy:
      return "lifecycle_busy";
    case RouteLifecycleTrackingRefreshStatus3D::kWorldRefreshUnavailable:
      return "world_refresh_unavailable";
    case RouteLifecycleTrackingRefreshStatus3D::kQueued:
      return "queued";
  }
  return "unknown";
}

RouteLifecycleCoordinator3D::RouteLifecycleCoordinator3D(
    ExecutionSupervisor3D& execution_supervisor,
    RouteLifecycleCoordinatorConfig3D config)
    : execution_supervisor_{execution_supervisor},
      config_{std::move(config)},
      route_materializer_{config_.materializer},
      route_activation_coordinator_{config_.activation} {
  if (!config_.vehicle_state_provider || !config_.activation_snapshot_provider ||
      !config_.activation_commit_boundary || !config_.tracking_context_provider ||
      !config_.replan_snapshot_provider || !config_.world_refresh_requester ||
      !config_.stamp_provider || !config_.update_handler ||
      !config_.rejection_handler || config_.static_route_lookahead_m <= 0.0 ||
      config_.tracking_world_refresh_margin_m < 0.0 ||
      !staticRouteExtensionConfigValid(config_.extension)) {
    throw std::invalid_argument{"route lifecycle coordinator ports are incomplete"};
  }
  route_planning_coordinator_ =
      std::make_unique<RoutePlanningCoordinator3D>(RoutePlanningCoordinatorConfig3D{
          .planner = config_.planner,
          .vehicle_state_provider = config_.vehicle_state_provider,
          .resident_route_generation_provider =
              [this]() {
                const std::shared_ptr<const ExecutionPlan3D> plan =
                    execution_supervisor_.plan();
                return plan != nullptr ? plan->routeGenerationHighWater() : 0U;
              },
          .update_handler =
              [this](RoutePlanningUpdateEvent3D event) {
                handleWorkerUpdate(std::move(event));
              },
          .rejection_handler =
              [this](const RoutePlanningRejection3D& rejection) {
                handleWorkerRejection(rejection);
              },
          .failure_handler =
              [this](const std::exception_ptr failure) { handleFailure(failure); },
      });
}

RouteLifecycleCoordinator3D::~RouteLifecycleCoordinator3D() {
  stop();
}

void RouteLifecycleCoordinator3D::start() {
  route_planning_coordinator_->start();
}

void RouteLifecycleCoordinator3D::stop() noexcept {
  route_planning_coordinator_->stop();
}

RoutePlanningEnqueueResult3D
RouteLifecycleCoordinator3D::enqueue(RoutePlanningRequest3D request,
                                     const RoutePlanningQueuePolicy3D policy) {
  const std::shared_ptr<const PlannerSearchTransaction3D> replacement =
      request.transaction;
  RoutePlanningEnqueueResult3D result =
      route_planning_coordinator_->enqueue(std::move(request), policy);
  if (result.displaced.has_value() && result.displaced->transaction != nullptr &&
      (replacement == nullptr || !result.lifecycleTransferredTo(*replacement))) {
    finishSearch(*result.displaced->transaction);
  }
  return result;
}

std::optional<RoutePlanningRequest3D> RouteLifecycleCoordinator3D::cancelPending() {
  std::optional<RoutePlanningRequest3D> cancelled =
      route_planning_coordinator_->cancelPending();
  if (cancelled.has_value() && cancelled->transaction != nullptr) {
    finishSearch(*cancelled->transaction);
  }
  return cancelled;
}

bool RouteLifecycleCoordinator3D::pending() const noexcept {
  return route_planning_coordinator_->pending();
}

RoutePlanningCoordinatorStatistics3D
RouteLifecycleCoordinator3D::statistics() const noexcept {
  return route_planning_coordinator_->statistics();
}

RouteLifecycleUpdate3D
RouteLifecycleCoordinator3D::advance(RoutePlanningUpdateEvent3D event) {
  RouteLifecycleUpdate3D result;
  if (event.request.transaction == nullptr) {
    return result;
  }

  const auto planning_started = std::chrono::steady_clock::now();
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      event.request.transaction;
  const ProductionWorldBuildTelemetry3D world_telemetry = event.request.world_telemetry;
  RoutePlannerUpdate3D planner_update = std::move(event.update);
  result.request = std::move(event.request);
  result.vehicle_state = event.vehicle_state;
  result.candidate = summarizeCandidate(planner_update.improved_incumbent);
  if (planner_update.planner_session != nullptr) {
    result.search_start = planner_update.planner_session->search_start;
  } else {
    result.search_start = event.vehicle_state.position;
  }
  result.search_running =
      planner_update.planner_invoked && planner_update.dispatch.continue_search;

  if (result.search_running && !result.candidate.available) {
    result.planner_update = std::move(planner_update);
    result.continuation_queued = queueContinuation(result);
    result.route_planning_ms = elapsedMilliseconds(planning_started);
    result.status = RouteLifecycleAdvanceStatus3D::kContinuationQueued;
    if (!result.continuation_queued) {
      finishSearch(*transaction);
    }
    {
      const std::scoped_lock lock{lifecycle_mutex_};
      latest_route_search_ms_ = result.planner_update.search_ms;
      planning_latency_tracker_.record(result.route_planning_ms,
                                       world_telemetry.build_ms);
    }
    observeRecoveryEpisode(transaction->objective.mission_epoch);
    return result;
  }

  const std::uint64_t candidate_generation =
      result.candidate.available ? nextRouteGeneration() : 0U;
  ProductionRouteMaterialization3D materialization;
  materialization.route.world = transaction->world;
  materialization.route.objective = transaction->objective;
  materialization.route.candidate_generation = candidate_generation;
  materialization.telemetry.world_build = world_telemetry;
  materialization.telemetry.route_search_ms = planner_update.search_ms;

  ProductionRouteActivationResult3D activation;
  activation.materialized = materialization.route;
  activation.telemetry = materialization.telemetry;
  if (result.candidate.available && candidate_generation != 0U) {
    const ProductionRouteActivationSnapshot3D materialization_snapshot =
        config_.activation_snapshot_provider();
    const std::shared_ptr<const ExecutionPlan3D> materialization_execution =
        materialization_snapshot.execution_authority != nullptr
            ? materialization_snapshot.execution_authority->plan()
            : nullptr;
    const CertifiedRouteSuffix3D* const active_route =
        materialization_execution != nullptr ? materialization_execution->route()
                                             : nullptr;
    const std::shared_ptr<const CertifiedRouteSuffix3D> active_route_snapshot =
        active_route != nullptr
            ? std::shared_ptr<const CertifiedRouteSuffix3D>{materialization_execution,
                                                            active_route}
            : nullptr;
    RouteSearchCandidate3D candidate =
        std::move(planner_update.improved_incumbent).value_or(RouteSearchCandidate3D{});
    planner_update.improved_incumbent.reset();
    materialization = route_materializer_.materialize(RouteMaterializationRequest3D{
        .transaction = transaction,
        .world_telemetry = world_telemetry,
        .current_position = event.vehicle_state.position,
        .candidate = std::move(candidate),
        .candidate_generation = candidate_generation,
        .active_route = active_route_snapshot,
        .activation_raw_world = materialization_snapshot.raw_world,
    });
    materialization.telemetry.route_search_ms = planner_update.search_ms;
    result.geometry_optimization_fallback =
        materialization.geometry_optimization_fallback;

    ProductionRouteActivationSnapshot3D activation_snapshot =
        config_.activation_snapshot_provider();
    const StaticRoutePlanningLatencyStats planning_latency =
        planningLatencyStatistics();
    PreparedRouteActivation3D prepared =
        route_activation_coordinator_.prepare(RouteActivationPreparationRequest3D{
            .transaction = transaction,
            .materialization = std::move(materialization),
            .snapshot = std::move(activation_snapshot),
            .planning_latency = planning_latency,
        });
    const RouteActivationCommitOperation3D commit_operation =
        [this](PreparedRouteActivation3D pending,
               const RouteActivationCommitContext3D& context) {
          return route_activation_coordinator_.commit(std::move(pending), context,
                                                      execution_supervisor_);
        };
    RouteActivationCommitResult3D committed =
        config_.activation_commit_boundary(std::move(prepared), commit_operation);
    activation = std::move(committed.result);
  }
  activation.telemetry.route_search_ms = planner_update.search_ms;
  result.activation = std::move(activation);

  result.planner_update = std::move(planner_update);
  if (result.search_running) {
    result.continuation_queued = queueContinuation(result);
  }
  observeRecoveryEpisode(transaction->objective.mission_epoch);

  result.route_planning_ms = elapsedMilliseconds(planning_started);
  const bool lifecycle_search = transaction->replacement() || transaction->initial();
  const bool latch_failed_search = [&]() {
    if (!lifecycle_search || result.activation.admission.certified_pending ||
        result.search_running) {
      return false;
    }
    const std::shared_ptr<const ExecutionPlan3D> resident_execution =
        execution_supervisor_.plan();
    const std::uint64_t resident_generation =
        resident_execution != nullptr ? resident_execution->routeGenerationHighWater()
                                      : 0U;
    return staticRouteSearchFailureLatchEligible(transaction->request,
                                                 resident_generation);
  }();
  const std::int64_t failed_search_stamp_ns =
      latch_failed_search ? config_.stamp_provider() : 0;
  {
    const std::scoped_lock lock{lifecycle_mutex_};
    latest_route_search_ms_ = result.planner_update.search_ms;
    planning_latency_tracker_.record(result.route_planning_ms,
                                     world_telemetry.build_ms);

    if (lifecycle_search) {
      if (result.activation.admission.certified_pending) {
        failed_search_latch_.clear();
      } else if (latch_failed_search) {
        const std::uint64_t failed_generation =
            transaction->replacement() ? transaction->request.base_route_generation
                                       : 0U;
        failed_search_latch_.recordFailure(StaticRouteSearchContext{
            .base_route_generation = failed_generation,
            .search_start = result.search_start,
            .objective = transaction->objective,
            .minimum_tracking_sample_sequence =
                result.activation.admission.required_objective_sample,
            .stamp_ns = failed_search_stamp_ns,
        });
        result.failed_search_latched = true;
      }
    } else if (result.activation.admission.certified_pending) {
      failed_search_latch_.clear();
    }
  }

  if (!result.continuation_queued) {
    finishSearch(*transaction, result.activation.admission.certified_pending);
  }

  const RouteLifecycleTrackingContext3D tracking = config_.tracking_context_provider();
  if (tracking.objective != nullptr && tracking.objective->continuous_tracking) {
    const std::uint64_t required_sample =
        tracking.objective->mission_epoch == tracking.minimum_route_mission_epoch
            ? tracking.minimum_route_sample_sequence
            : 0U;
    StaticRouteObjective resident_objective;
    const std::shared_ptr<const ExecutionPlan3D> resident_execution =
        execution_supervisor_.plan();
    if (resident_execution != nullptr && resident_execution->route() != nullptr) {
      resident_objective = resident_execution->route()->identity.proposal.objective;
    }
    if (!staticRouteObjectiveMatches(
            resident_objective, makeStaticRouteObjective(*tracking.objective),
            required_sample, std::numeric_limits<double>::infinity())) {
      result.tracking_followup = RouteLifecycleTrackingFollowup3D{
          .resident_objective = resident_objective,
          .required_mission_epoch = tracking.minimum_route_mission_epoch,
          .required_sample_sequence = required_sample,
          .required = true,
      };
    }
  }
  result.status = RouteLifecycleAdvanceStatus3D::kCompleted;
  return result;
}

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
              });
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
    if (latest_overlay_required) {
      const std::uint64_t minimum_raw_revision = std::max(
          snapshot.blocked_raw_revision, snapshot.resident_world->source_raw_revision);
      const std::shared_ptr<const PersistentPlannerWorld3D> raw_overlay =
          snapshot.latest_raw_world != nullptr
              ? captureObservedRouteSearchWorld3D(
                    *snapshot.latest_raw_world,
                    snapshot.resident_world->proprioceptive_free_space_seed,
                    snapshot.resident_world->launch_support_contact)
              : nullptr;
      if (raw_overlay == nullptr ||
          raw_overlay->producer_instance_id !=
              snapshot.resident_world->producer_instance_id ||
          raw_overlay->revision < minimum_raw_revision) {
        outcome.status = RouteLifecycleReplanStatus3D::kWaitingRawSnapshot;
        return complete(outcome);
      }
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
                                     request_identity, std::nullopt, reason);
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
    }
  }
  if (outcome.status == RouteLifecycleReplanStatus3D::kDeferredDuringExtension ||
      outcome.status == RouteLifecycleReplanStatus3D::kDeferredReplanInFlight) {
    return complete(outcome);
  }
  RoutePlanningEnqueueResult3D enqueue_result;
  try {
    enqueue_result = enqueue(RoutePlanningRequest3D{
        .transaction = transaction,
        .world_telemetry = snapshot.world_telemetry,
        .continuation_session = nullptr,
    });
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

StaticRoutePlanningLatencyStats
RouteLifecycleCoordinator3D::planningLatencyStatistics() const noexcept {
  const std::scoped_lock lock{lifecycle_mutex_};
  return planning_latency_tracker_.stats();
}

std::uint64_t RouteLifecycleCoordinator3D::recoverySequence() const noexcept {
  return recovery_episodes_.sequence();
}

std::uint64_t RouteLifecycleCoordinator3D::nextRouteGeneration() const noexcept {
  const std::shared_ptr<const ExecutionPlan3D> plan = execution_supervisor_.plan();
  const std::uint64_t current_generation =
      plan != nullptr ? plan->routeGenerationHighWater() : 0U;
  return current_generation == std::numeric_limits<std::uint64_t>::max()
             ? 0U
             : current_generation + 1U;
}

bool RouteLifecycleCoordinator3D::queueContinuation(
    const RouteLifecycleUpdate3D& update) {
  if (update.request.transaction == nullptr ||
      update.planner_update.planner_session == nullptr) {
    return false;
  }
  return enqueue(RoutePlanningRequest3D{
                     .transaction = update.request.transaction,
                     .world_telemetry = update.request.world_telemetry,
                     .continuation_session = update.planner_update.planner_session,
                 })
      .queued();
}

void RouteLifecycleCoordinator3D::handleWorkerUpdate(RoutePlanningUpdateEvent3D event) {
  RouteLifecycleUpdate3D update = advance(std::move(event));
  const RouteLifecycleTrackingFollowup3D tracking_followup = update.tracking_followup;
  config_.update_handler(std::move(update));
  if (tracking_followup.required && config_.tracking_followup_handler) {
    config_.tracking_followup_handler(tracking_followup);
  }
}

void RouteLifecycleCoordinator3D::handleWorkerRejection(
    const RoutePlanningRejection3D& rejection) {
  if (rejection.request.transaction != nullptr) {
    finishSearch(*rejection.request.transaction);
  }
  config_.rejection_handler(rejection);
}

void RouteLifecycleCoordinator3D::observeRecoveryEpisode(
    const std::uint64_t mission_epoch) noexcept {
  const RouteExecutionManagerSnapshot3D execution = execution_supervisor_.snapshot();
  const std::shared_ptr<const ExecutionPlan3D> plan = execution.plan();
  const bool recovery_active =
      (plan == nullptr || plan->route() == nullptr) && execution.pending == nullptr;
  static_cast<void>(recovery_episodes_.observe(mission_epoch, recovery_active));
}

void RouteLifecycleCoordinator3D::finishExtension(const std::uint64_t base_generation,
                                                  const bool route_activated) {
  std::optional<StaticRouteDeferredReplan> replay;
  {
    const std::scoped_lock lock{lifecycle_mutex_};
    if (extension_request_in_flight_ &&
        extension_in_flight_generation_ == base_generation) {
      extension_request_in_flight_ = false;
      extension_in_flight_generation_ = 0U;
    }
    replay = deferred_replan_latch_.finishExtension(base_generation, route_activated);
  }
  if (replay.has_value()) {
    replayDeferredReplan(*replay,
                         RouteLifecycleReplanOrigin3D::kDeferredExtensionReplay,
                         base_generation);
  }
}

void RouteLifecycleCoordinator3D::finishReplan(const std::uint64_t base_generation,
                                               const bool route_activated) {
  std::optional<StaticRouteDeferredReplan> replay;
  {
    const std::scoped_lock lock{lifecycle_mutex_};
    replan_gate_.finish(base_generation);
    replay = deferred_replan_latch_.finishReplan(base_generation, route_activated);
  }
  if (replay.has_value()) {
    replayDeferredReplan(*replay, RouteLifecycleReplanOrigin3D::kDeferredReplanReplay,
                         base_generation);
  }
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

void RouteLifecycleCoordinator3D::handleFailure(
    const std::exception_ptr failure) const noexcept {
  if (!config_.failure_handler) {
    return;
  }
  try {
    config_.failure_handler(failure);
  } catch (...) {
    static_cast<void>(std::current_exception());
  }
}

} // namespace drone_city_nav
