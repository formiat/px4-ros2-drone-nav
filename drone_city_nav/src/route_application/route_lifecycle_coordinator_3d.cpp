#include "route_lifecycle_coordinator_3d.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "production_mppi_route_world.hpp"
#include "route_lifecycle_refusal_3d.hpp"

namespace drone_city_nav {

namespace {

// One retry covers the ordinary case of a world, objective, or execution
// snapshot landing between capture and commit. Beyond that the lifecycle is
// losing a race it cannot win by repeating itself.
inline constexpr std::size_t kMaximumActivationAttempts{2U};

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point started) noexcept {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                   started)
      .count();
}

[[nodiscard]] RouteCandidateDisposition3D
classifyCandidateDisposition(const PlannerSearchTransaction3D& transaction,
                             const ProductionRouteActivationResult3D& activation,
                             const double required_certified_overlap_m) {
  const RouteAdmissionReport3D& admission = activation.admission;
  if (admission.certified_pending) {
    return RouteCandidateDisposition3D::kActivated;
  }
  // A newer world refusing the candidate within what the vehicle is committed
  // to -- its departure and the certified overlap -- is a route it cannot
  // enter, and the search that produced it starts again from the vehicle.
  // Refused beyond that, the candidate is enterable and the refusal is a block
  // ahead, the same thing a blocked release is: the persistent search repairs
  // the blocked part against the world it re-validates every update. Retiring
  // it for that restarted the feasibility search from nothing on every raw
  // revision that arrived between search and activation -- two a second --
  // and the recorded holds after a stop were one to three such restarts long.
  if (route_lifecycle_refusal_3d::searchInvalidatedByActivationWorld(transaction,
                                                                     admission) &&
      !route_lifecycle_refusal_3d::refusedBeyondTheCommittedRoute3D(
          activation, required_certified_overlap_m)) {
    return RouteCandidateDisposition3D::kRetireSearchAndReplan;
  }
  // A superseded snapshot says only that the commit base moved between capture
  // and commit. The candidate remains exactly what the search produced.
  if (admission.activation_status ==
      StaticRouteActivationStatus::kActivationSnapshotSuperseded) {
    return RouteCandidateDisposition3D::kRetrySameCandidate;
  }
  // The commit base moved past the base this search was opened on (its own
  // candidate was activated, or another route was): nothing the session can
  // still deliver activates, so continuing it would only repeat the stale
  // delivery every update. The lifecycle replans from the current base.
  if (admission.activation_status ==
          StaticRouteActivationStatus::kStaleRouteGeneration &&
      (transaction.replacement() || transaction.extension())) {
    return RouteCandidateDisposition3D::kRetireSearchAndReplan;
  }
  // A replacement stitched onto its incumbent whose prefix can no longer be
  // materialized, because the vehicle passed the stitch or the incumbent went,
  // delivers the same unusable candidate every update; the session is retired
  // and the replan decides afresh whether anything is left to stitch onto.
  if (transaction.replacement() && transaction.continuity_base.has_value() &&
      admission.activation_status ==
          StaticRouteActivationStatus::kCandidateValidationRejected &&
      admission.candidate_validation.status ==
          StaticRouteCandidateStatus::kInvalidPassageSpan) {
    return RouteCandidateDisposition3D::kRetireSearchAndReplan;
  }
  return RouteCandidateDisposition3D::kContinueForImprovement;
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
  // How long ago this search was asked for: the queue, every continuation and
  // this activation attempt. Recorded on the update that ends the search, so
  // the tracker holds complete measurements only.
  const bool search_latency_measured = transaction->requested_stamp_ns > 0;
  const double search_latency_ms =
      search_latency_measured
          ? std::max(0.0, static_cast<double>(config_.stamp_provider() -
                                              transaction->requested_stamp_ns) *
                              1.0e-6)
          : 0.0;
  const ProductionWorldBuildTelemetry3D world_telemetry = event.request.world_telemetry;
  RoutePlannerUpdate3D planner_update = std::move(event.update);
  result.request = std::move(event.request);
  result.vehicle_state = event.vehicle_state;
  result.candidate = summarizeCandidate(planner_update.improved_incumbent);
  if (result.candidate.available &&
      (transaction->replacement() || transaction->initial())) {
    const std::scoped_lock lock{lifecycle_mutex_};
    if (replan_gate_.inFlight() &&
        replan_gate_.generation() == transaction->request.base_route_generation) {
      replan_in_flight_published_ = true;
    }
  }
  if (planner_update.planner_session != nullptr) {
    result.search_start = planner_update.planner_session->search_start;
  } else {
    result.search_start = event.vehicle_state.position;
  }
  result.search_running =
      planner_update.planner_invoked && planner_update.dispatch.continue_search;
  // A search for an objective the mission has since left is finished here,
  // whatever it found: its continuation would keep the planner on the old leg
  // and its candidate would be rejected as stale at activation anyway.
  {
    const RouteLifecycleReplanSnapshot3D current = config_.replan_snapshot_provider();
    const bool objective_stale =
        current.objective != nullptr && transaction->objective.mission_epoch != 0U &&
        current.objective->mission_epoch != transaction->objective.mission_epoch;
    if (objective_stale) {
      finishSearch(*transaction);
      result.search_running = false;
      result.candidate = {};
      result.planner_update = std::move(planner_update);
      result.route_planning_ms = elapsedMilliseconds(planning_started);
      result.route_search_latency_ms = search_latency_ms;
      result.status = RouteLifecycleAdvanceStatus3D::kCompleted;
      return result;
    }
  }

  if (result.search_running && !result.candidate.available) {
    result.planner_update = std::move(planner_update);
    result.search_retired = searchRetired(*transaction);
    result.continuation_queued = queueContinuation(result);
    result.route_planning_ms = elapsedMilliseconds(planning_started);
    result.status = RouteLifecycleAdvanceStatus3D::kContinuationQueued;
    if (!result.continuation_queued) {
      finishSearch(*transaction);
    }
    result.route_search_latency_ms = search_latency_ms;
    {
      const std::scoped_lock lock{lifecycle_mutex_};
      latest_route_search_ms_ = result.planner_update.search_ms;
      planner_update_latency_tracker_.record(result.planner_update.search_ms,
                                             world_telemetry.build_ms);
    }
    observeRecoveryEpisode(transaction->objective.mission_epoch);
    return result;
  }

  ProductionRouteActivationResult3D activation;
  activation.materialized.world = transaction->world;
  activation.materialized.objective = transaction->objective;
  activation.telemetry.world_build = world_telemetry;
  activation.telemetry.route_search_ms = planner_update.search_ms;
  if (result.candidate.available) {
    // The lifecycle owns the candidate until it reaches a terminal disposition.
    // Activation is optimistic, so a commit base that moves underneath it is a
    // reason to try again against the current snapshot, not to throw away a
    // route the search already proved.
    const RouteSearchCandidate3D candidate =
        std::move(planner_update.improved_incumbent).value_or(RouteSearchCandidate3D{});
    planner_update.improved_incumbent.reset();
    const RouteActivationCommitOperation3D commit_operation =
        [this](PreparedRouteActivation3D pending,
               const RouteActivationCommitContext3D& context) {
          return route_activation_coordinator_.commit(std::move(pending), context,
                                                      execution_supervisor_);
        };
    while (result.activation_attempts < kMaximumActivationAttempts) {
      ++result.activation_attempts;
      const std::uint64_t candidate_generation = nextRouteGeneration();
      if (candidate_generation == 0U) {
        break;
      }
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
      ProductionRouteMaterialization3D materialization =
          route_materializer_.materialize(RouteMaterializationRequest3D{
              .transaction = transaction,
              .world_telemetry = world_telemetry,
              .current_position = event.vehicle_state.position,
              .candidate = candidate,
              .candidate_generation = candidate_generation,
              .active_route = active_route_snapshot,
              .activation_raw_world = materialization_snapshot.raw_world,
          });
      materialization.telemetry.route_search_ms = planner_update.search_ms;
      result.geometry_optimization_fallback =
          materialization.geometry_optimization_fallback;

      PreparedRouteActivation3D prepared =
          route_activation_coordinator_.prepare(RouteActivationPreparationRequest3D{
              .transaction = transaction,
              .materialization = std::move(materialization),
              .snapshot = config_.activation_snapshot_provider(),
              .planning_latency = planningLatencyStatistics(),
          });
      RouteActivationCommitResult3D committed =
          config_.activation_commit_boundary(std::move(prepared), commit_operation);
      activation = std::move(committed.result);
      result.candidate_disposition = classifyCandidateDisposition(
          *transaction, activation, config_.extension.required_certified_overlap_m);
      if (result.candidate_disposition !=
          RouteCandidateDisposition3D::kRetrySameCandidate) {
        break;
      }
    }
    // Retries are bounded. A commit base that keeps moving means the lifecycle
    // is racing a faster producer, so the session is retired and replanned
    // rather than retried indefinitely.
    if (result.candidate_disposition ==
        RouteCandidateDisposition3D::kRetrySameCandidate) {
      result.candidate_disposition =
          RouteCandidateDisposition3D::kRetireSearchAndReplan;
    }
  }
  activation.telemetry.route_search_ms = planner_update.search_ms;
  result.activation = std::move(activation);
  result.search_superseded_by_activation =
      result.search_running && result.candidate_disposition ==
                                   RouteCandidateDisposition3D::kRetireSearchAndReplan;

  result.planner_update = std::move(planner_update);
  // A route released as blocked and replaced from the vehicle rather than
  // stitched onto its own certified prefix is a route the vehicle cannot
  // follow at all. Left as the search's incumbent it kept the search
  // improving it while the vehicle had nothing to fly, and the feasibility
  // branch that finds the replacement stayed idle: recorded flights waited
  // one to three planner updates for a successor at every such block, which
  // is most of the time they spent without a route. A stitched replacement
  // keeps the incumbent, because its prefix is exactly what the vehicle is
  // still flying.
  const bool blocked_replacement_from_the_vehicle =
      transaction->replacement() &&
      transaction->release_reason == RouteReleaseReason3D::kBlocked &&
      !transaction->continuity_base.has_value();
  // A candidate the activation could not hand the vehicle over to, or whose
  // connector the raw evidence rejected, is not an incumbent worth improving:
  // the search keeps running, but from the vehicle rather than from it.
  // A refusal beyond the committed route keeps the incumbent only while the
  // vehicle is flying a route that refusal lies ahead of. A vehicle holding
  // no route at all has nothing to enter: the refused candidate was its only
  // offer, and keeping it as the incumbent left the feasibility search idle
  // -- the planner improves an incumbent instead of searching from the
  // vehicle -- while the compile refused the same route on every update
  // until the world happened to change (over a second per hold in recorded
  // flights).
  const bool refused_beyond_a_route_being_flown =
      route_lifecycle_refusal_3d::refusedBeyondTheCommittedRoute3D(
          result.activation, config_.extension.required_certified_overlap_m) &&
      !consumerHoldsNoRoute();
  result.incumbent_rejected =
      result.candidate_disposition ==
          RouteCandidateDisposition3D::kContinueForImprovement &&
      !refused_beyond_a_route_being_flown &&
      (blocked_replacement_from_the_vehicle ||
       result.activation.admission.activation_status ==
           StaticRouteActivationStatus::kDynamicHandoffRejected ||
       result.activation.admission.activation_status ==
           StaticRouteActivationStatus::kRouteCertificationRejected ||
       result.activation.admission.activation_status ==
           StaticRouteActivationStatus::kCandidateValidationRejected ||
       result.activation.admission.activation_status ==
           StaticRouteActivationStatus::kCandidateNotExecutable ||
       result.activation.admission.activation_status ==
           StaticRouteActivationStatus::kInvalidExecutionGeometry);
  if (result.search_running && !result.search_superseded_by_activation) {
    result.search_retired = searchRetired(*transaction);
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
  result.route_search_latency_ms = search_latency_ms;
  // The search ends here unless a continuation carries it on, so this is the
  // full lead time the request took.
  result.search_latency_complete = !result.search_running && search_latency_measured;
  {
    const std::scoped_lock lock{lifecycle_mutex_};
    latest_route_search_ms_ = result.planner_update.search_ms;
    planner_update_latency_tracker_.record(result.planner_update.search_ms,
                                           world_telemetry.build_ms);
    if (result.search_latency_complete) {
      planning_latency_tracker_.record(search_latency_ms, world_telemetry.build_ms);
    }

    if (lifecycle_search) {
      if (result.activation.admission.certified_pending) {
        failed_search_latch_.clear();
      } else if (latch_failed_search) {
        const std::uint64_t failed_generation =
            transaction->replacement() ? transaction->request.base_route_generation
                                       : 0U;
        const PlannerInputStatus3D planner_input =
            result.planner_update.planner_input_status;
        failed_search_latch_.recordFailure(StaticRouteSearchContext{
            .base_route_generation = failed_generation,
            .search_start = result.search_start,
            .objective = transaction->objective,
            .minimum_tracking_sample_sequence =
                result.activation.admission.required_objective_sample,
            .stamp_ns = failed_search_stamp_ns,
            .raw_revision = transaction->planner_world != nullptr
                                ? transaction->planner_world->revision
                                : 0U,
            .input_rejected =
                result.planner_update.planner_invoked &&
                (planner_input == PlannerInputStatus3D::kStartUnavailable ||
                 planner_input == PlannerInputStatus3D::kGoalUnavailable),
            .world_refused_candidate =
                route_lifecycle_refusal_3d::worldRefusedCandidate3D(
                    result.activation.admission.activation_status),
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

StaticRoutePlanningLatencyStats
RouteLifecycleCoordinator3D::planningLatencyStatistics() const noexcept {
  const std::scoped_lock lock{lifecycle_mutex_};
  return planning_latency_tracker_.stats();
}

StaticRoutePlanningLatencyStats
RouteLifecycleCoordinator3D::plannerUpdateLatencyStatistics() const noexcept {
  const std::scoped_lock lock{lifecycle_mutex_};
  return planner_update_latency_tracker_.stats();
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

bool RouteLifecycleCoordinator3D::searchRetired(
    const PlannerSearchTransaction3D& transaction) const noexcept {
  if (transaction.request.kind == StaticRouteSearchRequestKind::kInitial) {
    // The search an observed world raises for a vehicle without a route holds
    // no replan gate, so the gate cannot retire it; the route that ends it is
    // the one that becomes resident, however it was found. Until then its
    // continuation carries the search from one update straight into the next.
    // Judged by the gate it never held, it ended after every update and waited
    // for the next observed world to be raised again, so the planner searched
    // for a fraction of each second while the vehicle held at the start.
    const std::shared_ptr<const ExecutionPlan3D> resident_execution =
        execution_supervisor_.plan();
    return resident_execution != nullptr &&
           resident_execution->routeGenerationHighWater() != 0U;
  }
  const std::scoped_lock lock{lifecycle_mutex_};
  if (transaction.extension()) {
    return !extension_request_in_flight_ ||
           extension_in_flight_generation_ != transaction.request.base_route_generation;
  }
  return !replan_gate_.inFlight() ||
         replan_gate_.generation() != transaction.request.base_route_generation;
}

bool RouteLifecycleCoordinator3D::queueContinuation(
    const RouteLifecycleUpdate3D& update) {
  if (update.request.transaction == nullptr ||
      update.planner_update.planner_session == nullptr) {
    return false;
  }
  // A release that needed a fresh search already retired this search and
  // queued its replacement. Requeueing the retired search would displace that
  // replacement, and the release that raised it does not raise it again.
  if (searchRetired(*update.request.transaction)) {
    return false;
  }
  RoutePlanningRequest3D request{
      .transaction = update.request.transaction,
      .world_telemetry = update.request.world_telemetry,
      .continuation_session = update.planner_update.planner_session,
  };
  refreshContinuationWorld(request, update.planner_update.planner_telemetry);
  refreshContinuationStart(request);
  if (update.incumbent_rejected && request.continuation_session != nullptr) {
    auto rejected_session =
        std::make_shared<RoutePlannerSession3D>(*request.continuation_session);
    rejected_session->request.incumbent_rejection_sequence =
        ++incumbent_rejection_sequence_;
    request.continuation_session = std::move(rejected_session);
  }
  // A consumer without a route of this search — its execution revoked after a
  // stop, its pending route retired — needs the incumbent again, not its next
  // improvement. The continuation opens a new consumer session for it; the
  // request it would otherwise have raised was displaced by this very
  // continuation, or held behind its gate, and a search that answered only
  // with improvements left the vehicle holding until one came.
  request.renew_consumer_session = consumerHoldsNoRoute();
  // A running search carries the incumbent search state; dropping its
  // continuation behind a request that happened to be queued first would end
  // the search silently, and a displaced request loses nothing: the trigger
  // that raised it (a blocked route, a superseded objective, a vehicle without
  // a route) raises it again once the queue is free.
  return enqueue(std::move(request), RoutePlanningQueuePolicy3D::kReplacePending)
      .queued();
}

void RouteLifecycleCoordinator3D::refreshContinuationWorld(
    RoutePlanningRequest3D& request, const PlannerTelemetry3D& telemetry) const {
  // A persistent search continues against the newest raw evidence rather than
  // the world captured when it began. D* Lite repairs the exact occupied
  // difference incrementally, so the in-flight search itself absorbs a blocked
  // route instead of a deferred replan waiting for it to converge on a stale
  // world. A world is absorbed once the previous change has been repaired, or
  // after a bounded number of deferrals, so the search spends its budget on
  // repair instead of on re-scheduling every revision.
  if (!config_.replan_snapshot_provider || request.transaction == nullptr ||
      request.continuation_session == nullptr) {
    return;
  }
  // Every resident world is absorbed as it arrives: a skipped revision makes
  // the next world's dirty-chunk delta incomplete and forces the planner into
  // a full-grid occupied difference, which costs more than absorbing the
  // change it skipped.
  constexpr std::uint32_t kMaximumDeferredWorldRefreshes{0U};
  if (telemetry.repair_lattice_states_pending > 0U &&
      request.continuation_session->deferred_world_refreshes <
          kMaximumDeferredWorldRefreshes) {
    auto deferred_session =
        std::make_shared<RoutePlannerSession3D>(*request.continuation_session);
    ++deferred_session->deferred_world_refreshes;
    request.continuation_session = std::move(deferred_session);
    return;
  }
  const RouteLifecycleReplanSnapshot3D snapshot = config_.replan_snapshot_provider();
  const PlannerSearchTransaction3D& transaction = *request.transaction;
  const RoutePlannerSession3D& session = *request.continuation_session;
  if (snapshot.resident_world == nullptr ||
      !productionWorldGenerationCoherent(*snapshot.resident_world) ||
      transaction.world == nullptr ||
      snapshot.resident_world->producer_instance_id !=
          transaction.world->producer_instance_id) {
    return;
  }
  // Candidate worlds, newest first: the raw search overlay over the latest
  // raw world, then the resident planner world. Each must pair validly with
  // the resident world snapshot; the first valid, strictly newer candidate is
  // absorbed.
  // The resident planner world shares its chunk storage with the previous
  // one, so the planner's occupied difference against it costs only the
  // chunks that changed. A raw overlay is a fresh copy whose every chunk has
  // to be compared, so it is absorbed only when it carries blocking evidence
  // the resident world does not have yet.
  std::vector<std::shared_ptr<const PersistentPlannerWorld3D>> candidates;
  if (snapshot.resident_planner_world != nullptr) {
    candidates.push_back(snapshot.resident_planner_world);
  }
  const bool overlay_required =
      config_.observed_world && snapshot.latest_raw_world != nullptr &&
      snapshot.blocked_raw_revision > snapshot.resident_world->source_raw_revision;
  if (overlay_required) {
    std::shared_ptr<const PersistentPlannerWorld3D> raw_overlay =
        captureObservedRouteSearchWorld3D(
            *snapshot.latest_raw_world,
            snapshot.resident_world->proprioceptive_free_space_seed,
            snapshot.resident_world->launch_support_contact);
    if (raw_overlay != nullptr) {
      candidates.insert(candidates.begin(), std::move(raw_overlay));
    }
  }
  for (const std::shared_ptr<const PersistentPlannerWorld3D>& newest : candidates) {
    if (!newest->valid() ||
        newest->producer_instance_id != session.request.world.producer_instance_id ||
        newest->revision <= session.request.world.revision) {
      continue;
    }
    auto refreshed_transaction =
        std::make_shared<PlannerSearchTransaction3D>(transaction);
    refreshed_transaction->world = snapshot.resident_world;
    refreshed_transaction->planner_world = newest;
    if (!refreshed_transaction->valid()) {
      continue;
    }
    auto refreshed_session = std::make_shared<RoutePlannerSession3D>(session);
    refreshed_session->request.world = *newest;
    refreshed_session->deferred_world_refreshes = 0U;
    request.transaction = std::move(refreshed_transaction);
    request.continuation_session = std::move(refreshed_session);
    request.world_telemetry = snapshot.world_telemetry;
    return;
  }
  // Nothing newer could be absorbed this time; the next continuation tries
  // again without accumulating deferrals.
  auto retry_session = std::make_shared<RoutePlannerSession3D>(session);
  retry_session->deferred_world_refreshes = 0U;
  request.continuation_session = std::move(retry_session);
}

void RouteLifecycleCoordinator3D::refreshContinuationStart(
    RoutePlanningRequest3D& request) const {
  // A search anchored at the vehicle follows the vehicle: D* Lite rebases its
  // start incrementally, and a route planned from where the vehicle was a
  // second ago cannot be handed off. A certified stitch keeps its exact base.
  if (!config_.vehicle_state_provider || request.continuation_session == nullptr ||
      request.continuation_session->search_base_stitch_station_m.has_value()) {
    return;
  }
  const RoutePlannerVehicleState3D vehicle_state = config_.vehicle_state_provider();
  if (!vehicle_state.valid) {
    return;
  }
  const RoutePlannerSession3D& session = *request.continuation_session;
  if (distance3D(session.search_start, vehicle_state.position) <= 1.0e-6) {
    return;
  }
  auto refreshed_session = std::make_shared<RoutePlannerSession3D>(session);
  refreshed_session->request.start = vehicle_state.position;
  refreshed_session->request.velocity = vehicle_state.velocity;
  refreshed_session->search_start = vehicle_state.position;
  refreshed_session->search_velocity = vehicle_state.velocity;
  request.continuation_session = std::move(refreshed_session);
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

bool RouteLifecycleCoordinator3D::consumerHoldsNoRoute() const noexcept {
  const RouteExecutionManagerSnapshot3D execution = execution_supervisor_.snapshot();
  const std::shared_ptr<const ExecutionPlan3D> plan = execution.plan();
  return (plan == nullptr || plan->route() == nullptr) && execution.pending == nullptr;
}

void RouteLifecycleCoordinator3D::observeRecoveryEpisode(
    const std::uint64_t mission_epoch) noexcept {
  static_cast<void>(recovery_episodes_.observe(mission_epoch, consumerHoldsNoRoute()));
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
