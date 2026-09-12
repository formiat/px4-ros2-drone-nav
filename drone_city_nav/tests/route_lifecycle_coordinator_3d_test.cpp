#include "route_lifecycle_coordinator_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(RouteLifecycleCoordinator3DTest,
     DirectAdvanceOwnsMaterializationAdmissionAndAtomicPendingPublication) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  std::size_t commit_count{0U};
  RouteLifecycleCoordinator3D coordinator{
      supervisor, lifecycleConfig(input, supervisor, commit_count)};

  RoutePlanner3D planner{plannerConfig()};
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  RoutePlannerUpdate3D planner_update =
      planner.update(*input.transaction, vehicle_state);
  ASSERT_TRUE(planner_update.improved_incumbent.has_value());

  const RouteLifecycleUpdate3D result = coordinator.advance(RoutePlanningUpdateEvent3D{
      .request =
          RoutePlanningRequest3D{
              .transaction = input.transaction,
              .world_telemetry = {.build_ms = 12.5},
              .continuation_session = nullptr,
          },
      .vehicle_state = vehicle_state,
      .update = std::move(planner_update),
  });

  EXPECT_EQ(result.status, RouteLifecycleAdvanceStatus3D::kCompleted);
  EXPECT_TRUE(result.candidate.available);
  EXPECT_FALSE(result.search_superseded_by_activation);
  EXPECT_GE(result.candidate.point_count, 2U);
  EXPECT_EQ(commit_count, 1U);
  EXPECT_EQ(result.activation.materialized.candidate_generation, 1U);
  EXPECT_TRUE(result.activation.admission.trajectory_compile_attempted);
  EXPECT_TRUE(result.activation.admission.certified_pending)
      << "activation="
      << staticRouteActivationStatusName(result.activation.admission.activation_status)
      << " candidate="
      << staticRouteCandidateStatusName(
             result.activation.admission.candidate_validation.status)
      << " handoff="
      << dynamicHandoffStatus3DName(result.activation.admission.handoff.status)
      << " trajectory="
      << compiledTrajectoryFailureReason3DName(
             result.activation.admission.trajectory_validation.reason)
      << " publication="
      << routePublicationStatus3DName(
             result.activation.admission.assessment.publication.status)
      << " proposal_eligible="
      << result.activation.proposal.identity.activation_eligible
      << " assessment=" << result.activation.admission.assessment.accepted()
      << " projection=" << result.activation.admission.assessment.projection.valid
      << " cross_track=" << result.activation.admission.assessment.cross_track_accepted
      << " raw_world=" << result.activation.admission.assessment.raw_world_compatible
      << " raw_suffix="
      << result.activation.admission.assessment.raw_validation.accepted()
      << " decorations=" << result.activation.admission.decoration_validation.valid()
      << " report_world=" << result.activation.admission.world_compatible
      << " report_objective=" << result.activation.admission.objective_matches
      << " ready="
      << result.activation.admission.readyForArbitration(result.activation.proposal)
      << " route="
      << (result.activation.proposal.trajectory != nullptr &&
          result.activation.proposal.trajectory->route != nullptr)
      << " spans="
      << (result.activation.proposal.trajectory != nullptr &&
          result.activation.proposal.trajectory->constrained_spans != nullptr)
      << " fingerprint="
      << (result.activation.proposal.trajectory != nullptr &&
          result.activation.proposal.trajectory->materialized_route_fingerprint ==
              result.activation.proposal.identity.route_fingerprint)
      << " revision="
      << (result.activation.proposal.trajectory != nullptr
              ? result.activation.proposal.trajectory->compiled_trajectory_revision
              : 0U);
  ASSERT_NE(supervisor.pending(), nullptr);
  EXPECT_EQ(supervisor.pending()->route.identity.generation, 1U);
  EXPECT_GT(result.route_planning_ms, 0.0);
  // The tracked latency is the lead time of the whole request — queue, search
  // and activation — not the slice this update spent applying a result that
  // had already come back.
  EXPECT_TRUE(result.search_latency_complete);
  EXPECT_DOUBLE_EQ(result.route_search_latency_ms, kFixtureSearchLatencyMs);
  const StaticRoutePlanningLatencyStats latency =
      coordinator.planningLatencyStatistics();
  EXPECT_EQ(latency.sample_count, 1U);
  EXPECT_DOUBLE_EQ(latency.planning_p95_ms, kFixtureSearchLatencyMs);
  EXPECT_DOUBLE_EQ(latency.planning_p99_ms, kFixtureSearchLatencyMs);
  EXPECT_DOUBLE_EQ(latency.build_and_planning_p99_ms, kFixtureSearchLatencyMs + 12.5);
  // The update tracker answers a different question — how long one planner
  // update's search step took — and is measured from the planner's own report,
  // not from the request stamp.
  const StaticRoutePlanningLatencyStats update_latency =
      coordinator.plannerUpdateLatencyStatistics();
  EXPECT_EQ(update_latency.sample_count, 1U);
  EXPECT_DOUBLE_EQ(update_latency.planning_p95_ms, result.planner_update.search_ms);
}

TEST(RouteLifecycleCoordinator3DTest,
     NewerActivationWorldRawCollisionReleasesGateAndReplaysDeferredReplan) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  std::size_t initial_commit_count{0U};
  {
    RouteLifecycleCoordinator3D activation_coordinator{
        supervisor, lifecycleConfig(input, supervisor, initial_commit_count)};
    planAndActivateInitialRoute(activation_coordinator, supervisor, input);
  }
  ASSERT_NE(supervisor.plan(), nullptr);
  const std::uint64_t generation = supervisor.plan()->routeGenerationHighWater();
  const std::shared_ptr<const PlannerSearchTransaction3D> replacement =
      makePlannerSearchTransaction3D(input.world, input.planner_world,
                                     input.transaction->objective,
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kReplan,
                                         .base_route_generation = generation,
                                     },
                                     std::nullopt, RouteReleaseReason3D::kBlocked);
  ASSERT_NE(replacement, nullptr);

  std::size_t collision_commit_count{0U};
  std::vector<RouteLifecycleReplanOutcome3D> replan_outcomes;
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, collision_commit_count);
  config.tracking_world_refresh_margin_m = 3.0;
  bindResidentReplanSnapshot(config, input, supervisor, 600);
  config.replan_outcome_handler =
      [&replan_outcomes](const RouteLifecycleReplanOutcome3D& outcome) {
        replan_outcomes.push_back(outcome);
      };
  config.activation_commit_boundary =
      [&collision_commit_count, &replacement](PreparedRouteActivation3D prepared,
                                              const RouteActivationCommitOperation3D&) {
        ++collision_commit_count;
        ProductionRouteActivationResult3D activation = std::move(prepared.result);
        activation.admission.activation_status =
            StaticRouteActivationStatus::kCandidateValidationRejected;
        activation.admission.candidate_validation = StaticRouteCandidateValidation{
            .status = StaticRouteCandidateStatus::kRawCollision};
        activation.admission.snapshot_raw_revision =
            replacement->planner_world->revision + 1U;
        activation.admission.tracking_profile_activation_occupied_fingerprint =
            replacement->planner_world->occupied_fingerprint + 1U;
        activation.admission.certified_pending = false;
        return RouteActivationCommitResult3D{.result = std::move(activation)};
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};

  const RouteLifecycleTrackingRefreshOutcome3D refresh =
      coordinator.requestTrackingWorldRefresh(RouteLifecycleTrackingRefreshRequest3D{
          .world = input.world,
          .navigation = input.navigation,
          .objective = input.objective,
          .stamp_ns = 600,
      });
  ASSERT_EQ(refresh.status, RouteLifecycleTrackingRefreshStatus3D::kQueued);
  const RouteLifecycleReplanOutcome3D deferred =
      coordinator.requestReplan(RouteReleaseReason3D::kBlocked, generation);
  ASSERT_EQ(deferred.status, RouteLifecycleReplanStatus3D::kDeferredReplanInFlight);
  ASSERT_EQ(replan_outcomes.size(), 1U);

  RoutePlanner3D planner{plannerConfig()};
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  RoutePlannerUpdate3D planner_update = planner.update(*replacement, vehicle_state);
  ASSERT_TRUE(planner_update.improved_incumbent.has_value());
  ASSERT_NE(planner_update.planner_session, nullptr);
  planner_update.planner_invoked = true;
  planner_update.planner_progress = SearchProgress3D::kRunning;
  planner_update.dispatch.continue_search = true;

  const RouteLifecycleUpdate3D result = coordinator.advance(RoutePlanningUpdateEvent3D{
      .request =
          RoutePlanningRequest3D{
              .transaction = replacement,
              .continuation_session = nullptr,
          },
      .vehicle_state = vehicle_state,
      .update = std::move(planner_update),
  });

  EXPECT_EQ(result.status, RouteLifecycleAdvanceStatus3D::kCompleted);
  EXPECT_TRUE(result.search_running);
  EXPECT_TRUE(result.search_superseded_by_activation);
  EXPECT_FALSE(result.continuation_queued);
  EXPECT_EQ(collision_commit_count, 1U);
  EXPECT_EQ(result.activation.admission.candidate_validation.status,
            StaticRouteCandidateStatus::kRawCollision);
  EXPECT_NE(result.activation.admission.snapshot_raw_revision,
            result.planner_update.planner_telemetry.planned_on_revision);
  EXPECT_NE(
      result.activation.admission.tracking_profile_activation_occupied_fingerprint,
      replacement->planner_world->occupied_fingerprint);
  ASSERT_EQ(replan_outcomes.size(), 2U);
  const RouteLifecycleReplanOutcome3D& replay = replan_outcomes.back();
  EXPECT_EQ(replay.origin, RouteLifecycleReplanOrigin3D::kDeferredReplanReplay);
  EXPECT_EQ(replay.replay_completed_generation, generation);
  EXPECT_EQ(replay.status, RouteLifecycleReplanStatus3D::kRouteQueueBusy);
}

TEST(RouteLifecycleCoordinator3DTest,
     ADeferredBlockedReplacementDoesNotLatchTheSearchAsFailed) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  std::size_t initial_commit_count{0U};
  {
    RouteLifecycleCoordinator3D activation_coordinator{
        supervisor, lifecycleConfig(input, supervisor, initial_commit_count)};
    planAndActivateInitialRoute(activation_coordinator, supervisor, input);
  }
  ASSERT_NE(supervisor.plan(), nullptr);
  const std::uint64_t generation = supervisor.plan()->routeGenerationHighWater();
  const std::shared_ptr<const PlannerSearchTransaction3D> replacement =
      makePlannerSearchTransaction3D(input.world, input.planner_world,
                                     input.transaction->objective,
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kReplan,
                                         .base_route_generation = generation,
                                     },
                                     std::nullopt, RouteReleaseReason3D::kBlocked);
  ASSERT_NE(replacement, nullptr);

  std::size_t deferral_commit_count{0U};
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, deferral_commit_count);
  bindResidentReplanSnapshot(config, input, supervisor, 600);
  config.activation_commit_boundary =
      [&deferral_commit_count](PreparedRouteActivation3D prepared,
                               const RouteActivationCommitOperation3D&) {
        ++deferral_commit_count;
        ProductionRouteActivationResult3D activation = std::move(prepared.result);
        // The rule held the delivered candidate for its grace.
        activation.admission.activation_status =
            StaticRouteActivationStatus::kReplacementAwaitingSearch;
        activation.admission.blocked_replacement_assessed = true;
        activation.admission.blocked_replacement_deferred = true;
        activation.admission.certified_pending = false;
        return RouteActivationCommitResult3D{.result = std::move(activation)};
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};

  RoutePlanner3D planner{plannerConfig()};
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  RoutePlannerUpdate3D planner_update = planner.update(*replacement, vehicle_state);
  ASSERT_TRUE(planner_update.improved_incumbent.has_value());
  planner_update.planner_invoked = true;
  planner_update.planner_progress = SearchProgress3D::kConverged;
  planner_update.dispatch.continue_search = false;
  const RouteLifecycleUpdate3D result = coordinator.advance(RoutePlanningUpdateEvent3D{
      .request =
          RoutePlanningRequest3D{
              .transaction = replacement,
              .continuation_session = nullptr,
          },
      .vehicle_state = vehicle_state,
      .update = std::move(planner_update),
  });
  ASSERT_EQ(result.status, RouteLifecycleAdvanceStatus3D::kCompleted);
  ASSERT_TRUE(result.activation.admission.blocked_replacement_deferred);
  EXPECT_FALSE(result.failed_search_latched)
      << "the lifecycle's own deferral was recorded as a failed search";

  // The route the vehicle was flying is then blocked and released: the search
  // it needs now must not be suppressed by that deferral.
  const RouteLifecycleReplanOutcome3D outcome =
      coordinator.requestReplan(RouteReleaseReason3D::kBlocked, generation);
  EXPECT_NE(outcome.status, RouteLifecycleReplanStatus3D::kSuppressedFailedSearch);
}

TEST(RouteLifecycleCoordinator3DTest,
     SameWorldRawCollisionDoesNotSupersedeRunningSearchAndTheInitialSearchContinues) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  std::size_t commit_count{0U};
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, commit_count);
  config.activation_commit_boundary =
      [&commit_count, &input](PreparedRouteActivation3D prepared,
                              const RouteActivationCommitOperation3D&) {
        ++commit_count;
        ProductionRouteActivationResult3D activation = std::move(prepared.result);
        activation.admission.activation_status =
            StaticRouteActivationStatus::kCandidateValidationRejected;
        activation.admission.candidate_validation = StaticRouteCandidateValidation{
            .status = StaticRouteCandidateStatus::kRawCollision};
        activation.admission.snapshot_raw_revision =
            input.transaction->planner_world->revision;
        activation.admission.tracking_profile_activation_occupied_fingerprint =
            input.transaction->planner_world->occupied_fingerprint;
        activation.admission.certified_pending = false;
        return RouteActivationCommitResult3D{.result = std::move(activation)};
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
  coordinator.start();

  RoutePlanner3D planner{plannerConfig()};
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  RoutePlannerUpdate3D planner_update =
      planner.update(*input.transaction, vehicle_state);
  ASSERT_TRUE(planner_update.improved_incumbent.has_value());
  ASSERT_NE(planner_update.planner_session, nullptr);
  planner_update.planner_invoked = true;
  planner_update.planner_progress = SearchProgress3D::kRunning;
  planner_update.dispatch.continue_search = true;

  const RouteLifecycleUpdate3D result = coordinator.advance(RoutePlanningUpdateEvent3D{
      .request =
          RoutePlanningRequest3D{
              .transaction = input.transaction,
              .continuation_session = nullptr,
          },
      .vehicle_state = vehicle_state,
      .update = std::move(planner_update),
  });
  coordinator.stop();

  EXPECT_TRUE(result.search_running);
  EXPECT_FALSE(result.search_superseded_by_activation);
  // No route is resident, so the initial search is not retired: its
  // continuation is queued rather than left to the next observed world.
  EXPECT_FALSE(result.search_retired);
  EXPECT_TRUE(result.continuation_queued);
  EXPECT_GE(commit_count, 1U);
  EXPECT_EQ(result.activation.admission.snapshot_raw_revision,
            result.planner_update.planner_telemetry.planned_on_revision);
}

TEST(RouteLifecycleCoordinator3DTest,
     AnInitialSearchWithoutACandidateQueuesItsContinuationUntilARouteIsResident) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  ASSERT_EQ(input.transaction->request.kind, StaticRouteSearchRequestKind::kInitial);
  std::size_t commit_count{0U};
  {
    RouteLifecycleCoordinator3D coordinator{
        supervisor, lifecycleConfig(input, supervisor, commit_count)};
    coordinator.start();
    RoutePlanner3D planner{plannerConfig()};
    const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
    RoutePlannerUpdate3D planner_update =
        planner.update(*input.transaction, vehicle_state);
    ASSERT_NE(planner_update.planner_session, nullptr);
    // The update found nothing yet and the search is still running.
    planner_update.improved_incumbent.reset();
    planner_update.planner_invoked = true;
    planner_update.planner_progress = SearchProgress3D::kRunning;
    planner_update.dispatch.continue_search = true;

    const RouteLifecycleUpdate3D result =
        coordinator.advance(RoutePlanningUpdateEvent3D{
            .request =
                RoutePlanningRequest3D{
                    .transaction = input.transaction,
                    .continuation_session = nullptr,
                },
            .vehicle_state = vehicle_state,
            .update = std::move(planner_update),
        });
    coordinator.stop();

    EXPECT_EQ(result.status, RouteLifecycleAdvanceStatus3D::kContinuationQueued);
    EXPECT_TRUE(result.search_running);
    EXPECT_FALSE(result.search_retired);
    EXPECT_TRUE(result.continuation_queued);
  }

  // Once a route is resident the same search is retired: the route ended it.
  {
    RouteLifecycleCoordinator3D activation_coordinator{
        supervisor, lifecycleConfig(input, supervisor, commit_count)};
    planAndActivateInitialRoute(activation_coordinator, supervisor, input);
  }
  ASSERT_NE(supervisor.plan(), nullptr);
  ASSERT_NE(supervisor.plan()->routeGenerationHighWater(), 0U);
  RouteLifecycleCoordinator3D coordinator{
      supervisor, lifecycleConfig(input, supervisor, commit_count)};
  RoutePlanner3D planner{plannerConfig()};
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  RoutePlannerUpdate3D planner_update =
      planner.update(*input.transaction, vehicle_state);
  ASSERT_NE(planner_update.planner_session, nullptr);
  planner_update.improved_incumbent.reset();
  planner_update.planner_invoked = true;
  planner_update.planner_progress = SearchProgress3D::kRunning;
  planner_update.dispatch.continue_search = true;

  const RouteLifecycleUpdate3D retired = coordinator.advance(RoutePlanningUpdateEvent3D{
      .request =
          RoutePlanningRequest3D{
              .transaction = input.transaction,
              .continuation_session = nullptr,
          },
      .vehicle_state = vehicle_state,
      .update = std::move(planner_update),
  });

  EXPECT_EQ(retired.status, RouteLifecycleAdvanceStatus3D::kContinuationQueued);
  EXPECT_TRUE(retired.search_retired);
  EXPECT_FALSE(retired.continuation_queued);
  EXPECT_FALSE(coordinator.pending());
}

TEST(RouteLifecycleCoordinator3DTest,
     SupersededActivationSnapshotReleasesGateAndReplaysDeferredReplan) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  std::size_t initial_commit_count{0U};
  {
    RouteLifecycleCoordinator3D activation_coordinator{
        supervisor, lifecycleConfig(input, supervisor, initial_commit_count)};
    planAndActivateInitialRoute(activation_coordinator, supervisor, input);
  }
  ASSERT_NE(supervisor.plan(), nullptr);
  const std::uint64_t generation = supervisor.plan()->routeGenerationHighWater();
  const std::shared_ptr<const PlannerSearchTransaction3D> replacement =
      makePlannerSearchTransaction3D(input.world, input.planner_world,
                                     input.transaction->objective,
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kReplan,
                                         .base_route_generation = generation,
                                     },
                                     std::nullopt, RouteReleaseReason3D::kBlocked);
  ASSERT_NE(replacement, nullptr);

  std::size_t superseded_commit_count{0U};
  std::vector<RouteLifecycleReplanOutcome3D> replan_outcomes;
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, superseded_commit_count);
  config.tracking_world_refresh_margin_m = 3.0;
  bindResidentReplanSnapshot(config, input, supervisor, 600);
  config.replan_outcome_handler =
      [&replan_outcomes](const RouteLifecycleReplanOutcome3D& outcome) {
        replan_outcomes.push_back(outcome);
      };
  config.activation_commit_boundary =
      [&superseded_commit_count](PreparedRouteActivation3D prepared,
                                 const RouteActivationCommitOperation3D&) {
        ++superseded_commit_count;
        ProductionRouteActivationResult3D activation = std::move(prepared.result);
        activation.admission.activation_status =
            StaticRouteActivationStatus::kActivationSnapshotSuperseded;
        activation.admission.snapshot_current = false;
        activation.admission.resident_world_snapshot_current = false;
        activation.admission.certified_pending = false;
        return RouteActivationCommitResult3D{.result = std::move(activation)};
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};

  const RouteLifecycleTrackingRefreshOutcome3D refresh =
      coordinator.requestTrackingWorldRefresh(RouteLifecycleTrackingRefreshRequest3D{
          .world = input.world,
          .navigation = input.navigation,
          .objective = input.objective,
          .stamp_ns = 600,
      });
  ASSERT_EQ(refresh.status, RouteLifecycleTrackingRefreshStatus3D::kQueued);
  const RouteLifecycleReplanOutcome3D deferred =
      coordinator.requestReplan(RouteReleaseReason3D::kBlocked, generation);
  ASSERT_EQ(deferred.status, RouteLifecycleReplanStatus3D::kDeferredReplanInFlight);
  ASSERT_EQ(replan_outcomes.size(), 1U);

  RoutePlanner3D planner{plannerConfig()};
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  RoutePlannerUpdate3D planner_update = planner.update(*replacement, vehicle_state);
  ASSERT_TRUE(planner_update.improved_incumbent.has_value());
  ASSERT_NE(planner_update.planner_session, nullptr);
  planner_update.planner_invoked = true;
  planner_update.planner_progress = SearchProgress3D::kRunning;
  planner_update.dispatch.continue_search = true;

  const RouteLifecycleUpdate3D result = coordinator.advance(RoutePlanningUpdateEvent3D{
      .request =
          RoutePlanningRequest3D{
              .transaction = replacement,
              .continuation_session = nullptr,
          },
      .vehicle_state = vehicle_state,
      .update = std::move(planner_update),
  });

  EXPECT_EQ(result.status, RouteLifecycleAdvanceStatus3D::kCompleted);
  EXPECT_TRUE(result.search_running);
  EXPECT_TRUE(result.search_superseded_by_activation);
  EXPECT_FALSE(result.continuation_queued);
  // The candidate survives one supersession and is committed again; only a
  // second supersession retires the search.
  EXPECT_EQ(superseded_commit_count, 2U);
  EXPECT_EQ(result.activation_attempts, 2U);
  EXPECT_EQ(result.candidate_disposition,
            RouteCandidateDisposition3D::kRetireSearchAndReplan);
  EXPECT_EQ(result.activation.admission.activation_status,
            StaticRouteActivationStatus::kActivationSnapshotSuperseded);
  ASSERT_EQ(replan_outcomes.size(), 2U);
  const RouteLifecycleReplanOutcome3D& replay = replan_outcomes.back();
  EXPECT_EQ(replay.origin, RouteLifecycleReplanOrigin3D::kDeferredReplanReplay);
  EXPECT_EQ(replay.replay_completed_generation, generation);
  EXPECT_EQ(replay.status, RouteLifecycleReplanStatus3D::kRouteQueueBusy);
}

TEST(RouteLifecycleCoordinator3DTest,
     SupersededActivationSnapshotRetriesTheSameCandidateBeforeRetiring) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);

  std::size_t commit_count{0U};
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, commit_count);
  std::vector<std::size_t> committed_route_sample_counts;
  config.activation_commit_boundary =
      [&input, &commit_count,
       &committed_route_sample_counts](PreparedRouteActivation3D prepared,
                                       const RouteActivationCommitOperation3D& commit) {
        ++commit_count;
        committed_route_sample_counts.push_back(
            prepared.result.materialized.route != nullptr
                ? prepared.result.materialized.route->size()
                : 0U);
        if (commit_count == 1U) {
          ProductionRouteActivationResult3D activation = std::move(prepared.result);
          activation.admission.activation_status =
              StaticRouteActivationStatus::kActivationSnapshotSuperseded;
          activation.admission.snapshot_current = false;
          activation.admission.resident_world_snapshot_current = false;
          activation.admission.certified_pending = false;
          return RouteActivationCommitResult3D{.result = std::move(activation)};
        }
        return commit(std::move(prepared),
                      RouteActivationCommitContext3D{
                          .resident_world = input.world,
                          .objective = input.objective,
                          .raw_world = nullptr,
                          .minimum_tracking_route_mission_epoch = 0U,
                          .minimum_tracking_route_sample_sequence = 0U,
                      });
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};

  RoutePlanner3D planner{plannerConfig()};
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  RoutePlannerUpdate3D planner_update =
      planner.update(*input.transaction, vehicle_state);
  ASSERT_TRUE(planner_update.improved_incumbent.has_value());

  const RouteLifecycleUpdate3D result = coordinator.advance(RoutePlanningUpdateEvent3D{
      .request =
          RoutePlanningRequest3D{
              .transaction = input.transaction,
              .continuation_session = nullptr,
          },
      .vehicle_state = vehicle_state,
      .update = std::move(planner_update),
  });

  EXPECT_EQ(result.status, RouteLifecycleAdvanceStatus3D::kCompleted);
  EXPECT_EQ(commit_count, 2U);
  EXPECT_EQ(result.activation_attempts, 2U);
  EXPECT_EQ(result.candidate_disposition, RouteCandidateDisposition3D::kActivated);
  EXPECT_FALSE(result.search_superseded_by_activation);
  ASSERT_EQ(committed_route_sample_counts.size(), 2U);
  // The retry commits the same proven candidate, not a degraded substitute.
  EXPECT_EQ(committed_route_sample_counts.front(),
            committed_route_sample_counts.back());
  EXPECT_GT(committed_route_sample_counts.back(), 1U);
  EXPECT_NE(supervisor.plan(), nullptr);
}

TEST(RouteLifecycleCoordinator3DTest,
     TrackingRefreshGateIsOwnedAndReleasedByTheLifecycleBoundary) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  std::size_t commit_count{0U};
  std::size_t refresh_count{0U};
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, commit_count);
  config.tracking_world_refresh_margin_m = 3.0;
  config.world_refresh_requester =
      [&refresh_count](const std::uint64_t generation,
                       const RouteLifecycleWorldRefreshPurpose3D purpose) {
        EXPECT_EQ(purpose, RouteLifecycleWorldRefreshPurpose3D::kTrackingObjective);
        ++refresh_count;
        return RouteLifecycleWorldRefreshResult3D{
            .sequence = refresh_count,
            .base_route_generation = generation,
        };
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
  planAndActivateInitialRoute(coordinator, supervisor, input);

  const RouteLifecycleTrackingRefreshRequest3D request{
      .world = input.world,
      .navigation = input.navigation,
      .objective = input.objective,
      .stamp_ns = 200,
  };
  const RouteLifecycleTrackingRefreshOutcome3D first =
      coordinator.requestTrackingWorldRefresh(request);
  EXPECT_EQ(first.status, RouteLifecycleTrackingRefreshStatus3D::kQueued);
  EXPECT_EQ(refresh_count, 1U);

  const RouteLifecycleTrackingRefreshOutcome3D coalesced =
      coordinator.requestTrackingWorldRefresh(request);
  EXPECT_EQ(coalesced.status, RouteLifecycleTrackingRefreshStatus3D::kLifecycleBusy);
  EXPECT_EQ(refresh_count, 1U);

  coordinator.finishWorldRefresh(
      first.base_route_generation,
      RouteLifecycleWorldRefreshPurpose3D::kTrackingObjective);
  const RouteLifecycleTrackingRefreshOutcome3D after_completion =
      coordinator.requestTrackingWorldRefresh(request);
  EXPECT_EQ(after_completion.status, RouteLifecycleTrackingRefreshStatus3D::kQueued);
  EXPECT_EQ(refresh_count, 2U);
}

TEST(RouteLifecycleCoordinator3DTest,
     DeferredReplanReplayRetainsItsTypedLifecycleOrigin) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  std::size_t commit_count{0U};
  std::vector<RouteLifecycleReplanOutcome3D> replan_outcomes;
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, commit_count);
  config.tracking_world_refresh_margin_m = 3.0;
  bindResidentReplanSnapshot(config, input, supervisor, 300);
  config.replan_outcome_handler =
      [&replan_outcomes](const RouteLifecycleReplanOutcome3D& outcome) {
        replan_outcomes.push_back(outcome);
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
  planAndActivateInitialRoute(coordinator, supervisor, input);

  const std::uint64_t generation = supervisor.plan()->routeGenerationHighWater();
  const RouteLifecycleTrackingRefreshOutcome3D refresh =
      coordinator.requestTrackingWorldRefresh(RouteLifecycleTrackingRefreshRequest3D{
          .world = input.world,
          .navigation = input.navigation,
          .objective = input.objective,
          .stamp_ns = 300,
      });
  ASSERT_EQ(refresh.status, RouteLifecycleTrackingRefreshStatus3D::kQueued);

  const RouteLifecycleReplanOutcome3D deferred =
      coordinator.requestReplan(RouteReleaseReason3D::kBlocked, generation);
  ASSERT_EQ(deferred.status, RouteLifecycleReplanStatus3D::kDeferredReplanInFlight);
  ASSERT_EQ(replan_outcomes.size(), 1U);
  EXPECT_EQ(replan_outcomes.front().origin, RouteLifecycleReplanOrigin3D::kRequested);

  coordinator.finishWorldRefresh(
      generation, RouteLifecycleWorldRefreshPurpose3D::kTrackingObjective);
  ASSERT_EQ(replan_outcomes.size(), 2U);
  const RouteLifecycleReplanOutcome3D& replay = replan_outcomes.back();
  EXPECT_EQ(replay.origin, RouteLifecycleReplanOrigin3D::kDeferredReplanReplay);
  EXPECT_EQ(replay.replay_completed_generation, generation);
  EXPECT_EQ(replay.requested_route_generation, 0U);
  EXPECT_EQ(replay.status, RouteLifecycleReplanStatus3D::kRouteQueueBusy);
  EXPECT_EQ(replay.enqueue_status, RoutePlanningEnqueueStatus3D::kStopped);
}

TEST(RouteLifecycleCoordinator3DTest,
     FailedTrackingRefreshReplaysARequestDeferredInsideTheRuntimeBoundary) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  std::size_t commit_count{0U};
  std::vector<RouteLifecycleReplanOutcome3D> replan_outcomes;
  std::optional<RouteLifecycleReplanOutcome3D> nested_replan;
  RouteLifecycleCoordinator3D* coordinator_port{nullptr};
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, commit_count);
  config.tracking_world_refresh_margin_m = 3.0;
  bindResidentReplanSnapshot(config, input, supervisor, 400);
  config.world_refresh_requester =
      [&coordinator_port,
       &nested_replan](const std::uint64_t generation,
                       const RouteLifecycleWorldRefreshPurpose3D purpose) {
        EXPECT_EQ(purpose, RouteLifecycleWorldRefreshPurpose3D::kTrackingObjective);
        EXPECT_NE(coordinator_port, nullptr);
        if (coordinator_port == nullptr) {
          return RouteLifecycleWorldRefreshResult3D{};
        }
        nested_replan = coordinator_port->requestReplan(
            RouteReleaseReason3D::kObjectiveChanged, generation);
        return RouteLifecycleWorldRefreshResult3D{};
      };
  config.replan_outcome_handler =
      [&replan_outcomes](const RouteLifecycleReplanOutcome3D& outcome) {
        replan_outcomes.push_back(outcome);
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
  coordinator_port = &coordinator;
  planAndActivateInitialRoute(coordinator, supervisor, input);

  const RouteLifecycleTrackingRefreshOutcome3D refresh =
      coordinator.requestTrackingWorldRefresh(RouteLifecycleTrackingRefreshRequest3D{
          .world = input.world,
          .navigation = input.navigation,
          .objective = input.objective,
          .stamp_ns = 400,
      });

  ASSERT_TRUE(nested_replan.has_value());
  EXPECT_EQ(nested_replan.value_or(RouteLifecycleReplanOutcome3D{}).status,
            RouteLifecycleReplanStatus3D::kDeferredReplanInFlight);
  EXPECT_EQ(refresh.status,
            RouteLifecycleTrackingRefreshStatus3D::kWorldRefreshUnavailable);
  ASSERT_EQ(replan_outcomes.size(), 2U);
  EXPECT_EQ(replan_outcomes.front().origin, RouteLifecycleReplanOrigin3D::kRequested);
  const RouteLifecycleReplanOutcome3D& replay = replan_outcomes.back();
  EXPECT_EQ(replay.origin, RouteLifecycleReplanOrigin3D::kDeferredReplanReplay);
  EXPECT_EQ(replay.replay_completed_generation, refresh.base_route_generation);
  EXPECT_EQ(replay.status, RouteLifecycleReplanStatus3D::kRouteQueueBusy);
}

TEST(RouteLifecycleCoordinator3DTest,
     FailedExtensionReservationReplaysARequestDeferredInsideTheRuntimeBoundary) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  std::size_t commit_count{0U};
  {
    RouteLifecycleCoordinator3D activation_coordinator{
        supervisor, lifecycleConfig(input, supervisor, commit_count)};
    planAndActivateInitialRoute(activation_coordinator, supervisor, input);
  }

  const std::shared_ptr<const ExecutionPlan3D> execution = supervisor.plan();
  ASSERT_NE(execution, nullptr);
  ASSERT_NE(execution->route(), nullptr);
  SnapshotFixture3D continuation_fixture;
  continuation_fixture.objective.goal = Point3{20.0, 0.0, 5.0};
  continuation_fixture.proposal.objective = continuation_fixture.objective;
  continuation_fixture.proposal.intent.mission_target =
      continuation_fixture.objective.goal;
  continuation_fixture.proposal.reaches_mission_goal = false;
  continuation_fixture.proposal.evidence.reaches_mission_target = false;
  continuation_fixture.route.back().reference_speed_mps = 4.0;
  continuation_fixture.physical_route_fingerprint =
      routeFingerprint(continuation_fixture.route);
  continuation_fixture.proposal.route_fingerprint =
      continuation_fixture.physical_route_fingerprint;
  continuation_fixture.geometry = makeGeometry(
      continuation_fixture.route, continuation_fixture.physical_route_fingerprint,
      TrackingErrorTubeWorld3D{
          .observed_occupancy = &continuation_fixture.raw_occupancy,
          .occupied_content_fingerprint =
              continuation_fixture.raw_occupancy.occupiedSnapshot()
                  .contentFingerprint(),
      });
  continuation_fixture.decorations = makeDecorations(
      continuation_fixture.geometry, SnapshotFixture3D::kRouteGeneration);
  const std::optional<CertifiedRouteSuffix3D> continuation_route =
      continuation_fixture.certify();
  ASSERT_TRUE(continuation_route.has_value());
  auto active_route = std::make_shared<CertifiedRouteSuffix3D>(
      continuation_route.value_or(CertifiedRouteSuffix3D{}));
  ASSERT_TRUE(active_route->valid());
  auto extension_objective =
      std::make_shared<ProductionNavigationObjective>(*input.objective);
  extension_objective->goal = Point3{100.0, 1.5, 1.5};
  ProductionMppiNavigation moving_navigation = input.navigation;
  moving_navigation.state.vx = 5.0F;

  std::vector<RouteLifecycleReplanOutcome3D> replan_outcomes;
  std::optional<RouteLifecycleReplanOutcome3D> nested_replan;
  RouteLifecycleCoordinator3D* coordinator_port{nullptr};
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, commit_count);
  bindResidentReplanSnapshot(config, input, supervisor, 500);
  config.world_refresh_requester =
      [&coordinator_port,
       &nested_replan](const std::uint64_t generation,
                       const RouteLifecycleWorldRefreshPurpose3D purpose) {
        EXPECT_EQ(purpose, RouteLifecycleWorldRefreshPurpose3D::kRouteExtension);
        EXPECT_NE(coordinator_port, nullptr);
        if (coordinator_port == nullptr) {
          return RouteLifecycleWorldRefreshResult3D{};
        }
        nested_replan = coordinator_port->requestReplan(
            RouteReleaseReason3D::kObjectiveChanged, generation);
        return RouteLifecycleWorldRefreshResult3D{};
      };
  config.replan_outcome_handler =
      [&replan_outcomes](const RouteLifecycleReplanOutcome3D& outcome) {
        replan_outcomes.push_back(outcome);
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
  coordinator_port = &coordinator;

  const double route_end_station_m = active_route->endStationM();
  const RouteLifecycleExtensionOutcome3D extension =
      coordinator.requestExtension(RouteLifecycleExtensionRequest3D{
          .world = input.world,
          .world_telemetry = {.build_ms = 8'000.0},
          .active_route = active_route,
          .navigation = moving_navigation,
          .objective = extension_objective,
          .route_projection =
              RouteProgressProjection3D{
                  .valid = true,
                  .station_m = route_end_station_m,
                  .total_length_m = route_end_station_m,
                  .remaining_m = 0.0,
              },
          .stamp_ns = 500,
          .pending_successor = false,
      });

  EXPECT_TRUE(extension.decision.request_roi_refresh);
  ASSERT_TRUE(nested_replan.has_value());
  EXPECT_EQ(nested_replan.value_or(RouteLifecycleReplanOutcome3D{}).status,
            RouteLifecycleReplanStatus3D::kDeferredDuringExtension);
  EXPECT_EQ(extension.status,
            RouteLifecycleExtensionStatus3D::kWorldRefreshUnavailable);
  ASSERT_EQ(replan_outcomes.size(), 2U);
  EXPECT_EQ(replan_outcomes.front().origin, RouteLifecycleReplanOrigin3D::kRequested);
  const RouteLifecycleReplanOutcome3D& replay = replan_outcomes.back();
  EXPECT_EQ(replay.origin, RouteLifecycleReplanOrigin3D::kDeferredExtensionReplay);
  EXPECT_EQ(replay.requested_route_generation, active_route->identity.generation);
  EXPECT_EQ(replay.replay_completed_generation, active_route->identity.generation);
  EXPECT_EQ(replay.status, RouteLifecycleReplanStatus3D::kRouteQueueBusy);
}

// The retries of one blocked route's replacement share the stitch the first
// was searched from while it stays inside the window the vehicle admits; a
// stitch recomputed on every retry moved one overlap ahead of the vehicle
// while the planner offered the candidate from the previous stitch, and the
// two never met within the stitch tolerance until the vehicle stopped.
TEST(RouteLifecycleCoordinator3DTest, ARetryOfABlockedReplacementKeepsItsStitch) {
  const BlockedReplacementStitch3D none{};
  EXPECT_EQ(blockedReplacementStitchStationM(none, 7U, 12.0, 9.0, 15.0), 12.0);
  const BlockedReplacementStitch3D previous{.station_m = 12.0, .route_generation = 7U};
  // The vehicle moved on: the fresh stitch would be 12.4 m, the previous one
  // is still ahead of the minimum and short of the limit, so it is kept.
  EXPECT_EQ(blockedReplacementStitchStationM(previous, 7U, 12.4, 9.4, 15.0), 12.0);
  // The vehicle's stopping path reached the previous stitch: a fresh one.
  EXPECT_EQ(blockedReplacementStitchStationM(previous, 7U, 15.0, 12.1, 16.0), 15.0);
  // The block moved closer than the previous stitch: a fresh one, or none
  // when even that is past the limit.
  EXPECT_EQ(blockedReplacementStitchStationM(previous, 7U, 11.0, 9.0, 11.5), 11.0);
  EXPECT_EQ(blockedReplacementStitchStationM(previous, 7U, 12.4, 9.4, 11.5),
            std::nullopt);
  // Another route has its own stitch.
  EXPECT_EQ(blockedReplacementStitchStationM(previous, 8U, 12.4, 9.4, 15.0), 12.4);
}

// A route blocked far enough ahead is replaced onto its own certified prefix:
// the replan carries the incumbent as the successor's continuity base with a
// stitch limit one overlap short of the block, and the transaction admits a
// base on a blocked replacement and on nothing else. A block inside the
// overlap's reach leaves nothing worth keeping and the search starts from the
// vehicle.
TEST(RouteLifecycleCoordinator3DTest,
     ABlockedRouteIsReplacedOntoItsCertifiedPrefixWhenTheBlockIsFarAhead) {
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  std::size_t initial_commit_count{0U};
  {
    RouteLifecycleCoordinator3D activation_coordinator{
        supervisor, lifecycleConfig(input, supervisor, initial_commit_count)};
    planAndActivateInitialRoute(activation_coordinator, supervisor, input);
  }
  const std::shared_ptr<const ExecutionPlan3D> plan = supervisor.plan();
  ASSERT_NE(plan, nullptr);
  ASSERT_NE(plan->route(), nullptr);
  const std::uint64_t generation = plan->routeGenerationHighWater();
  const std::shared_ptr<const CertifiedRouteSuffix3D> active_route{plan, plan->route()};
  const double end_station_m = active_route->endStationM();
  ASSERT_GT(end_station_m, 3.0);
  const RouteProgressProjection3D projection{
      .valid = true,
      .station_m = active_route->progress.station_m,
      .total_length_m = end_station_m,
      .remaining_m = end_station_m - active_route->progress.station_m,
      .cross_track_m = 0.0,
  };

  const auto stitched_transaction = [&](const RouteReleaseReason3D reason) {
    return makePlannerSearchTransaction3D(
        input.world, input.planner_world, input.transaction->objective,
        StaticRouteSearchRequestIdentity{
            .kind = StaticRouteSearchRequestKind::kReplan,
            .base_route_generation = generation,
        },
        PlannerSearchContinuityBase3D{
            .route = active_route,
            .request_projection = projection,
            .stitch_limit_station_m = end_station_m - 1.0,
        },
        reason);
  };
  EXPECT_NE(stitched_transaction(RouteReleaseReason3D::kBlocked), nullptr);
  EXPECT_EQ(stitched_transaction(RouteReleaseReason3D::kDiverged), nullptr);

  // The stitch window is the vehicle's own stopping path at each end, so the
  // vehicle has to be moving for one to exist at all.
  ProductionMppiNavigation moving_navigation = input.navigation;
  moving_navigation.state.vx = 2.0F;
  const auto replan = [&](const double blocked_station_m) {
    std::size_t commit_count{0U};
    RouteLifecycleCoordinatorConfig3D config =
        lifecycleConfig(input, supervisor, commit_count);
    config.extension.required_certified_overlap_m = 1.0;
    config.replan_snapshot_provider = [&input, &supervisor, active_route, projection,
                                       blocked_station_m, moving_navigation]() {
      const std::shared_ptr<const ExecutionPlan3D> execution = supervisor.plan();
      return RouteLifecycleReplanSnapshot3D{
          .navigation = moving_navigation,
          .objective = input.objective,
          .resident_world = input.world,
          .resident_planner_world = input.planner_world,
          .latest_raw_world = nullptr,
          .world_telemetry = {},
          .committed_route_generation =
              execution != nullptr ? execution->routeGenerationHighWater() : 0U,
          .blocked_raw_revision = 0U,
          .active_route = active_route,
          .route_projection = projection,
          .blocked_station_m = blocked_station_m,
          .minimum_route_mission_epoch = 0U,
          .minimum_route_sample_sequence = 0U,
          .stamp_ns = 600,
      };
    };
    RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
    return coordinator.requestReplan(RouteReleaseReason3D::kBlocked, generation);
  };

  // The queue may still hold the activation's own search; the base is
  // decided before the request is queued either way.
  const auto reached_the_queue = [](const RouteLifecycleReplanStatus3D status) {
    return status == RouteLifecycleReplanStatus3D::kQueued ||
           status == RouteLifecycleReplanStatus3D::kRouteQueueBusy;
  };
  const RouteLifecycleReplanOutcome3D far = replan(end_station_m);
  EXPECT_TRUE(reached_the_queue(far.status))
      << routeLifecycleReplanStatus3DName(far.status);
  ASSERT_TRUE(far.stitch_limit_station_m.has_value());
  // One stopping path short of the block, not one certified overlap.
  EXPECT_LT(*far.stitch_limit_station_m, end_station_m);
  EXPECT_GT(*far.stitch_limit_station_m, active_route->progress.station_m);

  const RouteLifecycleReplanOutcome3D near =
      replan(active_route->progress.station_m + 0.2);
  EXPECT_TRUE(reached_the_queue(near.status))
      << routeLifecycleReplanStatus3DName(near.status);
  EXPECT_FALSE(near.stitch_limit_station_m.has_value());
}

} // namespace
} // namespace drone_city_nav
