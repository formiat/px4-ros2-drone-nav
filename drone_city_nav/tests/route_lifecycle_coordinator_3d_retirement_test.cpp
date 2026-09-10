#include "route_lifecycle_coordinator_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(RouteLifecycleCoordinator3DTest,
     AStaleRouteGenerationRetiresAReplacementSearchInsteadOfContinuingIt) {
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
                                     std::nullopt, RouteReleaseReason3D::kDiverged);
  ASSERT_NE(replacement, nullptr);

  std::size_t stale_commit_count{0U};
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, stale_commit_count);
  bindResidentReplanSnapshot(config, input, supervisor, 600);
  config.activation_commit_boundary =
      [&stale_commit_count](PreparedRouteActivation3D prepared,
                            const RouteActivationCommitOperation3D&) {
        ++stale_commit_count;
        ProductionRouteActivationResult3D activation = std::move(prepared.result);
        // The commit base moved past the search's base between its start and
        // this delivery.
        activation.admission.activation_status =
            StaticRouteActivationStatus::kStaleRouteGeneration;
        activation.admission.generation_assessed = true;
        activation.admission.generation_matches = false;
        activation.admission.certified_pending = false;
        return RouteActivationCommitResult3D{.result = std::move(activation)};
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};

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
  EXPECT_EQ(stale_commit_count, 1U);
  EXPECT_EQ(result.candidate_disposition,
            RouteCandidateDisposition3D::kRetireSearchAndReplan);
  EXPECT_TRUE(result.search_superseded_by_activation);
  EXPECT_FALSE(result.continuation_queued);
}

TEST(RouteLifecycleCoordinator3DTest,
     AContinuationOfARetiredSearchDoesNotDisplaceTheQueuedReplacement) {
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
  // The search that was retired was opened on another generation than the
  // one the release's replacement search is gated on.
  const std::shared_ptr<const PlannerSearchTransaction3D> retired =
      makePlannerSearchTransaction3D(input.world, input.planner_world,
                                     input.transaction->objective,
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kReplan,
                                         .base_route_generation = generation + 1U,
                                     },
                                     std::nullopt, RouteReleaseReason3D::kBlocked);
  ASSERT_NE(retired, nullptr);

  std::size_t commit_count{0U};
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, commit_count);
  bindResidentReplanSnapshot(config, input, supervisor, 600);
  config.activation_commit_boundary =
      [&commit_count](PreparedRouteActivation3D prepared,
                      const RouteActivationCommitOperation3D&) {
        ++commit_count;
        ProductionRouteActivationResult3D activation = std::move(prepared.result);
        activation.admission.activation_status =
            StaticRouteActivationStatus::kDynamicHandoffRejected;
        activation.admission.certified_pending = false;
        return RouteActivationCommitResult3D{.result = std::move(activation)};
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
  // The queue accepts requests only once the worker runs; the replacement
  // search it then serves keeps the gate on the resident generation.
  coordinator.start();
  const RouteLifecycleReplanOutcome3D queued =
      coordinator.requestReplan(RouteReleaseReason3D::kDiverged, generation);
  ASSERT_EQ(queued.status, RouteLifecycleReplanStatus3D::kQueued);

  RoutePlanner3D planner{plannerConfig()};
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  RoutePlannerUpdate3D planner_update = planner.update(*retired, vehicle_state);
  ASSERT_TRUE(planner_update.improved_incumbent.has_value());
  ASSERT_NE(planner_update.planner_session, nullptr);
  planner_update.planner_invoked = true;
  planner_update.planner_progress = SearchProgress3D::kRunning;
  planner_update.dispatch.continue_search = true;

  const RouteLifecycleUpdate3D result = coordinator.advance(RoutePlanningUpdateEvent3D{
      .request =
          RoutePlanningRequest3D{
              .transaction = retired,
              .continuation_session = nullptr,
          },
      .vehicle_state = vehicle_state,
      .update = std::move(planner_update),
  });

  EXPECT_EQ(result.status, RouteLifecycleAdvanceStatus3D::kCompleted);
  EXPECT_TRUE(result.search_running);
  EXPECT_FALSE(result.search_superseded_by_activation);
  EXPECT_TRUE(result.search_retired);
  EXPECT_FALSE(result.continuation_queued);
  coordinator.stop();
}

// The continuity base a stitched replacement carries: the route the vehicle is
// still flying, with a stitch window ahead of it.
[[nodiscard]] std::optional<PlannerSearchContinuityBase3D>
stitchBase(const ExecutionSupervisor3D& supervisor) {
  const std::shared_ptr<const ExecutionPlan3D> plan = supervisor.plan();
  if (plan == nullptr || plan->route() == nullptr) {
    return std::nullopt;
  }
  return PlannerSearchContinuityBase3D{
      .route = std::make_shared<const CertifiedRouteSuffix3D>(*plan->route()),
      .request_projection = {},
      .stitch_limit_station_m = 10.0,
      .minimum_stitch_station_m = 1.0,
  };
}

// A route released as blocked and replaced from the vehicle is a route the
// vehicle cannot follow: the search must start from the vehicle rather than
// keep improving it, so the replacement reaches the vehicle in the update it
// is found rather than one to three updates later.
TEST(RouteLifecycleCoordinator3DTest,
     ABlockedReplacementFromTheVehicleRejectsTheIncumbent) {
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
  const auto blocked_replacement = [&](const bool stitched) {
    return makePlannerSearchTransaction3D(
        input.world, input.planner_world, input.transaction->objective,
        StaticRouteSearchRequestIdentity{
            .kind = StaticRouteSearchRequestKind::kReplan,
            .base_route_generation = generation,
        },
        stitched ? stitchBase(supervisor)
                 : std::optional<PlannerSearchContinuityBase3D>{},
        RouteReleaseReason3D::kBlocked);
  };
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  const auto advance =
      [&](const std::shared_ptr<const PlannerSearchTransaction3D>& transaction) {
        std::size_t commit_count{0U};
        RouteLifecycleCoordinatorConfig3D config =
            lifecycleConfig(input, supervisor, commit_count);
        bindResidentReplanSnapshot(config, input, supervisor, 600);
        config.activation_commit_boundary =
            [&commit_count](PreparedRouteActivation3D prepared,
                            const RouteActivationCommitOperation3D&) {
              ++commit_count;
              ProductionRouteActivationResult3D activation = std::move(prepared.result);
              activation.admission.activation_status =
                  StaticRouteActivationStatus::kInsufficientSuccessorImprovement;
              activation.admission.certified_pending = false;
              return RouteActivationCommitResult3D{.result = std::move(activation)};
            };
        RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
        RoutePlanner3D planner{plannerConfig()};
        RoutePlannerUpdate3D planner_update =
            planner.update(*transaction, vehicle_state);
        planner_update.planner_invoked = true;
        planner_update.planner_progress = SearchProgress3D::kRunning;
        planner_update.dispatch.continue_search = true;
        return coordinator.advance(RoutePlanningUpdateEvent3D{
            .request =
                RoutePlanningRequest3D{
                    .transaction = transaction,
                    .continuation_session = nullptr,
                },
            .vehicle_state = vehicle_state,
            .update = std::move(planner_update),
        });
      };

  EXPECT_TRUE(advance(blocked_replacement(false)).incumbent_rejected);
  // A stitched replacement keeps it: its prefix is what the vehicle is flying.
  EXPECT_FALSE(advance(blocked_replacement(true)).incumbent_rejected);
}

TEST(RouteLifecycleCoordinator3DRetirementTest,
     ANewerWorldRefusingTheCandidateRetiresTheSessionWhereverTheRefusalLies) {
  // A raw revision newer than the search's world refused the candidate. The
  // session searches the world its transaction carries and a continuation
  // never sees the revision that refused it, so it is retired whether the
  // refusal lies within the committed route or beyond it, and the replan
  // opens a session on the world that refused.
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
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  const auto advance = [&](const bool refused_beyond_the_committed_route) {
    std::size_t commit_count{0U};
    RouteLifecycleCoordinatorConfig3D config =
        lifecycleConfig(input, supervisor, commit_count);
    bindResidentReplanSnapshot(config, input, supervisor, 600);
    config.extension.required_certified_overlap_m = 1.0;
    const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
        makePlannerSearchTransaction3D(
            input.world, input.planner_world, input.transaction->objective,
            StaticRouteSearchRequestIdentity{
                .kind = StaticRouteSearchRequestKind::kReplan,
                .base_route_generation = generation,
            },
            std::optional<PlannerSearchContinuityBase3D>{},
            RouteReleaseReason3D::kNoActiveRoute);
    const std::uint64_t search_raw_revision = transaction->planner_world->revision;
    const std::uint64_t search_occupied_fingerprint =
        transaction->planner_world->occupied_fingerprint;
    config.activation_commit_boundary =
        [&commit_count, refused_beyond_the_committed_route, search_raw_revision,
         search_occupied_fingerprint](PreparedRouteActivation3D prepared,
                                      const RouteActivationCommitOperation3D&) {
          ++commit_count;
          ProductionRouteActivationResult3D activation = std::move(prepared.result);
          const std::size_t sample_count = activation.materialized.route != nullptr
                                               ? activation.materialized.route->size()
                                               : 0U;
          activation.materialized.departure_end_station_m = 0.0;
          activation.admission.activation_status =
              StaticRouteActivationStatus::kCandidateValidationRejected;
          activation.admission.candidate_validation = StaticRouteCandidateValidation{
              .status = StaticRouteCandidateStatus::kRawCollision,
              .failure_segment_index =
                  refused_beyond_the_committed_route ? sample_count - 1U : 1U,
          };
          // The activation world is one revision past the search's, with
          // different occupied evidence.
          activation.admission.snapshot_raw_revision = search_raw_revision + 1U;
          activation.admission.tracking_profile_activation_occupied_fingerprint =
              search_occupied_fingerprint + 1U;
          activation.admission.certified_pending = false;
          return RouteActivationCommitResult3D{.result = std::move(activation)};
        };
    RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
    RoutePlanner3D planner{plannerConfig()};
    RoutePlannerUpdate3D planner_update = planner.update(*transaction, vehicle_state);
    planner_update.planner_invoked = true;
    planner_update.planner_progress = SearchProgress3D::kRunning;
    planner_update.dispatch.continue_search = true;
    return coordinator.advance(RoutePlanningUpdateEvent3D{
        .request =
            RoutePlanningRequest3D{
                .transaction = transaction,
                .continuation_session = nullptr,
            },
        .vehicle_state = vehicle_state,
        .update = std::move(planner_update),
    });
  };

  const RouteLifecycleUpdate3D within = advance(false);
  EXPECT_EQ(within.candidate_disposition,
            RouteCandidateDisposition3D::kRetireSearchAndReplan);
  EXPECT_TRUE(within.search_superseded_by_activation);

  const RouteLifecycleUpdate3D beyond = advance(true);
  EXPECT_EQ(beyond.candidate_disposition,
            RouteCandidateDisposition3D::kRetireSearchAndReplan);
  EXPECT_TRUE(beyond.search_superseded_by_activation);
}

TEST(RouteLifecycleCoordinator3DRetirementTest,
     ARefusalBeyondTheCommittedRouteKeepsTheIncumbent) {
  // The activation refused the candidate over its compiled geometry. Where the
  // refusal lies inside what the vehicle is committed to, the route is one it
  // cannot enter and the incumbent goes; where it lies beyond, it is a block
  // ahead like any other and the incumbent stays for the search to repair.
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
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  std::size_t observed_samples{0U};
  const auto advance = [&](const bool refused_beyond_the_committed_route) {
    std::size_t commit_count{0U};
    RouteLifecycleCoordinatorConfig3D config =
        lifecycleConfig(input, supervisor, commit_count);
    bindResidentReplanSnapshot(config, input, supervisor, 600);
    config.extension.required_certified_overlap_m = 1.0;
    config.activation_commit_boundary = [&commit_count, &observed_samples,
                                         refused_beyond_the_committed_route](
                                            PreparedRouteActivation3D prepared,
                                            const RouteActivationCommitOperation3D&) {
      ++commit_count;
      ProductionRouteActivationResult3D activation = std::move(prepared.result);
      activation.admission.activation_status =
          StaticRouteActivationStatus::kInvalidExecutionGeometry;
      const std::size_t sample_count = activation.materialized.route != nullptr
                                           ? activation.materialized.route->size()
                                           : 0U;
      observed_samples = sample_count;
      // The fixture flies its whole short route as a departure; the rule under
      // test is the station of the refusal against the committed length, so
      // the departure is staged out of the way and the overlap alone decides.
      activation.materialized.departure_end_station_m = 0.0;
      activation.admission.trajectory_validation = CompiledTrajectoryValidation3D{
          .reason = CompiledTrajectoryFailureReason3D::kInvalidTrackingErrorTubeSegment,
          .sample_index = refused_beyond_the_committed_route ? sample_count - 1U : 1U,
      };
      activation.admission.certified_pending = false;
      return RouteActivationCommitResult3D{.result = std::move(activation)};
    };
    RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
    RoutePlanner3D planner{plannerConfig()};
    const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
        makePlannerSearchTransaction3D(
            input.world, input.planner_world, input.transaction->objective,
            StaticRouteSearchRequestIdentity{
                .kind = StaticRouteSearchRequestKind::kReplan,
                .base_route_generation = generation,
            },
            std::optional<PlannerSearchContinuityBase3D>{},
            RouteReleaseReason3D::kNoActiveRoute);
    RoutePlannerUpdate3D planner_update = planner.update(*transaction, vehicle_state);
    planner_update.planner_invoked = true;
    planner_update.planner_progress = SearchProgress3D::kRunning;
    planner_update.dispatch.continue_search = true;
    return coordinator.advance(RoutePlanningUpdateEvent3D{
        .request =
            RoutePlanningRequest3D{
                .transaction = transaction,
                .continuation_session = nullptr,
            },
        .vehicle_state = vehicle_state,
        .update = std::move(planner_update),
    });
  };

  EXPECT_TRUE(advance(false).incumbent_rejected);
  EXPECT_FALSE(advance(true).incumbent_rejected) << "samples=" << observed_samples;
}

TEST(RouteLifecycleCoordinator3DRetirementTest,
     ARefusalBeyondTheCommittedRouteRejectsTheIncumbentOfAVehicleWithoutARoute) {
  // The same refusal, far ahead on the candidate, but the vehicle holds no
  // route: the refused candidate was its only offer, and keeping it as the
  // incumbent would leave the search improving a route the compile refuses
  // instead of searching from the vehicle.
  ExecutionSupervisor3D supervisor;
  const LifecycleFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  ASSERT_TRUE(supervisor.plan() == nullptr || supervisor.plan()->route() == nullptr);
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  std::size_t commit_count{0U};
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, commit_count);
  bindResidentReplanSnapshot(config, input, supervisor, 600);
  config.extension.required_certified_overlap_m = 1.0;
  config.activation_commit_boundary = [&commit_count](
                                          PreparedRouteActivation3D prepared,
                                          const RouteActivationCommitOperation3D&) {
    ++commit_count;
    ProductionRouteActivationResult3D activation = std::move(prepared.result);
    activation.admission.activation_status =
        StaticRouteActivationStatus::kInvalidExecutionGeometry;
    const std::size_t sample_count = activation.materialized.route != nullptr
                                         ? activation.materialized.route->size()
                                         : 0U;
    activation.materialized.departure_end_station_m = 0.0;
    activation.admission.trajectory_validation = CompiledTrajectoryValidation3D{
        .reason = CompiledTrajectoryFailureReason3D::kInvalidTrackingErrorTubeSegment,
        .sample_index = sample_count > 0U ? sample_count - 1U : 0U,
    };
    activation.admission.certified_pending = false;
    return RouteActivationCommitResult3D{.result = std::move(activation)};
  };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
  RoutePlanner3D planner{plannerConfig()};
  RoutePlannerUpdate3D planner_update =
      planner.update(*input.transaction, vehicle_state);
  planner_update.planner_invoked = true;
  planner_update.planner_progress = SearchProgress3D::kRunning;
  planner_update.dispatch.continue_search = true;
  const RouteLifecycleUpdate3D update = coordinator.advance(RoutePlanningUpdateEvent3D{
      .request =
          RoutePlanningRequest3D{
              .transaction = input.transaction,
              .continuation_session = nullptr,
          },
      .vehicle_state = vehicle_state,
      .update = std::move(planner_update),
  });

  ASSERT_GT(commit_count, 0U);
  EXPECT_TRUE(update.incumbent_rejected);
}

TEST(RouteLifecycleCoordinator3DRetirementTest,
     ACompileThatFailsOnItsOwnTermsKeepsTheIncumbent) {
  // A compile refusal is the raw world's verdict on the candidate only when a
  // segment's swept hull is what the evidence refused. Every other reason is
  // the pipeline's own and says nothing about where the vehicle may fly: one
  // recorded flight lost its route to twenty-four compiles that had no route
  // to compile at all.
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
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  std::size_t commit_count{0U};
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, commit_count);
  bindResidentReplanSnapshot(config, input, supervisor, 600);
  config.activation_commit_boundary =
      [&commit_count](PreparedRouteActivation3D prepared,
                      const RouteActivationCommitOperation3D&) {
        ++commit_count;
        ProductionRouteActivationResult3D activation = std::move(prepared.result);
        activation.admission.activation_status =
            StaticRouteActivationStatus::kInvalidExecutionGeometry;
        activation.admission.trajectory_validation = CompiledTrajectoryValidation3D{
            .reason = CompiledTrajectoryFailureReason3D::kMissingRoute,
            .sample_index = 0U,
        };
        activation.admission.certified_pending = false;
        return RouteActivationCommitResult3D{.result = std::move(activation)};
      };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
  RoutePlanner3D planner{plannerConfig()};
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(input.world, input.planner_world,
                                     input.transaction->objective,
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kReplan,
                                         .base_route_generation = generation,
                                     },
                                     std::optional<PlannerSearchContinuityBase3D>{},
                                     RouteReleaseReason3D::kNoActiveRoute);
  RoutePlannerUpdate3D planner_update = planner.update(*transaction, vehicle_state);
  planner_update.planner_invoked = true;
  planner_update.planner_progress = SearchProgress3D::kRunning;
  planner_update.dispatch.continue_search = true;

  const RouteLifecycleUpdate3D update = coordinator.advance(RoutePlanningUpdateEvent3D{
      .request =
          RoutePlanningRequest3D{
              .transaction = transaction,
              .continuation_session = nullptr,
          },
      .vehicle_state = vehicle_state,
      .update = std::move(planner_update),
  });

  EXPECT_FALSE(update.incumbent_rejected);
}

} // namespace

// A search that has delivered nothing is what a routeless vehicle waits on,
// and it keeps every later request deferred behind it. Past the retry
// interval the request retires it and takes its place; before that it waits,
// so an ordinary search is not restarted on every tick.
TEST(RouteLifecycleCoordinator3DRetirementTest,
     ARoutelessRequestRetiresASearchThatHasDeliveredNothing) {
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
  std::size_t commit_count{0U};
  RouteLifecycleCoordinatorConfig3D config =
      lifecycleConfig(input, supervisor, commit_count);
  std::int64_t stamp_ns{600};
  config.replan_snapshot_provider = [&input, &supervisor, &stamp_ns]() {
    const std::shared_ptr<const ExecutionPlan3D> execution = supervisor.plan();
    return RouteLifecycleReplanSnapshot3D{
        .navigation = input.navigation,
        .objective = input.objective,
        .resident_world = input.world,
        .resident_planner_world = input.planner_world,
        .latest_raw_world = nullptr,
        .world_telemetry = {},
        .committed_route_generation =
            execution != nullptr ? execution->routeGenerationHighWater() : 0U,
        .blocked_raw_revision = 0U,
        .minimum_route_mission_epoch = 0U,
        .minimum_route_sample_sequence = 0U,
        .stamp_ns = stamp_ns,
    };
  };
  RouteLifecycleCoordinator3D coordinator{supervisor, std::move(config)};
  coordinator.start();

  const RouteLifecycleReplanOutcome3D queued =
      coordinator.requestReplan(RouteReleaseReason3D::kNoActiveRoute, generation);
  ASSERT_EQ(queued.status, RouteLifecycleReplanStatus3D::kQueued)
      << routeLifecycleReplanStatus3DName(queued.status);

  // Inside the retry interval the in-flight search is left to run.
  stamp_ns += 400'000'000;
  const RouteLifecycleReplanOutcome3D early =
      coordinator.requestReplan(RouteReleaseReason3D::kNoActiveRoute, generation);
  EXPECT_FALSE(early.cleared_gate_generation.has_value());
  EXPECT_EQ(early.status, RouteLifecycleReplanStatus3D::kDeferredReplanInFlight)
      << routeLifecycleReplanStatus3DName(early.status);

  // Past it, the request takes the search's place instead of waiting.
  stamp_ns += 1'200'000'000;
  const RouteLifecycleReplanOutcome3D late =
      coordinator.requestReplan(RouteReleaseReason3D::kNoActiveRoute, generation);
  EXPECT_NE(late.status, RouteLifecycleReplanStatus3D::kDeferredReplanInFlight)
      << routeLifecycleReplanStatus3DName(late.status);
  EXPECT_TRUE(late.cleared_gate_generation.has_value());
}

} // namespace drone_city_nav
