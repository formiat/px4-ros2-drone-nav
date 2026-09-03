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

} // namespace
} // namespace drone_city_nav
