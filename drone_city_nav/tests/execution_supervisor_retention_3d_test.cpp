#include "drone_city_nav/execution_supervisor_3d.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::shared_ptr<const ExecutionPlan3D>
installRouteOwner(ExecutionSupervisor3D& supervisor, SnapshotFixture3D& fixture) {
  const std::optional<CertifiedRouteSuffix3D> certified = fixture.certify();
  if (!certified.has_value()) {
    return nullptr;
  }
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      supervisor.authority();
  const std::shared_ptr<const ExecutionPlan3D> initial =
      initial_authority != nullptr ? initial_authority->plan() : nullptr;
  if (initial == nullptr) {
    return nullptr;
  }
  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *initial, initial->version, *certified,
      SnapshotFixture3D::finitePlanForRoute(*initial, *certified,
                                            FiniteExecutionKind3D::kNominal, 100U));
  if (!activation.applied() || activation.next == nullptr) {
    return nullptr;
  }
  const ExecutionRoutePublicationStatus3D publication =
      supervisor.commitLease(ExecutionLeaseCommit3D{
          .kind = ExecutionLeaseCommitKind3D::kTransition,
          .expected_authority = initial_authority,
          .expected_plan = initial,
          .transition = activation,
          .expected_pending = nullptr,
          .owner = SnapshotFixture3D::committedOwner(*activation.next),
          .input = SnapshotFixture3D::committedInput(*activation.next),
      });
  return publication == ExecutionRoutePublicationStatus3D::kPublished
             ? supervisor.plan()
             : nullptr;
}

[[nodiscard]] ExecutionRetentionRequest3D routeRetentionRequest(
    const std::shared_ptr<const ExecutionPlan3D>& source,
    std::optional<RouteLifecycleEvent3D> lifecycle_event = std::nullopt,
    std::shared_ptr<const VersionedObservedRawWorld3D> lifecycle_world = nullptr) {
  const CertifiedRouteSuffix3D* const route =
      source != nullptr ? source->route() : nullptr;
  const FiniteExecutionState3D* const active =
      source != nullptr ? source->finiteExecution() : nullptr;
  if (route == nullptr || active == nullptr) {
    return {};
  }
  FiniteExecutionCertification3D evidence =
      SnapshotFixture3D::finiteCertificationForRoute(
          *route, FiniteExecutionKind3D::kRetained, active->trajectory_revision + 1U,
          active->source_navigation_revision + 1U, 0U, -1.0,
          active->execution_input.get(), active->latest_lidar_evidence.get());
  return ExecutionRetentionRequest3D{
      .lifecycle_source_plan = source,
      .lifecycle_event = lifecycle_event,
      .lifecycle_observed_raw_world = std::move(lifecycle_world),
      .execution_input = evidence.execution_input,
      .latest_lidar_evidence = evidence.latest_lidar_evidence,
      .exact_initial_state = evidence.horizon.states.front(),
      .exact_previous_control = evidence.execution_input->previousControl(),
      .finite_horizon_config = {},
      .now_ns = evidence.valid_from_ns,
      .lidar_validation_now_ns = evidence.latest_lidar_evidence->receiveStampNs(),
  };
}

TEST(ExecutionSupervisorRetention3DTest,
     PreparesAndCommitsRouteRetentionWithoutMutatingDuringPreparation) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority =
      supervisor.authority();
  ExecutionRetentionRequest3D request = routeRetentionRequest(active);

  const ExecutionRetentionResult3D prepared = supervisor.prepareRetention(request);

  ASSERT_TRUE(prepared.prepared()) << executionRetentionStatus3DName(prepared.status);
  EXPECT_EQ(prepared.kind, ExecutionRetentionKind3D::kRoute);
  EXPECT_EQ(prepared.expected_authority, expected_authority);
  EXPECT_EQ(prepared.expectedPlan(), active);
  EXPECT_EQ(supervisor.plan(), active);
  ASSERT_NE(prepared.transition, nullptr);
  ASSERT_NE(prepared.transition->next, nullptr);
  ASSERT_NE(prepared.transition->next->finiteExecution(), nullptr);
  EXPECT_EQ(prepared.transition->next->finiteExecution()->kind,
            FiniteExecutionKind3D::kRetained);
  EXPECT_EQ(prepared.prepared_trajectory_revision,
            active->finiteExecution()->trajectory_revision + 1U);
  EXPECT_EQ(prepared.actual_state_validation.status,
            mppi::FiniteExecutionPathStatus::kValid);

  EXPECT_EQ(
      supervisor.commitLease(ExecutionLeaseCommit3D{
          .kind = ExecutionLeaseCommitKind3D::kTransition,
          .expected_authority = prepared.expected_authority,
          .expected_plan = prepared.expectedPlan(),
          .transition = *prepared.transition,
          .expected_pending = nullptr,
          .owner = SnapshotFixture3D::committedOwner(*prepared.transition->next, 2U),
          .input = request.execution_input,
      }),
      ExecutionRoutePublicationStatus3D::kPublished);
  EXPECT_EQ(supervisor.plan(), prepared.transition->next);
}

TEST(ExecutionSupervisorRetention3DTest,
     RawInvalidationPreparesOnlyAnExactOwnerEmergencyBrakeTail) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->route(), nullptr);
  const RouteLifecycleEvent3D invalidation{
      .kind = RouteLifecycleEventKind3D::kRawInvalidated,
      .generation = active->route()->identity.generation,
      .raw_producer_instance_id = SnapshotFixture3D::kRawProducer,
      .raw_revision = SnapshotFixture3D::kLatestRawRevision + 1U,
  };
  const std::shared_ptr<const VersionedObservedRawWorld3D> invalidating_world =
      fixture.rawWorld(invalidation.raw_revision);
  ASSERT_NE(invalidating_world, nullptr);
  ExecutionRetentionRequest3D request =
      routeRetentionRequest(active, invalidation, invalidating_world);

  const ExecutionRetentionResult3D prepared = supervisor.prepareRetention(request);

  ASSERT_TRUE(prepared.prepared()) << executionRetentionStatus3DName(prepared.status);
  ASSERT_TRUE(prepared.braking_event.has_value());
  const RouteLifecycleEventKind3D braking_event =
      prepared.braking_event.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_EQ(braking_event, RouteLifecycleEventKind3D::kRawInvalidated);
  ASSERT_NE(prepared.transition->next->finiteExecution(), nullptr);
  EXPECT_EQ(prepared.transition->next->phase(), ExecutionRoutePhase3D::kBraking);
  EXPECT_EQ(prepared.transition->next->finiteExecution()->kind,
            FiniteExecutionKind3D::kEmergencyBrakeTail);
  EXPECT_EQ(prepared.transition->next->finiteExecution()->observed_raw_world,
            invalidating_world);
  EXPECT_EQ(supervisor.plan(), active);

  request.lifecycle_source_plan = makeInitialExecutionRouteSnapshot3D();
  const ExecutionRetentionResult3D stale = supervisor.prepareRetention(request);
  EXPECT_EQ(stale.status, ExecutionRetentionStatus3D::kInvalidLifecycleOwner);
  EXPECT_FALSE(stale.prepared());
  EXPECT_EQ(supervisor.plan(), active);
}

TEST(ExecutionSupervisorRetention3DTest,
     PreparesDirectTrackingRetentionFromTheResidentOwner) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> route_owner =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(route_owner, nullptr);
  ASSERT_NE(route_owner->route(), nullptr);
  const CertifiedRouteSuffix3D path_route = *route_owner->route();
  const DirectTrackingOwnerIdentity3D identity{
      .mission_epoch = fixture.objective.mission_epoch,
      .assignment_generation = fixture.objective.assignment_generation,
      .target_detection_id = 41U,
      .target_track_id = 42U,
      .objective_sample_sequence = fixture.objective.sample_sequence,
      .line_of_sight_generation = 7U,
  };
  const std::optional<DirectTrackingFiniteExecution3D> direct_execution =
      certifyDirectFixtureExecution(*route_owner, path_route, identity, 101U);
  ASSERT_TRUE(direct_execution.has_value());
  const DirectTrackingFiniteExecution3D& certified_direct =
      direct_execution.value(); // NOLINT(bugprone-unchecked-optional-access)
  const ExecutionRouteTransitionResult3D direct_transition =
      transferToDirectTracking3D(*route_owner, route_owner->version, certified_direct);
  ASSERT_TRUE(direct_transition.applied());
  const std::shared_ptr<const CommittedExecutionAuthority3D> route_authority =
      supervisor.authority();
  ASSERT_EQ(supervisor.commitLease(ExecutionLeaseCommit3D{
                .kind = ExecutionLeaseCommitKind3D::kTransition,
                .expected_authority = route_authority,
                .expected_plan = route_owner,
                .transition = direct_transition,
                .expected_pending = nullptr,
                .owner = SnapshotFixture3D::committedOwner(*direct_transition.next, 2U),
                .input = SnapshotFixture3D::committedInput(*direct_transition.next),
            }),
            ExecutionRoutePublicationStatus3D::kPublished);
  const std::shared_ptr<const ExecutionPlan3D> direct_owner = supervisor.plan();
  ASSERT_NE(direct_owner, nullptr);
  ASSERT_NE(direct_owner->directTrackingExecution(), nullptr);
  const std::optional<DirectTrackingFiniteExecution3D> retained_evidence =
      certifyDirectFixtureExecution(
          *direct_owner, path_route, identity,
          direct_owner->directTrackingExecution()->trajectory_revision + 1U,
          FiniteExecutionKind3D::kRetained);
  ASSERT_TRUE(retained_evidence.has_value());
  const DirectTrackingFiniteExecution3D& retained_direct =
      retained_evidence.value(); // NOLINT(bugprone-unchecked-optional-access)
  const ExecutionRetentionRequest3D request{
      .lifecycle_source_plan = direct_owner,
      .lifecycle_event = std::nullopt,
      .lifecycle_observed_raw_world = nullptr,
      .execution_input = retained_direct.execution_input,
      .latest_lidar_evidence = retained_direct.latest_lidar_evidence,
      .exact_initial_state = retained_direct.horizon->states.front(),
      .exact_previous_control = retained_direct.execution_input->previousControl(),
      .finite_horizon_config = {},
      .now_ns = retained_direct.valid_from_ns,
      .lidar_validation_now_ns =
          retained_direct.latest_lidar_evidence->receiveStampNs(),
  };

  const ExecutionRetentionResult3D prepared = supervisor.prepareRetention(request);

  ASSERT_TRUE(prepared.prepared()) << executionRetentionStatus3DName(prepared.status);
  EXPECT_EQ(prepared.kind, ExecutionRetentionKind3D::kDirectTracking);
  ASSERT_NE(prepared.transition->next->directTrackingExecution(), nullptr);
  EXPECT_EQ(prepared.transition->next->directTrackingExecution()->kind,
            FiniteExecutionKind3D::kRetained);
  EXPECT_EQ(
      prepared.transition->next->directTrackingExecution()->identity.target_track_id,
      identity.target_track_id);
  EXPECT_EQ(supervisor.plan(), direct_owner);
}

TEST(ExecutionSupervisorRetention3DTest,
     RejectsMissingCurrentLidarWithoutChangingTheResidentOwner) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ExecutionRetentionRequest3D request = routeRetentionRequest(active);
  request.latest_lidar_evidence.reset();

  const ExecutionRetentionResult3D rejected =
      supervisor.prepareRetention(std::move(request));

  EXPECT_EQ(rejected.status, ExecutionRetentionStatus3D::kValidationWorldUnavailable);
  EXPECT_FALSE(rejected.prepared());
  EXPECT_EQ(supervisor.plan(), active);
}

} // namespace
} // namespace drone_city_nav
