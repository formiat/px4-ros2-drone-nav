#include "drone_city_nav/execution_supervisor_3d.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::shared_ptr<const VersionedObservedRawWorld3D>
rawOwner(const ExecutionPlan3D& plan) {
  if (const FiniteExecutionState3D* const execution = plan.finiteExecution()) {
    return execution->observed_raw_world;
  }
  if (const DirectTrackingFiniteExecution3D* const execution =
          plan.directTrackingExecution()) {
    return execution->observed_raw_world;
  }
  if (const StationaryExecutionHold3D* const hold = plan.stationaryHold()) {
    return hold->observed_raw_world;
  }
  return nullptr;
}

[[nodiscard]] std::shared_ptr<const VersionedLatestLidarEvidence3D>
lidarOwner(const ExecutionPlan3D& plan) {
  if (const FiniteExecutionState3D* const execution = plan.finiteExecution()) {
    return execution->latest_lidar_evidence;
  }
  if (const DirectTrackingFiniteExecution3D* const execution =
          plan.directTrackingExecution()) {
    return execution->latest_lidar_evidence;
  }
  if (const StationaryExecutionHold3D* const hold = plan.stationaryHold()) {
    return hold->latest_lidar_evidence;
  }
  return nullptr;
}

[[nodiscard]] ExecutionHorizonNavigationWitness3D
navigationWitness(const VersionedExecutionInput3D& input) {
  return ExecutionHorizonNavigationWitness3D{
      .state = input.state(),
      .measured_equivalent_control = input.previousControl(),
      .pose_revision = input.poseRevision(),
      .source_timestamp_us = input.poseSourceTimestampUs(),
      .receive_stamp_ns = input.poseReceiveStampNs(),
      .measured_control_source_sequence = input.previousControlSourceSequence(),
      .measured_control_source_stamp_ns = input.previousControlSourceStampNs(),
      .measured_control_receive_stamp_ns = input.previousControlReceiveStampNs(),
      .measured_acceleration_authoritative = true,
  };
}

[[nodiscard]] ExecutionHorizonCommitRequest3D transitionRequest(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& authority,
    const ExecutionRouteTransitionResult3D& transition,
    const ExecutionHorizonCommitKind3D kind = ExecutionHorizonCommitKind3D::kTransition,
    std::shared_ptr<const PendingCertifiedRoute3D> pending = nullptr) {
  const std::shared_ptr<const ExecutionPlan3D> expected = authority->plan();
  const std::shared_ptr<const ExecutionPlan3D> next = transition.next;
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      SnapshotFixture3D::committedInput(*next);
  const std::shared_ptr<const VersionedObservedRawWorld3D> raw = rawOwner(*next);
  const ExecutionOwnerIdentity3D owner = SnapshotFixture3D::committedOwner(*next);
  return ExecutionHorizonCommitRequest3D{
      .candidate =
          ExecutionHorizonLeaseCandidate3D{
              .kind = kind,
              .expected_authority = authority,
              .expected_plan = expected,
              .certification_plan = expected,
              .progress_preparation = nullptr,
              .transition =
                  std::make_shared<const ExecutionRouteTransitionResult3D>(transition),
              .expected_pending = std::move(pending),
              .owner = owner,
              .expected_horizon_producer_instance_id = owner.producer_instance_id,
              .execution_input = input,
          },
      .runtime =
          ExecutionHorizonRuntimeCurrentness3D{
              .vehicle_status_authoritative = true,
              .current_raw_age_ms = 0.0,
              .maximum_raw_age_ms = 1000.0,
              .revocation_epoch_current = true,
              .objective_current = true,
              .navigation_authoritative = true,
              .offboard_session_currentness =
                  OffboardSessionPublicationCurrentnessStatus::kCurrent,
          },
      .navigation = navigationWitness(*input),
      .current_observed_raw_world = raw,
      .current_lidar_evidence = lidarOwner(*next),
      .publication_now_ns = owner.valid_from_ns,
      .maximum_control_feedback_age_ms = 1000.0,
  };
}

[[nodiscard]] ExecutionRouteTransitionResult3D
activeTransition(const ExecutionPlan3D& initial, SnapshotFixture3D& fixture) {
  const std::optional<CertifiedRouteSuffix3D> certified = fixture.certify();
  if (!certified.has_value()) {
    return {};
  }
  return activateCertifiedRoute3D(
      initial, initial.version,
      certified.value(), // NOLINT(bugprone-unchecked-optional-access)
      SnapshotFixture3D::finitePlanForRoute(
          initial,
          certified.value(), // NOLINT(bugprone-unchecked-optional-access)
          FiniteExecutionKind3D::kNominal, 100U));
}

[[nodiscard]] std::shared_ptr<const VersionedLatestLidarEvidence3D>
lidarAdvancedBy(const VersionedLatestLidarEvidence3D& previous,
                const std::int64_t advance_ns) {
  return VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
      .producer_instance_id = previous.producerInstanceId(),
      .sequence = previous.sequence() + 1U,
      .pose_generation = previous.poseGeneration() + 1U,
      .acquisition_stamp_ns = previous.acquisitionStampNs() + advance_ns,
      .receive_stamp_ns = previous.receiveStampNs() + advance_ns,
      .source_beam_count = previous.sourceBeamCount(),
      .invalid_beam_count = previous.invalidBeamCount(),
      .hit_points_map_m = previous.hitPointsMapM(),
  });
}

TEST(ExecutionSupervisorHorizon3DTest,
     CommitsTransitionAndUnchangedLeaseAgainstExactRuntimeEvidence) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      supervisor.authority();
  ASSERT_NE(initial_authority, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      activeTransition(*initial_authority->plan(), fixture);
  ASSERT_TRUE(transition.applied());

  const ExecutionHorizonCommitResult3D committed =
      supervisor.commitHorizon(transitionRequest(initial_authority, transition));

  ASSERT_TRUE(committed.committed())
      << executionHorizonCommitStatus3DName(committed.status);
  EXPECT_FALSE(committed.latest_evidence_revalidated);
  EXPECT_FALSE(committed.replaced_applied_control);
  const std::shared_ptr<const CommittedExecutionAuthority3D> active_authority =
      supervisor.authority();
  ASSERT_NE(active_authority, nullptr);
  EXPECT_EQ(active_authority->plan(), transition.next);
  ASSERT_TRUE(supervisor.publishAppliedControlIfSame(
      active_authority,
      SnapshotFixture3D::committedControl(active_authority->owner())));
  const std::shared_ptr<const CommittedExecutionAuthority3D> controlled_authority =
      supervisor.authority();
  ASSERT_NE(controlled_authority, nullptr);
  ASSERT_TRUE(controlled_authority->control().valid);

  ExecutionHorizonCommitRequest3D unchanged =
      transitionRequest(initial_authority, transition);
  unchanged.candidate.kind = ExecutionHorizonCommitKind3D::kUnchangedPlan;
  unchanged.candidate.expected_authority = controlled_authority;
  unchanged.candidate.expected_plan = controlled_authority->plan();
  unchanged.candidate.certification_plan = controlled_authority->plan();
  unchanged.candidate.transition.reset();
  unchanged.candidate.owner.sequence = 2U;
  const ExecutionHorizonCommitResult3D refreshed =
      supervisor.commitHorizon(std::move(unchanged));
  ASSERT_TRUE(refreshed.committed())
      << executionHorizonCommitStatus3DName(refreshed.status);
  EXPECT_TRUE(refreshed.replaced_applied_control);
  EXPECT_EQ(supervisor.authority()->owner().sequence, 2U);
  EXPECT_TRUE(supervisor.authority()->control().empty());

  const ExecutionHorizonCommitResult3D stale =
      supervisor.commitHorizon(transitionRequest(initial_authority, transition));
  EXPECT_EQ(stale.status, ExecutionHorizonCommitStatus3D::kAuthorityNotCurrent);
}

TEST(ExecutionSupervisorHorizon3DTest,
     CommitsPendingTransitionAndConsumesOnlyExactPendingOwner) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
      supervisor.authority();
  ASSERT_NE(authority, nullptr);
  const std::optional<CertifiedRouteSuffix3D> certified = fixture.certify();
  ASSERT_TRUE(certified.has_value());
  const CertifiedRouteSuffix3D& route =
      certified.value(); // NOLINT(bugprone-unchecked-optional-access)
  const PendingRoutePublicationResult3D pending =
      supervisor.publishPendingForCurrentBase(
          authority->plan(),
          PendingCertifiedRoute3D{
              .publication_sequence = 0U,
              .base_execution_owner_epoch = authority->plan()->execution_owner_epoch,
              .base_kind = PendingExecutionBaseKind3D::kEmpty,
              .base_route_generation = 0U,
              .base_geometry_revision = 0U,
              .base_continuity_id = 0U,
              .base_direct_tracking_identity = std::nullopt,
              .route_splice = std::nullopt,
              .route = route,
          });
  ASSERT_TRUE(pending.published());
  const ExecutionRouteTransitionResult3D transition = activateCertifiedRoute3D(
      *authority->plan(), authority->plan()->version, route,
      SnapshotFixture3D::finitePlanForRoute(*authority->plan(), route,
                                            FiniteExecutionKind3D::kNominal, 100U));
  ASSERT_TRUE(transition.applied());

  const ExecutionHorizonCommitResult3D committed =
      supervisor.commitHorizon(transitionRequest(
          authority, transition, ExecutionHorizonCommitKind3D::kPendingTransition,
          pending.pending));

  ASSERT_TRUE(committed.committed())
      << executionHorizonCommitStatus3DName(committed.status);
  EXPECT_EQ(supervisor.pending(), nullptr);
  EXPECT_EQ(supervisor.plan(), transition.next);
}

TEST(ExecutionSupervisorHorizon3DTest,
     RevalidatesFinitePathsWhenCurrentEvidenceAdvancesOnTheSameLineage) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
      supervisor.authority();
  ASSERT_NE(authority, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      activeTransition(*authority->plan(), fixture);
  ASSERT_TRUE(transition.applied());
  ExecutionHorizonCommitRequest3D request = transitionRequest(authority, transition);
  ASSERT_NE(request.current_lidar_evidence, nullptr);
  constexpr std::int64_t kEvidenceAdvanceNs{200'000'000LL};
  request.current_lidar_evidence =
      lidarAdvancedBy(*request.current_lidar_evidence, kEvidenceAdvanceNs);
  ASSERT_NE(request.current_lidar_evidence, nullptr);
  request.publication_now_ns = request.current_lidar_evidence->receiveStampNs();

  const ExecutionHorizonCommitResult3D committed =
      supervisor.commitHorizon(std::move(request));

  ASSERT_TRUE(committed.committed())
      << executionHorizonCommitStatus3DName(committed.status) << " currentness="
      << executionPublicationCurrentnessStatus3DName(committed.publication_currentness);
  EXPECT_TRUE(committed.latest_evidence_revalidated);
}

TEST(ExecutionSupervisorHorizon3DTest,
     RuntimeEvidenceFailureIsTypedAndLeavesAuthorityUnchanged) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
      supervisor.authority();
  ASSERT_NE(authority, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      activeTransition(*authority->plan(), fixture);
  ASSERT_TRUE(transition.applied());
  ExecutionHorizonCommitRequest3D request = transitionRequest(authority, transition);
  request.runtime.raw_world_identity_conflicted = true;

  const ExecutionHorizonCommitResult3D rejected =
      supervisor.commitHorizon(std::move(request));

  EXPECT_EQ(rejected.status, ExecutionHorizonCommitStatus3D::kRawWorldNotCurrent);
  EXPECT_EQ(rejected.revocation_request,
            ExecutionHorizonRevocationRequest3D::kUnavailableWorld);
  EXPECT_EQ(supervisor.authority(), authority);
}

TEST(ExecutionSupervisorHorizon3DTest,
     RuntimeAdmissionFailuresAreOrderedAndNeverMutateAuthority) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
      supervisor.authority();
  ASSERT_NE(authority, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      activeTransition(*authority->plan(), fixture);
  ASSERT_TRUE(transition.applied());
  const ExecutionHorizonCommitRequest3D admitted =
      transitionRequest(authority, transition);
  const auto expect_rejected = [&](ExecutionHorizonCommitRequest3D request,
                                   const ExecutionHorizonCommitStatus3D status) {
    EXPECT_EQ(supervisor.commitHorizon(std::move(request)).status, status);
    EXPECT_EQ(supervisor.authority(), authority);
  };

  ExecutionHorizonCommitRequest3D request = admitted;
  request.runtime.vehicle_status_authoritative = false;
  expect_rejected(std::move(request),
                  ExecutionHorizonCommitStatus3D::kVehicleStatusNotAuthoritative);
  request = admitted;
  request.runtime.raw_world_identity_conflicted = true;
  expect_rejected(std::move(request),
                  ExecutionHorizonCommitStatus3D::kRawWorldNotCurrent);
  request = admitted;
  request.publication_now_ns = request.candidate.owner.valid_until_ns;
  expect_rejected(std::move(request),
                  ExecutionHorizonCommitStatus3D::kHorizonTimeNotCurrent);
  request = admitted;
  request.runtime.revocation_epoch_current = false;
  expect_rejected(std::move(request),
                  ExecutionHorizonCommitStatus3D::kRevocationEpochChanged);
  request = admitted;
  request.runtime.objective_current = false;
  expect_rejected(std::move(request),
                  ExecutionHorizonCommitStatus3D::kObjectiveChanged);
  request = admitted;
  request.runtime.navigation_authoritative = false;
  expect_rejected(std::move(request),
                  ExecutionHorizonCommitStatus3D::kNavigationNotAuthoritative);
  request = admitted;
  request.runtime.offboard_session_currentness =
      OffboardSessionPublicationCurrentnessStatus::kProducerChanged;
  expect_rejected(std::move(request),
                  ExecutionHorizonCommitStatus3D::kOffboardSessionNotCurrent);
}

TEST(ExecutionSupervisorHorizon3DTest,
     NavigationAndControlWitnessMustOwnTheExactExecutionInput) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
      supervisor.authority();
  ASSERT_NE(authority, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      activeTransition(*authority->plan(), fixture);
  ASSERT_TRUE(transition.applied());
  ExecutionHorizonCommitRequest3D request = transitionRequest(authority, transition);
  ++request.navigation.pose_revision;

  EXPECT_EQ(supervisor.commitHorizon(request).status,
            ExecutionHorizonCommitStatus3D::kExecutionInputNotCurrent);
  --request.navigation.pose_revision;
  request.navigation.measured_acceleration_authoritative = false;
  EXPECT_EQ(supervisor.commitHorizon(request).status,
            ExecutionHorizonCommitStatus3D::kControlEvidenceNotCurrent);
  EXPECT_EQ(supervisor.authority(), authority);
}

} // namespace
} // namespace drone_city_nav
