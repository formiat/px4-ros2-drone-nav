#include "drone_city_nav/execution_supervisor_3d.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "execution_route_snapshot_3d_plan_test_support.hpp"
#include "execution_supervisor_horizon_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::shared_ptr<const ExecutionPlan3D>
installCertifiedRouteOwner(ExecutionSupervisor3D& supervisor,
                           const CertifiedRouteSuffix3D& certified) {
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      supervisor.authority();
  const std::shared_ptr<const ExecutionPlan3D> initial =
      initial_authority != nullptr ? initial_authority->plan() : nullptr;
  if (initial == nullptr) {
    return nullptr;
  }
  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *initial, initial->version, certified,
      SnapshotFixture3D::finitePlanForRoute(*initial, certified,
                                            FiniteExecutionKind3D::kNominal, 100U));
  if (!activation.applied() || activation.next == nullptr) {
    return nullptr;
  }
  const ExecutionHorizonCommitResult3D publication = commitExecutionHorizonForTest(
      supervisor, ExecutionHorizonTestTransaction3D{
                      .kind = ExecutionHorizonCommitKind3D::kTransition,
                      .expected_authority = initial_authority,
                      .expected_plan = initial,
                      .transition = activation,
                      .expected_pending = nullptr,
                      .owner = SnapshotFixture3D::committedOwner(*activation.next),
                      .input = SnapshotFixture3D::committedInput(*activation.next),
                  });
  return publication.committed() ? supervisor.plan() : nullptr;
}

[[nodiscard]] std::shared_ptr<const ExecutionPlan3D>
installRouteOwner(ExecutionSupervisor3D& supervisor, SnapshotFixture3D& fixture) {
  const std::optional<CertifiedRouteSuffix3D> certified = fixture.certify();
  return certified.has_value()
             ? installCertifiedRouteOwner(
                   supervisor,
                   certified.value()) // NOLINT(bugprone-unchecked-optional-access)
             : nullptr;
}

[[nodiscard]] ExecutionHoldRequest3D holdRequest(
    const std::shared_ptr<const ExecutionPlan3D>& source,
    const StationaryExecutionHoldCertification3D& certification,
    const ExecutionHoldIntent3D intent = ExecutionHoldIntent3D::kExplicitTransfer) {
  return ExecutionHoldRequest3D{
      .intent = intent,
      .requested_position = certification.position,
      .cycle_source_plan = source,
      .execution_input = certification.execution_input,
      .latest_lidar_evidence = certification.latest_lidar_evidence,
      .current_lidar_evidence = certification.latest_lidar_evidence,
      .current_observed_raw_world = certification.observed_raw_world,
      .stationary_capture_observed_raw_world = certification.observed_raw_world,
      .stationary_capture_static_world = certification.static_world,
      .selected_validation_policy = certification.validation_policy,
      .stationary_capture_validation_policy = certification.validation_policy,
      .validation_now_ns = certification.execution_input != nullptr
                               ? certification.execution_input->effectiveStampNs()
                               : 0,
  };
}

[[nodiscard]] ExecutionOwnerIdentity3D holdOwner(const ExecutionPlan3D& plan,
                                                 const std::uint64_t sequence) {
  ExecutionOwnerIdentity3D owner = SnapshotFixture3D::committedOwner(plan, sequence);
  const StationaryExecutionHold3D* const hold = plan.stationaryHold();
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      SnapshotFixture3D::committedInput(plan);
  if (hold == nullptr || input == nullptr) {
    return {};
  }
  owner.valid_from_ns = input->effectiveStampNs();
  owner.valid_until_ns = input->effectiveStampNs() + 1'000'000'000LL;
  owner.stationary_hold_position = hold->position;
  owner.execution_mode = ExecutionAuthorityMode3D::kPositionHold;
  owner.execution_reason = ExecutionAuthorityReason3D::kGoalCapture;
  owner.stationary_position_hold = true;
  return owner;
}

[[nodiscard]] std::shared_ptr<const VersionedExecutionInput3D>
stationaryCaptureInput(const VersionedExecutionInput3D& source) {
  const ExecutionStateProvenance3D source_sample{
      .x = ExecutionStateFieldProvenance3D::kSourceSample,
      .y = ExecutionStateFieldProvenance3D::kSourceSample,
      .z = ExecutionStateFieldProvenance3D::kSourceSample,
      .vx = ExecutionStateFieldProvenance3D::kSourceSample,
      .vy = ExecutionStateFieldProvenance3D::kSourceSample,
      .vz = ExecutionStateFieldProvenance3D::kSourceSample,
      .yaw = ExecutionStateFieldProvenance3D::kSourceSample,
      .yaw_rate = ExecutionStateFieldProvenance3D::kSourceSample,
  };
  return VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
      .capture_sequence = source.captureSequence() + 1U,
      .pose_revision = source.poseRevision() + 1U,
      .pose_source_timestamp_us = source.poseSourceTimestampUs() + 1U,
      .pose_receive_stamp_ns = source.effectiveStampNs(),
      .effective_stamp_ns = source.effectiveStampNs(),
      .state = source.state(),
      .full_state_authoritative = true,
      .state_provenance = source_sample,
      .purpose = ExecutionInputPurpose3D::kStationaryCaptureRearm,
      .previous_control = {},
      .previous_control_source = ExecutionPreviousControlEvidenceSource3D::kAssumedZero,
      .previous_control_source_sequence = source.captureSequence() + 1U,
      .previous_control_source_stamp_ns = source.effectiveStampNs(),
      .previous_control_receive_stamp_ns = source.effectiveStampNs(),
  });
}

TEST(ExecutionSupervisorHold3DTest,
     PreparesAndCommitsTerminalTransferWithoutMutatingDuringPreparation) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  const std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority =
      supervisor.authority();
  const StationaryExecutionHoldCertification3D certification =
      SnapshotFixture3D::holdCertification(*active);

  const ExecutionHoldPreparation3D prepared =
      supervisor.prepareHold(holdRequest(active, certification));

  ASSERT_TRUE(prepared.prepared())
      << executionHoldPreparationStatus3DName(prepared.status);
  EXPECT_EQ(prepared.kind, ExecutionHoldPreparationKind3D::kTransition);
  EXPECT_EQ(prepared.expected_authority, expected_authority);
  EXPECT_EQ(prepared.expectedPlan(), active);
  EXPECT_EQ(prepared.executionInput(), certification.execution_input);
  EXPECT_EQ(supervisor.plan(), active);
  ASSERT_NE(prepared.transition, nullptr);
  ASSERT_NE(prepared.transition->next, nullptr);
  ASSERT_NE(prepared.transition->next->stationaryHold(), nullptr);

  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kTransition,
                    .expected_authority = prepared.expected_authority,
                    .expected_plan = prepared.expectedPlan(),
                    .transition = *prepared.transition,
                    .expected_pending = nullptr,
                    .owner = holdOwner(*prepared.transition->next, 2U),
                    .input = prepared.executionInput(),
                })
                .status,
            ExecutionHorizonCommitStatus3D::kCommitted);
  EXPECT_EQ(supervisor.plan(), prepared.transition->next);
}

TEST(ExecutionSupervisorHold3DTest,
     ARestHoldTransfersFromARouteOwnerWhosePathTheNewestWorldBlocks) {
  // The newest raw world blocks the route ahead of the vehicle; the resting
  // vehicle itself stands clear. The hold takes the vehicle over from the
  // route's finite execution on that world: it validates the rest position,
  // never the path the evidence has just invalidated, and it is what the
  // node publishes when a stop has nothing left to brake.
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ObservedOccupancyGrid3D blocked_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> blocked_cell =
      blocked_occupancy.worldToCell(Point3{6.0, 0.0, 5.0});
  ASSERT_TRUE(blocked_cell.has_value());
  ASSERT_TRUE(
      blocked_occupancy.setState(blocked_cell.value(), ObservedVoxelState::kOccupied));
  StationaryExecutionHoldCertification3D certification =
      SnapshotFixture3D::holdCertification(*active);
  certification.observed_raw_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &blocked_occupancy);
  ASSERT_NE(certification.observed_raw_world, nullptr);

  const ExecutionHoldPreparation3D prepared =
      supervisor.prepareHold(holdRequest(active, certification));

  ASSERT_TRUE(prepared.prepared())
      << executionHoldPreparationStatus3DName(prepared.status);
  ASSERT_NE(prepared.transition, nullptr);
  ASSERT_NE(prepared.transition->next, nullptr);
  ASSERT_NE(prepared.transition->next->stationaryHold(), nullptr);
  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kTransition,
                    .expected_authority = prepared.expected_authority,
                    .expected_plan = prepared.expectedPlan(),
                    .transition = *prepared.transition,
                    .expected_pending = nullptr,
                    .owner = holdOwner(*prepared.transition->next, 2U),
                    .input = prepared.executionInput(),
                })
                .status,
            ExecutionHorizonCommitStatus3D::kCommitted);
  ASSERT_NE(supervisor.plan(), nullptr);
  ASSERT_NE(supervisor.plan()->stationaryHold(), nullptr);
  EXPECT_EQ(supervisor.plan()->finiteExecution(), nullptr);
}

TEST(ExecutionSupervisorHold3DTest,
     RefreshesAResidentInputAndUsesNoChangeOnlyForTheExactOwnerEvidence) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  const ExecutionHoldPreparation3D transfer = supervisor.prepareHold(
      holdRequest(active, SnapshotFixture3D::holdCertification(*active)));
  ASSERT_TRUE(transfer.prepared());
  ASSERT_NE(transfer.transition, nullptr);
  ASSERT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kTransition,
                    .expected_authority = transfer.expected_authority,
                    .expected_plan = transfer.expectedPlan(),
                    .transition = *transfer.transition,
                    .expected_pending = nullptr,
                    .owner = holdOwner(*transfer.transition->next, 2U),
                    .input = transfer.executionInput(),
                })
                .status,
            ExecutionHorizonCommitStatus3D::kCommitted);
  const std::shared_ptr<const ExecutionPlan3D> resident = supervisor.plan();
  ASSERT_NE(resident, nullptr);
  ASSERT_NE(resident->stationaryHold(), nullptr);
  const StationaryExecutionHold3D& resident_hold = *resident->stationaryHold();
  const StationaryExecutionHoldCertification3D exact{
      .position = resident_hold.position,
      .execution_input = resident_hold.terminal_execution_input,
      .observed_raw_world = resident_hold.observed_raw_world,
      .static_world = resident_hold.static_world,
      .validation_policy = resident_hold.validation_policy,
      .latest_lidar_evidence = resident_hold.latest_lidar_evidence,
  };

  const ExecutionHoldPreparation3D unchanged = supervisor.prepareHold(
      holdRequest(resident, exact, ExecutionHoldIntent3D::kRefreshResident));

  ASSERT_TRUE(unchanged.prepared());
  EXPECT_EQ(unchanged.kind, ExecutionHoldPreparationKind3D::kUnchangedPlan);
  EXPECT_EQ(unchanged.transition, nullptr);
  EXPECT_EQ(unchanged.preparedPlan(), resident);
  EXPECT_EQ(supervisor.plan(), resident);

  const StationaryExecutionHoldCertification3D advanced =
      SnapshotFixture3D::holdCertification(*resident);
  const ExecutionHoldPreparation3D refreshed = supervisor.prepareHold(
      holdRequest(resident, advanced, ExecutionHoldIntent3D::kRefreshResident));
  ASSERT_TRUE(refreshed.prepared());
  EXPECT_EQ(refreshed.kind, ExecutionHoldPreparationKind3D::kTransition);
  ASSERT_NE(refreshed.transition, nullptr);
  EXPECT_EQ(refreshed.executionInput(), advanced.execution_input);
  EXPECT_EQ(refreshed.transition->next->execution_owner_epoch,
            resident->execution_owner_epoch);
  EXPECT_EQ(supervisor.plan(), resident);
}

TEST(ExecutionSupervisorHold3DTest,
     StationaryCaptureRearmIsAnExplicitRevokedOwnerTransaction) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      supervisor.authority();
  ASSERT_NE(initial_authority, nullptr);
  const std::shared_ptr<const ExecutionPlan3D> initial = initial_authority->plan();
  ASSERT_NE(initial, nullptr);
  const ExecutionRouteTransitionResult3D revocation =
      revokeExecution3D(*initial, initial->version);
  ASSERT_TRUE(revocation.applied());
  ASSERT_EQ(supervisor.commitDetachedTransition(initial_authority, revocation),
            ExecutionRoutePublicationStatus3D::kPublished);
  const std::shared_ptr<const ExecutionPlan3D> revoked = supervisor.plan();
  ASSERT_NE(revoked, nullptr);
  ASSERT_EQ(revoked->phase(), ExecutionRoutePhase3D::kRevoked);
  const std::shared_ptr<const ExecutionPlan3D> fixture_active =
      fixture.activeSnapshot();
  ASSERT_NE(fixture_active, nullptr);
  StationaryExecutionHoldCertification3D capture =
      SnapshotFixture3D::holdCertification(*fixture_active);
  capture.execution_input = stationaryCaptureInput(*capture.execution_input);
  ASSERT_NE(capture.execution_input, nullptr);
  ExecutionHoldRequest3D request =
      holdRequest(revoked, capture,
                  ExecutionHoldIntent3D::kExplicitTransferWithStationaryCaptureRearm);

  const ExecutionHoldPreparation3D prepared = supervisor.prepareHold(request);

  ASSERT_TRUE(prepared.prepared())
      << executionHoldPreparationStatus3DName(prepared.status);
  EXPECT_TRUE(prepared.stationary_capture_rearm);
  EXPECT_EQ(prepared.kind, ExecutionHoldPreparationKind3D::kTransition);
  ASSERT_NE(prepared.transition, nullptr);
  ASSERT_NE(prepared.transition->next->stationaryHold(), nullptr);
  EXPECT_EQ(prepared.transition->next->stationaryHold()->origin,
            StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm);
  EXPECT_EQ(supervisor.plan(), revoked);

  request.intent = ExecutionHoldIntent3D::kExplicitTransfer;
  const ExecutionHoldPreparation3D unnamed = supervisor.prepareHold(request);
  EXPECT_EQ(unnamed.status, ExecutionHoldPreparationStatus3D::kIntentNotApplicable);
  EXPECT_FALSE(unnamed.prepared());
  EXPECT_EQ(supervisor.plan(), revoked);

  const ExecutionOwnerIdentity3D owner = holdOwner(*prepared.transition->next, 2U);
  const ExecutionHorizonTestTransaction3D horizon_commit{
      .kind = ExecutionHorizonCommitKind3D::kTransition,
      .expected_authority = prepared.expected_authority,
      .expected_plan = prepared.expectedPlan(),
      .transition = *prepared.transition,
      .expected_pending = nullptr,
      .owner = owner,
      .input = prepared.executionInput(),
      .stationary_capture_rearm_intent = false,
  };
  EXPECT_EQ(commitExecutionHorizonForTest(supervisor, horizon_commit).status,
            ExecutionHorizonCommitStatus3D::kControlEvidenceNotCurrent);
  ExecutionHorizonTestTransaction3D named_commit = horizon_commit;
  named_commit.stationary_capture_rearm_intent = true;
  const ExecutionHorizonCommitResult3D committed =
      commitExecutionHorizonForTest(supervisor, std::move(named_commit));
  EXPECT_TRUE(committed.committed())
      << executionHorizonCommitStatus3DName(committed.status);
  EXPECT_EQ(supervisor.plan(), prepared.transition->next);
}

TEST(ExecutionSupervisorHold3DTest,
     StaticWorldSupportsTerminalTransferAndStationaryCaptureRearm) {
  SnapshotFixture3D fixture;
  const OccupancyGrid3D static_occupancy{fixture.raw_occupancy.bounds(),
                                         fixture.validated_world.esdf_fingerprint};
  const std::optional<CertifiedRouteSuffix3D> static_route =
      certifyExecutionRoute3D(staticActivation(fixture, static_occupancy));
  ASSERT_TRUE(static_route.has_value());
  const CertifiedRouteSuffix3D& certified_static_route =
      static_route.value(); // NOLINT(bugprone-unchecked-optional-access)
  ExecutionSupervisor3D active_supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installCertifiedRouteOwner(active_supervisor, certified_static_route);
  ASSERT_NE(active, nullptr);
  const StationaryExecutionHoldCertification3D terminal =
      SnapshotFixture3D::holdCertification(*active);
  ASSERT_EQ(terminal.observed_raw_world, nullptr);
  ASSERT_NE(terminal.static_world, nullptr);
  ExecutionHoldRequest3D terminal_request = holdRequest(active, terminal);
  terminal_request.current_observed_raw_world.reset();
  terminal_request.raw_world_identity_conflicted = true;

  const ExecutionHoldPreparation3D transferred =
      active_supervisor.prepareHold(terminal_request);

  ASSERT_TRUE(transferred.prepared())
      << executionHoldPreparationStatus3DName(transferred.status);
  ASSERT_NE(transferred.transition, nullptr);
  ASSERT_NE(transferred.transition->next->stationaryHold(), nullptr);
  EXPECT_EQ(transferred.transition->next->stationaryHold()->static_world,
            terminal.static_world);
  EXPECT_EQ(active_supervisor.plan(), active);

  ExecutionSupervisor3D revoked_supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      revoked_supervisor.authority();
  ASSERT_NE(initial_authority, nullptr);
  const ExecutionRouteTransitionResult3D revocation =
      revokeExecution3D(*initial_authority->plan(), initial_authority->plan()->version);
  ASSERT_TRUE(revocation.applied());
  ASSERT_EQ(revoked_supervisor.commitDetachedTransition(initial_authority, revocation),
            ExecutionRoutePublicationStatus3D::kPublished);
  const std::shared_ptr<const ExecutionPlan3D> revoked = revoked_supervisor.plan();
  ASSERT_NE(revoked, nullptr);
  StationaryExecutionHoldCertification3D capture = terminal;
  capture.execution_input = stationaryCaptureInput(*terminal.execution_input);
  ASSERT_NE(capture.execution_input, nullptr);
  ExecutionHoldRequest3D capture_request =
      holdRequest(revoked, capture,
                  ExecutionHoldIntent3D::kExplicitTransferWithStationaryCaptureRearm);
  capture_request.current_observed_raw_world.reset();
  capture_request.raw_world_identity_conflicted = true;

  const ExecutionHoldPreparation3D rearmed =
      revoked_supervisor.prepareHold(capture_request);

  ASSERT_TRUE(rearmed.prepared())
      << executionHoldPreparationStatus3DName(rearmed.status);
  ASSERT_NE(rearmed.transition, nullptr);
  ASSERT_NE(rearmed.transition->next->stationaryHold(), nullptr);
  EXPECT_EQ(rearmed.transition->next->stationaryHold()->origin,
            StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm);
  EXPECT_EQ(rearmed.transition->next->stationaryHold()->static_world,
            terminal.static_world);
  EXPECT_EQ(revoked_supervisor.plan(), revoked);
}

TEST(ExecutionSupervisorHold3DTest,
     RejectsStaleSourceLidarAndRawOwnersWithoutMutatingAuthority) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
      supervisor.authority();
  const StationaryExecutionHoldCertification3D certification =
      SnapshotFixture3D::holdCertification(*active);
  ExecutionHoldRequest3D request = holdRequest(active, certification);

  request.cycle_source_plan = makeInitialExecutionRouteSnapshot3D();
  EXPECT_EQ(supervisor.prepareHold(request).status,
            ExecutionHoldPreparationStatus3D::kSourceNotCurrent);
  request.cycle_source_plan = active;
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  request.intent = static_cast<ExecutionHoldIntent3D>(255U);
  EXPECT_EQ(supervisor.prepareHold(request).status,
            ExecutionHoldPreparationStatus3D::kIntentNotApplicable);
  request.intent = ExecutionHoldIntent3D::kExplicitTransfer;
  request.current_lidar_evidence.reset();
  EXPECT_EQ(supervisor.prepareHold(request).status,
            ExecutionHoldPreparationStatus3D::kLidarEvidenceNotCurrent);
  request.current_lidar_evidence = certification.latest_lidar_evidence;
  request.current_observed_raw_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision, nullptr,
                       SnapshotFixture3D::kRawProducer + 1U);
  EXPECT_EQ(supervisor.prepareHold(request).status,
            ExecutionHoldPreparationStatus3D::kValidationWorldUnavailable);
  EXPECT_EQ(supervisor.authority(), authority);
  EXPECT_EQ(supervisor.plan(), active);
}

TEST(ExecutionSupervisorHold3DTest, PreparedHoldCannotCommitAcrossAnAuthorityRevision) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  const ExecutionHoldPreparation3D prepared = supervisor.prepareHold(
      holdRequest(active, SnapshotFixture3D::holdCertification(*active)));
  ASSERT_TRUE(prepared.prepared());
  ASSERT_NE(prepared.transition, nullptr);
  ASSERT_TRUE(prepared.expected_authority->owner().valid);
  EXPECT_TRUE(supervisor.publishAppliedControlIfSame(
      prepared.expected_authority,
      SnapshotFixture3D::committedControl(prepared.expected_authority->owner())));

  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kTransition,
                    .expected_authority = prepared.expected_authority,
                    .expected_plan = prepared.expectedPlan(),
                    .transition = *prepared.transition,
                    .expected_pending = nullptr,
                    .owner = holdOwner(*prepared.transition->next, 2U),
                    .input = prepared.executionInput(),
                })
                .status,
            ExecutionHorizonCommitStatus3D::kAuthorityNotCurrent);
  EXPECT_EQ(supervisor.plan(), active);
}

} // namespace
} // namespace drone_city_nav
