#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/finite_motion_horizon_3d.hpp"

#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::shared_ptr<const VersionedExecutionInput3D> stationaryCaptureInput(
    const VersionedExecutionInput3D& source,
    const std::optional<MotionState3D> state_override = std::nullopt) {
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
      .state = state_override.value_or(source.state()),
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

[[nodiscard]] std::shared_ptr<const VersionedLatestLidarEvidence3D>
lidarAtAge(const VersionedLatestLidarEvidence3D& source,
           const std::int64_t validation_stamp_ns, const std::int64_t age_ns,
           const std::uint64_t identity_increment) {
  return VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
      .producer_instance_id = source.producerInstanceId(),
      .sequence = source.sequence() + identity_increment,
      .pose_generation = source.poseGeneration() + identity_increment,
      .acquisition_stamp_ns = validation_stamp_ns - age_ns,
      .receive_stamp_ns = validation_stamp_ns - age_ns,
      .source_beam_count = source.sourceBeamCount(),
      .invalid_beam_count = source.invalidBeamCount(),
      .hit_points_map_m = source.hitPointsMapM(),
  });
}

TEST(ExecutionRouteSnapshot3DTest,
     ProgressPositionComesOnlyFromTheExactExecutionInputOwner) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  RouteExecutionObservation3D poisoned = fixture.executionObservation(
      {100.0, 100.0, 100.0}, SnapshotFixture3D::kLatestRawRevision + 1U,
      &fixture.raw_occupancy);
  const std::shared_ptr<const VersionedExecutionInput3D> exact_input =
      SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0});
  ASSERT_NE(exact_input, nullptr);

  const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active), poisoned, exact_input,
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U));

  ASSERT_TRUE(advanced.applied());
  ASSERT_TRUE(advanced.next->route() != nullptr);
  EXPECT_EQ(advanced.next->route()->progress.execution_input, exact_input);
  EXPECT_DOUBLE_EQ(advanced.next->route()->progress.last_observed_position.x, 4.0);
  EXPECT_DOUBLE_EQ(advanced.next->route()->progress.last_observed_position.y, 0.0);
  EXPECT_DOUBLE_EQ(advanced.next->route()->progress.last_observed_position.z, 5.0);
}

TEST(ExecutionRouteSnapshot3DTest,
     StaticProgressOwnerChangeRequiresFiniteExecutionRevalidation) {
  SnapshotFixture3D fixture;
  const OccupancyGrid3D static_occupancy{fixture.raw_occupancy.bounds(),
                                         fixture.validated_world.esdf_fingerprint};
  const std::optional<CertifiedRouteSuffix3D> suffix =
      certifyExecutionRoute3D(staticActivation(fixture, static_occupancy));
  ASSERT_TRUE(suffix.has_value());
  const CertifiedRouteSuffix3D& certified_suffix =
      suffix.value(); // NOLINT(bugprone-unchecked-optional-access)
  const std::shared_ptr<const ExecutionPlan3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  FiniteExecutionState3D initial_execution = SnapshotFixture3D::finiteExecutionForRoute(
      *initial, certified_suffix, FiniteExecutionKind3D::kNominal, true, 100U);
  const ExecutionRouteTransitionResult3D active = activateCertifiedRoute3D(
      *initial, initial->version, certified_suffix, std::move(initial_execution));
  ASSERT_TRUE(active.applied());
  const std::shared_ptr<const VersionedExecutionInput3D> exact_input =
      SnapshotFixture3D::progressInput(*active.next, {4.0, 0.0, 5.0});

  const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
      *active.next, SnapshotFixture3D::guard(*active.next),
      fixture.executionObservation({999.0, 999.0, 999.0}, 0U, nullptr), exact_input,
      nullptr);

  ASSERT_TRUE(advanced.applied());
  ASSERT_TRUE(advanced.next->route() != nullptr);
  ASSERT_TRUE(advanced.next->finiteExecution() != nullptr);
  EXPECT_EQ(advanced.next->route()->progress.execution_input, exact_input);
  EXPECT_TRUE(advanced.next->finiteExecution()->revalidation_required);
  EXPECT_TRUE(advanced.next->valid());
}

TEST(ExecutionRouteSnapshot3DTest,
     InitialActivationBindsVehicleEvidenceAfterRevokedOwner) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const CertifiedRouteSuffix3D& certified_suffix =
      suffix.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_EQ(certified_suffix.progress.execution_input, nullptr);
  const std::shared_ptr<const ExecutionPlan3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  const ExecutionRouteTransitionResult3D revoked =
      revokeExecution3D(*initial, initial->version);
  ASSERT_TRUE(revoked.applied());
  FiniteExecutionState3D candidate_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *revoked.next, certified_suffix, FiniteExecutionKind3D::kNominal, true, 100U);
  const std::shared_ptr<const VersionedExecutionInput3D> exact_input =
      candidate_execution.execution_input;

  const ExecutionRouteTransitionResult3D activated =
      activateCertifiedRoute3D(*revoked.next, revoked.next->version, certified_suffix,
                               std::move(candidate_execution));

  ASSERT_TRUE(activated.applied());
  ASSERT_TRUE(activated.next->route() != nullptr);
  EXPECT_FALSE(activated.next->stationaryHold() != nullptr);
  EXPECT_GT(activated.next->execution_owner_epoch, revoked.next->execution_owner_epoch);
  EXPECT_EQ(activated.next->route()->progress.execution_input, exact_input);
  EXPECT_DOUBLE_EQ(activated.next->route()->progress.last_observed_position.x, 2.0);
  EXPECT_TRUE(activated.next->valid());
}

TEST(ExecutionRouteSnapshot3DTest,
     HoldTransferRequiresReachedCertifiedTerminalRestAndPreservesLineage) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(active->finiteExecution() != nullptr);
  StationaryExecutionHoldCertification3D moving =
      SnapshotFixture3D::holdCertification(*active, false);
  EXPECT_EQ(
      transferToExecutionHold3D(*active, active->version, std::move(moving)).status,
      ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  const FiniteExecutionState3D& resident_execution = *active->finiteExecution();
  const FiniteMotionHorizon3D& resident_horizon = *resident_execution.horizon;
  std::size_t first_rest_state_index = 0U;
  while (
      first_rest_state_index < resident_horizon.states.size() &&
      !finiteMotionHorizonRestsFromState3D(resident_horizon, first_rest_state_index,
                                           kStationaryExecutionHoldPositionToleranceM,
                                           kStationaryExecutionHoldSpeedToleranceMps)) {
    ++first_rest_state_index;
  }
  ASSERT_GT(first_rest_state_index, 1U);
  ASSERT_LT(first_rest_state_index + 1U, resident_horizon.states.size());
  const auto lease_stamp_at = [&](const std::size_t state_index) {
    return resident_execution.valid_from_ns +
           static_cast<std::int64_t>(state_index) *
               resident_execution.control_interval_ns;
  };
  StationaryExecutionHoldCertification3D early = SnapshotFixture3D::holdCertification(
      *active, true, std::nullopt, lease_stamp_at(first_rest_state_index - 1U));
  EXPECT_EQ(
      transferToExecutionHold3D(*active, active->version, std::move(early)).status,
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  StationaryExecutionHoldCertification3D resting = SnapshotFixture3D::holdCertification(
      *active, true, std::nullopt, lease_stamp_at(first_rest_state_index));
  const ExecutionRouteTransitionResult3D rested_early =
      transferToExecutionHold3D(*active, active->version, std::move(resting));
  EXPECT_EQ(rested_early.status, ExecutionRouteTransitionStatus3D::kApplied);
  ASSERT_NE(rested_early.next, nullptr);
  EXPECT_TRUE(rested_early.next->stationaryHold() != nullptr);
  EXPECT_FALSE(rested_early.next->finiteExecution() != nullptr);

  const MotionState3D& terminal = active->finiteExecution()->horizon->states.back();
  const Point3 discontinuous_position{terminal.x + 1.0, terminal.y, terminal.z};
  StationaryExecutionHoldCertification3D discontinuous =
      SnapshotFixture3D::holdCertification(*active, true, discontinuous_position);
  EXPECT_EQ(
      transferToExecutionHold3D(*active, active->version, std::move(discontinuous))
          .status,
      ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  StationaryExecutionHoldCertification3D certification =
      SnapshotFixture3D::holdCertification(
          *active, true, std::nullopt, std::nullopt, 0.1F,
          MotionControl3D{.ax = 0.05F, .yaw_accel = 0.02F});
  const Point3 hold_position = certification.position;

  const ExecutionRouteTransitionResult3D transferred =
      transferToExecutionHold3D(*active, active->version, std::move(certification));

  ASSERT_TRUE(transferred.applied());
  ASSERT_NE(transferred.next, nullptr);
  EXPECT_EQ(transferred.next->version, active->version + 1U);
  EXPECT_EQ(transferred.next->phase(), ExecutionRoutePhase3D::kStopped);
  EXPECT_FALSE(transferred.next->route() != nullptr);
  EXPECT_FALSE(transferred.next->finiteExecution() != nullptr);
  EXPECT_FALSE(transferred.next->directTrackingExecution() != nullptr);
  ASSERT_TRUE(transferred.next->stationaryHold() != nullptr);
  EXPECT_DOUBLE_EQ(transferred.next->stationaryHold()->position.x, hold_position.x);
  EXPECT_DOUBLE_EQ(transferred.next->stationaryHold()->position.y, hold_position.y);
  EXPECT_DOUBLE_EQ(transferred.next->stationaryHold()->position.z, hold_position.z);
  EXPECT_EQ(transferred.next->stationaryHold()->hold_id,
            transferred.next->execution_owner_epoch);
  EXPECT_EQ(transferred.next->stationaryHold()->origin,
            StationaryExecutionHoldOrigin3D::kTerminalExecution);
  EXPECT_EQ(transferred.next->routeGenerationHighWater(),
            active->routeGenerationHighWater());
  EXPECT_TRUE(transferred.next->valid());

  StationaryExecutionHoldCertification3D stale_certification =
      SnapshotFixture3D::holdCertification(*transferred.next);
  const ExecutionRouteTransitionResult3D stale = transferToExecutionHold3D(
      *transferred.next, active->version, std::move(stale_certification));
  EXPECT_EQ(stale.status, ExecutionRouteTransitionStatus3D::kStaleSnapshotVersion);
  EXPECT_EQ(stale.next, nullptr);
}

TEST(ExecutionRouteSnapshot3DTest,
     HoldTransferRequiresLidarFreshAtTheTerminalCertificationStamp) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->finiteExecution() != nullptr);
  StationaryExecutionHoldCertification3D boundary =
      SnapshotFixture3D::holdCertification(*active);
  ASSERT_NE(boundary.execution_input, nullptr);
  ASSERT_NE(boundary.validation_policy, nullptr);
  ASSERT_NE(active->finiteExecution()->latest_lidar_evidence, nullptr);
  const std::int64_t maximum_age_ns = static_cast<std::int64_t>(
      boundary.validation_policy->latestLidarMaximumAgeMs() * 1.0e6);
  ASSERT_GT(maximum_age_ns, 0);
  boundary.latest_lidar_evidence =
      lidarAtAge(*active->finiteExecution()->latest_lidar_evidence,
                 boundary.execution_input->effectiveStampNs(), maximum_age_ns, 1U);
  ASSERT_NE(boundary.latest_lidar_evidence, nullptr);
  EXPECT_TRUE(transferToExecutionHold3D(*active, active->version, boundary).applied());

  StationaryExecutionHoldCertification3D expired =
      SnapshotFixture3D::holdCertification(*active);
  expired.latest_lidar_evidence =
      lidarAtAge(*active->finiteExecution()->latest_lidar_evidence,
                 expired.execution_input->effectiveStampNs(), maximum_age_ns + 1, 2U);
  ASSERT_NE(expired.latest_lidar_evidence, nullptr);
  EXPECT_EQ(
      transferToExecutionHold3D(*active, active->version, std::move(expired)).status,
      ExecutionRouteTransitionStatus3D::kInvalidCandidate);
}

TEST(ExecutionRouteSnapshot3DTest,
     StationaryCaptureRearmIsNamedRevokedOnlyAndPreservesPhysicalContinuity) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  StationaryExecutionHoldCertification3D terminal =
      SnapshotFixture3D::holdCertification(*active);
  ASSERT_NE(terminal.execution_input, nullptr);
  const ExecutionRouteTransitionResult3D revoked =
      revokeExecution3D(*active, active->version);
  ASSERT_TRUE(revoked.applied());

  EXPECT_EQ(
      transferToExecutionHold3D(*revoked.next, revoked.next->version, terminal).status,
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  EXPECT_EQ(
      armStationaryCaptureHold3D(*revoked.next, revoked.next->version, terminal).status,
      ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  terminal.execution_input = stationaryCaptureInput(*terminal.execution_input);
  ASSERT_NE(terminal.execution_input, nullptr);
  const ExecutionRouteTransitionResult3D rearmed =
      armStationaryCaptureHold3D(*revoked.next, revoked.next->version, terminal);
  ASSERT_TRUE(rearmed.applied());
  ASSERT_TRUE(rearmed.next->stationaryHold() != nullptr);
  EXPECT_EQ(rearmed.next->phase(), ExecutionRoutePhase3D::kStopped);
  EXPECT_EQ(rearmed.next->stationaryHold()->origin,
            StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm);
  EXPECT_EQ(rearmed.next->stationaryHold()->source_trajectory_revision, 0U);
  EXPECT_EQ(rearmed.next->stationaryHold()->hold_id,
            rearmed.next->execution_owner_epoch);
  EXPECT_TRUE(rearmed.next->valid());

  StationaryExecutionHoldCertification3D moving = terminal;
  MotionState3D moving_state = terminal.execution_input->state();
  moving_state.vx =
      static_cast<float>(kStationaryExecutionHoldSpeedToleranceMps + 0.01);
  moving.execution_input =
      stationaryCaptureInput(*terminal.execution_input, moving_state);
  ASSERT_NE(moving.execution_input, nullptr);
  EXPECT_EQ(armStationaryCaptureHold3D(*revoked.next, revoked.next->version,
                                       std::move(moving))
                .status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  // A return two metres away is an obstacle for a hold there; a return at the
  // vehicle's own pose is contact and leaves the hold certifiable.
  StationaryExecutionHoldCertification3D raw_unsafe = terminal;
  ASSERT_NE(raw_unsafe.latest_lidar_evidence, nullptr);
  const Point3 remote_hit{raw_unsafe.position.x - 2.0, raw_unsafe.position.y,
                          raw_unsafe.position.z};
  raw_unsafe.position = remote_hit;
  raw_unsafe.latest_lidar_evidence =
      VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
          .producer_instance_id =
              raw_unsafe.latest_lidar_evidence->producerInstanceId(),
          .sequence = raw_unsafe.latest_lidar_evidence->sequence() + 1U,
          .pose_generation = raw_unsafe.latest_lidar_evidence->poseGeneration() + 1U,
          .acquisition_stamp_ns = raw_unsafe.execution_input->effectiveStampNs(),
          .receive_stamp_ns = raw_unsafe.execution_input->effectiveStampNs(),
          .source_beam_count = 1U,
          .hit_points_map_m = {remote_hit},
      });
  ASSERT_NE(raw_unsafe.latest_lidar_evidence, nullptr);
  EXPECT_EQ(armStationaryCaptureHold3D(*revoked.next, revoked.next->version,
                                       std::move(raw_unsafe))
                .status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  StationaryExecutionHoldCertification3D jumped = terminal;
  jumped.position.x += kStationaryExecutionHoldPositionToleranceM + 0.01;
  EXPECT_EQ(armStationaryCaptureHold3D(*revoked.next, revoked.next->version,
                                       std::move(jumped))
                .status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  EXPECT_EQ(
      armStationaryCaptureHold3D(*active, active->version, std::move(terminal)).status,
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

TEST(ExecutionRouteSnapshot3DTest,
     HoldTransferReplacesDirectOwnerAndRefreshesInputBeforeExactReplay) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> route_owner = fixture.activeSnapshot();
  ASSERT_NE(route_owner, nullptr);
  ASSERT_TRUE(route_owner->route() != nullptr);
  const DirectTrackingOwnerIdentity3D identity{
      .mission_epoch = fixture.objective.mission_epoch,
      .assignment_generation = fixture.objective.assignment_generation,
      .target_detection_id = 41U,
      .target_track_id = 42U,
      .objective_sample_sequence = fixture.objective.sample_sequence,
      .line_of_sight_generation = 7U,
  };
  const std::optional<DirectTrackingFiniteExecution3D> direct_execution =
      certifyDirectFixtureExecution(*route_owner, *route_owner->route(), identity,
                                    101U);
  ASSERT_TRUE(direct_execution.has_value());
  const DirectTrackingFiniteExecution3D& certified_direct =
      direct_execution.value(); // NOLINT(bugprone-unchecked-optional-access)
  const ExecutionRouteTransitionResult3D direct =
      transferToDirectTracking3D(*route_owner, route_owner->version, certified_direct);
  ASSERT_TRUE(direct.applied());
  StationaryExecutionHoldCertification3D certification =
      SnapshotFixture3D::holdCertification(*direct.next);

  const ExecutionRouteTransitionResult3D transferred = transferToExecutionHold3D(
      *direct.next, direct.next->version, std::move(certification));

  ASSERT_TRUE(transferred.applied());
  ASSERT_NE(transferred.next, nullptr);
  EXPECT_FALSE(transferred.next->route() != nullptr);
  EXPECT_FALSE(transferred.next->finiteExecution() != nullptr);
  EXPECT_FALSE(transferred.next->directTrackingExecution() != nullptr);
  EXPECT_EQ(transferred.next->routeGenerationHighWater(),
            route_owner->routeGenerationHighWater());
  ASSERT_TRUE(transferred.next->stationaryHold() != nullptr);
  const ExecutionRouteTransitionResult3D refreshed = transferToExecutionHold3D(
      *transferred.next, transferred.next->version,
      SnapshotFixture3D::holdCertification(*transferred.next));
  ASSERT_TRUE(refreshed.applied());
  ASSERT_NE(refreshed.next, nullptr);
  ASSERT_NE(refreshed.next->stationaryHold(), nullptr);
  const StationaryExecutionHold3D& refreshed_hold = *refreshed.next->stationaryHold();
  EXPECT_EQ(refreshed_hold.hold_id, transferred.next->stationaryHold()->hold_id);
  EXPECT_EQ(refreshed.next->execution_owner_epoch,
            transferred.next->execution_owner_epoch);
  EXPECT_EQ(transferToExecutionHold3D(
                *refreshed.next, refreshed.next->version,
                StationaryExecutionHoldCertification3D{
                    .position = refreshed_hold.position,
                    .execution_input = refreshed_hold.terminal_execution_input,
                    .observed_raw_world = refreshed_hold.observed_raw_world,
                    .static_world = refreshed_hold.static_world,
                    .validation_policy = refreshed_hold.validation_policy,
                    .latest_lidar_evidence = refreshed_hold.latest_lidar_evidence,
                })
                .status,
            ExecutionRouteTransitionStatus3D::kNoChange);

  StationaryExecutionHoldCertification3D shifted_replay{
      .position = refreshed_hold.position,
      .execution_input = refreshed_hold.terminal_execution_input,
      .observed_raw_world = refreshed_hold.observed_raw_world,
      .static_world = refreshed_hold.static_world,
      .validation_policy = refreshed_hold.validation_policy,
      .latest_lidar_evidence = refreshed_hold.latest_lidar_evidence,
  };
  shifted_replay.position.x += 0.5e-6;
  const ExecutionRouteTransitionResult3D shifted = transferToExecutionHold3D(
      *refreshed.next, refreshed.next->version, std::move(shifted_replay));
  ASSERT_TRUE(shifted.applied());
  ASSERT_NE(shifted.next, nullptr);
  ASSERT_NE(shifted.next->stationaryHold(), nullptr);
  EXPECT_EQ(shifted.next->stationaryHold()->position.x,
            refreshed_hold.position.x + 0.5e-6);
}

TEST(ExecutionRouteSnapshot3DTest,
     HoldOwnerReplacementRequiresContinuousNewerEvidenceAndDedicatedTransitions) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> route_owner = fixture.activeSnapshot();
  ASSERT_NE(route_owner, nullptr);
  ASSERT_TRUE(route_owner->route() != nullptr);
  const ExecutionRouteTransitionResult3D held =
      transferToExecutionHold3D(*route_owner, route_owner->version,
                                SnapshotFixture3D::holdCertification(*route_owner));
  ASSERT_TRUE(held.applied());
  ASSERT_TRUE(held.next->stationaryHold() != nullptr);

  const DirectTrackingOwnerIdentity3D identity{
      .mission_epoch = fixture.objective.mission_epoch,
      .assignment_generation = fixture.objective.assignment_generation,
      .target_detection_id = 51U,
      .target_track_id = 52U,
      .objective_sample_sequence = fixture.objective.sample_sequence,
      .line_of_sight_generation = 8U,
  };
  const std::optional<CertifiedRouteSuffix3D> unbound_route = fixture.certify();
  ASSERT_TRUE(unbound_route.has_value());
  const CertifiedRouteSuffix3D& certified_unbound_route =
      unbound_route.value(); // NOLINT(bugprone-unchecked-optional-access)
  FiniteExecutionCertification3D discontinuous_finite =
      SnapshotFixture3D::finiteCertificationForRoute(
          certified_unbound_route, FiniteExecutionKind3D::kNominal, 101U, 101U, 0U,
          -1.0, held.next->stationaryHold()->terminal_execution_input.get());
  const std::optional<DirectTrackingFiniteExecution3D> discontinuous_direct =
      certifyDirectTrackingExecution3D(
          *held.next,
          DirectTrackingExecutionCertification3D{
              .identity = identity,
              .trajectory_revision = 101U,
              .target = {10.0, 0.0, 5.0},
              .horizon = std::move(discontinuous_finite.horizon),
              .observed_raw_world = certified_unbound_route.observed_raw_world,
              .static_world = certified_unbound_route.static_world,
              .validation_policy = certified_unbound_route.validation_policy,
              .execution_input = std::move(discontinuous_finite.execution_input),
              .latest_lidar_evidence =
                  std::move(discontinuous_finite.latest_lidar_evidence),
              .valid_from_ns = discontinuous_finite.valid_from_ns,
              .kind = FiniteExecutionKind3D::kNominal,
          });
  ASSERT_TRUE(discontinuous_direct.has_value());
  const DirectTrackingFiniteExecution3D& certified_discontinuous_direct =
      discontinuous_direct.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_EQ(transferToDirectTracking3D(*held.next, held.next->version,
                                       certified_discontinuous_direct)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  const std::optional<DirectTrackingFiniteExecution3D> continuous_direct =
      certifyDirectFixtureExecution(*held.next, *route_owner->route(), identity, 102U);
  ASSERT_TRUE(continuous_direct.has_value());
  const DirectTrackingFiniteExecution3D& certified_continuous_direct =
      continuous_direct.value(); // NOLINT(bugprone-unchecked-optional-access)
  const ExecutionRouteTransitionResult3D direct = transferToDirectTracking3D(
      *held.next, held.next->version, certified_continuous_direct);
  ASSERT_TRUE(direct.applied()) << executionRouteTransitionStatus3DName(direct.status);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = direct.next->routeGenerationHighWater() + 1U;
  successor_activation =
      rebindUnconstrainedDecorations(std::move(successor_activation));
  successor_activation.observation.position = Point3{
      direct.next->directTrackingExecution()->execution_input->state().x,
      direct.next->directTrackingExecution()->execution_input->state().y,
      direct.next->directTrackingExecution()->execution_input->state().z,
  };
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const CertifiedRouteSuffix3D& certified_successor =
      successor.value(); // NOLINT(bugprone-unchecked-optional-access)
  FiniteExecutionState3D successor_execution =
      SnapshotFixture3D::finiteExecutionForRoute(*direct.next, certified_successor,
                                                 FiniteExecutionKind3D::kNominal, true,
                                                 103U);
  EXPECT_EQ(activateCertifiedRoute3D(*direct.next, direct.next->version,
                                     certified_successor,
                                     std::move(successor_execution))
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

TEST(ExecutionRouteSnapshot3DTest,
     HoldCertificationRejectsRawAndLatestLidarCollisions) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);

  // A hold away from the vehicle is rejected when raw evidence blocks it: the
  // body does not stand there, so that evidence is an obstacle.
  const StationaryExecutionHoldCertification3D reference =
      SnapshotFixture3D::holdCertification(*active);
  const Point3 remote_position{reference.position.x - 2.0, reference.position.y,
                               reference.position.z};
  StationaryExecutionHoldCertification3D raw_blocked =
      SnapshotFixture3D::holdCertification(*active, true, remote_position);
  ObservedOccupancyGrid3D blocked_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> blocked_cell =
      blocked_occupancy.worldToCell(raw_blocked.position);
  ASSERT_TRUE(blocked_cell.has_value());
  ASSERT_TRUE(blocked_occupancy.setState(
      blocked_cell.value(), // NOLINT(bugprone-unchecked-optional-access)
      ObservedVoxelState::kOccupied));
  raw_blocked.observed_raw_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &blocked_occupancy);
  EXPECT_EQ(transferToExecutionHold3D(*active, active->version, std::move(raw_blocked))
                .status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  StationaryExecutionHoldCertification3D lidar_blocked =
      SnapshotFixture3D::holdCertification(*active, true, remote_position);
  lidar_blocked.latest_lidar_evidence = SnapshotFixture3D::newerLidarEvidence(
      *lidar_blocked.latest_lidar_evidence,
      std::vector<Point3>{lidar_blocked.position});
  ASSERT_NE(lidar_blocked.latest_lidar_evidence, nullptr);
  EXPECT_EQ(
      transferToExecutionHold3D(*active, active->version, std::move(lidar_blocked))
          .status,
      ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  // Evidence at the vehicle's own pose is contact: holding there stays
  // certifiable, so a vehicle standing in fresh evidence keeps a commanded
  // stop instead of losing every executable option.
  StationaryExecutionHoldCertification3D contact =
      SnapshotFixture3D::holdCertification(*active);
  ObservedOccupancyGrid3D contact_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> contact_cell =
      contact_occupancy.worldToCell(contact.position);
  ASSERT_TRUE(contact_cell.has_value());
  ASSERT_TRUE(contact_occupancy.setState(
      contact_cell.value(), // NOLINT(bugprone-unchecked-optional-access)
      ObservedVoxelState::kOccupied));
  contact.observed_raw_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &contact_occupancy);
  EXPECT_EQ(
      transferToExecutionHold3D(*active, active->version, std::move(contact)).status,
      ExecutionRouteTransitionStatus3D::kApplied);
}

TEST(ExecutionRouteSnapshot3DTest,
     HoldEvidenceRefreshPreservesStableOwnerIdentityAndTarget) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  const ExecutionRouteTransitionResult3D held = transferToExecutionHold3D(
      *active, active->version, SnapshotFixture3D::holdCertification(*active));
  ASSERT_TRUE(held.applied());
  ASSERT_TRUE(held.next->stationaryHold() != nullptr);

  StationaryExecutionHoldCertification3D refreshed =
      SnapshotFixture3D::holdCertification(*held.next);
  refreshed.latest_lidar_evidence =
      SnapshotFixture3D::newerLidarEvidence(*refreshed.latest_lidar_evidence);
  ASSERT_NE(refreshed.latest_lidar_evidence, nullptr);
  const ExecutionRouteTransitionResult3D updated =
      transferToExecutionHold3D(*held.next, held.next->version, std::move(refreshed));

  ASSERT_TRUE(updated.applied());
  ASSERT_TRUE(updated.next->stationaryHold() != nullptr);
  EXPECT_EQ(updated.next->version, held.next->version + 1U);
  EXPECT_EQ(updated.next->execution_owner_epoch, held.next->execution_owner_epoch);
  EXPECT_EQ(updated.next->stationaryHold()->hold_id,
            held.next->stationaryHold()->hold_id);
  EXPECT_EQ(updated.next->stationaryHold()->position.x,
            held.next->stationaryHold()->position.x);
  EXPECT_EQ(updated.next->stationaryHold()->position.y,
            held.next->stationaryHold()->position.y);
  EXPECT_EQ(updated.next->stationaryHold()->position.z,
            held.next->stationaryHold()->position.z);
}

TEST(ExecutionRouteSnapshot3DTest,
     EmptyOwnerCannotInventAStationaryHoldAndRevocationIsIdempotent) {
  const std::shared_ptr<const ExecutionPlan3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  EXPECT_THROW(SnapshotFixture3D::holdCertification(*initial), std::logic_error);

  const ExecutionRouteTransitionResult3D revoked =
      revokeExecution3D(*initial, initial->version);
  ASSERT_TRUE(revoked.applied());
  EXPECT_EQ(revoked.next->phase(), ExecutionRoutePhase3D::kRevoked);
  EXPECT_EQ(revoked.next->routeGenerationHighWater(), 0U);
  EXPECT_EQ(revoked.next->execution_owner_epoch, initial->execution_owner_epoch + 1U);
  EXPECT_EQ(revokeExecution3D(*revoked.next, revoked.next->version).status,
            ExecutionRouteTransitionStatus3D::kNoChange);
  EXPECT_EQ(revokeExecution3D(*revoked.next, initial->version).status,
            ExecutionRouteTransitionStatus3D::kStaleSnapshotVersion);

  ExecutionPlan3D exhausted = *revoked.next;
  exhausted.version = std::numeric_limits<std::uint64_t>::max();
  ASSERT_TRUE(exhausted.valid());
  EXPECT_EQ(revokeExecution3D(exhausted, exhausted.version).status,
            ExecutionRouteTransitionStatus3D::kVersionExhausted);
}

TEST(ExecutionRouteSnapshot3DTest,
     ProgressCanAdvanceWhileAnEarlierFiniteHorizonRemainsTheOwner) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  const ExecutionRouteTransitionResult3D following =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(*active));
  ASSERT_TRUE(following.applied());

  const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
      *following.next, SnapshotFixture3D::guard(*following.next),
      fixture.executionObservation({4.0, 0.0, 5.0},
                                   SnapshotFixture3D::kLatestRawRevision + 1U,
                                   &fixture.raw_occupancy),
      SnapshotFixture3D::progressInput(*following.next, {4.0, 0.0, 5.0}),
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U));

  ASSERT_TRUE(advanced.applied());
  ASSERT_TRUE(advanced.next->finiteExecution() != nullptr);
  EXPECT_TRUE(advanced.next->finiteExecution()->revalidation_required);
  EXPECT_DOUBLE_EQ(advanced.next->finiteExecution()->begin_route_station_m, 2.0);
  EXPECT_DOUBLE_EQ(advanced.next->route()->progress.station_m, 4.0);
  EXPECT_TRUE(advanced.next->valid());
}

TEST(ExecutionRouteSnapshot3DTest, GeometryValidationReportsTypedTerminalFailure) {
  const std::vector<RouteSample3D> route{RouteSample3D{.position = {0.0, 0.0, 5.0},
                                                       .tangent = {1.0, 0.0, 0.0},
                                                       .station_m = 0.0,
                                                       .reference_speed_mps = 4.0},
                                         RouteSample3D{.position = {4.0, 0.0, 5.0},
                                                       .tangent = {-1.0, 0.0, 0.0},
                                                       .station_m = 4.0,
                                                       .reference_speed_mps = 4.0}};

  const CompiledTrajectoryValidation3D validation =
      validateCompiledTrajectorySamples3D(route);

  EXPECT_FALSE(validation.valid());
  EXPECT_EQ(validation.reason,
            CompiledTrajectoryFailureReason3D::kTerminalTangentMismatch);
  EXPECT_EQ(validation.sample_index, 1U);
  EXPECT_STREQ(compiledTrajectoryFailureReason3DName(validation.reason),
               "terminal_tangent_mismatch");
}

TEST(ExecutionRouteSnapshot3DTest,
     ExecutionAssessmentRejectsWrongRawLineageAndANewRawCollision) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  RouteExecutionObservation3D wrong_producer = fixture.executionObservation(
      {4.0, 0.0, 5.0}, SnapshotFixture3D::kLatestRawRevision + 1U,
      &fixture.raw_occupancy);
  ++wrong_producer.latest_raw_producer_instance_id;
  EXPECT_EQ(advanceCertifiedRoute3D(
                *active, SnapshotFixture3D::guard(*active), wrong_producer,
                SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0}),
                fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, nullptr,
                                 SnapshotFixture3D::kRawProducer + 1U))
                .status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  ASSERT_TRUE(
      fixture.raw_occupancy.setState({12, 5, 5}, ObservedVoxelState::kOccupied));
  EXPECT_EQ(advanceCertifiedRoute3D(
                *active, SnapshotFixture3D::guard(*active),
                fixture.executionObservation({4.0, 0.0, 5.0},
                                             SnapshotFixture3D::kLatestRawRevision + 1U,
                                             &fixture.raw_occupancy),
                SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0}),
                fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U,
                                 &fixture.raw_occupancy))
                .status,
            ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected);
}

TEST(ExecutionRouteSnapshot3DTest,
     SnapshotOutlivesAllMutableActivationAndValidationInputs) {
  std::shared_ptr<const ExecutionPlan3D> survivor;
  {
    SnapshotFixture3D fixture;
    const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
    ASSERT_TRUE(active);
    const ExecutionRouteTransitionResult3D following =
        replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                                 SnapshotFixture3D::finiteExecution(*active));
    ASSERT_TRUE(following.applied());
    survivor = following.next;
  }

  ASSERT_TRUE(survivor);
  EXPECT_TRUE(survivor->valid());
  ASSERT_TRUE(survivor->route() != nullptr);
  ASSERT_TRUE(survivor->finiteExecution() != nullptr);
  EXPECT_EQ(survivor->route()->geometry->route->size(), 3U);
  EXPECT_EQ(survivor->finiteExecution()->horizon->states.size(),
            survivor->finiteExecution()->horizon->controls.size() + 1U);
  EXPECT_FLOAT_EQ(
      survivor->finiteExecution()->horizon->states.back().x,
      static_cast<float>(survivor->finiteExecution()->stop_boundary.position.x));
}

TEST(ExecutionRouteSnapshot3DTest, GuardsAndStoreEnforceVersionedCasPublication) {
  static_assert(
      std::is_const_v<std::remove_reference_t<
          decltype(std::declval<ExecutionRouteTransitionResult3D&>().status)>>);
  static_assert(std::is_const_v<std::remove_reference_t<
                    decltype(std::declval<ExecutionRouteTransitionResult3D&>().next)>>);
  SnapshotFixture3D fixture;
  RouteExecutionManager3D manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> initial = manager.plan();
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(initial);
  ASSERT_TRUE(suffix.has_value());
  const CertifiedRouteSuffix3D& certified_suffix =
      suffix.value(); // NOLINT(bugprone-unchecked-optional-access)
  FiniteExecutionState3D initial_execution = SnapshotFixture3D::finiteExecutionForRoute(
      *initial, certified_suffix, FiniteExecutionKind3D::kNominal, true, 100U);

  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *initial, initial->version, certified_suffix, std::move(initial_execution));
  ASSERT_TRUE(activation.applied());
  ASSERT_TRUE(activation.next);
  EXPECT_EQ(manager.publishDetachedTransition(initial_authority, activation),
            ExecutionRoutePublicationStatus3D::kPublished);
  EXPECT_EQ(manager.publishDetachedTransition(initial_authority, activation),
            ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion);
  EXPECT_EQ(manager.plan(), activation.next);

  RouteExecutionManager3D other_manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> other_authority =
      other_manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> other_initial = other_manager.plan();
  ASSERT_TRUE(other_initial);
  EXPECT_EQ(other_initial->version, initial->version);
  EXPECT_EQ(other_manager.publishDetachedTransition(other_authority, activation),
            ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion);

  const std::shared_ptr<const CommittedExecutionAuthority3D> resident_authority =
      manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> resident = manager.plan();
  ASSERT_EQ(resident, activation.next);
  const ExecutionRouteTransitionResult3D first_replacement = replaceFiniteExecution3D(
      *resident, SnapshotFixture3D::guard(*resident),
      SnapshotFixture3D::finiteExecution(*resident, FiniteExecutionKind3D::kNominal,
                                         true, 101U));
  const ExecutionRouteTransitionResult3D competing_replacement =
      replaceFiniteExecution3D(
          *resident, SnapshotFixture3D::guard(*resident),
          SnapshotFixture3D::finiteExecution(*resident, FiniteExecutionKind3D::kNominal,
                                             true, 102U));
  ASSERT_TRUE(first_replacement.applied());
  ASSERT_TRUE(competing_replacement.applied());
  EXPECT_EQ(manager.publishDetachedTransition(resident_authority, first_replacement),
            ExecutionRoutePublicationStatus3D::kPublished);
  EXPECT_EQ(
      manager.publishDetachedTransition(resident_authority, competing_replacement),
      ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion);

  FiniteExecutionState3D finite = SnapshotFixture3D::finiteExecution(*activation.next);
  ExecutionRouteTransitionGuard3D stale_guard =
      SnapshotFixture3D::guard(*activation.next);
  --stale_guard.expected_snapshot_version;
  EXPECT_EQ(replaceFiniteExecution3D(*activation.next, stale_guard, finite).status,
            ExecutionRouteTransitionStatus3D::kStaleSnapshotVersion);

  ExecutionRouteTransitionGuard3D generation_guard =
      SnapshotFixture3D::guard(*activation.next);
  ++generation_guard.expected_route_generation;
  EXPECT_EQ(replaceFiniteExecution3D(*activation.next, generation_guard, finite).status,
            ExecutionRouteTransitionStatus3D::kRouteGenerationMismatch);

  ExecutionRouteTransitionGuard3D geometry_guard =
      SnapshotFixture3D::guard(*activation.next);
  ++geometry_guard.expected_geometry_revision;
  EXPECT_EQ(replaceFiniteExecution3D(*activation.next, geometry_guard, finite).status,
            ExecutionRouteTransitionStatus3D::kGeometryRevisionMismatch);

  EXPECT_EQ(manager.publishDetachedTransition(manager.authority(),
                                              ExecutionRouteTransitionResult3D{}),
            ExecutionRoutePublicationStatus3D::kInvalidCandidate);
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteExecutionRequiresBoundTerminalRestAndCertifiedStopBoundary) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  FiniteExecutionState3D finite = SnapshotFixture3D::finiteExecution(*active);

  const ExecutionRouteTransitionResult3D accepted =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active), finite);
  ASSERT_TRUE(accepted.applied());
  ASSERT_TRUE(accepted.next);
  EXPECT_TRUE(accepted.next->valid());
  ASSERT_TRUE(accepted.next->finiteExecution() != nullptr);
  EXPECT_EQ(accepted.next->finiteExecution()->source_route_generation,
            active->route()->identity.generation);
  EXPECT_TRUE(finiteMotionHorizonHasTerminalRestState3D(
      *accepted.next->finiteExecution()->horizon));

  FiniteExecutionState3D wrong_route = finite;
  ++wrong_route.source_route_generation;
  EXPECT_EQ(
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active), wrong_route)
          .status,
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  FiniteExecutionState3D moving_terminal = SnapshotFixture3D::finiteExecution(
      *active, FiniteExecutionKind3D::kNominal, false);
  EXPECT_EQ(replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                                     moving_terminal)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  FiniteExecutionState3D mismatched_stop = finite;
  ++mismatched_stop.stop_boundary.trajectory_revision;
  EXPECT_EQ(replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                                     mismatched_stop)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  FiniteExecutionState3D inconsistent_counts = finite;
  auto inconsistent_horizon = std::make_shared<FiniteMotionHorizon3D>(*finite.horizon);
  ++inconsistent_horizon->arrival_control_count;
  inconsistent_counts.horizon = std::move(inconsistent_horizon);
  EXPECT_EQ(replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                                     inconsistent_counts)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  FiniteExecutionState3D inconsistent_interval = finite;
  ++inconsistent_interval.valid_until_ns;
  EXPECT_EQ(replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                                     inconsistent_interval)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

TEST(ExecutionRouteSnapshot3DTest,
     FollowingAcceptsARecertifiedRetainedOwnerButNotAnEmergencyTail) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);

  const ExecutionRouteTransitionResult3D retained = replaceFiniteExecution3D(
      *active, SnapshotFixture3D::guard(*active),
      SnapshotFixture3D::finiteExecution(*active, FiniteExecutionKind3D::kRetained,
                                         true, 101U));
  ASSERT_TRUE(retained.applied());
  ASSERT_NE(retained.next, nullptr);
  EXPECT_TRUE(retained.next->valid());
  EXPECT_EQ(retained.next->phase(), ExecutionRoutePhase3D::kFollowing);
  ASSERT_TRUE(retained.next->finiteExecution() != nullptr);
  EXPECT_EQ(retained.next->finiteExecution()->kind, FiniteExecutionKind3D::kRetained);

  const FiniteExecutionState3D emergency = SnapshotFixture3D::finiteExecution(
      *active, FiniteExecutionKind3D::kEmergencyBrakeTail, true, 101U);
  EXPECT_EQ(
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active), emergency)
          .status,
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

TEST(ExecutionRouteSnapshot3DTest, RejectsUnknownFiniteAndLifecycleEnumValues) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ASSERT_TRUE(active->route() != nullptr);

  FiniteExecutionState3D unknown_execution =
      SnapshotFixture3D::finiteExecution(*active);
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  unknown_execution.kind = static_cast<FiniteExecutionKind3D>(255U);
  EXPECT_FALSE(unknown_execution.validFor(&*active->route()));
  EXPECT_EQ(replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                                     unknown_execution)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  const RouteLifecycleEvent3D unknown_event{
      // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
      .kind = static_cast<RouteLifecycleEventKind3D>(255U),
      .generation = active->route()->identity.generation,
  };
  EXPECT_EQ(retireCertifiedRoute3D(*active, SnapshotFixture3D::guard(*active),
                                   unknown_event, std::nullopt)
                .status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteReplacementEnforcesCompleteInputAndLidarIdentityMonotonicity) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(active->finiteExecution() != nullptr);
  const FiniteExecutionState3D& resident = *active->finiteExecution();
  ASSERT_NE(resident.execution_input, nullptr);
  ASSERT_NE(resident.latest_lidar_evidence, nullptr);

  const auto input_capture = [&](const std::uint64_t capture_sequence,
                                 const MotionControl3D control) {
    return VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
        .capture_sequence = capture_sequence,
        .pose_revision = resident.execution_input->poseRevision(),
        .pose_source_timestamp_us = resident.execution_input->poseSourceTimestampUs(),
        .pose_receive_stamp_ns = resident.execution_input->poseReceiveStampNs(),
        .effective_stamp_ns = resident.execution_input->effectiveStampNs(),
        .state = resident.execution_input->state(),
        .full_state_authoritative = true,
        .state_provenance = resident.execution_input->stateProvenance(),
        .previous_control = control,
        .previous_control_source = resident.execution_input->previousControlSource(),
        .previous_control_source_producer_instance_id =
            resident.execution_input->previousControlSourceProducerInstanceId(),
        .previous_control_source_sequence =
            resident.execution_input->previousControlSourceSequence(),
        .previous_control_source_stamp_ns =
            resident.execution_input->previousControlSourceStampNs(),
        .previous_control_receive_stamp_ns =
            resident.execution_input->previousControlReceiveStampNs(),
    });
  };
  const auto lidar_capture = [&](const std::uint64_t sequence,
                                 const std::int64_t acquisition_stamp_ns,
                                 std::vector<Point3> points) {
    return VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
        .producer_instance_id = resident.latest_lidar_evidence->producerInstanceId(),
        .sequence = sequence,
        .pose_generation = resident.latest_lidar_evidence->poseGeneration(),
        .acquisition_stamp_ns = acquisition_stamp_ns,
        .receive_stamp_ns = resident.latest_lidar_evidence->receiveStampNs(),
        .source_beam_count = std::max<std::size_t>(
            resident.latest_lidar_evidence->sourceBeamCount(), points.size()),
        .invalid_beam_count = resident.latest_lidar_evidence->invalidBeamCount(),
        .hit_points_map_m = std::move(points),
    });
  };
  const auto certify_candidate =
      [&](std::shared_ptr<const VersionedExecutionInput3D> input,
          std::shared_ptr<const VersionedLatestLidarEvidence3D> lidar) {
        return certifyFiniteExecution3D(
            *active, *active->route(),
            FiniteExecutionCertification3D{
                .trajectory_revision = resident.trajectory_revision + 1U,
                .horizon = *resident.horizon,
                .execution_input = std::move(input),
                .latest_lidar_evidence = std::move(lidar),
                .valid_from_ns = resident.valid_from_ns,
                .kind = FiniteExecutionKind3D::kNominal,
            });
      };
  const auto expect_replacement_rejected =
      [&](std::optional<FiniteExecutionState3D> candidate) {
        ASSERT_TRUE(candidate.has_value());
        EXPECT_EQ(replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                                           std::move(candidate))
                      .status,
                  ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
      };

  MotionControl3D conflicting_control = resident.execution_input->previousControl();
  conflicting_control.ax += 0.01F;
  expect_replacement_rejected(certify_candidate(
      input_capture(resident.execution_input->captureSequence(), conflicting_control),
      resident.latest_lidar_evidence));

  expect_replacement_rejected(
      certify_candidate(input_capture(resident.execution_input->captureSequence() - 1U,
                                      resident.execution_input->previousControl()),
                        resident.latest_lidar_evidence));

  const std::shared_ptr<const VersionedExecutionInput3D> newer_input =
      input_capture(resident.execution_input->captureSequence() + 1U,
                    resident.execution_input->previousControl());
  ASSERT_NE(newer_input, nullptr);
  expect_replacement_rejected(certify_candidate(
      newer_input, lidar_capture(resident.latest_lidar_evidence->sequence(),
                                 resident.latest_lidar_evidence->acquisitionStampNs(),
                                 {Point3{-4.0, -4.0, 1.0}})));
  expect_replacement_rejected(certify_candidate(
      newer_input,
      lidar_capture(resident.latest_lidar_evidence->sequence() + 1U,
                    resident.latest_lidar_evidence->acquisitionStampNs(), {})));
}

} // namespace
} // namespace drone_city_nav
