#include "drone_city_nav/execution_route_geometry_3d.hpp"

#include "execution_route_snapshot_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::shared_ptr<const VersionedExecutionInput3D>
stationaryCaptureInput(const VersionedExecutionInput3D& source,
                       const std::optional<mppi::State> state_override = std::nullopt) {
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
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
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
  ASSERT_TRUE(advanced.next->route.has_value());
  EXPECT_EQ(advanced.next->route->progress.execution_input, exact_input);
  EXPECT_DOUBLE_EQ(advanced.next->route->progress.last_observed_position.x, 4.0);
  EXPECT_DOUBLE_EQ(advanced.next->route->progress.last_observed_position.y, 0.0);
  EXPECT_DOUBLE_EQ(advanced.next->route->progress.last_observed_position.z, 5.0);
}

TEST(ExecutionRouteSnapshot3DTest,
     StaticProgressOwnerChangeRequiresFiniteExecutionRevalidation) {
  SnapshotFixture3D fixture;
  const OccupancyGrid3D static_occupancy{fixture.raw_occupancy.bounds(),
                                         fixture.validated_world.esdf_fingerprint};
  const std::optional<CertifiedRouteSuffix3D> suffix =
      certifyExecutionRoute3D(staticActivation(fixture, static_occupancy));
  ASSERT_TRUE(suffix.has_value());
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  FiniteExecutionState3D initial_execution = SnapshotFixture3D::finiteExecutionForRoute(
      *initial, *suffix, FiniteExecutionKind3D::kNominal, true, 100U);
  const ExecutionRouteTransitionResult3D active = activateCertifiedRoute3D(
      *initial, initial->version, *suffix, std::move(initial_execution));
  ASSERT_TRUE(active.applied());
  const std::shared_ptr<const VersionedExecutionInput3D> exact_input =
      SnapshotFixture3D::progressInput(*active.next, {4.0, 0.0, 5.0});

  const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
      *active.next, SnapshotFixture3D::guard(*active.next),
      fixture.executionObservation({999.0, 999.0, 999.0}, 0U, nullptr), exact_input,
      nullptr);

  ASSERT_TRUE(advanced.applied());
  ASSERT_TRUE(advanced.next->route.has_value());
  ASSERT_TRUE(advanced.next->finite_execution.has_value());
  EXPECT_EQ(advanced.next->route->progress.execution_input, exact_input);
  EXPECT_TRUE(advanced.next->finite_execution->revalidation_required);
  EXPECT_TRUE(advanced.next->valid());
}

TEST(ExecutionRouteSnapshot3DTest,
     InitialActivationBindsVehicleEvidenceAfterRevokedOwner) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  ASSERT_EQ(suffix->progress.execution_input, nullptr);
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  const ExecutionRouteTransitionResult3D revoked =
      revokeExecution3D(*initial, initial->version);
  ASSERT_TRUE(revoked.applied());
  FiniteExecutionState3D candidate_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *revoked.next, *suffix, FiniteExecutionKind3D::kNominal, true, 100U);
  const std::shared_ptr<const VersionedExecutionInput3D> exact_input =
      candidate_execution.execution_input;

  const ExecutionRouteTransitionResult3D activated = activateCertifiedRoute3D(
      *revoked.next, revoked.next->version, *suffix, std::move(candidate_execution));

  ASSERT_TRUE(activated.applied());
  ASSERT_TRUE(activated.next->route.has_value());
  EXPECT_FALSE(activated.next->stationary_hold.has_value());
  EXPECT_GT(activated.next->execution_owner_epoch, revoked.next->execution_owner_epoch);
  EXPECT_EQ(activated.next->route->progress.execution_input, exact_input);
  EXPECT_DOUBLE_EQ(activated.next->route->progress.last_observed_position.x, 2.0);
  EXPECT_TRUE(activated.next->valid());
}

TEST(ExecutionRouteSnapshot3DTest,
     HoldTransferRequiresReachedCertifiedTerminalRestAndPreservesLineage) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_TRUE(active->finite_execution.has_value());
  StationaryExecutionHoldCertification3D moving =
      SnapshotFixture3D::holdCertification(*active, false);
  EXPECT_EQ(
      transferToExecutionHold3D(*active, active->version, std::move(moving)).status,
      ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  const std::int64_t before_terminal_ns = active->finite_execution->valid_until_ns - 1;
  StationaryExecutionHoldCertification3D early = SnapshotFixture3D::holdCertification(
      *active, true, std::nullopt, before_terminal_ns);
  EXPECT_EQ(
      transferToExecutionHold3D(*active, active->version, std::move(early)).status,
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  const mppi::State& terminal = active->finite_execution->horizon->states.back();
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
          mppi::Control{.ax = 0.05F, .yaw_accel = 0.02F});
  const Point3 hold_position = certification.position;

  const ExecutionRouteTransitionResult3D transferred =
      transferToExecutionHold3D(*active, active->version, std::move(certification));

  ASSERT_TRUE(transferred.applied());
  ASSERT_NE(transferred.next, nullptr);
  EXPECT_EQ(transferred.next->version, active->version + 1U);
  EXPECT_EQ(transferred.next->phase, ExecutionRoutePhase3D::kStopped);
  EXPECT_FALSE(transferred.next->route.has_value());
  EXPECT_FALSE(transferred.next->finite_execution.has_value());
  EXPECT_FALSE(transferred.next->direct_tracking_execution.has_value());
  ASSERT_TRUE(transferred.next->stationary_hold.has_value());
  EXPECT_DOUBLE_EQ(transferred.next->stationary_hold->position.x, hold_position.x);
  EXPECT_DOUBLE_EQ(transferred.next->stationary_hold->position.y, hold_position.y);
  EXPECT_DOUBLE_EQ(transferred.next->stationary_hold->position.z, hold_position.z);
  EXPECT_EQ(transferred.next->stationary_hold->hold_id,
            transferred.next->execution_owner_epoch);
  EXPECT_EQ(transferred.next->stationary_hold->origin,
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
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->finite_execution.has_value());
  StationaryExecutionHoldCertification3D boundary =
      SnapshotFixture3D::holdCertification(*active);
  ASSERT_NE(boundary.execution_input, nullptr);
  ASSERT_NE(boundary.validation_policy, nullptr);
  ASSERT_NE(active->finite_execution->latest_lidar_evidence, nullptr);
  const std::int64_t maximum_age_ns = static_cast<std::int64_t>(
      boundary.validation_policy->latestLidarMaximumAgeMs() * 1.0e6);
  ASSERT_GT(maximum_age_ns, 0);
  boundary.latest_lidar_evidence =
      lidarAtAge(*active->finite_execution->latest_lidar_evidence,
                 boundary.execution_input->effectiveStampNs(), maximum_age_ns, 1U);
  ASSERT_NE(boundary.latest_lidar_evidence, nullptr);
  EXPECT_TRUE(transferToExecutionHold3D(*active, active->version, boundary).applied());

  StationaryExecutionHoldCertification3D expired =
      SnapshotFixture3D::holdCertification(*active);
  expired.latest_lidar_evidence =
      lidarAtAge(*active->finite_execution->latest_lidar_evidence,
                 expired.execution_input->effectiveStampNs(), maximum_age_ns + 1, 2U);
  ASSERT_NE(expired.latest_lidar_evidence, nullptr);
  EXPECT_EQ(
      transferToExecutionHold3D(*active, active->version, std::move(expired)).status,
      ExecutionRouteTransitionStatus3D::kInvalidCandidate);
}

TEST(ExecutionRouteSnapshot3DTest,
     StationaryCaptureRearmIsNamedRevokedOnlyAndPreservesPhysicalContinuity) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
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
  ASSERT_TRUE(rearmed.next->stationary_hold.has_value());
  EXPECT_EQ(rearmed.next->phase, ExecutionRoutePhase3D::kStopped);
  EXPECT_EQ(rearmed.next->stationary_hold->origin,
            StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm);
  EXPECT_EQ(rearmed.next->stationary_hold->source_trajectory_revision, 0U);
  EXPECT_EQ(rearmed.next->stationary_hold->hold_id,
            rearmed.next->execution_owner_epoch);
  EXPECT_TRUE(rearmed.next->valid());

  StationaryExecutionHoldCertification3D moving = terminal;
  mppi::State moving_state = terminal.execution_input->state();
  moving_state.vx =
      static_cast<float>(kStationaryExecutionHoldSpeedToleranceMps + 0.01);
  moving.execution_input =
      stationaryCaptureInput(*terminal.execution_input, moving_state);
  ASSERT_NE(moving.execution_input, nullptr);
  EXPECT_EQ(armStationaryCaptureHold3D(*revoked.next, revoked.next->version,
                                       std::move(moving))
                .status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  StationaryExecutionHoldCertification3D raw_unsafe = terminal;
  ASSERT_NE(raw_unsafe.latest_lidar_evidence, nullptr);
  raw_unsafe.latest_lidar_evidence =
      VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
          .producer_instance_id =
              raw_unsafe.latest_lidar_evidence->producerInstanceId(),
          .sequence = raw_unsafe.latest_lidar_evidence->sequence() + 1U,
          .pose_generation = raw_unsafe.latest_lidar_evidence->poseGeneration() + 1U,
          .acquisition_stamp_ns = raw_unsafe.execution_input->effectiveStampNs(),
          .receive_stamp_ns = raw_unsafe.execution_input->effectiveStampNs(),
          .source_beam_count = 1U,
          .hit_points_map_m = {raw_unsafe.position},
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
     HoldTransferReplacesDirectOwnerAndStableEvidenceReplayIsNoChange) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> route_owner =
      fixture.activeSnapshot();
  ASSERT_NE(route_owner, nullptr);
  ASSERT_TRUE(route_owner->route.has_value());
  const DirectTrackingOwnerIdentity3D identity{
      .mission_epoch = fixture.objective.mission_epoch,
      .assignment_generation = fixture.objective.assignment_generation,
      .target_detection_id = 41U,
      .target_track_id = 42U,
      .objective_sample_sequence = fixture.objective.sample_sequence,
      .line_of_sight_generation = 7U,
  };
  const std::optional<DirectTrackingFiniteExecution3D> direct_execution =
      certifyDirectFixtureExecution(*route_owner, *route_owner->route, identity, 101U);
  ASSERT_TRUE(direct_execution.has_value());
  const ExecutionRouteTransitionResult3D direct =
      transferToDirectTracking3D(*route_owner, route_owner->version, *direct_execution);
  ASSERT_TRUE(direct.applied());
  StationaryExecutionHoldCertification3D certification =
      SnapshotFixture3D::holdCertification(*direct.next);

  const ExecutionRouteTransitionResult3D transferred = transferToExecutionHold3D(
      *direct.next, direct.next->version, std::move(certification));

  ASSERT_TRUE(transferred.applied());
  ASSERT_NE(transferred.next, nullptr);
  EXPECT_FALSE(transferred.next->route.has_value());
  EXPECT_FALSE(transferred.next->finite_execution.has_value());
  EXPECT_FALSE(transferred.next->direct_tracking_execution.has_value());
  EXPECT_EQ(transferred.next->routeGenerationHighWater(),
            route_owner->routeGenerationHighWater());
  ASSERT_TRUE(transferred.next->stationary_hold.has_value());
  EXPECT_EQ(
      transferToExecutionHold3D(*transferred.next, transferred.next->version,
                                SnapshotFixture3D::holdCertification(*transferred.next))
          .status,
      ExecutionRouteTransitionStatus3D::kNoChange);
}

TEST(ExecutionRouteSnapshot3DTest,
     HoldOwnerReplacementRequiresContinuousNewerEvidenceAndDedicatedTransitions) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> route_owner =
      fixture.activeSnapshot();
  ASSERT_NE(route_owner, nullptr);
  ASSERT_TRUE(route_owner->route.has_value());
  const ExecutionRouteTransitionResult3D held =
      transferToExecutionHold3D(*route_owner, route_owner->version,
                                SnapshotFixture3D::holdCertification(*route_owner));
  ASSERT_TRUE(held.applied());
  ASSERT_TRUE(held.next->stationary_hold.has_value());

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
  FiniteExecutionCertification3D discontinuous_finite =
      SnapshotFixture3D::finiteCertificationForRoute(
          *unbound_route, FiniteExecutionKind3D::kNominal, 101U, 101U, 0U, -1.0,
          held.next->stationary_hold->terminal_execution_input.get());
  const std::optional<DirectTrackingFiniteExecution3D> discontinuous_direct =
      certifyDirectTrackingExecution3D(
          *held.next,
          DirectTrackingExecutionCertification3D{
              .identity = identity,
              .trajectory_revision = 101U,
              .target = {10.0, 0.0, 5.0},
              .horizon = std::move(discontinuous_finite.horizon),
              .observed_raw_world = unbound_route->observed_raw_world,
              .static_world = unbound_route->static_world,
              .validation_policy = unbound_route->validation_policy,
              .execution_input = std::move(discontinuous_finite.execution_input),
              .latest_lidar_evidence =
                  std::move(discontinuous_finite.latest_lidar_evidence),
              .valid_from_ns = discontinuous_finite.valid_from_ns,
              .kind = FiniteExecutionKind3D::kNominal,
          });
  ASSERT_TRUE(discontinuous_direct.has_value());
  EXPECT_EQ(
      transferToDirectTracking3D(*held.next, held.next->version, *discontinuous_direct)
          .status,
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  const std::optional<DirectTrackingFiniteExecution3D> continuous_direct =
      certifyDirectFixtureExecution(*held.next, *route_owner->route, identity, 102U);
  ASSERT_TRUE(continuous_direct.has_value());
  const ExecutionRouteTransitionResult3D direct =
      transferToDirectTracking3D(*held.next, held.next->version, *continuous_direct);
  ASSERT_TRUE(direct.applied()) << executionRouteTransitionStatus3DName(direct.status);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = direct.next->routeGenerationHighWater() + 1U;
  successor_activation.observation.position = Point3{
      direct.next->direct_tracking_execution->execution_input->state().x,
      direct.next->direct_tracking_execution->execution_input->state().y,
      direct.next->direct_tracking_execution->execution_input->state().z,
  };
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  FiniteExecutionState3D successor_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *direct.next, *successor, FiniteExecutionKind3D::kNominal, true, 103U);
  EXPECT_EQ(activateCertifiedRoute3D(*direct.next, direct.next->version, *successor,
                                     std::move(successor_execution))
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

TEST(ExecutionRouteSnapshot3DTest,
     HoldCertificationRejectsRawAndLatestLidarCollisions) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);

  StationaryExecutionHoldCertification3D raw_blocked =
      SnapshotFixture3D::holdCertification(*active);
  ObservedOccupancyGrid3D blocked_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> blocked_cell =
      blocked_occupancy.worldToCell(raw_blocked.position);
  ASSERT_TRUE(blocked_cell.has_value());
  ASSERT_TRUE(blocked_occupancy.setState(*blocked_cell, ObservedVoxelState::kOccupied));
  raw_blocked.observed_raw_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &blocked_occupancy);
  EXPECT_EQ(transferToExecutionHold3D(*active, active->version, std::move(raw_blocked))
                .status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  StationaryExecutionHoldCertification3D lidar_blocked =
      SnapshotFixture3D::holdCertification(*active);
  lidar_blocked.latest_lidar_evidence = SnapshotFixture3D::newerLidarEvidence(
      *lidar_blocked.latest_lidar_evidence,
      std::vector<Point3>{lidar_blocked.position});
  ASSERT_NE(lidar_blocked.latest_lidar_evidence, nullptr);
  EXPECT_EQ(
      transferToExecutionHold3D(*active, active->version, std::move(lidar_blocked))
          .status,
      ExecutionRouteTransitionStatus3D::kInvalidCandidate);
}

TEST(ExecutionRouteSnapshot3DTest,
     HoldEvidenceRefreshPreservesStableOwnerIdentityAndTarget) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  const ExecutionRouteTransitionResult3D held = transferToExecutionHold3D(
      *active, active->version, SnapshotFixture3D::holdCertification(*active));
  ASSERT_TRUE(held.applied());
  ASSERT_TRUE(held.next->stationary_hold.has_value());

  StationaryExecutionHoldCertification3D refreshed =
      SnapshotFixture3D::holdCertification(*held.next);
  refreshed.latest_lidar_evidence =
      SnapshotFixture3D::newerLidarEvidence(*refreshed.latest_lidar_evidence);
  ASSERT_NE(refreshed.latest_lidar_evidence, nullptr);
  const ExecutionRouteTransitionResult3D updated =
      transferToExecutionHold3D(*held.next, held.next->version, std::move(refreshed));

  ASSERT_TRUE(updated.applied());
  ASSERT_TRUE(updated.next->stationary_hold.has_value());
  EXPECT_EQ(updated.next->version, held.next->version + 1U);
  EXPECT_EQ(updated.next->execution_owner_epoch, held.next->execution_owner_epoch);
  EXPECT_EQ(updated.next->stationary_hold->hold_id,
            held.next->stationary_hold->hold_id);
  EXPECT_EQ(updated.next->stationary_hold->position.x,
            held.next->stationary_hold->position.x);
  EXPECT_EQ(updated.next->stationary_hold->position.y,
            held.next->stationary_hold->position.y);
  EXPECT_EQ(updated.next->stationary_hold->position.z,
            held.next->stationary_hold->position.z);
}

TEST(ExecutionRouteSnapshot3DTest,
     EmptyOwnerCannotInventAStationaryHoldAndRevocationIsIdempotent) {
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  EXPECT_THROW(SnapshotFixture3D::holdCertification(*initial), std::logic_error);

  const ExecutionRouteTransitionResult3D revoked =
      revokeExecution3D(*initial, initial->version);
  ASSERT_TRUE(revoked.applied());
  EXPECT_EQ(revoked.next->phase, ExecutionRoutePhase3D::kRevoked);
  EXPECT_EQ(revoked.next->routeGenerationHighWater(), 0U);
  EXPECT_EQ(revoked.next->execution_owner_epoch, initial->execution_owner_epoch + 1U);
  EXPECT_EQ(revokeExecution3D(*revoked.next, revoked.next->version).status,
            ExecutionRouteTransitionStatus3D::kNoChange);
  EXPECT_EQ(revokeExecution3D(*revoked.next, initial->version).status,
            ExecutionRouteTransitionStatus3D::kStaleSnapshotVersion);

  ExecutionRouteSnapshot3D exhausted = *revoked.next;
  exhausted.version = std::numeric_limits<std::uint64_t>::max();
  ASSERT_TRUE(exhausted.valid());
  EXPECT_EQ(revokeExecution3D(exhausted, exhausted.version).status,
            ExecutionRouteTransitionStatus3D::kVersionExhausted);
}

TEST(ExecutionRouteSnapshot3DTest,
     ProgressCanAdvanceWhileAnEarlierFiniteHorizonRemainsTheOwner) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
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
  ASSERT_TRUE(advanced.next->finite_execution.has_value());
  EXPECT_TRUE(advanced.next->finite_execution->revalidation_required);
  EXPECT_DOUBLE_EQ(advanced.next->finite_execution->begin_route_station_m, 2.0);
  EXPECT_DOUBLE_EQ(advanced.next->route->progress.station_m, 4.0);
  EXPECT_TRUE(advanced.next->valid());
}

TEST(ExecutionRouteSnapshot3DTest,
     ChangedContentAdvanceTransfersOnlyTheRouteWorldOwner) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_TRUE(active->finite_execution.has_value());
  const std::uint64_t old_content =
      active->route->observed_raw_world->contentFingerprint();
  const auto old_finite_owner = active->finite_execution->observed_raw_world;

  ObservedOccupancyGrid3D changed_occupancy = fixture.raw_occupancy;
  ASSERT_TRUE(changed_occupancy.setState({0, 0, 0}, ObservedVoxelState::kOccupied));
  const auto changed_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &changed_occupancy);
  ASSERT_TRUE(changed_world);
  ASSERT_NE(changed_world->contentFingerprint(), old_content);
  RouteExecutionObservation3D poisoned_borrowed_fields =
      fixture.executionObservation({4.0, 0.0, 5.0}, 1U, nullptr);
  poisoned_borrowed_fields.latest_raw_producer_instance_id = 1U;

  const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active), poisoned_borrowed_fields,
      SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0}), changed_world);

  ASSERT_TRUE(advanced.applied());
  ASSERT_TRUE(advanced.next);
  ASSERT_TRUE(advanced.next->route.has_value());
  ASSERT_TRUE(advanced.next->finite_execution.has_value());
  EXPECT_EQ(advanced.next->route->observed_raw_world, changed_world);
  EXPECT_EQ(advanced.next->route->observed_raw_world->contentFingerprint(),
            changed_world->contentFingerprint());
  EXPECT_EQ(advanced.next->finite_execution->observed_raw_world, old_finite_owner);
  EXPECT_EQ(advanced.next->finite_execution->observed_raw_world->contentFingerprint(),
            old_content);
  EXPECT_TRUE(advanced.next->finite_execution->revalidation_required);
  EXPECT_TRUE(advanced.next->valid());

  ASSERT_TRUE(changed_occupancy.setState({1, 0, 0}, ObservedVoxelState::kOccupied));
  EXPECT_EQ(advanced.next->route->observed_raw_world->contentFingerprint(),
            changed_world->contentFingerprint());
  EXPECT_EQ(advanced.next->route->observed_raw_world->occupancy().state({1, 0, 0}),
            ObservedVoxelState::kUnknown);
}

TEST(ExecutionRouteSnapshot3DTest,
     ConstrainedAdvanceRejectsANewerWorldWithDifferentPassageGeometry) {
  SnapshotFixture3D fixture;
  fixture.geometry = makeConstrainedGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      SnapshotFixture3D::kRouteGeneration, fixture.raw_occupancy.occupiedSnapshot(),
      testPassageVolumeConfig());
  fixture.geometry_revision = fixture.geometry->executable_geometry_revision;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_FALSE(active->route->geometry->constrained_spans->empty());

  ObservedOccupancyGrid3D changed_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> lateral_wall =
      changed_occupancy.worldToCell(Point3{5.0, 2.0, 5.0});
  ASSERT_TRUE(lateral_wall.has_value());
  ASSERT_TRUE(changed_occupancy.setState(*lateral_wall, ObservedVoxelState::kOccupied));
  const auto changed_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &changed_occupancy);
  ASSERT_TRUE(changed_world);
  ASSERT_NE(changed_world->occupiedContentFingerprint(),
            active->route->observed_raw_world->occupiedContentFingerprint());

  const ExecutionRouteTransitionResult3D rejected = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active),
      fixture.executionObservation({4.0, 0.0, 5.0},
                                   SnapshotFixture3D::kLatestRawRevision + 1U,
                                   &changed_occupancy),
      SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0}), changed_world);

  EXPECT_EQ(rejected.status, ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  EXPECT_FALSE(rejected.next);
}

TEST(ExecutionRouteSnapshot3DTest,
     AdvanceRejectsDifferentContentAdvertisedAtTheCertifiedRevision) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ObservedOccupancyGrid3D changed_occupancy = fixture.raw_occupancy;
  ASSERT_TRUE(changed_occupancy.setState({0, 0, 0}, ObservedVoxelState::kOccupied));

  const ExecutionRouteTransitionResult3D rejected = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active),
      fixture.executionObservation({4.0, 0.0, 5.0},
                                   SnapshotFixture3D::kLatestRawRevision,
                                   &fixture.raw_occupancy),
      SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0}),
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision, &changed_occupancy));

  EXPECT_EQ(rejected.status, ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  EXPECT_FALSE(rejected.next);
}

TEST(ExecutionRouteSnapshot3DTest,
     CertificationRejectsForgedRotatingPassageFramesAfterRevisionRecomputation) {
  SnapshotFixture3D fixture;
  fixture.route = {
      RouteSample3D{.position = {0.0, 0.0, 5.0},
                    .tangent = {1.0, 0.0, 0.0},
                    .station_m = 0.0,
                    .reference_speed_mps = 4.0},
      RouteSample3D{.position = {4.0, 0.0, 5.0},
                    .tangent = {1.0, 0.0, 0.0},
                    .station_m = 4.0,
                    .reference_speed_mps = 4.0},
      RouteSample3D{.position = {4.09, 0.0, 5.0},
                    .tangent = {1.0, 0.0, 0.0},
                    .station_m = 4.09,
                    .reference_speed_mps = 4.0},
      RouteSample3D{.position = {10.0, 0.0, 5.0},
                    .tangent = {1.0, 0.0, 0.0},
                    .station_m = 10.0,
                    .reference_speed_mps = 0.0},
  };
  fixture.physical_route_fingerprint = routeFingerprint(fixture.route);
  fixture.proposal.route_fingerprint = fixture.physical_route_fingerprint;
  fixture.proposal.route_sample_count = fixture.route.size();
  auto geometry = std::make_shared<ExecutionRouteGeometry3D>(*makeConstrainedGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      SnapshotFixture3D::kRouteGeneration, fixture.raw_occupancy.occupiedSnapshot(),
      testPassageVolumeConfig()));
  auto volumes =
      std::make_shared<std::vector<PassageVolume>>(*geometry->passage_volumes);
  ASSERT_GE(volumes->front().cross_sections.size(), 4U);
  constexpr double kAngleRad{89.0 * std::numbers::pi / 180.0};
  const Vec3 rotated_lateral{0.0, std::cos(kAngleRad), std::sin(kAngleRad)};
  const Vec3 rotated_secondary{0.0, -std::sin(kAngleRad), std::cos(kAngleRad)};
  for (std::size_t index = volumes->front().cross_sections.size() / 2U;
       index < volumes->front().cross_sections.size(); ++index) {
    volumes->front().cross_sections[index].lateral_axis = rotated_lateral;
    volumes->front().cross_sections[index].secondary_axis = rotated_secondary;
  }
  geometry->passage_volumes = std::move(volumes);
  geometry->executable_geometry_revision = executionRouteGeometryRevision3D(*geometry);
  fixture.geometry = std::move(geometry);
  fixture.geometry_revision = fixture.geometry->executable_geometry_revision;
  EXPECT_FALSE(fixture.activeSnapshot());
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

  const ExecutionRouteGeometryValidation3D validation =
      validateExecutionRouteGeometrySamples3D(route);

  EXPECT_FALSE(validation.valid());
  EXPECT_EQ(validation.reason,
            ExecutionRouteGeometryFailureReason3D::kTerminalTangentMismatch);
  EXPECT_EQ(validation.sample_index, 1U);
  EXPECT_STREQ(executionRouteGeometryFailureReasonName3D(validation.reason),
               "terminal_tangent_mismatch");
}

TEST(ExecutionRouteSnapshot3DTest,
     ExecutionAssessmentRejectsWrongRawLineageAndANewRawCollision) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
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
  std::shared_ptr<const ExecutionRouteSnapshot3D> survivor;
  {
    SnapshotFixture3D fixture;
    const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
        fixture.activeSnapshot();
    ASSERT_TRUE(active);
    const ExecutionRouteTransitionResult3D following =
        replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                                 SnapshotFixture3D::finiteExecution(*active));
    ASSERT_TRUE(following.applied());
    survivor = following.next;
  }

  ASSERT_TRUE(survivor);
  EXPECT_TRUE(survivor->valid());
  ASSERT_TRUE(survivor->route.has_value());
  ASSERT_TRUE(survivor->finite_execution.has_value());
  EXPECT_EQ(survivor->route->geometry->route->size(), 3U);
  EXPECT_EQ(survivor->finite_execution->horizon->states.size(),
            survivor->finite_execution->horizon->controls.size() + 1U);
  EXPECT_FLOAT_EQ(
      survivor->finite_execution->horizon->states.back().x,
      static_cast<float>(survivor->finite_execution->stop_boundary.position.x));
}

TEST(ExecutionRouteSnapshot3DTest, GuardsAndStoreEnforceVersionedCasPublication) {
  static_assert(
      std::is_const_v<std::remove_reference_t<
          decltype(std::declval<ExecutionRouteTransitionResult3D&>().status)>>);
  static_assert(std::is_const_v<std::remove_reference_t<
                    decltype(std::declval<ExecutionRouteTransitionResult3D&>().next)>>);
  SnapshotFixture3D fixture;
  ExecutionRouteSnapshotStore3D store;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial = store.snapshot();
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(initial);
  ASSERT_TRUE(suffix.has_value());
  FiniteExecutionState3D initial_execution = SnapshotFixture3D::finiteExecutionForRoute(
      *initial, *suffix, FiniteExecutionKind3D::kNominal, true, 100U);

  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *initial, initial->version, *suffix, std::move(initial_execution));
  ASSERT_TRUE(activation.applied());
  ASSERT_TRUE(activation.next);
  EXPECT_EQ(store.publish(initial, activation),
            ExecutionRoutePublicationStatus3D::kPublished);
  EXPECT_EQ(store.publish(initial, activation),
            ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion);
  EXPECT_EQ(store.snapshot(), activation.next);

  ExecutionRouteSnapshotStore3D other_store;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> other_initial =
      other_store.snapshot();
  ASSERT_TRUE(other_initial);
  EXPECT_EQ(other_initial->version, initial->version);
  EXPECT_EQ(other_store.publish(other_initial, activation),
            ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion);

  const std::shared_ptr<const ExecutionRouteSnapshot3D> resident = store.snapshot();
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
  EXPECT_EQ(store.publish(resident, first_replacement),
            ExecutionRoutePublicationStatus3D::kPublished);
  EXPECT_EQ(store.publish(resident, competing_replacement),
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

  EXPECT_EQ(store.publish(store.snapshot(), ExecutionRouteTransitionResult3D{}),
            ExecutionRoutePublicationStatus3D::kInvalidCandidate);
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteExecutionRequiresBoundTerminalRestAndCertifiedStopBoundary) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  FiniteExecutionState3D finite = SnapshotFixture3D::finiteExecution(*active);

  const ExecutionRouteTransitionResult3D accepted =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active), finite);
  ASSERT_TRUE(accepted.applied());
  ASSERT_TRUE(accepted.next);
  EXPECT_TRUE(accepted.next->valid());
  ASSERT_TRUE(accepted.next->finite_execution.has_value());
  EXPECT_EQ(accepted.next->finite_execution->source_route_generation,
            active->route->identity.generation);
  EXPECT_TRUE(mppi::finiteHorizonHasTerminalRestState(
      *accepted.next->finite_execution->horizon));

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
  auto inconsistent_horizon = std::make_shared<mppi::FiniteHorizon>(*finite.horizon);
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
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);

  const ExecutionRouteTransitionResult3D retained = replaceFiniteExecution3D(
      *active, SnapshotFixture3D::guard(*active),
      SnapshotFixture3D::finiteExecution(*active, FiniteExecutionKind3D::kRetained,
                                         true, 101U));
  ASSERT_TRUE(retained.applied());
  ASSERT_NE(retained.next, nullptr);
  EXPECT_TRUE(retained.next->valid());
  EXPECT_EQ(retained.next->phase, ExecutionRoutePhase3D::kFollowing);
  ASSERT_TRUE(retained.next->finite_execution.has_value());
  EXPECT_EQ(retained.next->finite_execution->kind, FiniteExecutionKind3D::kRetained);

  const FiniteExecutionState3D emergency = SnapshotFixture3D::finiteExecution(
      *active, FiniteExecutionKind3D::kEmergencyBrakeTail, true, 101U);
  EXPECT_EQ(
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active), emergency)
          .status,
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

TEST(ExecutionRouteSnapshot3DTest, RejectsUnknownFiniteAndLifecycleEnumValues) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ASSERT_TRUE(active->route.has_value());

  FiniteExecutionState3D unknown_execution =
      SnapshotFixture3D::finiteExecution(*active);
  unknown_execution.kind = static_cast<FiniteExecutionKind3D>(255U);
  EXPECT_FALSE(unknown_execution.validFor(&*active->route));
  EXPECT_EQ(replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                                     unknown_execution)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  const RouteLifecycleEvent3D unknown_event{
      .kind = static_cast<RouteLifecycleEventKind3D>(255U),
      .generation = active->route->identity.generation,
  };
  EXPECT_EQ(retireCertifiedRoute3D(*active, SnapshotFixture3D::guard(*active),
                                   unknown_event, std::nullopt)
                .status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);
}

TEST(ExecutionRouteSnapshot3DTest, SafeReplacementCannotExtendDeadline) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ASSERT_TRUE(active->finite_execution.has_value());
  const RouteLifecycleEvent3D superseded{
      .kind = RouteLifecycleEventKind3D::kObjectiveSuperseded,
      .generation = SnapshotFixture3D::kRouteGeneration,
  };

  const FiniteExecutionState3D equal_deadline = SnapshotFixture3D::finiteExecution(
      *active, FiniteExecutionKind3D::kRetained, true, 101U);
  EXPECT_EQ(equal_deadline.valid_until_ns, active->finite_execution->valid_until_ns);
  const ExecutionRouteTransitionResult3D accepted = retireCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active), superseded, equal_deadline);
  ASSERT_TRUE(accepted.applied());
  EXPECT_EQ(accepted.next->phase, ExecutionRoutePhase3D::kBraking);

  const FiniteExecutionState3D extended_deadline = SnapshotFixture3D::finiteExecution(
      *active, FiniteExecutionKind3D::kEmergencyBrakeTail, true, 102U, 55U, 1U);
  EXPECT_GT(extended_deadline.valid_until_ns, active->finite_execution->valid_until_ns);
  EXPECT_EQ(retireCertifiedRoute3D(*active, SnapshotFixture3D::guard(*active),
                                   superseded, extended_deadline)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteReplacementEnforcesCompleteInputAndLidarIdentityMonotonicity) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_TRUE(active->finite_execution.has_value());
  const FiniteExecutionState3D& resident = *active->finite_execution;
  ASSERT_NE(resident.execution_input, nullptr);
  ASSERT_NE(resident.latest_lidar_evidence, nullptr);

  const auto input_capture = [&](const std::uint64_t capture_sequence,
                                 const mppi::Control control) {
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
            *active, *active->route,
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

  mppi::Control conflicting_control = resident.execution_input->previousControl();
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
