#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(ExecutionRouteSnapshot3DTest,
     NonphysicalRevocationSuspendsOnlyTheFiniteHorizonAndCanResume) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_EQ(active->phase(), ExecutionRoutePhase3D::kFollowing);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(active->finiteExecution() != nullptr);
  ASSERT_TRUE(active->brakingFallback() != nullptr);

  const ExecutionRouteTransitionResult3D suspended =
      suspendFiniteExecution3D(*active, active->version);

  ASSERT_TRUE(suspended.applied());
  ASSERT_NE(suspended.next, nullptr);
  EXPECT_TRUE(suspended.next->valid());
  EXPECT_FALSE(suspended.next->publishable());
  EXPECT_EQ(suspended.next->phase(), ExecutionRoutePhase3D::kAwaitingSuccessor);
  ASSERT_TRUE(suspended.next->route() != nullptr);
  EXPECT_EQ(suspended.next->route()->route_instance_id,
            active->route()->route_instance_id);
  EXPECT_FALSE(suspended.next->finiteExecution() != nullptr);
  EXPECT_FALSE(suspended.next->brakingFallback() != nullptr);
  EXPECT_EQ(suspendFiniteExecution3D(*suspended.next, suspended.next->version).status,
            ExecutionRouteTransitionStatus3D::kNoChange);

  FiniteExecutionPlan3D resumed_plan = SnapshotFixture3D::finitePlanForRoute(
      *suspended.next, *suspended.next->route(), FiniteExecutionKind3D::kNominal, 101U);
  const ExecutionRouteTransitionResult3D resumed = replaceFiniteExecutionPlan3D(
      *suspended.next, SnapshotFixture3D::guard(*suspended.next),
      std::move(resumed_plan));

  ASSERT_TRUE(resumed.applied());
  ASSERT_NE(resumed.next, nullptr);
  EXPECT_TRUE(resumed.next->valid());
  EXPECT_TRUE(resumed.next->publishable());
  EXPECT_EQ(resumed.next->phase(), ExecutionRoutePhase3D::kFollowing);
  ASSERT_TRUE(resumed.next->route() != nullptr);
  EXPECT_EQ(resumed.next->route()->route_instance_id,
            active->route()->route_instance_id);
  EXPECT_TRUE(resumed.next->finiteExecution() != nullptr);
  EXPECT_TRUE(resumed.next->brakingFallback() != nullptr);
}

TEST(ExecutionRouteSnapshot3DTest,
     SuspendedRouteAcceptsAFullyCertifiedSameIntentCurrentStateSuccessor) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  const ExecutionRouteTransitionResult3D suspended =
      suspendFiniteExecution3D(*active, active->version);
  ASSERT_TRUE(suspended.applied());
  ASSERT_NE(suspended.next, nullptr);
  ASSERT_TRUE(suspended.next->route() != nullptr);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
  successor_activation =
      rebindUnconstrainedDecorations(std::move(successor_activation));
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const FiniteExecutionState3D successor_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *suspended.next, *successor, FiniteExecutionKind3D::kNominal, true, 102U);

  const ExecutionRouteTransitionResult3D replaced = replaceCertifiedRouteAtHandoff3D(
      *suspended.next, SnapshotFixture3D::guard(*suspended.next), *successor,
      successor_execution);

  ASSERT_TRUE(replaced.applied());
  ASSERT_NE(replaced.next, nullptr);
  ASSERT_TRUE(replaced.next->route() != nullptr);
  EXPECT_TRUE(replaced.next->publishable());
  EXPECT_EQ(replaced.next->phase(), ExecutionRoutePhase3D::kFollowing);
  EXPECT_EQ(replaced.next->route()->identity.generation,
            SnapshotFixture3D::kRouteGeneration + 1U);
  EXPECT_EQ(replaced.next->route()->owner.id, suspended.next->route()->owner.id);
  EXPECT_EQ(replaced.next->execution_owner_epoch,
            suspended.next->execution_owner_epoch);
}

TEST(ExecutionRouteSnapshot3DTest,
     CurrentStateHandoffCanAdvanceBeyondSuspendedResidentProgress) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  const ExecutionRouteTransitionResult3D suspended =
      suspendFiniteExecution3D(*active, active->version);
  ASSERT_TRUE(suspended.applied());
  ASSERT_NE(suspended.next, nullptr);
  ASSERT_TRUE(suspended.next->route() != nullptr);
  const CertifiedRouteSuffix3D suspended_route = *suspended.next->route();
  ASSERT_TRUE(suspended_route.valid());

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
  successor_activation =
      rebindUnconstrainedDecorations(std::move(successor_activation));
  successor_activation.observation.position = {3.0, 0.0, 5.0};
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const CertifiedRouteSuffix3D successor_route =
      successor.value_or(CertifiedRouteSuffix3D{});
  ASSERT_TRUE(successor_route.valid());
  ASSERT_GT(distance3D(successor_route.progress.last_observed_position,
                       suspended_route.progress.last_observed_position),
            0.25);
  const FiniteExecutionState3D successor_execution =
      SnapshotFixture3D::finiteExecutionForRoute(*suspended.next, successor_route,
                                                 FiniteExecutionKind3D::kNominal, true,
                                                 102U);

  const ExecutionRouteTransitionResult3D replaced = replaceCertifiedRouteAtHandoff3D(
      *suspended.next, SnapshotFixture3D::guard(*suspended.next), successor_route,
      successor_execution);

  ASSERT_TRUE(replaced.applied());
  ASSERT_NE(replaced.next, nullptr);
  ASSERT_TRUE(replaced.next->route() != nullptr);
  const CertifiedRouteSuffix3D activated_route = *replaced.next->route();
  ASSERT_TRUE(activated_route.valid());
  EXPECT_TRUE(replaced.next->publishable());
  EXPECT_EQ(activated_route.identity.generation,
            SnapshotFixture3D::kRouteGeneration + 1U);
  EXPECT_EQ(activated_route.progress.last_observed_position.x, 3.0);
}

TEST(ExecutionRouteSnapshot3DTest,
     NewRawEvidenceInvalidatesOnlyTheRemainingPublishedFiniteTrajectory) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);

  FiniteExecutionCertification3D stationary_certification =
      SnapshotFixture3D::finiteCertificationForRoute(
          *active->route(), FiniteExecutionKind3D::kNominal, 101U);
  const std::optional<FiniteMotionHorizon3D> stationary_horizon =
      buildFiniteBrakingHorizon3D(
          stationary_certification.horizon.states.front(),
          stationary_certification.horizon.controls.size(),
          active->route()->validation_policy->dynamics(),
          stationary_certification.execution_input->previousControl());
  ASSERT_TRUE(stationary_horizon.has_value());
  stationary_certification.horizon = *stationary_horizon;
  const std::optional<FiniteExecutionState3D> stationary_execution =
      certifyFiniteExecution3D(*active, *active->route(),
                               std::move(stationary_certification));
  ASSERT_TRUE(stationary_execution.has_value());
  ASSERT_NE(stationary_execution->execution_input, nullptr);

  ObservedOccupancyGrid3D far_route_obstacle = fixture.raw_occupancy;
  const std::optional<GridIndex3D> far_route_cell =
      far_route_obstacle.worldToCell(Point3{8.0, 0.0, 5.0});
  ASSERT_TRUE(far_route_cell.has_value());
  ASSERT_TRUE(
      far_route_obstacle.setState(*far_route_cell, ObservedVoxelState::kOccupied));
  const std::shared_ptr<const VersionedObservedRawWorld3D> far_route_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &far_route_obstacle);
  ASSERT_NE(far_route_world, nullptr);

  const FiniteExecutionPathValidation3D unaffected =
      validateRemainingFiniteExecutionAgainstObservedWorld3D(
          *stationary_execution, *stationary_execution->execution_input,
          *far_route_world, stationary_execution->valid_from_ns);
  EXPECT_EQ(unaffected.status, FiniteExecutionPathStatus3D::kValid);

  // Evidence the body already overlaps at the vehicle's own pose is contact,
  // not an obstacle: the immediate braking path departs from it and stays
  // executable. Refusing it would leave a moving vehicle no validated motion.
  ObservedOccupancyGrid3D active_trajectory_obstacle = fixture.raw_occupancy;
  const MotionState3D& active_state = stationary_execution->execution_input->state();
  const std::optional<GridIndex3D> active_cell = active_trajectory_obstacle.worldToCell(
      Point3{active_state.x, active_state.y, active_state.z});
  ASSERT_TRUE(active_cell.has_value());
  ASSERT_TRUE(
      active_trajectory_obstacle.setState(*active_cell, ObservedVoxelState::kOccupied));
  const std::shared_ptr<const VersionedObservedRawWorld3D> active_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U,
                       &active_trajectory_obstacle);
  ASSERT_NE(active_world, nullptr);

  const FiniteExecutionPathValidation3D contact =
      validateRemainingFiniteExecutionAgainstObservedWorld3D(
          *stationary_execution, *stationary_execution->execution_input, *active_world,
          stationary_execution->valid_from_ns);
  EXPECT_EQ(contact.status, FiniteExecutionPathStatus3D::kValid);
}

TEST(ExecutionRouteSnapshot3DTest,
     ControlEvidenceRefreshIsSourceAwareAndMonotonicWithinAHorizon) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(active->finiteExecution() != nullptr);
  const VersionedExecutionInput3D& resident_input =
      *active->finiteExecution()->execution_input;

  const auto recertify = [&](const ExecutionPlan3D& snapshot,
                             const std::uint64_t trajectory_revision,
                             const ExecutionPreviousControlEvidenceSource3D source,
                             const std::uint64_t source_sequence,
                             const std::int64_t source_stamp_ns,
                             const std::int64_t receive_stamp_ns) {
    FiniteExecutionCertification3D certification =
        SnapshotFixture3D::finiteCertificationForRoute(
            *snapshot.route(), FiniteExecutionKind3D::kNominal, trajectory_revision);
    const std::shared_ptr<const VersionedExecutionInput3D>& captured =
        certification.execution_input;
    if (captured == nullptr) {
      return std::optional<FiniteExecutionState3D>{};
    }
    certification.execution_input =
        VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
            .capture_sequence = captured->captureSequence(),
            .pose_revision = captured->poseRevision(),
            .pose_source_timestamp_us = captured->poseSourceTimestampUs(),
            .pose_receive_stamp_ns = captured->poseReceiveStampNs(),
            .effective_stamp_ns = captured->effectiveStampNs(),
            .state = captured->state(),
            .full_state_authoritative = captured->fullStateAuthoritative(),
            .state_provenance = captured->stateProvenance(),
            .previous_control = captured->previousControl(),
            .previous_control_source = source,
            .previous_control_source_producer_instance_id =
                source == ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback
                    ? 91U
                    : 0U,
            .previous_control_source_sequence = source_sequence,
            .previous_control_source_stamp_ns = source_stamp_ns,
            .previous_control_receive_stamp_ns = receive_stamp_ns,
        });
    if (certification.execution_input == nullptr) {
      return std::optional<FiniteExecutionState3D>{};
    }
    return certifyFiniteExecution3D(snapshot, *snapshot.route(),
                                    std::move(certification));
  };

  const auto replace = [&](const ExecutionPlan3D& snapshot,
                           std::optional<FiniteExecutionState3D> candidate) {
    if (!candidate.has_value()) {
      return ExecutionRouteTransitionStatus3D::kInvalidCandidate;
    }
    return replaceFiniteExecution3D(snapshot, SnapshotFixture3D::guard(snapshot),
                                    std::move(candidate))
        .status;
  };

  FiniteExecutionCertification3D exact_interval_certification =
      SnapshotFixture3D::finiteCertificationForRoute(
          *active->route(), FiniteExecutionKind3D::kNominal, 101U);
  exact_interval_certification.execution_input =
      active->finiteExecution()->execution_input;
  EXPECT_EQ(replace(*active,
                    certifyFiniteExecution3D(*active, *active->route(),
                                             std::move(exact_interval_certification))),
            ExecutionRouteTransitionStatus3D::kApplied);
  EXPECT_EQ(
      replace(*active, recertify(*active, 101U, resident_input.previousControlSource(),
                                 resident_input.previousControlSourceSequence(),
                                 resident_input.previousControlSourceStampNs() + 1LL,
                                 resident_input.previousControlReceiveStampNs() + 1LL)),
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  EXPECT_EQ(
      replace(*active, recertify(*active, 101U, resident_input.previousControlSource(),
                                 resident_input.previousControlSourceSequence(),
                                 resident_input.previousControlSourceStampNs() - 1LL,
                                 resident_input.previousControlReceiveStampNs() + 1LL)),
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  constexpr std::uint64_t kHorizonSequence{500U};
  constexpr std::int64_t kFirstSourceStampNs{992'000'000LL};
  constexpr std::int64_t kFirstReceiveStampNs{993'000'000LL};
  const auto exercise_same_horizon_refresh =
      [&](const ExecutionPreviousControlEvidenceSource3D source,
          const ExecutionRouteTransitionStatus3D expected_status) {
        std::optional<FiniteExecutionState3D> switched_candidate =
            recertify(*active, 101U, source, kHorizonSequence, kFirstSourceStampNs,
                      kFirstReceiveStampNs);
        ASSERT_TRUE(switched_candidate.has_value());
        const ExecutionRouteTransitionResult3D switched = replaceFiniteExecution3D(
            *active, SnapshotFixture3D::guard(*active), std::move(switched_candidate));
        ASSERT_TRUE(switched.applied());
        ASSERT_NE(switched.next, nullptr);
        ASSERT_TRUE(switched.next->route() != nullptr);
        ASSERT_TRUE(switched.next->finiteExecution() != nullptr);

        FiniteExecutionCertification3D exact_interval =
            SnapshotFixture3D::finiteCertificationForRoute(
                *switched.next->route(), FiniteExecutionKind3D::kNominal, 102U);
        exact_interval.execution_input =
            switched.next->finiteExecution()->execution_input;
        EXPECT_EQ(replace(*switched.next, certifyFiniteExecution3D(
                                              *switched.next, *switched.next->route(),
                                              std::move(exact_interval))),
                  ExecutionRouteTransitionStatus3D::kApplied);
        EXPECT_EQ(replace(*switched.next,
                          recertify(*switched.next, 102U, source, kHorizonSequence,
                                    kFirstSourceStampNs, kFirstReceiveStampNs + 1LL)),
                  ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
        EXPECT_EQ(
            replace(*switched.next,
                    recertify(*switched.next, 102U, source, kHorizonSequence,
                              kFirstSourceStampNs + 1LL, kFirstReceiveStampNs + 1LL)),
            expected_status);
      };

  exercise_same_horizon_refresh(
      ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback,
      ExecutionRouteTransitionStatus3D::kApplied);
  exercise_same_horizon_refresh(
      ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration,
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

TEST(ExecutionRouteSnapshot3DTest,
     AtomicSuccessorReplacementRequiresExecutionCertifiedForTheSuccessor) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  const ExecutionRouteTransitionResult3D following =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(*active));
  ASSERT_TRUE(following.applied());
  ASSERT_TRUE(following.next);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
  successor_activation.geometry = makeGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      TrackingErrorTubeWorld3D{
          .observed_occupancy = &fixture.raw_occupancy,
          .occupied_content_fingerprint =
              fixture.raw_occupancy.occupiedSnapshot().contentFingerprint(),
      });
  successor_activation.decorations = makeDecorations(
      successor_activation.geometry, successor_activation.route_generation);
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const FiniteExecutionState3D predecessor_execution =
      SnapshotFixture3D::finiteExecution(*following.next,
                                         FiniteExecutionKind3D::kNominal, true, 102U);

  const ExecutionRouteTransitionResult3D replacement = replaceCertifiedRoute3D(
      *following.next, SnapshotFixture3D::guard(*following.next), *successor,
      predecessor_execution, testRouteSplice(*following.next->route(), *successor));

  EXPECT_EQ(replacement.status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  EXPECT_FALSE(replacement.next);
  EXPECT_EQ(following.next->route()->identity.generation,
            SnapshotFixture3D::kRouteGeneration);

  const FiniteExecutionState3D successor_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *following.next, *successor, FiniteExecutionKind3D::kNominal, true, 102U);
  CertifiedRouteSplice3D tampered_splice =
      testRouteSplice(*following.next->route(), *successor);
  ++tampered_splice.successor_geometry_revision;
  const ExecutionRouteTransitionResult3D tampered = replaceCertifiedRoute3D(
      *following.next, SnapshotFixture3D::guard(*following.next), *successor,
      successor_execution, tampered_splice);
  EXPECT_EQ(tampered.status, ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  EXPECT_EQ(tampered.detail, ExecutionRouteTransitionDetail3D::kSpliceNotReady);
  const ExecutionRouteTransitionResult3D accepted = replaceCertifiedRoute3D(
      *following.next, SnapshotFixture3D::guard(*following.next), *successor,
      successor_execution, testRouteSplice(*following.next->route(), *successor));
  ASSERT_TRUE(accepted.applied());
  ASSERT_TRUE(accepted.next->route() != nullptr);
  EXPECT_EQ(accepted.next->route()->identity.generation,
            SnapshotFixture3D::kRouteGeneration + 1U);
  EXPECT_EQ(accepted.next->route()->owner.id, following.next->route()->owner.id);
  EXPECT_EQ(accepted.next->execution_owner_epoch,
            following.next->execution_owner_epoch);
  ASSERT_TRUE(accepted.next->finiteExecution() != nullptr);
  EXPECT_EQ(accepted.next->finiteExecution()->source_route_generation,
            SnapshotFixture3D::kRouteGeneration + 1U);
}

TEST(ExecutionRouteSnapshot3DTest,
     AtomicNewObjectiveHandoffChangesOwnerWithoutInventingASplice) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  const ExecutionRouteTransitionResult3D following =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(*active));
  ASSERT_TRUE(following.applied());
  ASSERT_TRUE(following.next);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
  successor_activation =
      rebindUnconstrainedDecorations(std::move(successor_activation));
  ++successor_activation.proposal.objective.mission_epoch;
  successor_activation.proposal.intent.id =
      makeRouteIntentId3D(successor_activation.proposal.intent.mission_target,
                          successor_activation.proposal.objective.mission_epoch);
  successor_activation.observation.current_objective =
      successor_activation.proposal.objective;
  successor_activation.continuity_lineage.mission_epoch =
      successor_activation.proposal.objective.mission_epoch;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const FiniteExecutionState3D successor_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *following.next, *successor, FiniteExecutionKind3D::kNominal, true, 102U);

  const ExecutionRouteTransitionResult3D accepted = replaceCertifiedRouteAtHandoff3D(
      *following.next, SnapshotFixture3D::guard(*following.next), *successor,
      successor_execution);

  ASSERT_TRUE(accepted.applied());
  ASSERT_TRUE(accepted.next->route() != nullptr);
  EXPECT_EQ(accepted.next->route()->identity.generation,
            SnapshotFixture3D::kRouteGeneration + 1U);
  EXPECT_GT(accepted.next->execution_owner_epoch,
            following.next->execution_owner_epoch);
  EXPECT_NE(accepted.next->route()->owner.id, following.next->route()->owner.id);
}

} // namespace
} // namespace drone_city_nav
