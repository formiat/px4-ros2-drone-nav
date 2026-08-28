#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(ExecutionRouteSnapshot3DTest,
     ControlEvidenceRefreshIsSourceAwareAndMonotonicWithinAHorizon) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_TRUE(active->finite_execution.has_value());
  const VersionedExecutionInput3D& resident_input =
      *active->finite_execution->execution_input;

  const auto recertify = [&](const ExecutionRouteSnapshot3D& snapshot,
                             const std::uint64_t trajectory_revision,
                             const ExecutionPreviousControlEvidenceSource3D source,
                             const std::uint64_t source_sequence,
                             const std::int64_t source_stamp_ns,
                             const std::int64_t receive_stamp_ns) {
    FiniteExecutionCertification3D certification =
        SnapshotFixture3D::finiteCertificationForRoute(
            *snapshot.route, FiniteExecutionKind3D::kNominal, trajectory_revision);
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
    return certifyFiniteExecution3D(snapshot, *snapshot.route,
                                    std::move(certification));
  };

  const auto replace = [&](const ExecutionRouteSnapshot3D& snapshot,
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
          *active->route, FiniteExecutionKind3D::kNominal, 101U);
  exact_interval_certification.execution_input =
      active->finite_execution->execution_input;
  EXPECT_EQ(replace(*active,
                    certifyFiniteExecution3D(*active, *active->route,
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
        ASSERT_TRUE(switched.next->route.has_value());
        ASSERT_TRUE(switched.next->finite_execution.has_value());

        FiniteExecutionCertification3D exact_interval =
            SnapshotFixture3D::finiteCertificationForRoute(
                *switched.next->route, FiniteExecutionKind3D::kNominal, 102U);
        exact_interval.execution_input =
            switched.next->finite_execution->execution_input;
        EXPECT_EQ(replace(*switched.next, certifyFiniteExecution3D(
                                              *switched.next, *switched.next->route,
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
     RawInvalidationCertificationRequiresTheExactNewerOwnerAndABrakingKind) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  const ExecutionRouteTransitionResult3D following =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(*active));
  ASSERT_TRUE(following.applied());
  ASSERT_NE(following.next, nullptr);
  ASSERT_TRUE(following.next->route.has_value());
  const RouteLifecycleEvent3D invalidation{
      .kind = RouteLifecycleEventKind3D::kRawInvalidated,
      .generation = following.next->route->identity.generation,
      .raw_producer_instance_id = SnapshotFixture3D::kRawProducer,
      .raw_revision = SnapshotFixture3D::kLatestRawRevision + 1U,
  };
  const std::shared_ptr<const VersionedObservedRawWorld3D> newer_world =
      fixture.rawWorld(invalidation.raw_revision);
  ASSERT_NE(newer_world, nullptr);
  const auto certify_against =
      [&](const RouteLifecycleEvent3D& event,
          std::shared_ptr<const VersionedObservedRawWorld3D> world,
          const FiniteExecutionKind3D kind) {
        return certifyRawInvalidatedFiniteExecution3D(
            *following.next,
            RawInvalidatedFiniteExecutionCertification3D{
                .invalidation = event,
                .invalidating_observed_raw_world = std::move(world),
                .finite_execution = SnapshotFixture3D::finiteCertificationForRoute(
                    *following.next->route, kind, 102U),
            });
      };

  const std::optional<FiniteExecutionState3D> retained =
      certify_against(invalidation, newer_world, FiniteExecutionKind3D::kRetained);
  EXPECT_FALSE(retained.has_value());
  const std::optional<FiniteExecutionState3D> emergency = certify_against(
      invalidation, newer_world, FiniteExecutionKind3D::kEmergencyBrakeTail);
  ASSERT_TRUE(emergency.has_value());
  EXPECT_TRUE(emergency->validFor(&*following.next->route));
  EXPECT_EQ(emergency->observed_raw_world, newer_world);
  EXPECT_EQ(emergency->stop_boundary.raw_validated_through_revision,
            invalidation.raw_revision);
  const auto* const retained_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&emergency->certificate);
  const auto* const retained_lineage =
      std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
          &emergency->validation_proof.lineage);
  ASSERT_NE(retained_certificate, nullptr);
  ASSERT_NE(retained_lineage, nullptr);
  EXPECT_EQ(retained_certificate->validated_through_revision,
            SnapshotFixture3D::kLatestRawRevision);
  EXPECT_EQ(retained_certificate->observed_world_content_fingerprint,
            following.next->route->observed_raw_world->contentFingerprint());
  EXPECT_EQ(retained_lineage->validated_through_raw_revision,
            invalidation.raw_revision);
  EXPECT_EQ(retained_lineage->observed_world_content_fingerprint,
            newer_world->contentFingerprint());
  FiniteExecutionCertification3D replayed_progress =
      SnapshotFixture3D::finiteCertificationForRoute(
          *following.next->route, FiniteExecutionKind3D::kEmergencyBrakeTail, 103U);
  replayed_progress.execution_input = following.next->route->progress.execution_input;
  EXPECT_FALSE(certifyRawInvalidatedFiniteExecution3D(
                   *following.next,
                   RawInvalidatedFiniteExecutionCertification3D{
                       .invalidation = invalidation,
                       .invalidating_observed_raw_world = newer_world,
                       .finite_execution = std::move(replayed_progress),
                   })
                   .has_value());
  EXPECT_EQ(replaceFiniteExecution3D(
                *following.next, SnapshotFixture3D::guard(*following.next), emergency)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  const RouteLifecycleEvent3D superseded{
      .kind = RouteLifecycleEventKind3D::kObjectiveSuperseded,
      .generation = following.next->route->identity.generation,
  };
  EXPECT_EQ(retireCertifiedRoute3D(*following.next,
                                   SnapshotFixture3D::guard(*following.next),
                                   superseded, emergency)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  const ExecutionRouteTransitionResult3D retained_retirement =
      retireCertifiedRoute3D(*following.next, SnapshotFixture3D::guard(*following.next),
                             invalidation, emergency);
  ASSERT_TRUE(retained_retirement.applied());
  ASSERT_NE(retained_retirement.next, nullptr);
  ASSERT_TRUE(retained_retirement.next->finite_execution.has_value());
  EXPECT_EQ(retained_retirement.next->phase, ExecutionRoutePhase3D::kBraking);
  EXPECT_EQ(retained_retirement.next->finite_execution->kind,
            FiniteExecutionKind3D::kEmergencyBrakeTail);
  EXPECT_FALSE(
      certify_against(invalidation, nullptr, FiniteExecutionKind3D::kEmergencyBrakeTail)
          .has_value());

  RouteLifecycleEvent3D wrong_event = invalidation;
  wrong_event.kind = RouteLifecycleEventKind3D::kObjectiveSuperseded;
  EXPECT_FALSE(certify_against(wrong_event, newer_world,
                               FiniteExecutionKind3D::kEmergencyBrakeTail)
                   .has_value());
  wrong_event = invalidation;
  ++wrong_event.generation;
  EXPECT_FALSE(certify_against(wrong_event, newer_world,
                               FiniteExecutionKind3D::kEmergencyBrakeTail)
                   .has_value());

  RouteLifecycleEvent3D wrong_revision = invalidation;
  ++wrong_revision.raw_revision;
  EXPECT_FALSE(certify_against(wrong_revision, newer_world,
                               FiniteExecutionKind3D::kEmergencyBrakeTail)
                   .has_value());

  RouteLifecycleEvent3D older = invalidation;
  older.raw_revision = SnapshotFixture3D::kLatestRawRevision;
  EXPECT_FALSE(certify_against(older, fixture.rawWorld(older.raw_revision),
                               FiniteExecutionKind3D::kEmergencyBrakeTail)
                   .has_value());

  constexpr std::uint64_t kDifferentProducer{SnapshotFixture3D::kRawProducer + 1U};
  RouteLifecycleEvent3D different_producer = invalidation;
  different_producer.raw_producer_instance_id = kDifferentProducer;
  EXPECT_FALSE(certify_against(different_producer,
                               fixture.rawWorld(invalidation.raw_revision, nullptr,
                                                kDifferentProducer),
                               FiniteExecutionKind3D::kEmergencyBrakeTail)
                   .has_value());

  ObservedOccupancyGrid3D unsafe_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> blocked_cell =
      unsafe_occupancy.worldToCell(Point3{4.0, 0.0, 5.0});
  ASSERT_TRUE(blocked_cell.has_value());
  ASSERT_TRUE(unsafe_occupancy.setState(*blocked_cell, ObservedVoxelState::kOccupied));
  // The newly occupied voxel lies on the invalidated nominal suffix, but the
  // permanent immediate braking tail stops before reaching it and remains a
  // valid safety fallback.
  EXPECT_TRUE(
      certify_against(invalidation,
                      fixture.rawWorld(invalidation.raw_revision, &unsafe_occupancy),
                      FiniteExecutionKind3D::kEmergencyBrakeTail)
          .has_value());

  const mppi::State& current_state =
      following.next->route->progress.execution_input->state();
  const Point3 current_position{current_state.x, current_state.y, current_state.z};
  const std::optional<GridIndex3D> current_cell =
      unsafe_occupancy.worldToCell(current_position);
  ASSERT_TRUE(current_cell.has_value());
  ASSERT_TRUE(unsafe_occupancy.setState(*current_cell, ObservedVoxelState::kOccupied));
  EXPECT_FALSE(
      certify_against(invalidation,
                      fixture.rawWorld(invalidation.raw_revision, &unsafe_occupancy),
                      FiniteExecutionKind3D::kEmergencyBrakeTail)
          .has_value());

  EXPECT_FALSE(
      certify_against(invalidation, newer_world, FiniteExecutionKind3D::kNominal)
          .has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     RawInvalidationCertificationRederivesConstrainedPassageGeometry) {
  SnapshotFixture3D fixture;
  fixture.geometry = makeConstrainedGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      SnapshotFixture3D::kRouteGeneration, fixture.raw_occupancy.occupiedSnapshot(),
      testPassageVolumeConfig());
  fixture.geometry_revision = fixture.geometry->executable_geometry_revision;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());

  ObservedOccupancyGrid3D changed_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> lateral_wall =
      changed_occupancy.worldToCell(Point3{5.0, 2.0, 5.0});
  ASSERT_TRUE(lateral_wall.has_value());
  ASSERT_TRUE(changed_occupancy.setState(*lateral_wall, ObservedVoxelState::kOccupied));
  const RouteLifecycleEvent3D invalidation{
      .kind = RouteLifecycleEventKind3D::kRawInvalidated,
      .generation = active->route->identity.generation,
      .raw_producer_instance_id = SnapshotFixture3D::kRawProducer,
      .raw_revision = SnapshotFixture3D::kLatestRawRevision + 1U,
  };

  EXPECT_FALSE(
      certifyRawInvalidatedFiniteExecution3D(
          *active,
          RawInvalidatedFiniteExecutionCertification3D{
              .invalidation = invalidation,
              .invalidating_observed_raw_world =
                  fixture.rawWorld(invalidation.raw_revision, &changed_occupancy),
              .finite_execution = SnapshotFixture3D::finiteCertificationForRoute(
                  *active->route, FiniteExecutionKind3D::kEmergencyBrakeTail, 101U),
          })
          .has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     RawInvalidationRetirementAtomicallyCreditsTheCertifiedCurrentConnector) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  const ExecutionRouteTransitionResult3D following =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(*active));
  ASSERT_TRUE(following.applied());
  ASSERT_NE(following.next, nullptr);
  ASSERT_TRUE(following.next->route.has_value());
  const RouteLifecycleEvent3D invalidation{
      .kind = RouteLifecycleEventKind3D::kRawInvalidated,
      .generation = following.next->route->identity.generation,
      .raw_producer_instance_id = SnapshotFixture3D::kRawProducer,
      .raw_revision = SnapshotFixture3D::kLatestRawRevision + 1U,
  };
  const std::shared_ptr<const VersionedObservedRawWorld3D> invalidating_world =
      fixture.rawWorld(invalidation.raw_revision);
  ASSERT_NE(invalidating_world, nullptr);
  ObservedOccupancyGrid3D blocked_connector_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> blocked_connector_cell =
      blocked_connector_occupancy.worldToCell(Point3{3.0, 0.0, 5.0});
  ASSERT_TRUE(blocked_connector_cell.has_value());
  ASSERT_TRUE(blocked_connector_occupancy.setState(*blocked_connector_cell,
                                                   ObservedVoxelState::kOccupied));
  EXPECT_FALSE(
      certifyRawInvalidatedFiniteExecution3D(
          *following.next,
          RawInvalidatedFiniteExecutionCertification3D{
              .invalidation = invalidation,
              .invalidating_observed_raw_world = fixture.rawWorld(
                  invalidation.raw_revision, &blocked_connector_occupancy),
              .finite_execution = SnapshotFixture3D::finiteCertificationForRoute(
                  *following.next->route, FiniteExecutionKind3D::kEmergencyBrakeTail,
                  102U, 56U, 0U, 4.0),
          })
          .has_value());
  const FiniteExecutionState3D braking =
      SnapshotFixture3D::rawInvalidatedFiniteExecution(
          *following.next, invalidation, invalidating_world,
          FiniteExecutionKind3D::kEmergencyBrakeTail, true, 102U, 56U, 1U, 4.0);
  EXPECT_DOUBLE_EQ(braking.begin_route_station_m, 4.0);
  EXPECT_GT(braking.valid_until_ns, following.next->finite_execution->valid_until_ns);
  EXPECT_FALSE(braking.validFor(&*following.next->route));
  EXPECT_TRUE(braking.validFor(nullptr));

  const ExecutionRouteTransitionResult3D retired =
      retireCertifiedRoute3D(*following.next, SnapshotFixture3D::guard(*following.next),
                             invalidation, braking);

  ASSERT_TRUE(retired.applied());
  ASSERT_NE(retired.next, nullptr);
  ASSERT_TRUE(retired.next->route.has_value());
  ASSERT_TRUE(retired.next->finite_execution.has_value());
  EXPECT_TRUE(retired.next->valid());
  EXPECT_EQ(retired.next->phase, ExecutionRoutePhase3D::kBraking);
  EXPECT_DOUBLE_EQ(retired.next->route->progress.station_m, 4.0);
  EXPECT_DOUBLE_EQ(retired.next->route->progress.last_observed_position.x, 4.0);
  EXPECT_EQ(retired.next->route->progress.execution_input,
            retired.next->finite_execution->execution_input);
  EXPECT_EQ(retired.next->route->progress.execution_input->contentFingerprint(),
            braking.execution_input->contentFingerprint());
  EXPECT_DOUBLE_EQ(retired.next->finite_execution->begin_route_station_m, 4.0);
  EXPECT_DOUBLE_EQ(following.next->route->progress.station_m, 2.0);
}

TEST(ExecutionRouteSnapshot3DTest,
     RetirementKeepsRouteOwnershipAndRequiresASafeBrakingArtifact) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  const ExecutionRouteTransitionResult3D following =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(*active));
  ASSERT_TRUE(following.applied());
  ASSERT_TRUE(following.next);
  const RouteLifecycleEvent3D invalidated{
      .kind = RouteLifecycleEventKind3D::kRawInvalidated,
      .generation = following.next->route->identity.generation,
      .raw_producer_instance_id = SnapshotFixture3D::kRawProducer,
      .raw_revision = SnapshotFixture3D::kLatestRawRevision + 1U,
  };

  const ExecutionRouteTransitionResult3D rejected =
      retireCertifiedRoute3D(*following.next, SnapshotFixture3D::guard(*following.next),
                             invalidated, std::nullopt);
  EXPECT_EQ(rejected.status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  EXPECT_FALSE(rejected.next);
  ASSERT_TRUE(following.next->route.has_value());

  const FiniteExecutionState3D stale_braking = SnapshotFixture3D::finiteExecution(
      *following.next, FiniteExecutionKind3D::kEmergencyBrakeTail, true, 102U);
  EXPECT_EQ(retireCertifiedRoute3D(*following.next,
                                   SnapshotFixture3D::guard(*following.next),
                                   invalidated, stale_braking)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  const std::shared_ptr<const VersionedObservedRawWorld3D> invalidating_world =
      fixture.rawWorld(invalidated.raw_revision);
  ASSERT_NE(invalidating_world, nullptr);
  RouteLifecycleEvent3D later_invalidation = invalidated;
  ++later_invalidation.raw_revision;
  const std::shared_ptr<const VersionedObservedRawWorld3D> later_world =
      fixture.rawWorld(later_invalidation.raw_revision);
  ASSERT_NE(later_world, nullptr);
  const FiniteExecutionState3D later_world_braking =
      SnapshotFixture3D::rawInvalidatedFiniteExecution(
          *following.next, later_invalidation, later_world,
          FiniteExecutionKind3D::kEmergencyBrakeTail, true, 103U);
  EXPECT_EQ(retireCertifiedRoute3D(*following.next,
                                   SnapshotFixture3D::guard(*following.next),
                                   invalidated, later_world_braking)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  const FiniteExecutionState3D braking =
      SnapshotFixture3D::rawInvalidatedFiniteExecution(
          *following.next, invalidated, invalidating_world,
          FiniteExecutionKind3D::kEmergencyBrakeTail, true, 102U);
  const ExecutionRouteTransitionResult3D retired = retireCertifiedRoute3D(
      *following.next, SnapshotFixture3D::guard(*following.next), invalidated, braking);

  ASSERT_TRUE(retired.applied());
  ASSERT_TRUE(retired.next);
  EXPECT_TRUE(retired.next->valid());
  EXPECT_EQ(retired.next->phase, ExecutionRoutePhase3D::kBraking);
  ASSERT_TRUE(retired.next->route.has_value());
  EXPECT_EQ(retired.next->route->identity.generation,
            following.next->route->identity.generation);
  ASSERT_TRUE(retired.next->finite_execution.has_value());
  EXPECT_EQ(retired.next->finite_execution->kind,
            FiniteExecutionKind3D::kEmergencyBrakeTail);
  EXPECT_EQ(executionRouteEndpointSemantics3D(*retired.next),
            RouteEndpointSemantics3D::kEmergencyBrakeTail);

  const FiniteExecutionState3D nominal_resurrection =
      SnapshotFixture3D::finiteExecution(*retired.next, FiniteExecutionKind3D::kNominal,
                                         true, 103U);
  EXPECT_EQ(replaceFiniteExecution3D(*retired.next,
                                     SnapshotFixture3D::guard(*retired.next),
                                     nominal_resurrection)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

TEST(ExecutionRouteSnapshot3DTest,
     CompletedContinuationRetainsItsOwnerWithoutAllowingResurrection) {
  SnapshotFixture3D fixture;
  ExecutionRouteActivation3D continuation_activation = fixture.activation();
  continuation_activation.proposal.reaches_mission_goal = false;
  continuation_activation.proposal.evidence.reaches_mission_target = false;
  continuation_activation.geometry =
      withTerminalMppiSpeed(continuation_activation.geometry, 4.0F);
  const std::optional<CertifiedRouteSuffix3D> suffix =
      certifyExecutionRoute3D(continuation_activation);
  ASSERT_TRUE(suffix.has_value());
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  FiniteExecutionState3D initial_execution = SnapshotFixture3D::finiteExecutionForRoute(
      *initial, *suffix, FiniteExecutionKind3D::kNominal, true, 100U);
  const ExecutionRouteTransitionResult3D active = activateCertifiedRoute3D(
      *initial, initial->version, suffix.value(), std::move(initial_execution));
  ASSERT_TRUE(active.applied());
  const ExecutionRouteTransitionResult3D at_endpoint = advanceCertifiedRoute3D(
      *active.next, SnapshotFixture3D::guard(*active.next),
      fixture.executionObservation({10.0, 0.0, 5.0},
                                   SnapshotFixture3D::kLatestRawRevision + 1U,
                                   &fixture.raw_occupancy),
      SnapshotFixture3D::progressInput(*active.next, {10.0, 0.0, 5.0}),
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U));
  ASSERT_TRUE(at_endpoint.applied());
  const ExecutionRouteTransitionResult3D following = replaceFiniteExecution3D(
      *at_endpoint.next, SnapshotFixture3D::guard(*at_endpoint.next),
      SnapshotFixture3D::finiteExecution(*at_endpoint.next));
  ASSERT_TRUE(following.applied());
  const RouteLifecycleEvent3D completed{
      .kind = RouteLifecycleEventKind3D::kCompleted,
      .generation = SnapshotFixture3D::kRouteGeneration,
  };

  const ExecutionRouteTransitionResult3D awaiting =
      retireCertifiedRoute3D(*following.next, SnapshotFixture3D::guard(*following.next),
                             completed, std::nullopt);

  ASSERT_TRUE(awaiting.applied());
  EXPECT_EQ(awaiting.next->phase, ExecutionRoutePhase3D::kAwaitingSuccessor);
  EXPECT_TRUE(awaiting.next->route.has_value());
  EXPECT_TRUE(awaiting.next->finite_execution.has_value());
  EXPECT_EQ(replaceFiniteExecution3D(
                *awaiting.next, SnapshotFixture3D::guard(*awaiting.next), std::nullopt)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  const FiniteExecutionState3D emergency = SnapshotFixture3D::finiteExecution(
      *awaiting.next, FiniteExecutionKind3D::kEmergencyBrakeTail, true, 102U);
  EXPECT_EQ(replaceFiniteExecution3D(
                *awaiting.next, SnapshotFixture3D::guard(*awaiting.next), emergency)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  const FiniteExecutionState3D refreshed = SnapshotFixture3D::finiteExecution(
      *awaiting.next, FiniteExecutionKind3D::kNominal, true, 102U);
  const ExecutionRouteTransitionResult3D still_awaiting = replaceFiniteExecution3D(
      *awaiting.next, SnapshotFixture3D::guard(*awaiting.next), refreshed);
  ASSERT_TRUE(still_awaiting.applied());
  EXPECT_EQ(still_awaiting.next->phase, ExecutionRoutePhase3D::kAwaitingSuccessor);
}

TEST(ExecutionRouteSnapshot3DTest, RejectsPrematureRouteCompletion) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  const RouteLifecycleEvent3D completed{
      .kind = RouteLifecycleEventKind3D::kCompleted,
      .generation = SnapshotFixture3D::kRouteGeneration,
  };

  EXPECT_EQ(retireCertifiedRoute3D(*active, SnapshotFixture3D::guard(*active),
                                   completed, std::nullopt)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

TEST(ExecutionRouteSnapshot3DTest,
     AtomicSuccessorReplacementRequiresExecutionCertifiedForTheSuccessor) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
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
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const FiniteExecutionState3D predecessor_execution =
      SnapshotFixture3D::finiteExecution(*following.next,
                                         FiniteExecutionKind3D::kNominal, true, 102U);

  const ExecutionRouteTransitionResult3D replacement = replaceCertifiedRoute3D(
      *following.next, SnapshotFixture3D::guard(*following.next), *successor,
      predecessor_execution, testRouteSplice(*following.next->route, *successor));

  EXPECT_EQ(replacement.status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  EXPECT_FALSE(replacement.next);
  EXPECT_EQ(following.next->route->identity.generation,
            SnapshotFixture3D::kRouteGeneration);

  const FiniteExecutionState3D successor_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *following.next, *successor, FiniteExecutionKind3D::kNominal, true, 102U);
  CertifiedRouteSplice3D tampered_splice =
      testRouteSplice(*following.next->route, *successor);
  ++tampered_splice.successor_geometry_revision;
  EXPECT_EQ(replaceCertifiedRoute3D(*following.next,
                                    SnapshotFixture3D::guard(*following.next),
                                    *successor, successor_execution, tampered_splice)
                .status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  const ExecutionRouteTransitionResult3D accepted = replaceCertifiedRoute3D(
      *following.next, SnapshotFixture3D::guard(*following.next), *successor,
      successor_execution, testRouteSplice(*following.next->route, *successor));
  ASSERT_TRUE(accepted.applied());
  ASSERT_TRUE(accepted.next->route.has_value());
  EXPECT_EQ(accepted.next->route->identity.generation,
            SnapshotFixture3D::kRouteGeneration + 1U);
  EXPECT_EQ(accepted.next->route->owner.id, following.next->route->owner.id);
  EXPECT_EQ(accepted.next->execution_owner_epoch,
            following.next->execution_owner_epoch);
  ASSERT_TRUE(accepted.next->finite_execution.has_value());
  EXPECT_EQ(accepted.next->finite_execution->source_route_generation,
            SnapshotFixture3D::kRouteGeneration + 1U);
}

TEST(ExecutionRouteSnapshot3DTest,
     AtomicNewObjectiveHandoffChangesOwnerWithoutInventingASplice) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  const ExecutionRouteTransitionResult3D following =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(*active));
  ASSERT_TRUE(following.applied());
  ASSERT_TRUE(following.next);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
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
  ASSERT_TRUE(accepted.next->route.has_value());
  EXPECT_EQ(accepted.next->route->identity.generation,
            SnapshotFixture3D::kRouteGeneration + 1U);
  EXPECT_GT(accepted.next->execution_owner_epoch,
            following.next->execution_owner_epoch);
  EXPECT_NE(accepted.next->route->owner.id, following.next->route->owner.id);
}

TEST(ExecutionRouteSnapshot3DTest,
     SuccessorRequiresFreshEvidenceAndRejectsUnauthenticatedProducerSwitch) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  const ExecutionRouteTransitionResult3D following =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(*active));
  ASSERT_TRUE(following.applied());
  const std::uint64_t current_raw_revision = SnapshotFixture3D::kLatestRawRevision + 1U;
  const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
      *following.next, SnapshotFixture3D::guard(*following.next),
      fixture.executionObservation({4.0, 0.0, 5.0}, current_raw_revision,
                                   &fixture.raw_occupancy),
      SnapshotFixture3D::progressInput(*following.next, {4.0, 0.0, 5.0}),
      fixture.rawWorld(current_raw_revision));
  ASSERT_TRUE(advanced.applied());

  ExecutionRouteActivation3D stale_activation = fixture.activation();
  stale_activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
  stale_activation.observation.position = {4.0, 0.0, 5.0};
  const std::optional<CertifiedRouteSuffix3D> stale_successor =
      certifyExecutionRoute3D(stale_activation);
  ASSERT_TRUE(stale_successor.has_value());
  const FiniteExecutionState3D stale_execution =
      SnapshotFixture3D::finiteExecutionForRoute(*advanced.next, *stale_successor,
                                                 FiniteExecutionKind3D::kNominal, true,
                                                 102U);
  EXPECT_EQ(
      replaceCertifiedRoute3D(*advanced.next, SnapshotFixture3D::guard(*advanced.next),
                              *stale_successor, stale_execution,
                              testRouteSplice(*advanced.next->route, *stale_successor))
          .status,
      ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  ExecutionRouteActivation3D fresh_activation = stale_activation;
  fresh_activation.observation.latest_raw_revision = current_raw_revision;
  fresh_activation.observed_raw_world = fixture.rawWorld(current_raw_revision);
  const std::optional<CertifiedRouteSuffix3D> fresh_successor =
      certifyExecutionRoute3D(fresh_activation);
  ASSERT_TRUE(fresh_successor.has_value());
  const FiniteExecutionState3D fresh_execution =
      SnapshotFixture3D::finiteExecutionForRoute(*advanced.next, *fresh_successor,
                                                 FiniteExecutionKind3D::kNominal, true,
                                                 102U);
  const ExecutionRouteTransitionResult3D accepted = replaceCertifiedRoute3D(
      *advanced.next, SnapshotFixture3D::guard(*advanced.next), *fresh_successor,
      fresh_execution, testRouteSplice(*advanced.next->route, *fresh_successor));
  EXPECT_TRUE(accepted.applied());

  constexpr std::uint64_t kUnauthenticatedProducer{SnapshotFixture3D::kRawProducer +
                                                   100U};
  ExecutionRouteActivation3D switched_activation = stale_activation;
  switched_activation.proposal.planned_world.producer_instance_id =
      kUnauthenticatedProducer;
  switched_activation.proposal.validated_world.producer_instance_id =
      kUnauthenticatedProducer;
  switched_activation.observation.resident_world =
      switched_activation.proposal.validated_world;
  switched_activation.observation.latest_raw_producer_instance_id =
      kUnauthenticatedProducer;
  switched_activation.observed_raw_world = fixture.rawWorld(
      SnapshotFixture3D::kLatestRawRevision, nullptr, kUnauthenticatedProducer);
  const std::optional<CertifiedRouteSuffix3D> switched_successor =
      certifyExecutionRoute3D(switched_activation);
  ASSERT_TRUE(switched_successor.has_value());
  const FiniteExecutionState3D switched_execution =
      SnapshotFixture3D::finiteExecutionForRoute(*advanced.next, *switched_successor,
                                                 FiniteExecutionKind3D::kNominal, true,
                                                 102U);
  EXPECT_EQ(replaceCertifiedRoute3D(
                *advanced.next, SnapshotFixture3D::guard(*advanced.next),
                *switched_successor, switched_execution,
                testRouteSplice(*advanced.next->route, *switched_successor))
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

} // namespace
} // namespace drone_city_nav
