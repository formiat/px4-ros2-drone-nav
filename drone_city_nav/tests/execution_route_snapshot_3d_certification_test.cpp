
#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(ExecutionRouteSnapshot3DTest,
     FiniteExecutionRecognizesACopiedCertifiedRouteByInstanceId) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  CertifiedRouteSuffix3D copied_route = *active->route;
  copied_route.geometry =
      std::make_shared<const ExecutionRouteGeometry3D>(*active->route->geometry);
  ASSERT_NE(copied_route.geometry, active->route->geometry);
  ASSERT_TRUE(copied_route.valid());

  const std::optional<FiniteExecutionState3D> certified = certifyFiniteExecution3D(
      *active, copied_route,
      SnapshotFixture3D::finiteCertificationForRoute(copied_route));

  ASSERT_TRUE(certified.has_value());
  EXPECT_EQ(certified->source_route_instance_id, active->route->route_instance_id);

  const std::optional<CertifiedRouteSuffix3D> independently_certified =
      fixture.certify();
  ASSERT_TRUE(independently_certified.has_value());
  ASSERT_NE(independently_certified->route_instance_id,
            active->route->route_instance_id);
  EXPECT_FALSE(certifyFiniteExecution3D(*active, *independently_certified,
                                        SnapshotFixture3D::finiteCertificationForRoute(
                                            *independently_certified))
                   .has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     StaticCertificationSweepsTheExactWorldAndBindsProgressFootprint) {
  SnapshotFixture3D fixture;
  OccupancyGrid3D blocked_static{fixture.raw_occupancy.bounds(),
                                 fixture.validated_world.esdf_fingerprint};
  const std::optional<GridIndex3D> blocked_cell =
      blocked_static.worldToCell(Point3{5.0, 0.0, 5.0});
  ASSERT_TRUE(blocked_cell.has_value());
  blocked_static.setOccupied(*blocked_cell);
  EXPECT_FALSE(
      certifyExecutionRoute3D(staticActivation(fixture, blocked_static)).has_value());

  const OccupancyGrid3D truncated_static{GridBounds3D{-5.0, -5.0, 0.0, 1.0, 15, 10, 10},
                                         fixture.validated_world.esdf_fingerprint};
  EXPECT_FALSE(
      certifyExecutionRoute3D(staticActivation(fixture, truncated_static)).has_value());

  const OccupancyGrid3D clear_static{fixture.raw_occupancy.bounds(),
                                     fixture.validated_world.esdf_fingerprint};
  const std::optional<CertifiedRouteSuffix3D> suffix =
      certifyExecutionRoute3D(staticActivation(fixture, clear_static));
  ASSERT_TRUE(suffix.has_value());
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_TRUE(initial);
  FiniteExecutionState3D initial_execution = SnapshotFixture3D::finiteExecutionForRoute(
      *initial, *suffix, FiniteExecutionKind3D::kNominal, true, 100U);
  const ExecutionRouteTransitionResult3D active = activateCertifiedRoute3D(
      *initial, initial->version, *suffix, std::move(initial_execution));
  ASSERT_TRUE(active.applied());

  RouteExecutionObservation3D changed_footprint =
      fixture.executionObservation(Point3{4.0, 0.0, 5.0}, 0U, &fixture.raw_occupancy);
  changed_footprint.footprint.radius_m = 0.1;
  EXPECT_EQ(advanceCertifiedRoute3D(
                *active.next, SnapshotFixture3D::guard(*active.next), changed_footprint,
                SnapshotFixture3D::progressInput(*active.next, {4.0, 0.0, 5.0}),
                nullptr)
                .status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);
}

TEST(ExecutionRouteSnapshot3DTest, DerivesPlannedEndpointSemanticsFromTheProposal) {
  SnapshotFixture3D fixture;
  ExecutionRouteActivation3D activation = fixture.activation();

  const std::optional<CertifiedRouteSuffix3D> mission =
      certifyExecutionRoute3D(activation);
  ASSERT_TRUE(mission.has_value());
  EXPECT_EQ(mission->planned_endpoint_semantics,
            RouteEndpointSemantics3D::kMissionStop);
  CertifiedRouteSuffix3D tampered_mission = *mission;
  tampered_mission.planned_endpoint_semantics = RouteEndpointSemantics3D::kContinuation;
  EXPECT_FALSE(tampered_mission.valid());
  tampered_mission.planned_endpoint_semantics =
      static_cast<RouteEndpointSemantics3D>(255U);
  EXPECT_FALSE(tampered_mission.valid());

  activation.proposal.reaches_mission_goal = false;
  activation.proposal.evidence.reaches_mission_target = false;
  activation.geometry = withTerminalMppiSpeed(activation.geometry, 4.0F);
  const std::optional<CertifiedRouteSuffix3D> continuation =
      certifyExecutionRoute3D(activation);
  ASSERT_TRUE(continuation.has_value());
  EXPECT_EQ(continuation->planned_endpoint_semantics,
            RouteEndpointSemantics3D::kContinuation);
}

TEST(ExecutionRouteSnapshot3DTest,
     ContinuousTrackingMissionEndpointRemainsAContinuation) {
  SnapshotFixture3D fixture;
  ExecutionRouteActivation3D activation = fixture.activation();
  activation.proposal.objective.continuous_tracking = true;
  activation.proposal.objective.target_detection_id = 7U;
  activation.proposal.objective.target_track_id = 8U;
  activation.observation.current_objective.continuous_tracking = true;
  activation.observation.current_objective.target_detection_id = 7U;
  activation.observation.current_objective.target_track_id = 8U;
  activation.geometry = withTerminalMppiSpeed(activation.geometry, 4.0F);

  const std::optional<CertifiedRouteSuffix3D> continuation =
      certifyExecutionRoute3D(activation);

  ASSERT_TRUE(continuation.has_value());
  EXPECT_EQ(continuation->planned_endpoint_semantics,
            RouteEndpointSemantics3D::kContinuation);
  EXPECT_TRUE(continuation->valid());
}

TEST(ExecutionRouteSnapshot3DTest,
     EndpointSemanticsRejectsAMismatchedNominalSpeedProfile) {
  SnapshotFixture3D fixture;
  ExecutionRouteActivation3D continuation = fixture.activation();
  continuation.proposal.reaches_mission_goal = false;
  continuation.proposal.evidence.reaches_mission_target = false;
  EXPECT_FALSE(certifyExecutionRoute3D(continuation).has_value());

  ExecutionRouteActivation3D mission = fixture.activation();
  mission.geometry = withTerminalMppiSpeed(mission.geometry, 4.0F);
  EXPECT_FALSE(certifyExecutionRoute3D(mission).has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     RawCertificateIsProducedByTheActualActivationAssessment) {
  SnapshotFixture3D fixture;
  ASSERT_TRUE(
      fixture.raw_occupancy.setState({12, 5, 5}, ObservedVoxelState::kOccupied));

  EXPECT_FALSE(fixture.certify().has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     RouteCertificationRequiresAnOwnedPolicyCoveredByTheRouteFootprint) {
  SnapshotFixture3D fixture;
  ExecutionRouteActivation3D missing = fixture.activation();
  missing.validation_policy.reset();
  EXPECT_FALSE(certifyExecutionRoute3D(missing).has_value());

  SweptFootprintConfig oversized = fixture.validation_policy->sweptFootprint();
  oversized.radius_m = missing.observation.footprint.radius_m + 0.5;
  oversized.perimeter_samples = std::max<std::size_t>(oversized.perimeter_samples, 8U);
  oversized.radial_rings = std::max<std::size_t>(oversized.radial_rings, 1U);
  ExecutionRouteActivation3D uncovered = fixture.activation();
  uncovered.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      fixture.validation_policy->flightEnvelope(),
      fixture.validation_policy->dynamics(),
      fixture.validation_policy->altitudeEnvelope(), oversized,
      fixture.validation_policy->latestLidarMaximumAgeMs());
  ASSERT_NE(uncovered.validation_policy, nullptr);
  EXPECT_FALSE(certifyExecutionRoute3D(uncovered).has_value());

  const auto expect_uncovered_policy_rejected = [&](SweptFootprintConfig required) {
    ExecutionRouteActivation3D candidate = fixture.activation();
    candidate.validation_policy = VersionedExecutionValidationPolicy3D::capture(
        fixture.validation_policy->flightEnvelope(),
        fixture.validation_policy->dynamics(),
        fixture.validation_policy->altitudeEnvelope(), required,
        fixture.validation_policy->latestLidarMaximumAgeMs());
    ASSERT_NE(candidate.validation_policy, nullptr);
    EXPECT_FALSE(certifyExecutionRoute3D(candidate).has_value());
  };

  SweptFootprintConfig denser_perimeter = fixture.validation_policy->sweptFootprint();
  denser_perimeter.perimeter_samples += 1U;
  expect_uncovered_policy_rejected(denser_perimeter);

  SweptFootprintConfig denser_radial = fixture.validation_policy->sweptFootprint();
  denser_radial.radial_rings += 1U;
  expect_uncovered_policy_rejected(denser_radial);

  SweptFootprintConfig denser_axial = fixture.validation_policy->sweptFootprint();
  denser_axial.axial_samples += 1U;
  expect_uncovered_policy_rejected(denser_axial);

  SweptFootprintConfig finer_sweep = fixture.validation_policy->sweptFootprint();
  finer_sweep.sweep_step_m *= 0.5;
  expect_uncovered_policy_rejected(finer_sweep);

  SweptFootprintConfig greater_clearance = fixture.validation_policy->sweptFootprint();
  greater_clearance.safe_clearance_threshold_m += 0.1;
  expect_uncovered_policy_rejected(greater_clearance);

  SweptFootprintConfig larger_than_tube_profile =
      fixture.geometry->tracking_error_tube->physical_footprint;
  larger_than_tube_profile.radius_m += 0.1;
  ExecutionRouteActivation3D profile_uncovered = fixture.activation();
  profile_uncovered.observation.footprint = larger_than_tube_profile;
  profile_uncovered.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      fixture.validation_policy->flightEnvelope(),
      fixture.validation_policy->dynamics(),
      fixture.validation_policy->altitudeEnvelope(), larger_than_tube_profile,
      fixture.validation_policy->latestLidarMaximumAgeMs());
  ASSERT_NE(profile_uncovered.validation_policy, nullptr);
  EXPECT_FALSE(certifyExecutionRoute3D(profile_uncovered).has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteCertificationUsesRouteOwnedWorldPolicyAndExactExecutionEvidence) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ASSERT_TRUE(active->route.has_value());
  const CertifiedRouteSuffix3D& route = *active->route;
  const FiniteExecutionState3D baseline = SnapshotFixture3D::finiteExecution(*active);
  ASSERT_TRUE(baseline.horizon);

  const auto make_certification =
      [&](mppi::FiniteHorizon horizon,
          std::shared_ptr<const VersionedExecutionInput3D> execution_input,
          std::shared_ptr<const VersionedLatestLidarEvidence3D> lidar_evidence) {
        return FiniteExecutionCertification3D{
            .trajectory_revision = 102U,
            .horizon = std::move(horizon),
            .execution_input = std::move(execution_input),
            .latest_lidar_evidence = std::move(lidar_evidence),
            .valid_from_ns = 1'000'000'000LL,
            .kind = FiniteExecutionKind3D::kNominal,
        };
      };

  const FiniteExecutionCertificationResult3D accepted =
      certifyFiniteExecution3DDetailed(
          *active, route,
          make_certification(*baseline.horizon, baseline.execution_input,
                             baseline.latest_lidar_evidence));
  ASSERT_TRUE(accepted.certified());
  ASSERT_TRUE(accepted.execution.has_value());
  EXPECT_EQ(accepted.status, FiniteExecutionCertificationStatus3D::kCertified);
  EXPECT_EQ(finiteExecutionCertificationStatus3DName(accepted.status), "certified");
  EXPECT_EQ(accepted.route_adherence_status,
            FiniteExecutionRouteAdherenceStatus3D::kAccepted);
  EXPECT_EQ(finiteExecutionRouteAdherenceStatus3DName(accepted.route_adherence_status),
            "accepted");
  EXPECT_NE(accepted.execution->validation_proof.validation_contract_fingerprint, 0U);
  EXPECT_NE(accepted.execution->validation_proof.artifact_fingerprint, 0U);

  mppi::FiniteHorizon inconsistent = *baseline.horizon;
  inconsistent.states.at(1).x += 1.0F;
  const FiniteExecutionCertificationResult3D inconsistent_result =
      certifyFiniteExecution3DDetailed(
          *active, route,
          make_certification(std::move(inconsistent), baseline.execution_input,
                             baseline.latest_lidar_evidence));
  EXPECT_FALSE(inconsistent_result.certified());
  EXPECT_FALSE(inconsistent_result.execution.has_value());
  EXPECT_EQ(inconsistent_result.status,
            FiniteExecutionCertificationStatus3D::kHorizonContractRejected);
  EXPECT_EQ(finiteExecutionCertificationStatus3DName(inconsistent_result.status),
            "horizon_contract_rejected");
  EXPECT_EQ(inconsistent_result.route_adherence_status,
            FiniteExecutionRouteAdherenceStatus3D::kNotEvaluated);

  ASSERT_NE(baseline.execution_input, nullptr);
  ExecutionInputCapture3D mismatched_input_capture{
      .capture_sequence = 103U,
      .pose_revision = baseline.execution_input->poseRevision(),
      .pose_source_timestamp_us = baseline.execution_input->poseSourceTimestampUs(),
      .pose_receive_stamp_ns = baseline.execution_input->poseReceiveStampNs(),
      .effective_stamp_ns = baseline.execution_input->effectiveStampNs(),
      .state = baseline.execution_input->state(),
      .full_state_authoritative = true,
      .state_provenance = baseline.execution_input->stateProvenance(),
      .previous_control = baseline.execution_input->previousControl(),
      .previous_control_source = baseline.execution_input->previousControlSource(),
      .previous_control_source_producer_instance_id =
          baseline.execution_input->previousControlSourceProducerInstanceId(),
      .previous_control_source_sequence =
          baseline.execution_input->previousControlSourceSequence(),
      .previous_control_source_stamp_ns =
          baseline.execution_input->previousControlSourceStampNs(),
      .previous_control_receive_stamp_ns =
          baseline.execution_input->previousControlReceiveStampNs(),
  };
  mismatched_input_capture.state.vx += 1.0F;
  const auto mismatched_input =
      VersionedExecutionInput3D::capture(mismatched_input_capture);
  ASSERT_NE(mismatched_input, nullptr);
  EXPECT_FALSE(
      certifyFiniteExecution3D(*active, route,
                               make_certification(*baseline.horizon, mismatched_input,
                                                  baseline.latest_lidar_evidence))
          .has_value());

  EXPECT_FALSE(
      certifyFiniteExecution3D(
          *active, route,
          make_certification(*baseline.horizon, baseline.execution_input, nullptr))
          .has_value());

  const mppi::State& middle =
      baseline.horizon->states[baseline.horizon->states.size() / 2U];
  const auto blocking_lidar =
      VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
          .producer_instance_id = 77U,
          .sequence = 102U,
          .pose_generation = 55U,
          .acquisition_stamp_ns = 985'000'000LL,
          .receive_stamp_ns = 995'000'000LL,
          .source_beam_count = 1U,
          .hit_points_map_m = {Point3{middle.x, middle.y, middle.z}},
      });
  ASSERT_NE(blocking_lidar, nullptr);
  EXPECT_FALSE(certifyFiniteExecution3D(*active, route,
                                        make_certification(*baseline.horizon,
                                                           baseline.execution_input,
                                                           blocking_lidar))
                   .has_value());

  const auto stale_lidar =
      VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
          .producer_instance_id = 77U,
          .sequence = 103U,
          .pose_generation = 55U,
          .acquisition_stamp_ns = 800'000'000LL,
          .receive_stamp_ns = 810'000'000LL,
          .source_beam_count = 1U,
          .hit_points_map_m = {},
      });
  ASSERT_NE(stale_lidar, nullptr);
  EXPECT_FALSE(
      certifyFiniteExecution3D(
          *active, route,
          make_certification(*baseline.horizon, baseline.execution_input, stale_lidar))
          .has_value());
}

TEST(ExecutionRouteSnapshot3DTest, ValidationProofBindsExactWorldAndLidarOwnerContent) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_NE(active->route->observed_raw_world, nullptr);

  const DirectTrackingOwnerIdentity3D identity{
      .mission_epoch = fixture.objective.mission_epoch,
      .assignment_generation = fixture.objective.assignment_generation,
      .target_detection_id = 41U,
      .target_track_id = 42U,
      .objective_sample_sequence = fixture.objective.sample_sequence,
      .line_of_sight_generation = 7U,
  };
  const FiniteExecutionCertification3D finite =
      SnapshotFixture3D::finiteCertificationForRoute(
          *active->route, FiniteExecutionKind3D::kNominal, 101U, 101U);
  ASSERT_NE(finite.execution_input, nullptr);
  ASSERT_NE(finite.latest_lidar_evidence, nullptr);
  const auto certify_with =
      [&](std::shared_ptr<const VersionedObservedRawWorld3D> world,
          std::shared_ptr<const VersionedLatestLidarEvidence3D> lidar) {
        return certifyDirectTrackingExecution3D(
            *active, DirectTrackingExecutionCertification3D{
                         .identity = identity,
                         .trajectory_revision = finite.trajectory_revision,
                         .target = fixture.objective.goal,
                         .horizon = finite.horizon,
                         .observed_raw_world = std::move(world),
                         .static_world = nullptr,
                         .validation_policy = active->route->validation_policy,
                         .execution_input = finite.execution_input,
                         .latest_lidar_evidence = std::move(lidar),
                         .valid_from_ns = finite.valid_from_ns,
                         .kind = FiniteExecutionKind3D::kNominal,
                     });
      };

  const std::optional<DirectTrackingFiniteExecution3D> baseline =
      certify_with(active->route->observed_raw_world, finite.latest_lidar_evidence);
  ASSERT_TRUE(baseline.has_value());

  ObservedOccupancyGrid3D changed_occupancy =
      active->route->observed_raw_world->occupancy();
  ASSERT_TRUE(
      changed_occupancy.setState(GridIndex3D{0, 0, 0}, ObservedVoxelState::kOccupied));
  const std::shared_ptr<const VersionedObservedRawWorld3D> changed_world =
      VersionedObservedRawWorld3D::capture(active->route->observed_raw_world->version(),
                                           changed_occupancy, std::nullopt,
                                           std::nullopt);
  ASSERT_NE(changed_world, nullptr);
  ASSERT_NE(changed_world->contentFingerprint(),
            active->route->observed_raw_world->contentFingerprint());
  const std::optional<DirectTrackingFiniteExecution3D> changed_world_proof =
      certify_with(changed_world, finite.latest_lidar_evidence);
  ASSERT_TRUE(changed_world_proof.has_value());
  EXPECT_NE(changed_world_proof->validation_proof.validation_contract_fingerprint,
            baseline->validation_proof.validation_contract_fingerprint);

  const VersionedLatestLidarEvidence3D& baseline_lidar = *finite.latest_lidar_evidence;
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> changed_lidar =
      VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
          .producer_instance_id = baseline_lidar.producerInstanceId(),
          .sequence = baseline_lidar.sequence(),
          .pose_generation = baseline_lidar.poseGeneration(),
          .acquisition_stamp_ns = baseline_lidar.acquisitionStampNs(),
          .receive_stamp_ns = baseline_lidar.receiveStampNs(),
          .source_beam_count = baseline_lidar.sourceBeamCount(),
          .invalid_beam_count = baseline_lidar.invalidBeamCount(),
          .hit_points_map_m = {Point3{-4.0, -4.0, 1.0}},
      });
  ASSERT_NE(changed_lidar, nullptr);
  ASSERT_NE(changed_lidar->contentFingerprint(), baseline_lidar.contentFingerprint());
  const std::optional<DirectTrackingFiniteExecution3D> changed_lidar_proof =
      certify_with(active->route->observed_raw_world, changed_lidar);
  ASSERT_TRUE(changed_lidar_proof.has_value());
  EXPECT_NE(changed_lidar_proof->validation_proof.validation_contract_fingerprint,
            baseline->validation_proof.validation_contract_fingerprint);
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteCertificationRejectsMissingOrMismatchedWorldOwners) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_NE(active->route->observed_raw_world, nullptr);

  const auto certify_against = [&](const CertifiedRouteSuffix3D& route) {
    return certifyFiniteExecution3D(
        *active, route,
        SnapshotFixture3D::finiteCertificationForRoute(
            *active->route, FiniteExecutionKind3D::kNominal, 101U));
  };
  CertifiedRouteSuffix3D missing_owner = *active->route;
  missing_owner.observed_raw_world.reset();
  EXPECT_FALSE(certify_against(missing_owner).has_value());

  ObservedOccupancyGrid3D changed_occupancy =
      active->route->observed_raw_world->occupancy();
  ASSERT_TRUE(
      changed_occupancy.setState(GridIndex3D{0, 0, 0}, ObservedVoxelState::kOccupied));
  const std::shared_ptr<const VersionedObservedRawWorld3D> changed_world =
      VersionedObservedRawWorld3D::capture(active->route->observed_raw_world->version(),
                                           changed_occupancy, std::nullopt,
                                           std::nullopt);
  ASSERT_NE(changed_world, nullptr);
  ASSERT_NE(changed_world->contentFingerprint(),
            active->route->observed_raw_world->contentFingerprint());

  CertifiedRouteSuffix3D mismatched_owner = *active->route;
  mismatched_owner.observed_raw_world = changed_world;
  EXPECT_FALSE(certify_against(mismatched_owner).has_value());

  FiniteExecutionState3D tampered_execution =
      SnapshotFixture3D::finiteExecution(*active);
  tampered_execution.observed_raw_world = changed_world;
  EXPECT_FALSE(tampered_execution.validFor(&*active->route));
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteCertificationAcceptsInputAgeLimitsInclusivelyAndRejectsStaleEvidence) {
  SnapshotFixture3D fixture;
  fixture.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      fixture.validation_policy->flightEnvelope(),
      fixture.validation_policy->dynamics(),
      fixture.validation_policy->altitudeEnvelope(),
      fixture.validation_policy->sweptFootprint(), 100.0, 20.0, 10.0);
  ASSERT_NE(fixture.validation_policy, nullptr);
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());

  const auto certify_with_receive_stamps = [&](const std::int64_t pose_receive_stamp_ns,
                                               const std::int64_t
                                                   control_receive_stamp_ns) {
    FiniteExecutionCertification3D certification =
        SnapshotFixture3D::finiteCertificationForRoute(
            *active->route, FiniteExecutionKind3D::kNominal, 101U);
    const std::shared_ptr<const VersionedExecutionInput3D>& source =
        certification.execution_input;
    if (source == nullptr) {
      return std::optional<FiniteExecutionState3D>{};
    }
    certification.execution_input =
        VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
            .capture_sequence = source->captureSequence(),
            .pose_revision = source->poseRevision(),
            .pose_source_timestamp_us = source->poseSourceTimestampUs(),
            .pose_receive_stamp_ns = pose_receive_stamp_ns,
            .effective_stamp_ns = source->effectiveStampNs(),
            .state = source->state(),
            .full_state_authoritative = source->fullStateAuthoritative(),
            .state_provenance = source->stateProvenance(),
            .previous_control = source->previousControl(),
            .previous_control_source = source->previousControlSource(),
            .previous_control_source_producer_instance_id =
                source->previousControlSourceProducerInstanceId(),
            .previous_control_source_sequence = source->previousControlSourceSequence(),
            .previous_control_source_stamp_ns = source->previousControlSourceStampNs(),
            .previous_control_receive_stamp_ns = control_receive_stamp_ns,
        });
    if (certification.execution_input == nullptr) {
      return std::optional<FiniteExecutionState3D>{};
    }
    return certifyFiniteExecution3D(*active, *active->route, std::move(certification));
  };

  constexpr std::int64_t kValidFromNs{1'000'000'000LL};
  const std::optional<FiniteExecutionState3D> at_limits = certify_with_receive_stamps(
      kValidFromNs - 20'000'000LL, kValidFromNs - 10'000'000LL);
  ASSERT_TRUE(at_limits.has_value());
  EXPECT_TRUE(at_limits->validFor(&*active->route));
  EXPECT_FALSE(certify_with_receive_stamps(kValidFromNs - 20'000'001LL,
                                           kValidFromNs - 10'000'000LL)
                   .has_value());
  EXPECT_FALSE(certify_with_receive_stamps(kValidFromNs - 20'000'000LL,
                                           kValidFromNs - 10'000'001LL)
                   .has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     StaticFiniteCertificationUsesExactLidarWithoutASupportExemption) {
  SnapshotFixture3D fixture;
  const OccupancyGrid3D clear_static{fixture.raw_occupancy.bounds(),
                                     fixture.validated_world.esdf_fingerprint};
  const std::optional<CertifiedRouteSuffix3D> suffix =
      certifyExecutionRoute3D(staticActivation(fixture, clear_static));
  ASSERT_TRUE(suffix.has_value());
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_TRUE(initial);
  const FiniteExecutionState3D baseline = SnapshotFixture3D::finiteExecutionForRoute(
      *initial, *suffix, FiniteExecutionKind3D::kNominal, true, 101U);
  ASSERT_NE(baseline.horizon, nullptr);
  ASSERT_NE(baseline.execution_input, nullptr);
  ASSERT_NE(baseline.latest_lidar_evidence, nullptr);

  EXPECT_TRUE(certifyFiniteExecution3D(
                  *initial, *suffix,
                  FiniteExecutionCertification3D{
                      .trajectory_revision = 102U,
                      .horizon = *baseline.horizon,
                      .execution_input = baseline.execution_input,
                      .latest_lidar_evidence = baseline.latest_lidar_evidence,
                      .valid_from_ns = baseline.valid_from_ns,
                      .kind = FiniteExecutionKind3D::kNominal,
                  })
                  .has_value());

  const auto blocking_lidar =
      VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
          .producer_instance_id = 77U,
          .sequence = 101U,
          .pose_generation = baseline.execution_input->poseRevision(),
          .acquisition_stamp_ns = 985'000'000LL,
          .receive_stamp_ns = 995'000'000LL,
          .source_beam_count = 1U,
          .hit_points_map_m = {Point3{4.0, 0.0, 5.0}},
      });
  ASSERT_NE(blocking_lidar, nullptr);
  EXPECT_FALSE(certifyFiniteExecution3D(*initial, *suffix,
                                        FiniteExecutionCertification3D{
                                            .trajectory_revision = 103U,
                                            .horizon = *baseline.horizon,
                                            .execution_input = baseline.execution_input,
                                            .latest_lidar_evidence = blocking_lidar,
                                            .valid_from_ns = baseline.valid_from_ns,
                                            .kind = FiniteExecutionKind3D::kNominal,
                                        })
                   .has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteCertificationRejectsAPathThatCrossesTheEndpointThenReturnsToRest) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  const FiniteExecutionState3D baseline = SnapshotFixture3D::finiteExecution(*active);
  ASSERT_NE(baseline.execution_input, nullptr);
  ASSERT_NE(baseline.latest_lidar_evidence, nullptr);

  mppi::FiniteHorizon crossing;
  crossing.controls.reserve(121U);
  for (std::size_t index = 0U; index < 50U; ++index) {
    crossing.controls.push_back(mppi::Control{.ax = 0.344F});
  }
  for (std::size_t index = 0U; index < 50U; ++index) {
    crossing.controls.push_back(mppi::Control{.ax = -0.344F});
  }
  for (std::size_t index = 0U; index < 10U; ++index) {
    crossing.controls.push_back(mppi::Control{.ax = -0.4F});
  }
  for (std::size_t index = 0U; index < 10U; ++index) {
    crossing.controls.push_back(mppi::Control{.ax = 0.4F});
  }
  crossing.controls.push_back(mppi::Control{});
  crossing.nominal_prefix_control_count = crossing.controls.size();
  crossing.states.reserve(crossing.controls.size() + 1U);
  crossing.states.push_back(baseline.execution_input->state());
  for (const mppi::Control& control : crossing.controls) {
    crossing.states.push_back(mppi::integrateReference(
        crossing.states.back(), control, active->route->validation_policy->dynamics()));
  }
  ASSERT_TRUE(mppi::finiteHorizonHasTerminalRestState(crossing));
  EXPECT_GT(std::ranges::max(crossing.states, {}, &mppi::State::x).x, 10.5F);
  EXPECT_LT(crossing.states.back().x, 10.5F);

  EXPECT_FALSE(certifyFiniteExecution3D(
                   *active, *active->route,
                   FiniteExecutionCertification3D{
                       .trajectory_revision = 102U,
                       .horizon = std::move(crossing),
                       .execution_input = baseline.execution_input,
                       .latest_lidar_evidence = baseline.latest_lidar_evidence,
                       .valid_from_ns = baseline.valid_from_ns,
                       .kind = FiniteExecutionKind3D::kNominal,
                   })
                   .has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteCertificationDoesNotAccumulateSubToleranceStationRegression) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  constexpr double kBeginStationM{2.0};
  FiniteExecutionCertification3D certification =
      SnapshotFixture3D::finiteCertificationForRoute(
          *suffix, FiniteExecutionKind3D::kNominal, 102U, 55U, 0U, kBeginStationM);
  ASSERT_NE(certification.execution_input, nullptr);
  ASSERT_EQ(certification.horizon.controls.size(), 101U);

  constexpr std::size_t kAccelerationControlCount{50U};
  constexpr float kBackwardAccelerationMps2{2.0e-6F};
  const mppi::DynamicsConfig& dynamics = suffix->validation_policy->dynamics();
  const float drag = std::max(0.0F, 1.0F - dynamics.linear_drag_1ps * dynamics.dt_s);
  const float recovery_acceleration_mps2 =
      kBackwardAccelerationMps2 *
      std::pow(drag, static_cast<float>(kAccelerationControlCount));
  for (std::size_t index = 0U; index < kAccelerationControlCount; ++index) {
    certification.horizon.controls[index].ax = -kBackwardAccelerationMps2;
  }
  for (std::size_t index = kAccelerationControlCount;
       index < 2U * kAccelerationControlCount; ++index) {
    certification.horizon.controls[index].ax = recovery_acceleration_mps2;
  }
  certification.horizon.controls.back() = {};
  certification.horizon.states.front() = certification.execution_input->state();
  for (std::size_t index = 0U; index < certification.horizon.controls.size(); ++index) {
    certification.horizon.states[index + 1U] =
        mppi::integrateReference(certification.horizon.states[index],
                                 certification.horizon.controls[index], dynamics);
  }
  ASSERT_TRUE(mppi::finiteHorizonHasTerminalRestState(certification.horizon));
  ASSERT_LT(certification.horizon.states.back().x,
            certification.horizon.states.front().x - 1.0e-6F);
  for (std::size_t index = 1U; index < certification.horizon.states.size(); ++index) {
    EXPECT_GE(certification.horizon.states[index].x + 1.0e-6F,
              certification.horizon.states[index - 1U].x);
  }

  const FiniteExecutionCertificationResult3D result =
      certifyFiniteExecution3DDetailed(*initial, *suffix, std::move(certification));

  ASSERT_TRUE(result.certified())
      << "status=" << finiteExecutionCertificationStatus3DName(result.status)
      << " route_adherence_status="
      << finiteExecutionRouteAdherenceStatus3DName(result.route_adherence_status);
  ASSERT_TRUE(result.execution.has_value());
  EXPECT_DOUBLE_EQ(result.execution->begin_route_station_m, kBeginStationM);
  EXPECT_DOUBLE_EQ(result.execution->stop_boundary.station_m, kBeginStationM);
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteCertificationKeepsTerminalRestInsideTheCertifiedRouteCorridor) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  FiniteExecutionCertification3D certification =
      SnapshotFixture3D::finiteCertificationForRoute(
          *suffix, FiniteExecutionKind3D::kNominal, 102U);
  ASSERT_NE(certification.execution_input, nullptr);
  ASSERT_GE(certification.horizon.controls.size(), 101U);

  constexpr std::size_t kAccelerationControlCount{50U};
  constexpr float kTerminalLateralOffsetM{0.4F};
  const float lateral_acceleration_mps2 = kTerminalLateralOffsetM / 25.0F;
  for (std::size_t index = 0U; index < kAccelerationControlCount; ++index) {
    certification.horizon.controls[index].ay = lateral_acceleration_mps2;
  }
  for (std::size_t index = kAccelerationControlCount;
       index < 2U * kAccelerationControlCount; ++index) {
    certification.horizon.controls[index].ay = -lateral_acceleration_mps2;
  }
  certification.horizon.states.front() = certification.execution_input->state();
  for (std::size_t index = 0U; index < certification.horizon.controls.size(); ++index) {
    certification.horizon.states[index + 1U] = mppi::integrateReference(
        certification.horizon.states[index], certification.horizon.controls[index],
        suffix->validation_policy->dynamics());
  }
  ASSERT_TRUE(mppi::finiteHorizonHasTerminalRestState(certification.horizon));
  EXPECT_GT(certification.horizon.states.back().y, 0.25F);
  EXPECT_LT(certification.horizon.states.back().y, 2.0F);

  const FiniteExecutionCertificationResult3D result =
      certifyFiniteExecution3DDetailed(*initial, *suffix, std::move(certification));

  ASSERT_TRUE(result.certified())
      << "status=" << finiteExecutionCertificationStatus3DName(result.status)
      << " route_adherence_status="
      << finiteExecutionRouteAdherenceStatus3DName(result.route_adherence_status)
      << " route_adherence_failure_distance_m="
      << result.route_adherence_failure_distance_m;
  ASSERT_TRUE(result.execution.has_value());
  EXPECT_DOUBLE_EQ(result.execution->stop_boundary.position_tolerance_m, 0.25);
  EXPECT_GT(result.execution->stop_boundary.position.y, 0.25);
  EXPECT_LT(result.execution->stop_boundary.position.y, 2.0);
  const ExecutionRouteTransitionResult3D activation =
      activateCertifiedRoute3D(*initial, initial->version, *suffix, *result.execution);
  ASSERT_TRUE(activation.applied())
      << "transition_status="
      << executionRouteTransitionStatus3DName(activation.status);
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteCertificationDoesNotExtendAConstrainedSpanPastItsBoundary) {
  SnapshotFixture3D fixture;
  constexpr double kForwardTangentComponent{0.01};
  fixture.route.at(1).tangent =
      Vec3{kForwardTangentComponent,
           std::sqrt(1.0 - kForwardTangentComponent * kForwardTangentComponent), 0.0};
  PassageVolumeConfig passage_config = testPassageVolumeConfig();
  passage_config.lateral_probe_step_m = 0.0005;
  passage_config.secondary_probe_step_m = 0.0005;
  passage_config.maximum_cross_section_probe_m = 0.0005;
  passage_config.minimum_wall_clearance_m = 0.0;
  fixture.geometry = makeConstrainedGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      SnapshotFixture3D::kRouteGeneration, fixture.raw_occupancy.occupiedSnapshot(),
      passage_config, 0.0, 4.0);
  fixture.geometry_revision = fixture.geometry->executable_geometry_revision;

  ExecutionRouteActivation3D activation = fixture.activation();
  activation.geometry = fixture.geometry;
  activation.observation.position = {4.0, 0.0, 5.0};
  activation.passage_volume_config = passage_config;
  const std::optional<CertifiedRouteSuffix3D> suffix =
      certifyExecutionRoute3D(activation);
  ASSERT_TRUE(suffix.has_value());
  EXPECT_DOUBLE_EQ(suffix->progress.station_m, 4.0);

  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  FiniteExecutionCertification3D certification =
      SnapshotFixture3D::finiteCertificationForRoute(
          *suffix, FiniteExecutionKind3D::kNominal, 100U);
  ASSERT_GE(certification.horizon.states.size(), 2U);
  EXPECT_GT(static_cast<double>(certification.horizon.states.at(1).x) - 4.0,
            passage_config.maximum_cross_section_probe_m);
  const std::optional<FiniteExecutionState3D> finite_execution =
      certifyFiniteExecution3D(*initial, *suffix, std::move(certification));
  ASSERT_TRUE(finite_execution.has_value());

  const ExecutionRouteTransitionResult3D activated =
      activateCertifiedRoute3D(*initial, initial->version, *suffix, *finite_execution);
  EXPECT_TRUE(activated.applied());
}

TEST(ExecutionRouteSnapshot3DTest,
     ProgressAndRawCertificateAdvanceFromOneAcceptedAssessment) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_TRUE(active);
  constexpr std::uint64_t kRenewedRawRevision{SnapshotFixture3D::kLatestRawRevision +
                                              1U};

  const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active),
      fixture.executionObservation({4.0, 0.0, 5.0}, kRenewedRawRevision,
                                   &fixture.raw_occupancy),
      SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0}),
      fixture.rawWorld(kRenewedRawRevision));

  ASSERT_TRUE(advanced.applied());
  ASSERT_TRUE(advanced.next);
  ASSERT_TRUE(advanced.next->route.has_value());
  EXPECT_EQ(advanced.next->version, active->version + 1U);
  EXPECT_DOUBLE_EQ(advanced.next->route->progress.station_m, 4.0);
  const ObservedRawRouteCertificate3D* const renewed =
      std::get_if<ObservedRawRouteCertificate3D>(&advanced.next->route->certificate);
  ASSERT_NE(renewed, nullptr);
  EXPECT_EQ(renewed->validated_through_revision, kRenewedRawRevision);
  EXPECT_DOUBLE_EQ(renewed->suffix_start_station_m, 4.0);
  EXPECT_DOUBLE_EQ(active->route->progress.station_m, 2.0);

  const ExecutionRouteTransitionResult3D missing_raw = advanceCertifiedRoute3D(
      *advanced.next, SnapshotFixture3D::guard(*advanced.next),
      fixture.executionObservation({5.0, 0.0, 5.0}, kRenewedRawRevision, nullptr),
      SnapshotFixture3D::progressInput(*advanced.next, {5.0, 0.0, 5.0}), nullptr);
  EXPECT_EQ(missing_raw.status, ExecutionRouteTransitionStatus3D::kInvalidCandidate);

  RouteExecutionObservation3D changed_policy = fixture.executionObservation(
      {5.0, 0.0, 5.0}, kRenewedRawRevision, &fixture.raw_occupancy);
  changed_policy.footprint.radius_m = 0.1;
  const ExecutionRouteTransitionResult3D rejected_policy = advanceCertifiedRoute3D(
      *advanced.next, SnapshotFixture3D::guard(*advanced.next), changed_policy,
      SnapshotFixture3D::progressInput(*advanced.next, {5.0, 0.0, 5.0}),
      fixture.rawWorld(kRenewedRawRevision));
  EXPECT_EQ(rejected_policy.status,
            ExecutionRouteTransitionStatus3D::kInvalidCandidate);
}

TEST(ExecutionRouteSnapshot3DTest,
     ProgressRejectsReplayConflictingIdentityAndDelayedEvidence) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_NE(active->route->progress.execution_input, nullptr);
  const VersionedExecutionInput3D& resident = *active->route->progress.execution_input;
  const auto recapture =
      [&](const std::uint64_t capture_sequence, const std::uint64_t pose_revision,
          const std::uint64_t pose_source_timestamp_us,
          const std::int64_t pose_receive_stamp_ns, const mppi::State& state,
          const std::uint64_t control_source_sequence,
          const std::int64_t control_source_stamp_ns,
          const std::int64_t control_receive_stamp_ns) {
        return VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
            .capture_sequence = capture_sequence,
            .pose_revision = pose_revision,
            .pose_source_timestamp_us = pose_source_timestamp_us,
            .pose_receive_stamp_ns = pose_receive_stamp_ns,
            .effective_stamp_ns = resident.effectiveStampNs(),
            .state = state,
            .full_state_authoritative = resident.fullStateAuthoritative(),
            .state_provenance = resident.stateProvenance(),
            .previous_control = resident.previousControl(),
            .previous_control_source = resident.previousControlSource(),
            .previous_control_source_producer_instance_id =
                resident.previousControlSourceProducerInstanceId(),
            .previous_control_source_sequence = control_source_sequence,
            .previous_control_source_stamp_ns = control_source_stamp_ns,
            .previous_control_receive_stamp_ns = control_receive_stamp_ns,
        });
      };
  const RouteExecutionObservation3D observation = fixture.executionObservation(
      {4.0, 0.0, 5.0}, SnapshotFixture3D::kLatestRawRevision + 1U,
      &fixture.raw_occupancy);

  EXPECT_EQ(advanceCertifiedRoute3D(
                *active, SnapshotFixture3D::guard(*active), observation,
                active->route->progress.execution_input,
                fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U))
                .status,
            ExecutionRouteTransitionStatus3D::kNoChange);

  mppi::State conflicting_state = resident.state();
  conflicting_state.x = 4.0F;
  const std::shared_ptr<const VersionedExecutionInput3D> conflicting =
      recapture(resident.captureSequence(), resident.poseRevision(),
                resident.poseSourceTimestampUs(), resident.poseReceiveStampNs(),
                conflicting_state, resident.previousControlSourceSequence(),
                resident.previousControlSourceStampNs(),
                resident.previousControlReceiveStampNs());
  ASSERT_NE(conflicting, nullptr);
  EXPECT_EQ(advanceCertifiedRoute3D(
                *active, SnapshotFixture3D::guard(*active), observation, conflicting,
                fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U))
                .status,
            ExecutionRouteTransitionStatus3D::kNonMonotonicProgress);

  const std::shared_ptr<const VersionedExecutionInput3D> mixed_pose_identity =
      recapture(resident.captureSequence() + 1U, resident.poseRevision(),
                resident.poseSourceTimestampUs() + 1U, resident.poseReceiveStampNs(),
                resident.state(), resident.previousControlSourceSequence() + 1U,
                resident.previousControlSourceStampNs() + 1LL,
                resident.previousControlReceiveStampNs() + 1LL);
  ASSERT_NE(mixed_pose_identity, nullptr);
  EXPECT_EQ(advanceCertifiedRoute3D(
                *active, SnapshotFixture3D::guard(*active), observation,
                mixed_pose_identity,
                fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U))
                .status,
            ExecutionRouteTransitionStatus3D::kNonMonotonicProgress);

  const std::shared_ptr<const VersionedExecutionInput3D> reused_pose_timestamps =
      recapture(resident.captureSequence() + 1U, resident.poseRevision() + 1U,
                resident.poseSourceTimestampUs(), resident.poseReceiveStampNs(),
                resident.state(), resident.previousControlSourceSequence() + 1U,
                resident.previousControlSourceStampNs() + 1LL,
                resident.previousControlReceiveStampNs() + 1LL);
  ASSERT_NE(reused_pose_timestamps, nullptr);
  EXPECT_EQ(advanceCertifiedRoute3D(
                *active, SnapshotFixture3D::guard(*active), observation,
                reused_pose_timestamps,
                fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U))
                .status,
            ExecutionRouteTransitionStatus3D::kNonMonotonicProgress);

  mppi::State forged_source_sample_state = resident.state();
  forged_source_sample_state.x += 0.25F;
  const std::shared_ptr<const VersionedExecutionInput3D> forged_source_sample =
      VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
          .capture_sequence = resident.captureSequence() + 1U,
          .pose_revision = resident.poseRevision(),
          .pose_source_timestamp_us = resident.poseSourceTimestampUs(),
          .pose_receive_stamp_ns = resident.poseReceiveStampNs(),
          .effective_stamp_ns = resident.effectiveStampNs() + 1LL,
          .state = forged_source_sample_state,
          .full_state_authoritative = resident.fullStateAuthoritative(),
          .state_provenance = resident.stateProvenance(),
          .previous_control = resident.previousControl(),
          .previous_control_source = resident.previousControlSource(),
          .previous_control_source_producer_instance_id =
              resident.previousControlSourceProducerInstanceId(),
          .previous_control_source_sequence =
              resident.previousControlSourceSequence() + 1U,
          .previous_control_source_stamp_ns =
              resident.previousControlSourceStampNs() + 1LL,
          .previous_control_receive_stamp_ns =
              resident.previousControlReceiveStampNs() + 1LL,
      });
  ASSERT_NE(forged_source_sample, nullptr);
  EXPECT_EQ(advanceCertifiedRoute3D(
                *active, SnapshotFixture3D::guard(*active), observation,
                forged_source_sample,
                fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U))
                .status,
            ExecutionRouteTransitionStatus3D::kNonMonotonicProgress);

  ASSERT_GT(resident.captureSequence(), 1U);
  ASSERT_GT(resident.poseRevision(), 1U);
  ASSERT_GT(resident.poseSourceTimestampUs(), 1U);
  ASSERT_GT(resident.previousControlSourceSequence(), 1U);
  const std::shared_ptr<const VersionedExecutionInput3D> delayed = recapture(
      resident.captureSequence() - 1U, resident.poseRevision() - 1U,
      resident.poseSourceTimestampUs() - 1U, resident.poseReceiveStampNs() - 1LL,
      resident.state(), resident.previousControlSourceSequence() - 1U,
      resident.previousControlSourceStampNs() - 1LL,
      resident.previousControlReceiveStampNs() - 1LL);
  ASSERT_NE(delayed, nullptr);
  EXPECT_EQ(advanceCertifiedRoute3D(
                *active, SnapshotFixture3D::guard(*active), observation, delayed,
                fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U))
                .status,
            ExecutionRouteTransitionStatus3D::kNonMonotonicProgress);
}

} // namespace
} // namespace drone_city_nav
