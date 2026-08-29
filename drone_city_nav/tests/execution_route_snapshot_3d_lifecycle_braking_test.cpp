#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::shared_ptr<const VersionedLatestLidarEvidence3D>
lidarEvidenceWithPoints(const VersionedLatestLidarEvidence3D& identity_source,
                        std::vector<Point3> hit_points_map_m) {
  return VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
      .producer_instance_id = identity_source.producerInstanceId(),
      .sequence = identity_source.sequence(),
      .pose_generation = identity_source.poseGeneration(),
      .acquisition_stamp_ns = identity_source.acquisitionStampNs(),
      .receive_stamp_ns = identity_source.receiveStampNs(),
      .source_beam_count = std::max<std::size_t>(1U, hit_points_map_m.size()),
      .invalid_beam_count = identity_source.invalidBeamCount(),
      .hit_points_map_m = std::move(hit_points_map_m),
  });
}

TEST(ExecutionRouteSnapshot3DTest,
     FreshLidarInvalidatesOnlyAnIntersectedPublishedFiniteTrajectory) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_TRUE(active->finite_execution.has_value());
  const FiniteExecutionState3D& resident = *active->finite_execution;
  ASSERT_NE(resident.horizon, nullptr);
  ASSERT_FALSE(resident.horizon->states.empty());

  FiniteExecutionCertification3D current =
      SnapshotFixture3D::finiteCertificationForRoute(
          *active->route, FiniteExecutionKind3D::kEmergencyBrakeTail,
          resident.trajectory_revision + 2U);
  ASSERT_NE(current.execution_input, nullptr);
  ASSERT_NE(current.latest_lidar_evidence, nullptr);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> clear_lidar =
      lidarEvidenceWithPoints(*current.latest_lidar_evidence,
                              {Point3{-4.0, -4.0, 1.0}});
  ASSERT_NE(clear_lidar, nullptr);

  const mppi::FiniteExecutionPathValidation clear =
      validateRemainingFiniteExecutionAgainstLatestLidar3D(
          resident, *current.execution_input, *clear_lidar, current.valid_from_ns);
  EXPECT_EQ(clear.status, mppi::FiniteExecutionPathStatus::kValid);

  const mppi::State& intersected_state =
      resident.horizon->states[resident.horizon->states.size() / 2U];
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> blocking_lidar =
      SnapshotFixture3D::newerLidarEvidence(
          *clear_lidar,
          {Point3{intersected_state.x, intersected_state.y, intersected_state.z}});
  ASSERT_NE(blocking_lidar, nullptr);
  const mppi::FiniteExecutionPathValidation blocked =
      validateRemainingFiniteExecutionAgainstLatestLidar3D(
          resident, *current.execution_input, *blocking_lidar, current.valid_from_ns);
  EXPECT_EQ(blocked.status, mppi::FiniteExecutionPathStatus::kLatestLidarRawCollision);
}

TEST(ExecutionRouteSnapshot3DTest,
     LatestLidarInvalidationBindsEmergencyBrakingToExactPhysicalEvidence) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_TRUE(active->finite_execution.has_value());
  ASSERT_NE(active->finite_execution->horizon, nullptr);

  FiniteExecutionCertification3D braking =
      SnapshotFixture3D::finiteCertificationForRoute(
          *active->route, FiniteExecutionKind3D::kEmergencyBrakeTail,
          active->finite_execution->trajectory_revision + 2U);
  ASSERT_NE(braking.latest_lidar_evidence, nullptr);
  const mppi::State& intersected_state =
      active->finite_execution->horizon
          ->states[active->finite_execution->horizon->states.size() / 2U];
  braking.latest_lidar_evidence = lidarEvidenceWithPoints(
      *braking.latest_lidar_evidence,
      {Point3{intersected_state.x, intersected_state.y, intersected_state.z}});
  ASSERT_NE(braking.latest_lidar_evidence, nullptr);
  const RouteLifecycleEvent3D invalidation{
      .kind = RouteLifecycleEventKind3D::kLatestLidarInvalidated,
      .generation = active->route->identity.generation,
      .latest_lidar_evidence = braking.latest_lidar_evidence->evidenceId(),
  };

  RouteLifecycleEvent3D wrong_evidence = invalidation;
  ++wrong_evidence.latest_lidar_evidence.sequence;
  EXPECT_FALSE(certifyLifecycleBrakingFiniteExecution3D(
                   *active,
                   LifecycleBrakingFiniteExecutionCertification3D{
                       .lifecycle_event = wrong_evidence,
                       .finite_execution = braking,
                   })
                   .has_value());

  FiniteExecutionCertification3D clear_braking = braking;
  clear_braking.latest_lidar_evidence = SnapshotFixture3D::newerLidarEvidence(
      *braking.latest_lidar_evidence, {Point3{-4.0, -4.0, 1.0}});
  ASSERT_NE(clear_braking.latest_lidar_evidence, nullptr);
  const RouteLifecycleEvent3D nonintersecting_evidence{
      .kind = RouteLifecycleEventKind3D::kLatestLidarInvalidated,
      .generation = active->route->identity.generation,
      .latest_lidar_evidence = clear_braking.latest_lidar_evidence->evidenceId(),
  };
  EXPECT_FALSE(certifyLifecycleBrakingFiniteExecution3D(
                   *active,
                   LifecycleBrakingFiniteExecutionCertification3D{
                       .lifecycle_event = nonintersecting_evidence,
                       .finite_execution = std::move(clear_braking),
                   })
                   .has_value());

  const std::optional<FiniteExecutionState3D> certified =
      certifyLifecycleBrakingFiniteExecution3D(
          *active, LifecycleBrakingFiniteExecutionCertification3D{
                       .lifecycle_event = invalidation,
                       .finite_execution = std::move(braking),
                   });
  ASSERT_TRUE(certified.has_value());
  ASSERT_NE(certified->latest_lidar_evidence, nullptr);
  EXPECT_EQ(certified->latest_lidar_evidence->evidenceId(),
            invalidation.latest_lidar_evidence);

  const ExecutionRouteTransitionResult3D retired = retireCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active), invalidation, certified);
  ASSERT_TRUE(retired.applied())
      << executionRouteTransitionStatus3DName(retired.status);
  ASSERT_NE(retired.next, nullptr);
  ASSERT_TRUE(retired.next->finite_execution.has_value());
  EXPECT_EQ(retired.next->phase, ExecutionRoutePhase3D::kBraking);
  EXPECT_EQ(retired.next->finite_execution->kind,
            FiniteExecutionKind3D::kEmergencyBrakeTail);
}

TEST(ExecutionRouteSnapshot3DTest,
     LifecycleBrakingRetainsPhysicalOwnershipOutsideTheOldRouteCorridor) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  if (active == nullptr || !active->route.has_value() ||
      !active->finite_execution.has_value()) {
    ADD_FAILURE() << "The fixture must activate a complete route owner";
    return;
  }
  const CertifiedRouteSuffix3D& route = active->route.value();
  FiniteExecutionCertification3D certification =
      SnapshotFixture3D::finiteCertificationForRoute(
          route, FiniteExecutionKind3D::kEmergencyBrakeTail, 101U, 56U);
  ASSERT_GT(certification.horizon.controls.size(), 1U);
  certification.horizon.controls.pop_back();
  certification.horizon.states.pop_back();
  --certification.horizon.arrival_control_count;
  constexpr float kDivergedY{3.0F};
  for (mppi::State& state : certification.horizon.states) {
    state.y = kDivergedY;
  }
  const std::shared_ptr<const VersionedExecutionInput3D> source_input =
      certification.execution_input;
  ASSERT_NE(source_input, nullptr);
  certification
      .execution_input = VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
      .capture_sequence = source_input->captureSequence(),
      .pose_revision = source_input->poseRevision(),
      .pose_source_timestamp_us = source_input->poseSourceTimestampUs(),
      .pose_receive_stamp_ns = source_input->poseReceiveStampNs(),
      .effective_stamp_ns = source_input->effectiveStampNs(),
      .state = certification.horizon.states.front(),
      .full_state_authoritative = source_input->fullStateAuthoritative(),
      .state_provenance = source_input->stateProvenance(),
      .previous_control = source_input->previousControl(),
      .previous_control_source = source_input->previousControlSource(),
      .previous_control_source_producer_instance_id =
          source_input->previousControlSourceProducerInstanceId(),
      .previous_control_source_sequence = source_input->previousControlSourceSequence(),
      .previous_control_source_stamp_ns = source_input->previousControlSourceStampNs(),
      .previous_control_receive_stamp_ns =
          source_input->previousControlReceiveStampNs(),
  });
  ASSERT_NE(certification.execution_input, nullptr);

  const FiniteExecutionCertificationResult3D route_following =
      certifyFiniteExecution3DDetailed(*active, route, certification);
  EXPECT_FALSE(route_following.certified());
  EXPECT_EQ(route_following.status,
            FiniteExecutionCertificationStatus3D::kExecutionBindingRejected);

  const RouteLifecycleEvent3D divergence{
      .kind = RouteLifecycleEventKind3D::kCrossTrackExceeded,
      .generation = route.identity.generation,
  };
  const FiniteExecutionCertificationResult3D braking =
      certifyLifecycleBrakingFiniteExecution3DDetailed(
          *active, LifecycleBrakingFiniteExecutionCertification3D{
                       .lifecycle_event = divergence,
                       .finite_execution = std::move(certification),
                   });
  ASSERT_TRUE(braking.certified())
      << finiteExecutionCertificationStatus3DName(braking.status) << ' '
      << finiteExecutionRouteAdherenceStatus3DName(braking.route_adherence_status);
  if (!braking.execution.has_value()) {
    ADD_FAILURE() << "Lifecycle braking certification must own an execution";
    return;
  }
  const FiniteExecutionState3D& braking_execution = braking.execution.value();
  EXPECT_DOUBLE_EQ(braking_execution.begin_route_station_m, route.progress.station_m);
  EXPECT_DOUBLE_EQ(braking_execution.stop_boundary.station_m, route.progress.station_m);
  EXPECT_DOUBLE_EQ(braking_execution.stop_boundary.position.y, kDivergedY);

  const ExecutionRouteTransitionResult3D retired = retireCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active), divergence, braking_execution);
  ASSERT_TRUE(retired.applied())
      << executionRouteTransitionStatus3DName(retired.status);
  if (retired.next == nullptr || !retired.next->route.has_value() ||
      !retired.next->finite_execution.has_value() ||
      !retired.next->braking_fallback.has_value()) {
    ADD_FAILURE() << "Retirement must preserve a complete braking owner";
    return;
  }
  const ExecutionRouteSnapshot3D& braking_snapshot = *retired.next;
  EXPECT_EQ(braking_snapshot.phase, ExecutionRoutePhase3D::kBraking);
  EXPECT_DOUBLE_EQ(braking_snapshot.route.value().progress.station_m,
                   route.progress.station_m);
  EXPECT_DOUBLE_EQ(braking_snapshot.route.value().progress.last_observed_position.y,
                   kDivergedY);
  EXPECT_EQ(
      braking_snapshot.finite_execution.value().validation_proof.artifact_fingerprint,
      braking_snapshot.braking_fallback.value().validation_proof.artifact_fingerprint);
}

TEST(ExecutionRouteSnapshot3DTest,
     TrackingTubeViolationAtomicallyActivatesTheResidentBrakingTail) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_TRUE(active->finite_execution.has_value());
  ASSERT_TRUE(active->braking_fallback.has_value());
  const RouteLifecycleEvent3D violation{
      .kind = RouteLifecycleEventKind3D::kTrackingTubeExceeded,
      .generation = active->route->identity.generation,
  };

  const ExecutionRouteTransitionResult3D retired = retireCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active), violation, std::nullopt);

  ASSERT_TRUE(retired.applied())
      << executionRouteTransitionStatus3DName(retired.status);
  ASSERT_NE(retired.next, nullptr);
  ASSERT_TRUE(retired.next->route.has_value());
  ASSERT_TRUE(retired.next->finite_execution.has_value());
  ASSERT_TRUE(retired.next->braking_fallback.has_value());
  EXPECT_EQ(retired.next->phase, ExecutionRoutePhase3D::kBraking);
  EXPECT_EQ(retired.next->route->owner.id, active->route->owner.id);
  EXPECT_EQ(retired.next->finite_execution->validation_proof.artifact_fingerprint,
            retired.next->braking_fallback->validation_proof.artifact_fingerprint);
}

} // namespace
} // namespace drone_city_nav
