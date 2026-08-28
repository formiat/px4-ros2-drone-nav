#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

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
