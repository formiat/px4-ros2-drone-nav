#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(ExecutionRouteSnapshot3DTest, UnboundSuccessorBindsAtTheCurrentFinitePlanStart) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  if (!suffix.has_value() || initial == nullptr) {
    ADD_FAILURE() << "The fixture must provide an initial certified route";
    return;
  }
  const CertifiedRouteSuffix3D& successor = suffix.value();
  ASSERT_EQ(successor.progress.execution_input, nullptr);

  constexpr double kCurrentStationM{3.0};
  FiniteExecutionCertification3D command =
      SnapshotFixture3D::finiteCertificationForRoute(
          successor, FiniteExecutionKind3D::kNominal, 100U, 55U, 0U, kCurrentStationM);
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      command.execution_input;
  if (current_input == nullptr) {
    ADD_FAILURE() << "The command must own an execution input";
    return;
  }
  const std::optional<mppi::FiniteHorizon> braking_tail =
      mppi::buildFiniteBrakingHorizon(
          command.horizon.states.front(), command.horizon.controls.size(),
          successor.validation_policy->dynamics(), current_input->previousControl());
  if (!braking_tail.has_value()) {
    ADD_FAILURE() << "The command must produce a finite braking tail";
    return;
  }

  const FiniteExecutionPlanCertificationResult3D certification =
      certifyFiniteExecutionPlan3DDetailed(*initial, successor,
                                           FiniteExecutionPlanCertification3D{
                                               .command_horizon = std::move(command),
                                               .braking_tail = braking_tail.value(),
                                           });
  if (!certification.certified() || !certification.plan.has_value()) {
    ADD_FAILURE() << finiteExecutionCertificationStatus3DName(
        certification.command_horizon.status);
    return;
  }
  const FiniteExecutionPlan3D& plan = certification.plan.value();
  EXPECT_DOUBLE_EQ(plan.command_horizon.begin_route_station_m, kCurrentStationM);
  EXPECT_DOUBLE_EQ(plan.braking_tail.begin_route_station_m, kCurrentStationM);

  const ExecutionRouteTransitionResult3D activated =
      activateCertifiedRoute3D(*initial, initial->version, successor, plan);
  if (!activated.applied() || activated.next == nullptr) {
    ADD_FAILURE() << "The certified successor must activate atomically";
    return;
  }
  const ExecutionRouteSnapshot3D& active_snapshot = *activated.next;
  if (!active_snapshot.route.has_value()) {
    ADD_FAILURE() << "The activated snapshot must own the successor route";
    return;
  }
  const CertifiedRouteSuffix3D& active_route = active_snapshot.route.value();
  EXPECT_EQ(active_route.progress.execution_input, current_input);
  EXPECT_DOUBLE_EQ(active_route.progress.station_m, kCurrentStationM);
  EXPECT_TRUE(active_snapshot.publishable());
}

TEST(ExecutionRouteSnapshot3DTest,
     ProgressAndPermanentBrakingFallbackPublishAsOneAtomicPlan) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ExecutionRouteSnapshotStore3D store;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial = store.snapshot();
  if (!suffix.has_value() || initial == nullptr) {
    ADD_FAILURE() << "The fixture must provide an initial certified route";
    return;
  }
  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *initial, initial->version, suffix.value(),
      SnapshotFixture3D::finitePlanForRoute(*initial, suffix.value(),
                                            FiniteExecutionKind3D::kNominal, 100U));
  ASSERT_TRUE(activation.applied());
  ASSERT_EQ(store.publish(initial, activation),
            ExecutionRoutePublicationStatus3D::kPublished);
  const std::shared_ptr<const ExecutionRouteSnapshot3D> resident = store.snapshot();
  if (resident == nullptr || !resident->route.has_value()) {
    ADD_FAILURE() << "Activation must publish a route owner";
    return;
  }

  constexpr std::uint64_t kAdvancedRawRevision =
      SnapshotFixture3D::kLatestRawRevision + 1U;
  const std::shared_ptr<const VersionedExecutionInput3D> progress_input =
      SnapshotFixture3D::progressInput(*resident, {4.0, 0.0, 5.0});
  const ExecutionRouteTransitionResult3D progress = advanceCertifiedRoute3D(
      *resident, SnapshotFixture3D::guard(*resident),
      fixture.executionObservation({4.0, 0.0, 5.0}, kAdvancedRawRevision,
                                   &fixture.raw_occupancy),
      progress_input, fixture.rawWorld(kAdvancedRawRevision));
  ASSERT_TRUE(progress.applied());
  if (progress.next == nullptr) {
    ADD_FAILURE() << "Progress must produce a successor snapshot";
    return;
  }
  const ExecutionRouteSnapshot3D& progressed = *progress.next;
  if (!progressed.route.has_value() || !progressed.finite_execution.has_value() ||
      !progressed.braking_fallback.has_value()) {
    ADD_FAILURE() << "Progress must preserve the complete resident plan";
    return;
  }
  EXPECT_TRUE(progressed.valid());
  EXPECT_FALSE(progressed.publishable());
  EXPECT_TRUE(progressed.finite_execution.value().revalidation_required);
  EXPECT_TRUE(progressed.braking_fallback.value().revalidation_required);
  EXPECT_EQ(store.publish(resident, progress),
            ExecutionRoutePublicationStatus3D::kInvalidCandidate);
  EXPECT_EQ(store.snapshot(), resident);

  const FiniteExecutionPlan3D plan = SnapshotFixture3D::finitePlanForRoute(
      progressed, progressed.route.value(), FiniteExecutionKind3D::kNominal, 101U);
  const ExecutionRouteTransitionResult3D prepared = replaceFiniteExecutionPlan3D(
      progressed, SnapshotFixture3D::guard(progressed), plan);
  ASSERT_TRUE(prepared.applied());
  const ExecutionRouteTransitionResult3D composed =
      composeExecutionPlanTransition3D(*resident, progress, prepared);
  ASSERT_TRUE(composed.applied());
  if (composed.next == nullptr) {
    ADD_FAILURE() << "Composition must produce a successor snapshot";
    return;
  }
  const ExecutionRouteSnapshot3D& composed_snapshot = *composed.next;
  if (!composed_snapshot.route.has_value() ||
      !composed_snapshot.finite_execution.has_value() ||
      !composed_snapshot.braking_fallback.has_value()) {
    ADD_FAILURE() << "Composition must produce a complete publishable plan";
    return;
  }
  EXPECT_EQ(composed.predecessor, resident.get());
  EXPECT_EQ(composed_snapshot.version, resident->version + 2U);
  EXPECT_TRUE(composed_snapshot.publishable());

  const FiniteExecutionState3D& command = composed_snapshot.finite_execution.value();
  const FiniteExecutionState3D& braking = composed_snapshot.braking_fallback.value();
  ASSERT_NE(braking.horizon, nullptr);
  EXPECT_EQ(command.execution_input, braking.execution_input);
  EXPECT_EQ(command.latest_lidar_evidence, braking.latest_lidar_evidence);
  EXPECT_EQ(command.observed_raw_world, braking.observed_raw_world);
  EXPECT_EQ(command.source_snapshot_version, progressed.version);
  EXPECT_EQ(braking.source_snapshot_version, progressed.version);
  EXPECT_EQ(braking.kind, FiniteExecutionKind3D::kEmergencyBrakeTail);
  EXPECT_EQ(braking.horizon->nominal_prefix_control_count, 0U);
  EXPECT_EQ(braking.horizon->arrival_control_count, braking.horizon->controls.size());
  EXPECT_LE(braking.valid_until_ns, command.valid_until_ns);
  EXPECT_LE(braking.stop_boundary.station_m, command.stop_boundary.station_m);
  EXPECT_EQ(composed_snapshot.route.value().progress.execution_input,
            command.execution_input);

  ExecutionRouteSnapshot3D missing_fallback = composed_snapshot;
  missing_fallback.braking_fallback.reset();
  EXPECT_FALSE(missing_fallback.valid());
  EXPECT_FALSE(missing_fallback.publishable());

  ASSERT_EQ(store.publish(resident, composed),
            ExecutionRoutePublicationStatus3D::kPublished);
  EXPECT_EQ(store.snapshot(), composed.next);
}

TEST(ExecutionRouteSnapshot3DTest,
     SlightlyLaggingMeasuredPoseCannotRegressRetainedRouteProgress) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);

  constexpr double kRetainedStationM{4.0};
  constexpr std::uint64_t kAdvancedRawRevision =
      SnapshotFixture3D::kLatestRawRevision + 1U;
  const std::shared_ptr<const VersionedExecutionInput3D> progress_input =
      SnapshotFixture3D::progressInput(*active, {kRetainedStationM, 0.0, 5.0});
  const ExecutionRouteTransitionResult3D progress = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active),
      fixture.executionObservation({kRetainedStationM, 0.0, 5.0}, kAdvancedRawRevision,
                                   &fixture.raw_occupancy),
      progress_input, fixture.rawWorld(kAdvancedRawRevision));
  ASSERT_TRUE(progress.applied());
  if (progress.next == nullptr || !progress.next->route.has_value() ||
      !progress.next->finite_execution.has_value()) {
    ADD_FAILURE() << "Progress must preserve the route and finite execution";
    return;
  }
  const ExecutionRouteSnapshot3D& progressed = *progress.next;
  const CertifiedRouteSuffix3D& route =
      progressed.route.value(); // NOLINT(bugprone-unchecked-optional-access)
  const FiniteExecutionState3D& resident_execution =
      progressed.finite_execution.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_DOUBLE_EQ(route.progress.station_m, kRetainedStationM);

  FiniteExecutionCertification3D command =
      SnapshotFixture3D::finiteCertificationForRoute(
          route, FiniteExecutionKind3D::kNominal, 102U, 56U, 0U, kRetainedStationM,
          route.progress.execution_input.get(),
          resident_execution.latest_lidar_evidence.get());
  ASSERT_NE(command.execution_input, nullptr);
  mppi::State lagging_state = command.execution_input->state();
  lagging_state.x -= 0.1F;
  const VersionedExecutionInput3D& generated_input = *command.execution_input;
  command.execution_input = VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
      .capture_sequence = generated_input.captureSequence(),
      .pose_revision = generated_input.poseRevision(),
      .pose_source_timestamp_us = generated_input.poseSourceTimestampUs(),
      .pose_receive_stamp_ns = generated_input.poseReceiveStampNs(),
      .effective_stamp_ns = generated_input.effectiveStampNs(),
      .state = lagging_state,
      .full_state_authoritative = generated_input.fullStateAuthoritative(),
      .state_provenance = generated_input.stateProvenance(),
      .previous_control = generated_input.previousControl(),
      .previous_control_source = generated_input.previousControlSource(),
      .previous_control_source_producer_instance_id =
          generated_input.previousControlSourceProducerInstanceId(),
      .previous_control_source_sequence =
          generated_input.previousControlSourceSequence(),
      .previous_control_source_stamp_ns =
          generated_input.previousControlSourceStampNs(),
      .previous_control_receive_stamp_ns =
          generated_input.previousControlReceiveStampNs(),
  });
  ASSERT_NE(command.execution_input, nullptr);

  const std::optional<mppi::FiniteHorizon> stationary_horizon =
      mppi::buildFiniteBrakingHorizon(lagging_state, command.horizon.controls.size(),
                                      route.validation_policy->dynamics(),
                                      command.execution_input->previousControl());
  if (!stationary_horizon.has_value()) {
    ADD_FAILURE() << "The lagging state must produce a finite braking horizon";
    return;
  }
  command.horizon = stationary_horizon.value();

  FiniteExecutionPlanCertificationResult3D certification =
      certifyFiniteExecutionPlan3DDetailed(
          progressed, route,
          FiniteExecutionPlanCertification3D{
              .command_horizon = std::move(command),
              .braking_tail = stationary_horizon.value(),
          });
  ASSERT_TRUE(certification.certified())
      << "command_status="
      << finiteExecutionCertificationStatus3DName(certification.command_horizon.status)
      << " braking_status="
      << finiteExecutionCertificationStatus3DName(certification.braking_tail.status);
  if (!certification.plan.has_value()) {
    ADD_FAILURE() << "Certification must return the complete atomic plan";
    return;
  }
  FiniteExecutionPlan3D plan = std::move(certification.plan.value());
  EXPECT_DOUBLE_EQ(plan.command_horizon.begin_route_station_m, kRetainedStationM);
  EXPECT_DOUBLE_EQ(plan.command_horizon.stop_boundary.station_m, kRetainedStationM);
  EXPECT_DOUBLE_EQ(plan.braking_tail.stop_boundary.station_m, kRetainedStationM);

  const ExecutionRouteTransitionResult3D installed = replaceFiniteExecutionPlan3D(
      progressed, SnapshotFixture3D::guard(progressed), std::move(plan));
  ASSERT_TRUE(installed.applied())
      << "transition_status=" << executionRouteTransitionStatus3DName(installed.status);
  if (installed.next == nullptr || !installed.next->route.has_value()) {
    ADD_FAILURE() << "Installation must preserve the route owner";
    return;
  }
  const ExecutionRouteSnapshot3D& installed_snapshot = *installed.next;
  const CertifiedRouteSuffix3D& installed_route =
      installed_snapshot.route.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_DOUBLE_EQ(installed_route.progress.station_m, kRetainedStationM);
  EXPECT_TRUE(installed_snapshot.publishable());
}

TEST(ExecutionRouteSnapshot3DTest, SafeReplacementCannotExtendDeadline) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  if (active == nullptr) {
    ADD_FAILURE() << "The fixture must activate a route snapshot";
    return;
  }
  const ExecutionRouteSnapshot3D& active_snapshot = *active;
  if (!active_snapshot.finite_execution.has_value() ||
      !active_snapshot.braking_fallback.has_value()) {
    ADD_FAILURE() << "An active route must own a complete execution plan";
    return;
  }
  const RouteLifecycleEvent3D superseded{
      .kind = RouteLifecycleEventKind3D::kObjectiveSuperseded,
      .generation = SnapshotFixture3D::kRouteGeneration,
  };

  const FiniteExecutionState3D& fallback = active_snapshot.braking_fallback.value();
  const ExecutionRouteTransitionResult3D accepted =
      retireCertifiedRoute3D(active_snapshot, SnapshotFixture3D::guard(active_snapshot),
                             superseded, std::nullopt);
  if (!accepted.applied() || accepted.next == nullptr) {
    ADD_FAILURE() << "Retirement must produce a braking snapshot";
    return;
  }
  const ExecutionRouteSnapshot3D& braking_snapshot = *accepted.next;
  if (!braking_snapshot.finite_execution.has_value()) {
    ADD_FAILURE() << "Retirement must select the permanent braking fallback";
    return;
  }
  EXPECT_EQ(braking_snapshot.phase, ExecutionRoutePhase3D::kBraking);
  EXPECT_EQ(
      braking_snapshot.finite_execution.value().validation_proof.artifact_fingerprint,
      fallback.validation_proof.artifact_fingerprint);

  const FiniteExecutionState3D retained = SnapshotFixture3D::finiteExecution(
      active_snapshot, FiniteExecutionKind3D::kRetained, true, 101U);
  EXPECT_EQ(retireCertifiedRoute3D(active_snapshot,
                                   SnapshotFixture3D::guard(active_snapshot),
                                   superseded, retained)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  const FiniteExecutionState3D extended_deadline = SnapshotFixture3D::finiteExecution(
      active_snapshot, FiniteExecutionKind3D::kEmergencyBrakeTail, true, 102U, 55U, 1U);
  EXPECT_GT(extended_deadline.valid_until_ns,
            active_snapshot.finite_execution.value().valid_until_ns);
  EXPECT_EQ(retireCertifiedRoute3D(active_snapshot,
                                   SnapshotFixture3D::guard(active_snapshot),
                                   superseded, extended_deadline)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
}

} // namespace
} // namespace drone_city_nav
