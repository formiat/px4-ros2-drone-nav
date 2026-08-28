#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

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
