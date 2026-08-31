#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(ExecutionRouteSnapshot3DTest, RejectsSnapshotVersionOverflow) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ExecutionPlan3D exhausted = *active;
  exhausted.version = std::numeric_limits<std::uint64_t>::max();
  ASSERT_TRUE(exhausted.valid());
  FiniteExecutionState3D finite = SnapshotFixture3D::finiteExecution(exhausted);

  const ExecutionRouteTransitionResult3D replacement =
      replaceFiniteExecution3D(exhausted, SnapshotFixture3D::guard(exhausted), finite);

  EXPECT_EQ(replacement.status, ExecutionRouteTransitionStatus3D::kVersionExhausted);
  EXPECT_FALSE(replacement.next);
}

TEST(ExecutionRouteSnapshot3DTest,
     RevocationPublishesOnlyAgainstTheExactSnapshotAndPreservesHighWater) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  RouteExecutionManager3D manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> initial = manager.plan();
  ASSERT_NE(initial, nullptr);
  CertifiedRouteSuffix3D copied_route = *suffix;
  ASSERT_EQ(copied_route.geometry, suffix->geometry);
  ASSERT_TRUE(copied_route.valid());
  ASSERT_EQ(copied_route.route_instance_id, suffix->route_instance_id);
  FiniteExecutionState3D execution = SnapshotFixture3D::finiteExecutionForRoute(
      *initial, copied_route, FiniteExecutionKind3D::kNominal, true, 100U);
  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *initial, initial->version, copied_route, std::move(execution));
  ASSERT_TRUE(activation.applied());
  ASSERT_EQ(manager.publishDetachedTransition(initial_authority, activation),
            ExecutionRoutePublicationStatus3D::kPublished);
  const std::shared_ptr<const CommittedExecutionAuthority3D> active_authority =
      manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> active = manager.plan();
  ASSERT_EQ(active, activation.next);
  const ExecutionRouteTransitionResult3D revocation =
      revokeExecution3D(*active, active->version);
  ASSERT_TRUE(revocation.applied());

  EXPECT_EQ(manager.publishDetachedTransition(initial_authority, revocation),
            ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion);
  EXPECT_EQ(manager.plan(), active);
  ASSERT_EQ(manager.publishDetachedTransition(active_authority, revocation),
            ExecutionRoutePublicationStatus3D::kPublished);
  const std::shared_ptr<const ExecutionPlan3D> revoked = manager.plan();
  ASSERT_EQ(revoked, revocation.next);
  EXPECT_EQ(revoked->phase(), ExecutionRoutePhase3D::kRevoked);
  EXPECT_EQ(revoked->routeGenerationHighWater(), active->routeGenerationHighWater());
  EXPECT_EQ(revoked->execution_owner_epoch, active->execution_owner_epoch + 1U);
  EXPECT_EQ(revokeExecution3D(*revoked, revoked->version).status,
            ExecutionRouteTransitionStatus3D::kNoChange);
}

TEST(ExecutionRouteSnapshot3DTest,
     RouteBasedPendingPublicationRequiresACertifiedSplice) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const auto pending =
      std::make_shared<const PendingCertifiedRoute3D>(PendingCertifiedRoute3D{
          .publication_sequence = 1U,
          .base_execution_owner_epoch = active->execution_owner_epoch,
          .base_kind = PendingExecutionBaseKind3D::kRoute,
          .base_route_generation = active->route()->identity.generation,
          .base_geometry_revision =
              active->route()->geometry->compiled_trajectory_revision,
          .base_continuity_id = active->route()->continuity_id,
          .base_direct_tracking_identity = std::nullopt,
          .route_splice = std::nullopt,
          .route = *successor,
      });

  RouteExecutionManager3D manager;
  EXPECT_FALSE(pending->valid());
  EXPECT_FALSE(publishPendingDraftForCurrentBase(manager, *pending));
  EXPECT_EQ(manager.pending(), nullptr);
}

TEST(ExecutionRouteSnapshot3DTest,
     PendingCommitConsumesOnlyAfterSuccessfulExecutionCas) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const auto pending =
      std::make_shared<const PendingCertifiedRoute3D>(PendingCertifiedRoute3D{
          .publication_sequence = 1U,
          .base_execution_owner_epoch = 1U,
          .base_kind = PendingExecutionBaseKind3D::kEmpty,
          .base_route_generation = 0U,
          .base_geometry_revision = 0U,
          .base_continuity_id = 0U,
          .base_direct_tracking_identity = std::nullopt,
          .route_splice = std::nullopt,
          .route = *suffix,
      });

  RouteExecutionManager3D manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> initial = manager.plan();
  ASSERT_NE(initial, nullptr);
  FiniteExecutionState3D execution = SnapshotFixture3D::finiteExecutionForRoute(
      *initial, *suffix, FiniteExecutionKind3D::kNominal, true, 100U);
  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *initial, initial->version, *suffix, std::move(execution));
  ASSERT_TRUE(activation.applied());

  ASSERT_TRUE(publishPendingDraftForCurrentBase(manager, *pending));
  const std::shared_ptr<const PendingCertifiedRoute3D> sealed = manager.pending();
  ASSERT_NE(sealed, nullptr);
  const RouteExecutionManagerSnapshot3D before_commit = manager.snapshot();
  ASSERT_TRUE(before_commit.valid());
  EXPECT_EQ(before_commit.plan(), initial);
  EXPECT_EQ(before_commit.pending, sealed);
  EXPECT_TRUE(manager.commitPendingLeasedTransitionIfSame(
      sealed, initial_authority, activation,
      SnapshotFixture3D::committedOwner(*activation.next),
      SnapshotFixture3D::committedInput(*activation.next)));
  const RouteExecutionManagerSnapshot3D after_commit = manager.snapshot();
  ASSERT_TRUE(after_commit.valid());
  EXPECT_EQ(after_commit.plan(), activation.next);
  EXPECT_EQ(after_commit.pending, nullptr);

  const std::optional<CertifiedRouteSuffix3D> recertified = recertifyExecutionRoute3D(
      *suffix, fixture.activation().observation, suffix->observed_raw_world);
  ASSERT_TRUE(recertified.has_value());
  ASSERT_NE(recertified->route_instance_id, suffix->route_instance_id);
  ASSERT_EQ(recertified->parent_route_instance_id,
            std::optional<RouteInstanceId3D>{suffix->route_instance_id});
  RouteExecutionManager3D recertified_manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D>
      recertified_initial_authority = recertified_manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> recertified_initial =
      recertified_manager.plan();
  ASSERT_NE(recertified_initial, nullptr);
  FiniteExecutionState3D recertified_execution =
      SnapshotFixture3D::finiteExecutionForRoute(*recertified_initial, *recertified,
                                                 FiniteExecutionKind3D::kNominal, true,
                                                 101U);
  const ExecutionRouteTransitionResult3D recertified_activation =
      activateCertifiedRoute3D(*recertified_initial, recertified_initial->version,
                               *recertified, std::move(recertified_execution));
  ASSERT_TRUE(recertified_activation.applied());
  ASSERT_TRUE(publishPendingDraftForCurrentBase(recertified_manager, *pending));
  const std::shared_ptr<const PendingCertifiedRoute3D> recertified_pending =
      recertified_manager.pending();
  ASSERT_NE(recertified_pending, nullptr);
  EXPECT_TRUE(recertified_manager.commitPendingLeasedTransitionIfSame(
      recertified_pending, recertified_initial_authority, recertified_activation,
      SnapshotFixture3D::committedOwner(*recertified_activation.next),
      SnapshotFixture3D::committedInput(*recertified_activation.next)));
  EXPECT_EQ(recertified_manager.plan(), recertified_activation.next);
  EXPECT_EQ(recertified_manager.pending(), nullptr);

  RouteExecutionManager3D retained_manager;
  RouteExecutionManager3D foreign_manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> retained_authority =
      retained_manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> rejected_initial =
      retained_manager.plan();
  const std::shared_ptr<const ExecutionPlan3D> foreign_initial = foreign_manager.plan();
  ASSERT_NE(rejected_initial, nullptr);
  ASSERT_NE(foreign_initial, nullptr);
  FiniteExecutionState3D foreign_execution = SnapshotFixture3D::finiteExecutionForRoute(
      *foreign_initial, *suffix, FiniteExecutionKind3D::kNominal, true, 101U);
  const ExecutionRouteTransitionResult3D foreign_activation =
      activateCertifiedRoute3D(*foreign_initial, foreign_initial->version, *suffix,
                               std::move(foreign_execution));
  ASSERT_TRUE(foreign_activation.applied());

  ASSERT_TRUE(publishPendingDraftForCurrentBase(retained_manager, *pending));
  const std::shared_ptr<const PendingCertifiedRoute3D> retained =
      retained_manager.pending();
  ASSERT_NE(retained, nullptr);
  EXPECT_FALSE(retained_manager.commitPendingLeasedTransitionIfSame(
      retained, retained_authority, foreign_activation,
      SnapshotFixture3D::committedOwner(*foreign_activation.next),
      SnapshotFixture3D::committedInput(*foreign_activation.next)));
  EXPECT_EQ(retained_manager.plan(), rejected_initial);
  EXPECT_EQ(retained_manager.pending(), retained);

  const std::optional<CertifiedRouteSuffix3D> unrelated = fixture.certify();
  ASSERT_TRUE(unrelated.has_value());
  ASSERT_NE(unrelated->route_instance_id, suffix->route_instance_id);
  ASSERT_FALSE(unrelated->parent_route_instance_id.has_value());
  RouteExecutionManager3D unrelated_manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> unrelated_authority =
      unrelated_manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> unrelated_initial =
      unrelated_manager.plan();
  ASSERT_NE(unrelated_initial, nullptr);
  FiniteExecutionState3D unrelated_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *unrelated_initial, *unrelated, FiniteExecutionKind3D::kNominal, true, 102U);
  const ExecutionRouteTransitionResult3D unrelated_activation =
      activateCertifiedRoute3D(*unrelated_initial, unrelated_initial->version,
                               *unrelated, std::move(unrelated_execution));
  ASSERT_TRUE(unrelated_activation.applied());
  ASSERT_TRUE(publishPendingDraftForCurrentBase(unrelated_manager, *pending));
  const std::shared_ptr<const PendingCertifiedRoute3D> unrelated_pending =
      unrelated_manager.pending();
  ASSERT_NE(unrelated_pending, nullptr);
  EXPECT_FALSE(unrelated_manager.commitPendingLeasedTransitionIfSame(
      unrelated_pending, unrelated_authority, unrelated_activation,
      SnapshotFixture3D::committedOwner(*unrelated_activation.next),
      SnapshotFixture3D::committedInput(*unrelated_activation.next)));
  EXPECT_EQ(unrelated_manager.plan(), unrelated_initial);
  EXPECT_EQ(unrelated_manager.pending(), unrelated_pending);
}

TEST(ExecutionRouteSnapshot3DTest, RouteSplicePendingSurvivesExecutionProgressCasLoss) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> base = fixture.certify();
  ASSERT_TRUE(base.has_value());
  RouteExecutionManager3D manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> initial = manager.plan();
  ASSERT_NE(initial, nullptr);
  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *initial, initial->version, *base,
      SnapshotFixture3D::finiteExecutionForRoute(
          *initial, *base, FiniteExecutionKind3D::kNominal, true, 100U));
  ASSERT_TRUE(activation.applied());
  ASSERT_EQ(manager.publishDetachedTransition(initial_authority, activation),
            ExecutionRoutePublicationStatus3D::kPublished);
  const std::shared_ptr<const CommittedExecutionAuthority3D> active_authority =
      manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> active = manager.plan();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const CertifiedRouteSplice3D splice = testRouteSplice(*active->route(), *successor);
  const ExecutionRouteTransitionResult3D replacement = replaceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active), *successor,
      SnapshotFixture3D::finiteExecutionForRoute(
          *active, *successor, FiniteExecutionKind3D::kNominal, true, 101U),
      splice);
  ASSERT_TRUE(replacement.applied());
  const auto pending =
      std::make_shared<const PendingCertifiedRoute3D>(PendingCertifiedRoute3D{
          .publication_sequence = 1U,
          .base_execution_owner_epoch = active->execution_owner_epoch,
          .base_kind = PendingExecutionBaseKind3D::kRoute,
          .base_route_generation = active->route()->identity.generation,
          .base_geometry_revision =
              active->route()->geometry->compiled_trajectory_revision,
          .base_continuity_id = active->route()->continuity_id,
          .base_direct_tracking_identity = std::nullopt,
          .route_splice = splice,
          .route = *successor,
      });
  ASSERT_TRUE(publishPendingDraftForCurrentBase(manager, *pending));
  const std::shared_ptr<const PendingCertifiedRoute3D> sealed = manager.pending();
  ASSERT_NE(sealed, nullptr);

  constexpr std::uint64_t kAdvancedRawRevision =
      SnapshotFixture3D::kLatestRawRevision + 1U;
  const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active),
      fixture.executionObservation({4.0, 0.0, 5.0}, kAdvancedRawRevision,
                                   &fixture.raw_occupancy),
      SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0}),
      fixture.rawWorld(kAdvancedRawRevision));
  ASSERT_TRUE(advanced.applied());
  // A progress-only preparation no longer crosses the controller-visible CAS
  // boundary. It must be combined with a complete command/braking plan.
  ASSERT_EQ(manager.publishDetachedTransition(active_authority, advanced),
            ExecutionRoutePublicationStatus3D::kInvalidCandidate);

  EXPECT_TRUE(manager.commitPendingLeasedTransitionIfSame(
      sealed, active_authority, replacement,
      SnapshotFixture3D::committedOwner(*replacement.next),
      SnapshotFixture3D::committedInput(*replacement.next)));
  EXPECT_EQ(manager.plan(), replacement.next);
  EXPECT_EQ(manager.pending(), nullptr);
}

TEST(ExecutionRouteSnapshot3DTest,
     ConcurrentNewerPlanningCompletionCannotDisplaceCapturedActivation) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const auto first =
      std::make_shared<const PendingCertifiedRoute3D>(PendingCertifiedRoute3D{
          .publication_sequence = 1U,
          .base_execution_owner_epoch = 1U,
          .base_kind = PendingExecutionBaseKind3D::kEmpty,
          .base_route_generation = 0U,
          .base_geometry_revision = 0U,
          .base_continuity_id = 0U,
          .base_direct_tracking_identity = std::nullopt,
          .route_splice = std::nullopt,
          .route = *suffix,
      });
  auto newer_value = *first;
  newer_value.publication_sequence = 2U;
  const auto newer =
      std::make_shared<const PendingCertifiedRoute3D>(std::move(newer_value));

  RouteExecutionManager3D manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      manager.authority();
  const std::shared_ptr<const ExecutionPlan3D> initial = manager.plan();
  ASSERT_NE(initial, nullptr);
  FiniteExecutionState3D execution = SnapshotFixture3D::finiteExecutionForRoute(
      *initial, *suffix, FiniteExecutionKind3D::kNominal, true, 100U);
  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *initial, initial->version, *suffix, std::move(execution));
  ASSERT_TRUE(activation.applied());

  ASSERT_TRUE(publishPendingDraftForCurrentBase(manager, *first));
  std::barrier pending_captured{2};
  std::barrier pending_replaced{2};
  std::shared_ptr<const PendingCertifiedRoute3D> captured;
  bool committed{true};
  bool newer_published{false};

  std::thread consumer{[&] {
    captured = manager.pending();
    pending_captured.arrive_and_wait();
    pending_replaced.arrive_and_wait();
    committed = manager.commitPendingLeasedTransitionIfSame(
        captured, initial_authority, activation,
        SnapshotFixture3D::committedOwner(*activation.next),
        SnapshotFixture3D::committedInput(*activation.next));
  }};
  std::thread producer{[&] {
    pending_captured.arrive_and_wait();
    newer_published = publishPendingDraftForCurrentBase(manager, *newer);
    pending_replaced.arrive_and_wait();
  }};

  consumer.join();
  producer.join();

  ASSERT_NE(captured, nullptr);
  EXPECT_EQ(captured->publication_sequence, 1U);
  EXPECT_FALSE(newer_published);
  EXPECT_TRUE(committed);
  EXPECT_EQ(manager.plan(), activation.next);
  EXPECT_EQ(manager.pending(), nullptr);
}

TEST(ExecutionRouteSnapshot3DTest,
     PendingEligibilityAcceptsSemanticProgressButRejectsWrongBaseIdentity) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ASSERT_TRUE(active->route() != nullptr);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const PendingCertifiedRoute3D pending{
      .publication_sequence = 1U,
      .base_execution_owner_epoch = active->execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kRoute,
      .base_route_generation = active->route()->identity.generation,
      .base_geometry_revision = active->route()->geometry->compiled_trajectory_revision,
      .base_continuity_id = active->route()->continuity_id,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = testRouteSplice(*active->route(), *successor),
      .route = *successor,
  };
  ASSERT_TRUE(pending.valid());
  EXPECT_TRUE(pendingCertifiedRouteEligible3D(pending, *active));

  const ExecutionRouteTransitionResult3D newer_finite =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(*active));
  ASSERT_TRUE(newer_finite.applied());
  EXPECT_TRUE(pendingCertifiedRouteEligible3D(pending, *newer_finite.next));

  PendingCertifiedRoute3D wrong_geometry = pending;
  ++wrong_geometry.base_geometry_revision;
  EXPECT_FALSE(wrong_geometry.valid());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(wrong_geometry, *newer_finite.next));

  PendingCertifiedRoute3D wrong_continuity = pending;
  ++wrong_continuity.base_continuity_id;
  EXPECT_FALSE(wrong_continuity.valid());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(wrong_continuity, *newer_finite.next));

  PendingCertifiedRoute3D wrong_generation = pending;
  ++wrong_generation.base_route_generation;
  EXPECT_FALSE(wrong_generation.valid());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(wrong_generation, *newer_finite.next));
}

TEST(ExecutionRouteSnapshot3DTest,
     PendingRouteHandoffTracksOwnerWithoutRequiringOverlapProof) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ASSERT_TRUE(active->route() != nullptr);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = active->routeGenerationHighWater() + 1U;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const PendingCertifiedRoute3D pending{
      .publication_sequence = 1U,
      .base_execution_owner_epoch = active->execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kRouteHandoff,
      .base_route_generation = active->route()->identity.generation,
      .base_geometry_revision = active->route()->geometry->compiled_trajectory_revision,
      .base_continuity_id = active->route()->continuity_id,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = std::nullopt,
      .route = *successor,
  };

  ASSERT_TRUE(pending.valid());
  EXPECT_TRUE(pendingCertifiedRouteEligible3D(pending, *active));
  PendingCertifiedRoute3D unexpected_splice = pending;
  unexpected_splice.route_splice = testRouteSplice(*active->route(), *successor);
  EXPECT_FALSE(unexpected_splice.valid());
}

TEST(ExecutionRouteSnapshot3DTest, PendingInitialLineageDoesNotAliasRevokedOwner) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> first_route = fixture.certify();
  ASSERT_TRUE(first_route.has_value());
  const std::shared_ptr<const ExecutionPlan3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  const PendingCertifiedRoute3D initial_pending{
      .publication_sequence = 1U,
      .base_execution_owner_epoch = initial->execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kEmpty,
      .base_route_generation = 0U,
      .base_geometry_revision = 0U,
      .base_continuity_id = 0U,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = std::nullopt,
      .route = *first_route,
  };
  ASSERT_TRUE(initial_pending.valid());
  EXPECT_TRUE(pendingCertifiedRouteEligible3D(initial_pending, *initial));

  const ExecutionRouteTransitionResult3D revoked =
      revokeExecution3D(*initial, initial->version);
  ASSERT_TRUE(revoked.applied());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(initial_pending, *revoked.next));

  PendingCertifiedRoute3D revoked_pending = initial_pending;
  revoked_pending.base_execution_owner_epoch = revoked.next->execution_owner_epoch;
  revoked_pending.base_kind = PendingExecutionBaseKind3D::kRevoked;
  ASSERT_TRUE(revoked_pending.valid());
  EXPECT_TRUE(pendingCertifiedRouteEligible3D(revoked_pending, *revoked.next));
}

TEST(ExecutionRouteSnapshot3DTest,
     PendingHoldLineageSurvivesEvidenceRefreshButNotOwnerReplacement) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  const ExecutionRouteTransitionResult3D held = transferToExecutionHold3D(
      *active, active->version, SnapshotFixture3D::holdCertification(*active));
  ASSERT_TRUE(held.applied());
  ASSERT_NE(held.next, nullptr);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = active->routeGenerationHighWater() + 1U;
  successor_activation.observation.position = held.next->stationaryHold()->position;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const PendingCertifiedRoute3D pending{
      .publication_sequence = 1U,
      .base_execution_owner_epoch = held.next->execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kStationaryHold,
      .base_route_generation = held.next->routeGenerationHighWater(),
      .base_geometry_revision = 0U,
      .base_continuity_id = 0U,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = std::nullopt,
      .route = *successor,
  };
  ASSERT_TRUE(pending.valid());
  EXPECT_TRUE(pendingCertifiedRouteEligible3D(pending, *held.next));

  StationaryExecutionHoldCertification3D refresh =
      SnapshotFixture3D::holdCertification(*held.next);
  refresh.latest_lidar_evidence =
      SnapshotFixture3D::newerLidarEvidence(*refresh.latest_lidar_evidence);
  const ExecutionRouteTransitionResult3D refreshed =
      transferToExecutionHold3D(*held.next, held.next->version, std::move(refresh));
  ASSERT_TRUE(refreshed.applied());
  EXPECT_NE(refreshed.next->version, held.next->version);
  EXPECT_EQ(refreshed.next->execution_owner_epoch, held.next->execution_owner_epoch);
  EXPECT_TRUE(pendingCertifiedRouteEligible3D(pending, *refreshed.next));

  PendingCertifiedRoute3D wrong_owner = pending;
  ++wrong_owner.base_execution_owner_epoch;
  ASSERT_TRUE(wrong_owner.valid());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(wrong_owner, *refreshed.next));

  ExecutionPlan3D wrong_high_water = *refreshed.next;
  ++wrong_high_water.route_generation_high_water;
  ASSERT_TRUE(wrong_high_water.valid());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(pending, wrong_high_water));

  ExecutionPlan3D empty_owner = *refreshed.next;
  empty_owner.state = AwaitingSuccessorPlan3D{};
  ASSERT_TRUE(empty_owner.valid());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(pending, empty_owner));

  PendingCertifiedRoute3D invalid_hold = pending;
  invalid_hold.base_execution_owner_epoch = 0U;
  EXPECT_FALSE(invalid_hold.valid());
  invalid_hold = pending;
  invalid_hold.base_geometry_revision = 1U;
  EXPECT_FALSE(invalid_hold.valid());
  invalid_hold = pending;
  invalid_hold.base_continuity_id = 1U;
  EXPECT_FALSE(invalid_hold.valid());
  invalid_hold = pending;
  ++invalid_hold.base_route_generation;
  EXPECT_FALSE(invalid_hold.valid());
  invalid_hold = pending;
  invalid_hold.base_route_generation = std::numeric_limits<std::uint64_t>::max();
  EXPECT_FALSE(invalid_hold.valid());

  const DirectTrackingOwnerIdentity3D direct_identity{
      .mission_epoch = fixture.objective.mission_epoch,
      .assignment_generation = fixture.objective.assignment_generation,
      .target_detection_id = 41U,
      .target_track_id = 42U,
      .objective_sample_sequence = fixture.objective.sample_sequence,
      .line_of_sight_generation = 7U,
  };
  invalid_hold = pending;
  invalid_hold.base_direct_tracking_identity = direct_identity;
  EXPECT_FALSE(invalid_hold.valid());

  ExecutionRouteActivation3D discontinuous_activation = fixture.activation();
  discontinuous_activation.route_generation = active->routeGenerationHighWater() + 1U;
  const std::optional<CertifiedRouteSuffix3D> discontinuous_successor =
      certifyExecutionRoute3D(discontinuous_activation);
  ASSERT_TRUE(discontinuous_successor.has_value());
  const std::optional<FiniteExecutionState3D> discontinuous_execution =
      certifyFiniteExecution3D(
          *refreshed.next, *discontinuous_successor,
          SnapshotFixture3D::finiteCertificationForRoute(
              *discontinuous_successor, FiniteExecutionKind3D::kNominal, 102U));
  ASSERT_TRUE(discontinuous_execution.has_value());
  EXPECT_EQ(activateCertifiedRoute3D(*refreshed.next, refreshed.next->version,
                                     *discontinuous_successor, *discontinuous_execution)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  FiniteExecutionState3D successor_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *refreshed.next, *successor, FiniteExecutionKind3D::kNominal, true, 103U);
  const ExecutionRouteTransitionResult3D activated =
      activateCertifiedRoute3D(*refreshed.next, refreshed.next->version, *successor,
                               std::move(successor_execution));
  ASSERT_TRUE(activated.applied())
      << executionRouteTransitionStatus3DName(activated.status);
  ASSERT_NE(activated.next, nullptr);
  EXPECT_FALSE(activated.next->stationaryHold() != nullptr);
  ASSERT_TRUE(activated.next->route() != nullptr);
  EXPECT_EQ(activated.next->route()->identity.generation,
            active->routeGenerationHighWater() + 1U);
  EXPECT_EQ(activated.next->routeGenerationHighWater(),
            active->routeGenerationHighWater() + 1U);
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(pending, *activated.next));
}

TEST(ExecutionRouteSnapshot3DTest,
     HoldResidentRejectsRetiredRouteAndDirectPendingLineage) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  const ExecutionRouteTransitionResult3D held = transferToExecutionHold3D(
      *active, active->version, SnapshotFixture3D::holdCertification(*active));
  ASSERT_TRUE(held.applied());

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = active->routeGenerationHighWater() + 1U;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const PendingCertifiedRoute3D route_pending{
      .publication_sequence = 1U,
      .base_execution_owner_epoch = active->execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kRoute,
      .base_route_generation = active->route()->identity.generation,
      .base_geometry_revision = active->route()->geometry->compiled_trajectory_revision,
      .base_continuity_id = active->route()->continuity_id,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = testRouteSplice(*active->route(), *successor),
      .route = *successor,
  };
  ASSERT_TRUE(route_pending.valid());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(route_pending, *held.next));

  const DirectTrackingOwnerIdentity3D direct_identity{
      .mission_epoch = fixture.objective.mission_epoch,
      .assignment_generation = fixture.objective.assignment_generation,
      .target_detection_id = 41U,
      .target_track_id = 42U,
      .objective_sample_sequence = fixture.objective.sample_sequence,
      .line_of_sight_generation = 7U,
  };
  const PendingCertifiedRoute3D direct_pending{
      .publication_sequence = 2U,
      .base_execution_owner_epoch = active->execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kDirectTracking,
      .base_route_generation = active->routeGenerationHighWater(),
      .base_geometry_revision = 0U,
      .base_continuity_id = 0U,
      .base_direct_tracking_identity = direct_identity,
      .route_splice = std::nullopt,
      .route = *successor,
  };
  ASSERT_TRUE(direct_pending.valid());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(direct_pending, *held.next));
}

TEST(ExecutionRouteSnapshot3DTest, PendingRouteIsObsoleteAtMissionTerminalStop) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  const ExecutionRouteTransitionResult3D at_endpoint = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active),
      fixture.executionObservation({10.0, 0.0, 5.0},
                                   SnapshotFixture3D::kLatestRawRevision + 1U,
                                   &fixture.raw_occupancy),
      SnapshotFixture3D::progressInput(*active, {10.0, 0.0, 5.0}),
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U));
  ASSERT_TRUE(at_endpoint.applied());
  const ExecutionRouteTransitionResult3D terminal_execution = replaceFiniteExecution3D(
      *at_endpoint.next, SnapshotFixture3D::guard(*at_endpoint.next),
      SnapshotFixture3D::finiteExecution(*at_endpoint.next));
  ASSERT_TRUE(terminal_execution.applied());
  const ExecutionRouteTransitionResult3D stopped = retireCertifiedRoute3D(
      *terminal_execution.next, SnapshotFixture3D::guard(*terminal_execution.next),
      RouteLifecycleEvent3D{
          .kind = RouteLifecycleEventKind3D::kCompleted,
          .generation = SnapshotFixture3D::kRouteGeneration,
      },
      std::nullopt);
  ASSERT_TRUE(stopped.applied());
  ASSERT_EQ(stopped.next->phase(), ExecutionRoutePhase3D::kStopped);
  ASSERT_TRUE(stopped.next->route() != nullptr);
  ASSERT_EQ(stopped.next->route()->planned_endpoint_semantics,
            RouteEndpointSemantics3D::kMissionStop);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = stopped.next->routeGenerationHighWater() + 1U;
  successor_activation.observation.position = {10.0, 0.0, 5.0};
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const PendingCertifiedRoute3D pending{
      .publication_sequence = 1U,
      .base_execution_owner_epoch = stopped.next->execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kRoute,
      .base_route_generation = stopped.next->route()->identity.generation,
      .base_geometry_revision =
          stopped.next->route()->geometry->compiled_trajectory_revision,
      .base_continuity_id = stopped.next->route()->continuity_id,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = std::nullopt,
      .route = *successor,
  };
  ASSERT_FALSE(pending.valid());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(pending, *stopped.next));
}

TEST(ExecutionRouteSnapshot3DTest,
     OwnedObservedWorldDerivesRouteEvidenceWithoutRecopyingObservation) {
  SnapshotFixture3D fixture;
  const auto observation =
      std::make_shared<const ObservedOccupancyGrid3D>(fixture.raw_occupancy);
  const RawMapVersion version{
      .producer_instance_id = SnapshotFixture3D::kRawProducer,
      .revision = SnapshotFixture3D::kLatestRawRevision,
  };
  const std::shared_ptr<const VersionedObservedRawWorld3D> owner =
      VersionedObservedRawWorld3D::captureOwned(version, observation, std::nullopt,
                                                std::nullopt);
  ASSERT_NE(owner, nullptr);
  const std::shared_ptr<const OccupancyGrid3D> occupied = owner->occupiedSnapshot();
  ASSERT_NE(occupied, nullptr);

  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = {2.0, 0.0, 5.0},
      .body_axis = {},
      .footprint = testPassageVolumeConfig().footprint,
  };
  const std::shared_ptr<const VersionedObservedRawWorld3D> derived =
      owner->deriveRouteEvidence(seed, std::nullopt);

  ASSERT_NE(derived, nullptr);
  EXPECT_TRUE(owner->sharesObservationOwner(*derived));
  EXPECT_EQ(derived->occupiedSnapshot(), occupied);
  EXPECT_EQ(derived->occupiedContentFingerprint(), owner->occupiedContentFingerprint());
  EXPECT_NE(derived->contentFingerprint(), owner->contentFingerprint());
  EXPECT_EQ(derived->version().revision, owner->version().revision);
}

TEST(ExecutionRouteSnapshot3DTest,
     DirectTrackingOwnerTransfersAtomicallyAndRejectsStaleLineage) {
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
  const std::optional<DirectTrackingFiniteExecution3D> first_execution =
      certifyDirectFixtureExecution(*route_owner, *route_owner->route(), identity,
                                    101U);
  ASSERT_TRUE(first_execution.has_value());
  const ExecutionRouteTransitionResult3D direct =
      transferToDirectTracking3D(*route_owner, route_owner->version, *first_execution);
  ASSERT_TRUE(direct.applied());
  ASSERT_NE(direct.next, nullptr);
  EXPECT_EQ(direct.next->version, route_owner->version + 1U);
  EXPECT_EQ(direct.next->phase(), ExecutionRoutePhase3D::kDirectTracking);
  EXPECT_FALSE(direct.next->route() != nullptr);
  EXPECT_FALSE(direct.next->finiteExecution() != nullptr);
  ASSERT_TRUE(direct.next->directTrackingExecution() != nullptr);
  EXPECT_EQ(direct.next->routeGenerationHighWater(),
            route_owner->route()->identity.generation);

  DirectTrackingOwnerIdentity3D updated_identity = identity;
  ++updated_identity.objective_sample_sequence;
  const std::optional<DirectTrackingFiniteExecution3D> updated_execution =
      certifyDirectFixtureExecution(*direct.next, *route_owner->route(),
                                    updated_identity, 102U);
  ASSERT_TRUE(updated_execution.has_value());
  const ExecutionRouteTransitionResult3D updated = replaceDirectTrackingExecution3D(
      *direct.next, direct.next->version, *updated_execution);
  ASSERT_TRUE(updated.applied());
  EXPECT_EQ(updated.next->version, direct.next->version + 1U);

  const ExecutionRouteTransitionResult3D stale_version =
      replaceDirectTrackingExecution3D(*updated.next, direct.next->version,
                                       *updated_execution);
  EXPECT_EQ(stale_version.status,
            ExecutionRouteTransitionStatus3D::kStaleSnapshotVersion);

  DirectTrackingOwnerIdentity3D wrong_owner = updated_identity;
  ++wrong_owner.target_track_id;
  const std::optional<DirectTrackingFiniteExecution3D> wrong_owner_execution =
      certifyDirectFixtureExecution(*updated.next, *route_owner->route(), wrong_owner,
                                    103U);
  ASSERT_TRUE(wrong_owner_execution.has_value());
  EXPECT_EQ(replaceDirectTrackingExecution3D(*updated.next, updated.next->version,
                                             *wrong_owner_execution)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  DirectTrackingOwnerIdentity3D regressed_sample = updated_identity;
  --regressed_sample.objective_sample_sequence;
  const std::optional<DirectTrackingFiniteExecution3D> regressed_execution =
      certifyDirectFixtureExecution(*updated.next, *route_owner->route(),
                                    regressed_sample, 103U);
  ASSERT_TRUE(regressed_execution.has_value());
  EXPECT_EQ(replaceDirectTrackingExecution3D(*updated.next, updated.next->version,
                                             *regressed_execution)
                .status,
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);

  const std::optional<DirectTrackingFiniteExecution3D> retained_execution =
      certifyDirectFixtureExecution(*updated.next, *route_owner->route(),
                                    updated_identity, 103U,
                                    FiniteExecutionKind3D::kRetained);
  ASSERT_TRUE(retained_execution.has_value());
  const ExecutionRouteTransitionResult3D retained = replaceDirectTrackingExecution3D(
      *updated.next, updated.next->version, *retained_execution);
  ASSERT_TRUE(retained.applied());
  EXPECT_EQ(retained.next->directTrackingExecution()->kind,
            FiniteExecutionKind3D::kRetained);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation =
      route_owner->route()->identity.generation + 1U;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  FiniteExecutionCertification3D successor_certification =
      SnapshotFixture3D::finiteCertificationForRoute(
          *successor, FiniteExecutionKind3D::kNominal, 104U, 104U, 0U, -1.0,
          retained.next->directTrackingExecution()->execution_input.get());
  const std::optional<FiniteExecutionState3D> successor_execution =
      certifyFiniteExecution3D(*retained.next, *successor,
                               std::move(successor_certification));
  ASSERT_TRUE(successor_execution.has_value());

  const PendingCertifiedRoute3D pending{
      .publication_sequence = 1U,
      .base_execution_owner_epoch = retained.next->execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kDirectTracking,
      .base_route_generation = retained.next->routeGenerationHighWater(),
      .base_geometry_revision = 0U,
      .base_continuity_id = 0U,
      .base_direct_tracking_identity = updated_identity,
      .route_splice = std::nullopt,
      .route = *successor,
  };
  ASSERT_TRUE(pending.valid());
  EXPECT_TRUE(pendingCertifiedRouteEligible3D(pending, *retained.next));
  PendingCertifiedRoute3D wrong_pending_identity = pending;
  ++wrong_pending_identity.base_direct_tracking_identity->line_of_sight_generation;
  EXPECT_TRUE(wrong_pending_identity.valid());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(wrong_pending_identity, *retained.next));

  const PendingCertifiedRoute3D pre_direct_pending{
      .publication_sequence = 2U,
      .base_execution_owner_epoch = route_owner->execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kRoute,
      .base_route_generation = route_owner->route()->identity.generation,
      .base_geometry_revision =
          route_owner->route()->geometry->compiled_trajectory_revision,
      .base_continuity_id = route_owner->route()->continuity_id,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = testRouteSplice(*route_owner->route(), *successor),
      .route = *successor,
  };
  ASSERT_TRUE(pre_direct_pending.valid());
  EXPECT_FALSE(pendingCertifiedRouteEligible3D(pre_direct_pending, *retained.next));

  const ExecutionRouteTransitionResult3D route_restored =
      transferDirectTrackingToCertifiedRoute3D(*retained.next, retained.next->version,
                                               *successor, *successor_execution);
  ASSERT_TRUE(route_restored.applied());
  EXPECT_EQ(route_restored.next->phase(), ExecutionRoutePhase3D::kFollowing);
  EXPECT_FALSE(route_restored.next->directTrackingExecution() != nullptr);
  ASSERT_TRUE(route_restored.next->route() != nullptr);
  ASSERT_TRUE(route_restored.next->finiteExecution() != nullptr);
  EXPECT_EQ(route_restored.next->route()->identity.generation,
            successor->identity.generation);
  EXPECT_EQ(route_restored.next->version, retained.next->version + 1U);
}

TEST(ExecutionRouteSnapshot3DTest, ProgressConnectorUsesExactPreviousControlBodyAxes) {
  SnapshotFixture3D fixture;
  const SweptFootprintConfig oriented_footprint{
      .radius_m = 0.1,
      .lower_extent_m = 1.0,
      .upper_extent_m = 1.0,
      .perimeter_samples = 8U,
      .radial_rings = 1U,
      .axial_samples = 5U,
      .sweep_step_m = 0.05,
  };
  mppi::DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  dynamics.linear_drag_1ps = 0.0F;
  fixture.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      FlightEnvelopeConfig{}, dynamics, mppi::AltitudeEnvelopeConfig{},
      oriented_footprint, 100.0);
  fixture.execution_footprint = oriented_footprint;
  fixture.passage_volume_config.footprint = oriented_footprint;
  TrajectoryCompilerConfig3D compiler_config;
  compiler_config.physical_footprint = oriented_footprint;
  const TrajectoryCompilationResult3D compilation =
      TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
          .exact_initial_state = testVehicleState(fixture.route.front().position),
          .route_generation = SnapshotFixture3D::kRouteGeneration,
          .route = fixture.route,
          .constrained_spans = {},
          .passage_volumes = {},
          .cooperative_passage_assignments = {},
          .selected_passage_traversal_ids = {},
          .passage_volume_config = fixture.passage_volume_config,
          .endpoint_semantics = RouteEndpointSemantics3D::kMissionStop,
          .materialized_route_fingerprint = fixture.physical_route_fingerprint,
          .tracking_world =
              TrackingErrorTubeWorld3D{
                  .observed_occupancy = &fixture.raw_occupancy,
                  .occupied_content_fingerprint =
                      fixture.raw_occupancy.occupiedSnapshot().contentFingerprint(),
              },
          .config = compiler_config,
      });
  ASSERT_TRUE(compilation.compiled());
  fixture.geometry = compilation.trajectory;
  fixture.geometry_revision = fixture.geometry->compiled_trajectory_revision;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_NE(active->route()->progress.execution_input, nullptr);
  const VersionedExecutionInput3D& old_input =
      *active->route()->progress.execution_input;

  ObservedOccupancyGrid3D latest_occupancy{
      GridBounds3D{-5.0, -5.0, 0.0, 0.1, 200, 100, 100}};
  const Point3 tilted_only_obstacle{2.7, 0.0, 5.7};
  const std::optional<GridIndex3D> obstacle_cell =
      latest_occupancy.worldToCell(tilted_only_obstacle);
  ASSERT_TRUE(obstacle_cell.has_value());
  ASSERT_TRUE(latest_occupancy.setState(*obstacle_cell, ObservedVoxelState::kOccupied));
  const mppi::Control tilted_control{.ax = 9.80665F};
  const FootprintBodyAxis old_axis = bodyAxisFromWorldAcceleration(
      Vec3{old_input.previousControl().ax, old_input.previousControl().ay,
           old_input.previousControl().az});
  const FootprintBodyAxis tilted_axis = bodyAxisFromWorldAcceleration(
      Vec3{tilted_control.ax, tilted_control.ay, tilted_control.az});
  const Point3 position{old_input.state().x, old_input.state().y, old_input.state().z};
  EXPECT_TRUE(validateRawSweptFootprint(latest_occupancy, position, old_axis, position,
                                        old_axis, oriented_footprint)
                  .accepted());
  EXPECT_FALSE(validateRawSweptFootprint(latest_occupancy, position, old_axis, position,
                                         tilted_axis, oriented_footprint)
                   .accepted());

  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
          .capture_sequence = old_input.captureSequence() + 1U,
          .pose_revision = old_input.poseRevision(),
          .pose_source_timestamp_us = old_input.poseSourceTimestampUs(),
          .pose_receive_stamp_ns = old_input.poseReceiveStampNs(),
          .effective_stamp_ns = old_input.effectiveStampNs(),
          .state = old_input.state(),
          .full_state_authoritative = old_input.fullStateAuthoritative(),
          .state_provenance = old_input.stateProvenance(),
          .previous_control = tilted_control,
          .previous_control_source = old_input.previousControlSource(),
          .previous_control_source_producer_instance_id =
              old_input.previousControlSourceProducerInstanceId(),
          .previous_control_source_sequence =
              old_input.previousControlSourceSequence() + 1U,
          .previous_control_source_stamp_ns =
              old_input.previousControlSourceStampNs() + 1LL,
          .previous_control_receive_stamp_ns =
              old_input.previousControlReceiveStampNs() + 1LL,
      });
  ASSERT_NE(current_input, nullptr);
  RouteExecutionObservation3D observation = fixture.executionObservation(
      position, SnapshotFixture3D::kLatestRawRevision + 1U, &latest_occupancy);
  observation.footprint = oriented_footprint;
  const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active), observation, current_input,
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &latest_occupancy));
  EXPECT_EQ(advanced.status,
            ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected);
}

} // namespace
} // namespace drone_city_nav
