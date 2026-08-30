#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] PendingCertifiedRoute3D pendingForSnapshot(
    const ExecutionPlan3D& snapshot, const PendingExecutionBaseKind3D base_kind,
    const CertifiedRouteSuffix3D& route, const std::uint64_t publication_sequence) {
  const bool route_owner = base_kind == PendingExecutionBaseKind3D::kRoute ||
                           base_kind == PendingExecutionBaseKind3D::kRouteHandoff;
  const bool route_splice = base_kind == PendingExecutionBaseKind3D::kRoute;
  return PendingCertifiedRoute3D{
      .publication_sequence = publication_sequence,
      .base_execution_owner_epoch = snapshot.execution_owner_epoch,
      .base_kind = base_kind,
      .base_route_generation = snapshot.routeGenerationHighWater(),
      .base_geometry_revision =
          route_owner && snapshot.route() != nullptr
              ? snapshot.route()->geometry->compiled_trajectory_revision
              : 0U,
      .base_continuity_id = route_owner && snapshot.route() != nullptr
                                ? snapshot.route()->continuity_id
                                : 0U,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = route_splice && snapshot.route() != nullptr
                          ? std::optional<CertifiedRouteSplice3D>{testRouteSplice(
                                *snapshot.route(), route)}
                          : std::nullopt,
      .route = route,
  };
}

TEST(ExecutionRouteSnapshot3DTest,
     CertificationSharesTheSealedTrajectoryAndStartsAtTheActualProjection) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const CompiledTrajectory3D> original_geometry =
      fixture.geometry;

  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();

  ASSERT_TRUE(suffix.has_value());
  EXPECT_TRUE(suffix->valid());
  EXPECT_TRUE(suffix->route_instance_id.valid());
  EXPECT_TRUE(suffix->owner.valid());
  EXPECT_EQ(suffix->owner.active_intent.mission_epoch, fixture.objective.mission_epoch);
  EXPECT_EQ(suffix->geometry, original_geometry);
  EXPECT_EQ(suffix->geometry->route, original_geometry->route);
  EXPECT_EQ(routeFingerprint(*suffix->geometry->route),
            routeFingerprint(*original_geometry->route));
  EXPECT_EQ(suffix->geometry->compiled_trajectory_revision,
            original_geometry->compiled_trajectory_revision);
  EXPECT_DOUBLE_EQ(suffix->progress.station_m, 2.0);
  EXPECT_DOUBLE_EQ(suffix->remainingM(), 8.0);
  EXPECT_EQ(suffix->progress.route_generation, SnapshotFixture3D::kRouteGeneration);
  EXPECT_EQ(suffix->progress.geometry_revision, fixture.geometry_revision);
  EXPECT_NE(suffix->continuity_id, 0U);

  const ObservedRawRouteCertificate3D* const raw_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&suffix->certificate);
  ASSERT_NE(raw_certificate, nullptr);
  EXPECT_EQ(raw_certificate->route_instance_id, suffix->route_instance_id);
  EXPECT_EQ(raw_certificate->route_generation, SnapshotFixture3D::kRouteGeneration);
  EXPECT_EQ(raw_certificate->geometry_revision, fixture.geometry_revision);
  EXPECT_EQ(raw_certificate->physical_route_fingerprint,
            fixture.physical_route_fingerprint);
  EXPECT_EQ(raw_certificate->producer_instance_id, SnapshotFixture3D::kRawProducer);
  EXPECT_EQ(raw_certificate->validated_through_revision,
            SnapshotFixture3D::kLatestRawRevision);
  EXPECT_NE(raw_certificate->validation_policy_fingerprint, 0U);
  EXPECT_NE(raw_certificate->passage_geometry_revision, 0U);
  EXPECT_NE(raw_certificate->passage_volume_config_fingerprint, 0U);
  EXPECT_EQ(raw_certificate->geometry_derivation_occupancy_content_fingerprint,
            suffix->observed_raw_world->occupiedContentFingerprint());
  EXPECT_DOUBLE_EQ(raw_certificate->suffix_start_station_m, 2.0);
  EXPECT_DOUBLE_EQ(raw_certificate->certified_end_station_m, 10.0);

  const CertifiedRouteSuffix3D copied_suffix = *suffix;
  EXPECT_TRUE(copied_suffix.valid());
  EXPECT_EQ(copied_suffix.route_instance_id, suffix->route_instance_id);
  CertifiedRouteSuffix3D tampered_identity = copied_suffix;
  ++tampered_identity.route_instance_id.value;
  EXPECT_FALSE(tampered_identity.valid());

  const std::optional<CertifiedRouteSuffix3D> independently_certified =
      fixture.certify();
  ASSERT_TRUE(independently_certified.has_value());
  EXPECT_NE(independently_certified->route_instance_id, suffix->route_instance_id);
  EXPECT_NE(independently_certified->owner.id, suffix->owner.id);
}

TEST(ExecutionRouteSnapshot3DTest,
     PhysicalReplacementEligibilitySurvivesResidentBraking) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);

  const auto* const following = std::get_if<FollowingPlan3D>(&active->state);
  ASSERT_NE(following, nullptr);
  EXPECT_TRUE(executionRouteAcceptsCertifiedReplacement3D(*active));

  ExecutionPlan3D snapshot = *active;
  snapshot.state = BrakingPlan3D{
      .route = following->route,
      .execution = following->execution.braking_tail,
  };
  EXPECT_TRUE(executionRouteAcceptsCertifiedReplacement3D(snapshot));

  snapshot.state = AwaitingSuccessorPlan3D{
      .owner = SuspendedRoutePlan3D{.route = following->route},
  };
  EXPECT_TRUE(executionRouteAcceptsCertifiedReplacement3D(snapshot));

  snapshot.state = DirectTrackingPlan3D{};
  EXPECT_FALSE(executionRouteAcceptsCertifiedReplacement3D(snapshot));

  CertifiedRouteSuffix3D local_stop_route = following->route;
  local_stop_route.planned_endpoint_semantics = RouteEndpointSemantics3D::kLocalStop;
  snapshot.state = StationaryHoldPlan3D{
      .owner =
          CertifiedTerminalHoldPlan3D{
              .route = local_stop_route,
              .execution = following->execution,
          },
  };
  EXPECT_TRUE(executionRouteAcceptsCertifiedReplacement3D(snapshot));

  std::get<CertifiedTerminalHoldPlan3D>(
      std::get<StationaryHoldPlan3D>(snapshot.state).owner)
      .route.planned_endpoint_semantics = RouteEndpointSemantics3D::kMissionStop;
  EXPECT_FALSE(executionRouteAcceptsCertifiedReplacement3D(snapshot));

  snapshot.state = RevokedPlan3D{};
  EXPECT_FALSE(executionRouteAcceptsCertifiedReplacement3D(snapshot));
}

TEST(ExecutionRouteSnapshot3DTest,
     RecertificationReusesTheExactPreviouslySealedGeometryOwner) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> sealed = fixture.certify();
  ASSERT_TRUE(sealed.has_value());
  ASSERT_EQ(sealed->geometry, fixture.geometry);
  RouteActivationObservation3D observation = fixture.activation().observation;
  observation.position = Point3{3.0, 0.0, 5.0};

  const std::optional<CertifiedRouteSuffix3D> refreshed =
      recertifyExecutionRoute3D(*sealed, observation, sealed->observed_raw_world);

  ASSERT_TRUE(refreshed.has_value());
  EXPECT_NE(refreshed->route_instance_id, sealed->route_instance_id);
  EXPECT_EQ(refreshed->parent_route_instance_id,
            std::optional<RouteInstanceId3D>{sealed->route_instance_id});
  EXPECT_EQ(refreshed->geometry, sealed->geometry);
  EXPECT_EQ(refreshed->geometry->route, sealed->geometry->route);
  EXPECT_EQ(refreshed->owner.id, sealed->owner.id);
  EXPECT_TRUE(
      sameActiveIntent3D(refreshed->owner.active_intent, sealed->owner.active_intent));
  EXPECT_DOUBLE_EQ(refreshed->progress.station_m, 3.0);
  EXPECT_TRUE(refreshed->valid());

  EXPECT_FALSE(recertifyExecutionRoute3D(*sealed, observation, nullptr).has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     PendingRecertificationAdmitsASafeNewRawRevisionAndRejectsANewCollision) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> sealed = fixture.certify();
  ASSERT_TRUE(sealed.has_value());
  const CertifiedRouteSuffix3D& sealed_route = sealed.value();
  constexpr std::uint64_t kSafeRevision{SnapshotFixture3D::kLatestRawRevision + 1U};
  RouteActivationObservation3D safe_observation = fixture.observation();
  safe_observation.latest_raw_revision = kSafeRevision;
  const std::shared_ptr<const VersionedObservedRawWorld3D> safe_world =
      fixture.rawWorld(kSafeRevision);
  ASSERT_NE(safe_world, nullptr);
  safe_observation.latest_raw_occupancy = &safe_world->occupancy();

  const std::optional<CertifiedRouteSuffix3D> refreshed =
      recertifyExecutionRoute3D(sealed_route, safe_observation, safe_world);

  ASSERT_TRUE(refreshed.has_value());
  const CertifiedRouteSuffix3D& refreshed_route = refreshed.value();
  const auto* const refreshed_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&refreshed_route.certificate);
  ASSERT_NE(refreshed_certificate, nullptr);
  EXPECT_EQ(refreshed_certificate->validated_through_revision, kSafeRevision);
  EXPECT_EQ(refreshed_route.observed_raw_world, safe_world);

  ObservedOccupancyGrid3D blocked_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> blocked_cell =
      blocked_occupancy.worldToCell(Point3{5.0, 0.0, 5.0});
  ASSERT_TRUE(blocked_cell.has_value());
  ASSERT_TRUE(
      blocked_occupancy.setState(blocked_cell.value(), ObservedVoxelState::kOccupied));
  constexpr std::uint64_t kBlockedRevision{kSafeRevision + 1U};
  const std::shared_ptr<const VersionedObservedRawWorld3D> blocked_world =
      fixture.rawWorld(kBlockedRevision, &blocked_occupancy);
  ASSERT_NE(blocked_world, nullptr);
  RouteActivationObservation3D blocked_observation = safe_observation;
  blocked_observation.latest_raw_revision = kBlockedRevision;
  blocked_observation.latest_raw_occupancy = &blocked_world->occupancy();

  EXPECT_FALSE(
      recertifyExecutionRoute3D(sealed_route, blocked_observation, blocked_world)
          .has_value());
}

TEST(ExecutionRouteSnapshot3DTest, RejectsSameSizeGeometryWithAStaleFingerprint) {
  SnapshotFixture3D fixture;
  std::vector<RouteSample3D> changed = fixture.route;
  changed[1].position.y = 1.0;
  ExecutionRouteActivation3D activation = fixture.activation();
  activation.geometry = makeGeometry(changed, fixture.physical_route_fingerprint);

  EXPECT_EQ(changed.size(), fixture.route.size());
  EXPECT_FALSE(certifyExecutionRoute3D(activation).has_value());

  activation = fixture.activation();
  ++activation.proposal.route_fingerprint;
  EXPECT_FALSE(certifyExecutionRoute3D(activation).has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     AcceptsUnconstrainedGeometryWithPresentEmptySidecars) {
  SnapshotFixture3D fixture;

  ASSERT_NE(fixture.geometry->route, nullptr);
  ASSERT_NE(fixture.geometry->tracking_error_tube, nullptr);
  ASSERT_NE(fixture.geometry->constrained_spans, nullptr);
  ASSERT_NE(fixture.geometry->passage_volumes, nullptr);
  ASSERT_NE(fixture.geometry->cooperative_passage_assignments, nullptr);
  ASSERT_NE(fixture.geometry->selected_passage_traversal_ids, nullptr);
  EXPECT_FALSE(fixture.geometry->route->empty());
  EXPECT_TRUE(fixture.geometry->constrained_spans->empty());
  EXPECT_TRUE(fixture.geometry->passage_volumes->empty());
  EXPECT_TRUE(fixture.geometry->cooperative_passage_assignments->empty());
  EXPECT_TRUE(fixture.geometry->selected_passage_traversal_ids->empty());
  EXPECT_EQ(fixture.geometry->compiled_trajectory_revision,
            compiledTrajectoryRevision3D(*fixture.geometry));
  EXPECT_TRUE(fixture.certify().has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     RejectsWrongCandidateGenerationForASealedConstrainedTrajectory) {
  SnapshotFixture3D fixture;
  const auto geometry = makeConstrainedGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      SnapshotFixture3D::kRouteGeneration, fixture.raw_occupancy.occupiedSnapshot(),
      testPassageVolumeConfig());
  ASSERT_NE(geometry->compiled_trajectory_revision, 0U);

  ExecutionRouteActivation3D activation = fixture.activation();
  ++activation.route_generation;
  activation.geometry = geometry;

  EXPECT_FALSE(certifyExecutionRoute3D(activation).has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     TrajectoryCompilerRejectsCrossResourcePassageInconsistency) {
  SnapshotFixture3D fixture;
  const auto valid = makeConstrainedGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      SnapshotFixture3D::kRouteGeneration, fixture.raw_occupancy.occupiedSnapshot(),
      testPassageVolumeConfig());
  const OccupancyGrid3D occupied = fixture.raw_occupancy.occupiedSnapshot();
  std::vector<PassageVolume> inconsistent_volumes = *valid->passage_volumes;
  inconsistent_volumes.front().passage_traversal_id = "mismatched_volume";

  const TrajectoryCompilationResult3D rejected =
      TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
          .exact_initial_state = valid->exact_initial_state,
          .route_generation = SnapshotFixture3D::kRouteGeneration,
          .route = *valid->route,
          .constrained_spans = *valid->constrained_spans,
          .passage_volumes = std::move(inconsistent_volumes),
          .cooperative_passage_assignments = *valid->cooperative_passage_assignments,
          .selected_passage_traversal_ids = *valid->selected_passage_traversal_ids,
          .passage_volume_config = valid->passage_volume_config,
          .endpoint_semantics = valid->endpoint_semantics,
          .materialized_route_fingerprint = valid->materialized_route_fingerprint,
          .tracking_world =
              TrackingErrorTubeWorld3D{
                  .occupancy = &occupied,
                  .occupied_content_fingerprint = occupied.contentFingerprint(),
              },
      });

  EXPECT_FALSE(rejected.compiled());
  EXPECT_EQ(rejected.validation.reason,
            CompiledTrajectoryFailureReason3D::kInvalidPassageResources);
  EXPECT_EQ(rejected.trajectory, nullptr);
}

TEST(ExecutionRouteSnapshot3DTest,
     RejectsPassageGeometryDerivedFromDifferentObservedWorld) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const CompiledTrajectory3D> geometry = makeConstrainedGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      SnapshotFixture3D::kRouteGeneration, fixture.raw_occupancy.occupiedSnapshot(),
      testPassageVolumeConfig());
  ExecutionRouteActivation3D matching_activation = fixture.activation();
  matching_activation.geometry = geometry;
  ASSERT_TRUE(certifyExecutionRoute3D(matching_activation).has_value());

  ObservedOccupancyGrid3D changed_world = fixture.raw_occupancy;
  const std::optional<GridIndex3D> lateral_wall =
      changed_world.worldToCell(Point3{5.0, 2.0, 5.0});
  ASSERT_TRUE(lateral_wall.has_value());
  ASSERT_TRUE(changed_world.setState(*lateral_wall, ObservedVoxelState::kOccupied));
  ExecutionRouteActivation3D mismatched_activation = fixture.activation();
  mismatched_activation.geometry = geometry;
  mismatched_activation.observed_raw_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision, &changed_world);
  ASSERT_TRUE(mismatched_activation.observed_raw_world);

  EXPECT_FALSE(certifyExecutionRoute3D(mismatched_activation).has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     RejectsPassageGeometryDerivedFromDifferentStaticWorld) {
  SnapshotFixture3D fixture;
  const OccupancyGrid3D clear_world{fixture.raw_occupancy.bounds(),
                                    fixture.validated_world.esdf_fingerprint};
  const std::shared_ptr<const CompiledTrajectory3D> geometry = makeConstrainedGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      SnapshotFixture3D::kRouteGeneration, clear_world, testPassageVolumeConfig());
  ExecutionRouteActivation3D matching_activation =
      staticActivation(fixture, clear_world);
  matching_activation.geometry = geometry;
  ASSERT_TRUE(certifyExecutionRoute3D(matching_activation).has_value());

  OccupancyGrid3D changed_world{fixture.raw_occupancy.bounds(),
                                fixture.validated_world.esdf_fingerprint};
  const std::optional<GridIndex3D> lateral_wall =
      changed_world.worldToCell(Point3{5.0, 2.0, 5.0});
  ASSERT_TRUE(lateral_wall.has_value());
  changed_world.setOccupied(*lateral_wall);
  ExecutionRouteActivation3D mismatched_activation =
      staticActivation(fixture, changed_world);
  mismatched_activation.geometry = geometry;

  EXPECT_FALSE(certifyExecutionRoute3D(mismatched_activation).has_value());
}

TEST(ExecutionRouteSnapshot3DTest, RejectsPassageDerivationConfigurationMismatch) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const CompiledTrajectory3D> geometry = makeConstrainedGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      SnapshotFixture3D::kRouteGeneration, fixture.raw_occupancy.occupiedSnapshot(),
      testPassageVolumeConfig());
  ExecutionRouteActivation3D activation = fixture.activation();
  activation.geometry = geometry;
  ASSERT_TRUE(certifyExecutionRoute3D(activation).has_value());

  activation.passage_volume_config.minimum_wall_clearance_m += 0.25;
  EXPECT_FALSE(certifyExecutionRoute3D(activation).has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     StaticCertificationBindsTheExactValidatedWorldWithoutClaimingRawProof) {
  SnapshotFixture3D fixture;
  const OccupancyGrid3D static_occupancy{fixture.raw_occupancy.bounds(),
                                         fixture.validated_world.esdf_fingerprint};
  ExecutionRouteActivation3D activation = staticActivation(fixture, static_occupancy);
  ASSERT_TRUE(activation.static_world);

  const std::optional<CertifiedRouteSuffix3D> suffix =
      certifyExecutionRoute3D(activation);

  ASSERT_TRUE(suffix.has_value());
  const StaticRouteCertificate3D* const certificate =
      std::get_if<StaticRouteCertificate3D>(&suffix->certificate);
  ASSERT_NE(certificate, nullptr);
  EXPECT_EQ(certificate->world_certificate.esdf_fingerprint,
            activation.proposal.validated_world.esdf_fingerprint);
  EXPECT_EQ(certificate->world_certificate.raw_validated_through_revision,
            activation.proposal.validated_world.raw_validated_through_revision);
  EXPECT_DOUBLE_EQ(certificate->suffix_start_station_m, 2.0);

  const std::shared_ptr<const ExecutionPlan3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_TRUE(initial);
  FiniteExecutionState3D initial_execution = SnapshotFixture3D::finiteExecutionForRoute(
      *initial, *suffix, FiniteExecutionKind3D::kNominal, true, 100U);
  const ExecutionRouteTransitionResult3D active = activateCertifiedRoute3D(
      *initial, initial->version, suffix.value(), std::move(initial_execution));
  ASSERT_TRUE(active.applied());
  const ExecutionRouteTransitionResult3D finite =
      replaceFiniteExecution3D(*active.next, SnapshotFixture3D::guard(*active.next),
                               SnapshotFixture3D::finiteExecution(*active.next));
  EXPECT_TRUE(finite.applied());
}

TEST(ExecutionRouteSnapshot3DTest,
     CapturedWorldOwnersAreFrozenExclusiveAndContentBound) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> raw_suffix = fixture.certify();
  ASSERT_TRUE(raw_suffix.has_value());
  ASSERT_TRUE(raw_suffix->observed_raw_world);
  const std::uint64_t captured_raw_content =
      raw_suffix->observed_raw_world->contentFingerprint();

  const GridIndex3D distant_cell{0, 0, 0};
  ASSERT_TRUE(
      fixture.raw_occupancy.setState(distant_cell, ObservedVoxelState::kOccupied));
  EXPECT_EQ(raw_suffix->observed_raw_world->contentFingerprint(), captured_raw_content);
  EXPECT_EQ(raw_suffix->observed_raw_world->occupancy().state(distant_cell),
            ObservedVoxelState::kUnknown);

  CertifiedRouteSuffix3D missing_owner = *raw_suffix;
  missing_owner.observed_raw_world.reset();
  EXPECT_FALSE(missing_owner.valid());

  CertifiedRouteSuffix3D missing_policy = *raw_suffix;
  missing_policy.validation_policy.reset();
  EXPECT_FALSE(missing_policy.valid());

  mppi::DynamicsConfig changed_dynamics = raw_suffix->validation_policy->dynamics();
  changed_dynamics.dt_s *= 2.0F;
  CertifiedRouteSuffix3D changed_policy = *raw_suffix;
  changed_policy.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      raw_suffix->validation_policy->flightEnvelope(), changed_dynamics,
      raw_suffix->validation_policy->altitudeEnvelope(),
      raw_suffix->validation_policy->sweptFootprint(),
      raw_suffix->validation_policy->latestLidarMaximumAgeMs());
  ASSERT_NE(changed_policy.validation_policy, nullptr);
  EXPECT_FALSE(changed_policy.valid());

  CertifiedRouteSuffix3D swapped_content = *raw_suffix;
  swapped_content.observed_raw_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision, &fixture.raw_occupancy);
  ASSERT_TRUE(swapped_content.observed_raw_world);
  EXPECT_NE(swapped_content.observed_raw_world->contentFingerprint(),
            captured_raw_content);
  EXPECT_FALSE(swapped_content.valid());

  const OccupancyGrid3D static_occupancy{
      fixture.raw_occupancy.bounds(),
      fixture.validated_world.esdf_source_occupied_fingerprint};
  CertifiedRouteSuffix3D both_owners = *raw_suffix;
  both_owners.static_world =
      VersionedStaticWorld3D::capture(fixture.validated_world, static_occupancy);
  ASSERT_TRUE(both_owners.static_world);
  EXPECT_FALSE(both_owners.valid());

  OccupancyGrid3D mutable_static{fixture.raw_occupancy.bounds(),
                                 fixture.validated_world.esdf_fingerprint};
  ExecutionRouteActivation3D static_activation =
      staticActivation(fixture, mutable_static);
  const std::optional<CertifiedRouteSuffix3D> static_suffix =
      certifyExecutionRoute3D(static_activation);
  ASSERT_TRUE(static_suffix.has_value());
  ASSERT_TRUE(static_suffix->static_world);
  const std::uint64_t captured_static_content =
      static_suffix->static_world->contentFingerprint();
  mutable_static.setOccupied(distant_cell);
  EXPECT_EQ(static_suffix->static_world->contentFingerprint(), captured_static_content);
  EXPECT_FALSE(static_suffix->static_world->occupancy().isOccupied(distant_cell));

  CertifiedRouteSuffix3D missing_static_owner = *static_suffix;
  missing_static_owner.static_world.reset();
  EXPECT_FALSE(missing_static_owner.valid());
  CertifiedRouteSuffix3D changed_static_owner = *static_suffix;
  changed_static_owner.static_world = VersionedStaticWorld3D::capture(
      static_activation.proposal.validated_world, mutable_static);
  ASSERT_TRUE(changed_static_owner.static_world);
  EXPECT_NE(changed_static_owner.static_world->contentFingerprint(),
            captured_static_content);
  EXPECT_FALSE(changed_static_owner.valid());
}

TEST(ExecutionRouteSnapshot3DTest,
     OwnedStaticWorldCapturePreservesExactOccupancyOwnerAndFailsClosed) {
  SnapshotFixture3D fixture;
  const OccupancyGrid3D static_occupancy{fixture.raw_occupancy.bounds(),
                                         fixture.validated_world.esdf_fingerprint};
  const ExecutionRouteActivation3D activation =
      staticActivation(fixture, static_occupancy);
  const NavigationWorldCertificate3D certificate = activation.proposal.validated_world;
  const auto occupancy = std::make_shared<const OccupancyGrid3D>(static_occupancy);
  const std::uint64_t content_fingerprint = occupancy->contentFingerprint();

  const std::shared_ptr<const VersionedStaticWorld3D> owner =
      VersionedStaticWorld3D::captureOwned(certificate, occupancy);

  ASSERT_NE(owner, nullptr);
  EXPECT_EQ(&owner->occupancy(), occupancy.get());
  EXPECT_TRUE(owner->valid());
  EXPECT_NE(content_fingerprint, 0U);
  EXPECT_EQ(owner->contentFingerprint(), content_fingerprint);
  EXPECT_EQ(owner->certificate().esdf_fingerprint, certificate.esdf_fingerprint);

  EXPECT_EQ(VersionedStaticWorld3D::captureOwned(certificate, nullptr), nullptr);
  const auto mismatched_occupancy = std::make_shared<const OccupancyGrid3D>(
      fixture.raw_occupancy.bounds(), certificate.esdf_fingerprint + 1U);
  EXPECT_EQ(VersionedStaticWorld3D::captureOwned(certificate, mismatched_occupancy),
            nullptr);
}

TEST(ExecutionRouteSnapshot3DTest,
     ObservedWorldOwnerAuthenticatesLaunchSupportAgainstItsOccupancy) {
  SnapshotFixture3D fixture;
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = {2.0, 0.0, 5.0},
      .body_axis = {},
      .footprint = SweptFootprintConfig{.radius_m = 0.5,
                                        .lower_extent_m = 0.25,
                                        .upper_extent_m = 0.25},
  };
  const LaunchSupportContact3D canonical =
      makeVehicleLandedSupportContact3D(fixture.raw_occupancy.bounds(), seed);
  ASSERT_TRUE(launchSupportContactValid3D(canonical));
  const RawMapVersion version{
      .producer_instance_id = SnapshotFixture3D::kRawProducer,
      .revision = SnapshotFixture3D::kLatestRawRevision,
  };
  EXPECT_NE(VersionedObservedRawWorld3D::capture(version, fixture.raw_occupancy, seed,
                                                 canonical),
            nullptr);

  LaunchSupportContact3D negative_lateral = canonical;
  negative_lateral.maximum_lateral_departure_m = -0.1;
  EXPECT_FALSE(launchSupportContactValid3D(negative_lateral));
  EXPECT_EQ(VersionedObservedRawWorld3D::capture(version, fixture.raw_occupancy, seed,
                                                 negative_lateral),
            nullptr);

  LaunchSupportContact3D unknown_source = canonical;
  unknown_source.evidence_source = static_cast<LaunchSupportEvidenceSource>(255U);
  EXPECT_FALSE(launchSupportContactValid3D(unknown_source));
  EXPECT_EQ(VersionedObservedRawWorld3D::capture(version, fixture.raw_occupancy, seed,
                                                 unknown_source),
            nullptr);

  LaunchSupportContact3D inverted_box = canonical;
  std::swap(inverted_box.contact_cells.front().minimum.x,
            inverted_box.contact_cells.front().maximum.x);
  EXPECT_FALSE(launchSupportContactValid3D(inverted_box));
  EXPECT_EQ(VersionedObservedRawWorld3D::capture(version, fixture.raw_occupancy, seed,
                                                 inverted_box),
            nullptr);

  LaunchSupportContact3D forged_cell = canonical;
  forged_cell.contact_cells.front().minimum.x += 0.1;
  forged_cell.contact_cells.front().maximum.x += 0.1;
  ASSERT_TRUE(launchSupportContactValid3D(forged_cell));
  EXPECT_EQ(VersionedObservedRawWorld3D::capture(version, fixture.raw_occupancy, seed,
                                                 forged_cell),
            nullptr);

  LaunchSupportContact3D forged_evidence_count = canonical;
  forged_evidence_count.occupied_evidence_cells = 1U;
  EXPECT_FALSE(launchSupportContactValid3D(forged_evidence_count));
  EXPECT_EQ(VersionedObservedRawWorld3D::capture(version, fixture.raw_occupancy, seed,
                                                 forged_evidence_count),
            nullptr);
}

TEST(ExecutionRouteSnapshot3DTest,
     EmptyAndRevokedOwnersRecoverFromAnUnrefreshableEligiblePendingRoute) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_TRUE(route.has_value());
  const std::shared_ptr<const ExecutionPlan3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  const ExecutionRouteTransitionResult3D revoked =
      revokeExecution3D(*initial, initial->version);
  ASSERT_TRUE(revoked.applied());

  const auto exercise_recovery = [&](const std::shared_ptr<const ExecutionPlan3D>& base,
                                     const PendingExecutionBaseKind3D base_kind) {
    const auto pending = std::make_shared<const PendingCertifiedRoute3D>(
        pendingForSnapshot(*base, base_kind, *route, 1U));
    ASSERT_TRUE(pending->valid());
    ASSERT_TRUE(pendingCertifiedRouteEligible3D(*pending, *base));
    RouteExecutionManager3D manager;
    ASSERT_TRUE(manager.publishPending(pending));
    const std::shared_ptr<const PendingCertifiedRoute3D> expected = manager.pending();
    ASSERT_NE(expected, nullptr);

    // pending_activation=false is the post-resolve signal that current
    // recertification could not produce an executable activation.
    const PendingCertifiedRouteRecoveryResult3D recovery =
        recoverPendingCertifiedRouteLiveness3D(
            manager, expected, PendingCertifiedRouteRecoveryObservation3D{});

    EXPECT_TRUE(recovery.pending_acknowledged);
    EXPECT_TRUE(recovery.request_successor);
    EXPECT_EQ(manager.pending(), nullptr);
  };

  exercise_recovery(initial, PendingExecutionBaseKind3D::kEmpty);
  exercise_recovery(revoked.next, PendingExecutionBaseKind3D::kRevoked);
}

TEST(ExecutionRouteSnapshot3DTest,
     SpliceFreeRouteHandoffRetainsItsSnapshotCertificateUntilFiniteCommit) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = active->routeGenerationHighWater() + 1U;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const CertifiedRouteSuffix3D successor_route =
      successor.value_or(CertifiedRouteSuffix3D{});
  ASSERT_TRUE(successor_route.valid());
  const PendingCertifiedRoute3D pending = pendingForSnapshot(
      *active, PendingExecutionBaseKind3D::kRouteHandoff, successor_route, 1U);
  ASSERT_TRUE(pending.valid());

  EXPECT_TRUE(pendingCertifiedRouteRetainsSnapshotCertificate3D(pending));

  const PendingCertifiedRoute3D splice_candidate = pendingForSnapshot(
      *active, PendingExecutionBaseKind3D::kRoute, successor_route, 2U);
  ASSERT_TRUE(splice_candidate.valid());
  EXPECT_FALSE(pendingCertifiedRouteRetainsSnapshotCertificate3D(splice_candidate));
}

TEST(ExecutionRouteSnapshot3DTest,
     PendingRecoveryAcknowledgesTheStableResidentBeforeAcceptingANewerRoute) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_TRUE(route.has_value());
  const std::shared_ptr<const ExecutionPlan3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  const auto first = std::make_shared<const PendingCertifiedRoute3D>(
      pendingForSnapshot(*initial, PendingExecutionBaseKind3D::kEmpty, *route, 1U));
  PendingCertifiedRoute3D newer_value = *first;
  newer_value.publication_sequence = 2U;
  const auto newer =
      std::make_shared<const PendingCertifiedRoute3D>(std::move(newer_value));
  RouteExecutionManager3D manager;
  ASSERT_TRUE(manager.publishPending(first));
  const std::shared_ptr<const PendingCertifiedRoute3D> captured = manager.pending();
  ASSERT_NE(captured, nullptr);
  ASSERT_FALSE(manager.publishPending(newer));

  const PendingCertifiedRouteRecoveryResult3D recovery =
      recoverPendingCertifiedRouteLiveness3D(
          manager, captured, PendingCertifiedRouteRecoveryObservation3D{});

  EXPECT_TRUE(recovery.pending_acknowledged);
  EXPECT_TRUE(recovery.request_successor);
  EXPECT_EQ(manager.pending(), nullptr);
  ASSERT_TRUE(manager.publishPending(newer));
  const std::shared_ptr<const PendingCertifiedRoute3D> resident = manager.pending();
  ASSERT_NE(resident, nullptr);
  EXPECT_EQ(resident->publication_sequence, 2U);
}

TEST(ExecutionRouteSnapshot3DTest,
     PendingRecoveryConfirmsTheSharedMailboxBeforeTreatingNullAsEmpty) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_TRUE(route.has_value());
  const std::shared_ptr<const ExecutionPlan3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  const auto pending = std::make_shared<const PendingCertifiedRoute3D>(
      pendingForSnapshot(*initial, PendingExecutionBaseKind3D::kEmpty, *route, 1U));
  RouteExecutionManager3D manager;
  ASSERT_TRUE(manager.publishPending(pending));

  const PendingCertifiedRouteRecoveryResult3D occupied =
      recoverPendingCertifiedRouteLiveness3D(
          manager, nullptr, PendingCertifiedRouteRecoveryObservation3D{});
  EXPECT_FALSE(occupied.pending_acknowledged);
  EXPECT_FALSE(occupied.request_successor);
  const std::shared_ptr<const PendingCertifiedRoute3D> resident = manager.pending();
  ASSERT_NE(resident, nullptr);
  EXPECT_EQ(resident->publication_sequence, pending->publication_sequence);
  EXPECT_EQ(resident->route.identity.generation, pending->route.identity.generation);
  ASSERT_NE(resident->route.geometry, nullptr);
  ASSERT_NE(pending->route.geometry, nullptr);
  EXPECT_EQ(resident->route.geometry->compiled_trajectory_revision,
            pending->route.geometry->compiled_trajectory_revision);

  ASSERT_TRUE(manager.acknowledgePendingIfSame(resident));
  const PendingCertifiedRouteRecoveryResult3D empty =
      recoverPendingCertifiedRouteLiveness3D(
          manager, nullptr, PendingCertifiedRouteRecoveryObservation3D{});
  EXPECT_FALSE(empty.pending_acknowledged);
  EXPECT_TRUE(empty.request_successor);
}

TEST(ExecutionRouteSnapshot3DTest,
     DirectTrackingSuppressesPendingRecoveryAndSuccessorRequest) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_TRUE(route.has_value());
  const std::shared_ptr<const ExecutionPlan3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  const auto pending = std::make_shared<const PendingCertifiedRoute3D>(
      pendingForSnapshot(*initial, PendingExecutionBaseKind3D::kEmpty, *route, 1U));
  RouteExecutionManager3D manager;
  ASSERT_TRUE(manager.publishPending(pending));

  const PendingCertifiedRouteRecoveryResult3D recovery =
      recoverPendingCertifiedRouteLiveness3D(manager, nullptr,
                                             PendingCertifiedRouteRecoveryObservation3D{
                                                 .direct_tracking_requested = true,
                                             });

  EXPECT_FALSE(recovery.pending_acknowledged);
  EXPECT_FALSE(recovery.request_successor);
  EXPECT_NE(manager.pending(), nullptr);
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteAndStationaryHoldOwnersSuppressPendingRecovery) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(active->finiteExecution() != nullptr);
  const ExecutionRouteTransitionResult3D held = transferToExecutionHold3D(
      *active, active->version, SnapshotFixture3D::holdCertification(*active));
  ASSERT_TRUE(held.applied());
  ASSERT_NE(held.next, nullptr);
  ASSERT_TRUE(held.next->stationaryHold() != nullptr);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = active->routeGenerationHighWater() + 1U;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const auto exercise_owner = [&](const std::shared_ptr<const ExecutionPlan3D>& owner,
                                  const PendingExecutionBaseKind3D base_kind) {
    const auto pending = std::make_shared<const PendingCertifiedRoute3D>(
        pendingForSnapshot(*owner, base_kind, *successor, 1U));
    ASSERT_TRUE(pending->valid());
    ASSERT_TRUE(pendingCertifiedRouteEligible3D(*pending, *owner));
    RouteExecutionManager3D manager;
    ASSERT_TRUE(manager.publishPending(pending));
    const std::shared_ptr<const PendingCertifiedRoute3D> expected = manager.pending();
    ASSERT_NE(expected, nullptr);

    const bool execution_owner_available =
        owner->finiteExecution() != nullptr ||
        owner->directTrackingExecution() != nullptr ||
        owner->stationaryHold() != nullptr;
    const PendingCertifiedRouteRecoveryResult3D recovery =
        recoverPendingCertifiedRouteLiveness3D(
            manager, expected,
            PendingCertifiedRouteRecoveryObservation3D{
                .execution_owner_available = execution_owner_available,
            });

    EXPECT_FALSE(recovery.pending_acknowledged);
    EXPECT_FALSE(recovery.request_successor);
    EXPECT_EQ(manager.pending(), expected);
  };

  exercise_owner(active, PendingExecutionBaseKind3D::kRoute);
  exercise_owner(active, PendingExecutionBaseKind3D::kRouteHandoff);
  exercise_owner(held.next, PendingExecutionBaseKind3D::kStationaryHold);
}

} // namespace
} // namespace drone_city_nav
