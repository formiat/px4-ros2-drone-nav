#include "execution_route_snapshot_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] PendingCertifiedRoute3D
pendingForSnapshot(const ExecutionRouteSnapshot3D& snapshot,
                   const PendingExecutionBaseKind3D base_kind,
                   const CertifiedRouteSuffix3D& route,
                   const std::uint64_t publication_sequence) {
  const bool route_base = base_kind == PendingExecutionBaseKind3D::kRoute;
  return PendingCertifiedRoute3D{
      .publication_sequence = publication_sequence,
      .base_execution_owner_epoch = snapshot.execution_owner_epoch,
      .base_kind = base_kind,
      .base_route_generation = snapshot.routeGenerationHighWater(),
      .base_geometry_revision =
          route_base && snapshot.route.has_value()
              ? snapshot.route->geometry->executable_geometry_revision
              : 0U,
      .base_continuity_id =
          route_base && snapshot.route.has_value() ? snapshot.route->continuity_id : 0U,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = route_base && snapshot.route.has_value()
                          ? std::optional<CertifiedRouteSplice3D>{testRouteSplice(
                                *snapshot.route, route)}
                          : std::nullopt,
      .route = route,
  };
}

TEST(ExecutionRouteSnapshot3DTest,
     CertificationOwnsImmutableGeometryAndStartsAtTheActualProjection) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteGeometry3D> original_geometry =
      fixture.geometry;

  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();

  ASSERT_TRUE(suffix.has_value());
  EXPECT_TRUE(suffix->valid());
  EXPECT_NE(suffix->geometry, original_geometry);
  EXPECT_NE(suffix->geometry->route, original_geometry->route);
  EXPECT_EQ(routeFingerprint(*suffix->geometry->route),
            routeFingerprint(*original_geometry->route));
  EXPECT_EQ(suffix->geometry->executable_geometry_revision,
            original_geometry->executable_geometry_revision);
  EXPECT_DOUBLE_EQ(suffix->progress.station_m, 2.0);
  EXPECT_DOUBLE_EQ(suffix->remainingM(), 8.0);
  EXPECT_EQ(suffix->progress.route_generation, SnapshotFixture3D::kRouteGeneration);
  EXPECT_EQ(suffix->progress.geometry_revision, fixture.geometry_revision);
  EXPECT_NE(suffix->continuity_id, 0U);

  const ObservedRawRouteCertificate3D* const raw_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&suffix->certificate);
  ASSERT_NE(raw_certificate, nullptr);
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
  EXPECT_EQ(raw_certificate->passage_derivation_occupancy_content_fingerprint,
            suffix->observed_raw_world->occupiedContentFingerprint());
  EXPECT_DOUBLE_EQ(raw_certificate->suffix_start_station_m, 2.0);
  EXPECT_DOUBLE_EQ(raw_certificate->certified_end_station_m, 10.0);
}

TEST(ExecutionRouteSnapshot3DTest,
     CertifiedGeometryIsIsolatedFromRetainedMutableAliases) {
  SnapshotFixture3D fixture;
  auto mutable_route = std::make_shared<std::vector<RouteSample3D>>(fixture.route);
  auto mutable_geometry = std::make_shared<ExecutionRouteGeometry3D>(*fixture.geometry);
  mutable_geometry->route = mutable_route;
  mutable_geometry->executable_geometry_revision =
      executionRouteGeometryRevision3D(*mutable_geometry);
  ExecutionRouteActivation3D activation = fixture.activation();
  activation.geometry = mutable_geometry;

  const std::optional<CertifiedRouteSuffix3D> suffix =
      certifyExecutionRoute3D(activation);
  ASSERT_TRUE(suffix.has_value());
  ASSERT_NE(suffix->geometry->route, mutable_route);
  const Point3 captured_middle = suffix->geometry->route->at(1U).position;

  mutable_route->at(1U).position.y = 99.0;
  mutable_geometry->route_purpose = Lattice3DRoutePurpose::kObservationFrontier;

  EXPECT_TRUE(suffix->valid());
  EXPECT_DOUBLE_EQ(suffix->geometry->route->at(1U).position.x, captured_middle.x);
  EXPECT_DOUBLE_EQ(suffix->geometry->route->at(1U).position.y, captured_middle.y);
  EXPECT_DOUBLE_EQ(suffix->geometry->route->at(1U).position.z, captured_middle.z);
  EXPECT_EQ(suffix->geometry->route_purpose, Lattice3DRoutePurpose::kMissionTransit);
}

TEST(ExecutionRouteSnapshot3DTest,
     RecertificationReusesTheExactPreviouslySealedGeometryOwner) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> sealed = fixture.certify();
  ASSERT_TRUE(sealed.has_value());
  ASSERT_NE(sealed->geometry, fixture.geometry);
  RouteActivationObservation3D observation = fixture.activation().observation;
  observation.position = Point3{3.0, 0.0, 5.0};

  const std::optional<CertifiedRouteSuffix3D> refreshed =
      recertifyExecutionRoute3D(*sealed, observation, sealed->observed_raw_world);

  ASSERT_TRUE(refreshed.has_value());
  EXPECT_EQ(refreshed->geometry, sealed->geometry);
  EXPECT_EQ(refreshed->geometry->route, sealed->geometry->route);
  EXPECT_DOUBLE_EQ(refreshed->progress.station_m, 3.0);
  EXPECT_TRUE(refreshed->valid());

  EXPECT_FALSE(recertifyExecutionRoute3D(*sealed, observation, nullptr).has_value());
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
     RejectsAnExecutableSidecarChangedWithoutANewCanonicalRevision) {
  SnapshotFixture3D fixture;
  auto changed_geometry = std::make_shared<ExecutionRouteGeometry3D>(*fixture.geometry);
  auto changed_mppi_route =
      std::make_shared<std::vector<mppi::RouteSample3D>>(*fixture.geometry->mppi_route);
  changed_mppi_route->at(1).reference_speed_mps += 1.0F;
  changed_geometry->mppi_route = std::move(changed_mppi_route);
  ExecutionRouteActivation3D activation = fixture.activation();
  activation.geometry = changed_geometry;

  EXPECT_FALSE(certifyExecutionRoute3D(activation).has_value());

  changed_geometry->executable_geometry_revision =
      executionRouteGeometryRevision3D(*changed_geometry);
  EXPECT_TRUE(certifyExecutionRoute3D(activation).has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     AcceptsUnconstrainedGeometryWithPresentEmptySidecars) {
  SnapshotFixture3D fixture;

  ASSERT_NE(fixture.geometry->mppi_route, nullptr);
  ASSERT_NE(fixture.geometry->route, nullptr);
  ASSERT_NE(fixture.geometry->route_2d_projection, nullptr);
  ASSERT_NE(fixture.geometry->constrained_spans, nullptr);
  ASSERT_NE(fixture.geometry->passage_volumes, nullptr);
  ASSERT_NE(fixture.geometry->cooperative_passage_assignments, nullptr);
  ASSERT_NE(fixture.geometry->selected_passage_traversal_ids, nullptr);
  EXPECT_FALSE(fixture.geometry->route->empty());
  EXPECT_TRUE(fixture.geometry->constrained_spans->empty());
  EXPECT_TRUE(fixture.geometry->passage_volumes->empty());
  EXPECT_TRUE(fixture.geometry->cooperative_passage_assignments->empty());
  EXPECT_TRUE(fixture.geometry->selected_passage_traversal_ids->empty());
  EXPECT_EQ(fixture.geometry->executable_geometry_revision,
            executionRouteGeometryRevision3D(*fixture.geometry));
  EXPECT_TRUE(fixture.certify().has_value());
}

TEST(ExecutionRouteSnapshot3DTest, RejectsEachMissingGeometrySidecar) {
  SnapshotFixture3D fixture;
  const auto expect_missing_sidecar_rejected = [&](const std::string_view sidecar_name,
                                                   const auto& clear_sidecar) {
    SCOPED_TRACE(sidecar_name);
    auto geometry = std::make_shared<ExecutionRouteGeometry3D>(*fixture.geometry);
    clear_sidecar(*geometry);
    EXPECT_EQ(executionRouteGeometryRevision3D(*geometry), 0U);

    ExecutionRouteActivation3D activation = fixture.activation();
    activation.geometry = std::move(geometry);
    EXPECT_FALSE(certifyExecutionRoute3D(activation).has_value());
  };

  expect_missing_sidecar_rejected("mppi_route", [](ExecutionRouteGeometry3D& geometry) {
    geometry.mppi_route.reset();
  });
  expect_missing_sidecar_rejected(
      "route", [](ExecutionRouteGeometry3D& geometry) { geometry.route.reset(); });
  expect_missing_sidecar_rejected(
      "route_2d_projection",
      [](ExecutionRouteGeometry3D& geometry) { geometry.route_2d_projection.reset(); });
  expect_missing_sidecar_rejected(
      "constrained_spans",
      [](ExecutionRouteGeometry3D& geometry) { geometry.constrained_spans.reset(); });
  expect_missing_sidecar_rejected(
      "passage_volumes",
      [](ExecutionRouteGeometry3D& geometry) { geometry.passage_volumes.reset(); });
  expect_missing_sidecar_rejected("cooperative_passage_assignments",
                                  [](ExecutionRouteGeometry3D& geometry) {
                                    geometry.cooperative_passage_assignments.reset();
                                  });
  expect_missing_sidecar_rejected("selected_passage_traversal_ids",
                                  [](ExecutionRouteGeometry3D& geometry) {
                                    geometry.selected_passage_traversal_ids.reset();
                                  });
}

TEST(ExecutionRouteSnapshot3DTest,
     RejectsWrongCandidateGenerationAfterCanonicalRevisionIsRecomputed) {
  SnapshotFixture3D fixture;
  auto geometry = std::make_shared<ExecutionRouteGeometry3D>(*makeConstrainedGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      SnapshotFixture3D::kRouteGeneration, fixture.raw_occupancy.occupiedSnapshot(),
      testPassageVolumeConfig()));
  geometry->executable_geometry_revision = executionRouteGeometryRevision3D(*geometry);
  ASSERT_NE(geometry->executable_geometry_revision, 0U);

  ExecutionRouteActivation3D activation = fixture.activation();
  ++activation.route_generation;
  activation.geometry = std::move(geometry);

  EXPECT_FALSE(certifyExecutionRoute3D(activation).has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     RejectsCrossSidecarInconsistenciesAfterCanonicalRevisionIsRecomputed) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteGeometry3D> valid_geometry =
      makeConstrainedGeometry(fixture.route, fixture.physical_route_fingerprint,
                              SnapshotFixture3D::kRouteGeneration,
                              fixture.raw_occupancy.occupiedSnapshot(),
                              testPassageVolumeConfig());
  ExecutionRouteActivation3D valid_activation = fixture.activation();
  valid_activation.geometry = valid_geometry;
  ASSERT_NE(valid_geometry->executable_geometry_revision, 0U);
  ASSERT_TRUE(certifyExecutionRoute3D(valid_activation).has_value());

  const auto expect_inconsistency_rejected =
      [&](const std::string_view inconsistency_name, const auto& mutate) {
        SCOPED_TRACE(inconsistency_name);
        auto geometry = std::make_shared<ExecutionRouteGeometry3D>(*valid_geometry);
        mutate(*geometry);
        geometry->executable_geometry_revision =
            executionRouteGeometryRevision3D(*geometry);
        ASSERT_NE(geometry->executable_geometry_revision, 0U);
        ASSERT_EQ(geometry->executable_geometry_revision,
                  executionRouteGeometryRevision3D(*geometry));

        ExecutionRouteActivation3D activation = fixture.activation();
        activation.geometry = std::move(geometry);
        EXPECT_FALSE(certifyExecutionRoute3D(activation).has_value());
      };

  expect_inconsistency_rejected(
      "span traversal ID absent from selected IDs",
      [](ExecutionRouteGeometry3D& geometry) {
        auto spans = std::make_shared<std::vector<ConstrainedRouteSpan>>(
            *geometry.constrained_spans);
        spans->front().passage_traversal_id = "mismatched_span";
        geometry.constrained_spans = std::move(spans);
      });
  expect_inconsistency_rejected("selected traversal ID differs from span",
                                [](ExecutionRouteGeometry3D& geometry) {
                                  auto selected_ids =
                                      std::make_shared<std::vector<PassageTraversalId>>(
                                          *geometry.selected_passage_traversal_ids);
                                  selected_ids->front() = "mismatched_selection";
                                  geometry.selected_passage_traversal_ids =
                                      std::move(selected_ids);
                                });
  expect_inconsistency_rejected("selected traversal IDs contain an extra stale entry",
                                [](ExecutionRouteGeometry3D& geometry) {
                                  auto selected_ids =
                                      std::make_shared<std::vector<PassageTraversalId>>(
                                          *geometry.selected_passage_traversal_ids);
                                  selected_ids->push_back("stale_traversal");
                                  geometry.selected_passage_traversal_ids =
                                      std::move(selected_ids);
                                });
  expect_inconsistency_rejected(
      "volume traversal ID differs from span", [](ExecutionRouteGeometry3D& geometry) {
        auto volumes =
            std::make_shared<std::vector<PassageVolume>>(*geometry.passage_volumes);
        volumes->front().passage_traversal_id = "mismatched_volume";
        geometry.passage_volumes = std::move(volumes);
      });
  expect_inconsistency_rejected("volume span index differs from position",
                                [](ExecutionRouteGeometry3D& geometry) {
                                  auto volumes =
                                      std::make_shared<std::vector<PassageVolume>>(
                                          *geometry.passage_volumes);
                                  volumes->front().span_index = 1U;
                                  geometry.passage_volumes = std::move(volumes);
                                });
  expect_inconsistency_rejected("volume station interval differs from span",
                                [](ExecutionRouteGeometry3D& geometry) {
                                  auto volumes =
                                      std::make_shared<std::vector<PassageVolume>>(
                                          *geometry.passage_volumes);
                                  volumes->front().begin_station_m = 1.0;
                                  geometry.passage_volumes = std::move(volumes);
                                });
  expect_inconsistency_rejected(
      "assignment traversal ID differs from span",
      [](ExecutionRouteGeometry3D& geometry) {
        auto assignments = std::make_shared<std::vector<CooperativePassageAssignment>>(
            *geometry.cooperative_passage_assignments);
        assignments->front().passage_traversal_id = "mismatched_assignment";
        geometry.cooperative_passage_assignments = std::move(assignments);
      });
  expect_inconsistency_rejected(
      "assignment span index differs from position",
      [](ExecutionRouteGeometry3D& geometry) {
        auto assignments = std::make_shared<std::vector<CooperativePassageAssignment>>(
            *geometry.cooperative_passage_assignments);
        assignments->front().span_index = 1U;
        geometry.cooperative_passage_assignments = std::move(assignments);
      });
}

TEST(ExecutionRouteSnapshot3DTest,
     RejectsForgedPassageBoundsAndCentersAfterPublicFlagsAndRevisionsAreRecomputed) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteGeometry3D> valid_geometry =
      makeConstrainedGeometry(fixture.route, fixture.physical_route_fingerprint,
                              SnapshotFixture3D::kRouteGeneration,
                              fixture.raw_occupancy.occupiedSnapshot(),
                              testPassageVolumeConfig());
  ExecutionRouteActivation3D valid_activation = fixture.activation();
  valid_activation.geometry = valid_geometry;
  ASSERT_TRUE(certifyExecutionRoute3D(valid_activation).has_value());

  auto forged_geometry = std::make_shared<ExecutionRouteGeometry3D>(*valid_geometry);
  auto forged_volumes =
      std::make_shared<std::vector<PassageVolume>>(*valid_geometry->passage_volumes);
  ASSERT_TRUE(forged_volumes->front().raw_validated);
  forged_volumes->front().maximum_lateral_offset_m += 0.5;
  forged_volumes->front().cross_sections.front().maximum_lateral_offset_m += 0.5;
  forged_geometry->passage_volumes = std::move(forged_volumes);
  auto forged_assignments = std::make_shared<std::vector<CooperativePassageAssignment>>(
      *valid_geometry->cooperative_passage_assignments);
  forged_assignments->front().maximum_lateral_offset_m += 0.5;
  forged_geometry->cooperative_passage_assignments = std::move(forged_assignments);
  forged_geometry->executable_geometry_revision =
      executionRouteGeometryRevision3D(*forged_geometry);
  ASSERT_NE(forged_geometry->executable_geometry_revision, 0U);
  const ActivatedRouteIdentity3D forged_identity{
      .generation = valid_activation.route_generation,
      .proposal = valid_activation.proposal,
  };
  ASSERT_TRUE(executionRouteGeometryValid3D(*forged_geometry, forged_identity));

  ExecutionRouteActivation3D forged_activation = fixture.activation();
  forged_activation.geometry = std::move(forged_geometry);
  EXPECT_FALSE(certifyExecutionRoute3D(forged_activation).has_value());

  auto shifted_geometry = std::make_shared<ExecutionRouteGeometry3D>(*valid_geometry);
  auto shifted_volumes =
      std::make_shared<std::vector<PassageVolume>>(*valid_geometry->passage_volumes);
  shifted_volumes->front().cross_sections.front().center.y += 0.25;
  shifted_geometry->passage_volumes = std::move(shifted_volumes);
  shifted_geometry->executable_geometry_revision =
      executionRouteGeometryRevision3D(*shifted_geometry);
  ASSERT_TRUE(executionRouteGeometryValid3D(*shifted_geometry, forged_identity));

  ExecutionRouteActivation3D shifted_activation = fixture.activation();
  shifted_activation.geometry = std::move(shifted_geometry);
  EXPECT_FALSE(certifyExecutionRoute3D(shifted_activation).has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     RejectsPassageGeometryDerivedFromDifferentObservedWorld) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteGeometry3D> geometry =
      makeConstrainedGeometry(fixture.route, fixture.physical_route_fingerprint,
                              SnapshotFixture3D::kRouteGeneration,
                              fixture.raw_occupancy.occupiedSnapshot(),
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
     CooperativeAssignmentBoundsRemainInTheTraversalFrameAfterFinalRederivation) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteGeometry3D> canonical_geometry =
      makeConstrainedGeometry(fixture.route, fixture.physical_route_fingerprint,
                              SnapshotFixture3D::kRouteGeneration,
                              fixture.raw_occupancy.occupiedSnapshot(),
                              testPassageVolumeConfig());
  auto transformed_geometry =
      std::make_shared<ExecutionRouteGeometry3D>(*canonical_geometry);
  auto transformed_assignments =
      std::make_shared<std::vector<CooperativePassageAssignment>>(
          *canonical_geometry->cooperative_passage_assignments);
  CooperativePassageAssignment& transformed = transformed_assignments->front();
  transformed.requested_lateral_offset_m = -0.75;
  transformed.applied_lateral_offset_m = -0.75;
  transformed.minimum_lateral_offset_m += transformed.applied_lateral_offset_m;
  transformed.maximum_lateral_offset_m += transformed.applied_lateral_offset_m;
  transformed.status = CooperativePassageRouteStatus::kApplied;
  transformed_geometry->cooperative_passage_assignments = transformed_assignments;
  transformed_geometry->executable_geometry_revision =
      executionRouteGeometryRevision3D(*transformed_geometry);

  ExecutionRouteActivation3D transformed_activation = fixture.activation();
  transformed_activation.geometry = transformed_geometry;
  EXPECT_TRUE(certifyExecutionRoute3D(transformed_activation).has_value());

  auto stale_frame_geometry =
      std::make_shared<ExecutionRouteGeometry3D>(*transformed_geometry);
  auto stale_frame_assignments =
      std::make_shared<std::vector<CooperativePassageAssignment>>(
          *transformed_assignments);
  stale_frame_assignments->front().minimum_lateral_offset_m -=
      transformed.applied_lateral_offset_m;
  stale_frame_assignments->front().maximum_lateral_offset_m -=
      transformed.applied_lateral_offset_m;
  stale_frame_geometry->cooperative_passage_assignments =
      std::move(stale_frame_assignments);
  stale_frame_geometry->executable_geometry_revision =
      executionRouteGeometryRevision3D(*stale_frame_geometry);

  ExecutionRouteActivation3D stale_frame_activation = fixture.activation();
  stale_frame_activation.geometry = std::move(stale_frame_geometry);
  EXPECT_FALSE(certifyExecutionRoute3D(stale_frame_activation).has_value());
}

TEST(ExecutionRouteSnapshot3DTest,
     RejectsPassageGeometryDerivedFromDifferentStaticWorld) {
  SnapshotFixture3D fixture;
  const OccupancyGrid3D clear_world{fixture.raw_occupancy.bounds(),
                                    fixture.validated_world.esdf_fingerprint};
  const std::shared_ptr<const ExecutionRouteGeometry3D> geometry =
      makeConstrainedGeometry(fixture.route, fixture.physical_route_fingerprint,
                              SnapshotFixture3D::kRouteGeneration, clear_world,
                              testPassageVolumeConfig());
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
  const std::shared_ptr<const ExecutionRouteGeometry3D> geometry =
      makeConstrainedGeometry(fixture.route, fixture.physical_route_fingerprint,
                              SnapshotFixture3D::kRouteGeneration,
                              fixture.raw_occupancy.occupiedSnapshot(),
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

  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
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
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  const ExecutionRouteTransitionResult3D revoked =
      revokeExecution3D(*initial, initial->version);
  ASSERT_TRUE(revoked.applied());

  const auto exercise_recovery =
      [&](const std::shared_ptr<const ExecutionRouteSnapshot3D>& base,
          const PendingExecutionBaseKind3D base_kind) {
        const auto pending = std::make_shared<const PendingCertifiedRoute3D>(
            pendingForSnapshot(*base, base_kind, *route, 1U));
        ASSERT_TRUE(pending->valid());
        ASSERT_TRUE(pendingCertifiedRouteEligible3D(*pending, *base));
        PendingCertifiedRouteMailbox3D mailbox;
        ASSERT_TRUE(mailbox.publish(pending));
        const std::shared_ptr<const PendingCertifiedRoute3D> expected =
            mailbox.snapshot();
        ASSERT_NE(expected, nullptr);

        // pending_activation=false is the post-resolve signal that current
        // recertification could not produce an executable activation.
        const PendingCertifiedRouteRecoveryResult3D recovery =
            recoverPendingCertifiedRouteLiveness3D(
                mailbox, expected, PendingCertifiedRouteRecoveryObservation3D{});

        EXPECT_TRUE(recovery.pending_acknowledged);
        EXPECT_TRUE(recovery.request_successor);
        EXPECT_EQ(mailbox.snapshot(), nullptr);
      };

  exercise_recovery(initial, PendingExecutionBaseKind3D::kEmpty);
  exercise_recovery(revoked.next, PendingExecutionBaseKind3D::kRevoked);
}

TEST(ExecutionRouteSnapshot3DTest,
     PendingRecoveryPreservesANewerPublicationAndSuppressesAStaleRequest) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_TRUE(route.has_value());
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  const auto first = std::make_shared<const PendingCertifiedRoute3D>(
      pendingForSnapshot(*initial, PendingExecutionBaseKind3D::kEmpty, *route, 1U));
  PendingCertifiedRoute3D newer_value = *first;
  newer_value.publication_sequence = 2U;
  const auto newer =
      std::make_shared<const PendingCertifiedRoute3D>(std::move(newer_value));
  PendingCertifiedRouteMailbox3D mailbox;
  ASSERT_TRUE(mailbox.publish(first));
  const std::shared_ptr<const PendingCertifiedRoute3D> captured = mailbox.snapshot();
  ASSERT_NE(captured, nullptr);
  ASSERT_TRUE(mailbox.publish(newer));

  const PendingCertifiedRouteRecoveryResult3D recovery =
      recoverPendingCertifiedRouteLiveness3D(
          mailbox, captured, PendingCertifiedRouteRecoveryObservation3D{});

  EXPECT_FALSE(recovery.pending_acknowledged);
  EXPECT_FALSE(recovery.request_successor);
  const std::shared_ptr<const PendingCertifiedRoute3D> resident = mailbox.snapshot();
  ASSERT_NE(resident, nullptr);
  EXPECT_EQ(resident->publication_sequence, 2U);
}

TEST(ExecutionRouteSnapshot3DTest,
     PendingRecoveryConfirmsTheSharedMailboxBeforeTreatingNullAsEmpty) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_TRUE(route.has_value());
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  const auto pending = std::make_shared<const PendingCertifiedRoute3D>(
      pendingForSnapshot(*initial, PendingExecutionBaseKind3D::kEmpty, *route, 1U));
  PendingCertifiedRouteMailbox3D mailbox;
  ASSERT_TRUE(mailbox.publish(pending));

  const PendingCertifiedRouteRecoveryResult3D occupied =
      recoverPendingCertifiedRouteLiveness3D(
          mailbox, nullptr, PendingCertifiedRouteRecoveryObservation3D{});
  EXPECT_FALSE(occupied.pending_acknowledged);
  EXPECT_FALSE(occupied.request_successor);
  const std::shared_ptr<const PendingCertifiedRoute3D> resident = mailbox.snapshot();
  ASSERT_NE(resident, nullptr);
  EXPECT_EQ(resident->publication_sequence, pending->publication_sequence);
  EXPECT_EQ(resident->route.identity.generation, pending->route.identity.generation);
  ASSERT_NE(resident->route.geometry, nullptr);
  ASSERT_NE(pending->route.geometry, nullptr);
  EXPECT_EQ(resident->route.geometry->executable_geometry_revision,
            pending->route.geometry->executable_geometry_revision);

  ASSERT_TRUE(mailbox.acknowledgeIfSame(resident));
  const PendingCertifiedRouteRecoveryResult3D empty =
      recoverPendingCertifiedRouteLiveness3D(
          mailbox, nullptr, PendingCertifiedRouteRecoveryObservation3D{});
  EXPECT_FALSE(empty.pending_acknowledged);
  EXPECT_TRUE(empty.request_successor);
}

TEST(ExecutionRouteSnapshot3DTest,
     DirectTrackingSuppressesPendingRecoveryAndSuccessorRequest) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_TRUE(route.has_value());
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  const auto pending = std::make_shared<const PendingCertifiedRoute3D>(
      pendingForSnapshot(*initial, PendingExecutionBaseKind3D::kEmpty, *route, 1U));
  PendingCertifiedRouteMailbox3D mailbox;
  ASSERT_TRUE(mailbox.publish(pending));

  const PendingCertifiedRouteRecoveryResult3D recovery =
      recoverPendingCertifiedRouteLiveness3D(mailbox, nullptr,
                                             PendingCertifiedRouteRecoveryObservation3D{
                                                 .direct_tracking_requested = true,
                                             });

  EXPECT_FALSE(recovery.pending_acknowledged);
  EXPECT_FALSE(recovery.request_successor);
  EXPECT_NE(mailbox.snapshot(), nullptr);
}

TEST(ExecutionRouteSnapshot3DTest,
     FiniteAndStationaryHoldOwnersSuppressPendingRecovery) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_TRUE(active->finite_execution.has_value());
  const ExecutionRouteTransitionResult3D held = transferToExecutionHold3D(
      *active, active->version, SnapshotFixture3D::holdCertification(*active));
  ASSERT_TRUE(held.applied());
  ASSERT_NE(held.next, nullptr);
  ASSERT_TRUE(held.next->stationary_hold.has_value());

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = active->routeGenerationHighWater() + 1U;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const auto exercise_owner =
      [&](const std::shared_ptr<const ExecutionRouteSnapshot3D>& owner,
          const PendingExecutionBaseKind3D base_kind) {
        const auto pending = std::make_shared<const PendingCertifiedRoute3D>(
            pendingForSnapshot(*owner, base_kind, *successor, 1U));
        ASSERT_TRUE(pending->valid());
        ASSERT_TRUE(pendingCertifiedRouteEligible3D(*pending, *owner));
        PendingCertifiedRouteMailbox3D mailbox;
        ASSERT_TRUE(mailbox.publish(pending));
        const std::shared_ptr<const PendingCertifiedRoute3D> expected =
            mailbox.snapshot();
        ASSERT_NE(expected, nullptr);

        const bool execution_owner_available =
            owner->finite_execution.has_value() ||
            owner->direct_tracking_execution.has_value() ||
            owner->stationary_hold.has_value();
        const PendingCertifiedRouteRecoveryResult3D recovery =
            recoverPendingCertifiedRouteLiveness3D(
                mailbox, expected,
                PendingCertifiedRouteRecoveryObservation3D{
                    .execution_owner_available = execution_owner_available,
                });

        EXPECT_FALSE(recovery.pending_acknowledged);
        EXPECT_FALSE(recovery.request_successor);
        EXPECT_EQ(mailbox.snapshot(), expected);
      };

  exercise_owner(active, PendingExecutionBaseKind3D::kRoute);
  exercise_owner(held.next, PendingExecutionBaseKind3D::kStationaryHold);
}

} // namespace
} // namespace drone_city_nav
