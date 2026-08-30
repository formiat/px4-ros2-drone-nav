#include "drone_city_nav/compiled_trajectory_3d.hpp"

#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(ExecutionRouteSnapshot3DTest,
     ChangedContentAdvanceTransfersOnlyTheRouteWorldOwner) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(active->finiteExecution() != nullptr);
  const std::uint64_t old_content =
      active->route()->observed_raw_world->contentFingerprint();
  const auto old_finite_owner = active->finiteExecution()->observed_raw_world;

  ObservedOccupancyGrid3D changed_occupancy = fixture.raw_occupancy;
  ASSERT_TRUE(changed_occupancy.setState({0, 0, 0}, ObservedVoxelState::kOccupied));
  const auto changed_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &changed_occupancy);
  ASSERT_TRUE(changed_world);
  ASSERT_NE(changed_world->contentFingerprint(), old_content);
  RouteExecutionObservation3D poisoned_borrowed_fields =
      fixture.executionObservation({4.0, 0.0, 5.0}, 1U, nullptr);
  poisoned_borrowed_fields.latest_raw_producer_instance_id = 1U;

  const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active), poisoned_borrowed_fields,
      SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0}), changed_world);

  ASSERT_TRUE(advanced.applied());
  ASSERT_TRUE(advanced.next);
  ASSERT_TRUE(advanced.next->route() != nullptr);
  ASSERT_TRUE(advanced.next->finiteExecution() != nullptr);
  EXPECT_EQ(advanced.next->route()->observed_raw_world, changed_world);
  EXPECT_EQ(advanced.next->route()->observed_raw_world->contentFingerprint(),
            changed_world->contentFingerprint());
  EXPECT_EQ(advanced.next->finiteExecution()->observed_raw_world, old_finite_owner);
  EXPECT_EQ(advanced.next->finiteExecution()->observed_raw_world->contentFingerprint(),
            old_content);
  EXPECT_TRUE(advanced.next->finiteExecution()->revalidation_required);
  EXPECT_TRUE(advanced.next->valid());

  ASSERT_TRUE(changed_occupancy.setState({1, 0, 0}, ObservedVoxelState::kOccupied));
  EXPECT_EQ(advanced.next->route()->observed_raw_world->contentFingerprint(),
            changed_world->contentFingerprint());
  EXPECT_EQ(advanced.next->route()->observed_raw_world->occupancy().state({1, 0, 0}),
            ObservedVoxelState::kUnknown);
}

TEST(ExecutionRouteSnapshot3DTest,
     ChangedOccupancyRejectsOwnerTransferWhenItShrinksTheTrackingTube) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_NE(active->route()->geometry, nullptr);
  ASSERT_NE(active->route()->geometry->tracking_error_tube, nullptr);

  ObservedOccupancyGrid3D changed_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> tube_only_obstacle =
      changed_occupancy.worldToCell(Point3{6.5, 1.5, 5.5});
  ASSERT_TRUE(tube_only_obstacle.has_value());
  ASSERT_TRUE(
      changed_occupancy.setState(*tube_only_obstacle, ObservedVoxelState::kOccupied));
  EXPECT_TRUE(
      validateRawSweptFootprint(changed_occupancy, fixture.route.front().position,
                                FootprintBodyAxis{}, fixture.route.back().position,
                                FootprintBodyAxis{}, SweptFootprintConfig{})
          .accepted());
  const auto changed_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &changed_occupancy);
  ASSERT_TRUE(changed_world);
  EXPECT_FALSE(trackingErrorTubeProfile3DMatchesWorld(
      *active->route()->geometry->route,
      *active->route()->geometry->tracking_error_tube,
      TrackingErrorTubeWorld3D{
          .observed_occupancy = &changed_world->occupancy(),
          .occupied_content_fingerprint = changed_world->occupiedContentFingerprint(),
      }));

  const ExecutionRouteTransitionResult3D rejected = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active),
      fixture.executionObservation({4.0, 0.0, 5.0},
                                   SnapshotFixture3D::kLatestRawRevision + 1U,
                                   &changed_occupancy),
      SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0}), changed_world);

  EXPECT_EQ(rejected.status,
            ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected);
  EXPECT_FALSE(rejected.next);
}

TEST(ExecutionRouteSnapshot3DTest,
     AdvanceRejectsDifferentContentAdvertisedAtTheCertifiedRevision) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ObservedOccupancyGrid3D changed_occupancy = fixture.raw_occupancy;
  ASSERT_TRUE(changed_occupancy.setState({0, 0, 0}, ObservedVoxelState::kOccupied));

  const ExecutionRouteTransitionResult3D rejected = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active),
      fixture.executionObservation({4.0, 0.0, 5.0},
                                   SnapshotFixture3D::kLatestRawRevision,
                                   &fixture.raw_occupancy),
      SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0}),
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision, &changed_occupancy));

  EXPECT_EQ(rejected.status, ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  EXPECT_FALSE(rejected.next);
}

TEST(ExecutionRouteSnapshot3DTest,
     ConstrainedAdvanceRejectsANewerWorldWithDifferentPassageGeometry) {
  SnapshotFixture3D fixture;
  fixture.geometry = makeConstrainedGeometry(
      fixture.route, fixture.physical_route_fingerprint,
      SnapshotFixture3D::kRouteGeneration, fixture.raw_occupancy.occupiedSnapshot(),
      testPassageVolumeConfig());
  fixture.geometry_revision = fixture.geometry->compiled_trajectory_revision;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_FALSE(active->route()->geometry->constrained_spans->empty());

  ObservedOccupancyGrid3D changed_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> lateral_wall =
      changed_occupancy.worldToCell(Point3{5.0, 2.0, 5.0});
  ASSERT_TRUE(lateral_wall.has_value());
  ASSERT_TRUE(changed_occupancy.setState(*lateral_wall, ObservedVoxelState::kOccupied));
  const auto changed_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &changed_occupancy);
  ASSERT_TRUE(changed_world);
  ASSERT_NE(changed_world->occupiedContentFingerprint(),
            active->route()->observed_raw_world->occupiedContentFingerprint());

  const ExecutionRouteTransitionResult3D rejected = advanceCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active),
      fixture.executionObservation({4.0, 0.0, 5.0},
                                   SnapshotFixture3D::kLatestRawRevision + 1U,
                                   &changed_occupancy),
      SnapshotFixture3D::progressInput(*active, {4.0, 0.0, 5.0}), changed_world);

  EXPECT_EQ(rejected.status, ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  EXPECT_FALSE(rejected.next);
}

} // namespace
} // namespace drone_city_nav
