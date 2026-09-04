#include "drone_city_nav/execution_plan_3d.hpp"
#include "drone_city_nav/launch_support_contact_3d.hpp"
#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"
#include "drone_city_nav/versioned_world_evidence_3d.hpp"

#include <gtest/gtest.h>

#include <optional>

namespace drone_city_nav {
namespace {

[[nodiscard]] SweptFootprintConfig testFootprint() noexcept {
  return SweptFootprintConfig{.radius_m = 0.5,
                              .lower_extent_m = 0.25,
                              .upper_extent_m = 0.25,
                              .sweep_step_m = 0.125};
}

[[nodiscard]] ProprioceptiveFreeSpaceSeed3D seedAt(const Point3& position) noexcept {
  return ProprioceptiveFreeSpaceSeed3D{
      .position = position,
      .body_axis = FootprintBodyAxis{},
      .footprint = testFootprint(),
      .contact_tolerance_m = 0.125,
  };
}

TEST(LaunchSupportContact3D, AnchorRequiresContactReportedNowAndAVehicleAtRest) {
  const double tolerance = kStationaryExecutionHoldSpeedToleranceMps;
  EXPECT_TRUE(launchSupportAnchorAdmissible3D(true, 0.0, tolerance));
  EXPECT_TRUE(launchSupportAnchorAdmissible3D(true, tolerance, tolerance));
  // A detector latch that fired before the navigation stack could anchor it
  // says nothing about the pose the first evidence arrives at.
  EXPECT_FALSE(launchSupportAnchorAdmissible3D(false, 0.0, tolerance));
  EXPECT_FALSE(launchSupportAnchorAdmissible3D(true, tolerance + 0.01, tolerance));
  EXPECT_FALSE(launchSupportAnchorAdmissible3D(
      true, std::numeric_limits<double>::quiet_NaN(), tolerance));
}

TEST(LaunchSupportContact3D, ReleaseFollowsAnyDepartureNotOnlyAClimb) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.25, 40, 40, 40};
  const Point3 anchor{5.0, 5.0, 5.0};
  const LaunchSupportContact3D contact =
      makeVehicleLandedSupportContact3D(bounds, seedAt(anchor));
  ASSERT_TRUE(launchSupportContactValid3D(contact));

  // Still on the support: nothing to release.
  EXPECT_FALSE(launchSupportReleased3D(contact, anchor, true, bounds.resolution_m));
  // A body still touching occupied evidence keeps the support whatever it does.
  EXPECT_FALSE(launchSupportReleased3D(contact, Point3{25.0, 5.0, 5.0}, false,
                                       bounds.resolution_m));
  // Climbing off it releases, as before.
  EXPECT_TRUE(launchSupportReleased3D(contact, Point3{5.0, 5.0, 5.4}, true,
                                      bounds.resolution_m));
  // Leaving sideways at the same altitude releases too: a vehicle that flew
  // away owes the support nothing, and a support it never releases would
  // exempt evidence that is not contact for the rest of the flight.
  EXPECT_TRUE(launchSupportReleased3D(contact, Point3{25.0, 5.0, 5.0}, true,
                                      bounds.resolution_m));
  EXPECT_TRUE(launchSupportReleased3D(contact, Point3{5.0, 5.0, 4.6}, true,
                                      bounds.resolution_m));
}

TEST(LaunchSupportContact3D, RouteEvidenceDerivesWithTheVehiclePoseOfThisMoment) {
  // The support anchors at the launch pose while the proprioceptive seed is
  // the vehicle's pose now. Requiring the two to match once made every world
  // derivation fail as soon as the vehicle moved.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 0.25, 40, 40, 40});
  const Point3 anchor{5.0, 5.0, 5.0};
  const LaunchSupportContact3D contact =
      makeVehicleLandedSupportContact3D(occupancy->bounds(), seedAt(anchor));
  const RawMapVersion version{
      .producer_instance_id = 17U,
      .base_snapshot_revision = 1U,
      .revision = 4U,
  };
  const std::shared_ptr<const VersionedObservedRawWorld3D> world =
      VersionedObservedRawWorld3D::captureOwned(version, occupancy, seedAt(anchor),
                                                contact);
  ASSERT_NE(world, nullptr);

  const std::shared_ptr<const VersionedObservedRawWorld3D> moved =
      world->deriveRouteEvidence(seedAt(Point3{25.0, 5.0, 5.0}), contact);
  ASSERT_NE(moved, nullptr);
  EXPECT_TRUE(moved->valid());
  ASSERT_TRUE(moved->launchSupportContact().has_value());
  EXPECT_EQ(moved->launchSupportContact()->seed.position.x, anchor.x);
  ASSERT_TRUE(moved->proprioceptiveFreeSpaceSeed().has_value());
  EXPECT_EQ(moved->proprioceptiveFreeSpaceSeed()->position.x, 25.0);
}

TEST(ProprioceptiveContactSeed3D, ReportsTheVoxelQuantizationOfTheWorldItJudges) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 0.25, 40, 40, 40});
  const std::optional<ProprioceptiveFreeSpaceSeed3D> seed = proprioceptiveContactSeed3D(
      Point3{5.6, 5.0, 5.0}, FootprintBodyAxis{}, testFootprint(), occupancy.get());
  ASSERT_TRUE(seed.has_value());
  EXPECT_NEAR(seed->contact_tolerance_m, 0.125, 1.0e-12);
  EXPECT_EQ(seed->position.x, 5.6);

  EXPECT_FALSE(proprioceptiveContactSeed3D(Point3{5.6, 5.0, 5.0}, FootprintBodyAxis{},
                                           testFootprint(), nullptr)
                   .has_value());
}

} // namespace
} // namespace drone_city_nav
