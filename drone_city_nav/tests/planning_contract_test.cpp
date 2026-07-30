#include "drone_city_nav/collision_geometry.hpp"
#include "drone_city_nav/mppi_nominal_reseed.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {
namespace {

TEST(CollisionGeometryTest, CoversOutermostRotorEdge) {
  const double radius = effectiveCollisionRadiusM(VehicleFootprintGeometry{});

  EXPECT_NEAR(radius, std::hypot(0.48, 0.48) + 0.14, 1.0e-12);
  EXPECT_GT(radius, 0.81);
  EXPECT_LT(radius, 0.83);
}

TEST(MppiNominalReseedTrackerTest, ReseedsOnGuideGenerationChange) {
  MppiNominalReseedTracker tracker;

  const MppiNominalReseedUpdate first =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U});
  const MppiNominalReseedUpdate unchanged =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U});
  const MppiNominalReseedUpdate changed =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 2U});

  EXPECT_TRUE(first.requested);
  EXPECT_FALSE(unchanged.requested);
  EXPECT_TRUE(changed.requested);
  EXPECT_EQ(changed.generation, first.generation + 1U);
}

TEST(MppiNominalReseedTrackerTest, ReseedsAfterNoEligibleRollout) {
  MppiNominalReseedTracker tracker;
  static_cast<void>(
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U}));
  tracker.observeEligibleRolloutResult(false);

  const MppiNominalReseedUpdate update =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U});

  EXPECT_TRUE(update.requested);
}

TEST(MppiNominalReseedTrackerTest, ReseedsOnPersistentSafetyRejection) {
  MppiNominalReseedTracker tracker;
  static_cast<void>(
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U}));

  const MppiNominalReseedUpdate update = tracker.update(MppiNominalReseedObservation{
      .guide_generation = 1U,
      .safety_rejection_generation = 3U,
  });

  EXPECT_TRUE(update.requested);
}

} // namespace
} // namespace drone_city_nav
