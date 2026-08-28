#include "drone_city_nav/mppi_nominal_reseed.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(MppiNominalReseedTrackerTest, ReseedsOnRouteGenerationChange) {
  MppiNominalReseedTracker tracker;

  const MppiNominalReseedUpdate first =
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U});
  const MppiNominalReseedUpdate unchanged =
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U});
  const MppiNominalReseedUpdate changed =
      tracker.update(MppiNominalReseedObservation{.route_generation = 2U});

  EXPECT_TRUE(first.requested);
  EXPECT_FALSE(unchanged.requested);
  EXPECT_TRUE(changed.requested);
  EXPECT_EQ(changed.generation, first.generation + 1U);
}

TEST(MppiNominalReseedTrackerTest, ReseedsAfterNoEligibleRollout) {
  MppiNominalReseedTracker tracker;
  static_cast<void>(
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U}));
  static_cast<void>(tracker.observeEligibleRolloutResult(false, false));

  const MppiNominalReseedUpdate update =
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U});

  EXPECT_TRUE(update.requested);
}

TEST(MppiNominalReseedTrackerTest,
     InvalidatesFailedNominalOnceThenWaitsForReplacementRoute) {
  MppiNominalReseedTracker tracker;
  static_cast<void>(
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U}));
  static_cast<void>(tracker.observeEligibleRolloutResult(false, false));
  const MppiNominalReseedUpdate first =
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U});
  const MppiEligibleRolloutUpdate failed_reseed =
      tracker.observeEligibleRolloutResult(false, true);
  const MppiNominalReseedUpdate recovery_seed =
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U});
  static_cast<void>(tracker.observeEligibleRolloutResult(false, true));
  const MppiNominalReseedUpdate repeated =
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U});
  static_cast<void>(tracker.observeEligibleRolloutResult(true, false));
  static_cast<void>(tracker.observeEligibleRolloutResult(false, false));
  const MppiNominalReseedUpdate next_episode =
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U});

  EXPECT_TRUE(first.requested);
  EXPECT_TRUE(failed_reseed.route_replan_requested);
  EXPECT_EQ(failed_reseed.phase, MppiNoEligiblePhase::kReseedPending);
  EXPECT_TRUE(recovery_seed.requested);
  EXPECT_EQ(recovery_seed.no_eligible_phase, MppiNoEligiblePhase::kAwaitingRouteChange);
  EXPECT_FALSE(repeated.requested);
  EXPECT_TRUE(next_episode.requested);
  EXPECT_EQ(next_episode.generation, first.generation + 2U);
}

TEST(MppiNominalReseedTrackerTest, ReseedsAgainstReplacementRoute) {
  MppiNominalReseedTracker tracker;
  static_cast<void>(
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U}));
  static_cast<void>(tracker.observeEligibleRolloutResult(false, false));
  static_cast<void>(
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U}));
  const MppiEligibleRolloutUpdate failed =
      tracker.observeEligibleRolloutResult(false, true);

  const MppiNominalReseedUpdate replacement =
      tracker.update(MppiNominalReseedObservation{.route_generation = 2U});

  EXPECT_TRUE(failed.route_replan_requested);
  EXPECT_TRUE(replacement.requested);
  EXPECT_EQ(replacement.no_eligible_phase, MppiNoEligiblePhase::kAwaitingReseedResult);
}

TEST(MppiNominalReseedTrackerTest, ReseedsOnDirectTrackingManeuver) {
  MppiNominalReseedTracker tracker;
  static_cast<void>(
      tracker.update(MppiNominalReseedObservation{.route_generation = 1U}));

  const MppiNominalReseedUpdate update = tracker.update(MppiNominalReseedObservation{
      .route_generation = 1U,
      .direct_tracking_maneuver_generation = 1U,
  });
  const MppiNominalReseedUpdate repeated = tracker.update(MppiNominalReseedObservation{
      .route_generation = 1U,
      .direct_tracking_maneuver_generation = 1U,
  });

  EXPECT_TRUE(update.requested);
  EXPECT_FALSE(repeated.requested);
}

} // namespace
} // namespace drone_city_nav
