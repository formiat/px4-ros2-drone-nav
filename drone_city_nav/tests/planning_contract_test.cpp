#include "drone_city_nav/mppi_nominal_reseed.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

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

TEST(MppiNominalReseedTrackerTest, ReseedsOnlyOncePerNoEligibleEpisode) {
  MppiNominalReseedTracker tracker;
  static_cast<void>(
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U}));
  tracker.observeEligibleRolloutResult(false);
  const MppiNominalReseedUpdate first =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U});
  tracker.observeEligibleRolloutResult(false);
  const MppiNominalReseedUpdate repeated =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U});
  tracker.observeEligibleRolloutResult(true);
  tracker.observeEligibleRolloutResult(false);
  const MppiNominalReseedUpdate next_episode =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U});

  EXPECT_TRUE(first.requested);
  EXPECT_FALSE(repeated.requested);
  EXPECT_TRUE(next_episode.requested);
  EXPECT_EQ(next_episode.generation, first.generation + 1U);
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
