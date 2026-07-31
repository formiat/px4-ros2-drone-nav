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
  static_cast<void>(tracker.observeEligibleRolloutResult(false, false));

  const MppiNominalReseedUpdate update =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U});

  EXPECT_TRUE(update.requested);
}

TEST(MppiNominalReseedTrackerTest,
     InvalidatesFailedNominalOnceThenWaitsForReplacementGuide) {
  MppiNominalReseedTracker tracker;
  static_cast<void>(
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U}));
  static_cast<void>(tracker.observeEligibleRolloutResult(false, false));
  const MppiNominalReseedUpdate first =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U});
  const MppiEligibleRolloutUpdate failed_reseed =
      tracker.observeEligibleRolloutResult(false, true);
  const MppiNominalReseedUpdate recovery_seed =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U});
  static_cast<void>(tracker.observeEligibleRolloutResult(false, true));
  const MppiNominalReseedUpdate repeated =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U});
  static_cast<void>(tracker.observeEligibleRolloutResult(true, false));
  static_cast<void>(tracker.observeEligibleRolloutResult(false, false));
  const MppiNominalReseedUpdate next_episode =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U});

  EXPECT_TRUE(first.requested);
  EXPECT_TRUE(failed_reseed.guide_replan_requested);
  EXPECT_EQ(failed_reseed.phase, MppiNoEligiblePhase::kReseedPending);
  EXPECT_TRUE(recovery_seed.requested);
  EXPECT_EQ(recovery_seed.no_eligible_phase, MppiNoEligiblePhase::kAwaitingGuideChange);
  EXPECT_FALSE(repeated.requested);
  EXPECT_TRUE(next_episode.requested);
  EXPECT_EQ(next_episode.generation, first.generation + 2U);
}

TEST(MppiNominalReseedTrackerTest, ReseedsAgainstReplacementGuide) {
  MppiNominalReseedTracker tracker;
  static_cast<void>(
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U}));
  static_cast<void>(tracker.observeEligibleRolloutResult(false, false));
  static_cast<void>(
      tracker.update(MppiNominalReseedObservation{.guide_generation = 1U}));
  const MppiEligibleRolloutUpdate failed =
      tracker.observeEligibleRolloutResult(false, true);

  const MppiNominalReseedUpdate replacement =
      tracker.update(MppiNominalReseedObservation{.guide_generation = 2U});

  EXPECT_TRUE(failed.guide_replan_requested);
  EXPECT_TRUE(replacement.requested);
  EXPECT_EQ(replacement.no_eligible_phase, MppiNoEligiblePhase::kAwaitingReseedResult);
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
