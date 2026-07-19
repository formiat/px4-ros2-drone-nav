#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(AmbiguousLidarHitTrackerTest, ConfirmsOnlyIndependentScans) {
  AmbiguousLidarHitTracker tracker;
  const GridIndex cell{4, 7};

  EXPECT_FALSE(tracker.observe(cell, 100'000'000).confirmed);
  EXPECT_FALSE(tracker.observe(cell, 100'000'000).new_scan_vote);
  EXPECT_FALSE(tracker.observe(cell, 200'000'000).confirmed);
  const AmbiguousLidarHitConfirmation confirmation = tracker.observe(cell, 300'000'000);

  EXPECT_TRUE(confirmation.confirmed);
  EXPECT_EQ(confirmation.independent_scans, 3U);
}

TEST(AmbiguousLidarHitTrackerTest, GapRestartsEvidence) {
  AmbiguousLidarHitTracker tracker{
      AmbiguousLidarHitTrackerConfig{3U, 100'000'000, 500'000'000}};
  const GridIndex cell{1, 2};
  EXPECT_FALSE(tracker.observe(cell, 100'000'000).confirmed);
  EXPECT_FALSE(tracker.observe(cell, 150'000'000).confirmed);

  const AmbiguousLidarHitConfirmation restarted = tracker.observe(cell, 400'000'000);

  EXPECT_EQ(restarted.independent_scans, 1U);
  EXPECT_FALSE(restarted.confirmed);
}

} // namespace
} // namespace drone_city_nav
