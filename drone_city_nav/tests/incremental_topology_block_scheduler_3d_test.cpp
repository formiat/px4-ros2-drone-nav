#include "drone_city_nav/incremental_topology_block_scheduler_3d.hpp"

#include <gtest/gtest.h>

#include <array>

namespace drone_city_nav {
namespace {

TEST(IncrementalTopologyBlockScheduler3DTest,
     PrioritizesLocalCoverageBeforeTheDistantGoalCorridor) {
  const std::array pending{
      IncrementalTopologyBlockIndex3D{2, 3, 2},
      IncrementalTopologyBlockIndex3D{0, 2, 2},
      IncrementalTopologyBlockIndex3D{10, 2, 2},
      IncrementalTopologyBlockIndex3D{2, 2, 2},
  };
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 40, 40, 40};
  const IncrementalTopologyBuildPriority3D priority{
      .position = {10.0, 10.0, 10.0},
      .target = {50.0, 10.0, 10.0},
  };

  const std::vector selected =
      selectIncrementalTopologyBlocks3D(pending, 2U, bounds, 4, priority);

  ASSERT_EQ(selected.size(), 2U);
  EXPECT_EQ(selected[0], (IncrementalTopologyBlockIndex3D{2, 2, 2}));
  EXPECT_EQ(selected[1], (IncrementalTopologyBlockIndex3D{2, 3, 2}));
}

TEST(IncrementalTopologyBlockScheduler3DTest,
     UsesDeterministicSpatialOrderWithoutMissionPriority) {
  const std::array pending{
      IncrementalTopologyBlockIndex3D{1, 0, 1},
      IncrementalTopologyBlockIndex3D{2, 0, 0},
      IncrementalTopologyBlockIndex3D{0, 1, 0},
  };
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 16, 16, 16};

  const std::vector selected = selectIncrementalTopologyBlocks3D(
      pending, pending.size(), bounds, 4, std::nullopt);

  EXPECT_EQ(selected[0], (IncrementalTopologyBlockIndex3D{2, 0, 0}));
  EXPECT_EQ(selected[1], (IncrementalTopologyBlockIndex3D{0, 1, 0}));
  EXPECT_EQ(selected[2], (IncrementalTopologyBlockIndex3D{1, 0, 1}));
}

TEST(IncrementalTopologyBlockScheduler3DTest,
     SelectsTheForwardCorridorBeforeCloserLateralOrRearBacklog) {
  const std::array pending{
      IncrementalTopologyBlockIndex3D{0, 2, 2},
      IncrementalTopologyBlockIndex3D{4, 5, 2},
      IncrementalTopologyBlockIndex3D{6, 2, 2},
      IncrementalTopologyBlockIndex3D{2, 2, 2},
  };
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 40, 40, 40};
  const IncrementalTopologyBuildPriority3D priority{
      .position = {10.0, 10.0, 10.0},
      .target = {50.0, 10.0, 10.0},
      .local_radius_m = 5.0,
      .forward_corridor_radius_m = 6.0,
      .forward_corridor_lookahead_m = 30.0,
  };

  const std::vector selected =
      selectIncrementalTopologyBlocks3D(pending, 3U, bounds, 4, priority);

  ASSERT_EQ(selected.size(), 3U);
  EXPECT_EQ(selected[0], (IncrementalTopologyBlockIndex3D{2, 2, 2}));
  EXPECT_EQ(selected[1], (IncrementalTopologyBlockIndex3D{6, 2, 2}));
  EXPECT_EQ(incrementalTopologyBlockPriorityTier3D(selected[1], bounds, 4, priority),
            IncrementalTopologyBlockPriorityTier3D::kForwardCorridor);
  EXPECT_EQ(selected[2], (IncrementalTopologyBlockIndex3D{0, 2, 2}));
}

TEST(IncrementalTopologyBlockScheduler3DTest,
     ZeroProgressWatchdogBacksOffExponentiallyAndResetsOnProgress) {
  IncrementalTopologyProgressWatchdog3D watchdog{
      IncrementalTopologyProgressWatchdog3DConfig{
          .initial_backoff = std::chrono::milliseconds{5},
          .maximum_backoff = std::chrono::milliseconds{20},
          .warning_streak = 3U,
      }};

  const auto first = watchdog.observe(8U, 0U);
  EXPECT_EQ(first.zero_progress_streak, 1U);
  EXPECT_EQ(first.retry_backoff, std::chrono::milliseconds{5});
  EXPECT_FALSE(first.stalled);
  const auto second = watchdog.observe(8U, 0U);
  EXPECT_EQ(second.retry_backoff, std::chrono::milliseconds{10});
  const auto third = watchdog.observe(8U, 0U);
  EXPECT_EQ(third.retry_backoff, std::chrono::milliseconds{20});
  EXPECT_TRUE(third.stalled);
  EXPECT_EQ(watchdog.observe(8U, 0U).retry_backoff, std::chrono::milliseconds{20});

  EXPECT_EQ(watchdog.observe(7U, 1U).zero_progress_streak, 0U);
  EXPECT_EQ(watchdog.observe(0U, 0U).retry_backoff, std::chrono::milliseconds{0});
}

} // namespace
} // namespace drone_city_nav
