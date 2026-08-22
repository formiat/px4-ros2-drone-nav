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

} // namespace
} // namespace drone_city_nav
