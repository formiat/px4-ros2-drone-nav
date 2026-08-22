#include "drone_city_nav/incremental_topology_tile_scheduler_3d.hpp"

#include <gtest/gtest.h>

#include <array>

namespace drone_city_nav {
namespace {

TEST(IncrementalTopologyTileScheduler3DTest,
     PrioritizesLocalCoverageBeforeTheDistantGoalCorridor) {
  const std::array pending{
      IncrementalTopologyTileIndex3D{2, 3, 2},
      IncrementalTopologyTileIndex3D{0, 2, 2},
      IncrementalTopologyTileIndex3D{10, 2, 2},
      IncrementalTopologyTileIndex3D{2, 2, 2},
  };
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 40, 40, 40};
  const IncrementalTopologyBuildPriority3D priority{
      .position = {10.0, 10.0, 10.0},
      .target = {50.0, 10.0, 10.0},
  };

  const std::vector selected =
      selectIncrementalTopologyTiles3D(pending, 2U, bounds, 4, priority);

  ASSERT_EQ(selected.size(), 2U);
  EXPECT_EQ(selected[0], (IncrementalTopologyTileIndex3D{2, 2, 2}));
  EXPECT_EQ(selected[1], (IncrementalTopologyTileIndex3D{2, 3, 2}));
}

TEST(IncrementalTopologyTileScheduler3DTest,
     UsesDeterministicSpatialOrderWithoutMissionPriority) {
  const std::array pending{
      IncrementalTopologyTileIndex3D{1, 0, 1},
      IncrementalTopologyTileIndex3D{2, 0, 0},
      IncrementalTopologyTileIndex3D{0, 1, 0},
  };
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 16, 16, 16};

  const std::vector selected = selectIncrementalTopologyTiles3D(
      pending, pending.size(), bounds, 4, std::nullopt);

  EXPECT_EQ(selected[0], (IncrementalTopologyTileIndex3D{2, 0, 0}));
  EXPECT_EQ(selected[1], (IncrementalTopologyTileIndex3D{0, 1, 0}));
  EXPECT_EQ(selected[2], (IncrementalTopologyTileIndex3D{1, 0, 1}));
}

} // namespace
} // namespace drone_city_nav
