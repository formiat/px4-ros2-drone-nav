#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(OccupancyGrid3D, StoresSparseVoxelsAcrossChunks) {
  OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 0.5, 40, 40, 40}, 42U};
  grid.setOccupied({0, 0, 0});
  grid.setOccupied({17, 18, 19});
  grid.setOccupied({17, 18, 19});

  EXPECT_TRUE(grid.isOccupied({0, 0, 0}));
  EXPECT_TRUE(grid.isOccupied({17, 18, 19}));
  EXPECT_FALSE(grid.isOccupied({16, 18, 19}));
  EXPECT_EQ(grid.occupiedVoxelCount(), 2U);
  EXPECT_EQ(grid.occupiedChunkCount(), 2U);
  EXPECT_EQ(grid.fingerprint(), 42U);
}

TEST(OccupancyGrid3D, ConvertsWorldCoordinates) {
  OccupancyGrid3D grid{GridBounds3D{-1.0, -2.0, 3.0, 0.5, 4, 6, 8}};
  const std::optional<GridIndex3D> cell = grid.worldToCell({-0.74, -1.26, 4.26});
  if (!cell.has_value()) {
    FAIL() << "expected in-bounds cell";
  }
  const GridIndex3D index = cell.value_or(GridIndex3D{});
  EXPECT_EQ(index, (GridIndex3D{0, 1, 2}));
  const Point3 center = grid.cellCenter(index);
  EXPECT_DOUBLE_EQ(center.x, -0.75);
  EXPECT_DOUBLE_EQ(center.y, -1.25);
  EXPECT_DOUBLE_EQ(center.z, 4.25);
  EXPECT_FALSE(grid.worldToCell({10.0, 0.0, 0.0}).has_value());
}

TEST(OccupancyGrid3D, StoresGeneratedConstrainedFreeSpaceEdges) {
  OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 0.5, 40, 40, 40}};
  grid.addChannelEdge(ConstrainedFreeSpaceEdge{
      .id = "left_turn",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{1.0, 2.0, 5.0}, {3.0, 2.0, 5.0}, {3.0, 4.0, 5.0}}, 0.5,
          10.0),
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .minimum_clearance_m = 3.5,
      .speed_limit_mps = 10.0,
  });

  ASSERT_EQ(grid.channelEdges().size(), 1U);
  const ConstrainedFreeSpaceEdge& edge = grid.channelEdges().front();
  EXPECT_EQ(edge.id, "left_turn");
  EXPECT_DOUBLE_EQ(edge.entry.x, 1.0);
  EXPECT_DOUBLE_EQ(edge.entry.y, 2.0);
  EXPECT_DOUBLE_EQ(edge.entry.z, 5.0);
  EXPECT_DOUBLE_EQ(edge.exit.x, 3.0);
  EXPECT_DOUBLE_EQ(edge.exit.y, 4.0);
  EXPECT_DOUBLE_EQ(edge.exit.z, 5.0);
  EXPECT_DOUBLE_EQ(edge.minimum_clearance_m, 3.5);
}

TEST(OccupancyGrid3D, LoadsChannelGraphFromGeneratedArtifact) {
  const OccupancyGrid3D grid = OccupancyGrid3D::load(TEST_OCCUPANCY3D_PATH);

  ASSERT_EQ(grid.channelEdges().size(), 6U);
  EXPECT_EQ(grid.channelEdges()[0].id, "channel_11_19_l");
  EXPECT_EQ(grid.channelEdges()[1].id, "channel_54_162_straight");
  EXPECT_EQ(grid.channelEdges()[2].id, "channel_108_216_l");
  EXPECT_EQ(grid.channelEdges()[3].id, "channel_108_108_t:west_east");
  EXPECT_EQ(grid.channelEdges()[4].id, "channel_108_108_t:west_north");
  EXPECT_EQ(grid.channelEdges()[5].id, "channel_108_108_t:east_north");
  for (const ConstrainedFreeSpaceEdge& edge : grid.channelEdges()) {
    EXPECT_GT(edge.centerline.size(), 2U);
    EXPECT_DOUBLE_EQ(edge.min_z_m, 1.5);
    EXPECT_DOUBLE_EQ(edge.max_z_m, 8.5);
    EXPECT_DOUBLE_EQ(edge.minimum_clearance_m, 3.5);
    EXPECT_DOUBLE_EQ(edge.speed_limit_mps, 10.0);
  }
}

} // namespace
} // namespace drone_city_nav
