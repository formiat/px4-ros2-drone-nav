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

} // namespace
} // namespace drone_city_nav
