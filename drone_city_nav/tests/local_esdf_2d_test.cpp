#include "drone_city_nav/local_esdf_2d.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(LocalEsdf2DTest, SelectsCellAlignedWindowAndClipsAtWorldBoundary) {
  const GridBounds world{-10.0, -20.0, 0.5, 100, 80};

  const GridBounds centered = selectLocalEsdfBounds(world, Point2{5.2, -2.4}, 5.0);
  EXPECT_DOUBLE_EQ(centered.origin_x, 0.0);
  EXPECT_DOUBLE_EQ(centered.origin_y, -7.5);
  EXPECT_EQ(centered.width_cells, 20);
  EXPECT_EQ(centered.height_cells, 20);

  const GridBounds clipped = selectLocalEsdfBounds(world, Point2{-9.8, -19.8}, 5.0);
  EXPECT_DOUBLE_EQ(clipped.origin_x, world.origin_x);
  EXPECT_DOUBLE_EQ(clipped.origin_y, world.origin_y);
  EXPECT_EQ(clipped.width_cells, 10);
  EXPECT_EQ(clipped.height_cells, 10);
}

TEST(LocalEsdf2DTest, CropPreservesCellStatesAndOccupiedOnlyFingerprint) {
  OccupancyGrid2D world{GridBounds{0.0, 0.0, 1.0, 8, 6}};
  world.setFree(GridIndex{3, 2});
  world.setOccupied(GridIndex{4, 3});
  const GridBounds local_bounds{2.0, 1.0, 1.0, 4, 4};

  OccupancyGrid2D local = cropOccupancyGrid(world, local_bounds);
  EXPECT_EQ(local.state(GridIndex{1, 1}), CellState::kFree);
  EXPECT_EQ(local.state(GridIndex{2, 2}), CellState::kOccupied);
  EXPECT_EQ(local.state(GridIndex{0, 0}), CellState::kUnknown);

  const std::uint64_t occupied_fingerprint = local.occupiedFingerprint();
  local.setFree(GridIndex{0, 0});
  EXPECT_EQ(local.occupiedFingerprint(), occupied_fingerprint);
  local.setOccupied(GridIndex{0, 0});
  EXPECT_NE(local.occupiedFingerprint(), occupied_fingerprint);
}

TEST(LocalEsdf2DTest, RecentersOnlyTowardExpandableSides) {
  const GridBounds world{0.0, 0.0, 1.0, 100, 100};
  const GridBounds clipped_left{0.0, 20.0, 1.0, 40, 40};

  EXPECT_FALSE(localEsdfNeedsRecenter(clipped_left, world, Point2{2.0, 40.0}, 10.0));
  EXPECT_TRUE(localEsdfNeedsRecenter(clipped_left, world, Point2{35.0, 40.0}, 10.0));
  EXPECT_TRUE(localEsdfNeedsRecenter(clipped_left, world, Point2{20.0, 25.0}, 10.0));
  EXPECT_FALSE(localEsdfNeedsRecenter(clipped_left, world, Point2{20.0, 40.0}, 10.0));
}

TEST(LocalEsdf2DTest, RecentersWhenRawMapGeometryChanges) {
  const GridBounds active{10.0, 10.0, 1.0, 40, 40};

  EXPECT_TRUE(localEsdfNeedsRecenter(active, GridBounds{0.0, 0.0, 0.5, 200, 200},
                                     Point2{30.0, 30.0}, 10.0));
  EXPECT_TRUE(localEsdfNeedsRecenter(active, GridBounds{20.0, 20.0, 1.0, 40, 40},
                                     Point2{30.0, 30.0}, 10.0));
}

} // namespace
} // namespace drone_city_nav
