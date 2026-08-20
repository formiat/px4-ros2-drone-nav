#include "drone_city_nav/observed_esdf_3d.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(ObservedEsdf3DTest, PreservesUnknownFreeAndOccupiedSemantics) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 6, 4, 3};
  ObservedOccupancyGrid3D occupancy{bounds};
  static_cast<void>(
      occupancy.setState(GridIndex3D{1, 1, 1}, ObservedVoxelState::kFree));
  static_cast<void>(
      occupancy.setState(GridIndex3D{2, 1, 1}, ObservedVoxelState::kOccupied));

  const ObservedEsdf3D field = buildObservedEsdf3D(occupancy, bounds, 10.0);
  const auto index = [&](const int x, const int y, const int z) {
    return (static_cast<std::size_t>(z) *
                static_cast<std::size_t>(bounds.height_cells) +
            static_cast<std::size_t>(y)) *
               static_cast<std::size_t>(bounds.width_cells) +
           static_cast<std::size_t>(x);
  };

  EXPECT_TRUE(field.grid.outside_is_unknown);
  ASSERT_TRUE(field.local_occupancy);
  EXPECT_EQ(field.grid.depth, bounds.depth_cells);
  EXPECT_EQ(field.distances_m.at(index(0, 0, 0)), mppi::kUnknownEsdfDistanceM);
  EXPECT_GT(field.distances_m.at(index(1, 1, 1)), 0.0F);
  EXPECT_FLOAT_EQ(field.distances_m.at(index(2, 1, 1)), 0.0F);
  EXPECT_EQ(field.stats.known_voxels, 2U);
  EXPECT_EQ(field.stats.free_voxels, 1U);
  EXPECT_EQ(field.stats.occupied_voxels, 1U);
  EXPECT_EQ(field.stats.unknown_voxels, 70U);
  EXPECT_EQ(field.local_occupancy->state(GridIndex3D{0, 0, 0}),
            ObservedVoxelState::kUnknown);
  EXPECT_EQ(field.local_occupancy->state(GridIndex3D{1, 1, 1}),
            ObservedVoxelState::kFree);
  EXPECT_EQ(field.local_occupancy->state(GridIndex3D{2, 1, 1}),
            ObservedVoxelState::kOccupied);
}

TEST(ObservedEsdf3DTest, FingerprintIsDeterministicAndSensitiveToObservedState) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.5, 64, 64, 8};
  const GridBounds3D local =
      selectLocalObservedEsdfBounds(bounds, Point3{8.0, 8.0, 2.0}, 4.0);
  ObservedOccupancyGrid3D occupancy{bounds};
  const std::uint64_t empty = observedOccupancyFingerprint(occupancy, local);

  static_cast<void>(
      occupancy.setState(GridIndex3D{16, 16, 4}, ObservedVoxelState::kFree));
  const std::uint64_t free = observedOccupancyFingerprint(occupancy, local);
  EXPECT_NE(free, empty);
  EXPECT_EQ(free, observedOccupancyFingerprint(occupancy, local));

  static_cast<void>(
      occupancy.setState(GridIndex3D{16, 16, 4}, ObservedVoxelState::kOccupied));
  EXPECT_NE(observedOccupancyFingerprint(occupancy, local), free);
}

TEST(ObservedEsdf3DTest, RecenterHonorsWorldEdges) {
  const GridBounds3D world{0.0, 0.0, 0.0, 1.0, 100, 80, 20};
  const GridBounds3D left =
      selectLocalObservedEsdfBounds(world, Point3{2.0, 40.0, 5.0}, 10.0);

  EXPECT_DOUBLE_EQ(left.origin_x, world.origin_x);
  EXPECT_FALSE(
      localObservedEsdfNeedsRecenter(left, world, Point3{2.0, 40.0, 5.0}, 4.0));
  EXPECT_TRUE(
      localObservedEsdfNeedsRecenter(left, world, Point3{10.0, 40.0, 5.0}, 4.0));
}

} // namespace
} // namespace drone_city_nav
