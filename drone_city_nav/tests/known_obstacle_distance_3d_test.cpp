#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/known_obstacle_distance_3d.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <span>
#include <vector>

namespace drone_city_nav {
namespace {

void addOccupied(ObservedOccupancyGrid3D& observed, OccupancyGrid3D& dense,
                 const GridIndex3D cell) {
  ASSERT_TRUE(observed.setState(cell, ObservedVoxelState::kOccupied));
  dense.setOccupied(cell);
}

void expectMatchesDenseEdt(const KnownObstacleDistance3D& sparse,
                           const DistanceField3D& dense) {
  const GridBounds3D& bounds = sparse.bounds();
  const GridBounds3D& dense_bounds = dense.bounds();
  const int offset_x = static_cast<int>(
      std::llround((bounds.origin_x - dense_bounds.origin_x) / bounds.resolution_m));
  const int offset_y = static_cast<int>(
      std::llround((bounds.origin_y - dense_bounds.origin_y) / bounds.resolution_m));
  const int offset_z = static_cast<int>(
      std::llround((bounds.origin_z - dense_bounds.origin_z) / bounds.resolution_m));
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        const GridIndex3D cell{x, y, z};
        const float sparse_distance = sparse.distanceAt(cell);
        const float dense_distance =
            dense.distanceAt({x + offset_x, y + offset_y, z + offset_z});
        if (std::isinf(dense_distance)) {
          EXPECT_TRUE(std::isinf(sparse_distance));
        } else {
          EXPECT_NEAR(sparse_distance, dense_distance, 1.0e-5F);
        }
      }
    }
  }
}

TEST(KnownObstacleDistance3DTest, SparseProjectionMatchesExactDenseEdtIncludingHalo) {
  const GridBounds3D world{-3.0, -2.0, -1.0, 0.5, 28, 24, 16};
  const GridBounds3D local{0.0, 0.5, 0.5, 0.5, 12, 10, 8};
  constexpr double kMaximumDistanceM{2.25};
  ObservedOccupancyGrid3D observed{world};
  OccupancyGrid3D dense_occupancy{world};
  addOccupied(observed, dense_occupancy, {8, 8, 6});
  addOccupied(observed, dense_occupancy, {2, 7, 5});
  addOccupied(observed, dense_occupancy, {19, 10, 7});
  // This source is inside the rectangular halo but outside the exact capped
  // influence radius of every output voxel.
  addOccupied(observed, dense_occupancy, {1, 0, 0});

  const KnownObstacleDistance3DBuildResult sparse =
      buildKnownObstacleDistance3D(observed, local, kMaximumDistanceM);
  const GridBounds3D source_bounds =
      knownObstacleDistanceSourceBounds3D(world, local, kMaximumDistanceM);
  const DistanceField3D dense =
      DistanceField3D::buildLocal(dense_occupancy, source_bounds, kMaximumDistanceM);

  ASSERT_TRUE(sparse.field);
  ASSERT_TRUE(sparse.field->valid());
  EXPECT_EQ(sparse.mode, KnownObstacleDistance3DBuildMode::kFull);
  EXPECT_EQ(sparse.field->sourceVoxelCount(), 3U);
  EXPECT_EQ(sparse.stats.source_voxels, 3U);
  EXPECT_GT(sparse.stats.stored_distance_chunks, 0U);
  EXPECT_LT(sparse.stats.finite_distance_voxels,
            static_cast<std::size_t>(local.width_cells * local.height_cells *
                                     local.depth_cells));
  expectMatchesDenseEdt(*sparse.field, dense);
  const std::shared_ptr<const std::vector<float>> projection =
      sparse.field->materializeDense();
  ASSERT_TRUE(projection);
  EXPECT_EQ(projection->size(),
            static_cast<std::size_t>(local.width_cells * local.height_cells *
                                     local.depth_cells));
}

TEST(KnownObstacleDistance3DTest,
     IncrementalInsertAndRemovalReuseUnaffectedSparseChunksExactly) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 64, 40, 24};
  constexpr double kMaximumDistanceM{3.5};
  ObservedOccupancyGrid3D occupancy{bounds};
  ASSERT_TRUE(occupancy.setState({8, 8, 8}, ObservedVoxelState::kOccupied));
  ASSERT_TRUE(occupancy.setState({52, 31, 16}, ObservedVoxelState::kOccupied));
  const KnownObstacleDistance3DBuildResult initial =
      buildKnownObstacleDistance3D(occupancy, bounds, kMaximumDistanceM);
  const ObservedOccupancyGrid3D before_insert = occupancy;
  const GridIndex3D inserted_cell{28, 20, 12};
  ASSERT_TRUE(occupancy.setState(inserted_cell, ObservedVoxelState::kOccupied));
  const OccupancyChunkIndex3D dirty =
      ObservedOccupancyGrid3D::chunkIndex(inserted_cell);

  const KnownObstacleDistance3DBuildResult inserted =
      updateKnownObstacleDistance3D(occupancy, bounds, kMaximumDistanceM, initial.field,
                                    &before_insert, std::span{&dirty, 1U}, false, 0.5);
  const KnownObstacleDistance3DBuildResult inserted_full =
      buildKnownObstacleDistance3D(occupancy, bounds, kMaximumDistanceM);

  ASSERT_TRUE(inserted.field);
  ASSERT_TRUE(inserted_full.field);
  EXPECT_EQ(inserted.mode, KnownObstacleDistance3DBuildMode::kIncremental);
  EXPECT_EQ(inserted.stats.inserted_sources, 1U);
  EXPECT_EQ(inserted.stats.removed_sources, 0U);
  EXPECT_GT(inserted.stats.changed_chunks, 0U);
  EXPECT_GT(inserted.stats.reused_chunks, 0U);
  EXPECT_EQ(*inserted.field->materializeDense(),
            *inserted_full.field->materializeDense());
  EXPECT_EQ(inserted.field->sourceFingerprint(),
            inserted_full.field->sourceFingerprint());

  const ObservedOccupancyGrid3D before_remove = occupancy;
  ASSERT_TRUE(occupancy.setState(inserted_cell, ObservedVoxelState::kFree));
  const KnownObstacleDistance3DBuildResult removed = updateKnownObstacleDistance3D(
      occupancy, bounds, kMaximumDistanceM, inserted.field, &before_remove,
      std::span{&dirty, 1U}, false, 0.5);
  const KnownObstacleDistance3DBuildResult removed_full =
      buildKnownObstacleDistance3D(occupancy, bounds, kMaximumDistanceM);

  ASSERT_TRUE(removed.field);
  ASSERT_TRUE(removed_full.field);
  EXPECT_EQ(removed.mode, KnownObstacleDistance3DBuildMode::kIncremental);
  EXPECT_EQ(removed.stats.inserted_sources, 0U);
  EXPECT_EQ(removed.stats.removed_sources, 1U);
  EXPECT_GT(removed.stats.changed_chunks, 0U);
  EXPECT_GT(removed.stats.reused_chunks, 0U);
  EXPECT_EQ(*removed.field->materializeDense(),
            *removed_full.field->materializeDense());
  EXPECT_EQ(removed.field->sourceFingerprint(),
            removed_full.field->sourceFingerprint());
}

TEST(KnownObstacleDistance3DTest, EmptyWorldStoresNoDistanceChunks) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.5, 24, 16, 8};
  const ObservedOccupancyGrid3D occupancy{bounds};

  const KnownObstacleDistance3DBuildResult result =
      buildKnownObstacleDistance3D(occupancy, bounds, 2.0);

  ASSERT_TRUE(result.field);
  EXPECT_TRUE(result.field->valid());
  EXPECT_EQ(result.field->sourceVoxelCount(), 0U);
  EXPECT_EQ(result.field->sourceChunkCount(), 0U);
  EXPECT_EQ(result.field->storedDistanceChunkCount(), 0U);
  EXPECT_EQ(result.field->finiteDistanceVoxelCount(), 0U);
  EXPECT_TRUE(std::isinf(result.field->distanceAt({12, 8, 4})));
}

} // namespace
} // namespace drone_city_nav
