#include "drone_city_nav/bounded_worker_pool.hpp"
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

void expectMatchesDenseEdt(const KnownObstacleDistance3D& field,
                           const DistanceField3D& dense) {
  const GridBounds3D& bounds = field.bounds();
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
        const float actual = field.distanceAt(cell);
        const float expected =
            dense.distanceAt({x + offset_x, y + offset_y, z + offset_z});
        if (std::isinf(expected)) {
          EXPECT_TRUE(std::isinf(actual)) << x << ',' << y << ',' << z;
        } else {
          EXPECT_NEAR(actual, expected, 1.0e-5F) << x << ',' << y << ',' << z;
        }
      }
    }
  }
}

TEST(KnownObstacleDistance3DTest, DenseTransformMatchesReferenceEdtIncludingHalo) {
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

  const KnownObstacleDistance3DBuildResult built =
      buildKnownObstacleDistance3D(observed, local, kMaximumDistanceM);
  const GridBounds3D source_bounds =
      knownObstacleDistanceSourceBounds3D(world, local, kMaximumDistanceM);
  const DistanceField3D dense =
      DistanceField3D::buildLocal(dense_occupancy, source_bounds, kMaximumDistanceM);

  ASSERT_TRUE(built.field);
  ASSERT_TRUE(built.field->valid());
  EXPECT_EQ(built.mode, KnownObstacleDistance3DBuildMode::kFull);
  EXPECT_EQ(built.field->sourceVoxelCount(), 3U);
  EXPECT_EQ(built.stats.source_voxels, 3U);
  EXPECT_GT(built.stats.finite_distance_voxels, 0U);
  EXPECT_LT(built.stats.finite_distance_voxels,
            static_cast<std::size_t>(local.width_cells * local.height_cells *
                                     local.depth_cells));
  expectMatchesDenseEdt(*built.field, dense);
  const std::shared_ptr<const std::vector<float>>& projection =
      built.field->denseDistances();
  ASSERT_TRUE(projection);
  EXPECT_EQ(projection->size(),
            static_cast<std::size_t>(local.width_cells * local.height_cells *
                                     local.depth_cells));
}

TEST(KnownObstacleDistance3DTest, WorkerPoolTransformMatchesSerialTransform) {
  const GridBounds3D world{0.0, 0.0, 0.0, 0.5, 96, 80, 40};
  const GridBounds3D local{8.0, 8.0, 4.0, 0.5, 64, 48, 24};
  constexpr double kMaximumDistanceM{4.0};
  ObservedOccupancyGrid3D observed{world};
  OccupancyGrid3D dense_occupancy{world};
  for (int index = 0; index < 60; ++index) {
    addOccupied(observed, dense_occupancy,
                {(index * 37) % 96, (index * 53) % 80, (index * 11) % 40});
  }
  BoundedWorkerPool pool{4U};

  const KnownObstacleDistance3DBuildResult serial =
      buildKnownObstacleDistance3D(observed, local, kMaximumDistanceM);
  const KnownObstacleDistance3DBuildResult parallel =
      buildKnownObstacleDistance3D(observed, local, kMaximumDistanceM, {}, &pool);
  const GridBounds3D source_bounds =
      knownObstacleDistanceSourceBounds3D(world, local, kMaximumDistanceM);
  const DistanceField3D dense =
      DistanceField3D::buildLocal(dense_occupancy, source_bounds, kMaximumDistanceM);

  ASSERT_TRUE(serial.field && parallel.field);
  EXPECT_EQ(*serial.field->denseDistances(), *parallel.field->denseDistances());
  EXPECT_EQ(serial.field->sourceFingerprint(), parallel.field->sourceFingerprint());
  expectMatchesDenseEdt(*parallel.field, dense);
}

TEST(KnownObstacleDistance3DTest, ReusesPreviousFieldOnlyWhileSourcesAreIdentical) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 64, 40, 24};
  constexpr double kMaximumDistanceM{3.5};
  ObservedOccupancyGrid3D occupancy{bounds};
  ASSERT_TRUE(occupancy.setState({8, 8, 8}, ObservedVoxelState::kOccupied));
  ASSERT_TRUE(occupancy.setState({52, 31, 16}, ObservedVoxelState::kOccupied));
  const KnownObstacleDistance3DBuildResult initial =
      buildKnownObstacleDistance3D(occupancy, bounds, kMaximumDistanceM);

  // Free/unknown relabeling never changes the source identity.
  ASSERT_TRUE(occupancy.setState({20, 20, 10}, ObservedVoxelState::kFree));
  const KnownObstacleDistance3DBuildResult relabeled = updateKnownObstacleDistance3D(
      occupancy, bounds, kMaximumDistanceM, initial.field);
  EXPECT_EQ(relabeled.mode, KnownObstacleDistance3DBuildMode::kReused);
  EXPECT_EQ(relabeled.field, initial.field);
  EXPECT_EQ(relabeled.stats.transform_voxels, 0U);

  const GridIndex3D inserted_cell{28, 20, 12};
  ASSERT_TRUE(occupancy.setState(inserted_cell, ObservedVoxelState::kOccupied));
  const KnownObstacleDistance3DBuildResult inserted = updateKnownObstacleDistance3D(
      occupancy, bounds, kMaximumDistanceM, initial.field);
  const KnownObstacleDistance3DBuildResult inserted_full =
      buildKnownObstacleDistance3D(occupancy, bounds, kMaximumDistanceM);
  ASSERT_TRUE(inserted.field && inserted_full.field);
  EXPECT_EQ(inserted.mode, KnownObstacleDistance3DBuildMode::kFull);
  EXPECT_NE(inserted.field, initial.field);
  EXPECT_EQ(*inserted.field->denseDistances(), *inserted_full.field->denseDistances());
  EXPECT_EQ(inserted.field->sourceFingerprint(),
            inserted_full.field->sourceFingerprint());
  EXPECT_NE(inserted.field->sourceFingerprint(), initial.field->sourceFingerprint());

  ASSERT_TRUE(occupancy.setState(inserted_cell, ObservedVoxelState::kFree));
  const KnownObstacleDistance3DBuildResult removed = updateKnownObstacleDistance3D(
      occupancy, bounds, kMaximumDistanceM, inserted.field);
  EXPECT_EQ(removed.mode, KnownObstacleDistance3DBuildMode::kFull);
  EXPECT_EQ(*removed.field->denseDistances(), *initial.field->denseDistances());
  EXPECT_EQ(removed.field->sourceFingerprint(), initial.field->sourceFingerprint());
}

TEST(KnownObstacleDistance3DTest, SuppressedSourcesAreTreatedAsFree) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 24, 24, 12};
  ObservedOccupancyGrid3D occupancy{bounds};
  const GridIndex3D support{12, 12, 6};
  ASSERT_TRUE(occupancy.setState(support, ObservedVoxelState::kOccupied));
  const std::array<GridIndex3D, 1U> suppressed{support};

  const KnownObstacleDistance3DBuildResult with_source =
      buildKnownObstacleDistance3D(occupancy, bounds, 3.0);
  const KnownObstacleDistance3DBuildResult without_source =
      buildKnownObstacleDistance3D(occupancy, bounds, 3.0, suppressed);

  EXPECT_EQ(with_source.field->sourceVoxelCount(), 1U);
  EXPECT_FLOAT_EQ(with_source.field->distanceAt(support), 0.0F);
  EXPECT_EQ(without_source.field->sourceVoxelCount(), 0U);
  EXPECT_TRUE(std::isinf(without_source.field->distanceAt(support)));
  EXPECT_NE(with_source.field->sourceFingerprint(),
            without_source.field->sourceFingerprint());
}

TEST(KnownObstacleDistance3DTest, EmptyWorldIsValidAndEverywhereInfinite) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 0.5, 24, 16, 8};
  const ObservedOccupancyGrid3D occupancy{bounds};

  const KnownObstacleDistance3DBuildResult result =
      buildKnownObstacleDistance3D(occupancy, bounds, 2.0);

  ASSERT_TRUE(result.field);
  EXPECT_TRUE(result.field->valid());
  EXPECT_EQ(result.field->sourceVoxelCount(), 0U);
  EXPECT_EQ(result.field->finiteDistanceVoxelCount(), 0U);
  EXPECT_TRUE(std::isinf(result.field->distanceAt({12, 8, 4})));
  EXPECT_TRUE(std::isinf(result.field->distanceAt({-1, 0, 0})));
}

} // namespace
} // namespace drone_city_nav
