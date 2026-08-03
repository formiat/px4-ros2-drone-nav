#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/distance_field_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {
namespace {

TEST(DistanceField3D, ComputesEuclideanDistanceInThreeAxes) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 5, 5, 5}};
  occupancy.setOccupied({1, 1, 1});
  const DistanceField3D field = DistanceField3D::build(occupancy, 10.0);

  EXPECT_FLOAT_EQ(field.distanceAt({1, 1, 1}), 0.0F);
  EXPECT_FLOAT_EQ(field.distanceAt({2, 1, 1}), 1.0F);
  EXPECT_NEAR(field.distanceAt({2, 2, 2}), std::sqrt(3.0F), 1.0e-5F);
  EXPECT_EQ(field.stats().source_voxels, 1U);
  EXPECT_GE(field.stats().x_pass_ms, 0.0);
  EXPECT_GE(field.stats().y_pass_ms, 0.0);
  EXPECT_GE(field.stats().z_pass_ms, 0.0);
  EXPECT_GE(field.stats().finalize_ms, 0.0);
  EXPECT_GE(field.stats().duration_ms, field.stats().x_pass_ms +
                                           field.stats().y_pass_ms +
                                           field.stats().z_pass_ms);
}

TEST(DistanceField3D, KeepsFarAndSourceFreeWorldAtInfinity) {
  OccupancyGrid3D empty{GridBounds3D{0.0, 0.0, 0.0, 0.5, 3, 3, 3}};
  const DistanceField3D empty_field = DistanceField3D::build(empty, 2.0);
  EXPECT_TRUE(std::isinf(empty_field.distanceAt({1, 1, 1})));

  empty.setOccupied({0, 0, 0});
  const DistanceField3D capped = DistanceField3D::build(empty, 0.75);
  EXPECT_TRUE(std::isinf(capped.distanceAt({2, 2, 2})));
}

TEST(DistanceField3D, BuildsAlignedDenseLocalRegionFromSparseWorld) {
  OccupancyGrid3D occupancy{GridBounds3D{-10.0, -10.0, 0.0, 1.0, 20, 20, 6}};
  occupancy.setOccupied({12, 13, 2});
  const GridBounds3D local{0.0, 1.0, 1.0, 1.0, 6, 6, 4};

  const DistanceField3D field = DistanceField3D::buildLocal(occupancy, local, 10.0);

  EXPECT_EQ(field.bounds(), local);
  EXPECT_FLOAT_EQ(field.distanceAt({2, 2, 1}), 0.0F);
  EXPECT_FLOAT_EQ(field.distanceAt({3, 2, 1}), 1.0F);
  EXPECT_EQ(field.stats().source_voxels, 1U);
  EXPECT_EQ(field.stats().voxel_count, 144U);
}

TEST(DistanceField3D, ParallelBuildMatchesSerialBuildExactly) {
  OccupancyGrid3D occupancy{GridBounds3D{-2.0, -2.0, 0.0, 0.5, 9, 8, 7}};
  occupancy.setOccupied({1, 2, 3});
  occupancy.setOccupied({7, 5, 1});
  const DistanceField3D serial = DistanceField3D::build(occupancy, 10.0);
  BoundedWorkerPool worker_pool{4U};
  const DistanceField3D parallel =
      DistanceField3D::build(occupancy, 10.0, &worker_pool);

  ASSERT_EQ(serial.distancesM().size(), parallel.distancesM().size());
  for (std::size_t index = 0U; index < serial.distancesM().size(); ++index) {
    EXPECT_EQ(serial.distancesM()[index], parallel.distancesM()[index]);
  }
  EXPECT_EQ(serial.stats().source_voxels, parallel.stats().source_voxels);
}

} // namespace
} // namespace drone_city_nav
