#include "drone_city_nav/esdf_query.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(EsdfQueryTest, ConvertsCenterDistanceToConservativeOccupiedCellClearance) {
  const mppi::EsdfGrid grid{3, 1, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf{2.0F, 1.0F, 0.0F};

  const EsdfQueryResult result = queryConservativeEsdf(grid, esdf, 0.5F, 0.5F);

  EXPECT_EQ(result.status, EsdfQueryStatus::kValid);
  EXPECT_FALSE(result.raw_occupied);
  EXPECT_NEAR(result.clearance_m, 2.0F - 0.70710678F, 1.0e-5F);
}

TEST(EsdfQueryTest, AccountsForQueryOffsetInsideCell) {
  const mppi::EsdfGrid grid{3, 1, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf{2.0F, 1.0F, 0.0F};

  const EsdfQueryResult center = queryConservativeEsdf(grid, esdf, 0.5F, 0.5F);
  const EsdfQueryResult edge = queryConservativeEsdf(grid, esdf, 0.99F, 0.5F);

  ASSERT_EQ(center.status, EsdfQueryStatus::kValid);
  ASSERT_EQ(edge.status, EsdfQueryStatus::kValid);
  EXPECT_LT(edge.clearance_m, center.clearance_m);
}

TEST(EsdfQueryTest, PreservesPositiveInfinityAndRejectsInvalidSamples) {
  const mppi::EsdfGrid grid{2, 1, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf{std::numeric_limits<float>::infinity(),
                                std::numeric_limits<float>::quiet_NaN()};

  const EsdfQueryResult unbounded = queryConservativeEsdf(grid, esdf, 0.5F, 0.5F);
  const EsdfQueryResult invalid = queryConservativeEsdf(grid, esdf, 1.5F, 0.5F);
  const EsdfQueryResult outside = queryConservativeEsdf(grid, esdf, 2.5F, 0.5F);

  EXPECT_EQ(unbounded.status, EsdfQueryStatus::kValid);
  EXPECT_TRUE(std::isinf(unbounded.clearance_m));
  EXPECT_FALSE(unbounded.raw_occupied);
  EXPECT_EQ(invalid.status, EsdfQueryStatus::kInvalidDistance);
  EXPECT_TRUE(invalid.raw_occupied);
  EXPECT_EQ(outside.status, EsdfQueryStatus::kOutsideGrid);
  EXPECT_FALSE(outside.raw_occupied);
  EXPECT_TRUE(std::isinf(outside.clearance_m));
}

TEST(EsdfQueryTest, SeparatesRawOccupancyFromConservativeRiskClearance) {
  const mppi::EsdfGrid grid{2, 1, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf{1.0F, 0.0F};

  const EsdfQueryResult free_near_wall = queryConservativeEsdf(grid, esdf, 0.99F, 0.5F);
  const EsdfQueryResult occupied = queryConservativeEsdf(grid, esdf, 1.5F, 0.5F);

  ASSERT_EQ(free_near_wall.status, EsdfQueryStatus::kValid);
  EXPECT_FALSE(free_near_wall.raw_occupied);
  EXPECT_FLOAT_EQ(free_near_wall.clearance_m, 0.0F);
  ASSERT_EQ(occupied.status, EsdfQueryStatus::kValid);
  EXPECT_TRUE(occupied.raw_occupied);
}

TEST(EsdfQueryTest, QueriesThreeDimensionalGridUsingZMajorStorage) {
  mppi::EsdfGrid grid{2, 2, 1.0F, 10.0F, 20.0F};
  grid.depth = 2;
  grid.origin_z_m = 30.0F;
  const std::vector<float> esdf{
      4.0F, 4.0F, 4.0F, 4.0F, 3.0F, 2.0F, 1.0F, 0.0F,
  };

  const EsdfQueryResult free = queryConservativeEsdf3D(grid, esdf, 10.5F, 20.5F, 31.5F);
  const EsdfQueryResult occupied =
      queryConservativeEsdf3D(grid, esdf, 11.5F, 21.5F, 31.5F);
  const EsdfQueryResult outside =
      queryConservativeEsdf3D(grid, esdf, 10.5F, 20.5F, 32.5F);

  ASSERT_EQ(free.status, EsdfQueryStatus::kValid);
  EXPECT_FALSE(free.raw_occupied);
  EXPECT_NEAR(free.clearance_m, 3.0F - 0.8660254F, 1.0e-5F);
  ASSERT_EQ(occupied.status, EsdfQueryStatus::kValid);
  EXPECT_TRUE(occupied.raw_occupied);
  EXPECT_EQ(outside.status, EsdfQueryStatus::kOutsideGrid);
  EXPECT_FALSE(outside.raw_occupied);
  EXPECT_TRUE(std::isinf(outside.clearance_m));
}

} // namespace
} // namespace drone_city_nav
