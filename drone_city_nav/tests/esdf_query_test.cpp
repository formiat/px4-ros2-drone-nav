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
  EXPECT_EQ(invalid.status, EsdfQueryStatus::kInvalidDistance);
  EXPECT_EQ(outside.status, EsdfQueryStatus::kOutsideGrid);
}

} // namespace
} // namespace drone_city_nav
