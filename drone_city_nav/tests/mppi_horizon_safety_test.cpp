#include "drone_city_nav/mppi_horizon_safety.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {
namespace {

TEST(MppiHorizonSafetyTest, ExecutesCollisionFreeHorizon) {
  const mppi::EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(400U, 10.0F);
  const std::vector<mppi::State> horizon{mppi::State{1.0F, 1.0F},
                                         mppi::State{2.0F, 1.0F}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      mppi::State{1.0F, 1.0F}, horizon, esdf, grid, MppiHorizonSafetyConfig{});

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_TRUE(result.fallback_horizon.empty());
}

TEST(MppiHorizonSafetyTest, ProducesBrakingFallbackBeforeCollision) {
  const mppi::EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(400U, 10.0F);
  esdf[1U * 20U + 6U] = 0.0F;
  std::vector<mppi::State> horizon;
  horizon.reserve(8U);
  for (int index = 0; index < 8; ++index) {
    horizon.push_back(mppi::State{static_cast<float>(index), 1.0F});
  }
  mppi::State current{1.0F, 1.0F};
  current.vx = 2.0F;

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, esdf, grid,
      MppiHorizonSafetyConfig{.minimum_time_to_collision_s = 0.1, .dt_s = 0.2});

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kBrake);
  ASSERT_FALSE(result.fallback_horizon.empty());
  EXPECT_LT(result.fallback_horizon.back().vx, current.vx);
}

TEST(MppiHorizonSafetyTest, TreatsOutsideGridAsCollision) {
  const mppi::EsdfGrid grid{2, 2, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(4U, 10.0F);
  const std::vector<mppi::State> horizon{mppi::State{1.0F, 1.0F},
                                         mppi::State{3.0F, 1.0F}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      mppi::State{1.0F, 1.0F}, horizon, esdf, grid, MppiHorizonSafetyConfig{});

  EXPECT_NE(result.decision, MppiHorizonSafetyDecision::kExecute);
}

} // namespace
} // namespace drone_city_nav
