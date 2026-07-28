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

TEST(MppiHorizonSafetyTest, StaticFallbackStopsFromTwentyMetersPerSecond) {
  const mppi::EsdfGrid grid{200, 2, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(400U, 10.0F);
  esdf[100U] = 0.0F;
  const std::vector<mppi::State> horizon{mppi::State{0.5F, 0.5F},
                                         mppi::State{100.5F, 0.5F}};
  mppi::State current{0.5F, 0.5F};
  current.vx = 20.0F;

  const MppiHorizonSafetyResult result =
      evaluateMppiHorizonSafety(current, horizon, esdf, grid,
                                MppiHorizonSafetyConfig{
                                    .minimum_time_to_collision_s = 0.1,
                                    .fallback_duration_s = 3.0,
                                    .dt_s = 0.05,
                                });

  ASSERT_FALSE(result.fallback_horizon.empty());
  EXPECT_NEAR(result.fallback_horizon.back().vx, 0.0F, 1.0e-4F);
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

TEST(MppiHorizonSafetyTest, ExplicitFallbackBrakesWithoutAPlannedHorizon) {
  mppi::State current{1.0F, 2.0F, 3.0F};
  current.vx = 10.0F;

  const MppiHorizonSafetyResult result =
      buildMppiBrakingFallback(current, MppiHorizonSafetyConfig{
                                            .maximum_braking_acceleration_mps2 = 4.0,
                                            .fallback_duration_s = 3.0,
                                            .dt_s = 0.05,
                                        });

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kBrake);
  ASSERT_FALSE(result.fallback_controls.empty());
  ASSERT_EQ(result.fallback_horizon.size(), result.fallback_controls.size() + 1U);
  EXPECT_FLOAT_EQ(result.fallback_horizon.front().vx, 10.0F);
  EXPECT_NEAR(result.fallback_horizon.back().vx, 0.0F, 1.0e-4F);
}

TEST(MppiHorizonSafetyTest, ExplicitFallbackHoldsAnAlreadyStationaryVehicle) {
  const MppiHorizonSafetyResult result =
      buildMppiBrakingFallback(mppi::State{}, MppiHorizonSafetyConfig{});

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kHold);
  ASSERT_FALSE(result.fallback_horizon.empty());
  EXPECT_FLOAT_EQ(result.fallback_horizon.back().vx, 0.0F);
  EXPECT_FLOAT_EQ(result.fallback_horizon.back().vy, 0.0F);
  EXPECT_FLOAT_EQ(result.fallback_horizon.back().vz, 0.0F);
}

} // namespace
} // namespace drone_city_nav
