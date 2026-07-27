#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/mppi/passage_speed_policy.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

namespace drone_city_nav::mppi {
namespace {

TEST(MppiReferenceTest, DynamicsClampsAccelerationAndVelocity) {
  DynamicsConfig config{};
  config.dt_s = 1.0F;
  config.linear_drag_1ps = 0.0F;
  config.maximum_horizontal_acceleration_mps2 = 2.0F;
  config.maximum_vertical_acceleration_mps2 = 1.0F;
  config.maximum_horizontal_speed_mps = 3.0F;
  config.maximum_vertical_speed_mps = 0.5F;
  config.maximum_yaw_acceleration_radps2 = 1.0F;
  config.maximum_yaw_rate_radps = 0.75F;

  const State state =
      integrateReference(State{}, Control{10.0F, 10.0F, 10.0F, 10.0F}, config);

  EXPECT_NEAR(std::hypot(state.vx, state.vy), 2.0F, 1.0e-5F);
  EXPECT_FLOAT_EQ(state.vz, 0.5F);
  EXPECT_FLOAT_EQ(state.yaw_rate, 0.75F);
  EXPECT_NEAR(state.yaw, 0.75F, 1.0e-5F);
}

TEST(MppiReferenceTest, SimulationClassifiesPlanningExposure) {
  constexpr int kWidth = 4;
  constexpr int kHeight = 4;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 3.0F);
  const std::array<Control, 2> nominal{
      Control{1.0F, 0.0F, 0.0F, 0.0F},
      Control{1.0F, 0.0F, 0.0F, 0.0F},
  };
  const std::array<Control, 2> noise{};
  DynamicsConfig dynamics{};
  dynamics.dt_s = 0.1F;
  dynamics.linear_drag_1ps = 0.0F;

  const RolloutMetrics metrics =
      simulateReference(State{1.5F, 1.5F, 0.0F}, nominal, noise, dynamics, RiskConfig{},
                        CostConfig{}, grid, esdf, 3.0F, 1.5F, false);

  EXPECT_FALSE(metrics.collision);
  EXPECT_EQ(metrics.worst_tier, RiskTier::kPlanning);
  EXPECT_GT(metrics.planning_exposure_m, 0.0F);
  EXPECT_FLOAT_EQ(metrics.critical_exposure_m, 0.0F);
}

TEST(MppiReferenceTest, CollisionIsHardAndStopsEarly) {
  constexpr int kWidth = 4;
  constexpr int kHeight = 4;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 0.0F);
  const std::array<Control, 3> nominal{};
  const std::array<Control, 3> noise{};

  const RolloutMetrics metrics =
      simulateReference(State{1.5F, 1.5F, 0.0F}, nominal, noise, DynamicsConfig{},
                        RiskConfig{}, CostConfig{}, grid, esdf, 3.0F, 1.5F, true);

  EXPECT_TRUE(metrics.collision);
  EXPECT_EQ(metrics.worst_tier, RiskTier::kCollision);
  EXPECT_FLOAT_EQ(metrics.minimum_clearance_m, 0.0F);
}

TEST(MppiReferenceTest, PassageSpeedPolicyPreservesMapModeContract) {
  PassageSpeedPolicy policy{};

  EXPECT_FLOAT_EQ(activePassageSpeedLimitMps(policy), 10.0F);
  policy.use_static_map = false;
  EXPECT_FLOAT_EQ(activePassageSpeedLimitMps(policy), 5.0F);
}

} // namespace
} // namespace drone_city_nav::mppi
