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

TEST(MppiReferenceTest, NearWallFreeCellIsCriticalRatherThanCollision) {
  const EsdfGrid grid{2, 1, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf{1.0F, 0.0F};
  const std::array<Control, 1> controls{};
  const std::array<Control, 1> noise{};
  DynamicsConfig dynamics{};
  dynamics.dt_s = 0.1F;
  State initial{.x = 0.99F, .y = 0.5F};

  const RolloutMetrics metrics =
      simulateReference(initial, controls, noise, dynamics, RiskConfig{}, CostConfig{},
                        grid, esdf, 0.99F, 0.5F, false);

  EXPECT_FALSE(metrics.collision);
  EXPECT_EQ(metrics.worst_tier, RiskTier::kCritical);
}

TEST(MppiReferenceTest, HeadProgressIsMeasuredAtConfiguredEarlyHorizon) {
  constexpr int kWidth = 20;
  constexpr int kHeight = 20;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 20.0F);
  const std::array<Control, 4> controls{
      Control{.ax = 1.0F},
      Control{.ax = 1.0F},
      Control{.ax = 1.0F},
      Control{.ax = 1.0F},
  };
  const std::array<Control, 4> noise{};
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  dynamics.linear_drag_1ps = 0.0F;
  CostConfig costs;
  costs.head_progress_horizon_s = 0.2F;

  const RolloutMetrics metrics =
      simulateReference(State{1.5F, 1.5F, 0.0F}, controls, noise, dynamics,
                        RiskConfig{}, costs, grid, esdf, 10.0F, 1.5F, false);

  EXPECT_GT(metrics.costs.head_progress, 0.0F);
  EXPECT_GT(-metrics.costs.progress, metrics.costs.head_progress);
}

TEST(MppiReferenceTest, AppliedControlDefinesFirstJerkCost) {
  constexpr int kWidth = 20;
  constexpr int kHeight = 20;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 20.0F);
  const std::array<Control, 2> controls{
      Control{.ax = 2.0F},
      Control{.ax = 2.0F},
  };
  const std::array<Control, 2> noise{};

  const RolloutMetrics from_zero =
      simulateReference(State{1.5F, 1.5F, 0.0F}, controls, noise, DynamicsConfig{},
                        RiskConfig{}, CostConfig{}, grid, esdf, 10.0F, 1.5F, false);
  const RolloutMetrics from_applied = simulateReference(
      State{1.5F, 1.5F, 0.0F}, controls, noise, DynamicsConfig{}, RiskConfig{},
      CostConfig{}, grid, esdf, 10.0F, 1.5F, false, Control{.ax = 2.0F});

  EXPECT_GT(from_zero.costs.jerk, from_applied.costs.jerk);
  EXPECT_FLOAT_EQ(from_applied.costs.jerk, 0.0F);
}

TEST(MppiReferenceTest, ReferenceSpeedAddsTrackingCost) {
  constexpr int kWidth = 20;
  constexpr int kHeight = 20;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 20.0F);
  const std::array<Control, 2> controls{};
  const std::array<Control, 2> noise{};
  State initial{.x = 1.5F, .y = 1.5F, .vx = 5.0F};

  const RolloutMetrics disabled =
      simulateReference(initial, controls, noise, DynamicsConfig{}, RiskConfig{},
                        CostConfig{}, grid, esdf, 10.0F, 1.5F, false);
  const RolloutMetrics matched =
      simulateReference(initial, controls, noise, DynamicsConfig{}, RiskConfig{},
                        CostConfig{}, grid, esdf, 10.0F, 1.5F, false, Control{}, 5.0F);
  const RolloutMetrics faster =
      simulateReference(initial, controls, noise, DynamicsConfig{}, RiskConfig{},
                        CostConfig{}, grid, esdf, 10.0F, 1.5F, false, Control{}, 10.0F);

  EXPECT_FLOAT_EQ(disabled.costs.speed_tracking, 0.0F);
  EXPECT_LT(matched.costs.speed_tracking, faster.costs.speed_tracking);
  EXPECT_LT(matched.soft_cost, faster.soft_cost);
}

TEST(MppiReferenceTest, PassageSpeedPolicyPreservesMapModeContract) {
  PassageSpeedPolicy policy{};

  EXPECT_FLOAT_EQ(activePassageSpeedLimitMps(policy), 10.0F);
  policy.use_static_map = false;
  EXPECT_FLOAT_EQ(activePassageSpeedLimitMps(policy), 5.0F);
}

} // namespace
} // namespace drone_city_nav::mppi
