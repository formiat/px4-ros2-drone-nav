#include "drone_city_nav/mppi/mppi_altitude_envelope.hpp"
#include "drone_city_nav/mppi/mppi_clearance_cost.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

namespace drone_city_nav::mppi {
namespace {

TEST(MppiReferenceTest, ActiveRolloutBudgetUsesCapacityOrValidatedPrefix) {
  EXPECT_EQ(resolveMppiActiveRollouts(8192U, std::nullopt), 8192U);
  EXPECT_EQ(resolveMppiActiveRollouts(8192U, 4096U), 4096U);
  EXPECT_THROW(static_cast<void>(resolveMppiActiveRollouts(8192U, 0U)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(resolveMppiActiveRollouts(8192U, 8193U)),
               std::invalid_argument);
}

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

TEST(MppiReferenceTest, AnInheritedSpeedAboveTheCapIsShedAtTheMaximumDeceleration) {
  DynamicsConfig dynamics;
  dynamics.linear_drag_1ps = 0.0F;
  dynamics.dt_s = 0.1F;
  dynamics.maximum_horizontal_acceleration_mps2 = 4.0F;
  dynamics.maximum_horizontal_speed_mps = 10.0F;
  dynamics.maximum_translational_speed_mps = 6.0F;
  const State inherited{.vx = 9.0F};

  // Even a rollout that keeps accelerating loses at least one step of maximum
  // deceleration per step while above the cap ...
  const State kept = integrateReference(inherited, Control{.ax = 4.0F}, dynamics);
  EXPECT_NEAR(kept.vx, 8.6F, 1.0e-5F);
  // ... and a rollout that brakes is not clamped below what braking leaves.
  const State braked = integrateReference(inherited, Control{.ax = -4.0F}, dynamics);
  EXPECT_NEAR(braked.vx, 8.6F, 1.0e-5F);
  // Once under the cap the cap itself holds.
  const State under =
      integrateReference(State{.vx = 5.9F}, Control{.ax = 4.0F}, dynamics);
  EXPECT_NEAR(under.vx, 6.0F, 1.0e-5F);
}

TEST(MppiReferenceTest, DynamicsClampsTheCompleteTranslationalSpeedVector) {
  DynamicsConfig config{};
  config.dt_s = 1.0F;
  config.linear_drag_1ps = 0.0F;
  config.maximum_horizontal_acceleration_mps2 = 10.0F;
  config.maximum_vertical_acceleration_mps2 = 10.0F;
  config.maximum_horizontal_speed_mps = 10.0F;
  config.maximum_vertical_speed_mps = 10.0F;
  config.maximum_translational_speed_mps = 2.0F;

  const State state =
      integrateReference(State{}, Control{4.0F, 0.0F, 4.0F, 0.0F}, config);

  EXPECT_NEAR(std::hypot(std::hypot(state.vx, state.vy), state.vz), 2.0F, 1.0e-5F);
  EXPECT_GT(state.vx, 0.0F);
  EXPECT_GT(state.vz, 0.0F);
}

TEST(MppiReferenceTest, MeasuredAccelerationResolvesEquivalentDynamicControl) {
  DynamicsConfig config{};
  config.linear_drag_1ps = 0.1F;
  config.maximum_horizontal_acceleration_mps2 = 20.0F;
  config.maximum_vertical_acceleration_mps2 = 10.0F;
  const State state{.vx = 10.0F, .vy = -5.0F, .vz = 2.0F};

  const Control control =
      equivalentControlFromMeasuredAcceleration(state, 1.0F, 2.0F, -0.5F, config);

  EXPECT_FLOAT_EQ(control.ax, 2.0F);
  EXPECT_FLOAT_EQ(control.ay, 1.5F);
  EXPECT_FLOAT_EQ(control.az, -0.3F);
  EXPECT_FLOAT_EQ(control.yaw_accel, 0.0F);
}

TEST(MppiReferenceTest, MeasuredAccelerationControlRespectsDynamicLimits) {
  DynamicsConfig config{};
  config.linear_drag_1ps = 0.0F;
  config.maximum_horizontal_acceleration_mps2 = 4.0F;
  config.maximum_vertical_acceleration_mps2 = 2.0F;

  const Control control =
      equivalentControlFromMeasuredAcceleration(State{}, 10.0F, 10.0F, -3.0F, config);

  EXPECT_NEAR(std::hypot(control.ax, control.ay), 4.0F, 1.0e-5F);
  EXPECT_FLOAT_EQ(control.az, -2.0F);
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

  EXPECT_EQ(metrics.worst_tier, RiskTier::kPlanning);
  EXPECT_GT(metrics.planning_exposure_m, 0.0F);
  EXPECT_FLOAT_EQ(metrics.critical_exposure_m, 0.0F);
}

TEST(MppiReferenceTest, DerivedZeroClearanceIsCriticalButNotHardCollision) {
  constexpr int kWidth = 4;
  constexpr int kHeight = 4;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 0.0F);
  const std::array<Control, 3> nominal{};
  const std::array<Control, 3> noise{};

  const RolloutMetrics metrics =
      simulateReference(State{1.5F, 1.5F, 0.0F}, nominal, noise, DynamicsConfig{},
                        RiskConfig{}, CostConfig{}, grid, esdf, 3.0F, 1.5F, true);

  EXPECT_EQ(metrics.worst_tier, RiskTier::kCritical);
  EXPECT_FLOAT_EQ(metrics.minimum_clearance_m, 0.0F);
}

TEST(MppiReferenceTest, DerivedZeroClearanceKeepsFullHorizon) {
  constexpr int kWidth = 4;
  constexpr int kHeight = 4;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 0.0F);
  const std::array<Control, 3> controls{
      Control{.ax = 1.0F},
      Control{.ax = 1.0F},
      Control{.ax = 1.0F},
  };
  const std::array<Control, 3> noise{};
  ReferenceSimulationTrace trace;

  const RolloutMetrics metrics =
      simulateReference(State{1.5F, 1.5F, 0.0F}, controls, noise, DynamicsConfig{},
                        RiskConfig{}, CostConfig{}, grid, esdf, 3.0F, 1.5F, true,
                        Control{}, -1.0F, FootprintConfig{}, std::nullopt, &trace);

  EXPECT_EQ(metrics.worst_tier, RiskTier::kCritical);
  ASSERT_EQ(trace.horizon.size(), controls.size() + 1U);
  EXPECT_GT(trace.horizon.back().x, trace.horizon.at(1U).x);
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

  EXPECT_EQ(metrics.worst_tier, RiskTier::kCritical);
}

TEST(MppiReferenceTest, ClearanceExposureIsStronglyPenalizedWithoutBlockingMotion) {
  constexpr int kWidth = 20;
  constexpr int kHeight = 4;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> clear_esdf(static_cast<std::size_t>(kWidth * kHeight),
                                      10.0F);
  const std::vector<float> planning_esdf(static_cast<std::size_t>(kWidth * kHeight),
                                         3.0F);
  const std::vector<float> critical_esdf(static_cast<std::size_t>(kWidth * kHeight),
                                         0.5F);
  const std::array<Control, 2> controls{Control{.ax = 1.0F}, Control{.ax = 1.0F}};
  const std::array<Control, 2> noise{};
  DynamicsConfig dynamics{};
  dynamics.dt_s = 0.5F;
  dynamics.linear_drag_1ps = 0.0F;
  CostConfig costs{};

  const auto simulate = [&](const std::span<const float> esdf) {
    return simulateReference(State{1.5F, 1.5F, 0.0F}, controls, noise, dynamics,
                             RiskConfig{}, costs, grid, esdf, 10.0F, 1.5F, false);
  };
  const RolloutMetrics clear = simulate(clear_esdf);
  const RolloutMetrics planning = simulate(planning_esdf);
  const RolloutMetrics critical = simulate(critical_esdf);

  EXPECT_EQ(planning.worst_tier, RiskTier::kPlanning);
  EXPECT_EQ(critical.worst_tier, RiskTier::kCritical);
  EXPECT_GT(planning.soft_cost, clear.soft_cost);
  EXPECT_GT(critical.soft_cost, planning.soft_cost);
}

TEST(MppiReferenceTest, TheTubeLawReadsBothWays) {
  // The cost is the squared shortfall against the tracking error the speed
  // can accumulate; the speed cap is the inverse of the same expression.
  // Feeding one into the other has to close.
  constexpr float kResponse{0.15F};
  for (const float clearance_m : {0.1F, 0.5F, 1.0F, 3.0F}) {
    const float speed_mps = tubeAdmissibleSpeedMps(clearance_m, kResponse);
    EXPECT_NEAR(speed_mps * kResponse, clearance_m, 1.0e-5F);
    EXPECT_FLOAT_EQ(tubeClearanceDeficitM2(clearance_m, speed_mps, kResponse), 0.0F);
    EXPECT_GT(tubeClearanceDeficitM2(clearance_m, 2.0F * speed_mps, kResponse), 0.0F);
  }
  // At contact nothing is admissible, and standing still costs nothing.
  EXPECT_FLOAT_EQ(tubeAdmissibleSpeedMps(0.0F, kResponse), 0.0F);
  EXPECT_FLOAT_EQ(tubeClearanceDeficitM2(0.0F, 0.0F, kResponse), 0.0F);
  EXPECT_NEAR(tubeClearanceDeficitM2(0.0F, 2.0F, kResponse), 0.09F, 1.0e-6F);
}

TEST(MppiReferenceTest, TheStoppingLawPricesTheFreePathAhead) {
  // 4 m/s with 0.1 s reaction and 4 m/s^2 needs 0.4 + 2 = 2.4 m.
  EXPECT_NEAR(requiredStoppingDistanceM(4.0F, 0.1F, 4.0F), 2.4F, 1.0e-5F);
  EXPECT_FLOAT_EQ(stoppingDistanceDeficitM2(10.0F, 4.0F, 0.1F, 4.0F), 0.0F);
  EXPECT_FLOAT_EQ(stoppingDistanceDeficitM2(2.4F, 4.0F, 0.1F, 4.0F), 0.0F);
  const float moderate = stoppingDistanceDeficitM2(2.0F, 4.0F, 0.1F, 4.0F);
  const float close = stoppingDistanceDeficitM2(1.0F, 4.0F, 0.1F, 4.0F);
  EXPECT_GT(moderate, 0.0F);
  EXPECT_GT(close, moderate);
  // Nothing is owed while standing still.
  EXPECT_FLOAT_EQ(stoppingDistanceDeficitM2(0.0F, 0.0F, 0.1F, 4.0F), 0.0F);
}

TEST(MppiReferenceTest, TheClearancePreferencePricesDepthNotMotion) {
  EXPECT_FLOAT_EQ(clearancePreferenceDepth(6.0F, 6.0F), 0.0F);
  EXPECT_FLOAT_EQ(clearancePreferenceDepth(10.0F, 6.0F), 0.0F);
  EXPECT_NEAR(clearancePreferenceDepth(3.0F, 6.0F), 0.5F, 1.0e-6F);
  EXPECT_FLOAT_EQ(clearancePreferenceDepth(0.0F, 6.0F), 1.0F);
  EXPECT_FLOAT_EQ(
      clearancePreferenceDepth(std::numeric_limits<float>::infinity(), 6.0F), 0.0F);
}

TEST(MppiReferenceTest, BesideAWallOnlyTheTubeLawIsPriced) {
  // A wall that never gets nearer owes no braking distance: below the tube
  // speed the approach charge is zero however far the rollout flies, and only
  // a speed whose tracking error would not fit the clearance is priced.
  constexpr int kWidth = 16;
  const EsdfGrid grid{kWidth, 1, 1.0F, 0.0F, 0.0F};
  const std::array<Control, 4> controls{};
  const std::array<Control, 4> noise{};
  DynamicsConfig dynamics{};
  dynamics.dt_s = 0.25F;
  dynamics.linear_drag_1ps = 0.0F;
  CostConfig costs{};
  RiskConfig risk{};
  risk.tube_response_time_s = 0.15F;
  const auto simulate = [&](const float clearance_m, const float speed_mps) {
    const std::vector<float> esdf(static_cast<std::size_t>(kWidth), clearance_m);
    return simulateReference(State{.x = 0.5F, .y = 0.5F, .vx = speed_mps}, controls,
                             noise, dynamics, risk, costs, grid, esdf, 15.5F, 0.5F,
                             false);
  };

  const RolloutMetrics open_slow = simulate(10.0F, 0.5F);
  const RolloutMetrics close_slow = simulate(2.0F, 4.0F);
  const RolloutMetrics close_fast = simulate(2.0F, 9.0F);

  EXPECT_FLOAT_EQ(open_slow.costs.obstacle_approach_m2_s, 0.0F);
  // The conservative query leaves at least 2 - 0.5 - 0.707 = 0.79 m along the
  // row: 4 m/s needs 0.6 m, 9 m/s needs 1.35 m.
  EXPECT_FLOAT_EQ(close_slow.costs.obstacle_approach_m2_s, 0.0F);
  EXPECT_GT(close_fast.costs.obstacle_approach_m2_s, 0.0F);
  EXPECT_FLOAT_EQ(close_slow.costs.stopping_deficit_m2_s, 0.0F);
  EXPECT_FLOAT_EQ(close_fast.costs.stopping_deficit_m2_s, 0.0F);
  EXPECT_TRUE(std::isinf(close_fast.contact_distance_m));
  // Position is still priced, at either speed, and open space is not.
  EXPECT_FLOAT_EQ(open_slow.costs.clearance_preference_s, 0.0F);
  EXPECT_GT(close_slow.costs.clearance_preference_s, 0.0F);
  EXPECT_GT(close_fast.costs.clearance_preference_s, 0.0F);
}

TEST(MppiReferenceTest, AWallAheadChargesTheStatesThatCannotStopBeforeIt) {
  // One occupied cell three metres ahead of a rollout at 4 m/s: its envelope
  // enters the cell on the third step, and every state before that owed the
  // stopping law the free path it had. The same motion across an open grid
  // owes nothing.
  constexpr int kWidth = 6;
  constexpr int kHeight = 3;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  std::vector<float> walled(static_cast<std::size_t>(kWidth * kHeight), 0.0F);
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      walled[static_cast<std::size_t>(y) * static_cast<std::size_t>(kWidth) +
             static_cast<std::size_t>(x)] =
          std::hypot(static_cast<float>(x - 3), static_cast<float>(y - 1));
    }
  }
  const std::vector<float> open(static_cast<std::size_t>(kWidth * kHeight), 10.0F);
  const std::array<Control, 4> controls{};
  const std::array<Control, 4> noise{};
  DynamicsConfig dynamics{};
  dynamics.dt_s = 0.25F;
  dynamics.linear_drag_1ps = 0.0F;
  RiskConfig risk{};
  risk.stopping_response_time_s = 0.1F;
  risk.stopping_deceleration_mps2 = 4.0F;
  const auto simulate = [&](const std::span<const float> esdf) {
    return simulateReference(State{.x = 0.5F, .y = 1.5F, .vx = 4.0F}, controls, noise,
                             dynamics, risk, CostConfig{}, grid, esdf, 5.5F, 1.5F,
                             false);
  };

  const RolloutMetrics blocked = simulate(walled);
  const RolloutMetrics clear = simulate(open);

  EXPECT_NEAR(blocked.contact_distance_m, 3.0F, 1.0e-4F);
  EXPECT_GT(blocked.costs.stopping_deficit_m2_s, 0.0F);
  EXPECT_TRUE(std::isinf(clear.contact_distance_m));
  EXPECT_FLOAT_EQ(clear.costs.stopping_deficit_m2_s, 0.0F);
  EXPECT_GT(blocked.soft_cost, clear.soft_cost);
}

TEST(MppiReferenceTest, PhysicalFootprintAnnotatesAdjacentDerivedZeroClearance) {
  const EsdfGrid grid{4, 4, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(16U, 10.0F);
  esdf[1U * 4U + 2U] = 0.0F;
  const std::array<Control, 1> controls{};
  const std::array<Control, 1> noise{};
  const FootprintConfig footprint{.radius_m = 0.82F,
                                  .lower_extent_m = 0.23F,
                                  .upper_extent_m = 0.35F,
                                  .perimeter_samples = 12U,
                                  .radial_rings = 2U,
                                  .axial_samples = 3U};

  const RolloutMetrics metrics = simulateReference(
      State{1.5F, 1.5F, 0.0F}, controls, noise, DynamicsConfig{}, RiskConfig{},
      CostConfig{}, grid, esdf, 3.0F, 1.5F, false, Control{}, -1.0F, footprint);

  EXPECT_EQ(metrics.worst_tier, RiskTier::kCritical);
}

TEST(MppiReferenceTest, ComputationalBoundaryDoesNotEscalateDerivedRisk) {
  const EsdfGrid grid{4, 4, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(16U, 10.0F);
  const std::array<Control, 1> controls{};
  const std::array<Control, 1> noise{};
  const FootprintConfig footprint{.radius_m = 0.82F,
                                  .lower_extent_m = 0.23F,
                                  .upper_extent_m = 0.35F,
                                  .perimeter_samples = 12U,
                                  .radial_rings = 2U,
                                  .axial_samples = 3U};

  const RolloutMetrics metrics = simulateReference(
      State{0.1F, 1.5F, 0.0F}, controls, noise, DynamicsConfig{}, RiskConfig{},
      CostConfig{}, grid, esdf, 3.0F, 1.5F, false, Control{}, -1.0F, footprint);

  EXPECT_GE(metrics.minimum_clearance_m, 0.0F);
  EXPECT_EQ(metrics.worst_tier, RiskTier::kPreferred);
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

TEST(MppiReferenceTest, SpeedAboveTheReferenceAccruesOverspeedCost) {
  // The reference speed is the speed the policy can stop within: a rollout
  // above it sheds the excess under the overspeed weight, while one below it
  // pays only the symmetric tracking term.
  constexpr int kWidth = 20;
  constexpr int kHeight = 20;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 20.0F);
  const std::array<Control, 2> controls{};
  const std::array<Control, 2> noise{};
  DynamicsConfig dynamics;
  dynamics.linear_drag_1ps = 0.0F;
  dynamics.maximum_horizontal_speed_mps = 10.0F;
  dynamics.maximum_translational_speed_mps = 10.0F;
  const State initial{.x = 1.5F, .y = 1.5F, .vx = 5.0F};

  const RolloutMetrics below =
      simulateReference(initial, controls, noise, dynamics, RiskConfig{}, CostConfig{},
                        grid, esdf, 10.0F, 1.5F, false, Control{}, 8.0F);
  const RolloutMetrics above =
      simulateReference(initial, controls, noise, dynamics, RiskConfig{}, CostConfig{},
                        grid, esdf, 10.0F, 1.5F, false, Control{}, 2.0F);

  EXPECT_FLOAT_EQ(below.costs.overspeed, 0.0F);
  EXPECT_GT(above.costs.overspeed, 0.0F);
  EXPECT_GT(above.costs.speed_tracking, 0.0F);
  EXPECT_GT(above.soft_cost, below.soft_cost);
}

TEST(MppiReferenceTest, SpeedAboveTheDynamicsCapsAccruesOverspeedCost) {
  constexpr int kWidth = 20;
  constexpr int kHeight = 20;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 20.0F);
  const std::array<Control, 2> controls{};
  const std::array<Control, 2> noise{};
  DynamicsConfig dynamics;
  dynamics.linear_drag_1ps = 0.0F;
  dynamics.maximum_horizontal_speed_mps = 10.0F;
  dynamics.maximum_translational_speed_mps = 6.0F;

  // Under both caps: nothing to shed.
  const RolloutMetrics within = simulateReference(
      State{.x = 1.5F, .y = 1.5F, .vx = 5.0F}, controls, noise, dynamics, RiskConfig{},
      CostConfig{}, grid, esdf, 10.0F, 1.5F, false);
  // The inherited speed starts above the translational cap.
  const RolloutMetrics inherited = simulateReference(
      State{.x = 1.5F, .y = 1.5F, .vx = 9.0F}, controls, noise, dynamics, RiskConfig{},
      CostConfig{}, grid, esdf, 10.0F, 1.5F, false);
  // Braking controls shed no faster than the dynamics already do.
  const std::array<Control, 2> braking{Control{.ax = -4.0F}, Control{.ax = -4.0F}};
  const RolloutMetrics shedding = simulateReference(
      State{.x = 1.5F, .y = 1.5F, .vx = 9.0F}, braking, noise, dynamics, RiskConfig{},
      CostConfig{}, grid, esdf, 10.0F, 1.5F, false);

  EXPECT_FLOAT_EQ(within.costs.overspeed, 0.0F);
  EXPECT_GT(inherited.costs.overspeed, 0.0F);
  // The dynamics shed the excess at the maximum deceleration whatever the
  // controls ask, so both rollouts carry the same unavoidable excess.
  EXPECT_FLOAT_EQ(shedding.costs.overspeed, inherited.costs.overspeed);
  EXPECT_GT(inherited.soft_cost, within.soft_cost);
}

TEST(MppiReferenceTest, ReferenceSpeedUsesTheTotalThreeDimensionalVelocity) {
  constexpr int kWidth = 20;
  constexpr int kHeight = 20;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 20.0F);
  const std::array<Control, 2> controls{};
  const std::array<Control, 2> noise{};
  DynamicsConfig dynamics;
  dynamics.linear_drag_1ps = 0.0F;

  const RolloutMetrics vertical_only = simulateReference(
      State{.x = 1.5F, .y = 1.5F, .vz = 5.0F}, controls, noise, dynamics, RiskConfig{},
      CostConfig{}, grid, esdf, 10.0F, 1.5F, false, Control{}, 5.0F);
  const RolloutMetrics horizontal = simulateReference(
      State{.x = 1.5F, .y = 1.5F, .vx = 5.0F, .vz = 5.0F}, controls, noise, dynamics,
      RiskConfig{}, CostConfig{}, grid, esdf, 10.0F, 1.5F, false, Control{}, 5.0F);

  EXPECT_FLOAT_EQ(vertical_only.costs.speed_tracking, 0.0F);
  EXPECT_GT(horizontal.costs.speed_tracking, vertical_only.costs.speed_tracking);
}

TEST(MppiReferenceTest, InheritedSpeedAboveModelLimitShedsOneStepOfBrakingPerStep) {
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.05F;
  dynamics.linear_drag_1ps = 0.0F;
  dynamics.maximum_horizontal_speed_mps = 10.0F;
  dynamics.maximum_vertical_speed_mps = 5.0F;
  dynamics.maximum_translational_speed_mps = 10.0F;
  dynamics.maximum_yaw_rate_radps = 1.5F;

  const State initial{
      .vx = 12.0F,
      .vz = -6.0F,
      .yaw_rate = 2.0F,
  };
  // Without braking controls the excess is still shed by one step of the
  // maximum deceleration, never clamped to the cap at once.
  const State shed = integrateReference(initial, Control{}, dynamics);
  EXPECT_NEAR(shed.vx,
              initial.vx -
                  dynamics.maximum_horizontal_acceleration_mps2 * dynamics.dt_s,
              1.0e-5F);
  EXPECT_NEAR(shed.vz,
              initial.vz + dynamics.maximum_vertical_acceleration_mps2 * dynamics.dt_s,
              1.0e-5F);
  EXPECT_FLOAT_EQ(shed.yaw_rate, initial.yaw_rate);

  const State recovering = integrateReference(
      initial, Control{.ax = -4.0F, .az = 4.0F, .yaw_accel = -2.0F}, dynamics);
  EXPECT_LT(recovering.vx, initial.vx);
  EXPECT_GT(recovering.vz, initial.vz);
  EXPECT_LT(recovering.yaw_rate, initial.yaw_rate);
  EXPECT_GT(recovering.vx, dynamics.maximum_horizontal_speed_mps);
  EXPECT_LT(recovering.vz, -dynamics.maximum_vertical_speed_mps);
  EXPECT_GT(std::hypot(std::hypot(recovering.vx, recovering.vy), recovering.vz),
            dynamics.maximum_translational_speed_mps);
  EXPECT_GT(recovering.yaw_rate, dynamics.maximum_yaw_rate_radps);
}

TEST(MppiReferenceTest, PeerSeparationIsSoftAndTimeIndexed) {
  constexpr int kWidth = 40;
  constexpr int kHeight = 10;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 40.0F);
  const std::array<Control, 4> controls{};
  const std::array<Control, 4> noise{};
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.5F;
  dynamics.linear_drag_1ps = 0.0F;
  const auto near_samples = std::make_shared<const std::vector<DynamicAircraftSample>>(
      std::vector<DynamicAircraftSample>(4U, DynamicAircraftSample{.x = 4.0F}));
  const auto far_samples = std::make_shared<const std::vector<DynamicAircraftSample>>(
      std::vector<DynamicAircraftSample>(4U, DynamicAircraftSample{.x = 20.0F}));
  const std::array near_peer{DynamicAircraftTrajectory{
      .samples = near_samples, .footprint_radius_m = 0.82F, .active_steps = 4U}};
  const std::array far_peer{DynamicAircraftTrajectory{
      .samples = far_samples, .footprint_radius_m = 0.82F, .active_steps = 4U}};
  const State initial{.vx = 2.0F};

  const RolloutMetrics near =
      simulateReference(initial, controls, noise, dynamics, RiskConfig{}, CostConfig{},
                        grid, esdf, 30.0F, 0.0F, false, Control{}, -1.0F,
                        FootprintConfig{}, std::nullopt, nullptr, near_peer);
  const RolloutMetrics far =
      simulateReference(initial, controls, noise, dynamics, RiskConfig{}, CostConfig{},
                        grid, esdf, 30.0F, 0.0F, false, Control{}, -1.0F,
                        FootprintConfig{}, std::nullopt, nullptr, far_peer);

  EXPECT_GT(near.costs.peer_separation, 0.0F);
  EXPECT_LT(near.minimum_peer_separation_m, 5.0F);
  EXPECT_FLOAT_EQ(far.costs.peer_separation, 0.0F);
  EXPECT_GT(near.soft_cost, far.soft_cost);
}

TEST(MppiReferenceTest, NonCooperativeSurvivalCostDominatesInsideTenMeters) {
  constexpr int kWidth = 40;
  constexpr int kHeight = 10;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 40.0F);
  const std::array<Control, 4> controls{};
  const std::array<Control, 4> noise{};
  const auto samples_at = [](const float x_m) {
    return std::make_shared<const std::vector<DynamicAircraftSample>>(
        4U, DynamicAircraftSample{.x = x_m});
  };
  const std::array strong_aircraft{DynamicAircraftTrajectory{
      .samples = samples_at(8.0F), .footprint_radius_m = 0.82F, .active_steps = 4U}};
  const std::array anticipated_aircraft{DynamicAircraftTrajectory{
      .samples = samples_at(15.0F), .footprint_radius_m = 0.82F, .active_steps = 4U}};
  const DynamicAircraftCostPolicy policy{
      .strong_separation_m = 10.0F,
      .anticipation_separation_m = 20.0F,
      .strong_weight = 4000.0F,
      .anticipation_weight = 40.0F,
      .time_to_collision_gain_s = 1.0F,
      .maximum_time_to_collision_multiplier = 4.0F,
  };

  const RolloutMetrics strong = simulateReference(
      State{}, controls, noise, DynamicsConfig{}, RiskConfig{}, CostConfig{}, grid,
      esdf, 30.0F, 0.0F, false, Control{}, -1.0F, FootprintConfig{}, std::nullopt,
      nullptr, strong_aircraft, std::nullopt, CooperativeConfig{}, policy);
  const RolloutMetrics anticipated = simulateReference(
      State{}, controls, noise, DynamicsConfig{}, RiskConfig{}, CostConfig{}, grid,
      esdf, 30.0F, 0.0F, false, Control{}, -1.0F, FootprintConfig{}, std::nullopt,
      nullptr, anticipated_aircraft, std::nullopt, CooperativeConfig{}, policy);

  EXPECT_GT(strong.costs.dynamic_aircraft_survival,
            anticipated.costs.dynamic_aircraft_survival * 10.0F);
  EXPECT_GT(strong.soft_cost, anticipated.soft_cost * 10.0F);
  EXPECT_GT(anticipated.costs.dynamic_aircraft_anticipation, 0.0F);
}

TEST(MppiReferenceTest, NoPeersAddsNoCooperativeCost) {
  const EsdfGrid grid{10, 10, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(100U, 20.0F);
  const std::array<Control, 2> controls{};
  const std::array<Control, 2> noise{};

  const RolloutMetrics metrics =
      simulateReference(State{}, controls, noise, DynamicsConfig{}, RiskConfig{},
                        CostConfig{}, grid, esdf, 5.0F, 0.0F, false);

  EXPECT_FLOAT_EQ(metrics.costs.peer_separation, 0.0F);
  EXPECT_FLOAT_EQ(metrics.costs.maneuver_preference, 0.0F);
  EXPECT_TRUE(std::isinf(metrics.minimum_peer_separation_m));
}

TEST(MppiReferenceTest, MovingTargetUsesClosestApproachInsteadOfTerminalPoint) {
  constexpr int kWidth = 30;
  constexpr int kHeight = 4;
  const EsdfGrid grid{kWidth, kHeight, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(kWidth * kHeight), 30.0F);
  const std::array<Control, 20> controls{};
  const std::array<Control, 20> noise{};
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  dynamics.linear_drag_1ps = 0.0F;
  const State initial{.x = 0.5F, .y = 1.5F, .vx = 10.0F};

  const RolloutMetrics terminal_point =
      simulateReference(initial, controls, noise, dynamics, RiskConfig{}, CostConfig{},
                        grid, esdf, 5.5F, 1.5F, false);
  const RolloutMetrics moving_target = simulateReference(
      initial, controls, noise, dynamics, RiskConfig{}, CostConfig{}, grid, esdf, 5.5F,
      1.5F, false, Control{}, -1.0F, FootprintConfig{},
      MovingTargetReference{.state = State{.x = 5.5F, .y = 1.5F},
                            .capture_radius_m = 0.25F});

  EXPECT_GT(terminal_point.costs.terminal, 10.0F);
  EXPECT_NEAR(moving_target.minimum_target_separation_m, 0.0F, 1.0e-5F);
  EXPECT_NEAR(moving_target.predicted_capture_time_s, 0.5F, 1.0e-5F);
  EXPECT_FLOAT_EQ(moving_target.costs.terminal, 0.0F);
  EXPECT_LT(moving_target.soft_cost, terminal_point.soft_cost);
}

TEST(MppiReferenceTest, MovingTargetDiagnosticsUseDynamicClosestApproach) {
  RolloutMetrics metrics;
  metrics.costs.head_progress = 3.0F;
  metrics.costs.progress = -8.0F;

  const MppiProgressDiagnostics moving =
      resolveUnroutedProgressDiagnostics(metrics, true, -20.0F, -30.0F);
  const MppiProgressDiagnostics fixed =
      resolveUnroutedProgressDiagnostics(metrics, false, -20.0F, -30.0F);

  EXPECT_FLOAT_EQ(moving.head_progress_m, 3.0F);
  EXPECT_FLOAT_EQ(moving.terminal_progress_m, 8.0F);
  EXPECT_FLOAT_EQ(fixed.head_progress_m, -20.0F);
  EXPECT_FLOAT_EQ(fixed.terminal_progress_m, -30.0F);
}

TEST(MppiReferenceTest, MovingTargetVerticalMotionStopsAndRespectsBounds) {
  const MovingTargetReference stopping{
      .state = State{.z = 18.0F, .vz = 4.0F},
      .vertical_deceleration_mps2 = 4.0F,
      .minimum_z_m = 1.0F,
      .maximum_z_m = std::nextafter(32.0F, 1.0F),
      .bounded_vertical_motion = true,
  };
  const MovingTargetReference bounded{
      .state = State{.z = 31.0F, .vz = 10.0F},
      .vertical_deceleration_mps2 = 1.0F,
      .minimum_z_m = 1.0F,
      .maximum_z_m = std::nextafter(32.0F, 1.0F),
      .bounded_vertical_motion = true,
  };

  EXPECT_FLOAT_EQ(movingTargetAltitudeAt(stopping, 10.0F), 20.0F);
  EXPECT_FLOAT_EQ(movingTargetAltitudeAt(bounded, 10.0F), bounded.maximum_z_m);
  EXPECT_LT(movingTargetAltitudeAt(bounded, 10.0F), 32.0F);
}

TEST(MppiReferenceTest, FloatConversionCannotReopenHalfOpenUpperEnvelope) {
  const float maximum_z = std::nextafter(32.0F, 1.0F);
  const float rounded_double_boundary = static_cast<float>(std::nextafter(32.0, 1.0));

  ASSERT_FLOAT_EQ(rounded_double_boundary, 32.0F);
  EXPECT_FLOAT_EQ(clampMovingTargetAltitude(rounded_double_boundary, 1.0F, maximum_z),
                  maximum_z);
  EXPECT_LT(clampMovingTargetAltitude(rounded_double_boundary, 1.0F, maximum_z), 32.0F);
}

TEST(MppiReferenceTest, DetectsAltitudeEnvelopeViolationInsideRollout) {
  const EsdfGrid grid{10, 10, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(100U, 20.0F);
  const std::array<Control, 4> controls{};
  const std::array<Control, 4> noise{};
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.2F;
  dynamics.linear_drag_1ps = 0.0F;

  const RolloutMetrics metrics = simulateReference(
      State{.x = 1.5F, .y = 1.5F, .z = 1.1F, .vz = -1.0F}, controls, noise, dynamics,
      RiskConfig{}, CostConfig{}, grid, esdf, 5.0F, 1.5F, true, Control{}, -1.0F,
      FootprintConfig{}, std::nullopt, nullptr, {}, std::nullopt, CooperativeConfig{},
      std::nullopt, AltitudeEnvelopeConfig{.minimum_z_m = 1.0F, .maximum_z_m = 32.0F});

  EXPECT_TRUE(metrics.altitude_envelope_violation);
}

TEST(MppiReferenceTest, FlightEnvelopeRequiresJerkLimitedVerticalStoppingRoom) {
  DynamicsConfig dynamics;
  dynamics.maximum_vertical_acceleration_mps2 = 4.0F;
  dynamics.maximum_control_jerk_mps3 = 20.0F;
  const AltitudeEnvelopeConfig envelope{.minimum_z_m = 1.0F, .maximum_z_m = 32.0F};

  const State descending{.z = 7.0F, .vz = -5.0F};
  const Control accelerating_down{.az = -4.0F};
  EXPECT_NEAR(verticalStoppingDistanceM(descending.vz, accelerating_down.az, dynamics),
              5.2317F, 1.0e-3F);
  EXPECT_TRUE(altitudeEnvelopeDynamicallyRecoverable(descending, accelerating_down,
                                                     dynamics, envelope));
  EXPECT_FALSE(altitudeEnvelopeDynamicallyRecoverable(
      State{.z = 6.0F, .vz = -5.0F}, accelerating_down, dynamics, envelope));

  EXPECT_TRUE(altitudeEnvelopeDynamicallyRecoverable(
      State{.z = 26.0F, .vz = 5.0F}, Control{.az = 4.0F}, dynamics, envelope));
  EXPECT_FALSE(altitudeEnvelopeDynamicallyRecoverable(
      State{.z = 27.0F, .vz = 5.0F}, Control{.az = 4.0F}, dynamics, envelope));
}

TEST(MppiReferenceTest,
     FlightEnvelopeUsesGuaranteedVerticalStoppingAndReactionLatency) {
  DynamicsConfig dynamics;
  dynamics.maximum_vertical_acceleration_mps2 = 4.0F;
  dynamics.maximum_control_jerk_mps3 = 20.0F;
  const AltitudeEnvelopeConfig envelope{
      .minimum_z_m = 1.0F,
      .maximum_z_m = 32.0F,
      .guaranteed_vertical_deceleration_mps2 = 2.0F,
      .reaction_latency_s = 0.1F,
  };
  const Control accelerating_down{.az = -4.0F};

  EXPECT_NEAR(
      verticalStoppingDistanceM(-5.0F, accelerating_down.az, dynamics, 2.0F, 0.1F),
      10.3525F, 1.0e-3F);
  EXPECT_FALSE(altitudeEnvelopeDynamicallyRecoverable(
      State{.z = 11.0F, .vz = -5.0F}, accelerating_down, dynamics, envelope));
  EXPECT_TRUE(altitudeEnvelopeDynamicallyRecoverable(
      State{.z = 12.0F, .vz = -5.0F}, accelerating_down, dynamics, envelope));
}

} // namespace
} // namespace drone_city_nav::mppi
