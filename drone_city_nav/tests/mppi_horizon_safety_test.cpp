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

TEST(MppiHorizonSafetyTest, DelaysInterventionForDistantCollision) {
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

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kExecuteUntilDeadline);
  EXPECT_GT(result.latest_safe_intervention_time_s, 0.0);
  ASSERT_FALSE(result.fallback_horizon.empty());
  EXPECT_LT(result.fallback_horizon.back().vx, current.vx);
}

TEST(MppiHorizonSafetyTest, KeepsEarliestInterventionDeadlineAcrossTicks) {
  MppiSafetyInterventionTracker tracker;
  MppiHorizonSafetyResult first;
  first.decision = MppiHorizonSafetyDecision::kExecuteUntilDeadline;
  first.latest_safe_intervention_time_s = 1.0;
  const MppiSafetyInterventionUpdate first_update =
      tracker.update(1'000'000'000, first);

  MppiHorizonSafetyResult later = first;
  later.latest_safe_intervention_time_s = 2.0;
  const MppiSafetyInterventionUpdate later_update =
      tracker.update(1'500'000'000, later);
  const MppiSafetyInterventionUpdate expired = tracker.update(2'000'000'000, later);

  ASSERT_TRUE(first_update.deadline_ns.has_value());
  EXPECT_EQ(later_update.deadline_ns, first_update.deadline_ns);
  EXPECT_EQ(later_update.decision, MppiHorizonSafetyDecision::kExecuteUntilDeadline);
  EXPECT_EQ(expired.decision, MppiHorizonSafetyDecision::kBrake);
}

TEST(MppiHorizonSafetyTest, ProducesFallbackForEngineOnlyCollision) {
  const mppi::EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(400U, 10.0F);
  const std::vector<mppi::State> horizon{mppi::State{1.0F, 1.0F},
                                         mppi::State{2.0F, 1.0F}};
  mppi::State current{1.0F, 1.0F};
  current.vx = 2.0F;

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, esdf, grid, MppiHorizonSafetyConfig{}, true);

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kBrake);
  ASSERT_FALSE(result.fallback_controls.empty());
  ASSERT_EQ(result.fallback_horizon.size(), result.fallback_controls.size() + 1U);
  EXPECT_LT(result.fallback_horizon.back().vx, current.vx);
}

TEST(MppiHorizonSafetyTest, RejectsHorizonIntersectingKnownThreeDimensionalSolid) {
  const mppi::EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(400U, 10.0F);
  const std::vector<mppi::State> horizon{
      mppi::State{1.0F, 1.0F, 5.0F},
      mppi::State{5.0F, 5.0F, 5.0F},
  };
  const std::vector<mppi::KnownSolid> solids{
      mppi::KnownSolid{
          .center_x_m = 5.0F,
          .center_y_m = 5.0F,
          .normal_x = 1.0F,
          .normal_y = 0.0F,
          .lateral_x = 0.0F,
          .lateral_y = 1.0F,
          .half_depth_m = 1.0F,
          .half_width_m = 1.0F,
          .min_z_m = 4.0F,
          .max_z_m = 6.0F,
      },
  };

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      horizon.front(), horizon, esdf, grid, MppiHorizonSafetyConfig{}, false, solids);

  EXPECT_NE(result.decision, MppiHorizonSafetyDecision::kExecute);
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

TEST(MppiHorizonSafetyTest, SweptValidationFindsCollisionBetweenHorizonStates) {
  const mppi::EsdfGrid grid{5, 1, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf{2.0F, 1.0F, 0.0F, 1.0F, 2.0F};
  const mppi::State current{0.5F, 0.5F};
  const std::vector<mppi::State> horizon{current, mppi::State{4.5F, 0.5F}};

  const MppiHorizonSafetyResult result =
      evaluateMppiHorizonSafety(current, horizon, esdf, grid,
                                MppiHorizonSafetyConfig{
                                    .collision_radius_m = 0.1,
                                    .minimum_time_to_collision_s = 0.01,
                                    .dt_s = 1.0,
                                    .swept_validation_step_m = 0.25,
                                });

  EXPECT_NE(result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_GT(result.time_to_collision_s, 0.0);
  EXPECT_LT(result.time_to_collision_s, 1.0);
}

TEST(MppiHorizonSafetyTest, BrakingLifecycleLatchesPositionAfterVehicleSlows) {
  MppiBrakeHoldLifecycle lifecycle;
  mppi::State moving{2.0F, 3.0F, 4.0F};
  moving.vx = 1.0F;
  EXPECT_FALSE(lifecycle.update(true, moving, 0.2).position_hold);

  mppi::State stopped = moving;
  stopped.x = 2.5F;
  stopped.vx = 0.1F;
  const MppiBrakeHoldUpdate captured = lifecycle.update(true, stopped, 0.2);
  ASSERT_TRUE(captured.position_hold);
  EXPECT_FLOAT_EQ(captured.hold_state.x, 2.5F);

  stopped.x = 2.7F;
  const MppiBrakeHoldUpdate retained = lifecycle.update(true, stopped, 0.2);
  EXPECT_FLOAT_EQ(retained.hold_state.x, 2.5F);
  EXPECT_FALSE(lifecycle.update(false, stopped, 0.2).position_hold);
}

} // namespace
} // namespace drone_city_nav
