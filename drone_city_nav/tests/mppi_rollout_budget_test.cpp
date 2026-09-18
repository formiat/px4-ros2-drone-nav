#include "drone_city_nav/mppi_rollout_budget.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] MppiRolloutBudgetObservation safeStaticObservation() {
  return MppiRolloutBudgetObservation{
      .static_world = true,
      .route_available = true,
      .clearance_valid = true,
      .clearance_m = 12.0F,
      .world_age_ms = 0.0,
      .required_risk_tier = mppi::RiskTier::kPreferred,
  };
}

TEST(MppiRolloutBudgetTest, ReducesOpenStaticWork) {
  const MppiRolloutBudgetConfig config;
  MppiRolloutBudgetObservation observation = safeStaticObservation();

  MppiRolloutBudgetDecision decision = selectMppiRolloutBudget(config, observation);
  EXPECT_EQ(decision.active_rollouts, config.open_static_rollouts);
  EXPECT_EQ(decision.reason, MppiRolloutBudgetReason::kReducedOpenStatic);
}

TEST(MppiRolloutBudgetTest, KeepsFullBudgetForRiskOrLowClearance) {
  const MppiRolloutBudgetConfig config;
  MppiRolloutBudgetObservation observation = safeStaticObservation();
  observation.required_risk_tier = mppi::RiskTier::kPlanning;
  MppiRolloutBudgetDecision decision = selectMppiRolloutBudget(config, observation);
  EXPECT_EQ(decision.active_rollouts, config.full_rollouts);
  EXPECT_EQ(decision.reason, MppiRolloutBudgetReason::kFullElevatedRisk);

  observation.required_risk_tier = mppi::RiskTier::kPreferred;
  observation.clearance_m = config.minimum_reduced_clearance_m - 0.1F;
  decision = selectMppiRolloutBudget(config, observation);
  EXPECT_EQ(decision.active_rollouts, config.full_rollouts);
  EXPECT_EQ(decision.reason, MppiRolloutBudgetReason::kFullLowClearance);
}

TEST(MppiRolloutBudgetTest, KeepsFullBudgetForAnUncertainWorld) {
  const MppiRolloutBudgetConfig config;
  MppiRolloutBudgetObservation observation = safeStaticObservation();
  observation.world_age_ms = config.maximum_world_age_ms + 1.0;
  EXPECT_EQ(selectMppiRolloutBudget(config, observation).reason,
            MppiRolloutBudgetReason::kFullWorldUncertain);
}

TEST(MppiRolloutBudgetTest, NoStaticExplorationRetainsFullBudget) {
  const MppiRolloutBudgetConfig config;
  MppiRolloutBudgetObservation observation = safeStaticObservation();
  observation.static_world = false;

  const MppiRolloutBudgetDecision decision =
      selectMppiRolloutBudget(config, observation);

  EXPECT_EQ(decision.active_rollouts, config.full_rollouts);
  EXPECT_EQ(decision.reason, MppiRolloutBudgetReason::kFullNoStaticExploration);
}

} // namespace
} // namespace drone_city_nav
