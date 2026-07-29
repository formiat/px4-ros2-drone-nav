#include "drone_city_nav/mppi_risk_escalation.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(MppiRiskEscalationTest, EscalatesOnDistinctStallsAndRecoversAfterProgress) {
  MppiRiskEscalation escalation{MppiRiskEscalationConfig{.recovery_stable_cycles = 2U}};

  EXPECT_EQ(escalation.update({.reseed_generation = 1U}).maximum_eligible_tier,
            mppi::RiskTier::kPlanning);
  EXPECT_EQ(escalation.update({.reseed_generation = 2U}).maximum_eligible_tier,
            mppi::RiskTier::kCritical);
  EXPECT_EQ(escalation.update({.reseed_generation = 2U, .stable_progress = true})
                .maximum_eligible_tier,
            mppi::RiskTier::kCritical);
  EXPECT_EQ(escalation.update({.reseed_generation = 2U, .stable_progress = true})
                .maximum_eligible_tier,
            mppi::RiskTier::kPreferred);
}

TEST(MppiRiskEscalationTest, DoesNotCountRepeatedObservationAsAnotherStall) {
  MppiRiskEscalation escalation;

  EXPECT_EQ(escalation.update({.reseed_generation = 1U}).maximum_eligible_tier,
            mppi::RiskTier::kPlanning);
  EXPECT_EQ(escalation.update({.reseed_generation = 1U}).maximum_eligible_tier,
            mppi::RiskTier::kPlanning);
}

} // namespace
} // namespace drone_city_nav
