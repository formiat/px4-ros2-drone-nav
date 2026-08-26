#include <gtest/gtest.h>

#include <cstdint>

#include "production_mppi_execution_control.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] ProductionMppiExecutionHorizonOwner plannedOwner() {
  return {
      .valid_from_ns = 1'000,
      .valid_until_ns = 2'000,
      .producer_instance_id = 17U,
      .target_offboard_instance_id = 23U,
      .sequence = 31U,
      .execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED,
      .valid = true,
  };
}

TEST(ProductionMppiExecutionControlTest, AllowsPublicationWithoutPlannedOwner) {
  EXPECT_EQ(assessPlannedHorizonSupersession({}, false, 1'500),
            ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner);

  ProductionMppiExecutionHorizonOwner hold = plannedOwner();
  hold.execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD;
  EXPECT_EQ(assessPlannedHorizonSupersession(hold, false, 1'500),
            ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner);
}

TEST(ProductionMppiExecutionControlTest, DefersCurrentUnwitnessedOwner) {
  EXPECT_EQ(assessPlannedHorizonSupersession(plannedOwner(), false, 1'500),
            ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingOwnerWitness);
}

TEST(ProductionMppiExecutionControlTest, AllowsCurrentWitnessedOwner) {
  EXPECT_EQ(assessPlannedHorizonSupersession(plannedOwner(), true, 1'500),
            ProductionMppiHorizonSupersessionDecision::kAllowedWitnessedOwner);
}

TEST(ProductionMppiExecutionControlTest, RejectsOwnerOutsideItsLease) {
  EXPECT_EQ(assessPlannedHorizonSupersession(plannedOwner(), false, 999),
            ProductionMppiHorizonSupersessionDecision::kRejectedOwnerNotCurrent);
  EXPECT_EQ(assessPlannedHorizonSupersession(plannedOwner(), true, 2'000),
            ProductionMppiHorizonSupersessionDecision::kRejectedOwnerNotCurrent);
}

} // namespace
} // namespace drone_city_nav
