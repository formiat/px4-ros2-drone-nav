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

TEST(ProductionMppiExecutionControlTest,
     RequestsRouteSuccessorOnlyForResidentExecutionCollision) {
  EXPECT_EQ(physicalCollisionAction(
                ProductionMppiPhysicalTrajectoryAuthority::kUnownedCandidate),
            ProductionMppiPhysicalCollisionAction::kRejectCandidate);
  EXPECT_EQ(physicalCollisionAction(
                ProductionMppiPhysicalTrajectoryAuthority::kResidentExecutionOwner),
            ProductionMppiPhysicalCollisionAction::kRequestRouteSuccessor);
}

TEST(ProductionMppiExecutionControlTest,
     ContinuesExactWitnessedResidentForRetainedRetry) {
  const ProductionMppiExecutionHorizonOwner owner = plannedOwner();
  EXPECT_TRUE(
      canContinueResidentPlannedOwner(ProductionMppiResidentOwnerContinuationCheck{
          .owner = &owner,
          .now_ns = 1'500,
          .retained_candidate = true,
          .exact_snapshot_current = true,
          .execution_owner_matches = true,
          .revocation_pending = false,
          .owner_witnessed = true,
      }));
}

TEST(ProductionMppiExecutionControlTest,
     RejectsResidentContinuationWithoutEveryAuthorityWitness) {
  const ProductionMppiExecutionHorizonOwner owner = plannedOwner();
  ProductionMppiResidentOwnerContinuationCheck check{
      .owner = &owner,
      .now_ns = 1'500,
      .retained_candidate = true,
      .exact_snapshot_current = true,
      .execution_owner_matches = true,
      .revocation_pending = false,
      .owner_witnessed = true,
  };

  check.retained_candidate = false;
  EXPECT_FALSE(canContinueResidentPlannedOwner(check));
  check.retained_candidate = true;
  check.exact_snapshot_current = false;
  EXPECT_FALSE(canContinueResidentPlannedOwner(check));
  check.exact_snapshot_current = true;
  check.execution_owner_matches = false;
  EXPECT_FALSE(canContinueResidentPlannedOwner(check));
  check.execution_owner_matches = true;
  check.revocation_pending = true;
  EXPECT_FALSE(canContinueResidentPlannedOwner(check));
  check.revocation_pending = false;
  check.owner_witnessed = false;
  EXPECT_FALSE(canContinueResidentPlannedOwner(check));
  check.owner_witnessed = true;
  check.now_ns = owner.valid_until_ns;
  EXPECT_FALSE(canContinueResidentPlannedOwner(check));
}

} // namespace
} // namespace drone_city_nav
