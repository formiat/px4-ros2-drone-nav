#include <gtest/gtest.h>

#include <cstdint>

#include "production_mppi_execution_control.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] ExecutionOwnerIdentity3D plannedOwner() {
  return {
      .valid_from_ns = 1'000,
      .valid_until_ns = 2'000,
      .producer_instance_id = 17U,
      .target_offboard_instance_id = 23U,
      .sequence = 31U,
      .execution_mode = ExecutionAuthorityMode3D::kPlanned,
      .valid = true,
  };
}

TEST(ProductionMppiExecutionControlTest, AllowsPublicationWithoutPlannedOwner) {
  EXPECT_EQ(assessPlannedHorizonSupersession({}, false, 1'500),
            ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner);

  ExecutionOwnerIdentity3D hold = plannedOwner();
  hold.execution_mode = ExecutionAuthorityMode3D::kPositionHold;
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
                ProductionMppiPhysicalTrajectoryAuthority::kResidentOwner),
            ProductionMppiPhysicalCollisionAction::kRequestRouteSuccessor);
}

TEST(ProductionMppiExecutionControlTest,
     AllowsPhysicalRevocationAndGatesNonphysicalRevocationByPolicy) {
  EXPECT_TRUE(executionRevocationAllowed(true, false));
  EXPECT_TRUE(executionRevocationAllowed(true, true));
  EXPECT_TRUE(executionRevocationAllowed(false, true));
  EXPECT_FALSE(executionRevocationAllowed(false, false));
}

TEST(ProductionMppiExecutionControlTest,
     DistinguishesBackgroundRouteRepairFromFiniteExecutionInvalidation) {
  EXPECT_EQ(residentObstacleDisposition({}),
            ProductionMppiResidentObstacleDisposition::kClear);
  EXPECT_EQ(residentObstacleDisposition({.route_suffix_persistent_raw = true}),
            ProductionMppiResidentObstacleDisposition::kRouteSuffixReplacementRequired);
  EXPECT_EQ(residentObstacleDisposition({.finite_execution_latest_lidar = true}),
            ProductionMppiResidentObstacleDisposition::
                kLatestLidarFiniteExecutionInvalidated);
  EXPECT_EQ(residentObstacleDisposition({
                .route_suffix_persistent_raw = true,
                .finite_execution_persistent_raw = true,
                .finite_execution_latest_lidar = true,
            }),
            ProductionMppiResidentObstacleDisposition::
                kPersistentRawFiniteExecutionInvalidated);
}

TEST(ProductionMppiExecutionControlTest,
     ContinuesExactWitnessedResidentForRetainedRetry) {
  const ExecutionOwnerIdentity3D owner = plannedOwner();
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
  const ExecutionOwnerIdentity3D owner = plannedOwner();
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
