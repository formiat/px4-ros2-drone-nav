#include <gtest/gtest.h>

#include <cstdint>

#include "production_mppi_execution_control.hpp"

namespace drone_city_nav {
namespace {

constexpr std::int64_t kGraceNs{100'000'000};
constexpr std::int64_t kMaximumAcknowledgementAgeNs{200'000'000};

[[nodiscard]] ExecutionOwnerIdentity3D plannedOwner() {
  return {
      .valid_from_ns = 1'000'000'000,
      .valid_until_ns = 5'000'000'000,
      .producer_instance_id = 17U,
      .target_offboard_instance_id = 23U,
      .sequence = 31U,
      .execution_mode = ExecutionAuthorityMode3D::kPlanned,
      .valid = true,
  };
}

[[nodiscard]] ProductionMppiHorizonAcknowledgement
acknowledgementOf(const std::uint64_t sequence, const std::int64_t receive_stamp_ns) {
  return {
      .offboard_producer_instance_id = 23U,
      .horizon_producer_instance_id = 17U,
      .horizon_sequence = sequence,
      .source_stamp_ns = receive_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .valid = true,
  };
}

[[nodiscard]] ProductionMppiHorizonSupersessionCheck
check(const ExecutionOwnerIdentity3D& owner,
      const ProductionMppiHorizonAcknowledgement& acknowledgement,
      const std::int64_t now_ns, const bool witnessed = false,
      const std::int64_t publication_stamp_ns = 0) {
  return {
      .owner = &owner,
      .acknowledgement = &acknowledgement,
      .owner_witnessed = witnessed,
      .now_ns = now_ns,
      .owner_publication_stamp_ns = publication_stamp_ns,
      .acknowledgement_grace_ns = kGraceNs,
      .maximum_acknowledgement_age_ns = kMaximumAcknowledgementAgeNs,
  };
}

TEST(ProductionMppiExecutionControlTest, AllowsPublicationWithoutPlannedOwner) {
  const ProductionMppiHorizonAcknowledgement none{};
  const ExecutionOwnerIdentity3D empty{};
  EXPECT_EQ(assessPlannedHorizonSupersession(check(empty, none, 1'500'000'000)),
            ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner);

  ExecutionOwnerIdentity3D hold = plannedOwner();
  hold.execution_mode = ExecutionAuthorityMode3D::kPositionHold;
  EXPECT_EQ(assessPlannedHorizonSupersession(check(hold, none, 1'500'000'000)),
            ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner);
}

TEST(ProductionMppiExecutionControlTest, RejectsOwnerOutsideItsLease) {
  const ExecutionOwnerIdentity3D owner = plannedOwner();
  const ProductionMppiHorizonAcknowledgement acknowledged = acknowledgementOf(31U, 999);
  EXPECT_EQ(assessPlannedHorizonSupersession(check(owner, acknowledged, 999'999'999)),
            ProductionMppiHorizonSupersessionDecision::kRejectedOwnerNotCurrent);
  EXPECT_EQ(
      assessPlannedHorizonSupersession(check(owner, acknowledged, 5'000'000'000, true)),
      ProductionMppiHorizonSupersessionDecision::kRejectedOwnerNotCurrent);
}

TEST(ProductionMppiExecutionControlTest, AllowsExactlyWitnessedOwnerImmediately) {
  const ExecutionOwnerIdentity3D owner = plannedOwner();
  const ProductionMppiHorizonAcknowledgement none{};
  EXPECT_EQ(assessPlannedHorizonSupersession(check(owner, none, 1'010'000'000, true)),
            ProductionMppiHorizonSupersessionDecision::kAllowedWitnessedOwner);
}

TEST(ProductionMppiExecutionControlTest,
     AllowsReplacementWhenControllerAcknowledgedThisOrPreviousLease) {
  const ExecutionOwnerIdentity3D owner = plannedOwner();
  const std::int64_t now_ns = 1'010'000'000;
  EXPECT_EQ(assessPlannedHorizonSupersession(
                check(owner, acknowledgementOf(31U, now_ns - 20'000'000), now_ns)),
            ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgedPredecessor);
  EXPECT_EQ(assessPlannedHorizonSupersession(
                check(owner, acknowledgementOf(30U, now_ns - 20'000'000), now_ns)),
            ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgedPredecessor);
}

TEST(ProductionMppiExecutionControlTest,
     DefersWhileControllerIsTwoLeasesBehindInsideGrace) {
  const ExecutionOwnerIdentity3D owner = plannedOwner();
  const std::int64_t now_ns = 1'010'000'000;
  EXPECT_EQ(
      assessPlannedHorizonSupersession(
          check(owner, acknowledgementOf(29U, now_ns - 20'000'000), now_ns)),
      ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingAcknowledgement);
}

TEST(ProductionMppiExecutionControlTest, IgnoresStaleOrForeignAcknowledgements) {
  const ExecutionOwnerIdentity3D owner = plannedOwner();
  const std::int64_t now_ns = 1'010'000'000;
  EXPECT_EQ(
      assessPlannedHorizonSupersession(check(
          owner, acknowledgementOf(31U, now_ns - kMaximumAcknowledgementAgeNs - 1),
          now_ns)),
      ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingAcknowledgement);
  ProductionMppiHorizonAcknowledgement foreign =
      acknowledgementOf(31U, now_ns - 20'000'000);
  foreign.offboard_producer_instance_id = 99U;
  EXPECT_EQ(
      assessPlannedHorizonSupersession(check(owner, foreign, now_ns)),
      ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingAcknowledgement);
  ProductionMppiHorizonAcknowledgement other_planner =
      acknowledgementOf(31U, now_ns - 20'000'000);
  other_planner.horizon_producer_instance_id = 5U;
  EXPECT_EQ(
      assessPlannedHorizonSupersession(check(owner, other_planner, now_ns)),
      ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingAcknowledgement);
}

TEST(ProductionMppiExecutionControlTest,
     ReplacesUnacknowledgedOwnerAfterGraceMeasuredFromPublication) {
  const ExecutionOwnerIdentity3D owner = plannedOwner();
  const ProductionMppiHorizonAcknowledgement none{};
  const std::int64_t published_ns = 1'400'000'000;
  EXPECT_EQ(
      assessPlannedHorizonSupersession(
          check(owner, none, published_ns + kGraceNs - 1, false, published_ns)),
      ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingAcknowledgement);
  EXPECT_EQ(
      assessPlannedHorizonSupersession(
          check(owner, none, published_ns + kGraceNs, false, published_ns)),
      ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgementGraceElapsed);
  // Without a publication record the grace falls back to the lease start.
  EXPECT_EQ(
      assessPlannedHorizonSupersession(
          check(owner, none, owner.valid_from_ns + kGraceNs)),
      ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgementGraceElapsed);
}

TEST(ProductionMppiExecutionControlTest, ClassifiesAllowedDecisions) {
  EXPECT_TRUE(horizonSupersessionAllowed(
      ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner));
  EXPECT_TRUE(horizonSupersessionAllowed(
      ProductionMppiHorizonSupersessionDecision::kAllowedWitnessedOwner));
  EXPECT_TRUE(horizonSupersessionAllowed(
      ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgedPredecessor));
  EXPECT_TRUE(horizonSupersessionAllowed(
      ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgementGraceElapsed));
  EXPECT_FALSE(horizonSupersessionAllowed(
      ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingAcknowledgement));
  EXPECT_FALSE(horizonSupersessionAllowed(
      ProductionMppiHorizonSupersessionDecision::kRejectedOwnerNotCurrent));
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

} // namespace
} // namespace drone_city_nav
