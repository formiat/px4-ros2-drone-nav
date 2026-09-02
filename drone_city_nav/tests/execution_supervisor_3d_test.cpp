#include "drone_city_nav/execution_supervisor_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "execution_route_snapshot_3d_plan_test_support.hpp"
#include "execution_supervisor_horizon_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] ExecutionRouteTransitionResult3D
supervisorActivation(const ExecutionPlan3D& initial,
                     const CertifiedRouteSuffix3D& route) {
  return activateCertifiedRoute3D(
      initial, initial.version, route,
      SnapshotFixture3D::finiteExecutionForRoute(
          initial, route, FiniteExecutionKind3D::kNominal, true, 100U));
}

[[nodiscard]] PendingCertifiedRoute3D
supervisorPendingDraft(const ExecutionPlan3D& initial,
                       const CertifiedRouteSuffix3D& route) {
  return PendingCertifiedRoute3D{
      .publication_sequence = 0U,
      .base_execution_owner_epoch = initial.execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kEmpty,
      .base_route_generation = 0U,
      .base_geometry_revision = 0U,
      .base_continuity_id = 0U,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = std::nullopt,
      .route = route,
  };
}

TEST(ExecutionSupervisor3DTest,
     CommitsTransitionUnchangedLeaseAndControlAsCompleteAuthorities) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_TRUE(route.has_value());
  const CertifiedRouteSuffix3D& certified =
      route.value(); // NOLINT(bugprone-unchecked-optional-access)
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      supervisor.authority();
  ASSERT_NE(initial_authority, nullptr);
  const std::shared_ptr<const ExecutionPlan3D> initial = initial_authority->plan();
  ASSERT_NE(initial, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      supervisorActivation(*initial, certified);
  ASSERT_TRUE(transition.applied());
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      SnapshotFixture3D::committedInput(*transition.next);
  const ExecutionOwnerIdentity3D owner =
      SnapshotFixture3D::committedOwner(*transition.next, 7U);

  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kTransition,
                    .expected_authority = initial_authority,
                    .expected_plan = initial,
                    .transition = transition,
                    .expected_pending = nullptr,
                    .owner = owner,
                    .input = input,
                })
                .status,
            ExecutionHorizonCommitStatus3D::kCommitted);
  const std::shared_ptr<const CommittedExecutionAuthority3D> active =
      supervisor.authority();
  ASSERT_NE(active, nullptr);
  EXPECT_EQ(active->plan(), transition.next);
  EXPECT_EQ(active->input(), input);
  EXPECT_EQ(active->owner().sequence, 7U);
  EXPECT_TRUE(active->control().empty());

  ExecutionOwnerIdentity3D refreshed_owner = owner;
  refreshed_owner.sequence = 8U;
  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kUnchangedPlan,
                    .expected_authority = active,
                    .expected_plan = transition.next,
                    .transition = std::nullopt,
                    .expected_pending = nullptr,
                    .owner = refreshed_owner,
                    .input = input,
                })
                .status,
            ExecutionHorizonCommitStatus3D::kCommitted);
  const std::shared_ptr<const CommittedExecutionAuthority3D> refreshed =
      supervisor.authority();
  ASSERT_NE(refreshed, nullptr);
  EXPECT_EQ(refreshed->plan(), transition.next);
  EXPECT_EQ(refreshed->owner().sequence, 8U);

  const AppliedControlEvidence3D control =
      SnapshotFixture3D::committedControl(refreshed_owner);
  EXPECT_TRUE(supervisor.publishAppliedControlIfSame(refreshed, control));
  const std::shared_ptr<const CommittedExecutionAuthority3D> controlled =
      supervisor.authority();
  ASSERT_NE(controlled, nullptr);
  EXPECT_TRUE(controlled->control().validFor(refreshed_owner));
  EXPECT_TRUE(supervisor.clearAppliedControlIfSame(controlled));
  const std::shared_ptr<const CommittedExecutionAuthority3D> control_cleared =
      supervisor.authority();
  ASSERT_NE(control_cleared, nullptr);
  EXPECT_TRUE(control_cleared->control().empty());
  EXPECT_TRUE(supervisor.clearLeaseIfSame(control_cleared));
  const std::shared_ptr<const CommittedExecutionAuthority3D> lease_cleared =
      supervisor.authority();
  ASSERT_NE(lease_cleared, nullptr);
  EXPECT_TRUE(lease_cleared->owner().empty());
  EXPECT_EQ(lease_cleared->plan(), transition.next);

  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kTransition,
                    .expected_authority = initial_authority,
                    .expected_plan = initial,
                    .transition = transition,
                    .expected_pending = nullptr,
                    .owner = owner,
                    .input = input,
                })
                .status,
            ExecutionHorizonCommitStatus3D::kAuthorityNotCurrent);
}

TEST(ExecutionSupervisor3DTest,
     PendingLeaseCommitConsumesOnlyTheExactPendingAndAuthority) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_TRUE(route.has_value());
  const CertifiedRouteSuffix3D& certified =
      route.value(); // NOLINT(bugprone-unchecked-optional-access)
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      supervisor.authority();
  ASSERT_NE(initial_authority, nullptr);
  const std::shared_ptr<const ExecutionPlan3D> initial = initial_authority->plan();
  ASSERT_NE(initial, nullptr);
  const PendingRoutePublicationResult3D pending =
      supervisor.publishPendingForCurrentBase(
          initial, supervisorPendingDraft(*initial, certified));
  ASSERT_TRUE(pending.published());
  ASSERT_EQ(supervisor.pending(), pending.pending);
  const ExecutionRouteTransitionResult3D transition =
      supervisorActivation(*initial, certified);
  ASSERT_TRUE(transition.applied());

  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kPendingTransition,
                    .expected_authority = initial_authority,
                    .expected_plan = initial,
                    .transition = transition,
                    .expected_pending = pending.pending,
                    .owner = SnapshotFixture3D::committedOwner(*transition.next),
                    .input = SnapshotFixture3D::committedInput(*transition.next),
                })
                .status,
            ExecutionHorizonCommitStatus3D::kCommitted);
  EXPECT_EQ(supervisor.plan(), transition.next);
  EXPECT_EQ(supervisor.pending(), nullptr);
  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kPendingTransition,
                    .expected_authority = initial_authority,
                    .expected_plan = initial,
                    .transition = transition,
                    .expected_pending = pending.pending,
                    .owner = SnapshotFixture3D::committedOwner(*transition.next),
                    .input = SnapshotFixture3D::committedInput(*transition.next),
                })
                .status,
            ExecutionHorizonCommitStatus3D::kAuthorityNotCurrent);
}

TEST(ExecutionSupervisor3DTest,
     RejectsMalformedLeaseTransactionsWithoutMutatingAuthority) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_TRUE(route.has_value());
  const CertifiedRouteSuffix3D& certified =
      route.value(); // NOLINT(bugprone-unchecked-optional-access)
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      supervisor.authority();
  ASSERT_NE(initial_authority, nullptr);
  const std::shared_ptr<const ExecutionPlan3D> initial = initial_authority->plan();
  ASSERT_NE(initial, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      supervisorActivation(*initial, certified);
  ASSERT_TRUE(transition.applied());
  const ExecutionOwnerIdentity3D owner =
      SnapshotFixture3D::committedOwner(*transition.next);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      SnapshotFixture3D::committedInput(*transition.next);

  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kTransition,
                    .expected_authority = initial_authority,
                    .expected_plan = initial,
                    .transition = std::nullopt,
                    .expected_pending = nullptr,
                    .owner = owner,
                    .input = input,
                })
                .status,
            ExecutionHorizonCommitStatus3D::kInvalidRequest);
  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kUnchangedPlan,
                    .expected_authority = initial_authority,
                    .expected_plan = initial,
                    .transition = transition,
                    .expected_pending = nullptr,
                    .owner = owner,
                    .input = input,
                })
                .status,
            ExecutionHorizonCommitStatus3D::kInvalidRequest);
  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kPendingTransition,
                    .expected_authority = initial_authority,
                    .expected_plan = initial,
                    .transition = transition,
                    .expected_pending = nullptr,
                    .owner = owner,
                    .input = input,
                })
                .status,
            ExecutionHorizonCommitStatus3D::kInvalidRequest);
  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kTransition,
                    .expected_authority = initial_authority,
                    .expected_plan = transition.next,
                    .transition = transition,
                    .expected_pending = nullptr,
                    .owner = owner,
                    .input = input,
                })
                .status,
            ExecutionHorizonCommitStatus3D::kAuthorityNotCurrent);
  EXPECT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kTransition,
                    .expected_authority = initial_authority,
                    .expected_plan = initial,
                    .transition = transition,
                    .expected_pending = nullptr,
                    .owner = owner,
                    .input = nullptr,
                })
                .status,
            ExecutionHorizonCommitStatus3D::kInvalidRequest);
  EXPECT_EQ(supervisor.authority(), initial_authority);
  EXPECT_EQ(supervisor.pending(), nullptr);
}

TEST(ExecutionSupervisor3DTest, HoldFeedbackOfTheSameHoldIsCurrentPreviousControl) {
  ExecutionOwnerIdentity3D owner;
  owner.valid = true;
  owner.valid_from_ns = 1'000'000'000;
  owner.valid_until_ns = 4'000'000'000;
  owner.producer_instance_id = 7U;
  owner.target_offboard_instance_id = 9U;
  owner.sequence = 42U;
  owner.execution_mode = ExecutionAuthorityMode3D::kPositionHold;

  AppliedControlEvidence3D feedback;
  feedback.valid = true;
  feedback.control_authoritative = false;
  feedback.producer_instance_id = 9U;
  feedback.horizon_producer_instance_id = 7U;
  feedback.horizon_sequence = 42U;
  feedback.execution_mode = ExecutionAuthorityMode3D::kPositionHold;
  feedback.source_stamp_ns = 1'900'000'000;
  feedback.receive_stamp_ns = 1'910'000'000;

  const std::int64_t now_ns = 1'950'000'000;
  EXPECT_TRUE(appliedControlCurrentForExecutionInput3D(feedback, owner, now_ns, 200.0));

  // Hold feedback of a superseded hold is not the owner's.
  AppliedControlEvidence3D previous_hold = feedback;
  previous_hold.horizon_sequence = 41U;
  EXPECT_FALSE(
      appliedControlCurrentForExecutionInput3D(previous_hold, owner, now_ns, 200.0));

  // Planned feedback does not describe a hold owner, nor does hold feedback
  // describe a planned owner.
  AppliedControlEvidence3D planned_feedback = feedback;
  planned_feedback.execution_mode = ExecutionAuthorityMode3D::kPlanned;
  planned_feedback.control_authoritative = true;
  EXPECT_FALSE(
      appliedControlCurrentForExecutionInput3D(planned_feedback, owner, now_ns, 200.0));
  ExecutionOwnerIdentity3D planned_owner = owner;
  planned_owner.execution_mode = ExecutionAuthorityMode3D::kPlanned;
  EXPECT_FALSE(
      appliedControlCurrentForExecutionInput3D(feedback, planned_owner, now_ns, 200.0));
  EXPECT_TRUE(appliedControlCurrentForExecutionInput3D(planned_feedback, planned_owner,
                                                       now_ns, 200.0));
}

TEST(ExecutionSupervisor3DTest, ConcurrentLeaseCommitsHaveOneLinearizationWinner) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_TRUE(route.has_value());
  const CertifiedRouteSuffix3D& certified =
      route.value(); // NOLINT(bugprone-unchecked-optional-access)
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      supervisor.authority();
  ASSERT_NE(initial_authority, nullptr);
  const std::shared_ptr<const ExecutionPlan3D> initial = initial_authority->plan();
  ASSERT_NE(initial, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      supervisorActivation(*initial, certified);
  ASSERT_TRUE(transition.applied());
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      SnapshotFixture3D::committedInput(*transition.next);
  constexpr std::size_t kThreadCount{8U};
  std::array<ExecutionHorizonCommitStatus3D, kThreadCount> statuses{};
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);
  for (std::size_t index = 0U; index < kThreadCount; ++index) {
    threads.emplace_back([&, index] {
      statuses.at(index) = commitExecutionHorizonForTest(
                               supervisor,
                               ExecutionHorizonTestTransaction3D{
                                   .kind = ExecutionHorizonCommitKind3D::kTransition,
                                   .expected_authority = initial_authority,
                                   .expected_plan = initial,
                                   .transition = transition,
                                   .expected_pending = nullptr,
                                   .owner = SnapshotFixture3D::committedOwner(
                                       *transition.next, index + 1U),
                                   .input = input,
                               })
                               .status;
    });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(std::ranges::count(statuses, ExecutionHorizonCommitStatus3D::kCommitted),
            1);
  EXPECT_EQ(
      std::ranges::count(statuses,
                         ExecutionHorizonCommitStatus3D::kAuthorityNotCurrent) +
          std::ranges::count(statuses, ExecutionHorizonCommitStatus3D::kLeaseRejected),
      kThreadCount - 1U);
  EXPECT_EQ(supervisor.plan(), transition.next);
}

} // namespace
} // namespace drone_city_nav
