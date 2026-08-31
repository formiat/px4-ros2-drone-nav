#include <atomic>
#include <cstddef>
#include <memory>
#include <thread>

#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] ExecutionRouteTransitionResult3D
activeTransition(const ExecutionPlan3D& initial, const CertifiedRouteSuffix3D& suffix,
                 const std::uint64_t trajectory_revision = 100U) {
  return activateCertifiedRoute3D(
      initial, initial.version, suffix,
      SnapshotFixture3D::finiteExecutionForRoute(
          initial, suffix, FiniteExecutionKind3D::kNominal, true, trajectory_revision));
}

TEST(CommittedExecutionAuthority3DTest,
     LeasedTransitionPublishesPlanOwnerExactInputAndEmptyControlTogether) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const CertifiedRouteSuffix3D& certified =
      suffix.value(); // NOLINT(bugprone-unchecked-optional-access)
  RouteExecutionManager3D manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      manager.authority();
  ASSERT_NE(initial_authority, nullptr);
  ASSERT_TRUE(initial_authority->valid());
  EXPECT_EQ(initial_authority->revision(), 1U);
  EXPECT_TRUE(initial_authority->owner().empty());
  EXPECT_EQ(initial_authority->input(), nullptr);
  EXPECT_TRUE(initial_authority->control().empty());

  const std::shared_ptr<const ExecutionPlan3D> initial_plan = initial_authority->plan();
  ASSERT_NE(initial_plan, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      activeTransition(*initial_plan, certified);
  ASSERT_TRUE(transition.applied());
  ASSERT_NE(transition.next, nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> exact_input =
      SnapshotFixture3D::committedInput(*transition.next);
  const ExecutionOwnerIdentity3D owner =
      SnapshotFixture3D::committedOwner(*transition.next, 7U);
  ASSERT_NE(exact_input, nullptr);
  ASSERT_TRUE(owner.validFor(*transition.next));

  ExecutionOwnerIdentity3D inconsistent_owner = owner;
  inconsistent_owner.stationary_position_hold = true;
  EXPECT_FALSE(inconsistent_owner.validFor(*transition.next));
  EXPECT_EQ(manager.publishLeasedTransition(initial_authority, transition,
                                            inconsistent_owner, exact_input),
            ExecutionRoutePublicationStatus3D::kInvalidCandidate);
  EXPECT_EQ(manager.authority(), initial_authority);

  const FiniteExecutionState3D unrelated_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *initial_plan, certified, FiniteExecutionKind3D::kNominal, true, 101U);
  ASSERT_NE(unrelated_execution.execution_input, exact_input);
  EXPECT_EQ(manager.publishLeasedTransition(initial_authority, transition, owner,
                                            unrelated_execution.execution_input),
            ExecutionRoutePublicationStatus3D::kInvalidCandidate);
  EXPECT_EQ(manager.authority(), initial_authority);

  ASSERT_EQ(manager.publishLeasedTransition(initial_authority, transition, owner,
                                            exact_input),
            ExecutionRoutePublicationStatus3D::kPublished);
  const std::shared_ptr<const CommittedExecutionAuthority3D> committed =
      manager.authority();
  ASSERT_NE(committed, nullptr);
  EXPECT_NE(committed, initial_authority);
  EXPECT_TRUE(committed->valid());
  EXPECT_EQ(committed->revision(), initial_authority->revision() + 1U);
  EXPECT_EQ(committed->plan(), transition.next);
  EXPECT_EQ(committed->input(), exact_input);
  EXPECT_EQ(committed->owner().producer_instance_id, owner.producer_instance_id);
  EXPECT_EQ(committed->owner().sequence, owner.sequence);
  EXPECT_EQ(committed->owner().execution_owner_epoch,
            transition.next->execution_owner_epoch);
  EXPECT_TRUE(committed->control().empty());
}

TEST(CommittedExecutionAuthority3DTest,
     ControlReplacementAndLeaseClearRequireTheExactAuthorityRevision) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const CertifiedRouteSuffix3D& certified =
      suffix.value(); // NOLINT(bugprone-unchecked-optional-access)
  RouteExecutionManager3D manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial =
      manager.authority();
  ASSERT_NE(initial, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      activeTransition(*initial->plan(), certified);
  ASSERT_TRUE(transition.applied());
  const ExecutionOwnerIdentity3D owner =
      SnapshotFixture3D::committedOwner(*transition.next, 11U);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      SnapshotFixture3D::committedInput(*transition.next);
  ASSERT_EQ(manager.publishLeasedTransition(initial, transition, owner, input),
            ExecutionRoutePublicationStatus3D::kPublished);
  const std::shared_ptr<const CommittedExecutionAuthority3D> leased =
      manager.authority();
  ASSERT_NE(leased, nullptr);

  const AppliedControlEvidence3D control = SnapshotFixture3D::committedControl(owner);
  AppliedControlEvidence3D non_authoritative_planned_control = control;
  non_authoritative_planned_control.control_authoritative = false;
  non_authoritative_planned_control.yaw_acceleration_authoritative = false;
  EXPECT_FALSE(non_authoritative_planned_control.validFor(owner));
  EXPECT_FALSE(
      manager.publishAppliedControlIfSame(leased, non_authoritative_planned_control));
  EXPECT_EQ(manager.authority(), leased);
  ASSERT_TRUE(manager.publishAppliedControlIfSame(leased, control));
  const std::shared_ptr<const CommittedExecutionAuthority3D> witnessed =
      manager.authority();
  ASSERT_NE(witnessed, nullptr);
  EXPECT_EQ(witnessed->revision(), leased->revision() + 1U);
  EXPECT_EQ(witnessed->plan(), leased->plan());
  EXPECT_EQ(witnessed->input(), input);
  EXPECT_EQ(witnessed->control().content_fingerprint, control.content_fingerprint);
  EXPECT_TRUE(witnessed->control().validFor(witnessed->owner()));

  EXPECT_FALSE(manager.clearAppliedControlIfSame(leased));
  EXPECT_EQ(
      manager.publishLeaseForUnchangedPlanIfSame(leased, leased->plan(), owner, input),
      ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion);
  EXPECT_EQ(manager.authority(), witnessed);

  ASSERT_TRUE(manager.clearAppliedControlIfSame(witnessed));
  const std::shared_ptr<const CommittedExecutionAuthority3D> unwitnessed =
      manager.authority();
  ASSERT_NE(unwitnessed, nullptr);
  EXPECT_TRUE(unwitnessed->control().empty());
  EXPECT_EQ(unwitnessed->plan(), witnessed->plan());
  EXPECT_EQ(unwitnessed->input(), input);

  ASSERT_TRUE(manager.clearLeaseIfSame(unwitnessed));
  const std::shared_ptr<const CommittedExecutionAuthority3D> detached =
      manager.authority();
  ASSERT_NE(detached, nullptr);
  EXPECT_TRUE(detached->valid());
  EXPECT_EQ(detached->plan(), unwitnessed->plan());
  EXPECT_TRUE(detached->owner().empty());
  EXPECT_EQ(detached->input(), nullptr);
  EXPECT_TRUE(detached->control().empty());
  EXPECT_FALSE(manager.publishAppliedControlIfSame(unwitnessed, control));
  EXPECT_EQ(manager.authority(), detached);
}

TEST(CommittedExecutionAuthority3DTest,
     PendingCommitPublishesACompleteLeaseAndConsumesOnlyTheCapturedPendingRoute) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const CertifiedRouteSuffix3D& certified =
      suffix.value(); // NOLINT(bugprone-unchecked-optional-access)
  RouteExecutionManager3D manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial =
      manager.authority();
  ASSERT_NE(initial, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      activeTransition(*initial->plan(), certified);
  ASSERT_TRUE(transition.applied());
  const auto pending =
      std::make_shared<const PendingCertifiedRoute3D>(PendingCertifiedRoute3D{
          .publication_sequence = 1U,
          .base_execution_owner_epoch = initial->plan()->execution_owner_epoch,
          .base_kind = PendingExecutionBaseKind3D::kEmpty,
          .base_route_generation = 0U,
          .base_geometry_revision = 0U,
          .base_continuity_id = 0U,
          .base_direct_tracking_identity = std::nullopt,
          .route_splice = std::nullopt,
          .route = certified,
      });
  ASSERT_TRUE(publishPendingDraftForCurrentBase(manager, *pending));
  const std::shared_ptr<const PendingCertifiedRoute3D> sealed = manager.pending();
  ASSERT_NE(sealed, nullptr);
  ASSERT_NE(sealed, pending);
  const ExecutionOwnerIdentity3D owner =
      SnapshotFixture3D::committedOwner(*transition.next, 13U);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      SnapshotFixture3D::committedInput(*transition.next);

  ASSERT_EQ(
      manager.commitPendingLeasedTransition(sealed, initial, transition, owner, input),
      ExecutionRoutePublicationStatus3D::kPublished);
  const RouteExecutionManagerSnapshot3D committed = manager.snapshot();
  ASSERT_TRUE(committed.valid());
  ASSERT_NE(committed.authority, nullptr);
  EXPECT_EQ(committed.plan(), transition.next);
  EXPECT_EQ(committed.authority->input(), input);
  EXPECT_EQ(committed.authority->owner().sequence, owner.sequence);
  EXPECT_TRUE(committed.authority->control().empty());
  EXPECT_EQ(committed.pending, nullptr);
  EXPECT_EQ(
      manager.commitPendingLeasedTransition(sealed, initial, transition, owner, input),
      ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion);
}

TEST(CommittedExecutionAuthority3DTest,
     ConcurrentReadersObserveOnlyCompleteOldOrNewAuthorityRevisions) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const CertifiedRouteSuffix3D& certified =
      suffix.value(); // NOLINT(bugprone-unchecked-optional-access)
  RouteExecutionManager3D manager;
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial =
      manager.authority();
  ASSERT_NE(initial, nullptr);
  const ExecutionRouteTransitionResult3D transition =
      activeTransition(*initial->plan(), certified);
  ASSERT_TRUE(transition.applied());
  const ExecutionOwnerIdentity3D owner =
      SnapshotFixture3D::committedOwner(*transition.next, 17U);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      SnapshotFixture3D::committedInput(*transition.next);
  ASSERT_EQ(manager.publishLeasedTransition(initial, transition, owner, input),
            ExecutionRoutePublicationStatus3D::kPublished);
  const std::shared_ptr<const CommittedExecutionAuthority3D> leased =
      manager.authority();
  ASSERT_NE(leased, nullptr);
  const AppliedControlEvidence3D control = SnapshotFixture3D::committedControl(owner);

  constexpr std::size_t kReplacementCount{2'000U};
  std::atomic<bool> start{false};
  std::atomic<bool> done{false};
  std::atomic<bool> writer_failed{false};
  std::atomic<bool> mixed_revision_observed{false};
  std::thread reader{[&] {
    while (!start.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    while (!done.load(std::memory_order_acquire)) {
      const std::shared_ptr<const CommittedExecutionAuthority3D> observed =
          manager.authority();
      if (observed == nullptr || !observed->valid() ||
          observed->plan() != transition.next || observed->input() != input ||
          observed->owner().sequence != owner.sequence ||
          (observed->control().valid &&
           !observed->control().validFor(observed->owner())) ||
          (!observed->control().valid && !observed->control().empty())) {
        mixed_revision_observed.store(true, std::memory_order_release);
        return;
      }
    }
  }};
  std::thread writer{[&] {
    start.store(true, std::memory_order_release);
    for (std::size_t index = 0U; index < kReplacementCount; ++index) {
      const std::shared_ptr<const CommittedExecutionAuthority3D> current =
          manager.authority();
      const bool updated = current != nullptr && current->control().valid
                               ? manager.clearAppliedControlIfSame(current)
                               : manager.publishAppliedControlIfSame(current, control);
      if (!updated) {
        writer_failed.store(true, std::memory_order_release);
        break;
      }
    }
    done.store(true, std::memory_order_release);
  }};

  writer.join();
  reader.join();
  EXPECT_FALSE(writer_failed.load(std::memory_order_acquire));
  EXPECT_FALSE(mixed_revision_observed.load(std::memory_order_acquire));
  const std::shared_ptr<const CommittedExecutionAuthority3D> final =
      manager.authority();
  ASSERT_NE(final, nullptr);
  EXPECT_EQ(final->revision(), leased->revision() + kReplacementCount);
  EXPECT_TRUE(final->valid());
}

} // namespace
} // namespace drone_city_nav
