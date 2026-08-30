#include "drone_city_nav/execution_route_store_3d.hpp"

#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {

std::shared_ptr<const ExecutionPlan3D>
RouteExecutionManagerSnapshot3D::plan() const noexcept {
  return authority != nullptr ? authority->plan() : nullptr;
}

bool RouteExecutionManagerSnapshot3D::valid() const noexcept {
  return authority != nullptr && authority->valid() &&
         (pending == nullptr || pending->valid());
}

RouteExecutionManager3D::RouteExecutionManager3D() {
  std::shared_ptr<const ExecutionPlan3D> initial_plan =
      makeInitialExecutionRouteSnapshot3D();
  auto initial_authority = std::make_shared<const CommittedExecutionAuthority3D>(
      CommittedExecutionAuthority3D::CaptureToken{}, 1U, std::move(initial_plan),
      ExecutionOwnerIdentity3D{}, nullptr, AppliedControlEvidence3D{});
  if (!initial_authority->valid()) {
    throw std::logic_error{"invalid initial execution authority"};
  }
  authority_.store(std::move(initial_authority), std::memory_order_release);
}

RouteExecutionManagerSnapshot3D RouteExecutionManager3D::snapshot() const {
  const std::scoped_lock lock{mutex_};
  return RouteExecutionManagerSnapshot3D{
      .authority = authority_.load(std::memory_order_acquire),
      .pending = pending_,
  };
}

std::shared_ptr<const CommittedExecutionAuthority3D>
RouteExecutionManager3D::authority() const noexcept {
  return authority_.load(std::memory_order_acquire);
}

std::shared_ptr<const ExecutionPlan3D> RouteExecutionManager3D::plan() const {
  const std::shared_ptr<const CommittedExecutionAuthority3D> current = authority();
  return current != nullptr ? current->plan() : nullptr;
}

std::shared_ptr<const PendingCertifiedRoute3D>
RouteExecutionManager3D::pending() const {
  const std::scoped_lock lock{mutex_};
  return pending_;
}

ExecutionRoutePublicationStatus3D RouteExecutionManager3D::publishAuthorityLocked(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    std::shared_ptr<const ExecutionPlan3D> next_plan,
    const ExecutionOwnerIdentity3D& owner,
    std::shared_ptr<const VersionedExecutionInput3D> input,
    const AppliedControlEvidence3D& control) {
  const std::shared_ptr<const CommittedExecutionAuthority3D> current =
      authority_.load(std::memory_order_acquire);
  if (expected_authority == nullptr || current != expected_authority) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  if (!expected_authority->valid() || next_plan == nullptr || !next_plan->valid()) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  if (expected_authority->revision() == std::numeric_limits<std::uint64_t>::max()) {
    return ExecutionRoutePublicationStatus3D::kAuthorityRevisionExhausted;
  }

  auto next = std::make_shared<const CommittedExecutionAuthority3D>(
      CommittedExecutionAuthority3D::CaptureToken{},
      expected_authority->revision() + 1U, std::move(next_plan), owner,
      std::move(input), control);
  if (!next->valid()) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  authority_.store(std::move(next), std::memory_order_release);
  return ExecutionRoutePublicationStatus3D::kPublished;
}

ExecutionRoutePublicationStatus3D RouteExecutionManager3D::publishTransitionLocked(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const ExecutionRouteTransitionResult3D& transition,
    const ExecutionOwnerIdentity3D& owner,
    std::shared_ptr<const VersionedExecutionInput3D> input) {
  const std::shared_ptr<const CommittedExecutionAuthority3D> current =
      authority_.load(std::memory_order_acquire);
  if (expected_authority == nullptr || current != expected_authority) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  const std::shared_ptr<const ExecutionPlan3D>& expected_plan =
      expected_authority->plan();
  if (expected_plan == nullptr || !transition.applied() || transition.next == nullptr ||
      !transition.next->publishable() ||
      expected_plan->version == std::numeric_limits<std::uint64_t>::max() ||
      transition.next->version <= expected_plan->version) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  if (transition.predecessor != expected_plan.get()) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  return publishAuthorityLocked(expected_authority, transition.next, owner,
                                std::move(input), AppliedControlEvidence3D{});
}

ExecutionRoutePublicationStatus3D RouteExecutionManager3D::publishDetachedTransition(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const ExecutionRouteTransitionResult3D& transition) {
  const std::scoped_lock lock{mutex_};
  return publishTransitionLocked(expected_authority, transition,
                                 ExecutionOwnerIdentity3D{}, nullptr);
}

ExecutionRoutePublicationStatus3D RouteExecutionManager3D::publishLeasedTransition(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const ExecutionRouteTransitionResult3D& transition,
    const ExecutionOwnerIdentity3D& owner,
    std::shared_ptr<const VersionedExecutionInput3D> input) {
  const std::scoped_lock lock{mutex_};
  return publishTransitionLocked(expected_authority, transition, owner,
                                 std::move(input));
}

ExecutionRoutePublicationStatus3D
RouteExecutionManager3D::publishLeaseForUnchangedPlanIfSame(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const std::shared_ptr<const ExecutionPlan3D>& expected_plan,
    const ExecutionOwnerIdentity3D& owner,
    std::shared_ptr<const VersionedExecutionInput3D> input) {
  const std::scoped_lock lock{mutex_};
  const std::shared_ptr<const CommittedExecutionAuthority3D> current =
      authority_.load(std::memory_order_acquire);
  if (expected_authority == nullptr || current != expected_authority ||
      expected_authority->plan() != expected_plan) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  if (expected_plan == nullptr || !expected_plan->publishable()) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  return publishAuthorityLocked(expected_authority, expected_plan, owner,
                                std::move(input), AppliedControlEvidence3D{});
}

bool RouteExecutionManager3D::publishAppliedControlIfSame(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const AppliedControlEvidence3D& control) {
  const std::scoped_lock lock{mutex_};
  if (expected_authority == nullptr || !expected_authority->owner().valid ||
      expected_authority->input() == nullptr || !control.valid ||
      !control.validFor(expected_authority->owner())) {
    return false;
  }
  return publishAuthorityLocked(expected_authority, expected_authority->plan(),
                                expected_authority->owner(),
                                expected_authority->input(), control) ==
         ExecutionRoutePublicationStatus3D::kPublished;
}

bool RouteExecutionManager3D::clearAppliedControlIfSame(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority) {
  const std::scoped_lock lock{mutex_};
  if (expected_authority == nullptr || !expected_authority->control().valid) {
    return false;
  }
  return publishAuthorityLocked(expected_authority, expected_authority->plan(),
                                expected_authority->owner(),
                                expected_authority->input(),
                                AppliedControlEvidence3D{}) ==
         ExecutionRoutePublicationStatus3D::kPublished;
}

bool RouteExecutionManager3D::clearLeaseIfSame(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority) {
  const std::scoped_lock lock{mutex_};
  if (expected_authority == nullptr || !expected_authority->owner().valid) {
    return false;
  }
  return publishAuthorityLocked(expected_authority, expected_authority->plan(),
                                ExecutionOwnerIdentity3D{}, nullptr,
                                AppliedControlEvidence3D{}) ==
         ExecutionRoutePublicationStatus3D::kPublished;
}

bool RouteExecutionManager3D::publishPending(
    std::shared_ptr<const PendingCertifiedRoute3D> candidate) {
  if (candidate == nullptr || !candidate->valid()) {
    return false;
  }
  // Seal the publication boundary even when the caller retains a mutable alias
  // to the original allocation.
  const auto sealed = std::make_shared<const PendingCertifiedRoute3D>(*candidate);
  if (!sealed->valid()) {
    return false;
  }
  const std::scoped_lock lock{mutex_};
  if (pending_ != nullptr ||
      sealed->publication_sequence <= last_accepted_pending_sequence_) {
    return false;
  }
  pending_ = sealed;
  last_accepted_pending_sequence_ = sealed->publication_sequence;
  return true;
}

bool RouteExecutionManager3D::acknowledgePendingIfSame(
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending) {
  if (expected_pending == nullptr) {
    return false;
  }
  const std::scoped_lock lock{mutex_};
  if (pending_ != expected_pending) {
    return false;
  }
  pending_.reset();
  return true;
}

bool RouteExecutionManager3D::commitPendingLeasedTransitionIfSame(
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const ExecutionRouteTransitionResult3D& transition,
    const ExecutionOwnerIdentity3D& owner,
    std::shared_ptr<const VersionedExecutionInput3D> input) {
  if (expected_pending == nullptr || expected_authority == nullptr ||
      transition.next == nullptr) {
    return false;
  }
  const std::scoped_lock lock{mutex_};
  const std::shared_ptr<const CommittedExecutionAuthority3D> current =
      authority_.load(std::memory_order_acquire);
  const std::shared_ptr<const ExecutionPlan3D>& expected_plan =
      expected_authority->plan();
  const CertifiedRouteSuffix3D* const committed_route = transition.next->route();
  const bool committed_route_is_pending_revision =
      committed_route != nullptr &&
      (committed_route->route_instance_id ==
           expected_pending->route.route_instance_id ||
       committed_route->parent_route_instance_id ==
           std::optional<RouteInstanceId3D>{expected_pending->route.route_instance_id});
  if (current != expected_authority || pending_ != expected_pending ||
      expected_plan == nullptr ||
      !pendingCertifiedRouteEligible3D(*expected_pending, *expected_plan) ||
      !committed_route_is_pending_revision) {
    return false;
  }
  if (publishTransitionLocked(expected_authority, transition, owner,
                              std::move(input)) !=
      ExecutionRoutePublicationStatus3D::kPublished) {
    return false;
  }
  pending_.reset();
  return true;
}

} // namespace drone_city_nav
