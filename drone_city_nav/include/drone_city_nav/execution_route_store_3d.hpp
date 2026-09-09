#pragma once

#include "drone_city_nav/committed_execution_authority_3d.hpp"
#include "drone_city_nav/execution_route_transitions_3d.hpp"
#include "drone_city_nav/pending_certified_route_3d.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>

namespace drone_city_nav {

// What a transition committed without a lease is for. Almost every one leaves
// a plan a horizon can still be published from, and installing anything else
// would strand the vehicle on an authority nothing can drive it with. The
// exception is the transition whose whole purpose is to take the horizon away:
// a revocation, or the suspension a revocation prefers, which keeps the
// certified route for a successor to resume from and publishes nothing until
// one arrives.
enum class DetachedTransitionIntent3D : std::uint8_t {
  kKeepThePlanPublishable,
  kRelinquishTheHorizon,
};

enum class ExecutionRoutePublicationStatus3D : std::uint8_t {
  kPublished,
  kInvalidCandidate,
  kStaleSnapshotVersion,
  kAuthorityRevisionExhausted,
};

[[nodiscard]] constexpr const char* executionRoutePublicationStatus3DName(
    const ExecutionRoutePublicationStatus3D status) noexcept {
  switch (status) {
    case ExecutionRoutePublicationStatus3D::kPublished:
      return "published";
    case ExecutionRoutePublicationStatus3D::kInvalidCandidate:
      return "invalid_candidate";
    case ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion:
      return "stale_snapshot_version";
    case ExecutionRoutePublicationStatus3D::kAuthorityRevisionExhausted:
      return "authority_revision_exhausted";
  }
  return "unknown";
}

enum class PendingRoutePublicationStatus3D : std::uint8_t {
  kPublished,
  kReplaced,
  kInvalidCandidate,
  kStaleExecutionBase,
  kPendingChanged,
  kPendingOccupied,
  kSequenceExhausted,
};

struct PendingRoutePublicationResult3D {
  PendingRoutePublicationStatus3D status{
      PendingRoutePublicationStatus3D::kInvalidCandidate};
  std::shared_ptr<const PendingCertifiedRoute3D> pending;

  [[nodiscard]] bool published() const noexcept {
    return (status == PendingRoutePublicationStatus3D::kPublished ||
            status == PendingRoutePublicationStatus3D::kReplaced) &&
           pending != nullptr;
  }
};

// Compares only the semantic base that a pending route is certified to replace.
// Finite-horizon refreshes and control evidence do not change this identity;
// route, direct-tracking, hold, revocation, and owner-epoch changes do.
[[nodiscard]] bool
sameExecutionRouteBase3D(const std::shared_ptr<const ExecutionPlan3D>& first,
                         const std::shared_ptr<const ExecutionPlan3D>& second) noexcept;

struct RouteExecutionManagerSnapshot3D {
  std::shared_ptr<const CommittedExecutionAuthority3D> authority;
  std::shared_ptr<const PendingCertifiedRoute3D> pending;

  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> plan() const noexcept;
  [[nodiscard]] bool valid() const noexcept;
};

// Sole store for committed execution authority and pending certified routes.
// Readers atomically capture plan, owner, exact input, and applied-control
// evidence. Writers and pending-route changes are linearized by one mutex; the
// pure reducer remains the only semantic plan transition authority.
class RouteExecutionManager3D final {
public:
  RouteExecutionManager3D();

  [[nodiscard]] RouteExecutionManagerSnapshot3D snapshot() const;
  [[nodiscard]] std::shared_ptr<const CommittedExecutionAuthority3D>
  authority() const noexcept;
  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> plan() const;
  [[nodiscard]] std::shared_ptr<const PendingCertifiedRoute3D> pending() const;

  [[nodiscard]] ExecutionRoutePublicationStatus3D publishDetachedTransition(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
      const ExecutionRouteTransitionResult3D& transition,
      DetachedTransitionIntent3D intent =
          DetachedTransitionIntent3D::kKeepThePlanPublishable);

  [[nodiscard]] ExecutionRoutePublicationStatus3D publishLeasedTransition(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
      const ExecutionRouteTransitionResult3D& transition,
      const ExecutionOwnerIdentity3D& owner,
      std::shared_ptr<const VersionedExecutionInput3D> input);

  [[nodiscard]] ExecutionRoutePublicationStatus3D publishLeaseForUnchangedPlanIfSame(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
      const std::shared_ptr<const ExecutionPlan3D>& expected_plan,
      const ExecutionOwnerIdentity3D& owner,
      std::shared_ptr<const VersionedExecutionInput3D> input);

  [[nodiscard]] bool publishAppliedControlIfSame(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
      const AppliedControlEvidence3D& control);

  [[nodiscard]] bool clearAppliedControlIfSame(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority);

  [[nodiscard]] bool clearLeaseIfSame(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority);

  // Atomically verifies the semantic execution base, assigns the sole
  // manager-owned publication sequence, seals the candidate, and occupies the
  // pending slot. The candidate is an unsealed draft and must carry sequence
  // zero; callers cannot reserve or publish a sequence independently.
  [[nodiscard]] PendingRoutePublicationResult3D publishPendingForCurrentBase(
      const std::shared_ptr<const ExecutionPlan3D>& expected_execution_base,
      PendingCertifiedRoute3D candidate);

  // Replaces only the exact pending identity captured with the same semantic
  // execution base. Sequence allocation, candidate validation, and the pointer
  // swap are one transaction; a concurrent consume or replacement wins cleanly.
  [[nodiscard]] PendingRoutePublicationResult3D replacePendingForCurrentBase(
      const std::shared_ptr<const ExecutionPlan3D>& expected_execution_base,
      const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
      PendingCertifiedRoute3D candidate);

  [[nodiscard]] bool acknowledgePendingIfSame(
      const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending);

  [[nodiscard]] ExecutionRoutePublicationStatus3D commitPendingLeasedTransition(
      const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
      const ExecutionRouteTransitionResult3D& transition,
      const ExecutionOwnerIdentity3D& owner,
      std::shared_ptr<const VersionedExecutionInput3D> input);

private:
  [[nodiscard]] ExecutionRoutePublicationStatus3D publishAuthorityLocked(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
      std::shared_ptr<const ExecutionPlan3D> next_plan,
      const ExecutionOwnerIdentity3D& owner,
      std::shared_ptr<const VersionedExecutionInput3D> input,
      const AppliedControlEvidence3D& control);

  [[nodiscard]] ExecutionRoutePublicationStatus3D publishTransitionLocked(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
      const ExecutionRouteTransitionResult3D& transition,
      const ExecutionOwnerIdentity3D& owner,
      std::shared_ptr<const VersionedExecutionInput3D> input,
      DetachedTransitionIntent3D intent);

  mutable std::mutex mutex_;
  std::atomic<std::shared_ptr<const CommittedExecutionAuthority3D>> authority_;
  std::shared_ptr<const PendingCertifiedRoute3D> pending_;
  std::uint64_t last_accepted_pending_sequence_{0U};
};

} // namespace drone_city_nav
