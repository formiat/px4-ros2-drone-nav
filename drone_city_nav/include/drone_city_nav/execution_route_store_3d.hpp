#pragma once

#include "drone_city_nav/committed_execution_authority_3d.hpp"
#include "drone_city_nav/execution_route_transitions_3d.hpp"
#include "drone_city_nav/pending_certified_route_3d.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>

namespace drone_city_nav {

enum class ExecutionRoutePublicationStatus3D : std::uint8_t {
  kPublished,
  kInvalidCandidate,
  kStaleSnapshotVersion,
  kAuthorityRevisionExhausted,
};

enum class PendingRoutePublicationStatus3D : std::uint8_t {
  kPublished,
  kInvalidCandidate,
  kStaleExecutionBase,
  kPendingOccupied,
  kSequenceExhausted,
};

struct PendingRoutePublicationResult3D {
  PendingRoutePublicationStatus3D status{
      PendingRoutePublicationStatus3D::kInvalidCandidate};
  std::shared_ptr<const PendingCertifiedRoute3D> pending;

  [[nodiscard]] bool published() const noexcept {
    return status == PendingRoutePublicationStatus3D::kPublished && pending != nullptr;
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
      const ExecutionRouteTransitionResult3D& transition);

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

  [[nodiscard]] bool acknowledgePendingIfSame(
      const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending);

  [[nodiscard]] ExecutionRoutePublicationStatus3D commitPendingLeasedTransition(
      const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
      const ExecutionRouteTransitionResult3D& transition,
      const ExecutionOwnerIdentity3D& owner,
      std::shared_ptr<const VersionedExecutionInput3D> input);

  // Compatibility boolean for low-level callers that only need success. New
  // production orchestration consumes the typed status through the supervisor.
  [[nodiscard]] bool commitPendingLeasedTransitionIfSame(
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
      std::shared_ptr<const VersionedExecutionInput3D> input);

  mutable std::mutex mutex_;
  std::atomic<std::shared_ptr<const CommittedExecutionAuthority3D>> authority_;
  std::shared_ptr<const PendingCertifiedRoute3D> pending_;
  std::uint64_t last_accepted_pending_sequence_{0U};
};

} // namespace drone_city_nav
