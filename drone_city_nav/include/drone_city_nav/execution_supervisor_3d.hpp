#pragma once

#include "drone_city_nav/execution_hold_3d.hpp"
#include "drone_city_nav/execution_retention_3d.hpp"
#include "drone_city_nav/execution_route_store_3d.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace drone_city_nav {

enum class ExecutionLeaseCommitKind3D : std::uint8_t {
  kTransition,
  kUnchangedPlan,
  kPendingTransition,
};

// One owned lease-publication transaction. A transition is copied into the
// request so the supervisor never depends on a caller-owned temporary while it
// validates and publishes the new atomic execution authority.
struct ExecutionLeaseCommit3D {
  ExecutionLeaseCommitKind3D kind{ExecutionLeaseCommitKind3D::kUnchangedPlan};
  std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority;
  std::shared_ptr<const ExecutionPlan3D> expected_plan;
  std::optional<ExecutionRouteTransitionResult3D> transition;
  std::shared_ptr<const PendingCertifiedRoute3D> expected_pending;
  ExecutionOwnerIdentity3D owner{};
  std::shared_ptr<const VersionedExecutionInput3D> input;
};

// Production facade for the sole pending/active execution store. The manager
// remains the low-level linearizable store and pure transition gate; production
// orchestration cannot access it directly or split a lease commit into separate
// check and publication calls.
class ExecutionSupervisor3D final {
public:
  ExecutionSupervisor3D() = default;

  ExecutionSupervisor3D(const ExecutionSupervisor3D&) = delete;
  ExecutionSupervisor3D& operator=(const ExecutionSupervisor3D&) = delete;
  ExecutionSupervisor3D(ExecutionSupervisor3D&&) = delete;
  ExecutionSupervisor3D& operator=(ExecutionSupervisor3D&&) = delete;

  [[nodiscard]] RouteExecutionManagerSnapshot3D snapshot() const;
  [[nodiscard]] std::shared_ptr<const CommittedExecutionAuthority3D>
  authority() const noexcept;
  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> plan() const;
  [[nodiscard]] std::shared_ptr<const PendingCertifiedRoute3D> pending() const;

  [[nodiscard]] PendingRoutePublicationResult3D publishPendingForCurrentBase(
      const std::shared_ptr<const ExecutionPlan3D>& expected_execution_base,
      PendingCertifiedRoute3D candidate);

  [[nodiscard]] bool acknowledgePendingIfSame(
      const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending);

  [[nodiscard]] ExecutionRoutePublicationStatus3D
  commitLease(ExecutionLeaseCommit3D commit);

  [[nodiscard]] ExecutionRetentionResult3D
  prepareRetention(ExecutionRetentionRequest3D request) const;

  [[nodiscard]] ExecutionHoldPreparation3D
  prepareHold(ExecutionHoldRequest3D request) const;

  [[nodiscard]] ExecutionRoutePublicationStatus3D commitDetachedTransition(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
      const ExecutionRouteTransitionResult3D& transition);

  [[nodiscard]] bool publishAppliedControlIfSame(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
      const AppliedControlEvidence3D& control);

  [[nodiscard]] bool clearAppliedControlIfSame(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority);

  [[nodiscard]] bool clearLeaseIfSame(
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority);

private:
  RouteExecutionManager3D manager_{};
};

} // namespace drone_city_nav
