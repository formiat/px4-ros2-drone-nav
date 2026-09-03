#pragma once

#include "drone_city_nav/certified_route_splice_3d.hpp"
#include "drone_city_nav/execution_plan_3d.hpp"
#include "drone_city_nav/execution_route_transitions_3d.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace drone_city_nav {

enum class PendingExecutionBaseKind3D : std::uint8_t {
  kEmpty,
  kRoute,
  kRouteHandoff,
  kDirectTracking,
  kStationaryHold,
  kRevoked,
};

struct PendingCertifiedRoute3D {
  std::uint64_t publication_sequence{0U};
  std::uint64_t base_execution_owner_epoch{0U};
  PendingExecutionBaseKind3D base_kind{PendingExecutionBaseKind3D::kEmpty};
  std::uint64_t base_route_generation{0U};
  std::uint64_t base_geometry_revision{0U};
  std::uint64_t base_continuity_id{0U};
  std::optional<DirectTrackingOwnerIdentity3D> base_direct_tracking_identity;
  std::optional<CertifiedRouteSplice3D> route_splice;
  CertifiedRouteSuffix3D route{};

  [[nodiscard]] bool valid() const noexcept;
};

[[nodiscard]] bool
pendingCertifiedRouteEligible3D(const PendingCertifiedRoute3D& pending,
                                const ExecutionPlan3D& snapshot) noexcept;

// A splice-free successor is an immutable current-state planning transaction.
// Preserve its route-wide snapshot certificate while the actual command and
// braking horizons are independently validated against the latest physical
// evidence at commit. This admission rule is independent of why the background
// successor was requested: changing route guidance must not revoke or stall the
// resident finite execution owner.
[[nodiscard]] bool pendingCertifiedRouteRetainsSnapshotCertificate3D(
    const PendingCertifiedRoute3D& pending) noexcept;

// A pending activation the plan reducer rejects on the candidate's own contract
// cannot succeed by being offered again against the same base: the rejection
// depends on the pending route and the plan it extends, not on evidence that
// the next tick refreshes. That is every kInvalidCandidate rejection, and a
// kCertificateRegression whose detail names the successor's own certificate
// (its policy, kind, producer, validated revision, or world content), since
// the certificate a pending route carries never moves while the resident
// route's can only advance. Transient outcomes such as a moved snapshot
// version, an exhausted version, a finite-execution ordering conflict, or a
// regression of the finite execution the candidate is offered with are
// retried instead.
[[nodiscard]] bool pendingRouteActivationStructurallyRejected3D(
    ExecutionRouteTransitionStatus3D status,
    ExecutionRouteTransitionDetail3D detail) noexcept;

class ExecutionSupervisor3D;

struct PendingCertifiedRouteRecoveryObservation3D {
  bool direct_tracking_requested{false};
  bool execution_owner_available{false};
  bool pending_activation{false};
};

struct PendingCertifiedRouteRecoveryResult3D {
  bool pending_acknowledged{false};
  bool request_successor{false};
};

// Resolves the no-owner pending-plan liveness edge after pending-route
// recertification. A successor request is authorized only when there was no
// pending route or the exact failed pending identity was acknowledged.
[[nodiscard]] PendingCertifiedRouteRecoveryResult3D
recoverPendingCertifiedRouteLiveness3D(
    ExecutionSupervisor3D& supervisor,
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    const PendingCertifiedRouteRecoveryObservation3D& observation);

} // namespace drone_city_nav
