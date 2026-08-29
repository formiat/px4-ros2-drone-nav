#pragma once

#include "drone_city_nav/certified_route_splice_3d.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
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
                                const ExecutionRouteSnapshot3D& snapshot) noexcept;

// A splice-free successor is an immutable current-state planning transaction.
// Preserve its route-wide snapshot certificate while the actual command and
// braking horizons are independently validated against the latest physical
// evidence at commit. This admission rule is independent of why the background
// successor was requested: changing route guidance must not revoke or stall the
// resident finite execution owner.
[[nodiscard]] bool pendingCertifiedRouteRetainsSnapshotCertificate3D(
    const PendingCertifiedRoute3D& pending) noexcept;

class PendingCertifiedRouteMailbox3D final {
public:
  [[nodiscard]] bool publish(std::shared_ptr<const PendingCertifiedRoute3D> candidate);

  [[nodiscard]] std::shared_ptr<const PendingCertifiedRoute3D> snapshot() const;

  [[nodiscard]] bool
  acknowledgeIfSame(const std::shared_ptr<const PendingCertifiedRoute3D>& expected);

  // Linearizes the exact pending-route identity with the execution-store CAS.
  // A failed store publication leaves the pending route resident.
  [[nodiscard]] bool commitExecutionIfSame(
      const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
      ExecutionRouteSnapshotStore3D& execution_store,
      const std::shared_ptr<const ExecutionRouteSnapshot3D>& expected_snapshot,
      const ExecutionRouteTransitionResult3D& transition);

private:
  mutable std::mutex mutex_;
  std::shared_ptr<const PendingCertifiedRoute3D> pending_;
  std::uint64_t last_accepted_publication_sequence_{0U};
};

struct PendingCertifiedRouteRecoveryObservation3D {
  bool direct_tracking_requested{false};
  bool execution_owner_available{false};
  bool pending_activation{false};
};

struct PendingCertifiedRouteRecoveryResult3D {
  bool pending_acknowledged{false};
  bool request_successor{false};
};

// Resolves the no-owner mailbox liveness edge after pending-route
// recertification. A successor request is authorized only when there was no
// pending route or the exact failed pending identity was acknowledged.
[[nodiscard]] PendingCertifiedRouteRecoveryResult3D
recoverPendingCertifiedRouteLiveness3D(
    PendingCertifiedRouteMailbox3D& mailbox,
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    const PendingCertifiedRouteRecoveryObservation3D& observation);

} // namespace drone_city_nav
