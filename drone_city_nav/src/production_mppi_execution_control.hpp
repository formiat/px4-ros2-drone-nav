#pragma once

#include "drone_city_nav/committed_execution_authority_3d.hpp"
#include "drone_city_nav/control_contracts_3d.hpp"
#include "drone_city_nav/finite_execution_path_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace drone_city_nav {

struct ProductionMppiNavigation;
struct ProductionMppiVehicleStatus;

enum class ProductionMppiHorizonSupersessionDecision : std::uint8_t {
  kAllowedNoPlannedOwner,
  kAllowedWitnessedOwner,
  kDeferredAwaitingOwnerWitness,
  kRejectedOwnerNotCurrent,
};

// A planned execution lease must remain the wire owner until offboard has
// witnessed that exact tuple. Replacing an unwitnessed lease at planner rate
// can keep every real feedback sample one generation behind forever.
[[nodiscard]] constexpr ProductionMppiHorizonSupersessionDecision
assessPlannedHorizonSupersession(const ExecutionOwnerIdentity3D& owner,
                                 const bool owner_witnessed,
                                 const std::int64_t now_ns) noexcept {
  if (!owner.valid || owner.execution_mode != ExecutionAuthorityMode3D::kPlanned) {
    return ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner;
  }
  if (owner.valid_from_ns <= 0 || owner.valid_until_ns <= owner.valid_from_ns ||
      now_ns < owner.valid_from_ns || now_ns >= owner.valid_until_ns) {
    return ProductionMppiHorizonSupersessionDecision::kRejectedOwnerNotCurrent;
  }
  return owner_witnessed
             ? ProductionMppiHorizonSupersessionDecision::kAllowedWitnessedOwner
             : ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingOwnerWitness;
}

struct ProductionMppiResidentOwnerContinuationCheck {
  const ExecutionOwnerIdentity3D* owner{nullptr};
  std::int64_t now_ns{0};
  bool retained_candidate{false};
  bool exact_snapshot_current{false};
  bool execution_owner_matches{false};
  bool revocation_pending{false};
  bool owner_witnessed{false};
};

// A failed retained replacement may defer to the already-published lease only
// when that exact resident authority is still current and observed by offboard.
// This decision never extends the lease and never authorizes another snapshot.
[[nodiscard]] constexpr bool canContinueResidentPlannedOwner(
    const ProductionMppiResidentOwnerContinuationCheck& check) noexcept {
  return check.owner != nullptr && check.retained_candidate &&
         check.exact_snapshot_current && check.execution_owner_matches &&
         !check.revocation_pending &&
         assessPlannedHorizonSupersession(*check.owner, check.owner_witnessed,
                                          check.now_ns) ==
             ProductionMppiHorizonSupersessionDecision::kAllowedWitnessedOwner;
}

using ProductionMppiExecutionMode = ExecutionAuthorityMode3D;
using ProductionMppiExecutionReason = ExecutionAuthorityReason3D;

enum class ProductionMppiPhysicalTrajectoryAuthority : std::uint8_t {
  kUnownedCandidate,
  kResidentOwner,
};

enum class ProductionMppiPhysicalCollisionAction : std::uint8_t {
  kRejectCandidate,
  kRequestRouteSuccessor,
};

// Physical invalidation always permits a fail-closed transport revocation.
// Non-physical policy failures may revoke only when the explicit optional
// constraint is enabled; otherwise the resident finite owner remains in force.
[[nodiscard]] constexpr bool executionRevocationAllowed(
    const bool physical_route_invalidation,
    const bool nonphysical_execution_revocation_enabled) noexcept {
  return physical_route_invalidation || nonphysical_execution_revocation_enabled;
}

// A rejected candidate has never owned vehicle motion and therefore cannot
// invalidate the resident finite execution. Only physical evidence intersecting
// the already-published finite trajectory authorizes its emergency successor.
[[nodiscard]] constexpr ProductionMppiPhysicalCollisionAction physicalCollisionAction(
    const ProductionMppiPhysicalTrajectoryAuthority authority) noexcept {
  return authority == ProductionMppiPhysicalTrajectoryAuthority::kResidentOwner
             ? ProductionMppiPhysicalCollisionAction::kRequestRouteSuccessor
             : ProductionMppiPhysicalCollisionAction::kRejectCandidate;
}

enum class ProductionMppiResidentObstacleDisposition : std::uint8_t {
  kClear,
  kRouteSuffixReplacementRequired,
  kPersistentRawFiniteExecutionInvalidated,
  kLatestLidarFiniteExecutionInvalidated,
};

struct ProductionMppiResidentObstacleEvidence {
  bool route_suffix_persistent_raw{false};
  bool finite_execution_persistent_raw{false};
  bool finite_execution_latest_lidar{false};
};

// Only a hit on the published finite execution invalidates its owner. A hit on
// the farther route suffix requests a background replacement while that owner
// keeps executing. Persistent raw evidence takes precedence over the latest
// scan when both invalidate the finite execution.
[[nodiscard]] constexpr ProductionMppiResidentObstacleDisposition
residentObstacleDisposition(
    const ProductionMppiResidentObstacleEvidence& evidence) noexcept {
  if (evidence.finite_execution_persistent_raw) {
    return ProductionMppiResidentObstacleDisposition::
        kPersistentRawFiniteExecutionInvalidated;
  }
  if (evidence.finite_execution_latest_lidar) {
    return ProductionMppiResidentObstacleDisposition::
        kLatestLidarFiniteExecutionInvalidated;
  }
  if (evidence.route_suffix_persistent_raw) {
    return ProductionMppiResidentObstacleDisposition::kRouteSuffixReplacementRequired;
  }
  return ProductionMppiResidentObstacleDisposition::kClear;
}

struct ProductionMppiExecutionPublication {
  std::vector<MotionState3D> horizon;
  ProductionMppiExecutionMode mode{ProductionMppiExecutionMode::kPlanned};
  ProductionMppiExecutionReason reason{ProductionMppiExecutionReason::kNone};
  std::size_t planned_control_count{0U};
  std::size_t nominal_prefix_control_count{0U};
  std::size_t arrival_control_count{0U};
  std::size_t arrival_shaping_attempts{0U};
  MotionControl3D first_control{};
  std::uint64_t latest_lidar_obstacle_sequence{0U};
  std::size_t latest_lidar_obstacle_hit_count{0U};
  double latest_lidar_obstacle_age_ms{-1.0};
  bool finite_path_validation_backoff{false};
  FiniteExecutionPathStatus3D finite_path_validation_status{
      FiniteExecutionPathStatus3D::kInvalidContract};
  FiniteExecutionPathStatus3D finite_path_first_failed_validation_status{
      FiniteExecutionPathStatus3D::kValid};
  bool latest_lidar_obstacle_fresh{false};
  bool latest_lidar_obstacle_receive_time_fallback{false};
  bool latest_lidar_path_validation_backoff{false};
  bool retained_previous_finite_path{false};
  bool resident_owner_continues{false};
  bool terminal_rest_state{false};
  bool first_control_available{false};
  bool published{false};
};

enum class ProductionMppiPlanningState {
  kPlanned,
  kMissionCommandPositionHold,
  kCooperativePassageYieldHold,
  kMissionGoalPositionHold,
  kNoExecutableRouteHold,
};

enum class ProductionMppiPreviousControlSource {
  kUnavailable,
  kEngineFallback,
  kMeasuredAcceleration,
  kOffboardFeedback,
  kStationaryCaptureRearm,
};

[[nodiscard]] const char*
productionMppiExecutionModeName(ProductionMppiExecutionMode mode) noexcept;
[[nodiscard]] const char*
productionMppiExecutionReasonName(ProductionMppiExecutionReason reason) noexcept;
[[nodiscard]] const char*
productionMppiPlanningStateName(ProductionMppiPlanningState state) noexcept;
[[nodiscard]] const char* productionMppiPreviousControlSourceName(
    ProductionMppiPreviousControlSource source) noexcept;

[[nodiscard]] bool appliedControlAuthoritativeForExecution(
    const AppliedControlEvidence3D& control, const ExecutionOwnerIdentity3D& owner,
    std::int64_t now_ns, double maximum_age_ms) noexcept;

// Execution ownership is allowed only while the latest admitted PX4 status is
// from a stable timestamp epoch, is armed, and is fresh at the commit instant.
[[nodiscard]] bool
vehicleStatusAuthoritativeForExecution(const ProductionMppiVehicleStatus& status,
                                       bool timestamp_epoch_stable, std::int64_t now_ns,
                                       double maximum_age_ms) noexcept;

// Planned feedback is the exact execution evidence. A measured linear
// acceleration is the fail-closed fallback when no feedback owns the current
// horizon; an invented vertical axis must never authorize unknown-space
// promotion.
[[nodiscard]] std::optional<FootprintBodyAxis>
authoritativeBodyAxisForExecution(const AppliedControlEvidence3D& applied_control,
                                  const ExecutionOwnerIdentity3D& owner,
                                  const ProductionMppiNavigation& navigation,
                                  std::int64_t now_ns, double maximum_control_age_ms,
                                  double maximum_pose_age_ms) noexcept;

} // namespace drone_city_nav
