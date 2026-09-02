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
  kAllowedAcknowledgedPredecessor,
  kAllowedAcknowledgementGraceElapsed,
  kDeferredAwaitingAcknowledgement,
  kRejectedOwnerNotCurrent,
};

// The newest planned-horizon identity the current offboard process reported
// applying. It is a monotonic acknowledgement, not an exact control witness:
// the sequence only says which lease the controller is executing.
struct ProductionMppiHorizonAcknowledgement {
  std::uint64_t offboard_producer_instance_id{0U};
  std::uint64_t horizon_producer_instance_id{0U};
  std::uint64_t horizon_sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  bool valid{false};
};

// When the resident wire lease was published. Acknowledgement grace is
// measured from this instant rather than from the lease's capture time.
struct ProductionMppiHorizonPublicationRecord {
  std::uint64_t sequence{0U};
  std::int64_t publication_stamp_ns{0};
};

struct ProductionMppiHorizonSupersessionCheck {
  const ExecutionOwnerIdentity3D* owner{nullptr};
  const ProductionMppiHorizonAcknowledgement* acknowledgement{nullptr};
  // Exact fresh applied-control evidence for this owner.
  bool owner_witnessed{false};
  std::int64_t now_ns{0};
  // When this lease was put on the wire. Grace is measured from here because
  // the lease's valid_from is the earlier planning-capture instant.
  std::int64_t owner_publication_stamp_ns{0};
  std::int64_t acknowledgement_grace_ns{0};
  std::int64_t maximum_acknowledgement_age_ns{0};
};

// A planned lease may be replaced as soon as the controller has proven it is
// keeping up: it applied this lease, or it applied the immediate predecessor
// while this lease is still in flight. A lease the controller never
// acknowledges is replaced after a bounded grace instead of at its expiry, so
// one lost or delayed feedback sample cannot stall replanning for a whole
// horizon. Replacing at planner rate is intended receding-horizon behaviour;
// the predecessor rule only bounds how far the wire may run ahead of feedback.
[[nodiscard]] constexpr ProductionMppiHorizonSupersessionDecision
assessPlannedHorizonSupersession(
    const ProductionMppiHorizonSupersessionCheck& check) noexcept {
  if (check.owner == nullptr || !check.owner->valid ||
      check.owner->execution_mode != ExecutionAuthorityMode3D::kPlanned) {
    return ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner;
  }
  const ExecutionOwnerIdentity3D& owner = *check.owner;
  if (owner.valid_from_ns <= 0 || owner.valid_until_ns <= owner.valid_from_ns ||
      check.now_ns < owner.valid_from_ns || check.now_ns >= owner.valid_until_ns) {
    return ProductionMppiHorizonSupersessionDecision::kRejectedOwnerNotCurrent;
  }
  if (check.owner_witnessed) {
    return ProductionMppiHorizonSupersessionDecision::kAllowedWitnessedOwner;
  }
  const ProductionMppiHorizonAcknowledgement* const acknowledgement =
      check.acknowledgement;
  if (acknowledgement != nullptr && acknowledgement->valid &&
      acknowledgement->receive_stamp_ns > 0 &&
      check.maximum_acknowledgement_age_ns > 0 &&
      check.now_ns >= acknowledgement->receive_stamp_ns &&
      check.now_ns - acknowledgement->receive_stamp_ns <=
          check.maximum_acknowledgement_age_ns &&
      acknowledgement->offboard_producer_instance_id ==
          owner.target_offboard_instance_id &&
      acknowledgement->horizon_producer_instance_id == owner.producer_instance_id &&
      acknowledgement->horizon_sequence + 1U >= owner.sequence) {
    return ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgedPredecessor;
  }
  const std::int64_t publication_stamp_ns = check.owner_publication_stamp_ns > 0
                                                ? check.owner_publication_stamp_ns
                                                : owner.valid_from_ns;
  if (check.acknowledgement_grace_ns >= 0 &&
      check.now_ns - publication_stamp_ns >= check.acknowledgement_grace_ns) {
    return ProductionMppiHorizonSupersessionDecision::
        kAllowedAcknowledgementGraceElapsed;
  }
  return ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingAcknowledgement;
}

[[nodiscard]] constexpr bool horizonSupersessionAllowed(
    const ProductionMppiHorizonSupersessionDecision decision) noexcept {
  switch (decision) {
    case ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner:
    case ProductionMppiHorizonSupersessionDecision::kAllowedWitnessedOwner:
    case ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgedPredecessor:
    case ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgementGraceElapsed:
      return true;
    case ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingAcknowledgement:
    case ProductionMppiHorizonSupersessionDecision::kRejectedOwnerNotCurrent:
      return false;
  }
  return false;
}

[[nodiscard]] const char* productionMppiHorizonSupersessionDecisionName(
    ProductionMppiHorizonSupersessionDecision decision) noexcept;

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
