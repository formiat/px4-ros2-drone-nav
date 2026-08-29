#pragma once

#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/msg/mppi_control_feedback.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace drone_city_nav {

struct ProductionMppiNavigation;
struct ProductionMppiVehicleStatus;

struct ProductionMppiAppliedControl {
  mppi::Control control{};
  float yaw_rate_radps{0.0F};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t horizon_producer_instance_id{0U};
  std::uint64_t horizon_sequence{0U};
  std::uint64_t content_fingerprint{0U};
  std::uint8_t execution_mode{msg::MppiControlFeedback::EXECUTION_MODE_POSITION_HOLD};
  bool yaw_acceleration_authoritative{false};
  bool control_authoritative{false};
  bool valid{false};
};

struct ProductionMppiExecutionHorizonOwner {
  Point3 route_target{};
  Point3 stationary_hold_position{};
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t target_offboard_instance_id{0U};
  std::uint64_t sequence{0U};
  std::uint64_t snapshot_execution_owner_epoch{0U};
  std::uint8_t execution_mode{msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD};
  std::uint8_t execution_reason{msg::MppiTrajectoryHorizon::EXECUTION_REASON_NONE};
  bool stationary_position_hold{false};
  bool valid{false};
};

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
assessPlannedHorizonSupersession(const ProductionMppiExecutionHorizonOwner& owner,
                                 const bool owner_witnessed,
                                 const std::int64_t now_ns) noexcept {
  if (!owner.valid ||
      owner.execution_mode != msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED) {
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
  const ProductionMppiExecutionHorizonOwner* owner{nullptr};
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

enum class ProductionMppiExecutionMode : std::uint8_t {
  kPlanned = msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED,
  kPositionHold = msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD,
  kRevoked = msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED,
};

enum class ProductionMppiExecutionReason : std::uint8_t {
  kNone = msg::MppiTrajectoryHorizon::EXECUTION_REASON_NONE,
  kNoExecutableHorizon =
      msg::MppiTrajectoryHorizon::EXECUTION_REASON_NO_EXECUTABLE_HORIZON,
  kCooperativePassageYield =
      msg::MppiTrajectoryHorizon::EXECUTION_REASON_COOPERATIVE_PASSAGE_YIELD,
  kGoalCapture = msg::MppiTrajectoryHorizon::EXECUTION_REASON_GOAL_CAPTURE,
  kNoExecutableRoute = msg::MppiTrajectoryHorizon::EXECUTION_REASON_NO_EXECUTABLE_ROUTE,
  kUnavailableWorld = msg::MppiTrajectoryHorizon::EXECUTION_REASON_UNAVAILABLE_WORLD,
};

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
  std::vector<mppi::State> horizon;
  ProductionMppiExecutionMode mode{ProductionMppiExecutionMode::kPlanned};
  ProductionMppiExecutionReason reason{ProductionMppiExecutionReason::kNone};
  std::size_t planned_control_count{0U};
  std::size_t nominal_prefix_control_count{0U};
  std::size_t arrival_control_count{0U};
  std::size_t arrival_shaping_attempts{0U};
  mppi::Control first_control{};
  std::uint64_t latest_lidar_obstacle_sequence{0U};
  std::size_t latest_lidar_obstacle_hit_count{0U};
  double latest_lidar_obstacle_age_ms{-1.0};
  bool finite_path_validation_backoff{false};
  mppi::FiniteExecutionPathStatus finite_path_validation_status{
      mppi::FiniteExecutionPathStatus::kInvalidContract};
  mppi::FiniteExecutionPathStatus finite_path_first_failed_validation_status{
      mppi::FiniteExecutionPathStatus::kValid};
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
    const ProductionMppiAppliedControl& control,
    const ProductionMppiExecutionHorizonOwner& owner, std::int64_t now_ns,
    double maximum_age_ms) noexcept;

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
authoritativeBodyAxisForExecution(const ProductionMppiAppliedControl& applied_control,
                                  const ProductionMppiExecutionHorizonOwner& owner,
                                  const ProductionMppiNavigation& navigation,
                                  std::int64_t now_ns, double maximum_control_age_ms,
                                  double maximum_pose_age_ms) noexcept;

} // namespace drone_city_nav
