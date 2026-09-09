#include "drone_city_nav/pending_certified_route_3d.hpp"

#include "drone_city_nav/execution_supervisor_3d.hpp"

#include <limits>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool
sameDirectTrackingIdentity(const DirectTrackingOwnerIdentity3D& first,
                           const DirectTrackingOwnerIdentity3D& second) noexcept {
  return first.mission_epoch == second.mission_epoch &&
         first.assignment_generation == second.assignment_generation &&
         first.target_detection_id == second.target_detection_id &&
         first.target_track_id == second.target_track_id &&
         first.objective_sample_sequence == second.objective_sample_sequence &&
         first.line_of_sight_generation == second.line_of_sight_generation;
}

template<typename ExecutionStore>
[[nodiscard]] PendingCertifiedRouteRecoveryResult3D
recoverPendingCertifiedRouteLivenessImpl(
    ExecutionStore& store,
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    const PendingCertifiedRouteRecoveryObservation3D& observation) {
  if (observation.direct_tracking_requested || observation.execution_owner_available ||
      observation.pending_activation) {
    return {};
  }
  if (expected_pending == nullptr) {
    // A caller-local null does not prove that the shared pending slot is empty: a
    // newer route may have won publication after the caller's earlier read or
    // failed acknowledgement. Confirm the shared state at this linearization
    // point before authorizing another successor request.
    const bool mailbox_empty = store.pending() == nullptr;
    return PendingCertifiedRouteRecoveryResult3D{
        .pending_acknowledged = false,
        .request_successor = mailbox_empty,
    };
  }
  const bool acknowledged = store.acknowledgePendingIfSame(expected_pending);
  return PendingCertifiedRouteRecoveryResult3D{
      .pending_acknowledged = acknowledged,
      .request_successor = acknowledged,
  };
}

} // namespace

bool PendingCertifiedRoute3D::valid() const noexcept {
  if (publication_sequence == 0U || base_execution_owner_epoch == 0U ||
      !route.valid() ||
      base_route_generation == std::numeric_limits<std::uint64_t>::max()) {
    return false;
  }
  const bool successor_generation =
      route.identity.generation == base_route_generation + 1U;
  switch (base_kind) {
    case PendingExecutionBaseKind3D::kEmpty:
      return base_route_generation == 0U && base_geometry_revision == 0U &&
             base_continuity_id == 0U && !base_direct_tracking_identity.has_value() &&
             !route_splice.has_value() && successor_generation;
    case PendingExecutionBaseKind3D::kRoute:
      return base_route_generation != 0U && base_geometry_revision != 0U &&
             base_continuity_id != 0U && !base_direct_tracking_identity.has_value() &&
             route_splice.has_value() && route_splice->structurallyValid() &&
             route_splice->base_route_generation == base_route_generation &&
             route_splice->base_geometry_revision == base_geometry_revision &&
             route_splice->base_continuity_id == base_continuity_id &&
             route_splice->successor_route_generation == route.identity.generation &&
             route_splice->successor_geometry_revision ==
                 route.geometry->compiled_trajectory_revision &&
             route_splice->successor_continuity_id == route.continuity_id &&
             successor_generation;
    case PendingExecutionBaseKind3D::kRouteHandoff:
      return base_route_generation != 0U && base_geometry_revision != 0U &&
             base_continuity_id != 0U && !base_direct_tracking_identity.has_value() &&
             !route_splice.has_value() && successor_generation;
    case PendingExecutionBaseKind3D::kDirectTracking:
      return base_geometry_revision == 0U && base_continuity_id == 0U &&
             base_direct_tracking_identity.has_value() &&
             base_direct_tracking_identity->valid() && !route_splice.has_value() &&
             successor_generation;
    case PendingExecutionBaseKind3D::kStationaryHold:
    case PendingExecutionBaseKind3D::kStop:
    case PendingExecutionBaseKind3D::kRevoked:
      return base_geometry_revision == 0U && base_continuity_id == 0U &&
             !base_direct_tracking_identity.has_value() && !route_splice.has_value() &&
             successor_generation;
  }
  return false;
}

std::string_view
pendingRouteEligibility3DName(const PendingRouteEligibility3D eligibility) noexcept {
  switch (eligibility) {
    case PendingRouteEligibility3D::kEligible:
      return "eligible";
    case PendingRouteEligibility3D::kInvalidPending:
      return "invalid_pending";
    case PendingRouteEligibility3D::kInvalidSnapshot:
      return "invalid_snapshot";
    case PendingRouteEligibility3D::kOwnerEpochMismatch:
      return "owner_epoch_mismatch";
    case PendingRouteEligibility3D::kBaseGenerationMismatch:
      return "base_generation_mismatch";
    case PendingRouteEligibility3D::kBaseOwnerMismatch:
      return "base_owner_mismatch";
  }
  return "unknown";
}

PendingRouteEligibility3D
pendingCertifiedRouteEligibility3D(const PendingCertifiedRoute3D& pending,
                                   const ExecutionPlan3D& snapshot) noexcept {
  if (!pending.valid()) {
    return PendingRouteEligibility3D::kInvalidPending;
  }
  if (!snapshot.valid()) {
    return PendingRouteEligibility3D::kInvalidSnapshot;
  }
  if (snapshot.execution_owner_epoch != pending.base_execution_owner_epoch) {
    return PendingRouteEligibility3D::kOwnerEpochMismatch;
  }
  if (snapshot.routeGenerationHighWater() != pending.base_route_generation) {
    return PendingRouteEligibility3D::kBaseGenerationMismatch;
  }
  return pendingCertifiedRouteEligible3D(pending, snapshot)
             ? PendingRouteEligibility3D::kEligible
             : PendingRouteEligibility3D::kBaseOwnerMismatch;
}

bool pendingCertifiedRouteEligible3D(const PendingCertifiedRoute3D& pending,
                                     const ExecutionPlan3D& snapshot) noexcept {
  if (!pending.valid() || !snapshot.valid()) {
    return false;
  }
  if (snapshot.execution_owner_epoch != pending.base_execution_owner_epoch ||
      snapshot.routeGenerationHighWater() != pending.base_route_generation) {
    return false;
  }
  const CertifiedRouteSuffix3D* const route = snapshot.route();
  const DirectTrackingFiniteExecution3D* const direct =
      snapshot.directTrackingExecution();
  const StationaryExecutionHold3D* const hold = snapshot.stationaryHold();
  switch (pending.base_kind) {
    case PendingExecutionBaseKind3D::kEmpty:
      return snapshot.phase() == ExecutionRoutePhase3D::kAwaitingSuccessor &&
             route == nullptr && snapshot.finiteExecution() == nullptr &&
             direct == nullptr && hold == nullptr;
    case PendingExecutionBaseKind3D::kRoute:
      return executionRouteAcceptsCertifiedReplacement3D(snapshot) &&
             route != nullptr && route->geometry != nullptr &&
             route->identity.generation == pending.base_route_generation &&
             route->geometry->compiled_trajectory_revision ==
                 pending.base_geometry_revision &&
             route->continuity_id == pending.base_continuity_id &&
             pending.route_splice.has_value() &&
             pending.route_splice->validFor(*route, pending.route);
    case PendingExecutionBaseKind3D::kRouteHandoff:
      if (pending.route_splice.has_value()) {
        return false;
      }
      // The base route may have been revoked by physical evidence after the
      // replacement was certified. The replacement was planned from the
      // vehicle and is re-validated on activation, so a revoked, empty plan
      // whose last route generation is the base still hands it off.
      if (snapshot.phase() == ExecutionRoutePhase3D::kRevoked && route == nullptr &&
          snapshot.finiteExecution() == nullptr && direct == nullptr &&
          hold == nullptr) {
        return true;
      }
      return executionRouteAcceptsCertifiedReplacement3D(snapshot) &&
             route != nullptr && route->geometry != nullptr &&
             route->identity.generation == pending.base_route_generation &&
             route->geometry->compiled_trajectory_revision ==
                 pending.base_geometry_revision &&
             route->continuity_id == pending.base_continuity_id;
    case PendingExecutionBaseKind3D::kDirectTracking:
      return snapshot.phase() == ExecutionRoutePhase3D::kDirectTracking &&
             direct != nullptr && pending.base_direct_tracking_identity.has_value() &&
             sameDirectTrackingIdentity(*pending.base_direct_tracking_identity,
                                        direct->identity);
    case PendingExecutionBaseKind3D::kStationaryHold:
      return snapshot.phase() == ExecutionRoutePhase3D::kStopped && hold != nullptr &&
             hold->hold_id == pending.base_execution_owner_epoch;
    case PendingExecutionBaseKind3D::kStop:
      // A stop follows no route, so a successor is planned from the vehicle and
      // hands off from the stop itself, flown to rest or still braking.
      return snapshot.phase() == ExecutionRoutePhase3D::kStopping &&
             snapshot.stopExecution() != nullptr && route == nullptr &&
             snapshot.finiteExecution() == nullptr && direct == nullptr &&
             hold == nullptr;
    case PendingExecutionBaseKind3D::kRevoked:
      return snapshot.phase() == ExecutionRoutePhase3D::kRevoked && route == nullptr &&
             snapshot.finiteExecution() == nullptr && direct == nullptr &&
             hold == nullptr;
  }
  return false;
}

bool pendingCertifiedRouteRetainsSnapshotCertificate3D(
    const PendingCertifiedRoute3D& pending) noexcept {
  return pending.valid() &&
         pending.base_kind == PendingExecutionBaseKind3D::kRouteHandoff;
}

bool pendingRouteActivationStructurallyRejected3D(
    const ExecutionRouteTransitionStatus3D status,
    const ExecutionRouteTransitionDetail3D detail) noexcept {
  if (status == ExecutionRouteTransitionStatus3D::kInvalidCandidate) {
    return true;
  }
  if (status != ExecutionRouteTransitionStatus3D::kCertificateRegression) {
    return false;
  }
  switch (detail) {
    case ExecutionRouteTransitionDetail3D::kSuccessorValidationPolicyMismatch:
    case ExecutionRouteTransitionDetail3D::kSuccessorCertificateKindMismatch:
    case ExecutionRouteTransitionDetail3D::kSuccessorProducerMismatch:
    case ExecutionRouteTransitionDetail3D::kSuccessorCertificateOlder:
    case ExecutionRouteTransitionDetail3D::kSuccessorWorldContentMismatch:
      return true;
    default:
      return false;
  }
}

PendingCertifiedRouteRecoveryResult3D recoverPendingCertifiedRouteLiveness3D(
    ExecutionSupervisor3D& supervisor,
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    const PendingCertifiedRouteRecoveryObservation3D& observation) {
  return recoverPendingCertifiedRouteLivenessImpl(supervisor, expected_pending,
                                                  observation);
}

} // namespace drone_city_nav
