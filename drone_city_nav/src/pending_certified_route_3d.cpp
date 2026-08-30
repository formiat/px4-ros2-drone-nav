#include "drone_city_nav/pending_certified_route_3d.hpp"

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
    case PendingExecutionBaseKind3D::kRevoked:
      return base_geometry_revision == 0U && base_continuity_id == 0U &&
             !base_direct_tracking_identity.has_value() && !route_splice.has_value() &&
             successor_generation;
  }
  return false;
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
      return executionRouteAcceptsCertifiedReplacement3D(snapshot) &&
             route != nullptr && route->geometry != nullptr &&
             route->identity.generation == pending.base_route_generation &&
             route->geometry->compiled_trajectory_revision ==
                 pending.base_geometry_revision &&
             route->continuity_id == pending.base_continuity_id &&
             !pending.route_splice.has_value();
    case PendingExecutionBaseKind3D::kDirectTracking:
      return snapshot.phase() == ExecutionRoutePhase3D::kDirectTracking &&
             direct != nullptr && pending.base_direct_tracking_identity.has_value() &&
             sameDirectTrackingIdentity(*pending.base_direct_tracking_identity,
                                        direct->identity);
    case PendingExecutionBaseKind3D::kStationaryHold:
      return snapshot.phase() == ExecutionRoutePhase3D::kStopped && hold != nullptr &&
             hold->hold_id == pending.base_execution_owner_epoch;
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

bool PendingCertifiedRouteMailbox3D::publish(
    std::shared_ptr<const PendingCertifiedRoute3D> candidate) {
  if (candidate == nullptr || !candidate->valid()) {
    return false;
  }
  // Seal the publication at the mailbox boundary. A shared_ptr<const T> can
  // still have a mutable alias owned by the caller; retaining the caller's
  // control block would therefore not make the pending route immutable.
  const auto sealed = std::make_shared<const PendingCertifiedRoute3D>(*candidate);
  if (!sealed->valid()) {
    return false;
  }
  const std::scoped_lock lock{mutex_};
  // A certified candidate is an execution transaction, not a latest-value
  // estimate. Keep the first resident identity until execution either commits
  // or explicitly acknowledges it; a newer planning completion cannot silently
  // displace a route that is already waiting for admission.
  if (pending_ != nullptr ||
      sealed->publication_sequence <= last_accepted_publication_sequence_) {
    return false;
  }
  pending_ = sealed;
  last_accepted_publication_sequence_ = sealed->publication_sequence;
  return true;
}

std::shared_ptr<const PendingCertifiedRoute3D>
PendingCertifiedRouteMailbox3D::snapshot() const {
  const std::scoped_lock lock{mutex_};
  return pending_;
}

bool PendingCertifiedRouteMailbox3D::acknowledgeIfSame(
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected) {
  if (expected == nullptr) {
    return false;
  }
  const std::scoped_lock lock{mutex_};
  if (pending_ != expected) {
    return false;
  }
  pending_.reset();
  return true;
}

bool PendingCertifiedRouteMailbox3D::commitExecutionIfSame(
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    ExecutionRouteSnapshotStore3D& execution_store,
    const std::shared_ptr<const ExecutionPlan3D>& expected_snapshot,
    const ExecutionRouteTransitionResult3D& transition) {
  if (expected_pending == nullptr || expected_snapshot == nullptr ||
      transition.next == nullptr) {
    return false;
  }
  const std::scoped_lock lock{mutex_};
  const CertifiedRouteSuffix3D* const committed_route = transition.next->route();
  const bool committed_route_is_pending_revision =
      committed_route != nullptr &&
      (committed_route->route_instance_id ==
           expected_pending->route.route_instance_id ||
       committed_route->parent_route_instance_id ==
           std::optional<RouteInstanceId3D>{expected_pending->route.route_instance_id});
  if (pending_ != expected_pending ||
      !pendingCertifiedRouteEligible3D(*expected_pending, *expected_snapshot) ||
      !committed_route_is_pending_revision) {
    return false;
  }
  if (execution_store.publish(expected_snapshot, transition) !=
      ExecutionRoutePublicationStatus3D::kPublished) {
    return false;
  }
  pending_.reset();
  return true;
}

PendingCertifiedRouteRecoveryResult3D recoverPendingCertifiedRouteLiveness3D(
    PendingCertifiedRouteMailbox3D& mailbox,
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    const PendingCertifiedRouteRecoveryObservation3D& observation) {
  if (observation.direct_tracking_requested || observation.execution_owner_available ||
      observation.pending_activation) {
    return {};
  }
  if (expected_pending == nullptr) {
    // A caller-local null does not prove that the shared mailbox is empty: a
    // newer route may have won publication after the caller's earlier read or
    // failed acknowledgement. Confirm the shared state at this linearization
    // point before authorizing another successor request.
    const bool mailbox_empty = mailbox.snapshot() == nullptr;
    return PendingCertifiedRouteRecoveryResult3D{
        .pending_acknowledged = false,
        .request_successor = mailbox_empty,
    };
  }
  const bool acknowledged = mailbox.acknowledgeIfSame(expected_pending);
  return PendingCertifiedRouteRecoveryResult3D{
      .pending_acknowledged = acknowledged,
      .request_successor = acknowledged,
  };
}

} // namespace drone_city_nav
