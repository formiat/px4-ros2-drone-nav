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
                 route.geometry->executable_geometry_revision &&
             route_splice->successor_continuity_id == route.continuity_id &&
             successor_generation;
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

bool pendingCertifiedRouteEligible3D(
    const PendingCertifiedRoute3D& pending,
    const ExecutionRouteSnapshot3D& snapshot) noexcept {
  if (!pending.valid() || !snapshot.valid()) {
    return false;
  }
  if (snapshot.execution_owner_epoch != pending.base_execution_owner_epoch ||
      snapshot.routeGenerationHighWater() != pending.base_route_generation) {
    return false;
  }
  switch (pending.base_kind) {
    case PendingExecutionBaseKind3D::kEmpty:
      return snapshot.phase == ExecutionRoutePhase3D::kAwaitingSuccessor &&
             !snapshot.route.has_value() && !snapshot.finite_execution.has_value() &&
             !snapshot.direct_tracking_execution.has_value() &&
             !snapshot.stationary_hold.has_value();
    case PendingExecutionBaseKind3D::kRoute:
      return (snapshot.phase == ExecutionRoutePhase3D::kFollowing ||
              snapshot.phase == ExecutionRoutePhase3D::kAwaitingSuccessor ||
              snapshot.phase == ExecutionRoutePhase3D::kBraking ||
              (snapshot.phase == ExecutionRoutePhase3D::kStopped &&
               snapshot.route.has_value() &&
               snapshot.route->planned_endpoint_semantics ==
                   RouteEndpointSemantics3D::kObservationStop)) &&
             snapshot.route.has_value() && snapshot.route->geometry != nullptr &&
             snapshot.route->identity.generation == pending.base_route_generation &&
             snapshot.route->geometry->executable_geometry_revision ==
                 pending.base_geometry_revision &&
             snapshot.route->continuity_id == pending.base_continuity_id &&
             pending.route_splice.has_value() &&
             pending.route_splice->validFor(*snapshot.route, pending.route);
    case PendingExecutionBaseKind3D::kDirectTracking:
      return snapshot.phase == ExecutionRoutePhase3D::kDirectTracking &&
             snapshot.direct_tracking_execution.has_value() &&
             pending.base_direct_tracking_identity.has_value() &&
             sameDirectTrackingIdentity(*pending.base_direct_tracking_identity,
                                        snapshot.direct_tracking_execution->identity);
    case PendingExecutionBaseKind3D::kStationaryHold:
      return snapshot.phase == ExecutionRoutePhase3D::kStopped &&
             snapshot.stationary_hold.has_value() &&
             snapshot.stationary_hold->hold_id == pending.base_execution_owner_epoch &&
             !snapshot.route.has_value() && !snapshot.finite_execution.has_value() &&
             !snapshot.direct_tracking_execution.has_value();
    case PendingExecutionBaseKind3D::kRevoked:
      return snapshot.phase == ExecutionRoutePhase3D::kRevoked &&
             !snapshot.route.has_value() && !snapshot.finite_execution.has_value() &&
             !snapshot.direct_tracking_execution.has_value() &&
             !snapshot.stationary_hold.has_value();
  }
  return false;
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
  if (sealed->publication_sequence <= last_accepted_publication_sequence_) {
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
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& expected_snapshot,
    const ExecutionRouteTransitionResult3D& transition) {
  if (expected_pending == nullptr || expected_snapshot == nullptr ||
      transition.next == nullptr) {
    return false;
  }
  const std::scoped_lock lock{mutex_};
  const CertifiedRouteSuffix3D* const committed_route =
      transition.next->route.has_value()
          ? std::addressof(transition.next->route.value())
          : nullptr;
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
