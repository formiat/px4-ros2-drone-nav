#include "drone_city_nav/execution_route_snapshot_3d.hpp"

#include <limits>
#include <memory>
#include <mutex>
#include <string_view>
#include <utility>

namespace drone_city_nav {

std::shared_ptr<const ExecutionRouteSnapshot3D> makeInitialExecutionRouteSnapshot3D() {
  return std::make_shared<const ExecutionRouteSnapshot3D>(ExecutionRouteSnapshot3D{
      .version = 1U,
      .phase = ExecutionRoutePhase3D::kAwaitingSuccessor,
      .route = std::nullopt,
      .finite_execution = std::nullopt,
      .braking_fallback = std::nullopt,
      .direct_tracking_execution = std::nullopt,
      .stationary_hold = std::nullopt,
      .execution_owner_epoch = 1U,
      .route_generation_high_water = 0U,
  });
}

bool ExecutionRouteTransitionResult3D::applied() const noexcept {
  return authorized_ && status == ExecutionRouteTransitionStatus3D::kApplied &&
         predecessor != nullptr && next != nullptr;
}

ExecutionRouteTransitionResult3D::ExecutionRouteTransitionResult3D(
    const ExecutionRouteTransitionStatus3D status_value,
    const ExecutionRouteSnapshot3D* const predecessor_value,
    std::shared_ptr<const ExecutionRouteSnapshot3D> next_value)
    : status{status_value},
      predecessor{predecessor_value},
      next{std::move(next_value)},
      authorized_{true} {
}

RouteEndpointSemantics3D
executionRouteEndpointSemantics3D(const ExecutionRouteSnapshot3D& snapshot) noexcept {
  if (snapshot.finite_execution.has_value() &&
      snapshot.finite_execution->kind == FiniteExecutionKind3D::kEmergencyBrakeTail) {
    return RouteEndpointSemantics3D::kEmergencyBrakeTail;
  }
  return snapshot.route.has_value() ? snapshot.route->planned_endpoint_semantics
                                    : RouteEndpointSemantics3D::kContinuation;
}

std::string_view finiteExecutionKind3DName(const FiniteExecutionKind3D kind) noexcept {
  switch (kind) {
    case FiniteExecutionKind3D::kNominal:
      return "nominal";
    case FiniteExecutionKind3D::kRetained:
      return "retained";
    case FiniteExecutionKind3D::kEmergencyBrakeTail:
      return "emergency_brake_tail";
  }
  return "invalid";
}

std::string_view executionRoutePhase3DName(const ExecutionRoutePhase3D phase) noexcept {
  switch (phase) {
    case ExecutionRoutePhase3D::kFollowing:
      return "following";
    case ExecutionRoutePhase3D::kDirectTracking:
      return "direct_tracking";
    case ExecutionRoutePhase3D::kAwaitingSuccessor:
      return "awaiting_successor";
    case ExecutionRoutePhase3D::kBraking:
      return "braking";
    case ExecutionRoutePhase3D::kStopped:
      return "stopped";
    case ExecutionRoutePhase3D::kRevoked:
      return "revoked";
  }
  return "invalid";
}

std::string_view executionRouteTransitionStatus3DName(
    const ExecutionRouteTransitionStatus3D status) noexcept {
  switch (status) {
    case ExecutionRouteTransitionStatus3D::kApplied:
      return "applied";
    case ExecutionRouteTransitionStatus3D::kNoChange:
      return "no_change";
    case ExecutionRouteTransitionStatus3D::kInvalidCurrentSnapshot:
      return "invalid_current_snapshot";
    case ExecutionRouteTransitionStatus3D::kInvalidCandidate:
      return "invalid_candidate";
    case ExecutionRouteTransitionStatus3D::kStaleSnapshotVersion:
      return "stale_snapshot_version";
    case ExecutionRouteTransitionStatus3D::kRouteGenerationMismatch:
      return "route_generation_mismatch";
    case ExecutionRouteTransitionStatus3D::kGeometryRevisionMismatch:
      return "geometry_revision_mismatch";
    case ExecutionRouteTransitionStatus3D::kNonMonotonicProgress:
      return "non_monotonic_progress";
    case ExecutionRouteTransitionStatus3D::kCertificateRegression:
      return "certificate_regression";
    case ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected:
      return "execution_assessment_rejected";
    case ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict:
      return "finite_execution_conflict";
    case ExecutionRouteTransitionStatus3D::kVersionExhausted:
      return "version_exhausted";
  }
  return "invalid";
}

ExecutionRouteSnapshotStore3D::ExecutionRouteSnapshotStore3D()
    : snapshot_(makeInitialExecutionRouteSnapshot3D()) {
}

std::shared_ptr<const ExecutionRouteSnapshot3D>
ExecutionRouteSnapshotStore3D::snapshot() const {
  const std::scoped_lock lock{mutex_};
  return snapshot_;
}

ExecutionRoutePublicationStatus3D ExecutionRouteSnapshotStore3D::publish(
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& expected_snapshot,
    const ExecutionRouteTransitionResult3D& transition) {
  const std::scoped_lock lock{mutex_};
  if (snapshot_ == nullptr || expected_snapshot == nullptr ||
      snapshot_ != expected_snapshot) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  if (!transition.applied() || !transition.next->publishable() ||
      expected_snapshot->version == std::numeric_limits<std::uint64_t>::max() ||
      transition.next->version <= expected_snapshot->version) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  if (transition.predecessor != expected_snapshot.get()) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  snapshot_ = transition.next;
  return ExecutionRoutePublicationStatus3D::kPublished;
}

} // namespace drone_city_nav
