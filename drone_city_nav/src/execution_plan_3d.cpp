#include "drone_city_nav/execution_route_transitions_3d.hpp"

#include <memory>
#include <string_view>
#include <utility>

namespace drone_city_nav {

std::shared_ptr<const ExecutionPlan3D> makeInitialExecutionRouteSnapshot3D() {
  return std::make_shared<const ExecutionPlan3D>(ExecutionPlan3D{
      .version = 1U,
      .state = AwaitingSuccessorPlan3D{},
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
    const ExecutionPlan3D* const predecessor_value,
    std::shared_ptr<const ExecutionPlan3D> next_value)
    : status{status_value},
      predecessor{predecessor_value},
      next{std::move(next_value)},
      authorized_{true} {
}

RouteEndpointSemantics3D
executionRouteEndpointSemantics3D(const ExecutionPlan3D& snapshot) noexcept {
  const FiniteExecutionState3D* const finite_execution = snapshot.finiteExecution();
  if (finite_execution != nullptr &&
      finite_execution->kind == FiniteExecutionKind3D::kEmergencyBrakeTail) {
    return RouteEndpointSemantics3D::kEmergencyBrakeTail;
  }
  const CertifiedRouteSuffix3D* const route = snapshot.route();
  return route != nullptr ? route->planned_endpoint_semantics
                          : RouteEndpointSemantics3D::kContinuation;
}

bool executionRouteAcceptsCertifiedReplacement3D(
    const ExecutionPlan3D& snapshot) noexcept {
  const CertifiedRouteSuffix3D* const route = snapshot.route();
  if (route == nullptr) {
    return false;
  }
  // A vehicle stopped before the end of its route, whatever the route's
  // endpoint semantics, is exactly the state a certified replacement resolves:
  // it stopped because the remaining route could not be executed. Only a
  // route stopped at its own mission endpoint keeps the stop exclusive.
  constexpr double kEndpointStopToleranceM{0.5};
  return snapshot.phase() == ExecutionRoutePhase3D::kFollowing ||
         snapshot.phase() == ExecutionRoutePhase3D::kAwaitingSuccessor ||
         snapshot.phase() == ExecutionRoutePhase3D::kBraking ||
         (snapshot.phase() == ExecutionRoutePhase3D::kStopped &&
          (route->planned_endpoint_semantics == RouteEndpointSemantics3D::kLocalStop ||
           route->remainingM() > kEndpointStopToleranceM));
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

} // namespace drone_city_nav
