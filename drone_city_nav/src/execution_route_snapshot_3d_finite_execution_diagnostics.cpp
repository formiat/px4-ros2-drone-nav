#include "drone_city_nav/execution_route_certification_3d.hpp"

#include <string_view>

namespace drone_city_nav {

bool FiniteExecutionCertificationResult3D::certified() const noexcept {
  return status == FiniteExecutionCertificationStatus3D::kCertified &&
         execution.has_value();
}

bool FiniteExecutionPlanCertificationResult3D::certified() const noexcept {
  return command_horizon.certified() && braking_tail.certified() && plan.has_value();
}

std::string_view finiteExecutionCertificationStatus3DName(
    const FiniteExecutionCertificationStatus3D status) noexcept {
  switch (status) {
    case FiniteExecutionCertificationStatus3D::kCertified:
      return "certified";
    case FiniteExecutionCertificationStatus3D::kInvalidInput:
      return "invalid_input";
    case FiniteExecutionCertificationStatus3D::kTargetRelationRejected:
      return "target_relation_rejected";
    case FiniteExecutionCertificationStatus3D::kProgressRelationRejected:
      return "progress_relation_rejected";
    case FiniteExecutionCertificationStatus3D::kEvidenceContractRejected:
      return "evidence_contract_rejected";
    case FiniteExecutionCertificationStatus3D::kCollisionPolicyInvalid:
      return "collision_policy_invalid";
    case FiniteExecutionCertificationStatus3D::kHorizonContractRejected:
      return "horizon_contract_rejected";
    case FiniteExecutionCertificationStatus3D::kInitialStateMismatch:
      return "initial_state_mismatch";
    case FiniteExecutionCertificationStatus3D::kExecutionBindingRejected:
      return "execution_binding_rejected";
    case FiniteExecutionCertificationStatus3D::kRouteAdherenceRejected:
      return "route_adherence_rejected";
    case FiniteExecutionCertificationStatus3D::kTrackingTubeHandoffRejected:
      return "tracking_tube_handoff_rejected";
    case FiniteExecutionCertificationStatus3D::kTerminalBoundaryInvalid:
      return "terminal_boundary_invalid";
    case FiniteExecutionCertificationStatus3D::kPathValidationRejected:
      return "path_validation_rejected";
    case FiniteExecutionCertificationStatus3D::kStaticWorldFingerprintMismatch:
      return "static_world_fingerprint_mismatch";
    case FiniteExecutionCertificationStatus3D::kRawValidationLineageRejected:
      return "raw_validation_lineage_rejected";
    case FiniteExecutionCertificationStatus3D::kValidationLineageRejected:
      return "validation_lineage_rejected";
    case FiniteExecutionCertificationStatus3D::kValidationContractInvalid:
      return "validation_contract_invalid";
    case FiniteExecutionCertificationStatus3D::kInvalidArtifact:
      return "invalid_artifact";
  }
  return "unknown";
}

std::string_view finiteExecutionRouteAdherenceStatus3DName(
    const FiniteExecutionRouteAdherenceStatus3D status) noexcept {
  switch (status) {
    case FiniteExecutionRouteAdherenceStatus3D::kNotEvaluated:
      return "not_evaluated";
    case FiniteExecutionRouteAdherenceStatus3D::kAccepted:
      return "accepted";
    case FiniteExecutionRouteAdherenceStatus3D::kInvalidInput:
      return "invalid_input";
    case FiniteExecutionRouteAdherenceStatus3D::kInitialProjectionInvalid:
      return "initial_projection_invalid";
    case FiniteExecutionRouteAdherenceStatus3D::kInitialCrossTrackExceeded:
      return "initial_cross_track_exceeded";
    case FiniteExecutionRouteAdherenceStatus3D::kInitialStationMismatch:
      return "initial_station_mismatch";
    case FiniteExecutionRouteAdherenceStatus3D::kInitialConstraintRejected:
      return "initial_constraint_rejected";
    case FiniteExecutionRouteAdherenceStatus3D::kNonFiniteSegment:
      return "non_finite_segment";
    case FiniteExecutionRouteAdherenceStatus3D::kProjectionInvalid:
      return "projection_invalid";
    case FiniteExecutionRouteAdherenceStatus3D::kStationRegression:
      return "station_regression";
    case FiniteExecutionRouteAdherenceStatus3D::kCrossTrackExceeded:
      return "cross_track_exceeded";
    case FiniteExecutionRouteAdherenceStatus3D::kTrackingTubeExceeded:
      return "tracking_tube_exceeded";
    case FiniteExecutionRouteAdherenceStatus3D::kConstraintRejected:
      return "constraint_rejected";
    case FiniteExecutionRouteAdherenceStatus3D::kPassageCrossingRejected:
      return "passage_crossing_rejected";
    case FiniteExecutionRouteAdherenceStatus3D::kTerminalCrossTrackExceeded:
      return "terminal_cross_track_exceeded";
  }
  return "unknown";
}

} // namespace drone_city_nav
