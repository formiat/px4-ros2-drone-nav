#pragma once

#include "drone_city_nav/execution_plan_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"
#include "drone_city_nav/tracking_error_tube_handoff_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string_view>

namespace drone_city_nav {

struct ExecutionRouteActivation3D {
  std::uint64_t route_generation{0U};
  MaterializedRouteProposal3D proposal{};
  std::shared_ptr<const CompiledTrajectory3D> geometry;
  std::shared_ptr<const RouteDecorations3D> decorations;
  RouteActivationObservation3D observation{};
  RouteContinuityLineage3D continuity_lineage{};
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  std::optional<RouteOwnerIdentity3D> retained_route_owner;
};

struct FiniteExecutionCertification3D {
  std::uint64_t trajectory_revision{0U};
  FiniteMotionHorizon3D horizon{};
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::int64_t valid_from_ns{0};
  FiniteExecutionKind3D kind{FiniteExecutionKind3D::kNominal};
};

struct FiniteExecutionPlanCertification3D {
  FiniteExecutionCertification3D command_horizon{};
  FiniteMotionHorizon3D braking_tail{};
};

enum class FiniteExecutionCertificationStatus3D : std::uint8_t {
  kCertified,
  kInvalidInput,
  kTargetRelationRejected,
  kProgressRelationRejected,
  kEvidenceContractRejected,
  kCollisionPolicyInvalid,
  kHorizonContractRejected,
  kInitialStateMismatch,
  kExecutionBindingRejected,
  kRouteAdherenceRejected,
  kTrackingTubeHandoffRejected,
  kTerminalBoundaryInvalid,
  kPathValidationRejected,
  kStaticWorldFingerprintMismatch,
  kRawValidationLineageRejected,
  kValidationLineageRejected,
  kValidationContractInvalid,
  kInvalidArtifact,
};

enum class FiniteExecutionRouteAdherenceStatus3D : std::uint8_t {
  kNotEvaluated,
  kAccepted,
  kInvalidInput,
  kInitialProjectionInvalid,
  kInitialCrossTrackExceeded,
  kInitialStationMismatch,
  kInitialConstraintRejected,
  kNonFiniteSegment,
  kProjectionInvalid,
  kStationRegression,
  kCrossTrackExceeded,
  kTrackingTubeExceeded,
  kConstraintRejected,
  kPassageCrossingRejected,
  kTerminalCrossTrackExceeded,
};

struct FiniteExecutionCertificationResult3D {
  FiniteExecutionCertificationStatus3D status{
      FiniteExecutionCertificationStatus3D::kInvalidInput};
  // Which dynamics law a kHorizonContractRejected verdict broke. The physical
  // path validator applies the same law, so the two stages agree; carrying the
  // reason out is what makes a refusal readable in the log.
  MotionDynamicsConsistency3D dynamics_consistency{
      MotionDynamicsConsistency3D::kConsistent};
  std::optional<FiniteExecutionState3D> execution;
  FiniteExecutionRouteAdherenceStatus3D route_adherence_status{
      FiniteExecutionRouteAdherenceStatus3D::kNotEvaluated};
  std::size_t route_adherence_failure_state_index{0U};
  double route_adherence_failure_distance_m{-1.0};

  [[nodiscard]] bool certified() const noexcept;
};

struct FiniteExecutionPlanCertificationResult3D {
  FiniteExecutionCertificationResult3D command_horizon{};
  FiniteExecutionCertificationResult3D braking_tail{};
  std::optional<FiniteExecutionPlan3D> plan;

  [[nodiscard]] bool certified() const noexcept;
};

struct DirectTrackingExecutionCertification3D {
  DirectTrackingOwnerIdentity3D identity{};
  std::uint64_t trajectory_revision{0U};
  Point3 target{};
  FiniteMotionHorizon3D horizon{};
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::int64_t valid_from_ns{0};
  FiniteExecutionKind3D kind{FiniteExecutionKind3D::kNominal};
};

// Everything a stop is certified from: the braking horizon itself and the
// evidence it is swept against. No route, no adherence corridor, no route
// certificate: a stop answers only to physics.
struct StopExecutionCertification3D {
  std::uint64_t trajectory_revision{0U};
  FiniteMotionHorizon3D horizon{};
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::int64_t valid_from_ns{0};
  // Require the pose the stop rests in to keep the whole margin the envelope
  // carries over the hull. It is a preference, not a rule: a stop that rests
  // close to evidence is still better than no stop at all, so the caller drops
  // the requirement when no rung of the clearance ladder can meet it.
  bool require_rest_clearance{true};
  // How much of the policy envelope's clearance the braking sweep gives up:
  // 0 is the envelope as configured, 1 the physical body. A stop is the last
  // motion the vehicle can be given, so when the envelope cannot clear the
  // evidence around it the sweep gives up only as much clearance as it must.
  double clearance_reduction{0.0};
};

struct StationaryExecutionHoldCertification3D {
  Point3 position{};
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
};

[[nodiscard]] bool
compiledTrajectoryValid3D(const CompiledTrajectory3D& geometry,
                          const ActivatedRouteIdentity3D& identity) noexcept;

// Why a route could not be certified. A candidate the activation accepted and
// the handoff accepted, yet no certified route came of it, otherwise reaches
// the log as a bare handoff rejection with nothing to act on.
enum class RouteCertificationStatus3D : std::uint8_t {
  kNotAttempted,
  kCertified,
  kInvalidInput,
  kFootprintNotContained,
  kRawEvidenceLineageMismatch,
  kRawEvidenceNotCurrent,
  kStaticWorldMismatch,
  kIdentityRejected,
  kRetainedOwnerMismatch,
  kGeometryInvalid,
  kTubeFootprintNotContained,
  kPassageFootprintMismatch,
  kTrackingTubeWorldMismatch,
  kPassageGeometryMismatch,
  kDerivationFingerprintInvalid,
  kAssessmentRejected,
  kAssessmentPublicationIncompatible,
  kAssessmentObjectiveMismatch,
  kAssessmentProjectionInvalid,
  kAssessmentCrossTrackExceeded,
  kAssessmentRawWorldIncompatible,
  kAssessmentRawValidationRejected,
  kRawConnectorNotValidated,
  kRawSuffixNotValidated,
  kPolicyFingerprintInvalid,
  kStaticSuffixRejected,
  kInvalidArtifact,
};

[[nodiscard]] std::string_view
routeCertificationStatus3DName(RouteCertificationStatus3D status) noexcept;

// `status`, when given, names the verdict: kCertified, or the first rule the
// activation broke.
[[nodiscard]] std::optional<CertifiedRouteSuffix3D>
certifyExecutionRoute3D(const ExecutionRouteActivation3D& activation,
                        RouteCertificationStatus3D* status = nullptr);

[[nodiscard]] std::optional<CertifiedRouteSuffix3D> recertifyExecutionRoute3D(
    const CertifiedRouteSuffix3D& sealed_source,
    const RouteActivationObservation3D& observation,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world,
    RouteCertificationStatus3D* status = nullptr);

[[nodiscard]] std::optional<FiniteExecutionState3D>
certifyFiniteExecution3D(const ExecutionPlan3D& current,
                         const CertifiedRouteSuffix3D& target_route,
                         FiniteExecutionCertification3D certification);

[[nodiscard]] FiniteExecutionCertificationResult3D
certifyFiniteExecution3DDetailed(const ExecutionPlan3D& current,
                                 const CertifiedRouteSuffix3D& target_route,
                                 FiniteExecutionCertification3D certification);

[[nodiscard]] FiniteExecutionPlanCertificationResult3D
certifyFiniteExecutionPlan3DDetailed(const ExecutionPlan3D& current,
                                     const CertifiedRouteSuffix3D& target_route,
                                     FiniteExecutionPlanCertification3D certification);

// Certifies the command horizon once and takes the first of `braking_tails`,
// in order, the world admits as the plan's braking tail; the tails are the
// stops along the horizon, earliest first. With none admitted the result
// carries the last tail's rejection.
[[nodiscard]] FiniteExecutionPlanCertificationResult3D
certifyFiniteExecutionPlan3DDetailed(
    const ExecutionPlan3D& current, const CertifiedRouteSuffix3D& target_route,
    FiniteExecutionCertification3D command_horizon,
    std::span<const FiniteMotionHorizon3D> braking_tails);

[[nodiscard]] std::optional<FiniteExecutionState3D>
certifyFiniteExecution3D(const ExecutionPlan3D& current,
                         FiniteExecutionCertification3D certification);

[[nodiscard]] TrackingErrorTubeHandoffAssessment3D assessCertifiedTrackingTubeHandoff3D(
    const ExecutionPlan3D& current, const CertifiedRouteSuffix3D& target_route,
    const VersionedExecutionInput3D& current_execution_input) noexcept;

[[nodiscard]] std::optional<DirectTrackingFiniteExecution3D>
certifyDirectTrackingExecution3D(const ExecutionPlan3D& current,
                                 DirectTrackingExecutionCertification3D certification);

// Why a stop could not be certified. A stop is the last thing a vehicle with
// no executable route can do, so a refusal that reaches the log as a bare
// "next_plan_invalid" leaves nothing to act on.
enum class StopCertificationStatus3D : std::uint8_t {
  kCertified,
  kInvalidInput,
  kEvidenceContractRejected,
  kHorizonContractRejected,
  kInitialStateMismatch,
  kPathValidationRejected,
  kRestClearanceRejected,
  kValidationContractInvalid,
  kInvalidArtifact,
};

struct StopCertificationResult3D {
  StopCertificationStatus3D status{StopCertificationStatus3D::kInvalidInput};
  // Which dynamics law a kHorizonContractRejected verdict broke.
  MotionDynamicsConsistency3D dynamics_consistency{
      MotionDynamicsConsistency3D::kConsistent};
  // Which physical rule a kPathValidationRejected verdict broke.
  FiniteExecutionPathStatus3D path_validation_status{
      FiniteExecutionPathStatus3D::kValid};
  // How much of the envelope's clearance the verdict was reached with given
  // up; 1 means the physical body alone.
  double clearance_reduction{0.0};
  // No rung of the clearance ladder could rest the vehicle with the margin the
  // envelope carries over the hull, so the stop was certified without it.
  bool rest_clearance_relaxed{false};
  // Which segment of the braking path a rejected verdict was reached on. Zero
  // is the segment the vehicle is already on.
  std::size_t path_validation_failure_segment_index{0U};
  std::optional<StopExecution3D> execution;

  [[nodiscard]] bool certified() const noexcept {
    return status == StopCertificationStatus3D::kCertified && execution.has_value();
  }
};

[[nodiscard]] StopCertificationResult3D
certifyStopExecution3D(const ExecutionPlan3D& current,
                       StopExecutionCertification3D certification);

[[nodiscard]] std::string_view
stopCertificationStatus3DName(StopCertificationStatus3D status) noexcept;

[[nodiscard]] std::string_view finiteExecutionCertificationStatus3DName(
    FiniteExecutionCertificationStatus3D status) noexcept;

[[nodiscard]] std::string_view finiteExecutionRouteAdherenceStatus3DName(
    FiniteExecutionRouteAdherenceStatus3D status) noexcept;

} // namespace drone_city_nav
