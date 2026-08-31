#pragma once

#include "drone_city_nav/execution_plan_3d.hpp"
#include "drone_city_nav/tracking_error_tube_handoff_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
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
  kRawInvalidationContractRejected,
  kLifecycleBrakingContractRejected,
  kEvidenceContractRejected,
  kCollisionPolicyInvalid,
  kHorizonContractRejected,
  kInitialStateMismatch,
  kExecutionBindingRejected,
  kRawInvalidationConnectorRejected,
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

struct RawInvalidatedFiniteExecutionCertification3D {
  RouteLifecycleEvent3D invalidation{};
  std::shared_ptr<const VersionedObservedRawWorld3D> invalidating_observed_raw_world;
  FiniteExecutionCertification3D finite_execution{};
};

struct LifecycleBrakingFiniteExecutionCertification3D {
  RouteLifecycleEvent3D lifecycle_event{};
  FiniteExecutionCertification3D finite_execution{};
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

[[nodiscard]] std::optional<CertifiedRouteSuffix3D>
certifyExecutionRoute3D(const ExecutionRouteActivation3D& activation);

[[nodiscard]] std::optional<CertifiedRouteSuffix3D> recertifyExecutionRoute3D(
    const CertifiedRouteSuffix3D& sealed_source,
    const RouteActivationObservation3D& observation,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world);

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

[[nodiscard]] std::optional<FiniteExecutionState3D>
certifyFiniteExecution3D(const ExecutionPlan3D& current,
                         FiniteExecutionCertification3D certification);

[[nodiscard]] TrackingErrorTubeHandoffAssessment3D assessCertifiedTrackingTubeHandoff3D(
    const ExecutionPlan3D& current, const CertifiedRouteSuffix3D& target_route,
    const VersionedExecutionInput3D& current_execution_input) noexcept;

[[nodiscard]] std::optional<FiniteExecutionState3D>
certifyRawInvalidatedFiniteExecution3D(
    const ExecutionPlan3D& current,
    RawInvalidatedFiniteExecutionCertification3D certification);

[[nodiscard]] FiniteExecutionCertificationResult3D
certifyRawInvalidatedFiniteExecution3DDetailed(
    const ExecutionPlan3D& current,
    RawInvalidatedFiniteExecutionCertification3D certification);

[[nodiscard]] std::optional<FiniteExecutionState3D>
certifyLifecycleBrakingFiniteExecution3D(
    const ExecutionPlan3D& current,
    LifecycleBrakingFiniteExecutionCertification3D certification);

[[nodiscard]] FiniteExecutionCertificationResult3D
certifyLifecycleBrakingFiniteExecution3DDetailed(
    const ExecutionPlan3D& current,
    LifecycleBrakingFiniteExecutionCertification3D certification);

[[nodiscard]] std::optional<DirectTrackingFiniteExecution3D>
certifyDirectTrackingExecution3D(const ExecutionPlan3D& current,
                                 DirectTrackingExecutionCertification3D certification);

[[nodiscard]] std::string_view finiteExecutionCertificationStatus3DName(
    FiniteExecutionCertificationStatus3D status) noexcept;

[[nodiscard]] std::string_view finiteExecutionRouteAdherenceStatus3DName(
    FiniteExecutionRouteAdherenceStatus3D status) noexcept;

} // namespace drone_city_nav
