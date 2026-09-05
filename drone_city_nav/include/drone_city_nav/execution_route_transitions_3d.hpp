#pragma once

#include "drone_city_nav/execution_route_certification_3d.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <variant>

namespace drone_city_nav {

struct CertifiedRouteSplice3D;

enum class ExecutionRouteTransitionStatus3D : std::uint8_t {
  kApplied,
  kNoChange,
  kInvalidCurrentSnapshot,
  kInvalidCandidate,
  kStaleSnapshotVersion,
  kRouteGenerationMismatch,
  kGeometryRevisionMismatch,
  kNonMonotonicProgress,
  kCertificateRegression,
  kExecutionAssessmentRejected,
  kFiniteExecutionConflict,
  kVersionExhausted,
};

// Names the contract check that rejected a transition. A status such as
// kInvalidCandidate covers several unrelated predicates; the detail tells a
// reader of the diagnostics which one failed without re-running the reducer.
enum class ExecutionRouteTransitionDetail3D : std::uint8_t {
  kNone,
  kNextPlanInvalid,
  kActiveIntentConflict,
  kSuccessorIdentityMismatch,
  kSpliceNotReady,
  kReplacementWithoutSplice,
  kActivationIdentityMismatch,
  kActivationBindingInvalid,
  kProgressExecutionInputStale,
  kProgressWorldOlderThanCertificate,
  kProgressValidationPolicyChanged,
  kProgressPassageGeometryChanged,
  kProgressUnexpectedObservedWorld,
  kProgressObservedTravelInvalid,
  kProgressCompositionMismatch,
  kLifecycleEventUnknown,
  kHoldCertificationStale,
  kHoldPointUnsafe,
  // A successor's evidence is older than the resident route's. The successor
  // kinds name the successor's own certificate; the execution kinds name the
  // finite execution it is offered with, which the next tick refreshes.
  kSuccessorValidationPolicyMismatch,
  kSuccessorCertificateKindMismatch,
  kSuccessorProducerMismatch,
  kSuccessorCertificateOlder,
  kSuccessorWorldContentMismatch,
  kSuccessorExecutionEvidenceOlder,
  kSuccessorExecutionInputOlder,
  kResidentProgressInputMissing,
  kProgressRawRevisionOlder,
};

[[nodiscard]] std::string_view
executionRouteTransitionDetail3DName(ExecutionRouteTransitionDetail3D detail) noexcept;

struct ExecutionRouteTransitionGuard3D {
  std::uint64_t expected_snapshot_version{0U};
  std::uint64_t expected_route_generation{0U};
  std::uint64_t expected_geometry_revision{0U};
};

struct ActivateCertifiedRouteCommand3D {
  std::uint64_t expected_snapshot_version{0U};
  CertifiedRouteSuffix3D candidate{};
  FiniteExecutionPlan3D candidate_execution{};
};

struct AdvanceCertifiedRouteCommand3D {
  ExecutionRouteTransitionGuard3D guard{};
  RouteExecutionObservation3D observation{};
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
};

struct ReplaceFiniteExecutionPlanCommand3D {
  ExecutionRouteTransitionGuard3D guard{};
  FiniteExecutionPlan3D execution{};
};

struct CompleteCertifiedRouteCommand3D {
  ExecutionRouteTransitionGuard3D guard{};
  RouteLifecycleEvent3D event{};
};

struct ReplaceCertifiedRouteCommand3D {
  ExecutionRouteTransitionGuard3D guard{};
  CertifiedRouteSuffix3D successor{};
  FiniteExecutionPlan3D successor_execution{};
  std::shared_ptr<const CertifiedRouteSplice3D> splice;
};

struct ReplaceCertifiedRouteAtHandoffCommand3D {
  ExecutionRouteTransitionGuard3D guard{};
  CertifiedRouteSuffix3D successor{};
  FiniteExecutionPlan3D successor_execution{};
};

struct TransferToDirectTrackingCommand3D {
  std::uint64_t expected_snapshot_version{0U};
  DirectTrackingFiniteExecution3D direct_execution{};
};

struct ReplaceDirectTrackingExecutionCommand3D {
  std::uint64_t expected_snapshot_version{0U};
  DirectTrackingFiniteExecution3D direct_execution{};
};

struct TransferDirectTrackingToCertifiedRouteCommand3D {
  std::uint64_t expected_snapshot_version{0U};
  CertifiedRouteSuffix3D successor{};
  FiniteExecutionPlan3D successor_execution{};
};

struct TransferToExecutionHoldCommand3D {
  std::uint64_t expected_snapshot_version{0U};
  StationaryExecutionHoldCertification3D certification{};
};

struct ArmStationaryCaptureHoldCommand3D {
  std::uint64_t expected_snapshot_version{0U};
  StationaryExecutionHoldCertification3D certification{};
};

struct EnterStopExecutionCommand3D {
  std::uint64_t expected_snapshot_version{0U};
  StopExecutionCertification3D certification{};
};

struct RevokeExecutionCommand3D {
  std::uint64_t expected_snapshot_version{0U};
};

struct SuspendFiniteExecutionCommand3D {
  std::uint64_t expected_snapshot_version{0U};
};

using ExecutionPlanTransitionCommand3D = std::variant<
    ActivateCertifiedRouteCommand3D, AdvanceCertifiedRouteCommand3D,
    ReplaceFiniteExecutionPlanCommand3D, CompleteCertifiedRouteCommand3D,
    ReplaceCertifiedRouteCommand3D, ReplaceCertifiedRouteAtHandoffCommand3D,
    TransferToDirectTrackingCommand3D, ReplaceDirectTrackingExecutionCommand3D,
    TransferDirectTrackingToCertifiedRouteCommand3D, TransferToExecutionHoldCommand3D,
    ArmStationaryCaptureHoldCommand3D, EnterStopExecutionCommand3D,
    RevokeExecutionCommand3D, SuspendFiniteExecutionCommand3D>;

class ExecutionRouteTransitionFactory3D;
class RouteExecutionManager3D;

struct ExecutionRouteTransitionResult3D {
  ExecutionRouteTransitionResult3D() = default;

  const ExecutionRouteTransitionStatus3D status{
      ExecutionRouteTransitionStatus3D::kInvalidCandidate};
  const ExecutionRouteTransitionDetail3D detail{
      ExecutionRouteTransitionDetail3D::kNone};
  const ExecutionPlan3D* const predecessor{nullptr};
  const std::shared_ptr<const ExecutionPlan3D> next;

  [[nodiscard]] bool applied() const noexcept;

private:
  ExecutionRouteTransitionResult3D(ExecutionRouteTransitionStatus3D status_value,
                                   ExecutionRouteTransitionDetail3D detail_value,
                                   const ExecutionPlan3D* predecessor_value,
                                   std::shared_ptr<const ExecutionPlan3D> next_value);

  bool authorized_{false};

  friend class ExecutionRouteTransitionFactory3D;
  friend class RouteExecutionManager3D;
};

[[nodiscard]] ExecutionRouteTransitionResult3D
reduceExecutionPlan3D(const ExecutionPlan3D& current,
                      ExecutionPlanTransitionCommand3D command);

[[nodiscard]] ExecutionRouteTransitionResult3D activateCertifiedRoute3D(
    const ExecutionPlan3D& current, std::uint64_t expected_snapshot_version,
    CertifiedRouteSuffix3D candidate, FiniteExecutionPlan3D candidate_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D advanceCertifiedRoute3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    RouteExecutionObservation3D observation,
    std::shared_ptr<const VersionedExecutionInput3D> execution_input,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world);

[[nodiscard]] ExecutionRouteTransitionResult3D
replaceFiniteExecutionPlan3D(const ExecutionPlan3D& current,
                             const ExecutionRouteTransitionGuard3D& guard,
                             FiniteExecutionPlan3D execution);

// The route reached its own end: a continuation route parks its plan for the
// successor, a mission or local stop hands it to the certified terminal hold.
// Completion is the only lifecycle event a route retires on; every other one
// means the vehicle needs a stop, not a finished route.
[[nodiscard]] ExecutionRouteTransitionResult3D
completeCertifiedRoute3D(const ExecutionPlan3D& current,
                         const ExecutionRouteTransitionGuard3D& guard,
                         const RouteLifecycleEvent3D& event);

[[nodiscard]] ExecutionRouteTransitionResult3D replaceCertifiedRoute3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution,
    const CertifiedRouteSplice3D& splice);

[[nodiscard]] ExecutionRouteTransitionResult3D replaceCertifiedRouteAtHandoff3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D
transferToDirectTracking3D(const ExecutionPlan3D& current,
                           std::uint64_t expected_snapshot_version,
                           DirectTrackingFiniteExecution3D direct_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D
replaceDirectTrackingExecution3D(const ExecutionPlan3D& current,
                                 std::uint64_t expected_snapshot_version,
                                 DirectTrackingFiniteExecution3D direct_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D transferDirectTrackingToCertifiedRoute3D(
    const ExecutionPlan3D& current, std::uint64_t expected_snapshot_version,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D composeExecutionPlanTransition3D(
    const ExecutionPlan3D& resident,
    const ExecutionRouteTransitionResult3D& prepared_progress,
    const ExecutionRouteTransitionResult3D& prepared_execution_plan);

[[nodiscard]] ExecutionRouteTransitionResult3D
transferToExecutionHold3D(const ExecutionPlan3D& current,
                          std::uint64_t expected_snapshot_version,
                          StationaryExecutionHoldCertification3D certification);

[[nodiscard]] ExecutionRouteTransitionResult3D
armStationaryCaptureHold3D(const ExecutionPlan3D& current,
                           std::uint64_t expected_snapshot_version,
                           StationaryExecutionHoldCertification3D certification);

// Leaves whatever the plan was executing and takes ownership of the vehicle
// with the certified braking trajectory. Admissible from every phase that can
// still be moving, because a stop must never depend on the route state that
// failed.
[[nodiscard]] ExecutionRouteTransitionResult3D
enterStopExecution3D(const ExecutionPlan3D& current,
                     std::uint64_t expected_snapshot_version,
                     StopExecutionCertification3D certification);

[[nodiscard]] ExecutionRouteTransitionResult3D
revokeExecution3D(const ExecutionPlan3D& current,
                  std::uint64_t expected_snapshot_version);

[[nodiscard]] ExecutionRouteTransitionResult3D
suspendFiniteExecution3D(const ExecutionPlan3D& current,
                         std::uint64_t expected_snapshot_version);

[[nodiscard]] std::string_view
executionRouteTransitionStatus3DName(ExecutionRouteTransitionStatus3D status) noexcept;

} // namespace drone_city_nav
