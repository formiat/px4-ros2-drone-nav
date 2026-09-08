#pragma once

#include "drone_city_nav/execution_route_transitions_3d.hpp"
#include "drone_city_nav/finite_execution_path_3d.hpp"
#include "drone_city_nav/finite_motion_horizon_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"
#include "drone_city_nav/observed_world_content_hash_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav::execution_route_snapshot_3d_internal {

inline constexpr double kStationToleranceM{1.0e-6};
inline constexpr double kGeometryTolerance{1.0e-4};
inline constexpr double kExecutionBindingToleranceM{0.25};
inline constexpr double kCompletionStationToleranceM{0.5};
inline constexpr double kFiniteStopPositionToleranceM{0.25};
inline constexpr double kMaximumRouteCrossTrackM{
    kFiniteExecutionRouteCrossTrackToleranceM3D};
inline constexpr double kTerminalBoundaryToleranceM{0.5};
inline constexpr double kTerminalBoundaryActivationDistanceM{10.0};
inline constexpr double kMaximumStationCreditPerTravel{1.0};
using observed_world_content_3d::kFnvOffset;
using observed_world_content_3d::kFnvPrime;

using observed_world_content_3d::canonicalDoubleBits;
using observed_world_content_3d::footprintConservativelyContains;
using observed_world_content_3d::footprintValid;
using observed_world_content_3d::hashAxis;
using observed_world_content_3d::hashFootprint;
using observed_world_content_3d::hashGridBounds;
using observed_world_content_3d::hashLaunchSupportContact;
using observed_world_content_3d::hashObservedOccupancy;
using observed_world_content_3d::hashPoint;
using observed_world_content_3d::hashProprioceptiveFreeSpaceSeed;
using observed_world_content_3d::hashValue;
using observed_world_content_3d::launchSupportMatchesOwnedOccupancy;
using observed_world_content_3d::observedOccupancyContentFingerprint;
using observed_world_content_3d::observedWorldContentFingerprintFromObservation;
using observed_world_content_3d::sameAxisAlignedBox;
using observed_world_content_3d::sameCanonicalLaunchSupport;
using observed_world_content_3d::sameFootprintConfig;
using observed_world_content_3d::sameFreeSpaceSeed;
using observed_world_content_3d::validationPolicyFingerprint;

struct ValidationContractOwners3D {
  const VersionedObservedRawWorld3D* observed_raw_world{nullptr};
  const VersionedStaticWorld3D* static_world{nullptr};
  const VersionedLatestLidarEvidence3D* latest_lidar_evidence{nullptr};
};

struct ValidationWorldOwnerContent3D {
  std::uint64_t kind{0U};
  std::uint64_t content_fingerprint{0U};
};

struct PassageFrame3D {
  Point3 center{};
  Vec3 tangent{};
  Vec3 lateral_axis{};
  Vec3 secondary_axis{};
  double minimum_lateral_offset_m{0.0};
  double maximum_lateral_offset_m{0.0};
  double minimum_secondary_offset_m{0.0};
  double maximum_secondary_offset_m{0.0};
  bool valid{false};
};

struct StationedRoutePoint3D {
  Point3 point{};
  double station_m{0.0};
};

struct RouteAdherenceAssessment3D {
  RouteProjection3D begin{};
  RouteProjection3D stop{};
  FiniteExecutionRouteAdherenceStatus3D status{
      FiniteExecutionRouteAdherenceStatus3D::kNotEvaluated};
  std::size_t failure_state_index{0U};
  double failure_distance_m{-1.0};
  bool accepted{false};
};

struct CertificateView3D {
  RouteInstanceId3D route_instance_id{};
  std::uint64_t route_generation{0U};
  std::uint64_t geometry_revision{0U};
  std::uint64_t physical_route_fingerprint{0U};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t validated_through_revision{0U};
  std::uint64_t validation_policy_fingerprint{0U};
  std::uint64_t execution_validation_policy_fingerprint{0U};
  std::uint64_t world_content_fingerprint{0U};
  std::uint64_t route_decorations_revision{0U};
  std::uint64_t passage_volume_config_fingerprint{0U};
  std::uint64_t geometry_derivation_occupancy_content_fingerprint{0U};
  double suffix_start_station_m{0.0};
  double certified_end_station_m{0.0};
  bool observed_raw{false};
};

enum class ExecutionInputProgressRelation3D : std::uint8_t {
  kInvalid,
  kReplay,
  kStrictlyNewer,
};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept;

[[nodiscard]] bool finiteVector(const Vec3& vector) noexcept;

[[nodiscard]] bool finiteState(const MotionState3D& state) noexcept;

[[nodiscard]] bool finiteControl(const MotionControl3D& control) noexcept;

[[nodiscard]] bool knownFiniteExecutionKind(const FiniteExecutionKind3D kind) noexcept;

[[nodiscard]] bool
knownRouteLifecycleEventKind(const RouteLifecycleEventKind3D kind) noexcept;

[[nodiscard]] bool nearlyEqual(const double first, const double second,
                               const double tolerance = kGeometryTolerance) noexcept;

[[nodiscard]] bool
sameWorldCertificate(const NavigationWorldCertificate3D& first,
                     const NavigationWorldCertificate3D& second) noexcept;

[[nodiscard]] bool staticWorldNotOlder(const VersionedStaticWorld3D& candidate,
                                       const VersionedStaticWorld3D& previous) noexcept;

[[nodiscard]] bool
observedRawLineage(const NavigationWorldCertificate3D& certificate) noexcept;

void hashValidationTerminalBoundary(
    std::uint64_t& hash,
    const std::optional<FiniteExecutionPathTerminalBoundary3D>& boundary);

[[nodiscard]] std::optional<ValidationWorldOwnerContent3D>
validationWorldOwnerContent(const FiniteExecutionPathWorld3D& world,
                            const ValidationContractOwners3D& owners) noexcept;

[[nodiscard]] std::uint64_t validationLidarOwnerContentFingerprint(
    const std::span<const Point3> points,
    const VersionedLatestLidarEvidence3D* const owner) noexcept;

[[nodiscard]] std::uint64_t
validationContractFingerprint(const FiniteExecutionPathWorld3D& world,
                              const MotionControl3D& previous_applied_control,
                              const ValidationContractOwners3D& owners);

[[nodiscard]] std::optional<FiniteExecutionPathTerminalBoundary3D>
makeValidationTerminalBoundary(
    const std::optional<FiniteRouteTerminalBoundary3D>& boundary,
    const CertifiedRouteSuffix3D& route,
    std::span<const ControlRouteSample3D> mppi_reference);

[[nodiscard]] std::optional<FiniteRouteTerminalBoundary3D>
canonicalFiniteRouteTerminalBoundary(const CertifiedRouteSuffix3D& route,
                                     const double initial_station_m) noexcept;

[[nodiscard]] bool sameTerminalBoundary(
    const std::optional<FiniteRouteTerminalBoundary3D>& first,
    const std::optional<FiniteRouteTerminalBoundary3D>& second) noexcept;

[[nodiscard]] bool
latestLidarEvidenceFreshAt(const VersionedLatestLidarEvidence3D& evidence,
                           const VersionedExecutionValidationPolicy3D& policy,
                           const std::int64_t validation_stamp_ns) noexcept;

[[nodiscard]] bool stationaryHoldRawSafe(
    const Point3& position, const VersionedExecutionInput3D& execution_input,
    const VersionedObservedRawWorld3D* observed_raw_world,
    const VersionedStaticWorld3D* static_world,
    const VersionedExecutionValidationPolicy3D& validation_policy,
    const VersionedLatestLidarEvidence3D& latest_lidar_evidence) noexcept;

[[nodiscard]] std::vector<TimedExecutionPathPoint3D>
timedExecutionPathPoints(const FiniteMotionHorizon3D& horizon,
                         const MotionControl3D& previous_applied_control,
                         const std::int64_t control_interval_ns);

[[nodiscard]] bool finiteStateNearlyEqual(const MotionState3D& first,
                                          const MotionState3D& second) noexcept;

// The boolean form of finiteMotionHorizonDynamicsConsistency3D, for callers
// that only admit or refuse.
[[nodiscard]] bool
finiteHorizonDynamicallyConsistent(const FiniteMotionHorizon3D& horizon,
                                   const MotionControl3D& previous_applied_control,
                                   const MotionDynamicsConfig3D& dynamics) noexcept;

[[nodiscard]] double vectorNorm(const Vec3& vector) noexcept;

[[nodiscard]] double vectorDot(const Vec3& first, const Vec3& second) noexcept;

[[nodiscard]] bool samePointExact(const Point3& first, const Point3& second) noexcept;

[[nodiscard]] bool sameVectorExact(const Vec3& first, const Vec3& second) noexcept;

[[nodiscard]] bool
samePassageCrossSectionExact(const PassageCrossSection& first,
                             const PassageCrossSection& second) noexcept;

[[nodiscard]] bool samePassageVolumeExact(const PassageVolume& first,
                                          const PassageVolume& second) noexcept;

[[nodiscard]] bool
sameRouteEnvelopeSampleExact(const RouteEnvelopeSample& first,
                             const RouteEnvelopeSample& second) noexcept;

[[nodiscard]] bool canonicalPassageGeometryMatchesWorld(
    const CompiledTrajectory3D& geometry, const RouteDecorations3D& decorations,
    const OccupancyGrid3D& occupancy, const PassageVolumeConfig& expected_config);

[[nodiscard]] bool canonicalPassageGeometryMatchesObservedWorld(
    const CompiledTrajectory3D& geometry, const RouteDecorations3D& decorations,
    const VersionedObservedRawWorld3D& world,
    const PassageVolumeConfig& expected_config);

[[nodiscard]] Vec3 normalizedVector(const Vec3& vector) noexcept;

[[nodiscard]] PassageFrame3D
passageFrameBetweenSections(const PassageCrossSection& lower_section,
                            const PassageCrossSection& upper_section,
                            const double station_m) noexcept;

[[nodiscard]] PassageFrame3D passageFrameAtStation(const PassageVolume& volume,
                                                   const double station_m) noexcept;

[[nodiscard]] bool pointInsidePassageVolume(const PassageVolume& volume,
                                            const Point3& point,
                                            const double station_m) noexcept;

[[nodiscard]] bool constrainedPointAccepted(const CompiledTrajectory3D& geometry,
                                            const RouteDecorations3D& decorations,
                                            const Point3& point,
                                            const double station_m) noexcept;

[[nodiscard]] std::vector<double>
constrainedStationEvents(const CompiledTrajectory3D& geometry,
                         const RouteDecorations3D& decorations,
                         const double begin_station_m, const double end_station_m);

[[nodiscard]] bool pointInsidePassageFrame(const PassageFrame3D& frame,
                                           const Point3& point) noexcept;

[[nodiscard]] Point3 interpolatePoint(const Point3& first, const Point3& second,
                                      const double ratio) noexcept;

[[nodiscard]] Vec3 pointOffset(const Point3& point, const Point3& center) noexcept;

[[nodiscard]] bool projectionContinuouslyInsideBounds(
    const Vec3& begin_offset, const Vec3& end_offset, const Vec3& begin_axis,
    const Vec3& end_axis, const double axis_derivative_bound,
    const double minimum_offset_m, const double maximum_offset_m) noexcept;

[[nodiscard]] bool passageSectionIntervalAccepted(
    const PassageCrossSection& lower_section, const PassageCrossSection& upper_section,
    const Point3& begin, const double begin_station_m, const Point3& end,
    const double end_station_m, const std::size_t depth = 0U) noexcept;

[[nodiscard]] bool constrainedSegmentAccepted(const CompiledTrajectory3D& geometry,
                                              const RouteDecorations3D& decorations,
                                              const Point3& begin,
                                              const double begin_station_m,
                                              const Point3& end,
                                              const double end_station_m) noexcept;

[[nodiscard]] Point3 statePoint(const MotionState3D& state) noexcept;

[[nodiscard]] bool validateOrderedPassageCrossings(
    const CompiledTrajectory3D& geometry, const RouteDecorations3D& decorations,
    const std::span<const StationedRoutePoint3D> path, const double begin_station_m,
    const double end_station_m) noexcept;

[[nodiscard]] RouteAdherenceAssessment3D validateFiniteRouteAdherence(
    const CompiledTrajectory3D& geometry, const RouteDecorations3D& decorations,
    const std::span<const MotionState3D> states, const double initial_station_m,
    const double minimum_station_m, const double maximum_station_m,
    std::optional<double> maximum_cross_track_m,
    std::optional<double> terminal_cross_track_tolerance_m,
    double requested_sweep_step_m, bool allow_initial_handoff,
    bool enforce_tracking_tube);

[[nodiscard]] bool certifiedTrackingTubeHandoffPending(
    const ExecutionPlan3D& current,
    const CertifiedRouteSuffix3D& target_route) noexcept;

[[nodiscard]] bool validateTrackingTubeHandoffClearance(
    const CertifiedRouteSuffix3D& route, const FiniteMotionHorizon3D& horizon,
    double begin_route_station_m, const FiniteExecutionPathWorld3D& world) noexcept;

[[nodiscard]] bool validStationInterval(const double begin_station_m,
                                        const double end_station_m,
                                        const double route_end_station_m) noexcept;

[[nodiscard]] std::uint64_t expectedStaticOccupancyFingerprint(
    const StaticRouteCertificate3D& certificate) noexcept;

[[nodiscard]] CertificateView3D
certificateView(const RouteSuffixCertificate3D& certificate) noexcept;

void hashDouble(std::uint64_t& hash, const double value) noexcept;

void hashFloat(std::uint64_t& hash, const float value) noexcept;

void hashVector(std::uint64_t& hash, const Vec3& vector) noexcept;

void hashCertificate(std::uint64_t& hash,
                     const RouteSuffixCertificate3D& certificate) noexcept;

[[nodiscard]] bool validationLineageValidForCertificate(
    const FiniteExecutionValidationLineage3D& lineage,
    const RouteSuffixCertificate3D& certificate) noexcept;

[[nodiscard]] std::uint64_t
finiteExecutionArtifactFingerprint(const FiniteExecutionState3D& execution) noexcept;

[[nodiscard]] bool
sameDirectTrackingOwner(const DirectTrackingOwnerIdentity3D& first,
                        const DirectTrackingOwnerIdentity3D& second) noexcept;

[[nodiscard]] std::uint64_t directTrackingExecutionArtifactFingerprint(
    const DirectTrackingFiniteExecution3D& execution) noexcept;

[[nodiscard]] bool certificateValidForSource(
    const RouteSuffixCertificate3D& certificate, RouteInstanceId3D route_instance_id,
    const std::uint64_t route_generation, const std::uint64_t geometry_revision,
    const std::uint64_t physical_route_fingerprint) noexcept;

[[nodiscard]] bool
sameCertificateBinding(const RouteSuffixCertificate3D& first,
                       const RouteSuffixCertificate3D& second) noexcept;

[[nodiscard]] bool
certificateNotNewerThan(const RouteSuffixCertificate3D& artifact,
                        const RouteSuffixCertificate3D& route) noexcept;

[[nodiscard]] bool
certificateEligibleForRevalidation(const RouteSuffixCertificate3D& artifact,
                                   const RouteSuffixCertificate3D& route) noexcept;

[[nodiscard]] bool sameCertificate(const RouteSuffixCertificate3D& first,
                                   const RouteSuffixCertificate3D& second) noexcept;

[[nodiscard]] bool rawWorldMatchesCertificate(
    const std::shared_ptr<const VersionedObservedRawWorld3D>& world,
    const ObservedRawRouteCertificate3D& certificate,
    const bool require_exact_revision) noexcept;

[[nodiscard]] bool staticWorldMatchesCertificate(
    const std::shared_ptr<const VersionedStaticWorld3D>& world,
    const StaticRouteCertificate3D& certificate) noexcept;

[[nodiscard]] bool validateStaticRouteSuffixAgainstOwner(
    const VersionedStaticWorld3D& world, const std::span<const RouteSample3D> route,
    const RouteProjection3D& projection,
    const RouteActivationObservation3D& observation,
    const FlightEnvelopeConfig& flight_envelope) noexcept;

[[nodiscard]] bool
finiteWorldOwnerMatchesProof(const FiniteExecutionState3D& execution) noexcept;

[[nodiscard]] bool directTrackingWorldOwnerMatchesProof(
    const DirectTrackingFiniteExecution3D& execution) noexcept;

[[nodiscard]] std::uint64_t
rawValidatedRevision(const FiniteExecutionValidationLineage3D& lineage) noexcept;

[[nodiscard]] bool finiteExecutionValidatedAgainstNewerRawWorld(
    const FiniteExecutionState3D& execution) noexcept;

[[nodiscard]] bool
rawInvalidationProofMatchesEvent(const FiniteExecutionState3D& execution,
                                 const RouteLifecycleEvent3D& event) noexcept;

[[nodiscard]] bool
latestLidarInvalidationProofMatchesEvent(const FiniteExecutionState3D& execution,
                                         const RouteLifecycleEvent3D& event) noexcept;

[[nodiscard]] bool
terminalStopBoundaryValid(const CertifiedStopBoundary3D& boundary,
                          const FiniteExecutionState3D& execution) noexcept;

[[nodiscard]] ExecutionRouteTransitionResult3D transitionFailure(
    ExecutionRouteTransitionStatus3D status,
    ExecutionRouteTransitionDetail3D detail = ExecutionRouteTransitionDetail3D::kNone);

[[nodiscard]] ExecutionRouteTransitionStatus3D
checkCurrentAndVersion(const ExecutionPlan3D& current,
                       const std::uint64_t expected_snapshot_version) noexcept;

[[nodiscard]] ExecutionRouteTransitionStatus3D
checkGuard(const ExecutionPlan3D& current,
           const ExecutionRouteTransitionGuard3D& guard) noexcept;

[[nodiscard]] ExecutionRouteTransitionResult3D
finishTransition(const ExecutionPlan3D& current, ExecutionPlan3D next);

[[nodiscard]] const CertifiedRouteSuffix3D*
routePointer(const ExecutionPlan3D& snapshot) noexcept;

[[nodiscard]] CertifiedRouteSuffix3D* routePointer(ExecutionPlan3D& snapshot) noexcept;

[[nodiscard]] FiniteExecutionState3D*
finiteExecutionPointer(ExecutionPlan3D& snapshot) noexcept;

[[nodiscard]] FiniteExecutionState3D*
brakingFallbackPointer(ExecutionPlan3D& snapshot) noexcept;

[[nodiscard]] bool sameControl(const MotionControl3D& first,
                               const MotionControl3D& second) noexcept;

[[nodiscard]] bool sameStateExact(const MotionState3D& first,
                                  const MotionState3D& second) noexcept;

[[nodiscard]] bool
sameStateProvenance(const ExecutionStateProvenance3D& first,
                    const ExecutionStateProvenance3D& second) noexcept;

[[nodiscard]] bool
sameSourceSampleStateUpdateAllowed(const VersionedExecutionInput3D& candidate,
                                   const VersionedExecutionInput3D& previous) noexcept;

[[nodiscard]] ExecutionInputProgressRelation3D
executionInputProgressRelation(const VersionedExecutionInput3D& candidate,
                               const VersionedExecutionInput3D& previous) noexcept;

[[nodiscard]] bool
executionInputNotOlder(const VersionedExecutionInput3D& candidate,
                       const VersionedExecutionInput3D& previous) noexcept;

[[nodiscard]] Point3
executionInputPosition(const VersionedExecutionInput3D& input) noexcept;

void bindProgressToExecutionInput(
    CertifiedRouteProgress3D& progress,
    const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
    const double station_m);

[[nodiscard]] std::optional<RouteAdherenceAssessment3D>
validateExecutionProgressConnector(
    const CertifiedRouteSuffix3D& route, const Point3& execution_position,
    const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& observed_raw_world,
    std::span<const Point3> latest_lidar_obstacle_points);

[[nodiscard]] bool
latestLidarEvidenceNotOlder(const VersionedLatestLidarEvidence3D& candidate,
                            const VersionedLatestLidarEvidence3D& previous) noexcept;

[[nodiscard]] bool
directTrackingWorldNotOlder(const DirectTrackingFiniteExecution3D& candidate,
                            const DirectTrackingFiniteExecution3D& previous) noexcept;

[[nodiscard]] bool directTrackingExecutionNotOlder(
    const DirectTrackingFiniteExecution3D& candidate,
    const DirectTrackingFiniteExecution3D& previous) noexcept;

[[nodiscard]] bool directTrackingEvidenceNotOlderThanRoute(
    const DirectTrackingFiniteExecution3D& candidate,
    const FiniteExecutionState3D& previous) noexcept;

[[nodiscard]] bool routeExecutionEvidenceNotOlderThanDirect(
    const FiniteExecutionState3D& candidate,
    const DirectTrackingFiniteExecution3D& previous) noexcept;

[[nodiscard]] bool
routeExecutionEvidenceNotOlderThanStop(const FiniteExecutionState3D& candidate,
                                       const StopExecution3D& previous) noexcept;

[[nodiscard]] bool routeExecutionEvidenceNotOlderThanHold(
    const FiniteExecutionState3D& candidate,
    const StationaryExecutionHold3D& previous) noexcept;

[[nodiscard]] bool directTrackingEvidenceNotOlderThanHold(
    const DirectTrackingFiniteExecution3D& candidate,
    const StationaryExecutionHold3D& previous) noexcept;

[[nodiscard]] bool
candidateFiniteExecutionValid(const FiniteExecutionState3D& candidate,
                              const ExecutionPlan3D& current,
                              const CertifiedRouteSuffix3D* route,
                              const bool require_current_certificate) noexcept;

// Whether a successor's evidence is at least as current as the resident
// route's. kNone means it is; any other detail names the evidence that is
// older or incompatible.
[[nodiscard]] ExecutionRouteTransitionDetail3D
successorEvidenceRegression(const CertifiedRouteSuffix3D& current_route,
                            const FiniteExecutionState3D& current_execution,
                            const CertifiedRouteSuffix3D& successor,
                            const FiniteExecutionState3D& successor_execution) noexcept;

[[nodiscard]] ExecutionRouteTransitionDetail3D successorRouteEvidenceRegression(
    const CertifiedRouteSuffix3D& current_route,
    const CertifiedRouteSuffix3D& successor,
    const FiniteExecutionState3D& successor_execution) noexcept;

[[nodiscard]] std::optional<CertifiedRouteSuffix3D>
certifyExecutionRoute3DImpl(const ExecutionRouteActivation3D& activation,
                            const bool reuse_sealed_geometry_owner);

[[nodiscard]] FiniteExecutionCertificationResult3D
certifyFiniteExecutionAgainstOwnedWorld3D(
    const ExecutionPlan3D& current, const CertifiedRouteSuffix3D& target_route,
    FiniteExecutionCertification3D certification,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_validation_world,
    const RouteLifecycleEvent3D* const lifecycle_event);

[[nodiscard]] ExecutionRouteTransitionResult3D applyActivateCertifiedRouteCommand3D(
    const ExecutionPlan3D& current, std::uint64_t expected_snapshot_version,
    CertifiedRouteSuffix3D candidate, FiniteExecutionPlan3D candidate_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D applyAdvanceCertifiedRouteCommand3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    RouteExecutionObservation3D observation,
    std::shared_ptr<const VersionedExecutionInput3D> execution_input,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world);

[[nodiscard]] ExecutionRouteTransitionResult3D
applyReplaceFiniteExecutionPlanCommand3D(const ExecutionPlan3D& current,
                                         const ExecutionRouteTransitionGuard3D& guard,
                                         FiniteExecutionPlan3D execution);

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCompleteCertifiedRouteCommand3D(const ExecutionPlan3D& current,
                                     const ExecutionRouteTransitionGuard3D& guard,
                                     const RouteLifecycleEvent3D& event);

[[nodiscard]] ExecutionRouteTransitionResult3D applyReplaceCertifiedRouteCommand3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution,
    const CertifiedRouteSplice3D& splice);

[[nodiscard]] ExecutionRouteTransitionResult3D
applyReplaceCertifiedRouteAtHandoffCommand3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D applyTransferToDirectTrackingCommand3D(
    const ExecutionPlan3D& current, std::uint64_t expected_snapshot_version,
    DirectTrackingFiniteExecution3D direct_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D
applyReplaceDirectTrackingExecutionCommand3D(
    const ExecutionPlan3D& current, std::uint64_t expected_snapshot_version,
    DirectTrackingFiniteExecution3D direct_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D
applyTransferDirectTrackingToCertifiedRouteCommand3D(
    const ExecutionPlan3D& current, std::uint64_t expected_snapshot_version,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D applyTransferToExecutionHoldCommand3D(
    const ExecutionPlan3D& current, std::uint64_t expected_snapshot_version,
    StationaryExecutionHoldCertification3D certification);

[[nodiscard]] ExecutionRouteTransitionResult3D applyArmStationaryCaptureHoldCommand3D(
    const ExecutionPlan3D& current, std::uint64_t expected_snapshot_version,
    StationaryExecutionHoldCertification3D certification);

[[nodiscard]] ExecutionRouteTransitionResult3D
applyEnterStopExecutionCommand3D(const ExecutionPlan3D& current,
                                 std::uint64_t expected_snapshot_version,
                                 StopExecutionCertification3D certification,
                                 StopCertificationResult3D* certification_report);

[[nodiscard]] ExecutionRouteTransitionResult3D
applyRevokeExecutionCommand3D(const ExecutionPlan3D& current,
                              std::uint64_t expected_snapshot_version);

[[nodiscard]] ExecutionRouteTransitionResult3D
applySuspendFiniteExecutionCommand3D(const ExecutionPlan3D& current,
                                     std::uint64_t expected_snapshot_version);

} // namespace drone_city_nav::execution_route_snapshot_3d_internal
