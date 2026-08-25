#pragma once

#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"

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
inline constexpr double kMaximumRouteCrossTrackM{2.0};
inline constexpr double kTerminalBoundaryToleranceM{0.5};
inline constexpr double kTerminalBoundaryActivationDistanceM{10.0};
inline constexpr double kMaximumStationCreditPerTravel{1.0};
inline constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
inline constexpr std::uint64_t kFnvPrime{1099511628211ULL};

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
  std::uint64_t route_generation{0U};
  std::uint64_t geometry_revision{0U};
  std::uint64_t physical_route_fingerprint{0U};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t validated_through_revision{0U};
  std::uint64_t validation_policy_fingerprint{0U};
  std::uint64_t execution_validation_policy_fingerprint{0U};
  std::uint64_t world_content_fingerprint{0U};
  std::uint64_t passage_geometry_revision{0U};
  std::uint64_t passage_volume_config_fingerprint{0U};
  std::uint64_t passage_derivation_occupancy_content_fingerprint{0U};
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

[[nodiscard]] bool finiteState(const mppi::State& state) noexcept;

[[nodiscard]] bool finiteControl(const mppi::Control& control) noexcept;

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

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept;

[[nodiscard]] std::uint64_t canonicalDoubleBits(const double value) noexcept;

void hashPoint(std::uint64_t& hash, const Point3& point) noexcept;

void hashAxis(std::uint64_t& hash, const FootprintBodyAxis& axis) noexcept;

[[nodiscard]] bool footprintValid(const SweptFootprintConfig& footprint) noexcept;

[[nodiscard]] bool sameFootprintConfig(const SweptFootprintConfig& first,
                                       const SweptFootprintConfig& second) noexcept;

[[nodiscard]] bool
footprintConservativelyContains(const SweptFootprintConfig& outer,
                                const SweptFootprintConfig& inner) noexcept;

[[nodiscard]] bool
sameFreeSpaceSeed(const ProprioceptiveFreeSpaceSeed3D& first,
                  const ProprioceptiveFreeSpaceSeed3D& second) noexcept;

[[nodiscard]] bool sameAxisAlignedBox(const AxisAlignedBox3D& first,
                                      const AxisAlignedBox3D& second) noexcept;

[[nodiscard]] bool
sameCanonicalLaunchSupport(const LaunchSupportContact3D& candidate,
                           const LaunchSupportContact3D& canonical) noexcept;

[[nodiscard]] bool
launchSupportMatchesOwnedOccupancy(const LaunchSupportContact3D& support,
                                   const ObservedOccupancyGrid3D& occupancy);

void hashFootprint(std::uint64_t& hash, const SweptFootprintConfig& footprint) noexcept;

[[nodiscard]] bool hashProprioceptiveFreeSpaceSeed(
    std::uint64_t& hash,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed) noexcept;

[[nodiscard]] bool hashLaunchSupportContact(
    std::uint64_t& hash,
    const LaunchSupportContact3D* const launch_support_contact) noexcept;

[[nodiscard]] std::uint64_t validationPolicyFingerprint(
    const SweptFootprintConfig& footprint,
    const ObservedSpaceValidationPolicy observed_policy,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
    const LaunchSupportContact3D* const launch_support_contact) noexcept;

void hashGridBounds(std::uint64_t& hash, const GridBounds3D& bounds) noexcept;

void hashObservedOccupancy(std::uint64_t& hash,
                           const ObservedOccupancyGrid3D& occupancy);

[[nodiscard]] std::uint64_t
observedOccupancyContentFingerprint(const ObservedOccupancyGrid3D& occupancy);

[[nodiscard]] std::uint64_t observedWorldContentFingerprintFromObservation(
    const std::uint64_t observation_content_fingerprint,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
    const LaunchSupportContact3D* const launch_support_contact);

void hashValidationTerminalBoundary(
    std::uint64_t& hash,
    const std::optional<mppi::FiniteExecutionPathTerminalBoundary>& boundary);

[[nodiscard]] std::optional<ValidationWorldOwnerContent3D>
validationWorldOwnerContent(const mppi::FiniteExecutionPathWorld& world,
                            const ValidationContractOwners3D& owners) noexcept;

[[nodiscard]] std::uint64_t validationLidarOwnerContentFingerprint(
    const std::span<const Point3> points,
    const VersionedLatestLidarEvidence3D* const owner) noexcept;

[[nodiscard]] std::uint64_t
validationContractFingerprint(const mppi::FiniteExecutionPathWorld& world,
                              const mppi::Control& previous_applied_control,
                              const ValidationContractOwners3D& owners);

[[nodiscard]] std::optional<mppi::FiniteExecutionPathTerminalBoundary>
makeValidationTerminalBoundary(
    const std::optional<FiniteRouteTerminalBoundary3D>& boundary,
    const CertifiedRouteSuffix3D& route);

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

[[nodiscard]] std::vector<mppi::TimedExecutionPathPoint>
timedExecutionPathPoints(const mppi::FiniteHorizon& horizon,
                         const mppi::Control& previous_applied_control,
                         const std::int64_t control_interval_ns);

[[nodiscard]] bool finiteStateNearlyEqual(const mppi::State& first,
                                          const mppi::State& second) noexcept;

[[nodiscard]] bool
finiteHorizonDynamicallyConsistent(const mppi::FiniteHorizon& horizon,
                                   const mppi::Control& previous_applied_control,
                                   const mppi::DynamicsConfig& dynamics) noexcept;

[[nodiscard]] bool
validRouteSamples(const std::span<const RouteSample3D> route) noexcept;

[[nodiscard]] double vectorNorm(const Vec3& vector) noexcept;

[[nodiscard]] double vectorDot(const Vec3& first, const Vec3& second) noexcept;

[[nodiscard]] bool validEnvelopeSamples(const ConstrainedRouteSpan& span) noexcept;

[[nodiscard]] bool
validTraversalSegmentSpans(const ConstrainedRouteSpan& span) noexcept;

[[nodiscard]] bool
validPassageCrossSection(const PassageCrossSection& section) noexcept;

[[nodiscard]] bool validPassageVolume(const PassageVolume& volume,
                                      const ConstrainedRouteSpan& span,
                                      const std::size_t span_index) noexcept;

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

[[nodiscard]] bool
canonicalPassageGeometryMatchesWorld(const ExecutionRouteGeometry3D& geometry,
                                     const OccupancyGrid3D& occupancy,
                                     const PassageVolumeConfig& expected_config);

[[nodiscard]] bool canonicalPassageGeometryMatchesObservedWorld(
    const ExecutionRouteGeometry3D& geometry, const VersionedObservedRawWorld3D& world,
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

[[nodiscard]] bool constrainedPointAccepted(const ExecutionRouteGeometry3D& geometry,
                                            const Point3& point,
                                            const double station_m) noexcept;

[[nodiscard]] std::vector<double>
constrainedStationEvents(const ExecutionRouteGeometry3D& geometry,
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

[[nodiscard]] bool constrainedSegmentAccepted(const ExecutionRouteGeometry3D& geometry,
                                              const Point3& begin,
                                              const double begin_station_m,
                                              const Point3& end,
                                              const double end_station_m) noexcept;

[[nodiscard]] Point3 statePoint(const mppi::State& state) noexcept;

[[nodiscard]] bool
validateOrderedPassageCrossings(const ExecutionRouteGeometry3D& geometry,
                                const std::span<const StationedRoutePoint3D> path,
                                const double begin_station_m,
                                const double end_station_m) noexcept;

[[nodiscard]] RouteAdherenceAssessment3D validateFiniteRouteAdherence(
    const ExecutionRouteGeometry3D& geometry, const std::span<const mppi::State> states,
    const double initial_station_m, const double minimum_station_m,
    const double maximum_station_m, const double maximum_cross_track_m,
    const double terminal_cross_track_tolerance_m, const double requested_sweep_step_m,
    const bool allow_initial_handoff);

[[nodiscard]] bool validMppiRoute(const std::span<const mppi::RouteSample3D> mppi_route,
                                  const std::span<const RouteSample3D> route) noexcept;

[[nodiscard]] std::shared_ptr<const ExecutionRouteGeometry3D>
captureExecutionRouteGeometry3D(const ExecutionRouteGeometry3D& source);

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

[[nodiscard]] bool
certificateValidForSource(const RouteSuffixCertificate3D& certificate,
                          const std::uint64_t route_generation,
                          const std::uint64_t geometry_revision,
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
    const RouteActivationObservation3D& observation) noexcept;

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
terminalStopBoundaryValid(const CertifiedStopBoundary3D& boundary,
                          const FiniteExecutionState3D& execution) noexcept;

[[nodiscard]] ExecutionRouteTransitionResult3D
transitionFailure(const ExecutionRouteTransitionStatus3D status);

[[nodiscard]] ExecutionRouteTransitionStatus3D
checkCurrentAndVersion(const ExecutionRouteSnapshot3D& current,
                       const std::uint64_t expected_snapshot_version) noexcept;

[[nodiscard]] ExecutionRouteTransitionStatus3D
checkGuard(const ExecutionRouteSnapshot3D& current,
           const ExecutionRouteTransitionGuard3D& guard) noexcept;

[[nodiscard]] ExecutionRouteTransitionResult3D
finishTransition(const ExecutionRouteSnapshot3D& current,
                 ExecutionRouteSnapshot3D next);

[[nodiscard]] const CertifiedRouteSuffix3D*
routePointer(const ExecutionRouteSnapshot3D& snapshot) noexcept;

[[nodiscard]] CertifiedRouteSuffix3D*
routePointer(ExecutionRouteSnapshot3D& snapshot) noexcept;

[[nodiscard]] bool sameControl(const mppi::Control& first,
                               const mppi::Control& second) noexcept;

[[nodiscard]] bool sameStateExact(const mppi::State& first,
                                  const mppi::State& second) noexcept;

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

[[nodiscard]] bool routeExecutionEvidenceNotOlderThanHold(
    const FiniteExecutionState3D& candidate,
    const StationaryExecutionHold3D& previous) noexcept;

[[nodiscard]] bool directTrackingEvidenceNotOlderThanHold(
    const DirectTrackingFiniteExecution3D& candidate,
    const StationaryExecutionHold3D& previous) noexcept;

[[nodiscard]] bool
candidateFiniteExecutionValid(const FiniteExecutionState3D& candidate,
                              const ExecutionRouteSnapshot3D& current,
                              const CertifiedRouteSuffix3D* route,
                              const bool require_current_certificate) noexcept;

[[nodiscard]] bool
successorEvidenceNotOlder(const CertifiedRouteSuffix3D& current_route,
                          const FiniteExecutionState3D& current_execution,
                          const CertifiedRouteSuffix3D& successor,
                          const FiniteExecutionState3D& successor_execution) noexcept;

[[nodiscard]] std::optional<CertifiedRouteSuffix3D>
certifyExecutionRoute3DImpl(const ExecutionRouteActivation3D& activation,
                            const bool reuse_sealed_geometry_owner);

[[nodiscard]] FiniteExecutionCertificationResult3D
certifyFiniteExecutionAgainstOwnedWorld3D(
    const ExecutionRouteSnapshot3D& current, const CertifiedRouteSuffix3D& target_route,
    FiniteExecutionCertification3D certification,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_validation_world,
    const RouteLifecycleEvent3D* const raw_invalidation);

} // namespace drone_city_nav::execution_route_snapshot_3d_internal
