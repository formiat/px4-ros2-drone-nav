#pragma once

#include "drone_city_nav/execution_evidence_3d.hpp"
#include "drone_city_nav/execution_route_geometry_3d.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/route_execution_contract_3d.hpp"
#include "drone_city_nav/route_lifecycle_3d.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>
#include <variant>

namespace drone_city_nav {

class VersionedObservedRawWorld3D;
class VersionedStaticWorld3D;
struct CertifiedRouteSplice3D;

struct CertifiedRouteProgress3D {
  std::uint64_t route_generation{0U};
  std::uint64_t geometry_revision{0U};
  double station_m{0.0};
  Point3 last_observed_position{};
  // Null only while a certified route is waiting for activation. An activated
  // route owns the exact immutable navigation evidence that established its
  // current progress and position.
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;

  [[nodiscard]] bool valid() const noexcept;
};

struct RouteInstanceId3D {
  std::uint64_t value{0U};

  [[nodiscard]] bool valid() const noexcept {
    return value != 0U;
  }

  friend bool operator==(const RouteInstanceId3D&, const RouteInstanceId3D&) = default;
};

struct StaticRouteCertificate3D {
  RouteInstanceId3D route_instance_id{};
  std::uint64_t route_generation{0U};
  std::uint64_t geometry_revision{0U};
  std::uint64_t physical_route_fingerprint{0U};
  std::uint64_t static_occupancy_content_fingerprint{0U};
  std::uint64_t validation_policy_fingerprint{0U};
  std::uint64_t execution_validation_policy_fingerprint{0U};
  std::uint64_t passage_geometry_revision{0U};
  std::uint64_t passage_volume_config_fingerprint{0U};
  std::uint64_t passage_derivation_occupancy_content_fingerprint{0U};
  NavigationWorldCertificate3D world_certificate{};
  double suffix_start_station_m{0.0};
  double certified_end_station_m{0.0};

  [[nodiscard]] bool validFor(RouteInstanceId3D expected_route_instance_id,
                              const ActivatedRouteIdentity3D& identity,
                              std::uint64_t expected_geometry_revision,
                              std::uint64_t expected_physical_route_fingerprint,
                              double route_end_station_m) const noexcept;
};

struct ObservedRawRouteCertificate3D {
  RouteInstanceId3D route_instance_id{};
  std::uint64_t route_generation{0U};
  std::uint64_t geometry_revision{0U};
  std::uint64_t physical_route_fingerprint{0U};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t validated_through_revision{0U};
  std::uint64_t validation_policy_fingerprint{0U};
  std::uint64_t execution_validation_policy_fingerprint{0U};
  std::uint64_t observed_world_content_fingerprint{0U};
  std::uint64_t passage_geometry_revision{0U};
  std::uint64_t passage_volume_config_fingerprint{0U};
  std::uint64_t passage_derivation_occupancy_content_fingerprint{0U};
  double suffix_start_station_m{0.0};
  double certified_end_station_m{0.0};

  [[nodiscard]] bool validFor(RouteInstanceId3D expected_route_instance_id,
                              const ActivatedRouteIdentity3D& identity,
                              std::uint64_t expected_geometry_revision,
                              std::uint64_t expected_physical_route_fingerprint,
                              double route_end_station_m) const noexcept;
};

using RouteSuffixCertificate3D =
    std::variant<StaticRouteCertificate3D, ObservedRawRouteCertificate3D>;

struct CertifiedRouteSuffix3D {
  RouteInstanceId3D route_instance_id{};
  // A recertified route is a new immutable semantic revision, but it retains
  // explicit provenance to the sealed route revision from which it was
  // derived. Copies preserve both identities.
  std::optional<RouteInstanceId3D> parent_route_instance_id;
  ActivatedRouteIdentity3D identity{};
  std::shared_ptr<const ExecutionRouteGeometry3D> geometry;
  RouteSuffixCertificate3D certificate{StaticRouteCertificate3D{}};
  CertifiedRouteProgress3D progress{};
  RouteContinuityLineage3D continuity_lineage{};
  std::uint64_t continuity_id{0U};
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  RouteEndpointSemantics3D planned_endpoint_semantics{
      RouteEndpointSemantics3D::kContinuation};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] double endStationM() const noexcept;
  [[nodiscard]] double remainingM() const noexcept;
};

enum class FiniteExecutionKind3D : std::uint8_t {
  kNominal,
  kRetained,
  kEmergencyBrakeTail,
};

struct FiniteRouteTerminalBoundary3D {
  Point3 endpoint{};
  Vec3 forward{};
  double tolerance_m{0.5};
  double activation_distance_m{0.0};
  double maximum_cross_track_m{0.0};
  double initial_route_station_m{0.0};
  double activation_route_station_m{0.0};

  [[nodiscard]] bool valid(double route_end_station_m) const noexcept;
};

struct CertifiedStopBoundary3D {
  RouteInstanceId3D route_instance_id{};
  std::uint64_t route_generation{0U};
  std::uint64_t geometry_revision{0U};
  std::uint64_t physical_route_fingerprint{0U};
  std::uint64_t trajectory_revision{0U};
  std::uint64_t raw_validated_through_revision{0U};
  double station_m{0.0};
  double position_tolerance_m{0.25};
  Point3 position{};
};

struct StaticFiniteExecutionValidationLineage3D {
  NavigationWorldCertificate3D world_certificate{};
  std::uint64_t static_occupancy_content_fingerprint{0U};
  std::uint64_t validation_policy_fingerprint{0U};
};

struct ObservedRawFiniteExecutionValidationLineage3D {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t validated_through_raw_revision{0U};
  std::uint64_t validation_policy_fingerprint{0U};
  std::uint64_t observed_world_content_fingerprint{0U};
};

using FiniteExecutionValidationLineage3D =
    std::variant<StaticFiniteExecutionValidationLineage3D,
                 ObservedRawFiniteExecutionValidationLineage3D>;

struct FiniteExecutionValidationProof3D {
  std::uint64_t artifact_fingerprint{0U};
  std::uint64_t validation_contract_fingerprint{0U};
  FiniteExecutionValidationLineage3D lineage{
      StaticFiniteExecutionValidationLineage3D{}};
};

struct FiniteExecutionState3D {
  std::uint64_t trajectory_revision{0U};
  std::uint64_t source_snapshot_version{0U};
  std::uint64_t source_navigation_revision{0U};
  RouteInstanceId3D source_route_instance_id{};
  std::uint64_t source_route_generation{0U};
  std::uint64_t source_geometry_revision{0U};
  std::uint64_t source_physical_route_fingerprint{0U};
  RouteSuffixCertificate3D certificate{StaticRouteCertificate3D{}};
  std::shared_ptr<const mppi::FiniteHorizon> horizon;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::optional<FiniteRouteTerminalBoundary3D> terminal_boundary;
  CertifiedStopBoundary3D stop_boundary{};
  double begin_route_station_m{0.0};
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  std::int64_t control_interval_ns{0};
  FiniteExecutionKind3D kind{FiniteExecutionKind3D::kNominal};
  FiniteExecutionValidationProof3D validation_proof{};
  bool revalidation_required{false};

  [[nodiscard]] bool validFor(const CertifiedRouteSuffix3D* route) const noexcept;
};

struct DirectTrackingOwnerIdentity3D {
  std::uint64_t mission_epoch{0U};
  std::uint64_t assignment_generation{0U};
  std::uint64_t target_detection_id{0U};
  std::uint64_t target_track_id{0U};
  std::uint64_t objective_sample_sequence{0U};
  std::uint64_t line_of_sight_generation{0U};

  [[nodiscard]] bool valid() const noexcept;
};

struct DirectTrackingFiniteExecution3D {
  DirectTrackingOwnerIdentity3D identity{};
  std::uint64_t trajectory_revision{0U};
  std::uint64_t source_snapshot_version{0U};
  std::uint64_t source_navigation_revision{0U};
  Point3 target{};
  std::shared_ptr<const mppi::FiniteHorizon> horizon;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  std::int64_t control_interval_ns{0};
  FiniteExecutionKind3D kind{FiniteExecutionKind3D::kNominal};
  FiniteExecutionValidationProof3D validation_proof{};

  [[nodiscard]] bool valid() const noexcept;
};

// A handoff is physically stationary at the controller boundary when the
// vehicle is inside the certified terminal position tolerance and below the
// operational stop thresholds. Previous acceleration remains measured evidence
// for footprint attitude; sensor/controller noise must not be mistaken for
// translational motion.
inline constexpr double kStationaryExecutionHoldPositionToleranceM{0.25};
inline constexpr double kStationaryExecutionHoldSpeedToleranceMps{0.25};
inline constexpr double kStationaryExecutionHoldYawRateToleranceRadps{0.25};

enum class StationaryExecutionHoldOrigin3D : std::uint8_t {
  kTerminalExecution,
  kStationaryCaptureRearm,
};

struct StationaryExecutionHold3D {
  std::uint64_t hold_id{0U};
  std::uint64_t source_trajectory_revision{0U};
  StationaryExecutionHoldOrigin3D origin{
      StationaryExecutionHoldOrigin3D::kTerminalExecution};
  Point3 position{};
  std::shared_ptr<const VersionedExecutionInput3D> terminal_execution_input;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;

  [[nodiscard]] bool valid() const noexcept;
};

enum class ExecutionRoutePhase3D : std::uint8_t {
  kFollowing,
  kDirectTracking,
  kAwaitingSuccessor,
  kBraking,
  kStopped,
  kRevoked,
};

struct ExecutionRouteSnapshot3D {
  std::uint64_t version{0U};
  ExecutionRoutePhase3D phase{ExecutionRoutePhase3D::kAwaitingSuccessor};
  std::optional<CertifiedRouteSuffix3D> route;
  std::optional<FiniteExecutionState3D> finite_execution;
  std::optional<DirectTrackingFiniteExecution3D> direct_tracking_execution;
  std::optional<StationaryExecutionHold3D> stationary_hold;
  std::uint64_t execution_owner_epoch{0U};
  std::uint64_t route_generation_high_water{0U};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] std::uint64_t routeGenerationHighWater() const noexcept;
};

struct ExecutionRouteActivation3D {
  std::uint64_t route_generation{0U};
  MaterializedRouteProposal3D proposal{};
  std::shared_ptr<const ExecutionRouteGeometry3D> geometry;
  RouteActivationObservation3D observation{};
  PassageVolumeConfig passage_volume_config{};
  RouteContinuityLineage3D continuity_lineage{};
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
};

class VersionedObservedRawWorld3D final {
private:
  struct CaptureToken final {};

public:
  [[nodiscard]] static std::shared_ptr<const VersionedObservedRawWorld3D>
  capture(RawMapVersion version, const ObservedOccupancyGrid3D& occupancy,
          std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
          std::optional<LaunchSupportContact3D> launch_support_contact);

  [[nodiscard]] static std::shared_ptr<const VersionedObservedRawWorld3D> captureOwned(
      RawMapVersion version, std::shared_ptr<const ObservedOccupancyGrid3D> occupancy,
      std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
      std::optional<LaunchSupportContact3D> launch_support_contact);

  [[nodiscard]] const RawMapVersion& version() const noexcept;
  [[nodiscard]] const ObservedOccupancyGrid3D& occupancy() const noexcept;
  [[nodiscard]] std::uint64_t contentFingerprint() const noexcept;
  // Exact binary occupancy input used for passage-volume derivation.
  [[nodiscard]] std::uint64_t occupiedContentFingerprint() const noexcept;
  [[nodiscard]] std::shared_ptr<const OccupancyGrid3D>
  occupiedSnapshot() const noexcept;
  [[nodiscard]] bool
  sharesObservationOwner(const VersionedObservedRawWorld3D& other) const noexcept;
  [[nodiscard]] std::shared_ptr<const VersionedObservedRawWorld3D> deriveRouteEvidence(
      std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
      std::optional<LaunchSupportContact3D> launch_support_contact) const;
  [[nodiscard]] const std::optional<ProprioceptiveFreeSpaceSeed3D>&
  proprioceptiveFreeSpaceSeed() const noexcept;
  [[nodiscard]] const std::optional<LaunchSupportContact3D>&
  launchSupportContact() const noexcept;
  [[nodiscard]] bool valid() const noexcept;

  VersionedObservedRawWorld3D(
      CaptureToken, RawMapVersion version,
      std::shared_ptr<const ObservedOccupancyGrid3D> occupancy,
      std::shared_ptr<const OccupancyGrid3D> occupied_snapshot,
      std::uint64_t observation_content_fingerprint,
      std::uint64_t occupied_content_fingerprint,
      std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
      std::optional<LaunchSupportContact3D> launch_support_contact);

private:
  RawMapVersion version_{};
  std::shared_ptr<const ObservedOccupancyGrid3D> occupancy_;
  std::shared_ptr<const OccupancyGrid3D> occupied_snapshot_;
  std::uint64_t observation_content_fingerprint_{0U};
  std::uint64_t content_fingerprint_{0U};
  std::uint64_t occupied_content_fingerprint_{0U};
  std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed_;
  std::optional<LaunchSupportContact3D> launch_support_contact_;
};

class VersionedStaticWorld3D final {
private:
  struct CaptureToken final {};

public:
  [[nodiscard]] static std::shared_ptr<const VersionedStaticWorld3D>
  capture(NavigationWorldCertificate3D certificate, const OccupancyGrid3D& occupancy);

  [[nodiscard]] static std::shared_ptr<const VersionedStaticWorld3D>
  captureOwned(NavigationWorldCertificate3D certificate,
               std::shared_ptr<const OccupancyGrid3D> occupancy);

  [[nodiscard]] const NavigationWorldCertificate3D& certificate() const noexcept;
  [[nodiscard]] const OccupancyGrid3D& occupancy() const noexcept;
  [[nodiscard]] std::uint64_t contentFingerprint() const noexcept;
  [[nodiscard]] bool valid() const noexcept;

  VersionedStaticWorld3D(CaptureToken, NavigationWorldCertificate3D certificate,
                         std::shared_ptr<const OccupancyGrid3D> occupancy,
                         std::uint64_t content_fingerprint);

private:
  NavigationWorldCertificate3D certificate_{};
  std::shared_ptr<const OccupancyGrid3D> occupancy_;
  std::uint64_t content_fingerprint_{0U};
};

struct FiniteExecutionCertification3D {
  std::uint64_t trajectory_revision{0U};
  mppi::FiniteHorizon horizon{};
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::int64_t valid_from_ns{0};
  FiniteExecutionKind3D kind{FiniteExecutionKind3D::kNominal};
};

enum class FiniteExecutionCertificationStatus3D : std::uint8_t {
  kCertified,
  kInvalidInput,
  kTargetRelationRejected,
  kProgressRelationRejected,
  kRawInvalidationContractRejected,
  kEvidenceContractRejected,
  kCollisionPolicyInvalid,
  kHorizonContractRejected,
  kInitialStateMismatch,
  kExecutionBindingRejected,
  kRawInvalidationConnectorRejected,
  kRouteAdherenceRejected,
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

struct RawInvalidatedFiniteExecutionCertification3D {
  RouteLifecycleEvent3D invalidation{};
  std::shared_ptr<const VersionedObservedRawWorld3D> invalidating_observed_raw_world;
  FiniteExecutionCertification3D finite_execution{};
};

struct DirectTrackingExecutionCertification3D {
  DirectTrackingOwnerIdentity3D identity{};
  std::uint64_t trajectory_revision{0U};
  Point3 target{};
  mppi::FiniteHorizon horizon{};
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

enum class ExecutionRoutePublicationStatus3D : std::uint8_t {
  kPublished,
  kInvalidCandidate,
  kStaleSnapshotVersion,
};

struct ExecutionRouteTransitionGuard3D {
  std::uint64_t expected_snapshot_version{0U};
  std::uint64_t expected_route_generation{0U};
  std::uint64_t expected_geometry_revision{0U};
};

class ExecutionRouteTransitionFactory3D;

struct ExecutionRouteTransitionResult3D {
  ExecutionRouteTransitionResult3D() = default;

  const ExecutionRouteTransitionStatus3D status{
      ExecutionRouteTransitionStatus3D::kInvalidCandidate};
  const ExecutionRouteSnapshot3D* const predecessor{nullptr};
  const std::shared_ptr<const ExecutionRouteSnapshot3D> next;

  [[nodiscard]] bool applied() const noexcept;

private:
  ExecutionRouteTransitionResult3D(
      ExecutionRouteTransitionStatus3D status_value,
      const ExecutionRouteSnapshot3D* predecessor_value,
      std::shared_ptr<const ExecutionRouteSnapshot3D> next_value);

  bool authorized_{false};

  friend class ExecutionRouteTransitionFactory3D;
  friend class ExecutionRouteSnapshotStore3D;
};

[[nodiscard]] bool
executionRouteGeometryValid3D(const ExecutionRouteGeometry3D& geometry,
                              const ActivatedRouteIdentity3D& identity) noexcept;

[[nodiscard]] std::optional<CertifiedRouteSuffix3D>
certifyExecutionRoute3D(const ExecutionRouteActivation3D& activation);

// Revalidates an already sealed certificate against a newer observation while
// retaining its immutable geometry owner. External activation candidates must
// continue to use certifyExecutionRoute3D so their geometry is defensively
// captured at the trust boundary.
[[nodiscard]] std::optional<CertifiedRouteSuffix3D> recertifyExecutionRoute3D(
    const CertifiedRouteSuffix3D& sealed_source,
    const RouteActivationObservation3D& observation,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world);

[[nodiscard]] std::optional<FiniteExecutionState3D>
certifyFiniteExecution3D(const ExecutionRouteSnapshot3D& current,
                         const CertifiedRouteSuffix3D& target_route,
                         FiniteExecutionCertification3D certification);

[[nodiscard]] FiniteExecutionCertificationResult3D
certifyFiniteExecution3DDetailed(const ExecutionRouteSnapshot3D& current,
                                 const CertifiedRouteSuffix3D& target_route,
                                 FiniteExecutionCertification3D certification);

[[nodiscard]] std::optional<FiniteExecutionState3D>
certifyFiniteExecution3D(const ExecutionRouteSnapshot3D& current,
                         FiniteExecutionCertification3D certification);

[[nodiscard]] std::optional<FiniteExecutionState3D>
certifyRawInvalidatedFiniteExecution3D(
    const ExecutionRouteSnapshot3D& current,
    RawInvalidatedFiniteExecutionCertification3D certification);

[[nodiscard]] std::optional<DirectTrackingFiniteExecution3D>
certifyDirectTrackingExecution3D(const ExecutionRouteSnapshot3D& current,
                                 DirectTrackingExecutionCertification3D certification);

[[nodiscard]] std::shared_ptr<const ExecutionRouteSnapshot3D>
makeInitialExecutionRouteSnapshot3D();

[[nodiscard]] ExecutionRouteTransitionResult3D activateCertifiedRoute3D(
    const ExecutionRouteSnapshot3D& current, std::uint64_t expected_snapshot_version,
    CertifiedRouteSuffix3D candidate, FiniteExecutionState3D candidate_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D advanceCertifiedRoute3D(
    const ExecutionRouteSnapshot3D& current,
    const ExecutionRouteTransitionGuard3D& guard,
    RouteExecutionObservation3D observation,
    std::shared_ptr<const VersionedExecutionInput3D> execution_input,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world);

[[nodiscard]] ExecutionRouteTransitionResult3D
replaceFiniteExecution3D(const ExecutionRouteSnapshot3D& current,
                         const ExecutionRouteTransitionGuard3D& guard,
                         std::optional<FiniteExecutionState3D> execution);

[[nodiscard]] ExecutionRouteTransitionResult3D
requireFiniteExecutionRevalidation3D(const ExecutionRouteSnapshot3D& current,
                                     const ExecutionRouteTransitionGuard3D& guard);

[[nodiscard]] ExecutionRouteTransitionResult3D
retireCertifiedRoute3D(const ExecutionRouteSnapshot3D& current,
                       const ExecutionRouteTransitionGuard3D& guard,
                       const RouteLifecycleEvent3D& event,
                       std::optional<FiniteExecutionState3D> retained_safe_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D
replaceCertifiedRoute3D(const ExecutionRouteSnapshot3D& current,
                        const ExecutionRouteTransitionGuard3D& guard,
                        CertifiedRouteSuffix3D successor,
                        std::optional<FiniteExecutionState3D> successor_execution,
                        const CertifiedRouteSplice3D& splice);

[[nodiscard]] ExecutionRouteTransitionResult3D
transferToDirectTracking3D(const ExecutionRouteSnapshot3D& current,
                           std::uint64_t expected_snapshot_version,
                           DirectTrackingFiniteExecution3D direct_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D
replaceDirectTrackingExecution3D(const ExecutionRouteSnapshot3D& current,
                                 std::uint64_t expected_snapshot_version,
                                 DirectTrackingFiniteExecution3D direct_execution);

[[nodiscard]] ExecutionRouteTransitionResult3D transferDirectTrackingToCertifiedRoute3D(
    const ExecutionRouteSnapshot3D& current, std::uint64_t expected_snapshot_version,
    CertifiedRouteSuffix3D successor, FiniteExecutionState3D successor_execution);

// A stationary hold may replace a finite execution owner only after its
// certified terminal-rest state has actually been reached. Revalidation of an
// existing hold preserves its stable hold_id and target.
[[nodiscard]] ExecutionRouteTransitionResult3D
transferToExecutionHold3D(const ExecutionRouteSnapshot3D& current,
                          std::uint64_t expected_snapshot_version,
                          StationaryExecutionHoldCertification3D certification);

// A mission capture may re-arm a stationary hold after an explicit revocation.
// This is a distinct transition: it cannot invent a hold for any other empty
// owner and it accepts only the typed, zero-control stationary-capture input.
[[nodiscard]] ExecutionRouteTransitionResult3D
armStationaryCaptureHold3D(const ExecutionRouteSnapshot3D& current,
                           std::uint64_t expected_snapshot_version,
                           StationaryExecutionHoldCertification3D certification);

// Revocation removes all executable authority while preserving monotonic route
// and owner lineage. It does not certify the offboard-local fail-safe action.
[[nodiscard]] ExecutionRouteTransitionResult3D
revokeExecution3D(const ExecutionRouteSnapshot3D& current,
                  std::uint64_t expected_snapshot_version);

[[nodiscard]] RouteEndpointSemantics3D
executionRouteEndpointSemantics3D(const ExecutionRouteSnapshot3D& snapshot) noexcept;

[[nodiscard]] std::string_view
finiteExecutionKind3DName(FiniteExecutionKind3D kind) noexcept;

[[nodiscard]] std::string_view finiteExecutionCertificationStatus3DName(
    FiniteExecutionCertificationStatus3D status) noexcept;

[[nodiscard]] std::string_view finiteExecutionRouteAdherenceStatus3DName(
    FiniteExecutionRouteAdherenceStatus3D status) noexcept;

[[nodiscard]] std::string_view
executionRoutePhase3DName(ExecutionRoutePhase3D phase) noexcept;

[[nodiscard]] std::string_view
executionRouteTransitionStatus3DName(ExecutionRouteTransitionStatus3D status) noexcept;

class ExecutionRouteSnapshotStore3D final {
public:
  ExecutionRouteSnapshotStore3D();

  [[nodiscard]] std::shared_ptr<const ExecutionRouteSnapshot3D> snapshot() const;
  [[nodiscard]] ExecutionRoutePublicationStatus3D
  publish(const std::shared_ptr<const ExecutionRouteSnapshot3D>& expected_snapshot,
          const ExecutionRouteTransitionResult3D& transition);

private:
  mutable std::mutex mutex_;
  std::shared_ptr<const ExecutionRouteSnapshot3D> snapshot_;
};

} // namespace drone_city_nav
