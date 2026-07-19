#pragma once

#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"
#include "drone_city_nav/lidar_beam_observation.hpp"

#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace drone_city_nav {

struct GroundLidarRejectionConfig {
  bool enabled{true};
  double ground_altitude_m{0.05};
  double closer_range_tolerance_m{0.5};
  double farther_range_tolerance_m{1.5};
};

enum class LidarExpectedSurfaceProviderStatus {
  kReady,
  kDisabled,
  kUnavailable,
};

enum class LidarExpectedSurfaceKind {
  kNone,
  kKnownStatic,
  kGround,
  kTied,
};

enum class LidarIngestionAction {
  kIntegrateFreeAndHit,
  kIntegrateFreeOnly,
  kSuppressAllUpdates,
};

enum class LidarIngestionReason {
  kNoExpectedSurface,
  kObstacleBeforeExpectedSurface,
  kObstacleInsideOpening,
  kExpectedKnownStatic,
  kUnexpectedKnownStatic,
  kAmbiguousKnownStatic,
  kExpectedGround,
  kAmbiguousGround,
  kTiedExpectedSurfaces,
  kClassificationUnavailable,
};

enum class LidarIngestionDiagnosticClass {
  kExpectedGround,
  kCloserObstacle,
  kAmbiguousGround,
  kClassificationUnavailable,
  kClassificationDisabled,
  kNonGroundAltitudeRejected,
  kAmbiguousKnownStatic,
  kOpeningObstacle,
  kInvariantFallback,
  kCount,
};

struct LidarIngestionDecision {
  LidarIngestionAction action{LidarIngestionAction::kSuppressAllUpdates};
  LidarIngestionReason reason{LidarIngestionReason::kClassificationUnavailable};
  LidarExpectedSurfaceKind expected_surface{LidarExpectedSurfaceKind::kNone};
  LidarExpectedSurfaceProviderStatus ground_provider{
      LidarExpectedSurfaceProviderStatus::kDisabled};
  LidarExpectedSurfaceProviderStatus known_static_provider{
      LidarExpectedSurfaceProviderStatus::kDisabled};
  double expected_range_m{std::numeric_limits<double>::quiet_NaN()};
  double range_delta_m{std::numeric_limits<double>::quiet_NaN()};
  bool ground_candidate_considered{false};
  std::optional<double> expected_ground_range_m;
  std::optional<KnownStaticExpectedSurface> known_static_surface;
  bool known_static_result_available{false};
  KnownStaticLidarHitResult known_static_result{};
  AmbiguousLidarHitResolution ambiguous_resolution{
      AmbiguousLidarHitResolution::kPending};
  std::size_t ambiguous_evidence_count{0U};
  std::size_t ambiguous_expired_candidates{0U};
  double ambiguous_viewpoint_translation_m{0.0};
  double ambiguous_viewpoint_direction_change_rad{0.0};
};

struct LidarIngestionDecisionSnapshot {
  LidarIngestionAction action{LidarIngestionAction::kSuppressAllUpdates};
  LidarIngestionReason reason{LidarIngestionReason::kClassificationUnavailable};
  LidarExpectedSurfaceKind expected_surface{LidarExpectedSurfaceKind::kNone};
  double expected_range_m{std::numeric_limits<double>::quiet_NaN()};
  double range_delta_m{std::numeric_limits<double>::quiet_NaN()};
};

enum class LidarIngestionDecisionValidation {
  kValid,
  kActionNotAccepted,
  kInvalidMetadata,
};

struct LidarIngestionDecisionDiagnostic {
  LidarBeamObservation observation;
  LidarIngestionDiagnosticClass diagnostic_class{
      LidarIngestionDiagnosticClass::kClassificationUnavailable};
  LidarIngestionReason reason{LidarIngestionReason::kClassificationUnavailable};
  LidarExpectedSurfaceKind expected_surface{LidarExpectedSurfaceKind::kNone};
  LidarExpectedSurfaceProviderStatus ground_provider{
      LidarExpectedSurfaceProviderStatus::kDisabled};
  LidarExpectedSurfaceProviderStatus known_static_provider{
      LidarExpectedSurfaceProviderStatus::kDisabled};
  double expected_range_m{std::numeric_limits<double>::quiet_NaN()};
  double range_delta_m{std::numeric_limits<double>::quiet_NaN()};
  std::string structure_id;
  std::string opening_id;
  std::string part_id;
  KnownStaticEndpointRelation endpoint_relation{KnownStaticEndpointRelation::kOutside};
  double endpoint_solid_distance_m{std::numeric_limits<double>::infinity()};
  double endpoint_opening_margin_m{-std::numeric_limits<double>::infinity()};
  double distance_before_solid_m{std::numeric_limits<double>::quiet_NaN()};
  double incidence_angle_rad{std::numeric_limits<double>::quiet_NaN()};
  AmbiguousLidarHitResolution ambiguous_resolution{
      AmbiguousLidarHitResolution::kPending};
  std::size_t ambiguous_evidence_count{0U};
  double ambiguous_viewpoint_translation_m{0.0};
  double ambiguous_viewpoint_direction_change_rad{0.0};
};

struct LidarIngestionDecisionStats {
  std::size_t expected_ground_suppressed{0U};
  std::size_t closer_obstacles_retained{0U};
  std::size_t ambiguous_ground_suppressed{0U};
  std::size_t ground_classification_unavailable{0U};
  std::size_t ground_classification_disabled{0U};
  std::size_t non_ground_altitude_rejected{0U};
  std::size_t closer_side_static_suppressed{0U};
  std::size_t closer_side_static_pending{0U};
  std::size_t closer_side_static_confirmed{0U};
  std::size_t detached_obstacles_confirmed{0U};
  std::size_t opening_obstacles_integrated{0U};
  std::size_t ambiguous_expired{0U};
  std::size_t invariant_fallbacks{0U};
  std::vector<LidarIngestionDecisionDiagnostic> diagnostics;
};

struct LidarIngestionRepresentativeDiagnostics {
  std::array<const LidarIngestionDecisionDiagnostic*,
             static_cast<std::size_t>(LidarIngestionDiagnosticClass::kCount)>
      samples{};
  std::size_t count{0U};
};

[[nodiscard]] LidarIngestionDecision
evaluateLidarIngestion(const LidarBeamObservation& observation,
                       const KnownStaticLidarHitClassifier* known_static_classifier,
                       const GroundLidarRejectionConfig* ground_config) noexcept;

[[nodiscard]] LidarIngestionDecision
resolveAmbiguousKnownStaticIngestion(const LidarBeamObservation& observation,
                                     LidarIngestionDecision decision,
                                     AmbiguousLidarHitTracker* tracker);

[[nodiscard]] LidarIngestionDecisionSnapshot
makeLidarIngestionDecisionSnapshot(const LidarIngestionDecision& decision) noexcept;

[[nodiscard]] LidarIngestionDecisionValidation validateAcceptedLidarIngestionDecision(
    const LidarIngestionDecisionSnapshot& decision) noexcept;

[[nodiscard]] LidarIngestionDecisionSnapshot
conservativeUnknownObstacleDecisionSnapshot() noexcept;

[[nodiscard]] LidarIngestionDecision
normalizeAcceptedLidarIngestionDecision(const LidarBeamObservation& observation,
                                        LidarIngestionDecision decision,
                                        LidarIngestionDecisionStats& stats);

void recordLidarIngestionDecision(const LidarBeamObservation& observation,
                                  const LidarIngestionDecision& decision,
                                  bool altitude_rejected,
                                  LidarIngestionDecisionStats& stats);

[[nodiscard]] LidarIngestionRepresentativeDiagnostics
representativeLidarIngestionDiagnostics(
    const LidarIngestionDecisionStats& stats) noexcept;

[[nodiscard]] std::string
formatLidarIngestionRepresentativeDiagnostics(const LidarIngestionDecisionStats& stats);

[[nodiscard]] const char*
lidarIngestionReasonName(LidarIngestionReason reason) noexcept;

[[nodiscard]] const char*
lidarIngestionActionName(LidarIngestionAction action) noexcept;

[[nodiscard]] const char* lidarIngestionDiagnosticClassName(
    LidarIngestionDiagnosticClass diagnostic_class) noexcept;

[[nodiscard]] const char*
lidarExpectedSurfaceKindName(LidarExpectedSurfaceKind kind) noexcept;

[[nodiscard]] const char* lidarExpectedSurfaceProviderStatusName(
    LidarExpectedSurfaceProviderStatus status) noexcept;

} // namespace drone_city_nav
