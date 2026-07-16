#pragma once

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
  bool known_static_result_available{false};
  KnownStaticLidarHitResult known_static_result{};
};

struct LidarIngestionDecisionSnapshot {
  LidarIngestionAction action{LidarIngestionAction::kSuppressAllUpdates};
  LidarIngestionReason reason{LidarIngestionReason::kClassificationUnavailable};
  LidarExpectedSurfaceKind expected_surface{LidarExpectedSurfaceKind::kNone};
  double expected_range_m{std::numeric_limits<double>::quiet_NaN()};
  double range_delta_m{std::numeric_limits<double>::quiet_NaN()};
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
};

struct LidarIngestionDecisionStats {
  std::size_t expected_ground_suppressed{0U};
  std::size_t closer_obstacles_retained{0U};
  std::size_t ambiguous_ground_suppressed{0U};
  std::size_t ground_classification_unavailable{0U};
  std::size_t ground_classification_disabled{0U};
  std::size_t non_ground_altitude_rejected{0U};
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

[[nodiscard]] LidarIngestionDecisionSnapshot
makeLidarIngestionDecisionSnapshot(const LidarIngestionDecision& decision) noexcept;

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
