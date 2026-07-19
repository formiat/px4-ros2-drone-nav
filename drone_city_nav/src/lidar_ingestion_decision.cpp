#include "drone_city_nav/lidar_ingestion_decision.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kDirectionNormTolerance = 1.0e-6;
constexpr double kDirectionZEpsilon = 1.0e-9;
constexpr double kRangeTieToleranceM = 1.0e-6;
constexpr double kRangeComparisonEpsilonM = 1.0e-9;
constexpr std::size_t kMaxDecisionDiagnosticsPerClass = 4U;

struct SurfaceCandidate {
  LidarExpectedSurfaceKind kind{LidarExpectedSurfaceKind::kNone};
  double range_m{0.0};
  double closer_tolerance_m{0.0};
};

[[nodiscard]] bool finitePoint3(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool
validObservationGeometry(const LidarBeamObservation& observation) noexcept {
  const Point3& direction = observation.projection.ray_direction_map;
  const double direction_norm_sq =
      direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
  return observation.projection.endpoint_xyz_valid &&
         finitePoint3(observation.projection.ray_origin_map_m) &&
         finitePoint3(direction) &&
         finitePoint3(observation.projection.endpoint_map_m) &&
         std::isfinite(direction_norm_sq) &&
         std::abs(direction_norm_sq - 1.0) <= kDirectionNormTolerance &&
         std::isfinite(observation.effective_max_range_m) &&
         observation.effective_max_range_m > 0.0 &&
         (!observation.attitude_compensation_required ||
          (observation.source_attitude_valid &&
           observation.projection.attitude_compensation_applied));
}

[[nodiscard]] bool
validGroundConfig(const GroundLidarRejectionConfig& config) noexcept {
  return std::isfinite(config.ground_altitude_m) &&
         std::isfinite(config.closer_range_tolerance_m) &&
         config.closer_range_tolerance_m >= 0.0 &&
         std::isfinite(config.farther_range_tolerance_m) &&
         config.farther_range_tolerance_m >= 0.0;
}

[[nodiscard]] LidarIngestionAction
legacyAction(const LidarBeamObservation& observation) noexcept {
  return observation.projection.hit ? LidarIngestionAction::kIntegrateFreeAndHit
                                    : LidarIngestionAction::kIntegrateFreeOnly;
}

[[nodiscard]] std::optional<double>
groundIntersectionRange(const LidarBeamObservation& observation,
                        const GroundLidarRejectionConfig& config) noexcept {
  const Point3& origin = observation.projection.ray_origin_map_m;
  const Point3& direction = observation.projection.ray_direction_map;
  if (direction.z >= -kDirectionZEpsilon) {
    return std::nullopt;
  }
  const double range_m = (config.ground_altitude_m - origin.z) / direction.z;
  if (!std::isfinite(range_m) || range_m <= 0.0 ||
      range_m > observation.effective_max_range_m) {
    return std::nullopt;
  }
  return range_m;
}

[[nodiscard]] LidarIngestionReason
knownStaticReason(const KnownStaticLidarHitClassification classification) noexcept {
  switch (classification) {
    case KnownStaticLidarHitClassification::kExpectedStatic:
      return LidarIngestionReason::kExpectedKnownStatic;
    case KnownStaticLidarHitClassification::kUnexpected:
      return LidarIngestionReason::kUnexpectedKnownStatic;
    case KnownStaticLidarHitClassification::kAmbiguous:
      return LidarIngestionReason::kAmbiguousKnownStatic;
  }
  return LidarIngestionReason::kClassificationUnavailable;
}

[[nodiscard]] std::size_t
diagnosticClassIndex(const LidarIngestionDiagnosticClass diagnostic_class) noexcept {
  return static_cast<std::size_t>(diagnostic_class);
}

void appendDiagnostic(const LidarBeamObservation& observation,
                      const LidarIngestionDecision& decision,
                      const LidarIngestionDiagnosticClass diagnostic_class,
                      LidarIngestionDecisionStats& stats) {
  const std::size_t existing = static_cast<std::size_t>(std::count_if(
      stats.diagnostics.begin(), stats.diagnostics.end(),
      [diagnostic_class](const LidarIngestionDecisionDiagnostic& diagnostic) {
        return diagnostic.diagnostic_class == diagnostic_class;
      }));
  if (existing >= kMaxDecisionDiagnosticsPerClass) {
    return;
  }
  LidarIngestionDecisionDiagnostic diagnostic{
      .observation = observation,
      .diagnostic_class = diagnostic_class,
      .reason = decision.reason,
      .expected_surface = decision.expected_surface,
      .ground_provider = decision.ground_provider,
      .known_static_provider = decision.known_static_provider,
      .expected_range_m = decision.expected_range_m,
      .range_delta_m = decision.range_delta_m,
      .structure_id = {},
      .opening_id = {},
      .part_id = {},
      .endpoint_relation = KnownStaticEndpointRelation::kOutside,
      .endpoint_solid_distance_m = std::numeric_limits<double>::infinity(),
      .endpoint_opening_margin_m = -std::numeric_limits<double>::infinity(),
      .distance_before_solid_m = std::numeric_limits<double>::quiet_NaN(),
      .incidence_angle_rad = std::numeric_limits<double>::quiet_NaN(),
      .ambiguous_resolution = decision.ambiguous_resolution,
      .ambiguous_evidence_count = decision.ambiguous_evidence_count,
      .ambiguous_viewpoint_translation_m = decision.ambiguous_viewpoint_translation_m,
      .ambiguous_viewpoint_direction_change_rad =
          decision.ambiguous_viewpoint_direction_change_rad,
  };
  if (decision.known_static_result_available) {
    diagnostic.structure_id = decision.known_static_result.structure_id;
    diagnostic.opening_id = decision.known_static_result.opening_id;
    diagnostic.part_id = decision.known_static_result.part_id;
    diagnostic.endpoint_relation = decision.known_static_result.endpoint_relation;
    diagnostic.endpoint_solid_distance_m =
        decision.known_static_result.endpoint_solid_distance_m;
    diagnostic.endpoint_opening_margin_m =
        decision.known_static_result.endpoint_opening_margin_m;
    diagnostic.distance_before_solid_m =
        decision.known_static_result.distance_before_solid_m;
    diagnostic.incidence_angle_rad = decision.known_static_result.incidence_angle_rad;
  }
  stats.diagnostics.push_back(std::move(diagnostic));
}

void applyKnownStaticResult(LidarIngestionDecision& decision,
                            const KnownStaticLidarHitResult& result) noexcept {
  decision.known_static_result = result;
  decision.known_static_result_available = true;
  decision.range_delta_m = result.range_delta_m;
  if (result.endpoint_relation == KnownStaticEndpointRelation::kInsideOpening) {
    decision.action = LidarIngestionAction::kIntegrateFreeAndHit;
    decision.reason = LidarIngestionReason::kObstacleInsideOpening;
    return;
  }
  decision.reason = knownStaticReason(result.classification);
  switch (result.classification) {
    case KnownStaticLidarHitClassification::kExpectedStatic:
    case KnownStaticLidarHitClassification::kAmbiguous:
      decision.action = LidarIngestionAction::kSuppressAllUpdates;
      return;
    case KnownStaticLidarHitClassification::kUnexpected:
      decision.action = LidarIngestionAction::kIntegrateFreeAndHit;
      return;
  }
}

} // namespace

LidarIngestionDecision
evaluateLidarIngestion(const LidarBeamObservation& observation,
                       const KnownStaticLidarHitClassifier* known_static_classifier,
                       const GroundLidarRejectionConfig* ground_config) noexcept {
  LidarIngestionDecision decision{};
  decision.action = legacyAction(observation);
  decision.reason = LidarIngestionReason::kNoExpectedSurface;
  decision.known_static_provider = known_static_classifier != nullptr
                                       ? LidarExpectedSurfaceProviderStatus::kReady
                                       : LidarExpectedSurfaceProviderStatus::kDisabled;
  if (ground_config == nullptr || !ground_config->enabled) {
    decision.ground_provider = LidarExpectedSurfaceProviderStatus::kDisabled;
  } else if (!validGroundConfig(*ground_config)) {
    decision.ground_provider = LidarExpectedSurfaceProviderStatus::kUnavailable;
  } else {
    decision.ground_provider = LidarExpectedSurfaceProviderStatus::kReady;
  }

  if (!validObservationGeometry(observation)) {
    if (ground_config != nullptr && ground_config->enabled) {
      decision.ground_provider = LidarExpectedSurfaceProviderStatus::kUnavailable;
    }
    if (known_static_classifier != nullptr) {
      decision.known_static_provider = LidarExpectedSurfaceProviderStatus::kUnavailable;
    }
    decision.reason = LidarIngestionReason::kClassificationUnavailable;
    return decision;
  }

  std::optional<SurfaceCandidate> ground_candidate;
  if (decision.ground_provider == LidarExpectedSurfaceProviderStatus::kReady) {
    const std::optional<double> range =
        groundIntersectionRange(observation, *ground_config);
    if (range.has_value()) {
      decision.ground_candidate_considered = true;
      decision.expected_ground_range_m = range;
      ground_candidate = SurfaceCandidate{LidarExpectedSurfaceKind::kGround, *range,
                                          ground_config->closer_range_tolerance_m};
    }
  }

  std::optional<SurfaceCandidate> known_candidate;
  std::optional<KnownStaticExpectedSurface> known_surface;
  if (known_static_classifier != nullptr) {
    known_surface = known_static_classifier->nearestExpectedSurface(
        observation.projection.ray_origin_map_m,
        observation.projection.ray_direction_map, observation.effective_max_range_m);
    if (known_surface.has_value()) {
      decision.known_static_surface = *known_surface;
      known_candidate = SurfaceCandidate{
          LidarExpectedSurfaceKind::kKnownStatic, known_surface->range_m,
          known_static_classifier->closerRangeToleranceM()};
    }
  }

  const bool measured_hit =
      observation.projection.hit && std::isfinite(observation.measured_range_m);
  std::optional<KnownStaticLidarHitResult> measured_known_result;
  if (known_static_classifier != nullptr && measured_hit) {
    measured_known_result = known_static_classifier->classify(
        observation.projection.ray_origin_map_m,
        observation.projection.ray_direction_map, observation.measured_range_m);
    if (measured_known_result->endpoint_relation ==
        KnownStaticEndpointRelation::kInsideOpening) {
      decision.expected_surface = LidarExpectedSurfaceKind::kKnownStatic;
      applyKnownStaticResult(decision, *measured_known_result);
      return decision;
    }
  }

  if (!ground_candidate.has_value() && !known_candidate.has_value()) {
    if (measured_known_result.has_value() &&
        (measured_known_result->volume_matched ||
         measured_known_result->endpoint_relation !=
             KnownStaticEndpointRelation::kOutside)) {
      decision.expected_surface = LidarExpectedSurfaceKind::kKnownStatic;
      applyKnownStaticResult(decision, *measured_known_result);
    }
    return decision;
  }

  const double nearest_range =
      std::min(ground_candidate.has_value() ? ground_candidate->range_m
                                            : std::numeric_limits<double>::infinity(),
               known_candidate.has_value() ? known_candidate->range_m
                                           : std::numeric_limits<double>::infinity());
  const bool ground_tied =
      ground_candidate.has_value() &&
      std::abs(ground_candidate->range_m - nearest_range) <= kRangeTieToleranceM;
  const bool known_tied =
      known_candidate.has_value() &&
      std::abs(known_candidate->range_m - nearest_range) <= kRangeTieToleranceM;
  const bool hit_before_ground =
      !ground_tied || (measured_hit && observation.measured_range_m <
                                           ground_candidate->range_m -
                                               ground_candidate->closer_tolerance_m -
                                               kRangeComparisonEpsilonM);
  const bool hit_before_known =
      !known_tied || (measured_hit && observation.measured_range_m <
                                          known_candidate->range_m -
                                              known_candidate->closer_tolerance_m -
                                              kRangeComparisonEpsilonM);
  if (measured_hit && hit_before_ground && hit_before_known) {
    if (known_tied && measured_known_result.has_value() &&
        measured_known_result->classification !=
            KnownStaticLidarHitClassification::kUnexpected) {
      decision.expected_surface = LidarExpectedSurfaceKind::kKnownStatic;
      decision.expected_range_m = known_candidate->range_m;
      applyKnownStaticResult(decision, *measured_known_result);
      return decision;
    }
    decision.action = LidarIngestionAction::kIntegrateFreeAndHit;
    decision.reason = LidarIngestionReason::kObstacleBeforeExpectedSurface;
    decision.expected_surface = ground_tied ? LidarExpectedSurfaceKind::kGround
                                            : LidarExpectedSurfaceKind::kKnownStatic;
    decision.expected_range_m = nearest_range;
    decision.range_delta_m = observation.measured_range_m - nearest_range;
    if (known_tied && measured_known_result.has_value()) {
      decision.known_static_result = *measured_known_result;
      decision.known_static_result_available = true;
    }
    return decision;
  }

  if (ground_tied && known_tied) {
    const KnownStaticExpectedSurface& tied_known_surface =
        *known_surface; // NOLINT(bugprone-unchecked-optional-access)
    decision.action = LidarIngestionAction::kSuppressAllUpdates;
    decision.reason = LidarIngestionReason::kTiedExpectedSurfaces;
    decision.expected_surface = LidarExpectedSurfaceKind::kTied;
    decision.expected_range_m = nearest_range;
    decision.range_delta_m = measured_hit ? observation.measured_range_m - nearest_range
                                          : std::numeric_limits<double>::quiet_NaN();
    decision.known_static_result = KnownStaticLidarHitResult{
        .classification = KnownStaticLidarHitClassification::kAmbiguous,
        .expected_range_m = tied_known_surface.range_m,
        .range_delta_m = decision.range_delta_m,
        .part_kind = tied_known_surface.part_kind,
        .structure_id = tied_known_surface.structure_id,
        .opening_id = tied_known_surface.opening_id,
        .part_id = tied_known_surface.part_id,
        .volume_matched = true,
        .confident_face_interior = tied_known_surface.confident_face_interior,
    };
    decision.known_static_result_available = true;
    return decision;
  }

  if (known_tied) {
    decision.expected_surface = LidarExpectedSurfaceKind::kKnownStatic;
    decision.expected_range_m = known_candidate->range_m;
    if (!measured_hit) {
      decision.action = LidarIngestionAction::kIntegrateFreeOnly;
      return decision;
    }
    if (!measured_known_result.has_value()) {
      decision.action = LidarIngestionAction::kSuppressAllUpdates;
      decision.reason = LidarIngestionReason::kClassificationUnavailable;
      return decision;
    }
    applyKnownStaticResult(decision, *measured_known_result);
    return decision;
  }

  const SurfaceCandidate& selected_ground =
      *ground_candidate; // NOLINT(bugprone-unchecked-optional-access)
  decision.expected_surface = LidarExpectedSurfaceKind::kGround;
  decision.expected_range_m = selected_ground.range_m;
  decision.action = LidarIngestionAction::kSuppressAllUpdates;
  if (!measured_hit) {
    decision.reason = LidarIngestionReason::kAmbiguousGround;
    return decision;
  }
  decision.range_delta_m = observation.measured_range_m - selected_ground.range_m;
  if (observation.measured_range_m <= selected_ground.range_m +
                                          ground_config->farther_range_tolerance_m +
                                          kRangeComparisonEpsilonM) {
    decision.reason = LidarIngestionReason::kExpectedGround;
  } else {
    decision.reason = LidarIngestionReason::kAmbiguousGround;
  }
  return decision;
}

LidarIngestionDecision
resolveAmbiguousKnownStaticIngestion(const LidarBeamObservation& observation,
                                     LidarIngestionDecision decision,
                                     AmbiguousLidarHitTracker* tracker) {
  if (!decision.known_static_result_available ||
      decision.reason != LidarIngestionReason::kAmbiguousKnownStatic ||
      decision.known_static_result.classification !=
          KnownStaticLidarHitClassification::kAmbiguous) {
    return decision;
  }
  decision.action = LidarIngestionAction::kSuppressAllUpdates;
  decision.reason = LidarIngestionReason::kAmbiguousKnownStatic;
  if (tracker == nullptr) {
    return decision;
  }
  std::int64_t scan_stamp_ns = 0;
  if (observation.scan_stamp_valid) {
    scan_stamp_ns = observation.scan_stamp_ns;
  } else if (observation.receive_stamp_valid) {
    scan_stamp_ns = observation.receive_stamp_ns;
  }
  const KnownStaticLidarHitResult& result = decision.known_static_result;
  const AmbiguousLidarHitConfirmation confirmation =
      tracker->observe(AmbiguousStaticHitObservation{
          .structure_id = result.structure_id,
          .part_id = result.part_id,
          .endpoint_map_m = observation.projection.endpoint_map_m,
          .ray_origin_map_m = observation.projection.ray_origin_map_m,
          .ray_direction_map = observation.projection.ray_direction_map,
          .endpoint_relation = result.endpoint_relation,
          .endpoint_solid_distance_m = result.endpoint_solid_distance_m,
          .distance_before_solid_m = result.distance_before_solid_m,
          .range_residual_m = result.range_delta_m,
          .scan_stamp_ns = scan_stamp_ns,
      });
  decision.ambiguous_resolution = confirmation.resolution;
  decision.ambiguous_evidence_count = confirmation.independent_scans;
  decision.ambiguous_expired_candidates = confirmation.expired_candidates;
  decision.ambiguous_viewpoint_translation_m = confirmation.viewpoint_translation_m;
  decision.ambiguous_viewpoint_direction_change_rad =
      confirmation.viewpoint_direction_change_rad;
  switch (confirmation.resolution) {
    case AmbiguousLidarHitResolution::kPending:
      return decision;
    case AmbiguousLidarHitResolution::kConfirmedStaticAttached:
      decision.known_static_result.classification =
          KnownStaticLidarHitClassification::kExpectedStatic;
      decision.reason = LidarIngestionReason::kExpectedKnownStatic;
      return decision;
    case AmbiguousLidarHitResolution::kConfirmedDetachedObstacle:
      decision.known_static_result.classification =
          KnownStaticLidarHitClassification::kUnexpected;
      decision.action = LidarIngestionAction::kIntegrateFreeAndHit;
      decision.reason = LidarIngestionReason::kObstacleBeforeExpectedSurface;
      return decision;
  }
  return decision;
}

LidarIngestionDecisionSnapshot
makeLidarIngestionDecisionSnapshot(const LidarIngestionDecision& decision) noexcept {
  return LidarIngestionDecisionSnapshot{
      .action = decision.action,
      .reason = decision.reason,
      .expected_surface = decision.expected_surface,
      .expected_range_m = decision.expected_range_m,
      .range_delta_m = decision.range_delta_m,
  };
}

void recordLidarIngestionDecision(const LidarBeamObservation& observation,
                                  const LidarIngestionDecision& decision,
                                  const bool altitude_rejected,
                                  LidarIngestionDecisionStats& stats) {
  const bool closer_side_known_static =
      decision.known_static_result_available &&
      std::isfinite(decision.known_static_result.distance_before_solid_m) &&
      decision.known_static_result.distance_before_solid_m > 0.0;
  if (decision.ground_provider == LidarExpectedSurfaceProviderStatus::kUnavailable) {
    ++stats.ground_classification_unavailable;
    appendDiagnostic(observation, decision,
                     LidarIngestionDiagnosticClass::kClassificationUnavailable, stats);
  } else if (decision.ground_provider ==
             LidarExpectedSurfaceProviderStatus::kDisabled) {
    ++stats.ground_classification_disabled;
    appendDiagnostic(observation, decision,
                     LidarIngestionDiagnosticClass::kClassificationDisabled, stats);
  }
  switch (decision.reason) {
    case LidarIngestionReason::kExpectedGround:
      ++stats.expected_ground_suppressed;
      appendDiagnostic(observation, decision,
                       LidarIngestionDiagnosticClass::kExpectedGround, stats);
      break;
    case LidarIngestionReason::kObstacleBeforeExpectedSurface:
      if (!altitude_rejected) {
        ++stats.closer_obstacles_retained;
        appendDiagnostic(observation, decision,
                         LidarIngestionDiagnosticClass::kCloserObstacle, stats);
      }
      break;
    case LidarIngestionReason::kObstacleInsideOpening:
      ++stats.opening_obstacles_integrated;
      appendDiagnostic(observation, decision,
                       LidarIngestionDiagnosticClass::kOpeningObstacle, stats);
      break;
    case LidarIngestionReason::kAmbiguousKnownStatic:
      if (closer_side_known_static) {
        ++stats.closer_side_static_pending;
      }
      appendDiagnostic(observation, decision,
                       LidarIngestionDiagnosticClass::kAmbiguousKnownStatic, stats);
      break;
    case LidarIngestionReason::kAmbiguousGround:
    case LidarIngestionReason::kTiedExpectedSurfaces:
      ++stats.ambiguous_ground_suppressed;
      appendDiagnostic(observation, decision,
                       LidarIngestionDiagnosticClass::kAmbiguousGround, stats);
      break;
    default:
      break;
  }
  if (closer_side_known_static &&
      decision.reason == LidarIngestionReason::kExpectedKnownStatic) {
    if (decision.ambiguous_resolution ==
        AmbiguousLidarHitResolution::kConfirmedStaticAttached) {
      ++stats.closer_side_static_confirmed;
    } else {
      ++stats.closer_side_static_suppressed;
    }
  }
  if (decision.ambiguous_resolution ==
      AmbiguousLidarHitResolution::kConfirmedDetachedObstacle) {
    ++stats.detached_obstacles_confirmed;
  }
  stats.ambiguous_expired += decision.ambiguous_expired_candidates;
  const bool ground_explained =
      decision.expected_surface == LidarExpectedSurfaceKind::kGround ||
      decision.expected_surface == LidarExpectedSurfaceKind::kTied;
  if (altitude_rejected && !ground_explained) {
    ++stats.non_ground_altitude_rejected;
    appendDiagnostic(observation, decision,
                     LidarIngestionDiagnosticClass::kNonGroundAltitudeRejected, stats);
  }
}

LidarIngestionRepresentativeDiagnostics representativeLidarIngestionDiagnostics(
    const LidarIngestionDecisionStats& stats) noexcept {
  LidarIngestionRepresentativeDiagnostics representatives;
  std::array<bool, static_cast<std::size_t>(LidarIngestionDiagnosticClass::kCount)>
      seen{};
  for (const LidarIngestionDecisionDiagnostic& diagnostic : stats.diagnostics) {
    const std::size_t index = diagnosticClassIndex(diagnostic.diagnostic_class);
    if (index >= seen.size() || seen.at(index)) {
      continue;
    }
    seen.at(index) = true;
    representatives.samples.at(representatives.count) = &diagnostic;
    ++representatives.count;
  }
  return representatives;
}

std::string formatLidarIngestionRepresentativeDiagnostics(
    const LidarIngestionDecisionStats& stats) {
  const LidarIngestionRepresentativeDiagnostics representatives =
      representativeLidarIngestionDiagnostics(stats);
  std::ostringstream stream;
  for (std::size_t i = 0U; i < representatives.count; ++i) {
    const LidarIngestionDecisionDiagnostic& diagnostic = *representatives.samples.at(i);
    const LidarBeamObservation& observation = diagnostic.observation;
    if (i != 0U) {
      stream << "; ";
    }
    stream << "class=" << lidarIngestionDiagnosticClassName(diagnostic.diagnostic_class)
           << " reason=" << lidarIngestionReasonName(diagnostic.reason)
           << " surface=" << lidarExpectedSurfaceKindName(diagnostic.expected_surface)
           << " ground_provider="
           << lidarExpectedSurfaceProviderStatusName(diagnostic.ground_provider)
           << " known_provider="
           << lidarExpectedSurfaceProviderStatusName(diagnostic.known_static_provider)
           << " beam=" << observation.beam_index << " endpoint=("
           << observation.projection.endpoint_map_m.x << ','
           << observation.projection.endpoint_map_m.y << ','
           << observation.projection.endpoint_map_m.z
           << ") measured=" << observation.measured_range_m
           << " expected=" << diagnostic.expected_range_m
           << " delta=" << diagnostic.range_delta_m << " ray_origin=("
           << observation.projection.ray_origin_map_m.x << ','
           << observation.projection.ray_origin_map_m.y << ','
           << observation.projection.ray_origin_map_m.z << ") ray_dir=("
           << observation.projection.ray_direction_map.x << ','
           << observation.projection.ray_direction_map.y << ','
           << observation.projection.ray_direction_map.z << ") source_attitude=(valid="
           << (observation.source_attitude_valid ? "true" : "false")
           << " roll=" << observation.source_roll_rad
           << " pitch=" << observation.source_pitch_rad
           << " tilt=" << observation.source_tilt_rad << ") applied_attitude=(applied="
           << (observation.projection.attitude_compensation_applied ? "true" : "false")
           << " roll=" << observation.projection.applied_roll_rad
           << " pitch=" << observation.projection.applied_pitch_rad
           << " tilt=" << observation.projection.applied_tilt_rad << ") pose_aligned="
           << (observation.timestamp_aligned_pose ? "true" : "false")
           << " endpoint_relation="
           << knownStaticEndpointRelationName(diagnostic.endpoint_relation)
           << " solid_distance=" << diagnostic.endpoint_solid_distance_m
           << " opening_margin=" << diagnostic.endpoint_opening_margin_m
           << " distance_before_solid=" << diagnostic.distance_before_solid_m
           << " incidence_angle=" << diagnostic.incidence_angle_rad
           << " evidence=" << diagnostic.ambiguous_evidence_count
           << " viewpoint_translation=" << diagnostic.ambiguous_viewpoint_translation_m
           << " viewpoint_direction_delta="
           << diagnostic.ambiguous_viewpoint_direction_change_rad << " resolution="
           << ambiguousLidarHitResolutionName(diagnostic.ambiguous_resolution);
  }
  return stream.str();
}

const char* lidarIngestionReasonName(const LidarIngestionReason reason) noexcept {
  switch (reason) {
    case LidarIngestionReason::kNoExpectedSurface:
      return "no_expected_surface";
    case LidarIngestionReason::kObstacleBeforeExpectedSurface:
      return "obstacle_before_expected_surface";
    case LidarIngestionReason::kObstacleInsideOpening:
      return "obstacle_inside_opening";
    case LidarIngestionReason::kExpectedKnownStatic:
      return "expected_known_static";
    case LidarIngestionReason::kUnexpectedKnownStatic:
      return "unexpected_known_static";
    case LidarIngestionReason::kAmbiguousKnownStatic:
      return "ambiguous_known_static";
    case LidarIngestionReason::kExpectedGround:
      return "expected_ground";
    case LidarIngestionReason::kAmbiguousGround:
      return "ambiguous_ground";
    case LidarIngestionReason::kTiedExpectedSurfaces:
      return "tied_expected_surfaces";
    case LidarIngestionReason::kClassificationUnavailable:
      return "classification_unavailable";
  }
  return "unknown";
}

const char* lidarIngestionActionName(const LidarIngestionAction action) noexcept {
  switch (action) {
    case LidarIngestionAction::kIntegrateFreeAndHit:
      return "integrate_free_and_hit";
    case LidarIngestionAction::kIntegrateFreeOnly:
      return "integrate_free_only";
    case LidarIngestionAction::kSuppressAllUpdates:
      return "suppress_all_updates";
  }
  return "unknown";
}

const char* lidarIngestionDiagnosticClassName(
    const LidarIngestionDiagnosticClass diagnostic_class) noexcept {
  switch (diagnostic_class) {
    case LidarIngestionDiagnosticClass::kExpectedGround:
      return "expected_ground";
    case LidarIngestionDiagnosticClass::kCloserObstacle:
      return "closer_obstacle";
    case LidarIngestionDiagnosticClass::kAmbiguousGround:
      return "ambiguous_ground";
    case LidarIngestionDiagnosticClass::kClassificationUnavailable:
      return "classification_unavailable";
    case LidarIngestionDiagnosticClass::kClassificationDisabled:
      return "classification_disabled";
    case LidarIngestionDiagnosticClass::kNonGroundAltitudeRejected:
      return "non_ground_altitude_rejected";
    case LidarIngestionDiagnosticClass::kAmbiguousKnownStatic:
      return "ambiguous_known_static";
    case LidarIngestionDiagnosticClass::kOpeningObstacle:
      return "opening_obstacle";
    case LidarIngestionDiagnosticClass::kCount:
      break;
  }
  return "unknown";
}

const char* lidarExpectedSurfaceKindName(const LidarExpectedSurfaceKind kind) noexcept {
  switch (kind) {
    case LidarExpectedSurfaceKind::kNone:
      return "none";
    case LidarExpectedSurfaceKind::kKnownStatic:
      return "known_static";
    case LidarExpectedSurfaceKind::kGround:
      return "ground";
    case LidarExpectedSurfaceKind::kTied:
      return "tied";
  }
  return "unknown";
}

const char* lidarExpectedSurfaceProviderStatusName(
    const LidarExpectedSurfaceProviderStatus status) noexcept {
  switch (status) {
    case LidarExpectedSurfaceProviderStatus::kReady:
      return "ready";
    case LidarExpectedSurfaceProviderStatus::kDisabled:
      return "disabled";
    case LidarExpectedSurfaceProviderStatus::kUnavailable:
      return "unavailable";
  }
  return "unknown";
}

} // namespace drone_city_nav
