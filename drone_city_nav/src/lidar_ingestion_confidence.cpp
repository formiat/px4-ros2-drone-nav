#include "drone_city_nav/lidar_ingestion_decision.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <string_view>

namespace drone_city_nav {
namespace {

struct UncertainCandidate {
  UncertainLidarHitKind kind{UncertainLidarHitKind::kNone};
  UncertainLidarHitEvidence evidence{UncertainLidarHitEvidence::kDetachedObstacle};
  std::string_view association_id;
  std::string_view part_id;
  double endpoint_surface_distance_m{0.0};
  double distance_before_surface_m{0.0};
  double range_residual_m{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] std::int64_t
observationScanStamp(const LidarBeamObservation& observation) noexcept {
  if (observation.scan_stamp_valid) {
    return observation.scan_stamp_ns;
  }
  return observation.receive_stamp_valid ? observation.receive_stamp_ns : 0;
}

[[nodiscard]] bool
validConfidenceConfig(const LidarIngestionConfidenceConfig& config) noexcept {
  return std::isfinite(config.reliable_range_margin_m) &&
         config.reliable_range_margin_m >= 0.0;
}

[[nodiscard]] bool
validGroundConfidenceConfig(const GroundLidarRejectionConfig* const config) noexcept {
  return config != nullptr && config->enabled &&
         std::isfinite(config->ground_altitude_m) &&
         std::isfinite(config->candidate_endpoint_altitude_tolerance_m) &&
         config->candidate_endpoint_altitude_tolerance_m >= 0.0 &&
         std::isfinite(config->attached_endpoint_altitude_tolerance_m) &&
         config->attached_endpoint_altitude_tolerance_m >= 0.0 &&
         config->attached_endpoint_altitude_tolerance_m <=
             config->candidate_endpoint_altitude_tolerance_m;
}

[[nodiscard]] bool
sourceTimestampAligned(const LidarBeamObservation& observation) noexcept {
  return observation.timestamp_aligned_pose &&
         observation.projection_pose_source ==
             LidarProjectionPoseSource::kSourceTimestampAligned;
}

[[nodiscard]] bool
reliableMeasuredRange(const LidarBeamObservation& observation,
                      const LidarIngestionConfidenceConfig& config) noexcept {
  return std::isfinite(observation.measured_range_m) &&
         std::isfinite(observation.effective_max_range_m) &&
         observation.measured_range_m > 0.0 &&
         observation.measured_range_m <=
             observation.effective_max_range_m - config.reliable_range_margin_m;
}

[[nodiscard]] bool
attachedKnownStaticRelation(const KnownStaticEndpointRelation relation) noexcept {
  return relation == KnownStaticEndpointRelation::kInsideSolid ||
         relation == KnownStaticEndpointRelation::kNearSurface ||
         relation == KnownStaticEndpointRelation::kInsideOpeningBoundary;
}

[[nodiscard]] std::optional<UncertainCandidate>
knownStaticCandidate(const LidarIngestionDecision& decision) noexcept {
  if (!decision.known_static_result_available ||
      decision.reason != LidarIngestionReason::kAmbiguousKnownStatic ||
      decision.known_static_result.classification !=
          KnownStaticLidarHitClassification::kAmbiguous) {
    return std::nullopt;
  }
  const KnownStaticLidarHitResult& result = decision.known_static_result;
  return UncertainCandidate{
      .kind = UncertainLidarHitKind::kKnownStaticBoundary,
      .evidence = attachedKnownStaticRelation(result.endpoint_relation)
                      ? UncertainLidarHitEvidence::kExpectedSurfaceAttached
                      : UncertainLidarHitEvidence::kDetachedObstacle,
      .association_id = result.structure_id,
      .part_id = result.part_id,
      .endpoint_surface_distance_m = result.endpoint_solid_distance_m,
      .distance_before_surface_m = result.distance_before_solid_m,
      .range_residual_m = result.range_delta_m,
  };
}

[[nodiscard]] bool
groundRelatedDecision(const LidarIngestionDecision& decision) noexcept {
  return decision.reason == LidarIngestionReason::kAmbiguousGround ||
         decision.reason == LidarIngestionReason::kClassificationUnavailable ||
         decision.reason == LidarIngestionReason::kNoExpectedSurface ||
         (decision.reason == LidarIngestionReason::kObstacleBeforeExpectedSurface &&
          (decision.expected_surface == LidarExpectedSurfaceKind::kGround ||
           decision.expected_surface == LidarExpectedSurfaceKind::kTied));
}

[[nodiscard]] std::optional<UncertainCandidate>
groundCandidate(const LidarBeamObservation& observation,
                const LidarIngestionDecision& decision,
                const GroundLidarRejectionConfig* const config) noexcept {
  if (!observation.projection.hit || !validGroundConfidenceConfig(config) ||
      !groundRelatedDecision(decision)) {
    return std::nullopt;
  }
  const double endpoint_distance_m =
      std::abs(observation.projection.endpoint_map_m.z - config->ground_altitude_m);
  const bool intrinsically_ambiguous =
      decision.reason == LidarIngestionReason::kAmbiguousGround;
  if (!intrinsically_ambiguous &&
      endpoint_distance_m > config->candidate_endpoint_altitude_tolerance_m) {
    return std::nullopt;
  }
  const double range_residual_m =
      decision.expected_ground_range_m.has_value()
          ? observation.measured_range_m - *decision.expected_ground_range_m
          : std::numeric_limits<double>::quiet_NaN();
  const bool range_matches_ground =
      !decision.expected_ground_range_m.has_value() ||
      (std::isfinite(range_residual_m) &&
       range_residual_m >= -config->closer_range_tolerance_m &&
       range_residual_m <= config->farther_range_tolerance_m);
  return UncertainCandidate{
      .kind = UncertainLidarHitKind::kGroundCandidate,
      .evidence =
          endpoint_distance_m <= config->attached_endpoint_altitude_tolerance_m &&
                  range_matches_ground
              ? UncertainLidarHitEvidence::kExpectedSurfaceAttached
              : UncertainLidarHitEvidence::kDetachedObstacle,
      .association_id = "ground",
      .part_id = "ground_plane",
      .endpoint_surface_distance_m = endpoint_distance_m,
      .distance_before_surface_m =
          decision.expected_ground_range_m.has_value()
              ? *decision.expected_ground_range_m - observation.measured_range_m
              : endpoint_distance_m,
      .range_residual_m = range_residual_m,
  };
}

[[nodiscard]] std::optional<UncertainCandidate>
projectionCandidate(const LidarBeamObservation& observation,
                    const LidarIngestionDecision& decision,
                    const LidarIngestionConfidenceConfig& config) noexcept {
  if (!observation.projection.hit ||
      decision.action != LidarIngestionAction::kIntegrateFreeAndHit ||
      (decision.reason != LidarIngestionReason::kNoExpectedSurface &&
       decision.reason != LidarIngestionReason::kClassificationUnavailable)) {
    return std::nullopt;
  }
  const bool uncertain_alignment =
      config.require_source_timestamp_alignment_for_unknown &&
      !sourceTimestampAligned(observation);
  if (!uncertain_alignment && reliableMeasuredRange(observation, config)) {
    return std::nullopt;
  }
  return UncertainCandidate{
      .kind = UncertainLidarHitKind::kProjectionUncertainUnknown,
      .evidence = UncertainLidarHitEvidence::kDetachedObstacle,
      .association_id = "projection",
      .part_id = "unknown_obstacle",
      .endpoint_surface_distance_m = std::numeric_limits<double>::quiet_NaN(),
      .distance_before_surface_m = std::numeric_limits<double>::quiet_NaN(),
      .range_residual_m = std::numeric_limits<double>::quiet_NaN(),
  };
}

[[nodiscard]] std::optional<UncertainCandidate>
selectCandidate(const LidarBeamObservation& observation,
                const LidarIngestionDecision& decision,
                const GroundLidarRejectionConfig* const ground_config,
                const LidarIngestionConfidenceConfig& confidence_config) noexcept {
  if (decision.reason == LidarIngestionReason::kObstacleInsideOpening) {
    return std::nullopt;
  }
  if (const auto known_static = knownStaticCandidate(decision);
      known_static.has_value()) {
    return known_static;
  }
  if (const auto ground = groundCandidate(observation, decision, ground_config);
      ground.has_value()) {
    return ground;
  }
  return projectionCandidate(observation, decision, confidence_config);
}

void applyPendingCandidate(LidarIngestionDecision& decision,
                           const UncertainCandidate& candidate) noexcept {
  decision.action = LidarIngestionAction::kSuppressAllUpdates;
  decision.uncertain_kind = candidate.kind;
  decision.uncertain_evidence = candidate.evidence;
  if (candidate.kind == UncertainLidarHitKind::kGroundCandidate) {
    decision.reason = LidarIngestionReason::kAmbiguousGround;
    decision.expected_surface = LidarExpectedSurfaceKind::kGround;
  } else if (candidate.kind == UncertainLidarHitKind::kProjectionUncertainUnknown) {
    decision.reason = LidarIngestionReason::kProjectionUncertainUnknown;
    decision.expected_surface = LidarExpectedSurfaceKind::kNone;
    decision.expected_range_m = std::numeric_limits<double>::quiet_NaN();
    decision.range_delta_m = std::numeric_limits<double>::quiet_NaN();
  }
}

void applyConfirmedExpectedSurface(LidarIngestionDecision& decision) noexcept {
  decision.action = LidarIngestionAction::kSuppressAllUpdates;
  if (decision.uncertain_kind == UncertainLidarHitKind::kKnownStaticBoundary) {
    decision.known_static_result.classification =
        KnownStaticLidarHitClassification::kExpectedStatic;
    decision.reason = LidarIngestionReason::kExpectedKnownStatic;
  } else if (decision.uncertain_kind == UncertainLidarHitKind::kGroundCandidate) {
    decision.reason = LidarIngestionReason::kExpectedGround;
    decision.expected_surface = LidarExpectedSurfaceKind::kGround;
  }
}

void applyConfirmedObstacle(LidarIngestionDecision& decision) noexcept {
  decision.action = LidarIngestionAction::kIntegrateFreeAndHit;
  if (decision.uncertain_kind == UncertainLidarHitKind::kKnownStaticBoundary) {
    decision.known_static_result.classification =
        KnownStaticLidarHitClassification::kUnexpected;
    decision.reason = LidarIngestionReason::kObstacleBeforeExpectedSurface;
    return;
  }
  if (decision.uncertain_kind == UncertainLidarHitKind::kGroundCandidate &&
      std::isfinite(decision.expected_range_m)) {
    decision.reason = LidarIngestionReason::kObstacleBeforeExpectedSurface;
    decision.expected_surface = LidarExpectedSurfaceKind::kGround;
    return;
  }
  decision.reason = LidarIngestionReason::kNoExpectedSurface;
  decision.expected_surface = LidarExpectedSurfaceKind::kNone;
  decision.expected_range_m = std::numeric_limits<double>::quiet_NaN();
  decision.range_delta_m = std::numeric_limits<double>::quiet_NaN();
}

} // namespace

LidarIngestionDecision
resolveUncertainLidarIngestion(const LidarBeamObservation& observation,
                               LidarIngestionDecision decision,
                               const GroundLidarRejectionConfig* const ground_config,
                               const LidarIngestionConfidenceConfig& confidence_config,
                               UncertainLidarHitTracker* const tracker) {
  const std::int64_t scan_stamp_ns = observationScanStamp(observation);
  if (tracker != nullptr) {
    decision.ambiguous_expired_candidates = tracker->expire(scan_stamp_ns);
  }
  if (!confidence_config.enabled || !validConfidenceConfig(confidence_config)) {
    return decision;
  }
  const std::optional<UncertainCandidate> candidate =
      selectCandidate(observation, decision, ground_config, confidence_config);
  if (!candidate.has_value()) {
    return decision;
  }
  applyPendingCandidate(decision, *candidate);
  if (tracker == nullptr) {
    return decision;
  }
  const UncertainLidarHitConfirmation confirmation =
      tracker->observe(UncertainLidarHitObservation{
          .kind = candidate->kind,
          .evidence = candidate->evidence,
          .association_id = candidate->association_id,
          .part_id = candidate->part_id,
          .endpoint_map_m = observation.projection.endpoint_map_m,
          .ray_origin_map_m = observation.projection.ray_origin_map_m,
          .ray_direction_map = observation.projection.ray_direction_map,
          .endpoint_surface_distance_m = candidate->endpoint_surface_distance_m,
          .distance_before_surface_m = candidate->distance_before_surface_m,
          .range_residual_m = candidate->range_residual_m,
          .scan_stamp_ns = scan_stamp_ns,
      });
  decision.ambiguous_resolution = confirmation.resolution;
  decision.ambiguous_evidence_count = confirmation.independent_scans;
  decision.ambiguous_expired_candidates += confirmation.expired_candidates;
  decision.ambiguous_viewpoint_translation_m = confirmation.viewpoint_translation_m;
  decision.ambiguous_viewpoint_direction_change_rad =
      confirmation.viewpoint_direction_change_rad;
  decision.ambiguous_opening_boundary_evidence =
      candidate->kind == UncertainLidarHitKind::kKnownStaticBoundary &&
      decision.known_static_result.endpoint_relation ==
          KnownStaticEndpointRelation::kInsideOpeningBoundary;
  switch (confirmation.resolution) {
    case UncertainLidarHitResolution::kPending:
      return decision;
    case UncertainLidarHitResolution::kConfirmedExpectedSurface:
      applyConfirmedExpectedSurface(decision);
      return decision;
    case UncertainLidarHitResolution::kConfirmedObstacle:
      applyConfirmedObstacle(decision);
      return decision;
  }
  return decision;
}

} // namespace drone_city_nav
