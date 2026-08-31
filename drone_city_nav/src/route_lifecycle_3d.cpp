#include "drone_city_nav/route_lifecycle_3d.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>

namespace drone_city_nav {
namespace {

constexpr double kMaximumStationCreditPerTravel{1.0};
constexpr double kStationCreditToleranceM{1.0e-6};

[[nodiscard]] bool sameProducerLineage(const NavigationWorldCertificate3D& first,
                                       const NavigationWorldCertificate3D& second) {
  return first.producer_instance_id == second.producer_instance_id;
}

[[nodiscard]] bool
validatesPlannedWorld(const NavigationWorldCertificate3D& planned,
                      const NavigationWorldCertificate3D& validated) {
  const bool local_generation_valid =
      (planned.local_world_generation == 0U &&
       validated.local_world_generation == 0U) ||
      (planned.local_world_generation != 0U &&
       validated.local_world_generation >= planned.local_world_generation);
  return planned.valid() && validated.valid() &&
         sameProducerLineage(planned, validated) && local_generation_valid &&
         validated.esdf_source_raw_revision >= planned.esdf_source_raw_revision &&
         validated.raw_validated_through_revision >=
             validated.esdf_source_raw_revision &&
         validated.raw_validated_through_revision >=
             planned.raw_validated_through_revision;
}

[[nodiscard]] std::size_t
firstRouteSampleAfter(const std::span<const RouteSample3D> route,
                      const double station_m) {
  return static_cast<std::size_t>(std::distance(
      route.begin(),
      std::upper_bound(route.begin(), route.end(), station_m,
                       [](const double station, const RouteSample3D& sample) {
                         return station < sample.station_m;
                       })));
}

[[nodiscard]] std::size_t
routeSegmentAtStation(const std::span<const RouteSample3D> route,
                      const double station_m) {
  if (route.size() < 2U) {
    return 0U;
  }
  const std::size_t next = firstRouteSampleAfter(route, station_m);
  return std::min(next == 0U ? 0U : next - 1U, route.size() - 2U);
}

[[nodiscard]] RawRouteSuffixStatus3D
collisionStatus(const OccupiedCollisionStatus3D status) noexcept {
  switch (status) {
    case OccupiedCollisionStatus3D::kClear:
      return RawRouteSuffixStatus3D::kValid;
    case OccupiedCollisionStatus3D::kOutsideFlightEnvelope:
      return RawRouteSuffixStatus3D::kOutsideFlightEnvelope;
    case OccupiedCollisionStatus3D::kRawCollision:
      return RawRouteSuffixStatus3D::kRawCollision;
    case OccupiedCollisionStatus3D::kInvalidInput:
      return RawRouteSuffixStatus3D::kInvalidCollisionWorld;
  }
  return RawRouteSuffixStatus3D::kInvalidCollisionWorld;
}

[[nodiscard]] bool
validateCollisionSegment(const OccupiedCollisionOracle3D& oracle, const Point3& first,
                         const Point3& second, const std::size_t failure_route_segment,
                         RawRouteSuffixValidation3D& result) noexcept {
  const OccupiedCollisionResult3D validation =
      oracle.validateSegment(first, FootprintBodyAxis{}, second, FootprintBodyAxis{});
  if (validation.clear()) {
    return true;
  }
  result.status = collisionStatus(validation.status);
  result.failure_route_segment = failure_route_segment;
  result.failure_point = validation.failure_point;
  return false;
}

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool samePoint(const Point3& first, const Point3& second,
                             const double tolerance_m) noexcept {
  return distance3D(first, second) <= tolerance_m;
}

[[nodiscard]] RawRouteSuffixValidation3D validateRawRouteConnector3D(
    const std::span<const RouteSample3D> route, const Point3& position,
    const RouteProjection3D& projection,
    const OccupiedCollisionOracle3D& collision_oracle) noexcept {
  if (route.size() < 2U) {
    return {.status = RawRouteSuffixStatus3D::kInvalidRoute};
  }
  if (!projection.valid || !std::isfinite(projection.station_m)) {
    return {.status = RawRouteSuffixStatus3D::kInvalidProjection};
  }

  const std::size_t first_route_segment =
      routeSegmentAtStation(route, projection.station_m);
  RawRouteSuffixValidation3D result{
      .status = RawRouteSuffixStatus3D::kValid,
      .first_validated_route_segment = first_route_segment,
      .validated_from_station_m = projection.station_m,
      .connector_validated = true,
  };
  if (!validateCollisionSegment(collision_oracle, position, projection.point,
                                first_route_segment, result)) {
    return result;
  }
  return result;
}

} // namespace

bool NavigationWorldCertificate3D::valid() const noexcept {
  return esdf_fingerprint != 0U &&
         raw_validated_through_revision >= esdf_source_raw_revision;
}

bool ActiveIntent3D::valid() const noexcept {
  return route_intent_id != 0U && mission_epoch != 0U && finitePoint(mission_target);
}

bool RouteOwnerIdentity3D::valid() const noexcept {
  return id != 0U && active_intent.valid();
}

std::optional<ActiveIntent3D>
activeIntent3D(const MaterializedRouteProposal3D& proposal) noexcept {
  if (!proposal.objective.available || proposal.objective.mission_epoch == 0U ||
      !proposal.intent.valid || proposal.intent.id == 0U ||
      !finitePoint(proposal.intent.mission_target) ||
      (!proposal.objective.continuous_tracking &&
       !samePoint(proposal.objective.goal, proposal.intent.mission_target, 1.0e-6))) {
    return std::nullopt;
  }
  ActiveIntent3D intent{
      .route_intent_id = proposal.intent.id,
      .mission_epoch = proposal.objective.mission_epoch,
      .assignment_generation = proposal.objective.assignment_generation,
      .target_detection_id = proposal.objective.target_detection_id,
      .target_track_id = proposal.objective.target_track_id,
      .mission_target = proposal.intent.mission_target,
      .continuous_tracking = proposal.objective.continuous_tracking,
  };
  return intent.valid() ? std::optional<ActiveIntent3D>{intent} : std::nullopt;
}

bool sameActiveIntent3D(const ActiveIntent3D& first, const ActiveIntent3D& second,
                        const double target_tolerance_m) noexcept {
  if (!first.valid() || !second.valid() || !std::isfinite(target_tolerance_m) ||
      target_tolerance_m < 0.0 || first.mission_epoch != second.mission_epoch ||
      first.assignment_generation != second.assignment_generation ||
      first.target_detection_id != second.target_detection_id ||
      first.target_track_id != second.target_track_id ||
      first.continuous_tracking != second.continuous_tracking) {
    return false;
  }
  if (first.continuous_tracking) {
    return true;
  }
  return first.route_intent_id == second.route_intent_id &&
         samePoint(first.mission_target, second.mission_target, target_tolerance_m);
}

bool RawRouteSuffixValidation3D::accepted() const noexcept {
  return status == RawRouteSuffixStatus3D::kValid;
}

bool RouteExecutionAssessment3D::usable() const noexcept {
  return status == RouteExecutionStatus3D::kUsable;
}

bool RouteExecutionAssessment3D::replacementRequired() const noexcept {
  switch (status) {
    case RouteExecutionStatus3D::kUsable:
    case RouteExecutionStatus3D::kNoActiveRoute:
      return false;
    case RouteExecutionStatus3D::kWorldLineageMismatch:
    case RouteExecutionStatus3D::kObjectiveMismatch:
    case RouteExecutionStatus3D::kInvalidRoute:
    case RouteExecutionStatus3D::kInvalidProjection:
    case RouteExecutionStatus3D::kExcessiveCrossTrack:
    case RouteExecutionStatus3D::kTrackingTubeViolation:
    case RouteExecutionStatus3D::kRawCollision:
      return true;
  }
  return true;
}

bool RouteProposalReplacementAssessment3D::replacementAllowed() const noexcept {
  return status == RouteProposalReplacementStatus3D::kReplace;
}

bool RoutePublicationAssessment3D::compatible() const noexcept {
  return status == RoutePublicationStatus3D::kCompatible;
}

bool RouteActivationAssessment3D::accepted() const noexcept {
  return publication.compatible() && objective_matches && projection.valid &&
         cross_track_accepted && raw_world_compatible && raw_validation.accepted();
}

std::optional<ActivatedRouteIdentity3D>
activateRouteProposal3D(const MaterializedRouteProposal3D& proposal,
                        const std::uint64_t generation) noexcept {
  if (generation == 0U || !proposal.activation_eligible ||
      proposal.route_sample_count < 2U || proposal.route_fingerprint == 0U ||
      !proposal.objective.available || !proposal.intent.valid ||
      !proposal.evidence.physical_executable ||
      !validatesPlannedWorld(proposal.planned_world, proposal.validated_world)) {
    return std::nullopt;
  }
  return ActivatedRouteIdentity3D{.generation = generation, .proposal = proposal};
}

RouteProposalReplacementAssessment3D assessRouteProposalReplacement3D(
    const ActivatedRouteIdentity3D* const active_route,
    const MaterializedRouteProposal3D& candidate,
    const RouteProposalReplacementObservation3D& observation) noexcept {
  if (active_route == nullptr) {
    return {};
  }
  if (!std::isfinite(observation.mission_target_tolerance_m) ||
      observation.mission_target_tolerance_m < 0.0) {
    return {.status = RouteProposalReplacementStatus3D::kRejectIntentConflict};
  }
  const std::optional<ActiveIntent3D> active = activeIntent3D(active_route->proposal);
  const std::optional<ActiveIntent3D> replacement = activeIntent3D(candidate);
  if (!active.has_value() || !replacement.has_value()) {
    return {.status = RouteProposalReplacementStatus3D::kRejectIntentConflict};
  }
  if (sameActiveIntent3D(*active, *replacement,
                         observation.mission_target_tolerance_m)) {
    // A safety replan changes executable geometry, not mission ownership.
    // Route/world certification and the finite current-state handoff remain
    // mandatory before this replacement can be published.
    if (observation.continuity_preserving_successor ||
        observation.materially_improved_point_to_point_successor ||
        observation.safety_replan_requested) {
      return {};
    }
    return {.status = RouteProposalReplacementStatus3D::kRetainEquivalentActiveSegment};
  }
  const bool ownership_lineage_changed =
      active->mission_epoch != replacement->mission_epoch ||
      active->assignment_generation != replacement->assignment_generation ||
      active->target_detection_id != replacement->target_detection_id ||
      active->target_track_id != replacement->target_track_id ||
      active->continuous_tracking != replacement->continuous_tracking;
  if (ownership_lineage_changed) {
    return {};
  }
  return {.status = RouteProposalReplacementStatus3D::kRejectIntentConflict};
}

RoutePublicationAssessment3D
assessRoutePublication3D(const MaterializedRouteProposal3D& proposal,
                         const NavigationWorldCertificate3D& resident_world) noexcept {
  if (!validatesPlannedWorld(proposal.planned_world, proposal.validated_world)) {
    return {.status = RoutePublicationStatus3D::kInvalidProposalWorld};
  }
  if (!resident_world.valid()) {
    return {.status = RoutePublicationStatus3D::kInvalidResidentWorld};
  }
  if (!sameProducerLineage(proposal.validated_world, resident_world)) {
    return {.status = RoutePublicationStatus3D::kWorldLineageMismatch};
  }
  if (resident_world.esdf_source_raw_revision <
      proposal.planned_world.esdf_source_raw_revision) {
    return {.status = RoutePublicationStatus3D::kResidentWorldPredatesPlan};
  }
  if (proposal.planned_world.local_world_generation != 0U &&
      resident_world.local_world_generation <
          proposal.planned_world.local_world_generation) {
    return {.status = RoutePublicationStatus3D::kResidentWorldPredatesPlan};
  }
  return {.status = RoutePublicationStatus3D::kCompatible};
}

RouteActivationAssessment3D
assessRouteActivation3D(const MaterializedRouteProposal3D& proposal,
                        const std::span<const RouteSample3D> route,
                        const RouteActivationObservation3D& observation) noexcept {
  RouteActivationAssessment3D result{
      .publication = assessRoutePublication3D(proposal, observation.resident_world),
      .raw_validated_through_revision =
          proposal.validated_world.raw_validated_through_revision,
      .raw_world_compatible = !observation.raw_validation_required,
  };
  result.objective_matches =
      staticRouteObjectiveMatches(proposal.objective, observation.current_objective,
                                  observation.minimum_tracking_sample_sequence,
                                  std::numeric_limits<double>::infinity());
  if (observation.raw_validation_required) {
    result.raw_world_compatible =
        observation.latest_raw_occupancy != nullptr &&
        observation.latest_raw_producer_instance_id ==
            observation.resident_world.producer_instance_id &&
        observation.latest_raw_revision >=
            observation.resident_world.esdf_source_raw_revision &&
        observation.latest_raw_revision >=
            proposal.validated_world.raw_validated_through_revision;
  }
  if (!result.publication.compatible() || !result.objective_matches ||
      route.size() < 2U || !std::isfinite(observation.maximum_cross_track_m) ||
      observation.maximum_cross_track_m <= 0.0) {
    return result;
  }

  const double maximum_station_m =
      std::min(route.back().station_m,
               route.front().station_m +
                   kMaximumStationCreditPerTravel *
                       distance3D(route.front().position, observation.position) +
                   kStationCreditToleranceM);
  result.projection = projectOntoRoute3DWithinStationWindow(
      route, observation.position, route.front().station_m, maximum_station_m);
  result.cross_track_accepted =
      result.projection.valid &&
      result.projection.distance_m <= observation.maximum_cross_track_m;
  if (!result.cross_track_accepted) {
    return result;
  }

  if (!observation.raw_validation_required) {
    result.raw_validation = RawRouteSuffixValidation3D{
        .status = RawRouteSuffixStatus3D::kValid,
    };
    return result;
  }
  if (!result.raw_world_compatible) {
    return result;
  }

  result.raw_validation = validateRawRouteSuffix3D(
      route, observation.position, result.projection,
      OccupiedCollisionWorld3D{
          .observed_occupancy = observation.latest_raw_occupancy,
          .static_occupancy = nullptr,
          .planar_occupancy = nullptr,
          .raw_point_cloud = {},
          .launch_support_contact = observation.launch_support_contact,
          .footprint = observation.footprint,
          .flight_envelope = observation.flight_envelope,
      });
  if (result.raw_validation.accepted() && result.raw_validation.connector_validated &&
      result.raw_validation.suffix_validated) {
    result.raw_validated_through_revision = std::max(
        result.raw_validated_through_revision, observation.latest_raw_revision);
  }
  return result;
}

RawRouteSuffixValidation3D
validateRawRouteSuffix3D(const std::span<const RouteSample3D> route,
                         const Point3& position, const RouteProjection3D& projection,
                         const OccupiedCollisionWorld3D& collision_world) noexcept {
  const OccupiedCollisionOracle3D collision_oracle{collision_world};
  RawRouteSuffixValidation3D result =
      validateRawRouteConnector3D(route, position, projection, collision_oracle);
  if (!result.accepted()) {
    return result;
  }
  const std::size_t first_route_sample =
      firstRouteSampleAfter(route, projection.station_m);
  Point3 previous = projection.point;
  for (std::size_t index = first_route_sample; index < route.size(); ++index) {
    if (!validateCollisionSegment(collision_oracle, previous, route[index].position,
                                  index == 0U ? 0U : index - 1U, result)) {
      return result;
    }
    previous = route[index].position;
  }
  result.suffix_validated = true;
  return result;
}

RouteExecutionAssessment3D
assessRouteExecution3D(const ActivatedRouteIdentity3D* const active_route,
                       const std::span<const RouteSample3D> route,
                       const RouteExecutionObservation3D& observation) noexcept {
  RouteExecutionAssessment3D result;
  if (active_route == nullptr) {
    return result;
  }
  result.validated_through_raw_revision =
      std::max(active_route->proposal.validated_world.raw_validated_through_revision,
               observation.previously_validated_through_raw_revision);
  if (!staticRouteObjectiveMatches(active_route->proposal.objective,
                                   observation.current_objective,
                                   observation.minimum_tracking_sample_sequence,
                                   std::numeric_limits<double>::infinity())) {
    result.status = RouteExecutionStatus3D::kObjectiveMismatch;
    return result;
  }
  if (route.size() < 2U) {
    result.status = RouteExecutionStatus3D::kInvalidRoute;
    return result;
  }
  if (!std::isfinite(observation.minimum_station_m) ||
      std::isnan(observation.maximum_station_m) ||
      observation.maximum_station_m < observation.minimum_station_m ||
      !std::isfinite(observation.maximum_cross_track_m) ||
      observation.maximum_cross_track_m <= 0.0) {
    result.status = RouteExecutionStatus3D::kInvalidProjection;
    return result;
  }

  const double maximum_station_m =
      std::min(observation.maximum_station_m, route.back().station_m);
  result.projection = projectOntoRoute3DWithinStationWindow(
      route, observation.position, observation.minimum_station_m, maximum_station_m);
  if (!result.projection.valid) {
    result.status = RouteExecutionStatus3D::kInvalidProjection;
    return result;
  }
  if (result.projection.distance_m > observation.maximum_cross_track_m) {
    result.status = RouteExecutionStatus3D::kExcessiveCrossTrack;
    return result;
  }
  if (observation.latest_raw_occupancy != nullptr &&
      observation.latest_raw_producer_instance_id !=
          active_route->proposal.validated_world.producer_instance_id) {
    result.status = RouteExecutionStatus3D::kWorldLineageMismatch;
    return result;
  }
  if (observation.latest_raw_occupancy != nullptr &&
      observation.latest_raw_revision >= result.validated_through_raw_revision) {
    const bool suffix_validation_required =
        observation.latest_raw_revision > result.validated_through_raw_revision;
    RouteProjection3D validation_projection = result.projection;
    if (std::isfinite(observation.minimum_station_m) &&
        observation.minimum_station_m > validation_projection.station_m) {
      validation_projection.station_m =
          std::min(observation.minimum_station_m, route.back().station_m);
      validation_projection.point =
          sampleRoute3DAtStation(route, validation_projection.station_m).position;
      validation_projection.distance_m =
          distance3D(observation.position, validation_projection.point);
      validation_projection.remaining_m =
          std::max(0.0, route.back().station_m - validation_projection.station_m);
    }
    const OccupiedCollisionWorld3D collision_world{
        .observed_occupancy = observation.latest_raw_occupancy,
        .static_occupancy = nullptr,
        .planar_occupancy = nullptr,
        .raw_point_cloud = {},
        .launch_support_contact = observation.launch_support_contact,
        .footprint = observation.footprint,
        .flight_envelope = observation.flight_envelope,
    };
    const OccupiedCollisionOracle3D collision_oracle{collision_world};
    result.raw_validation =
        suffix_validation_required
            ? validateRawRouteSuffix3D(route, observation.position,
                                       validation_projection, collision_world)
            : validateRawRouteConnector3D(route, observation.position,
                                          validation_projection, collision_oracle);
    if (!result.raw_validation.accepted()) {
      result.status =
          result.raw_validation.status == RawRouteSuffixStatus3D::kRawCollision
              ? RouteExecutionStatus3D::kRawCollision
              : RouteExecutionStatus3D::kInvalidRoute;
      return result;
    }
    if (result.raw_validation.suffix_validated) {
      result.validated_through_raw_revision = std::max(
          result.validated_through_raw_revision, observation.latest_raw_revision);
    }
  } else {
    result.raw_validation =
        RawRouteSuffixValidation3D{.status = RawRouteSuffixStatus3D::kValid};
  }
  result.status = RouteExecutionStatus3D::kUsable;
  return result;
}

RouteSegmentCompletionAssessment3D
assessRouteSegmentCompletion3D(const std::span<const RouteSample3D> route,
                               const std::uint64_t expected_generation,
                               const RouteSegmentCompletionObservation3D& observation,
                               const RouteSegmentCompletionConfig3D& config) noexcept {
  RouteSegmentCompletionAssessment3D result;
  result.generation_matches =
      expected_generation != 0U && observation.route_generation == expected_generation;
  if (!result.generation_matches || route.size() < 2U ||
      !std::isfinite(observation.minimum_station_m) ||
      !std::isfinite(config.capture_radius_m) || config.capture_radius_m < 0.0 ||
      !std::isfinite(config.terminal_station_tolerance_m) ||
      config.terminal_station_tolerance_m < 0.0 ||
      !std::isfinite(route.back().station_m)) {
    return result;
  }

  result.projection =
      projectOntoRoute3D(route, observation.position, observation.minimum_station_m);
  result.endpoint_distance_m = distance3D(observation.position, route.back().position);
  result.monotonic_station_m =
      result.projection.valid
          ? std::max(observation.minimum_station_m, result.projection.station_m)
          : observation.minimum_station_m;
  result.terminal_station_reached =
      result.projection.valid && std::isfinite(result.monotonic_station_m) &&
      result.monotonic_station_m + config.terminal_station_tolerance_m >=
          route.back().station_m;
  result.captured = result.terminal_station_reached &&
                    std::isfinite(result.endpoint_distance_m) &&
                    result.endpoint_distance_m <= config.capture_radius_m;
  return result;
}

std::string_view
routeLifecycleEventKind3DName(const RouteLifecycleEventKind3D kind) noexcept {
  switch (kind) {
    case RouteLifecycleEventKind3D::kCompleted:
      return "completed";
    case RouteLifecycleEventKind3D::kRawInvalidated:
      return "raw_invalidated";
    case RouteLifecycleEventKind3D::kLatestLidarInvalidated:
      return "latest_lidar_invalidated";
    case RouteLifecycleEventKind3D::kObjectiveSuperseded:
      return "objective_superseded";
    case RouteLifecycleEventKind3D::kControlCandidateRejected:
      return "control_candidate_rejected";
    case RouteLifecycleEventKind3D::kCrossTrackExceeded:
      return "cross_track_exceeded";
    case RouteLifecycleEventKind3D::kTrackingTubeExceeded:
      return "tracking_tube_exceeded";
  }
  return "invalid_event";
}

std::string_view
routePublicationStatus3DName(const RoutePublicationStatus3D status) noexcept {
  switch (status) {
    case RoutePublicationStatus3D::kNotAssessed:
      return "not_assessed";
    case RoutePublicationStatus3D::kCompatible:
      return "compatible";
    case RoutePublicationStatus3D::kInvalidProposalWorld:
      return "invalid_proposal_world";
    case RoutePublicationStatus3D::kInvalidResidentWorld:
      return "invalid_resident_world";
    case RoutePublicationStatus3D::kWorldLineageMismatch:
      return "world_lineage_mismatch";
    case RoutePublicationStatus3D::kResidentWorldPredatesPlan:
      return "resident_world_predates_plan";
  }
  return "invalid_status";
}

std::string_view
rawRouteSuffixStatus3DName(const RawRouteSuffixStatus3D status) noexcept {
  switch (status) {
    case RawRouteSuffixStatus3D::kValid:
      return "valid";
    case RawRouteSuffixStatus3D::kInvalidRoute:
      return "invalid_route";
    case RawRouteSuffixStatus3D::kInvalidProjection:
      return "invalid_projection";
    case RawRouteSuffixStatus3D::kOutsideFlightEnvelope:
      return "outside_flight_envelope";
    case RawRouteSuffixStatus3D::kRawCollision:
      return "raw_collision";
    case RawRouteSuffixStatus3D::kInvalidCollisionWorld:
      return "invalid_collision_world";
  }
  return "invalid_status";
}

std::string_view
routeExecutionStatus3DName(const RouteExecutionStatus3D status) noexcept {
  switch (status) {
    case RouteExecutionStatus3D::kUsable:
      return "usable";
    case RouteExecutionStatus3D::kNoActiveRoute:
      return "no_active_route";
    case RouteExecutionStatus3D::kWorldLineageMismatch:
      return "world_lineage_mismatch";
    case RouteExecutionStatus3D::kObjectiveMismatch:
      return "objective_mismatch";
    case RouteExecutionStatus3D::kInvalidRoute:
      return "invalid_route";
    case RouteExecutionStatus3D::kInvalidProjection:
      return "invalid_projection";
    case RouteExecutionStatus3D::kExcessiveCrossTrack:
      return "excessive_cross_track";
    case RouteExecutionStatus3D::kTrackingTubeViolation:
      return "tracking_tube_violation";
    case RouteExecutionStatus3D::kRawCollision:
      return "raw_collision";
  }
  return "invalid_status";
}

} // namespace drone_city_nav
