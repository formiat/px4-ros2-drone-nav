#include "drone_city_nav/route_lifecycle_3d.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool sameProducerLineage(const NavigationWorldCertificate3D& first,
                                       const NavigationWorldCertificate3D& second) {
  return first.producer_instance_id == second.producer_instance_id;
}

[[nodiscard]] bool
validatesPlannedWorld(const NavigationWorldCertificate3D& planned,
                      const NavigationWorldCertificate3D& validated) {
  return planned.valid() && validated.valid() &&
         sameProducerLineage(planned, validated) &&
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

[[nodiscard]] bool
rawCollision(const ObservedOccupancyGrid3D& occupancy, const Point3& first,
             const Point3& second, const SweptFootprintConfig& footprint,
             const ProprioceptiveFreeSpaceSeed3D* const proprioceptive_free_space_seed,
             const LaunchSupportContact3D* const launch_support_contact,
             Point3& failure_point) {
  const SweptFootprintResult validation = validateObservedSweptFootprint(
      occupancy, first, FootprintBodyAxis{}, second, FootprintBodyAxis{}, footprint,
      ObservedSpaceValidationPolicy::kAllowUnknown, proprioceptive_free_space_seed,
      launch_support_contact);
  if (validation.status != SweptFootprintStatus::kRawCollision) {
    return false;
  }
  failure_point = validation.failure_point;
  return true;
}

[[nodiscard]] bool isStrategicMissionIntent(const RouteIntent3D& intent) noexcept {
  return intent.valid && intent.id != 0U &&
         intent.source == RouteIntentSource3D::kTopology &&
         intent.purpose == RouteIntentPurpose3D::kMissionTransit &&
         intent.strategic_continuation_available &&
         intent.intent_reaches_mission_target;
}

[[nodiscard]] bool samePoint(const Point3& first, const Point3& second,
                             const double tolerance_m) noexcept {
  return distance3D(first, second) <= tolerance_m;
}

} // namespace

bool NavigationWorldCertificate3D::valid() const noexcept {
  return esdf_fingerprint != 0U &&
         raw_validated_through_revision >= esdf_source_raw_revision;
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
  if (active_route == nullptr || observation.safety_replan_requested ||
      !std::isfinite(observation.segment_target_tolerance_m) ||
      observation.segment_target_tolerance_m < 0.0) {
    return {};
  }
  const RouteIntent3D& active = active_route->proposal.intent;
  const RouteIntent3D& replacement = candidate.intent;
  if (!isStrategicMissionIntent(active) || !isStrategicMissionIntent(replacement) ||
      active.id != replacement.id ||
      active.target_identity != replacement.target_identity ||
      !samePoint(active.mission_target, replacement.mission_target,
                 observation.segment_target_tolerance_m) ||
      !samePoint(active.intent_target, replacement.intent_target,
                 observation.segment_target_tolerance_m) ||
      !samePoint(active.segment_target, replacement.segment_target,
                 observation.segment_target_tolerance_m)) {
    return {};
  }
  return {.status = RouteProposalReplacementStatus3D::kRetainEquivalentActiveSegment};
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
  return {.status = RoutePublicationStatus3D::kCompatible};
}

RawRouteSuffixValidation3D validateRawRouteSuffix3D(
    const std::span<const RouteSample3D> route, const Point3& position,
    const RouteProjection3D& projection, const ObservedOccupancyGrid3D& occupancy,
    const SweptFootprintConfig& footprint,
    const ProprioceptiveFreeSpaceSeed3D* const proprioceptive_free_space_seed,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  if (route.size() < 2U) {
    return {.status = RawRouteSuffixStatus3D::kInvalidRoute};
  }
  if (!projection.valid || !std::isfinite(projection.station_m)) {
    return {.status = RawRouteSuffixStatus3D::kInvalidProjection};
  }

  const std::size_t first_route_sample =
      firstRouteSampleAfter(route, projection.station_m);
  const std::size_t first_route_segment =
      routeSegmentAtStation(route, projection.station_m);
  RawRouteSuffixValidation3D result{
      .status = RawRouteSuffixStatus3D::kValid,
      .first_validated_route_segment = first_route_segment,
      .connector_validated = true,
  };
  if (rawCollision(occupancy, position, projection.point, footprint,
                   proprioceptive_free_space_seed, launch_support_contact,
                   result.failure_point)) {
    result.status = RawRouteSuffixStatus3D::kRawCollision;
    result.failure_route_segment = first_route_segment;
    return result;
  }

  Point3 previous = projection.point;
  for (std::size_t index = first_route_sample; index < route.size(); ++index) {
    if (rawCollision(occupancy, previous, route[index].position, footprint,
                     proprioceptive_free_space_seed, launch_support_contact,
                     result.failure_point)) {
      result.status = RawRouteSuffixStatus3D::kRawCollision;
      result.failure_route_segment = index == 0U ? 0U : index - 1U;
      return result;
    }
    previous = route[index].position;
  }
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

  result.projection =
      projectOntoRoute3D(route, observation.position, observation.minimum_station_m);
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
      observation.latest_raw_revision > result.validated_through_raw_revision) {
    result.raw_validation = validateRawRouteSuffix3D(
        route, observation.position, result.projection,
        *observation.latest_raw_occupancy, observation.footprint,
        observation.proprioceptive_free_space_seed, observation.launch_support_contact);
    if (!result.raw_validation.accepted()) {
      result.status =
          result.raw_validation.status == RawRouteSuffixStatus3D::kRawCollision
              ? RouteExecutionStatus3D::kRawCollision
              : RouteExecutionStatus3D::kInvalidRoute;
      return result;
    }
    result.validated_through_raw_revision = std::max(
        result.validated_through_raw_revision, observation.latest_raw_revision);
  } else {
    result.raw_validation =
        RawRouteSuffixValidation3D{.status = RawRouteSuffixStatus3D::kValid};
  }
  result.status = RouteExecutionStatus3D::kUsable;
  return result;
}

RouteSegmentCompletionAssessment3D
assessRouteSegmentCompletion3D(const std::span<const RouteSample3D> route,
                               const Point3& position,
                               const RouteSegmentCompletionConfig3D& config) noexcept {
  RouteSegmentCompletionAssessment3D result;
  if (route.size() < 2U || !std::isfinite(config.capture_radius_m) ||
      config.capture_radius_m < 0.0) {
    return result;
  }

  result.projection = projectOntoRoute3D(route, position);
  result.endpoint_distance_m = distance3D(position, route.back().position);
  result.captured = result.projection.valid &&
                    std::isfinite(result.endpoint_distance_m) &&
                    result.endpoint_distance_m <= config.capture_radius_m;
  return result;
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
    case RawRouteSuffixStatus3D::kRawCollision:
      return "raw_collision";
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
    case RouteExecutionStatus3D::kRawCollision:
      return "raw_collision";
  }
  return "invalid_status";
}

} // namespace drone_city_nav
