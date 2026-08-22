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
  const SweptFootprintResult validation = validateRawSweptFootprint(
      occupancy, first, FootprintBodyAxis{}, second, FootprintBodyAxis{}, footprint,
      proprioceptive_free_space_seed, launch_support_contact);
  if (validation.status != SweptFootprintStatus::kRawCollision) {
    return false;
  }
  failure_point = validation.failure_point;
  return true;
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
