#include "drone_city_nav/route_planning_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  hash ^= value;
  hash *= kFnvPrime;
}

void hashPoint(std::uint64_t& hash, const Point3& point) noexcept {
  hashValue(hash, std::bit_cast<std::uint64_t>(point.x));
  hashValue(hash, std::bit_cast<std::uint64_t>(point.y));
  hashValue(hash, std::bit_cast<std::uint64_t>(point.z));
}

void mergeFootprintEvidence(SegmentEvidence3D& target,
                            const SweptFootprintResult& source) noexcept {
  target.raw_collision = target.raw_collision || source.evidence.raw_collision;
  target.outside_grid_exposure =
      target.outside_grid_exposure || source.evidence.outside_grid_exposure;
  target.unknown_exposure = target.unknown_exposure || source.evidence.unknown_exposure;
  target.invalid_esdf_exposure =
      target.invalid_esdf_exposure || source.evidence.invalid_esdf_exposure;
  if (source.evidence.known_clearance_observed) {
    target.known_clearance_observed = true;
    target.minimum_known_clearance_m = std::min(
        target.minimum_known_clearance_m, source.evidence.minimum_known_clearance_m);
  }
}

void mergeDerivedDistanceEvidence(SegmentEvidence3D& target,
                                  const SweptFootprintResult& source) noexcept {
  target.outside_grid_exposure =
      target.outside_grid_exposure || source.evidence.outside_grid_exposure;
  target.unknown_exposure = target.unknown_exposure || source.evidence.unknown_exposure;
  target.invalid_esdf_exposure =
      target.invalid_esdf_exposure || source.evidence.invalid_esdf_exposure;
  if (source.evidence.known_clearance_observed) {
    target.known_clearance_observed = true;
    target.minimum_known_clearance_m = std::min(
        target.minimum_known_clearance_m, source.evidence.minimum_known_clearance_m);
  }
}

[[nodiscard]] SegmentEvidenceStatus3D
rejectedStatus(const SweptFootprintResult& result, const bool require_known_free_space,
               const bool reject_invalid_esdf) noexcept {
  if (result.evidence.raw_collision ||
      result.status == SweptFootprintStatus::kRawCollision) {
    return SegmentEvidenceStatus3D::kRawCollision;
  }
  if (reject_invalid_esdf && (result.evidence.invalid_esdf_exposure ||
                              result.status == SweptFootprintStatus::kInvalidEsdf)) {
    return SegmentEvidenceStatus3D::kInvalidEsdf;
  }
  if (result.status == SweptFootprintStatus::kOutsideGrid) {
    return SegmentEvidenceStatus3D::kOutsideGrid;
  }
  if (require_known_free_space &&
      result.status == SweptFootprintStatus::kUnknownSpace) {
    return SegmentEvidenceStatus3D::kUnknownRejected;
  }
  return SegmentEvidenceStatus3D::kValid;
}

} // namespace

double routeNetCoordinateProgress3D(const Point3& start, const Point3& endpoint,
                                    const Point3& mission_target) noexcept {
  return distance3D(start, endpoint) + distance3D(start, mission_target) -
         distance3D(endpoint, mission_target);
}

std::uint64_t makeRouteIntentId3D(const Point3& mission_target,
                                  const std::uint64_t mission_epoch,
                                  const std::uint64_t assignment_generation,
                                  const std::uint64_t target_detection_id,
                                  const std::uint64_t target_track_id) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, mission_epoch);
  hashValue(hash, assignment_generation);
  hashValue(hash, target_detection_id);
  hashValue(hash, target_track_id);
  hashPoint(hash, mission_target);
  return hash == 0U ? 1U : hash;
}

SegmentEvidence3D evaluateSegmentEvidence3D(
    const RouteIntent3D& intent, const std::span<const RouteSample3D> route,
    const Point3& search_start, const bool planner_executable,
    const bool reaches_mission_target, const double objective_cost,
    const SegmentEvidenceWorld3D& world) noexcept {
  SegmentEvidence3D result{
      .planned_on_revision = intent.planned_on_revision,
      .validated_through_revision = world.validated_through_revision,
      .status = SegmentEvidenceStatus3D::kInvalidWorld,
      .objective_cost = objective_cost,
      .planner_executable = planner_executable,
      .reaches_mission_target = reaches_mission_target,
  };
  if (!planner_executable) {
    result.status = SegmentEvidenceStatus3D::kPlannerRejected;
    return result;
  }
  if (route.size() < 2U) {
    result.status = SegmentEvidenceStatus3D::kEmptyRoute;
    return result;
  }
  result.materialized = true;
  result.endpoint_displacement_m = distance3D(search_start, route.back().position);
  result.mission_progress_m = distance3D(search_start, intent.mission_target) -
                              distance3D(route.back().position, intent.mission_target);
  result.net_coordinate_progress_m = routeNetCoordinateProgress3D(
      search_start, route.back().position, intent.mission_target);
  for (std::size_t index = 1U; index < route.size(); ++index) {
    result.route_length_m +=
        distance3D(route[index - 1U].position, route[index].position);
  }
  const bool observed_raw_authority = world.latest_observed_occupancy != nullptr;
  const bool static_raw_authority = world.static_occupancy != nullptr;
  const bool derived_distance_available =
      world.grid != nullptr && !world.esdf_m.empty();
  if (!observed_raw_authority && !static_raw_authority && !derived_distance_available) {
    return result;
  }
  if (!std::ranges::all_of(route, [&](const RouteSample3D& sample) {
        return insideFlightEnvelope(sample.position, world.flight_envelope);
      })) {
    result.status = SegmentEvidenceStatus3D::kOutsideFlightEnvelope;
    return result;
  }

  for (std::size_t index = 1U; index < route.size(); ++index) {
    const Point3& first = route[index - 1U].position;
    const Point3& second = route[index].position;
    if (observed_raw_authority) {
      const ObservedSpaceValidationPolicy observed_policy =
          world.require_known_free_space
              ? ObservedSpaceValidationPolicy::kRequireKnownFree
              : ObservedSpaceValidationPolicy::kAllowUnknown;
      const SweptFootprintResult raw_validation = validateObservedSweptFootprint(
          *world.latest_observed_occupancy, first, FootprintBodyAxis{}, second,
          FootprintBodyAxis{}, world.footprint, observed_policy,
          world.proprioceptive_free_space_seed, world.launch_support_contact);
      mergeFootprintEvidence(result, raw_validation);
      const SegmentEvidenceStatus3D status =
          rejectedStatus(raw_validation, world.require_known_free_space, false);
      if (status != SegmentEvidenceStatus3D::kValid) {
        result.status = status;
        result.failure_segment_index = index - 1U;
        result.failure_point = raw_validation.failure_point;
        return result;
      }
    } else if (static_raw_authority) {
      const SweptFootprintResult raw_validation = validateKnownStaticSweptFootprint(
          *world.static_occupancy, first, FootprintBodyAxis{}, second,
          FootprintBodyAxis{}, world.footprint);
      mergeFootprintEvidence(result, raw_validation);
      const SegmentEvidenceStatus3D status =
          rejectedStatus(raw_validation, false, false);
      if (status != SegmentEvidenceStatus3D::kValid) {
        result.status = status;
        result.failure_segment_index = index - 1U;
        result.failure_point = raw_validation.failure_point;
        return result;
      }
    }
    if (derived_distance_available) {
      const SweptFootprintResult derived_validation = validateSweptFootprint(
          *world.grid, world.esdf_m, first, second, world.footprint);
      if (observed_raw_authority || static_raw_authority) {
        mergeDerivedDistanceEvidence(result, derived_validation);
      } else {
        mergeFootprintEvidence(result, derived_validation);
        const SegmentEvidenceStatus3D status =
            rejectedStatus(derived_validation, world.require_known_free_space,
                           world.reject_invalid_esdf);
        if (status != SegmentEvidenceStatus3D::kValid) {
          result.status = status;
          result.failure_segment_index = index - 1U;
          result.failure_point = derived_validation.failure_point;
          return result;
        }
      }
    }
  }
  result.status = SegmentEvidenceStatus3D::kValid;
  result.physical_executable = true;
  return result;
}

const char* segmentEvidenceStatus3DName(const SegmentEvidenceStatus3D status) noexcept {
  switch (status) {
    case SegmentEvidenceStatus3D::kValid:
      return "valid";
    case SegmentEvidenceStatus3D::kEmptyRoute:
      return "empty_route";
    case SegmentEvidenceStatus3D::kPlannerRejected:
      return "planner_rejected";
    case SegmentEvidenceStatus3D::kInvalidWorld:
      return "invalid_world";
    case SegmentEvidenceStatus3D::kOutsideFlightEnvelope:
      return "outside_flight_envelope";
    case SegmentEvidenceStatus3D::kOutsideGrid:
      return "outside_grid";
    case SegmentEvidenceStatus3D::kUnknownRejected:
      return "unknown_rejected";
    case SegmentEvidenceStatus3D::kInvalidEsdf:
      return "invalid_esdf";
    case SegmentEvidenceStatus3D::kRawCollision:
      return "raw_collision";
  }
  return "unknown";
}

} // namespace drone_city_nav
