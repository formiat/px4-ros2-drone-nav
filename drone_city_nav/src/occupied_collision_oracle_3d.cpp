#include "drone_city_nav/occupied_collision_oracle_3d.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupiedCollisionResult3D
classifyRawValidation(const SweptFootprintResult& validation) noexcept {
  if (validation.status == SweptFootprintStatus::kRawCollision) {
    return {.status = OccupiedCollisionStatus3D::kRawCollision,
            .failure_point = validation.failure_point};
  }
  if (validation.status == SweptFootprintStatus::kInvalidInput) {
    return {.status = OccupiedCollisionStatus3D::kInvalidInput,
            .failure_point = validation.failure_point};
  }
  // Raw occupied-only validators deliberately treat outside-grid and unknown
  // cells as clear.
  return {.status = OccupiedCollisionStatus3D::kClear};
}

[[nodiscard]] OccupiedCollisionResult3D
withSource(OccupiedCollisionResult3D result,
           const OccupiedCollisionSource3D source) noexcept {
  result.source = source;
  return result;
}

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool validBodyAxis(const FootprintBodyAxis& axis) noexcept {
  if (!std::isfinite(axis.x) || !std::isfinite(axis.y) || !std::isfinite(axis.z)) {
    return false;
  }
  const double norm_squared = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
  return std::isfinite(norm_squared) && norm_squared > 1.0e-18;
}

[[nodiscard]] bool validFootprint(const SweptFootprintConfig& footprint) noexcept {
  return std::isfinite(footprint.radius_m) && footprint.radius_m >= 0.0 &&
         std::isfinite(footprint.body_radius_m) && footprint.body_radius_m >= 0.0 &&
         std::isfinite(footprint.body_lower_extent_m) &&
         footprint.body_lower_extent_m >= 0.0 &&
         std::isfinite(footprint.body_upper_extent_m) &&
         footprint.body_upper_extent_m >= 0.0 &&
         std::isfinite(footprint.lower_extent_m) && footprint.lower_extent_m >= 0.0 &&
         std::isfinite(footprint.upper_extent_m) && footprint.upper_extent_m >= 0.0 &&
         std::isfinite(footprint.sweep_step_m) && footprint.sweep_step_m > 0.0 &&
         std::isfinite(footprint.safe_clearance_threshold_m) &&
         footprint.safe_clearance_threshold_m >= 0.0;
}

[[nodiscard]] bool finiteRawPointCloud(const std::span<const Point3> points) noexcept {
  return std::ranges::all_of(points, finitePoint);
}

[[nodiscard]] bool
validProprioceptiveSeed(const ProprioceptiveFreeSpaceSeed3D* const seed) noexcept {
  return seed == nullptr ||
         (finitePoint(seed->position) && validBodyAxis(seed->body_axis) &&
          validFootprint(seed->footprint) && std::isfinite(seed->contact_tolerance_m) &&
          seed->contact_tolerance_m >= 0.0);
}

[[nodiscard]] bool validWorldContract(const OccupiedCollisionWorld3D& world) noexcept {
  return validFootprint(world.footprint) &&
         validProprioceptiveSeed(world.proprioceptive_free_space_seed) &&
         finiteRawPointCloud(world.raw_point_cloud.points()) &&
         (!world.flight_envelope ||
          evaluateFlightEnvelopeAltitude(world.flight_envelope->minimum_target_z_m,
                                         *world.flight_envelope) ==
              FlightEnvelopeStatus::kValid);
}

} // namespace

OccupiedCollisionOracle3D::OccupiedCollisionOracle3D(
    OccupiedCollisionWorld3D world) noexcept
    : world_{world},
      world_valid_{validWorldContract(world_)} {
}

OccupiedCollisionResult3D OccupiedCollisionOracle3D::validatePoint(
    const Point3& position, const FootprintBodyAxis& body_axis) const noexcept {
  if (!finitePoint(position) || !validBodyAxis(body_axis) || !world_valid_) {
    return {.status = OccupiedCollisionStatus3D::kInvalidInput,
            .failure_point = position};
  }
  if (world_.flight_envelope &&
      !insideFlightEnvelope(position, *world_.flight_envelope)) {
    return {.status = OccupiedCollisionStatus3D::kOutsideFlightEnvelope,
            .source = OccupiedCollisionSource3D::kFlightEnvelope,
            .failure_point = position};
  }
  if (world_.observed_occupancy != nullptr) {
    const OccupiedCollisionResult3D observed = withSource(
        classifyRawValidation(validateRawFootprintAt(
            *world_.observed_occupancy, position, body_axis, world_.footprint,
            world_.launch_support_contact, world_.proprioceptive_free_space_seed)),
        OccupiedCollisionSource3D::kObservedOccupancy);
    if (!observed.clear()) {
      return observed;
    }
  }
  if (world_.static_occupancy != nullptr) {
    const OccupiedCollisionResult3D known =
        withSource(classifyRawValidation(validateRawFootprintAt(
                       *world_.static_occupancy, position, body_axis, world_.footprint,
                       world_.proprioceptive_free_space_seed)),
                   OccupiedCollisionSource3D::kStaticOccupancy);
    if (!known.clear()) {
      return known;
    }
  }
  if (world_.planar_occupancy != nullptr) {
    const OccupiedCollisionResult3D planar =
        withSource(classifyRawValidation(validateRawFootprintAt(
                       *world_.planar_occupancy, position, world_.footprint)),
                   OccupiedCollisionSource3D::kPlanarOccupancy);
    if (!planar.clear()) {
      return planar;
    }
  }
  if (!world_.raw_point_cloud.empty()) {
    const OccupiedCollisionResult3D point_cloud = withSource(
        classifyRawValidation(validateRawPointCloudFootprintAt(
            world_.raw_point_cloud, position, body_axis, world_.footprint,
            world_.launch_support_contact, world_.proprioceptive_free_space_seed)),
        OccupiedCollisionSource3D::kRawPointCloud);
    if (!point_cloud.clear()) {
      return point_cloud;
    }
  }
  return {.status = OccupiedCollisionStatus3D::kClear};
}

OccupiedCollisionResult3D OccupiedCollisionOracle3D::validateSegment(
    const Point3& first, const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis) const noexcept {
  if (!finitePoint(first) || !finitePoint(second) || !validBodyAxis(first_body_axis) ||
      !validBodyAxis(second_body_axis) || !world_valid_) {
    return {.status = OccupiedCollisionStatus3D::kInvalidInput,
            .failure_point = !finitePoint(first) ? first : second};
  }
  if (world_.flight_envelope &&
      !segmentInsideFlightEnvelope(first, second, *world_.flight_envelope)) {
    return {.status = OccupiedCollisionStatus3D::kOutsideFlightEnvelope,
            .source = OccupiedCollisionSource3D::kFlightEnvelope,
            .failure_point =
                !insideFlightEnvelope(first, *world_.flight_envelope) ? first : second};
  }
  if (world_.observed_occupancy != nullptr) {
    const OccupiedCollisionResult3D observed = withSource(
        classifyRawValidation(validateRawSweptFootprint(
            *world_.observed_occupancy, first, first_body_axis, second,
            second_body_axis, world_.footprint, world_.launch_support_contact,
            world_.proprioceptive_free_space_seed)),
        OccupiedCollisionSource3D::kObservedOccupancy);
    if (!observed.clear()) {
      return observed;
    }
  }
  if (world_.static_occupancy != nullptr) {
    const OccupiedCollisionResult3D known = withSource(
        classifyRawValidation(validateRawSweptFootprint(
            *world_.static_occupancy, first, first_body_axis, second, second_body_axis,
            world_.footprint, world_.proprioceptive_free_space_seed)),
        OccupiedCollisionSource3D::kStaticOccupancy);
    if (!known.clear()) {
      return known;
    }
  }
  if (world_.planar_occupancy != nullptr) {
    const OccupiedCollisionResult3D planar =
        withSource(classifyRawValidation(validateRawSweptFootprint(
                       *world_.planar_occupancy, first, second, world_.footprint)),
                   OccupiedCollisionSource3D::kPlanarOccupancy);
    if (!planar.clear()) {
      return planar;
    }
  }
  if (!world_.raw_point_cloud.empty()) {
    const OccupiedCollisionResult3D point_cloud = withSource(
        classifyRawValidation(validateRawPointCloudSweptFootprint(
            world_.raw_point_cloud, first, first_body_axis, second, second_body_axis,
            world_.footprint, world_.launch_support_contact,
            world_.proprioceptive_free_space_seed)),
        OccupiedCollisionSource3D::kRawPointCloud);
    if (!point_cloud.clear()) {
      return point_cloud;
    }
  }
  return {.status = OccupiedCollisionStatus3D::kClear};
}

const OccupiedCollisionWorld3D& OccupiedCollisionOracle3D::world() const noexcept {
  return world_;
}

const char*
occupiedCollisionStatus3DName(const OccupiedCollisionStatus3D status) noexcept {
  switch (status) {
    case OccupiedCollisionStatus3D::kClear:
      return "clear";
    case OccupiedCollisionStatus3D::kOutsideFlightEnvelope:
      return "outside_flight_envelope";
    case OccupiedCollisionStatus3D::kRawCollision:
      return "raw_collision";
    case OccupiedCollisionStatus3D::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

} // namespace drone_city_nav
