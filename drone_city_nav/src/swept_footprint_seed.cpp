#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>
#include <vector>

#include "swept_footprint_internal.hpp"

namespace drone_city_nav {

// The proprioceptive seed's exemptions and the test of a box against the
// body's cylinder the validators share.
namespace {

using swept_footprint_detail::normalized;

[[nodiscard]] double squaredDistanceToBox(const Point3& point, const Point3& minimum,
                                          const Point3& maximum) noexcept {
  const double dx =
      point.x < minimum.x ? minimum.x - point.x : std::max(0.0, point.x - maximum.x);
  const double dy =
      point.y < minimum.y ? minimum.y - point.y : std::max(0.0, point.y - maximum.y);
  const double dz =
      point.z < minimum.z ? minimum.z - point.z : std::max(0.0, point.z - maximum.z);
  return dx * dx + dy * dy + dz * dz;
}

[[nodiscard]] bool conservativeLessOrEqual(const double value,
                                           const double limit) noexcept {
  constexpr double kContactTolerance = 64.0 * std::numeric_limits<double>::epsilon();
  const double scale = std::max({1.0, std::abs(value), std::abs(limit)});
  return value <= limit + kContactTolerance * scale;
}

[[nodiscard]] double squaredDistanceSegmentToBox(const Point3& first,
                                                 const Point3& second,
                                                 const Point3& minimum,
                                                 const Point3& maximum) noexcept {
  const Point3 direction{second.x - first.x, second.y - first.y, second.z - first.z};
  const auto distance_at = [&](const double ratio) noexcept {
    return squaredDistanceToBox(Point3{first.x + direction.x * ratio,
                                       first.y + direction.y * ratio,
                                       first.z + direction.z * ratio},
                                minimum, maximum);
  };

  // Point-to-box squared distance is a convex piecewise quadratic along the
  // segment. Box-face crossing ratios delimit every quadratic piece, so the
  // exact minimum is one of the breakpoints or a stationary point inside a
  // piece. A fixed-iteration search can overestimate this minimum and miss a
  // tangent or shallow collision.
  std::array<double, 8U> breakpoint_storage{0.0, 1.0};
  const std::span<double, 8U> breakpoints{breakpoint_storage};
  std::size_t breakpoint_count{2U};
  const auto add_axis_breakpoints = [&](const double start, const double delta,
                                        const double lower,
                                        const double upper) noexcept {
    if (delta == 0.0) {
      return;
    }
    for (const double boundary : {lower, upper}) {
      const double ratio = (boundary - start) / delta;
      if (ratio > 0.0 && ratio < 1.0) {
        breakpoints[breakpoint_count++] = ratio;
      }
    }
  };
  add_axis_breakpoints(first.x, direction.x, minimum.x, maximum.x);
  add_axis_breakpoints(first.y, direction.y, minimum.y, maximum.y);
  add_axis_breakpoints(first.z, direction.z, minimum.z, maximum.z);
  const std::span<double> active_breakpoints = breakpoints.first(breakpoint_count);
  std::sort(active_breakpoints.begin(), active_breakpoints.end());
  const auto unique_end =
      std::unique(active_breakpoints.begin(), active_breakpoints.end());
  breakpoint_count =
      static_cast<std::size_t>(std::distance(active_breakpoints.begin(), unique_end));

  double minimum_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < breakpoint_count; ++index) {
    minimum_distance_squared =
        std::min(minimum_distance_squared, distance_at(breakpoints[index]));
    if (index + 1U >= breakpoint_count ||
        !(breakpoints[index + 1U] > breakpoints[index])) {
      continue;
    }
    const double midpoint = 0.5 * (breakpoints[index] + breakpoints[index + 1U]);
    double derivative_constant{0.0};
    double derivative_slope{0.0};
    const auto add_active_axis = [&](const double start, const double delta,
                                     const double lower, const double upper) noexcept {
      const double coordinate = start + delta * midpoint;
      double active_boundary{0.0};
      if (coordinate < lower) {
        active_boundary = lower;
      } else if (coordinate > upper) {
        active_boundary = upper;
      } else {
        return;
      }
      derivative_constant += delta * (start - active_boundary);
      derivative_slope += delta * delta;
    };
    add_active_axis(first.x, direction.x, minimum.x, maximum.x);
    add_active_axis(first.y, direction.y, minimum.y, maximum.y);
    add_active_axis(first.z, direction.z, minimum.z, maximum.z);
    if (derivative_slope > 0.0) {
      const double stationary_ratio =
          std::clamp(-derivative_constant / derivative_slope, breakpoints[index],
                     breakpoints[index + 1U]);
      minimum_distance_squared =
          std::min(minimum_distance_squared, distance_at(stationary_ratio));
    }
  }
  return minimum_distance_squared;
}

} // namespace

namespace swept_footprint_detail {

[[nodiscard]] bool boxIntersectsFiniteCylinder(const Point3& center,
                                               const FootprintBodyAxis& axis,
                                               const Point3& lower, const Point3& upper,
                                               const double lower_extent_m,
                                               const double upper_extent_m,
                                               const double radius_squared) noexcept {
  const Point3 box_center{0.5 * (lower.x + upper.x), 0.5 * (lower.y + upper.y),
                          0.5 * (lower.z + upper.z)};
  const Point3 box_half_extent{0.5 * (upper.x - lower.x), 0.5 * (upper.y - lower.y),
                               0.5 * (upper.z - lower.z)};
  const Point3 relative_center{box_center.x - center.x, box_center.y - center.y,
                               box_center.z - center.z};
  const double projected_center = relative_center.x * axis.x +
                                  relative_center.y * axis.y +
                                  relative_center.z * axis.z;
  const double projected_half_extent = box_half_extent.x * std::abs(axis.x) +
                                       box_half_extent.y * std::abs(axis.y) +
                                       box_half_extent.z * std::abs(axis.z);
  if (!conservativeLessOrEqual(-lower_extent_m,
                               projected_center + projected_half_extent) ||
      !conservativeLessOrEqual(projected_center - projected_half_extent,
                               upper_extent_m)) {
    return false;
  }

  const auto squared_interval_distance = [](const double value, const double minimum,
                                            const double maximum) noexcept {
    if (value < minimum) {
      const double distance = minimum - value;
      return distance * distance;
    }
    if (value > maximum) {
      const double distance = value - maximum;
      return distance * distance;
    }
    return 0.0;
  };
  constexpr double kCardinalAxisTolerance{1.0e-12};
  if (std::abs(axis.x) >= 1.0 - kCardinalAxisTolerance) {
    return conservativeLessOrEqual(
        squared_interval_distance(center.y, lower.y, upper.y) +
            squared_interval_distance(center.z, lower.z, upper.z),
        radius_squared);
  }
  if (std::abs(axis.y) >= 1.0 - kCardinalAxisTolerance) {
    return conservativeLessOrEqual(
        squared_interval_distance(center.x, lower.x, upper.x) +
            squared_interval_distance(center.z, lower.z, upper.z),
        radius_squared);
  }
  if (std::abs(axis.z) >= 1.0 - kCardinalAxisTolerance) {
    return conservativeLessOrEqual(
        squared_interval_distance(center.x, lower.x, upper.x) +
            squared_interval_distance(center.y, lower.y, upper.y),
        radius_squared);
  }

  const Point3 axis_lower{center.x - lower_extent_m * axis.x,
                          center.y - lower_extent_m * axis.y,
                          center.z - lower_extent_m * axis.z};
  const Point3 axis_upper{center.x + upper_extent_m * axis.x,
                          center.y + upper_extent_m * axis.y,
                          center.z + upper_extent_m * axis.z};
  return conservativeLessOrEqual(
      squaredDistanceSegmentToBox(axis_lower, axis_upper, lower, upper),
      radius_squared);
}

bool pointIntersectsBody(const Point3& point, const Point3& center,
                         const FootprintBodyAxis& requested_axis,
                         const SweptFootprintConfig& config) noexcept {
  const FootprintBodyAxis axis = normalized(requested_axis);
  const Point3 delta{point.x - center.x, point.y - center.y, point.z - center.z};
  const double axial = delta.x * axis.x + delta.y * axis.y + delta.z * axis.z;
  if (axial < -std::max(0.0, config.lower_extent_m) ||
      axial > std::max(0.0, config.upper_extent_m)) {
    return false;
  }
  const double distance_squared =
      delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
  const double radial_squared = std::max(0.0, distance_squared - axial * axial);
  const double radius = std::max(0.0, config.radius_m);
  return radial_squared <= radius * radius;
}

} // namespace swept_footprint_detail

namespace {

using swept_footprint_detail::boxIntersectsFiniteCylinder;
using swept_footprint_detail::physicalBody;
using swept_footprint_detail::pointIntersectsBody;

[[nodiscard]] std::pair<double, double>
projectedBoxInterval(const Point3& minimum, const Point3& maximum, const Point3& origin,
                     const FootprintBodyAxis& axis) noexcept {
  const Point3 center{0.5 * (minimum.x + maximum.x), 0.5 * (minimum.y + maximum.y),
                      0.5 * (minimum.z + maximum.z)};
  const Point3 half_extent{0.5 * (maximum.x - minimum.x), 0.5 * (maximum.y - minimum.y),
                           0.5 * (maximum.z - minimum.z)};
  const Point3 relative{center.x - origin.x, center.y - origin.y, center.z - origin.z};
  const double projected_center =
      relative.x * axis.x + relative.y * axis.y + relative.z * axis.z;
  const double projected_half_extent = half_extent.x * std::abs(axis.x) +
                                       half_extent.y * std::abs(axis.y) +
                                       half_extent.z * std::abs(axis.z);
  return {projected_center - projected_half_extent,
          projected_center + projected_half_extent};
}

[[nodiscard]] bool
seedAllowsSupportContact(const ProprioceptiveFreeSpaceSeed3D& seed,
                         const Point3& box_minimum, const Point3& box_maximum,
                         const double occupancy_resolution_m) noexcept {
  const FootprintBodyAxis seed_axis = normalized(seed.body_axis);
  if (!boxIntersectsFiniteCylinder(seed.position, seed_axis, box_minimum, box_maximum,
                                   std::max(0.0, seed.footprint.lower_extent_m),
                                   std::max(0.0, seed.footprint.upper_extent_m),
                                   std::max(0.0, seed.footprint.radius_m) *
                                       std::max(0.0, seed.footprint.radius_m))) {
    return false;
  }
  const auto [box_minimum_axial, box_maximum_axial] =
      projectedBoxInterval(box_minimum, box_maximum, seed.position, seed_axis);
  const double lower_cap = -std::max(0.0, seed.footprint.lower_extent_m);
  constexpr double kContactToleranceM{1.0e-9};
  const double quantization_tolerance_m = std::max(0.0, occupancy_resolution_m);
  return box_minimum_axial <=
             lower_cap + quantization_tolerance_m + kContactToleranceM &&
         box_maximum_axial >=
             lower_cap - quantization_tolerance_m - kContactToleranceM &&
         box_minimum_axial < kContactToleranceM;
}

[[nodiscard]] double
seedContactToleranceM(const ProprioceptiveFreeSpaceSeed3D& seed) noexcept {
  return std::isfinite(seed.contact_tolerance_m)
             ? std::max(0.0, seed.contact_tolerance_m)
             : 0.0;
}

[[nodiscard]] SweptFootprintConfig
contactWidenedFootprint(const SweptFootprintConfig& footprint,
                        const double tolerance_m) noexcept {
  SweptFootprintConfig widened = footprint;
  widened.radius_m = std::max(0.0, footprint.radius_m) + tolerance_m;
  widened.lower_extent_m = std::max(0.0, footprint.lower_extent_m) + tolerance_m;
  widened.upper_extent_m = std::max(0.0, footprint.upper_extent_m) + tolerance_m;
  return widened;
}

} // namespace

bool proprioceptiveSeedAllowsSupportContact(
    const ProprioceptiveFreeSpaceSeed3D& seed, const Point3& box_minimum,
    const Point3& box_maximum, const double occupancy_resolution_m) noexcept {
  return seedAllowsSupportContact(seed, box_minimum, box_maximum,
                                  occupancy_resolution_m);
}

// Evidence the envelope overlaps at the seed is contact: the vehicle is there,
// so that evidence cannot be an obstacle for a validation run from that pose.
// The exemption removes the envelope's clearance margin from that evidence and
// nothing else: it carries no condition on where the body then moves — no
// closer than now, only along, only away — since such a condition is a
// prohibition on moving through free space and freezes a vehicle in contact.
// What it never removes is the body. Evidence in the margin between the body
// and the envelope at the seed stays binding for the body: a candidate pose
// whose physical body reaches it is a collision. Evidence the body itself
// overlaps at the seed is contact through and through - the vehicle
// demonstrably stands there - and stays allowed exactly as far in as the
// vehicle already is: an occupied voxel says nothing about where inside it
// the surface lies, so pressing deeper into it is not moving through free
// space. How far in is the depth of the body in the evidence, the least
// shrink of the body that would clear it, not the distance of the body's
// centre from the voxel: a body resting a tenth of a metre into the top of a
// wall moves along that wall at the same depth without going deeper. The
// depth is judged voxel by voxel: evidence the body reaches only at the
// candidate, never at the seed or along its departure, stays a collision, so
// a vehicle in contact with one wall of a shaft does not gain that depth
// against the other. Departing, holding and every free direction remain
// open.
SweptFootprintConfig
swept_footprint_detail::physicalBody(const SweptFootprintConfig& footprint) noexcept {
  SweptFootprintConfig body = footprint;
  body.radius_m = std::min(std::max(0.0, footprint.body_radius_m),
                           std::max(0.0, footprint.radius_m));
  body.lower_extent_m = std::min(std::max(0.0, footprint.body_lower_extent_m),
                                 std::max(0.0, footprint.lower_extent_m));
  body.upper_extent_m = std::min(std::max(0.0, footprint.body_upper_extent_m),
                                 std::max(0.0, footprint.upper_extent_m));
  return body;
}

// Found by bisection on the intersection test.
double bodyDepthInBoxM(const Point3& pose, const FootprintBodyAxis& axis,
                       const Point3& box_minimum, const Point3& box_maximum,
                       const SweptFootprintConfig& body) noexcept {
  const auto reaches = [&](const double shrink_m) {
    const double radius = body.radius_m - shrink_m;
    return boxIntersectsFiniteCylinder(pose, axis, box_minimum, box_maximum,
                                       body.lower_extent_m - shrink_m,
                                       body.upper_extent_m - shrink_m, radius * radius);
  };
  if (!reaches(0.0)) {
    return 0.0;
  }
  double clear_m = std::min({body.radius_m, body.lower_extent_m, body.upper_extent_m});
  if (!(clear_m > 0.0) || reaches(clear_m)) {
    return std::max(0.0, clear_m);
  }
  double reached_m = 0.0;
  for (int iteration = 0; iteration < 24; ++iteration) {
    const double midpoint_m = std::midpoint(reached_m, clear_m);
    (reaches(midpoint_m) ? reached_m : clear_m) = midpoint_m;
  }
  return reached_m;
}

// The depth of the body at `pose` at an obstacle point: how far inside the
// body's radius and extents the point lies. Zero when the point is outside.
[[nodiscard]] double bodyDepthAtPointM(const Point3& point, const Point3& pose,
                                       const FootprintBodyAxis& requested_axis,
                                       const SweptFootprintConfig& body) noexcept {
  const FootprintBodyAxis axis = normalized(requested_axis);
  const Point3 delta{point.x - pose.x, point.y - pose.y, point.z - pose.z};
  const double axial = delta.x * axis.x + delta.y * axis.y + delta.z * axis.z;
  const double radial = std::sqrt(std::max(0.0, delta.x * delta.x + delta.y * delta.y +
                                                    delta.z * delta.z - axial * axial));
  const double axial_depth =
      axial >= 0.0 ? body.upper_extent_m - axial : body.lower_extent_m + axial;
  return std::max(0.0, std::min(body.radius_m - radial, axial_depth));
}

// Contact the body already has may not deepen; float noise must not turn a
// hold into a collision.
constexpr double kContactDepthToleranceM{1.0e-3};

// The poses the seed stands for: the seed itself and, when the vehicle is
// leaving along a departure, every pose along that chain at the sweep step.
// The chain is the route's departure and starts where the route was planned,
// while the vehicle departs from where it is; the chain is walked as given
// and again carried to the seed, so the departure counts from both.
template<typename Visit>
void forEachContactPose(const ProprioceptiveFreeSpaceSeed3D& seed, Visit&& visit) {
  visit(seed.position);
  if (seed.departure_chain.empty()) {
    return;
  }
  const double step_m = std::max(seed.footprint.sweep_step_m, 1.0e-3);
  const auto walk = [&](const Point3& carry) {
    Point3 previous = seed.position;
    for (const Point3& chain_pose : seed.departure_chain) {
      const Point3 pose{chain_pose.x + carry.x, chain_pose.y + carry.y,
                        chain_pose.z + carry.z};
      const double length_m = distance3D(previous, pose);
      const auto count = static_cast<std::size_t>(std::ceil(length_m / step_m));
      for (std::size_t index = 1U; index <= count; ++index) {
        const double ratio = static_cast<double>(index) / static_cast<double>(count);
        visit(Point3{std::lerp(previous.x, pose.x, ratio),
                     std::lerp(previous.y, pose.y, ratio),
                     std::lerp(previous.z, pose.z, ratio)});
      }
      previous = pose;
    }
  };
  walk(Point3{});
  const Point3& front = seed.departure_chain.front();
  const Point3 carry{seed.position.x - front.x, seed.position.y - front.y,
                     seed.position.z - front.z};
  if (std::hypot(std::hypot(carry.x, carry.y), carry.z) > kContactDepthToleranceM) {
    walk(carry);
  }
}

bool proprioceptiveSeedExemptsBoxAtAll(
    const ProprioceptiveFreeSpaceSeed3D& seed,
    const std::span<const Point3> candidate_positions, const Point3& box_minimum,
    const Point3& box_maximum) noexcept {
  const double tolerance_m = seedContactToleranceM(seed);
  const SweptFootprintConfig contact_envelope =
      contactWidenedFootprint(seed.footprint, tolerance_m);
  const FootprintBodyAxis axis = normalized(seed.body_axis);
  const SweptFootprintConfig body = physicalBody(seed.footprint);
  // Contact: the envelope overlaps the box at some pose the vehicle stands
  // for. Where the body itself overlaps it there, the vehicle is already that
  // deep, and the candidates may not go deeper than the deepest such pose. The
  // body shrunk by the candidates' depth reaching the box at a pose is that
  // pose being as deep, so the walk ends at the first one and bisects none:
  // beside a wall a bisection per pose per voxel per body position made one
  // path validation cost 30 to 55 ms and a horizon's assembly 430 ms (r805).
  double deepest_m = 0.0;
  for (const Point3& position : candidate_positions) {
    deepest_m = std::max(
        deepest_m, bodyDepthInBoxM(position, axis, box_minimum, box_maximum, body));
  }
  const double required_m = deepest_m - kContactDepthToleranceM;
  const SweptFootprintConfig& reached = required_m > 0.0 ? body : contact_envelope;
  const double shrink_m = std::max(0.0, required_m);
  const double radius_m = reached.radius_m - shrink_m;
  bool exempt{false};
  forEachContactPose(seed, [&](const Point3& pose) {
    exempt = exempt || boxIntersectsFiniteCylinder(pose, axis, box_minimum, box_maximum,
                                                   reached.lower_extent_m - shrink_m,
                                                   reached.upper_extent_m - shrink_m,
                                                   radius_m * radius_m);
  });
  return !candidate_positions.empty() && exempt;
}

bool proprioceptiveSeedExemptsPointAtAll(
    const ProprioceptiveFreeSpaceSeed3D& seed,
    const std::span<const Point3> candidate_positions,
    const Point3& obstacle_point) noexcept {
  const double tolerance_m = seedContactToleranceM(seed);
  const SweptFootprintConfig contact_envelope =
      contactWidenedFootprint(seed.footprint, tolerance_m);
  const SweptFootprintConfig body = physicalBody(seed.footprint);
  bool contact{false};
  double reference_depth_m = 0.0;
  forEachContactPose(seed, [&](const Point3& pose) {
    if (!pointIntersectsBody(obstacle_point, pose, seed.body_axis, contact_envelope)) {
      return;
    }
    contact = true;
    reference_depth_m =
        std::max(reference_depth_m,
                 bodyDepthAtPointM(obstacle_point, pose, seed.body_axis, body));
  });
  return contact &&
         std::ranges::all_of(candidate_positions, [&](const Point3& position) {
           return bodyDepthAtPointM(obstacle_point, position, seed.body_axis, body) <=
                  reference_depth_m + kContactDepthToleranceM;
         });
}

bool proprioceptiveSeedExemptsPoint(const ProprioceptiveFreeSpaceSeed3D& seed,
                                    const Point3& candidate_position,
                                    const Point3& obstacle_point) noexcept {
  return proprioceptiveSeedExemptsPointAtAll(
      seed, std::span<const Point3>{&candidate_position, 1U}, obstacle_point);
}

} // namespace drone_city_nav
