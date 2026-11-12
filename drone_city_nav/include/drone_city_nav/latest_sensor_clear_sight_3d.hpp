#pragma once

#include "drone_city_nav/types.hpp"

namespace drone_city_nav {

// The cone of a frame's clear sight: from `origin_map` about `axis_map` every
// beam within `half_angle_rad` was a valid one that returned nothing nearer
// than `range_m`. The sensor's own contract makes that free space: an
// obstacle of the size it promises to detect would have returned one of
// those beams. Nought of range is no cone.
struct LatestSensorClearSight3D {
  Point3 origin_map{};
  Vec3 axis_map{};
  double half_angle_rad{0.0};
  double range_m{0.0};
};

[[nodiscard]] bool
latestSensorClearSightValid(const LatestSensorClearSight3D& sight) noexcept;

// Whether a body of `margin_m` about the segment lies inside the cone and its
// range.
[[nodiscard]] bool clearSightCoversSegment(const LatestSensorClearSight3D& sight,
                                           const Point3& first, const Point3& second,
                                           double margin_m) noexcept;

} // namespace drone_city_nav
