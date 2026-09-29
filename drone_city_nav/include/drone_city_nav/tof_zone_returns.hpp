#pragma once

#include "drone_city_nav/stereo_depth_returns.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

// A multizone time-of-flight sensor: a square matrix of zones over a square
// field of view, one range a zone. A zone is a cone, not a ray: it answers
// with the nearest surface anywhere in its cross-section, and a narrow object
// in it returns a range that belongs to no single direction. Its evidence is
// therefore spread over `sub_rays` by `sub_rays` rays across the zone: a
// surface fills the whole cross-section at the measured range, and a zone
// that saw nothing is free across its cross-section up to the rated range.
struct TofZoneReturnsConfig {
  std::size_t zones_per_side{8U};
  double field_of_view_rad{0.7853981633974483};
  double maximum_range_m{2.8};
  double minimum_range_m{0.05};
  std::size_t sub_rays{3U};
  // The sensor's orientation and position in the frame the returns are
  // published in, as a pose states them: its forward-left-up frame pitched
  // about the frame's y axis, then turned about its z axis. A pitch of -pi/2
  // looks up, +pi/2 down; a pitch of zero looks along the yaw on the
  // horizontal (roadmap item 17 stage 4's ring).
  double pitch_rad{-1.5707963267948966};
  double yaw_rad{0.0};
  Point3 position_m{};
};

[[nodiscard]] bool
tofZoneReturnsConfigIsValid(const TofZoneReturnsConfig& config) noexcept;

// `zone_points_sensor_flu` holds one return a zone, row by row, in the
// sensor's own forward-left-up frame whose x axis is the boresight; a return
// that is not finite is a zone that saw nothing within its range. The rays
// are returned as end points in the publishing frame. They leave the sensor,
// not that frame's origin: `position_m` is part of every end point, and the
// consumer that casts them from the origin bends each by that offset.
[[nodiscard]] std::vector<StereoDepthReturn>
tofZoneReturns(std::span<const Point3> zone_points_sensor_flu,
               const TofZoneReturnsConfig& config);

} // namespace drone_city_nav
