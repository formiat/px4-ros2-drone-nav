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
// `zone_intensities`, one a zone in the same order or empty, is the return's
// intensity as the driver reports it relative to a matte surface's: a zone
// returning under kSmokeIntensityShare of it, and over nought (nought is no
// reading), is smoke (roadmap item 18 stage 2, specification K38), its rays
// observed unobservable. On the smoke rig the ring inside smoke of 1/m
// returned from every zone at 0.012 of a surface's intensity; a wall returns
// one. The ring returns from smoke at its aperture (the rig: 0.05 m), inside
// the body the memory filters as the vehicle's own; the smoke's rays are
// placed at the body's edge instead, kSmokeShellM, so that the plume the
// vehicle is in enters the memory where it stands, and the way it came stays
// open (the memory never marks the flown path).
inline constexpr float kSmokeIntensityShare{0.5F};
inline constexpr double kSmokeShellM{0.7};
[[nodiscard]] std::vector<StereoDepthReturn>
tofZoneReturns(std::span<const Point3> zone_points_sensor_flu,
               const TofZoneReturnsConfig& config,
               std::span<const float> zone_intensities = {});

} // namespace drone_city_nav
