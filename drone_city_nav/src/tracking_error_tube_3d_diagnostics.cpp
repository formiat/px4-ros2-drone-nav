#include "drone_city_nav/tracking_error_tube_3d.hpp"

#include <cstddef>
#include <cstdio>
#include <span>
#include <string>

namespace drone_city_nav {

std::string
describeTrackingErrorTubeConstraints3D(const std::span<const RouteSample3D> route,
                                       const TrackingErrorTubeProfile3D& profile,
                                       const std::size_t maximum_ranges) {
  if (!trackingErrorTubeProfile3DIsValid(profile, route.size()) ||
      profile.constrained_segment_count == 0U) {
    return {};
  }
  constexpr double kSpeedToleranceMps{1.0e-6};
  const double unconstrained_mps = profile.unconstrained_speed_limit_mps;
  std::string description;
  std::size_t named_ranges{0U};
  std::size_t omitted_ranges{0U};
  std::size_t index{0U};
  while (index < route.size()) {
    if (profile.speed_limits_mps[index] + kSpeedToleranceMps >= unconstrained_mps) {
      ++index;
      continue;
    }
    const std::size_t begin = index;
    std::size_t lowest = index;
    while (index < route.size() &&
           profile.speed_limits_mps[index] + kSpeedToleranceMps < unconstrained_mps) {
      if (profile.speed_limits_mps[index] < profile.speed_limits_mps[lowest]) {
        lowest = index;
      }
      ++index;
    }
    if (named_ranges >= maximum_ranges) {
      ++omitted_ranges;
      continue;
    }
    const RouteSample3D& lowest_sample = route[lowest];
    char entry[160];
    std::snprintf(entry, sizeof(entry), "%s%.1f-%.1f:%.2f@(%.1f,%.1f,%.1f)",
                  named_ranges == 0U ? "" : ";", route[begin].station_m,
                  route[index - 1U].station_m, profile.speed_limits_mps[lowest],
                  lowest_sample.position.x, lowest_sample.position.y,
                  lowest_sample.position.z);
    description += entry;
    ++named_ranges;
  }
  if (omitted_ranges > 0U) {
    description += ";+" + std::to_string(omitted_ranges);
  }
  return description;
}

} // namespace drone_city_nav
