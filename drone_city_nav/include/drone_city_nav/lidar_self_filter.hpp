#pragma once

#include "drone_city_nav/types.hpp"

namespace drone_city_nav {

struct LidarSelfFilterConfig {
  double horizontal_radius_m{0.9};
  double upward_extent_m{0.5};
  double downward_extent_m{0.5};
};

[[nodiscard]] bool
lidarSelfFilterConfigIsValid(const LidarSelfFilterConfig& config) noexcept;

[[nodiscard]] bool isLidarSelfReturn(const Point3& point_body_frd,
                                     const LidarSelfFilterConfig& config) noexcept;

} // namespace drone_city_nav
