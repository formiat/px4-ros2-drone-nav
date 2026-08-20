#include "drone_city_nav/lidar_self_filter.hpp"

#include <cmath>

namespace drone_city_nav {

bool lidarSelfFilterConfigIsValid(const LidarSelfFilterConfig& config) noexcept {
  return std::isfinite(config.horizontal_radius_m) &&
         config.horizontal_radius_m > 0.0 && std::isfinite(config.upward_extent_m) &&
         config.upward_extent_m >= 0.0 && std::isfinite(config.downward_extent_m) &&
         config.downward_extent_m >= 0.0;
}

bool isLidarSelfReturn(const Point3& point_body_frd,
                       const LidarSelfFilterConfig& config) noexcept {
  if (!lidarSelfFilterConfigIsValid(config) || !std::isfinite(point_body_frd.x) ||
      !std::isfinite(point_body_frd.y) || !std::isfinite(point_body_frd.z)) {
    return false;
  }
  const double radial_distance_squared =
      point_body_frd.x * point_body_frd.x + point_body_frd.y * point_body_frd.y;
  return radial_distance_squared <=
             config.horizontal_radius_m * config.horizontal_radius_m &&
         point_body_frd.z >= -config.upward_extent_m &&
         point_body_frd.z <= config.downward_extent_m;
}

} // namespace drone_city_nav
