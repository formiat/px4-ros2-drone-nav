#include "drone_city_nav/latest_sensor_clear_sight_3d.hpp"

#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool covers(const LatestSensorClearSight3D& sight, const Point3& point,
                          const double margin_m) noexcept {
  const Vec3 offset{point.x - sight.origin_map.x, point.y - sight.origin_map.y,
                    point.z - sight.origin_map.z};
  const double along_m = offset.x * sight.axis_map.x + offset.y * sight.axis_map.y +
                         offset.z * sight.axis_map.z;
  const double distance_m = std::hypot(offset.x, offset.y, offset.z);
  if (!(along_m > 0.0) || distance_m + margin_m > sight.range_m) {
    return false;
  }
  const double across_m =
      std::sqrt(std::max(0.0, distance_m * distance_m - along_m * along_m));
  return across_m + margin_m <= along_m * std::tan(sight.half_angle_rad);
}

} // namespace

bool latestSensorClearSightValid(const LatestSensorClearSight3D& sight) noexcept {
  if (!std::isfinite(sight.range_m) || sight.range_m < 0.0 ||
      !std::isfinite(sight.half_angle_rad) || sight.half_angle_rad < 0.0 ||
      sight.half_angle_rad >= 0.5 * std::numbers::pi) {
    return false;
  }
  if (sight.range_m == 0.0) {
    return true;
  }
  const double axis_norm =
      std::hypot(sight.axis_map.x, sight.axis_map.y, sight.axis_map.z);
  return std::isfinite(sight.origin_map.x) && std::isfinite(sight.origin_map.y) &&
         std::isfinite(sight.origin_map.z) && std::isfinite(axis_norm) &&
         std::abs(axis_norm - 1.0) <= 1.0e-3 && sight.half_angle_rad > 0.0;
}

bool clearSightCoversSegment(const LatestSensorClearSight3D& sight, const Point3& first,
                             const Point3& second, const double margin_m) noexcept {
  return sight.range_m > 0.0 && margin_m >= 0.0 && covers(sight, first, margin_m) &&
         covers(sight, second, margin_m);
}

} // namespace drone_city_nav
