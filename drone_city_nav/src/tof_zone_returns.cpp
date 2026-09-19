#include "drone_city_nav/tof_zone_returns.hpp"

#include <cmath>

namespace drone_city_nav {

bool tofZoneReturnsConfigIsValid(const TofZoneReturnsConfig& config) noexcept {
  return config.zones_per_side > 0U && config.sub_rays > 0U &&
         std::isfinite(config.field_of_view_rad) && config.field_of_view_rad > 0.0 &&
         config.field_of_view_rad < 3.0 && std::isfinite(config.minimum_range_m) &&
         config.minimum_range_m >= 0.0 && std::isfinite(config.maximum_range_m) &&
         config.maximum_range_m > config.minimum_range_m &&
         std::isfinite(config.position_m.x) && std::isfinite(config.position_m.y) &&
         std::isfinite(config.position_m.z);
}

std::vector<StereoDepthReturn>
tofZoneReturns(const std::span<const Point3> zone_points_sensor_flu,
               const TofZoneReturnsConfig& config) {
  std::vector<StereoDepthReturn> returns;
  const std::size_t side = config.zones_per_side;
  if (!tofZoneReturnsConfigIsValid(config) ||
      zone_points_sensor_flu.size() != side * side) {
    return returns;
  }
  const double zone_pitch_rad = config.field_of_view_rad / static_cast<double>(side);
  const double sub_pitch_rad = zone_pitch_rad / static_cast<double>(config.sub_rays);
  returns.reserve(side * side * config.sub_rays * config.sub_rays);
  for (std::size_t row = 0U; row < side; ++row) {
    for (std::size_t column = 0U; column < side; ++column) {
      const Point3& point = zone_points_sensor_flu[row * side + column];
      const bool finite =
          std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
      const double measured_m =
          finite ? std::hypot(std::hypot(point.x, point.y), point.z) : 0.0;
      if (finite && measured_m < config.minimum_range_m) {
        continue;
      }
      const bool hit = finite && measured_m <= config.maximum_range_m;
      const double range_m = hit ? measured_m : config.maximum_range_m;
      // The zone's corner angle about the boresight, then its sub-rays' centres.
      const double zone_vertical_rad =
          -0.5 * config.field_of_view_rad + static_cast<double>(row) * zone_pitch_rad;
      const double zone_horizontal_rad = -0.5 * config.field_of_view_rad +
                                         static_cast<double>(column) * zone_pitch_rad;
      for (std::size_t sub_row = 0U; sub_row < config.sub_rays; ++sub_row) {
        for (std::size_t sub_column = 0U; sub_column < config.sub_rays; ++sub_column) {
          const double vertical_rad =
              zone_vertical_rad + (static_cast<double>(sub_row) + 0.5) * sub_pitch_rad;
          const double horizontal_rad =
              zone_horizontal_rad +
              (static_cast<double>(sub_column) + 0.5) * sub_pitch_rad;
          // Sensor frame: x the boresight, y left, z up.
          const double along = std::cos(vertical_rad) * std::cos(horizontal_rad);
          const double left = std::cos(vertical_rad) * std::sin(horizontal_rad);
          const double up = std::sin(vertical_rad);
          // Looking up the boresight is the publishing frame's +z and the
          // sensor's up is its -x; looking down, -z and +x.
          const double sign = config.looks_up ? 1.0 : -1.0;
          returns.push_back(StereoDepthReturn{
              .point = Point3{config.position_m.x - sign * up * range_m,
                              config.position_m.y + left * range_m,
                              config.position_m.z + sign * along * range_m},
              .hit = hit});
        }
      }
    }
  }
  return returns;
}

} // namespace drone_city_nav
