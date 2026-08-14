#include "drone_city_nav/radar_visibility.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace drone_city_nav {

RadarVisibilityStatus radarLineOfSightStatus(const OccupancyGrid3D& occupancy,
                                             const Point3& radar_position,
                                             const Point3& target_position,
                                             const double sample_spacing_m) noexcept {
  const double range_m = distance3D(radar_position, target_position);
  if (!std::isfinite(range_m) || !std::isfinite(sample_spacing_m) ||
      !(sample_spacing_m > 0.0)) {
    return RadarVisibilityStatus::kInvalidInput;
  }
  const auto radar_cell = occupancy.worldToCell(radar_position);
  const auto target_cell = occupancy.worldToCell(target_position);
  if (!radar_cell.has_value() || !target_cell.has_value()) {
    return RadarVisibilityStatus::kOutsideWorld;
  }
  if (range_m <= 1.0e-9) {
    return RadarVisibilityStatus::kVisible;
  }

  const std::size_t segment_count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(range_m / sample_spacing_m)));
  for (std::size_t sample = 0U; sample <= segment_count; ++sample) {
    const double fraction =
        static_cast<double>(sample) / static_cast<double>(segment_count);
    const Point3 point{
        radar_position.x + (target_position.x - radar_position.x) * fraction,
        radar_position.y + (target_position.y - radar_position.y) * fraction,
        radar_position.z + (target_position.z - radar_position.z) * fraction,
    };
    const auto cell = occupancy.worldToCell(point);
    if (!cell.has_value()) {
      return RadarVisibilityStatus::kOutsideWorld;
    }
    if (occupancy.isOccupied(*cell)) {
      return RadarVisibilityStatus::kOccluded;
    }
  }
  return RadarVisibilityStatus::kVisible;
}

} // namespace drone_city_nav
