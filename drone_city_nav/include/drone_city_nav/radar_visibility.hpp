#pragma once

#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

namespace drone_city_nav {

enum class RadarVisibilityStatus {
  kVisible,
  kOccluded,
  kOutsideWorld,
  kInvalidInput,
};

[[nodiscard]] RadarVisibilityStatus
radarLineOfSightStatus(const OccupancyGrid3D& occupancy, const Point3& radar_position,
                       const Point3& target_position, double sample_spacing_m) noexcept;

} // namespace drone_city_nav
