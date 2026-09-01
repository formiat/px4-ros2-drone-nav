#pragma once

#include "drone_city_nav/footprint_geometry_3d.hpp"
#include "drone_city_nav/grid_config.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"

#include <optional>

namespace drone_city_nav {

// Deriving launch-support contact tests the physical body against observed
// occupancy, so it is collision reasoning over world evidence rather than a
// property of the observed world itself.
[[nodiscard]] std::optional<LaunchSupportContact3D>
detectLaunchSupportContact3D(const ObservedOccupancyGrid3D& occupancy,
                             const ProprioceptiveFreeSpaceSeed3D& seed);

[[nodiscard]] LaunchSupportContact3D
makeVehicleLandedSupportContact3D(const GridBounds3D& bounds,
                                  const ProprioceptiveFreeSpaceSeed3D& seed);

} // namespace drone_city_nav
