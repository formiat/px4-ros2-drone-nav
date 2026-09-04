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

// Whether a pose may anchor a launch-support contact. The support exempts
// occupied evidence around its anchor, so the anchor must be a pose the
// vehicle demonstrably rested at: the land detector reports contact now and
// the vehicle is at rest. A detector latch that fired earlier is not enough.
// By the time the first evidence reaches the navigation stack the vehicle may
// already be airborne, and a support anchored in mid-air exempts evidence that
// is not contact and never releases.
[[nodiscard]] bool launchSupportAnchorAdmissible3D(bool land_contact_now,
                                                   double speed_mps,
                                                   double maximum_speed_mps) noexcept;

// Whether the vehicle has left the support behind. The body must be clear of
// occupied evidence without the exemption, and the vehicle must have left the
// contact envelope in any direction: a vehicle that departs sideways owes the
// support nothing, exactly like one that climbs off it.
[[nodiscard]] bool launchSupportReleased3D(const LaunchSupportContact3D& contact,
                                           const Point3& position,
                                           bool body_clear_without_support,
                                           double occupancy_resolution_m) noexcept;

} // namespace drone_city_nav
