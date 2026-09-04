#pragma once

#include "drone_city_nav/footprint_geometry_3d.hpp"
#include "drone_city_nav/motion_state_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <optional>

namespace drone_city_nav {

// The vehicle's own pose as contact evidence. The body demonstrably occupies
// its volume there, so occupied evidence overlapping that volume is contact
// rather than an obstacle for validations run from that pose: a validator
// suppresses it while the body keeps the stand-off it already has from it.
//
// The seed is transient execution evidence, like the newest lidar returns: it
// is always the vehicle's current pose, never the pose a resident route was
// certified from. A seed frozen into a certified world would stop exempting
// evidence that closed in on the vehicle since, and the vehicle would have no
// validated motion left at all, not even a stop.
[[nodiscard]] ProprioceptiveFreeSpaceSeed3D
proprioceptiveContactSeed3D(const Point3& position, const FootprintBodyAxis& body_axis,
                            const SweptFootprintConfig& footprint,
                            double occupancy_resolution_m) noexcept;

[[nodiscard]] ProprioceptiveFreeSpaceSeed3D proprioceptiveContactSeed3D(
    const Point3& position, const MotionControl3D& previous_control,
    const SweptFootprintConfig& footprint, double occupancy_resolution_m) noexcept;

// The same seed against one observed world, which also supplies the stand-off
// the body keeps from the evidence it touches. Returns no seed without an
// observed world: a static world is not evidence the vehicle observed from
// where it stands.
[[nodiscard]] std::optional<ProprioceptiveFreeSpaceSeed3D>
proprioceptiveContactSeed3D(const Point3& position, const FootprintBodyAxis& body_axis,
                            const SweptFootprintConfig& footprint,
                            const ObservedOccupancyGrid3D* observed_occupancy);

[[nodiscard]] std::optional<ProprioceptiveFreeSpaceSeed3D>
proprioceptiveContactSeed3D(const Point3& position,
                            const MotionControl3D& previous_control,
                            const SweptFootprintConfig& footprint,
                            const ObservedOccupancyGrid3D* observed_occupancy);

} // namespace drone_city_nav
