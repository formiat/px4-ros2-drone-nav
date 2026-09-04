#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"

#include "drone_city_nav/swept_footprint.hpp"

#include <cmath>

namespace drone_city_nav {

ProprioceptiveFreeSpaceSeed3D
proprioceptiveContactSeed3D(const Point3& position,
                            const MotionControl3D& previous_control,
                            const SweptFootprintConfig& footprint,
                            const double occupancy_resolution_m) noexcept {
  // Half a voxel: the largest positional ambiguity of one occupied cell.
  const double tolerance_m =
      std::isfinite(occupancy_resolution_m) && occupancy_resolution_m > 0.0
          ? 0.5 * occupancy_resolution_m
          : 0.0;
  return ProprioceptiveFreeSpaceSeed3D{
      .position = position,
      .body_axis = bodyAxisFromWorldAcceleration(
          Vec3{previous_control.ax, previous_control.ay, previous_control.az}),
      .footprint = footprint,
      .contact_tolerance_m = tolerance_m,
  };
}

std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptiveContactSeed3D(
    const Point3& position, const MotionControl3D& previous_control,
    const SweptFootprintConfig& footprint,
    const ObservedOccupancyGrid3D* const observed_occupancy) noexcept {
  if (observed_occupancy == nullptr) {
    return std::nullopt;
  }
  return proprioceptiveContactSeed3D(position, previous_control, footprint,
                                     observed_occupancy->bounds().resolution_m);
}

} // namespace drone_city_nav
