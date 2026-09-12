#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"

#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

// Half a voxel: the largest positional ambiguity of one occupied cell.
[[nodiscard]] double contactToleranceM(const double occupancy_resolution_m) noexcept {
  return std::isfinite(occupancy_resolution_m) && occupancy_resolution_m > 0.0
             ? 0.5 * occupancy_resolution_m
             : 0.0;
}

} // namespace

ProprioceptiveFreeSpaceSeed3D
proprioceptiveContactSeed3D(const Point3& position, const FootprintBodyAxis& body_axis,
                            const SweptFootprintConfig& footprint,
                            const double occupancy_resolution_m) noexcept {
  return ProprioceptiveFreeSpaceSeed3D{
      .position = position,
      .body_axis = body_axis,
      .footprint = footprint,
      .contact_tolerance_m = contactToleranceM(occupancy_resolution_m),
  };
}

ProprioceptiveFreeSpaceSeed3D
proprioceptiveContactSeed3D(const Point3& position,
                            const MotionControl3D& previous_control,
                            const SweptFootprintConfig& footprint,
                            const double occupancy_resolution_m) noexcept {
  return proprioceptiveContactSeed3D(
      position,
      bodyAxisFromWorldAcceleration(
          Vec3{previous_control.ax, previous_control.ay, previous_control.az}),
      footprint, occupancy_resolution_m);
}

std::optional<ProprioceptiveFreeSpaceSeed3D>
proprioceptiveContactSeed3D(const Point3& position, const FootprintBodyAxis& body_axis,
                            const SweptFootprintConfig& footprint,
                            const ObservedOccupancyGrid3D* const observed_occupancy) {
  if (observed_occupancy == nullptr) {
    return std::nullopt;
  }
  return proprioceptiveContactSeed3D(position, body_axis, footprint,
                                     observed_occupancy->bounds().resolution_m);
}

std::optional<ProprioceptiveFreeSpaceSeed3D>
proprioceptiveContactSeed3D(const Point3& position,
                            const MotionControl3D& previous_control,
                            const SweptFootprintConfig& footprint,
                            const ObservedOccupancyGrid3D* const observed_occupancy) {
  return proprioceptiveContactSeed3D(
      position,
      bodyAxisFromWorldAcceleration(
          Vec3{previous_control.ax, previous_control.ay, previous_control.az}),
      footprint, observed_occupancy);
}

} // namespace drone_city_nav
