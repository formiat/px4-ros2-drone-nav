#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"

#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

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

// The depth the body has in the occupied voxels it overlaps at the seed.
[[nodiscard]] double contactDepthM(const Point3& position,
                                   const FootprintBodyAxis& body_axis,
                                   const SweptFootprintConfig& footprint,
                                   const ObservedOccupancyGrid3D& occupancy) {
  const SweptFootprintConfig body = physicalBodyFootprint(footprint);
  const double reach_m =
      std::max({body.radius_m, body.lower_extent_m, body.upper_extent_m, 0.0}) *
      std::numbers::sqrt2;
  const GridBounds3D& bounds = occupancy.bounds();
  const double half_m = 0.5 * bounds.resolution_m;
  // The cells the body can reach, clamped to the grid: a body at its edge is
  // judged against the part of it the grid holds.
  const auto cell_range = [&](const double origin, const double center,
                              const int cells) {
    const auto clamp = [&](const double coordinate) {
      return static_cast<int>(
          std::clamp(std::floor((coordinate - origin) / bounds.resolution_m), 0.0,
                     static_cast<double>(std::max(cells - 1, 0))));
    };
    return std::pair{clamp(center - reach_m), clamp(center + reach_m)};
  };
  const auto [x_lower, x_upper] =
      cell_range(bounds.origin_x, position.x, bounds.width_cells);
  const auto [y_lower, y_upper] =
      cell_range(bounds.origin_y, position.y, bounds.height_cells);
  const auto [z_lower, z_upper] =
      cell_range(bounds.origin_z, position.z, bounds.depth_cells);
  double depth_m = 0.0;
  for (int x = x_lower; x <= x_upper; ++x) {
    for (int y = y_lower; y <= y_upper; ++y) {
      for (int z = z_lower; z <= z_upper; ++z) {
        const GridIndex3D index{x, y, z};
        if (!occupancy.isOccupied(index)) {
          continue;
        }
        const Point3 center = occupancy.cellCenter(index);
        depth_m = std::max(
            depth_m,
            bodyDepthInBoxM(
                position, body_axis,
                Point3{center.x - half_m, center.y - half_m, center.z - half_m},
                Point3{center.x + half_m, center.y + half_m, center.z + half_m}, body));
      }
    }
  }
  return depth_m;
}

std::optional<ProprioceptiveFreeSpaceSeed3D>
proprioceptiveContactSeed3D(const Point3& position, const FootprintBodyAxis& body_axis,
                            const SweptFootprintConfig& footprint,
                            const ObservedOccupancyGrid3D* const observed_occupancy) {
  if (observed_occupancy == nullptr) {
    return std::nullopt;
  }
  ProprioceptiveFreeSpaceSeed3D seed = proprioceptiveContactSeed3D(
      position, body_axis, footprint, observed_occupancy->bounds().resolution_m);
  seed.contact_depth_m =
      contactDepthM(position, body_axis, footprint, *observed_occupancy);
  return seed;
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
