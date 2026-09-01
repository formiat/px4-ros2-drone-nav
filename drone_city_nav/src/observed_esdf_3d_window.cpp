#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool sameResolution(const GridBounds3D& first,
                                  const GridBounds3D& second) noexcept {
  return std::abs(first.resolution_m - second.resolution_m) <= 1.0e-9;
}

[[nodiscard]] int clampedCell(const double coordinate, const double origin,
                              const double resolution_m,
                              const int cell_count) noexcept {
  return std::clamp(static_cast<int>(std::floor((coordinate - origin) / resolution_m)),
                    0, cell_count - 1);
}

} // namespace

GridBounds3D selectLocalObservedEsdfBounds(const GridBounds3D& world_bounds,
                                           const Point3& position,
                                           const LocalObservedEsdfWindow3D& window) {
  if (!(world_bounds.resolution_m > 0.0) || world_bounds.width_cells <= 0 ||
      world_bounds.height_cells <= 0 || world_bounds.depth_cells <= 0 ||
      !localObservedEsdfWindow3DIsValid(window) || !std::isfinite(position.x) ||
      !std::isfinite(position.y) || !std::isfinite(position.z)) {
    throw std::invalid_argument{"invalid local observed ESDF bounds request"};
  }
  const int minimum_x =
      clampedCell(position.x - window.horizontal_half_extent_m, world_bounds.origin_x,
                  world_bounds.resolution_m, world_bounds.width_cells);
  const int maximum_x =
      clampedCell(position.x + window.horizontal_half_extent_m, world_bounds.origin_x,
                  world_bounds.resolution_m, world_bounds.width_cells);
  const int minimum_y =
      clampedCell(position.y - window.horizontal_half_extent_m, world_bounds.origin_y,
                  world_bounds.resolution_m, world_bounds.height_cells);
  const int maximum_y =
      clampedCell(position.y + window.horizontal_half_extent_m, world_bounds.origin_y,
                  world_bounds.resolution_m, world_bounds.height_cells);
  const int minimum_z =
      clampedCell(position.z - window.vertical_half_extent_m, world_bounds.origin_z,
                  world_bounds.resolution_m, world_bounds.depth_cells);
  const int maximum_z =
      clampedCell(position.z + window.vertical_half_extent_m, world_bounds.origin_z,
                  world_bounds.resolution_m, world_bounds.depth_cells);
  return GridBounds3D{
      .origin_x = world_bounds.origin_x +
                  static_cast<double>(minimum_x) * world_bounds.resolution_m,
      .origin_y = world_bounds.origin_y +
                  static_cast<double>(minimum_y) * world_bounds.resolution_m,
      .origin_z = world_bounds.origin_z +
                  static_cast<double>(minimum_z) * world_bounds.resolution_m,
      .resolution_m = world_bounds.resolution_m,
      .width_cells = maximum_x - minimum_x + 1,
      .height_cells = maximum_y - minimum_y + 1,
      .depth_cells = maximum_z - minimum_z + 1,
  };
}

bool localObservedEsdfNeedsRecenter(const GridBounds3D& local_bounds,
                                    const GridBounds3D& world_bounds,
                                    const Point3& position,
                                    const LocalObservedEsdfWindow3D& window) noexcept {
  if (!sameResolution(local_bounds, world_bounds) ||
      !localObservedEsdfWindow3DIsValid(window) || !std::isfinite(position.x) ||
      !std::isfinite(position.y) || !std::isfinite(position.z)) {
    return true;
  }
  const double local_maximum_x =
      local_bounds.origin_x + local_bounds.width_cells * local_bounds.resolution_m;
  const double local_maximum_y =
      local_bounds.origin_y + local_bounds.height_cells * local_bounds.resolution_m;
  const double local_maximum_z =
      local_bounds.origin_z + local_bounds.depth_cells * local_bounds.resolution_m;
  const double world_maximum_x =
      world_bounds.origin_x + world_bounds.width_cells * world_bounds.resolution_m;
  const double world_maximum_y =
      world_bounds.origin_y + world_bounds.height_cells * world_bounds.resolution_m;
  const double world_maximum_z =
      world_bounds.origin_z + world_bounds.depth_cells * world_bounds.resolution_m;
  const bool room_left = local_bounds.origin_x > world_bounds.origin_x + 1.0e-9;
  const bool room_right = local_maximum_x < world_maximum_x - 1.0e-9;
  const bool room_down = local_bounds.origin_y > world_bounds.origin_y + 1.0e-9;
  const bool room_up = local_maximum_y < world_maximum_y - 1.0e-9;
  const bool room_below = local_bounds.origin_z > world_bounds.origin_z + 1.0e-9;
  const bool room_above = local_maximum_z < world_maximum_z - 1.0e-9;
  return (room_left &&
          position.x - local_bounds.origin_x < window.horizontal_recenter_margin_m) ||
         (room_right &&
          local_maximum_x - position.x < window.horizontal_recenter_margin_m) ||
         (room_down &&
          position.y - local_bounds.origin_y < window.horizontal_recenter_margin_m) ||
         (room_up &&
          local_maximum_y - position.y < window.horizontal_recenter_margin_m) ||
         (room_below &&
          position.z - local_bounds.origin_z < window.vertical_recenter_margin_m) ||
         (room_above &&
          local_maximum_z - position.z < window.vertical_recenter_margin_m);
}

bool localObservedEsdfWindow3DIsValid(
    const LocalObservedEsdfWindow3D& window) noexcept {
  return std::isfinite(window.horizontal_half_extent_m) &&
         window.horizontal_half_extent_m > 0.0 &&
         std::isfinite(window.vertical_half_extent_m) &&
         window.vertical_half_extent_m > 0.0 &&
         std::isfinite(window.horizontal_recenter_margin_m) &&
         window.horizontal_recenter_margin_m >= 0.0 &&
         window.horizontal_recenter_margin_m < window.horizontal_half_extent_m &&
         std::isfinite(window.vertical_recenter_margin_m) &&
         window.vertical_recenter_margin_m >= 0.0 &&
         window.vertical_recenter_margin_m < window.vertical_half_extent_m;
}

} // namespace drone_city_nav
