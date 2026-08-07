#include "drone_city_nav/local_esdf_2d.hpp"

#include "drone_city_nav/grid_config.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] int cellForCoordinate(const double coordinate, const double origin,
                                    const double resolution) {
  return static_cast<int>(std::floor((coordinate - origin) / resolution));
}

[[nodiscard]] bool sameResolution(const GridBounds& first,
                                  const GridBounds& second) noexcept {
  return std::abs(first.resolution_m - second.resolution_m) <= 1.0e-9;
}

} // namespace

GridBounds selectLocalEsdfBounds(const GridBounds& world_bounds, const Point2 center,
                                 const double half_extent_m) {
  if (!gridBoundsUsable(world_bounds) || !std::isfinite(center.x) ||
      !std::isfinite(center.y) || !(half_extent_m > 0.0)) {
    throw std::invalid_argument{"Invalid local ESDF window request"};
  }
  const int half_cells = std::max(
      1, static_cast<int>(std::ceil(half_extent_m / world_bounds.resolution_m)));
  const int center_x = std::clamp(
      cellForCoordinate(center.x, world_bounds.origin_x, world_bounds.resolution_m), 0,
      world_bounds.width_cells - 1);
  const int center_y = std::clamp(
      cellForCoordinate(center.y, world_bounds.origin_y, world_bounds.resolution_m), 0,
      world_bounds.height_cells - 1);
  const int minimum_x = std::max(0, center_x - half_cells);
  const int maximum_x =
      std::min(world_bounds.width_cells - 1, center_x + half_cells - 1);
  const int minimum_y = std::max(0, center_y - half_cells);
  const int maximum_y =
      std::min(world_bounds.height_cells - 1, center_y + half_cells - 1);
  return GridBounds{
      .origin_x = world_bounds.origin_x +
                  static_cast<double>(minimum_x) * world_bounds.resolution_m,
      .origin_y = world_bounds.origin_y +
                  static_cast<double>(minimum_y) * world_bounds.resolution_m,
      .resolution_m = world_bounds.resolution_m,
      .width_cells = maximum_x - minimum_x + 1,
      .height_cells = maximum_y - minimum_y + 1,
  };
}

OccupancyGrid2D cropOccupancyGrid(const OccupancyGrid2D& source,
                                  const GridBounds& local_bounds) {
  const GridBounds& world = source.bounds();
  if (!gridBoundsUsable(local_bounds) || !sameResolution(world, local_bounds)) {
    throw std::invalid_argument{"Local occupancy bounds do not match source grid"};
  }
  const int source_minimum_x =
      cellForCoordinate(local_bounds.origin_x, world.origin_x, world.resolution_m);
  const int source_minimum_y =
      cellForCoordinate(local_bounds.origin_y, world.origin_y, world.resolution_m);
  const int source_maximum_x = source_minimum_x + local_bounds.width_cells - 1;
  const int source_maximum_y = source_minimum_y + local_bounds.height_cells - 1;
  if (source_minimum_x < 0 || source_minimum_y < 0 ||
      source_maximum_x >= world.width_cells || source_maximum_y >= world.height_cells) {
    throw std::invalid_argument{"Local occupancy bounds exceed source grid"};
  }

  OccupancyGrid2D result{local_bounds};
  for (int y = 0; y < local_bounds.height_cells; ++y) {
    for (int x = 0; x < local_bounds.width_cells; ++x) {
      const GridIndex source_cell{source_minimum_x + x, source_minimum_y + y};
      const GridIndex destination_cell{x, y};
      switch (source.state(source_cell)) {
        case CellState::kUnknown:
          break;
        case CellState::kFree:
          result.setFree(destination_cell);
          break;
        case CellState::kOccupied:
          result.setOccupied(destination_cell);
          break;
      }
    }
  }
  return result;
}

bool localEsdfNeedsRecenter(const GridBounds& active_bounds,
                            const GridBounds& world_bounds, const Point2 position,
                            const double core_margin_m) noexcept {
  if (!gridBoundsUsable(active_bounds) || !gridBoundsUsable(world_bounds) ||
      !std::isfinite(position.x) || !std::isfinite(position.y) ||
      !(core_margin_m >= 0.0) || !sameResolution(active_bounds, world_bounds)) {
    return true;
  }
  const double active_maximum_x =
      active_bounds.origin_x + active_bounds.width_cells * active_bounds.resolution_m;
  const double active_maximum_y =
      active_bounds.origin_y + active_bounds.height_cells * active_bounds.resolution_m;
  const double world_maximum_x =
      world_bounds.origin_x + world_bounds.width_cells * world_bounds.resolution_m;
  const double world_maximum_y =
      world_bounds.origin_y + world_bounds.height_cells * world_bounds.resolution_m;
  if (active_bounds.origin_x < world_bounds.origin_x - 1.0e-9 ||
      active_bounds.origin_y < world_bounds.origin_y - 1.0e-9 ||
      active_maximum_x > world_maximum_x + 1.0e-9 ||
      active_maximum_y > world_maximum_y + 1.0e-9) {
    return true;
  }
  const bool near_expandable_left =
      active_bounds.origin_x > world_bounds.origin_x + 1.0e-9 &&
      position.x - active_bounds.origin_x < core_margin_m;
  const bool near_expandable_right = active_maximum_x < world_maximum_x - 1.0e-9 &&
                                     active_maximum_x - position.x < core_margin_m;
  const bool near_expandable_bottom =
      active_bounds.origin_y > world_bounds.origin_y + 1.0e-9 &&
      position.y - active_bounds.origin_y < core_margin_m;
  const bool near_expandable_top = active_maximum_y < world_maximum_y - 1.0e-9 &&
                                   active_maximum_y - position.y < core_margin_m;
  return near_expandable_left || near_expandable_right || near_expandable_bottom ||
         near_expandable_top;
}

} // namespace drone_city_nav
