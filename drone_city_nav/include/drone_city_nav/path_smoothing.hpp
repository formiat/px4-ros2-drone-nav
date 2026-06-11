#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <vector>

namespace drone_city_nav {

[[nodiscard]] bool hasLineOfSight(const OccupancyGrid2D &grid, GridIndex start,
                                  GridIndex end);
[[nodiscard]] std::vector<GridIndex>
smoothPath(const OccupancyGrid2D &grid, const std::vector<GridIndex> &path);
[[nodiscard]] std::vector<Point2>
cellsToPoints(const OccupancyGrid2D &grid, const std::vector<GridIndex> &path);

} // namespace drone_city_nav
