#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <span>
#include <vector>

namespace drone_city_nav {

struct PathSmoothingConfig {
  double minimum_obstacle_clearance_m{0.0};
};

[[nodiscard]] bool hasLineOfSight(const OccupancyGrid2D& grid, GridIndex start,
                                  GridIndex end,
                                  const PathSmoothingConfig& config = {});
[[nodiscard]] std::vector<GridIndex> smoothPath(const OccupancyGrid2D& grid,
                                                const std::vector<GridIndex>& path,
                                                const PathSmoothingConfig& config = {});
[[nodiscard]] std::vector<Point2> cellsToPoints(const OccupancyGrid2D& grid,
                                                const std::vector<GridIndex>& path);
[[nodiscard]] std::vector<Point2>
collapseCollinearPath(std::span<const Point2> path_points, double lateral_tolerance_m);

} // namespace drone_city_nav
