#include "drone_city_nav/path_smoothing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] double nearestBlockedDistanceM(const OccupancyGrid2D& grid,
                                             const GridIndex cell,
                                             const double max_distance_m) {
  if (!(max_distance_m > 0.0) || !(grid.resolution() > 0.0)) {
    return std::numeric_limits<double>::infinity();
  }

  const int radius_cells =
      static_cast<int>(std::ceil(max_distance_m / grid.resolution()));
  double nearest_distance_m = std::numeric_limits<double>::infinity();
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const GridIndex candidate{cell.x + dx, cell.y + dy};
      if (!grid.contains(candidate) || !grid.isBlocked(candidate)) {
        continue;
      }
      nearest_distance_m =
          std::min(nearest_distance_m,
                   distance(grid.cellCenter(cell), grid.cellCenter(candidate)));
    }
  }

  return nearest_distance_m;
}

[[nodiscard]] bool cellHasRequiredClearance(const OccupancyGrid2D& grid,
                                            const GridIndex cell,
                                            const PathSmoothingConfig& config) {
  if (!(config.minimum_obstacle_clearance_m > 0.0)) {
    return true;
  }
  return nearestBlockedDistanceM(grid, cell, config.minimum_obstacle_clearance_m) >=
         config.minimum_obstacle_clearance_m;
}

} // namespace

bool hasLineOfSight(const OccupancyGrid2D& grid, const GridIndex start,
                    const GridIndex end, const PathSmoothingConfig& config) {
  if (!grid.contains(start) || !grid.contains(end)) {
    return false;
  }

  const auto line_cells = grid.cellsOnLine(start, end);
  return std::ranges::none_of(line_cells, [&grid, &config](const GridIndex cell) {
    return grid.isBlocked(cell) || !cellHasRequiredClearance(grid, cell, config);
  });
}

std::vector<GridIndex> smoothPath(const OccupancyGrid2D& grid,
                                  const std::vector<GridIndex>& path,
                                  const PathSmoothingConfig& config) {
  if (path.size() <= 2U) {
    return path;
  }

  std::vector<GridIndex> smoothed;
  smoothed.reserve(path.size());

  std::size_t anchor = 0U;
  smoothed.push_back(path.front());

  while (anchor < path.size() - 1U) {
    std::size_t next = path.size() - 1U;
    while (next > anchor + 1U &&
           !hasLineOfSight(grid, path[anchor], path[next], config)) {
      --next;
    }

    smoothed.push_back(path[next]);
    anchor = next;
  }

  return smoothed;
}

std::vector<Point2> cellsToPoints(const OccupancyGrid2D& grid,
                                  const std::vector<GridIndex>& path) {
  std::vector<Point2> points;
  points.reserve(path.size());
  for (const GridIndex cell : path) {
    points.push_back(grid.cellCenter(cell));
  }
  return points;
}

} // namespace drone_city_nav
