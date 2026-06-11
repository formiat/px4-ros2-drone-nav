#include "drone_city_nav/path_smoothing.hpp"

namespace drone_city_nav {

bool hasLineOfSight(const OccupancyGrid2D &grid, const GridIndex start,
                    const GridIndex end) {
  if (!grid.contains(start) || !grid.contains(end)) {
    return false;
  }

  for (const GridIndex cell : grid.cellsOnLine(start, end)) {
    if (grid.isBlocked(cell)) {
      return false;
    }
  }

  return true;
}

std::vector<GridIndex> smoothPath(const OccupancyGrid2D &grid,
                                  const std::vector<GridIndex> &path) {
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
           !hasLineOfSight(grid, path[anchor], path[next])) {
      --next;
    }

    smoothed.push_back(path[next]);
    anchor = next;
  }

  return smoothed;
}

std::vector<Point2> cellsToPoints(const OccupancyGrid2D &grid,
                                  const std::vector<GridIndex> &path) {
  std::vector<Point2> points;
  points.reserve(path.size());
  for (const GridIndex cell : path) {
    points.push_back(grid.cellCenter(cell));
  }
  return points;
}

} // namespace drone_city_nav
