#include "drone_city_nav/path_smoothing.hpp"

#include <algorithm>

namespace drone_city_nav {

bool hasLineOfSight(const OccupancyGrid2D& grid, const GridIndex start,
                    const GridIndex end) {
  if (!grid.contains(start) || !grid.contains(end)) {
    return false;
  }

  const auto line_cells = grid.cellsOnLine(start, end);
  return std::ranges::none_of(
      line_cells, [&grid](const GridIndex cell) { return grid.isBlocked(cell); });
}

std::vector<GridIndex> smoothPath(const OccupancyGrid2D& grid,
                                  const std::vector<GridIndex>& path) {
  if (path.size() <= 2U) {
    return path;
  }

  std::vector<GridIndex> smoothed;
  smoothed.reserve(path.size());

  std::size_t anchor = 0U;
  smoothed.push_back(path.front());

  while (anchor < path.size() - 1U) {
    std::size_t next = path.size() - 1U;
    while (next > anchor + 1U && !hasLineOfSight(grid, path[anchor], path[next])) {
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
