#include "drone_city_nav/path_smoothing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] double nearestOccupiedDistanceM(const OccupancyGrid2D& grid,
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
      if (!grid.contains(candidate) || !grid.isOccupied(candidate)) {
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
  return nearestOccupiedDistanceM(grid, cell, config.minimum_obstacle_clearance_m) >=
         config.minimum_obstacle_clearance_m;
}

[[nodiscard]] double cross(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] bool pointLiesOnSegmentLine(const Point2 segment_start,
                                          const Point2 point, const Point2 segment_end,
                                          const double lateral_tolerance_m) {
  const Point2 segment = segment_end - segment_start;
  const Point2 relative_point = point - segment_start;
  const double segment_length_sq = segment.x * segment.x + segment.y * segment.y;
  if (segment_length_sq <= std::numeric_limits<double>::epsilon()) {
    return distance(segment_start, point) <= lateral_tolerance_m;
  }

  const double projection = dot(relative_point, segment);
  if (projection < 0.0 || projection > segment_length_sq) {
    return false;
  }

  const double lateral_error_m =
      std::abs(cross(relative_point, segment)) / std::sqrt(segment_length_sq);
  return lateral_error_m <= lateral_tolerance_m;
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

std::vector<Point2> collapseCollinearPath(std::span<const Point2> path_points,
                                          const double lateral_tolerance_m) {
  if (path_points.size() <= 2U) {
    return {path_points.begin(), path_points.end()};
  }

  const double tolerance_m = std::max(0.0, lateral_tolerance_m);
  std::vector<Point2> collapsed;
  collapsed.reserve(path_points.size());
  collapsed.push_back(path_points.front());

  for (std::size_t i = 1U; i + 1U < path_points.size(); ++i) {
    const Point2 previous_kept = collapsed.back();
    const Point2 current = path_points[i];
    const Point2 next = path_points[i + 1U];
    if (pointLiesOnSegmentLine(previous_kept, current, next, tolerance_m)) {
      continue;
    }
    collapsed.push_back(current);
  }

  const Point2 last = path_points.back();
  if (squaredDistance(collapsed.back(), last) >
      std::numeric_limits<double>::epsilon()) {
    collapsed.push_back(last);
  }
  return collapsed;
}

} // namespace drone_city_nav
