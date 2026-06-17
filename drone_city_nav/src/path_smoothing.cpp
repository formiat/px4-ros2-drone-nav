#include "drone_city_nav/path_smoothing.hpp"

#include "drone_city_nav/clearance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] ClearanceField2D
buildSmoothingClearanceField(const OccupancyGrid2D& grid,
                             const PathSmoothingConfig& config) {
  return ClearanceField2D::build(grid,
                                 std::max(0.0, config.minimum_obstacle_clearance_m),
                                 ClearanceSource::kOccupied);
}

[[nodiscard]] bool cellHasRequiredClearance(const ClearanceField2D& clearance_field,
                                            const GridIndex cell,
                                            const PathSmoothingConfig& config) {
  if (!(config.minimum_obstacle_clearance_m > 0.0)) {
    return true;
  }
  return clearance_field.distanceAt(cell) >= config.minimum_obstacle_clearance_m;
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

[[nodiscard]] bool hasLineOfSight(const OccupancyGrid2D& grid,
                                  const ClearanceField2D& clearance_field,
                                  const GridIndex start, const GridIndex end,
                                  const PathSmoothingConfig& config) {
  if (!grid.contains(start) || !grid.contains(end)) {
    return false;
  }

  const auto line_cells = grid.cellsOnLine(start, end);
  return std::ranges::none_of(
      line_cells, [&grid, &clearance_field, &config](const GridIndex cell) {
        return grid.isBlocked(cell) ||
               !cellHasRequiredClearance(clearance_field, cell, config);
      });
}

} // namespace

bool hasLineOfSight(const OccupancyGrid2D& grid, const GridIndex start,
                    const GridIndex end, const PathSmoothingConfig& config) {
  const ClearanceField2D clearance_field = buildSmoothingClearanceField(grid, config);
  return hasLineOfSight(grid, clearance_field, start, end, config);
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
  const ClearanceField2D clearance_field = buildSmoothingClearanceField(grid, config);

  while (anchor < path.size() - 1U) {
    std::size_t next = path.size() - 1U;
    while (next > anchor + 1U &&
           !hasLineOfSight(grid, clearance_field, path[anchor], path[next], config)) {
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
