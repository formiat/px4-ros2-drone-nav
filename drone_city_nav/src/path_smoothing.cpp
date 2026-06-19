#include "drone_city_nav/path_smoothing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace drone_city_nav {
namespace {

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

void recordRejectedLineOfSight(PathSmoothingStats& stats,
                               const LineOfSightCheck& check) {
  ++stats.rejected_segments;
  switch (check.reason) {
    case LineOfSightBlockReason::kClear:
      break;
    case LineOfSightBlockReason::kOutsideGrid:
      ++stats.rejected_outside_grid;
      break;
    case LineOfSightBlockReason::kProhibited:
      ++stats.rejected_prohibited;
      stats.rejected_prohibited_cells += check.prohibited_cells;
      break;
  }
}

} // namespace

LineOfSightCheck checkLineOfSight(const OccupancyGrid2D& grid, const GridIndex start,
                                  const GridIndex end) {
  LineOfSightCheck result{};
  if (!grid.contains(start) || !grid.contains(end)) {
    result.reason = LineOfSightBlockReason::kOutsideGrid;
    return result;
  }

  const auto line_cells = grid.cellsOnLine(start, end);
  result.checked_cells = line_cells.size();
  for (const GridIndex cell : line_cells) {
    if (grid.isProhibited(cell)) {
      ++result.prohibited_cells;
    }
  }
  if (result.prohibited_cells > 0U) {
    result.reason = LineOfSightBlockReason::kProhibited;
    return result;
  }

  result.clear = true;
  result.reason = LineOfSightBlockReason::kClear;
  return result;
}

bool hasLineOfSight(const OccupancyGrid2D& grid, const GridIndex start,
                    const GridIndex end) {
  return checkLineOfSight(grid, start, end).clear;
}

std::vector<GridIndex> smoothPath(const OccupancyGrid2D& grid,
                                  const std::vector<GridIndex>& path) {
  return smoothPathWithStats(grid, path).path;
}

PathSmoothingResult smoothPathWithStats(const OccupancyGrid2D& grid,
                                        const std::vector<GridIndex>& path) {
  PathSmoothingResult result{};
  result.stats.input_points = path.size();
  if (path.size() <= 2U) {
    result.path = path;
    result.stats.output_points = result.path.size();
    if (path.size() == 2U) {
      result.stats.accepted_segments = 1U;
    }
    return result;
  }

  std::vector<GridIndex> smoothed;
  smoothed.reserve(path.size());

  std::size_t anchor = 0U;
  smoothed.push_back(path.front());

  while (anchor < path.size() - 1U) {
    std::size_t next = path.size() - 1U;
    LineOfSightCheck check = checkLineOfSight(grid, path[anchor], path[next]);
    ++result.stats.line_of_sight_checks;
    while (next > anchor + 1U && !check.clear) {
      recordRejectedLineOfSight(result.stats, check);
      --next;
      check = checkLineOfSight(grid, path[anchor], path[next]);
      ++result.stats.line_of_sight_checks;
    }

    if (!check.clear) {
      recordRejectedLineOfSight(result.stats, check);
    }
    ++result.stats.accepted_segments;
    if (next > anchor + 1U) {
      ++result.stats.shortcut_segments;
    } else {
      ++result.stats.forced_adjacent_segments;
    }
    smoothed.push_back(path[next]);
    anchor = next;
  }

  result.path = std::move(smoothed);
  result.stats.output_points = result.path.size();
  return result;
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
