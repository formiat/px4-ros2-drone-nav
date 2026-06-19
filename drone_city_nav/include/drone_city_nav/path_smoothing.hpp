#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class LineOfSightBlockReason {
  kClear,
  kOutsideGrid,
  kProhibited,
};

struct LineOfSightCheck {
  bool clear{false};
  LineOfSightBlockReason reason{LineOfSightBlockReason::kOutsideGrid};
  std::size_t checked_cells{0U};
  std::size_t prohibited_cells{0U};
};

struct PathSmoothingStats {
  std::size_t input_points{0U};
  std::size_t output_points{0U};
  std::size_t line_of_sight_checks{0U};
  std::size_t accepted_segments{0U};
  std::size_t shortcut_segments{0U};
  std::size_t forced_adjacent_segments{0U};
  std::size_t rejected_segments{0U};
  std::size_t rejected_outside_grid{0U};
  std::size_t rejected_prohibited{0U};
  std::size_t rejected_prohibited_cells{0U};
};

struct PathSmoothingResult {
  std::vector<GridIndex> path;
  PathSmoothingStats stats{};
};

[[nodiscard]] bool hasLineOfSight(const OccupancyGrid2D& grid, GridIndex start,
                                  GridIndex end);
[[nodiscard]] LineOfSightCheck checkLineOfSight(const OccupancyGrid2D& grid,
                                                GridIndex start, GridIndex end);
[[nodiscard]] std::vector<GridIndex> smoothPath(const OccupancyGrid2D& grid,
                                                const std::vector<GridIndex>& path);
[[nodiscard]] PathSmoothingResult
smoothPathWithStats(const OccupancyGrid2D& grid, const std::vector<GridIndex>& path);
[[nodiscard]] std::vector<Point2> cellsToPoints(const OccupancyGrid2D& grid,
                                                const std::vector<GridIndex>& path);
[[nodiscard]] std::vector<Point2>
collapseCollinearPath(std::span<const Point2> path_points, double lateral_tolerance_m);

} // namespace drone_city_nav
