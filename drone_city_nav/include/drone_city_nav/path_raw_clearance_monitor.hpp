#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <limits>
#include <span>

namespace drone_city_nav {

struct PathRawClearanceMonitorConfig {
  double trigger_clearance_m{5.0};
  double arm_clearance_m{5.5};
  double min_violation_length_m{2.0};
  double sample_step_m{0.5};
};

struct PathRawClearanceViolation {
  bool detected{false};
  double entry_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double length_m{0.0};
  double min_clearance_m{std::numeric_limits<double>::infinity()};
  Point2 entry_point{};
  GridIndex nearest_raw_cell{};
  Point2 nearest_raw_cell_center{};
  bool nearest_raw_cell_available{false};
};

struct PathRawClearanceEvaluation {
  bool valid{false};
  double current_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  bool current_position_arms{false};
  PathRawClearanceViolation violation{};
};

[[nodiscard]] PathRawClearanceEvaluation
evaluatePathRawClearance(const OccupancyGrid2D& raw_grid,
                         std::span<const Point2> remaining_path,
                         const PathRawClearanceMonitorConfig& config);

} // namespace drone_city_nav
