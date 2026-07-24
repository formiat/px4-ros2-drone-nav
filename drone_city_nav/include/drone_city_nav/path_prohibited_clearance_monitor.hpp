#pragma once

#include "drone_city_nav/clearance_field.hpp"

#include <cstddef>
#include <limits>
#include <span>

namespace drone_city_nav {

struct PathProhibitedClearanceMonitorConfig {
  double trigger_clearance_m{5.0};
  double arm_clearance_m{5.5};
  double min_violation_length_m{2.0};
  double sample_step_m{0.5};
};

struct PathProhibitedClearanceViolation {
  bool detected{false};
  double entry_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double length_m{0.0};
  double min_clearance_m{std::numeric_limits<double>::infinity()};
  Point2 entry_point{};
  GridIndex nearest_prohibited_cell{};
  Point2 nearest_prohibited_cell_center{};
  bool nearest_prohibited_cell_available{false};
};

struct PathProhibitedClearanceEvaluation {
  bool valid{false};
  double current_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  bool current_position_arms{false};
  PathProhibitedClearanceViolation violation{};
};

[[nodiscard]] PathProhibitedClearanceEvaluation
evaluatePathProhibitedClearance(const OccupancyGrid2D& prohibited_grid,
                                const ClearanceField2D& prohibited_clearance,
                                std::span<const Point2> remaining_path,
                                const PathProhibitedClearanceMonitorConfig& config);

} // namespace drone_city_nav
