#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <vector>

namespace drone_city_nav {

struct AStarConfig {
  double heuristic_weight{1.0};
  double turn_cost_weight{0.0};
  bool evasive_maneuvering_enabled{false};
  double evasive_maneuvering_straight_cost_weight{1.0};
  bool initial_heading_bias_enabled{false};
  double initial_heading_bias_min_speed_mps{0.5};
  double initial_heading_bias_weight{50.0};
  double initial_heading_bias_velocity_x_mps{0.0};
  double initial_heading_bias_velocity_y_mps{0.0};
};

enum class AStarStatus {
  kSuccess,
  kInvalidStartOrGoal,
  kProhibitedStartOrGoal,
  kUnreachable,
  kStateSpaceTooLarge,
};

struct AStarResult {
  bool success{false};
  AStarStatus status{AStarStatus::kUnreachable};
  std::size_t expanded_cells{0};
  double total_cost{0.0};
  std::vector<GridIndex> path;
};

[[nodiscard]] const char* astarStatusName(AStarStatus status) noexcept;

class AStarPlanner {
public:
  [[nodiscard]] AStarResult plan(const OccupancyGrid2D& grid, GridIndex start,
                                 GridIndex goal, const AStarConfig& config = {}) const;
};

} // namespace drone_city_nav
