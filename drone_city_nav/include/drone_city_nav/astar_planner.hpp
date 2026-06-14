#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <vector>

namespace drone_city_nav {

struct AStarConfig {
  std::size_t max_expansions{100000};
  double obstacle_clearance_cost_radius_m{0.0};
  double obstacle_clearance_cost_weight{0.0};
  double turn_cost_weight{0.0};
};

struct AStarResult {
  bool success{false};
  std::size_t expanded_cells{0};
  std::vector<GridIndex> path;
};

class AStarPlanner {
public:
  [[nodiscard]] AStarResult plan(const OccupancyGrid2D& grid, GridIndex start,
                                 GridIndex goal, const AStarConfig& config = {}) const;
};

} // namespace drone_city_nav
