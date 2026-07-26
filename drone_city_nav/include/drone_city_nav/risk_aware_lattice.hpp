#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

struct RiskAwareLatticeConfig {
  int heading_bins{16};
  double primitive_length_m{4.0};
  double primitive_sample_step_m{0.5};
  double goal_tolerance_m{5.0};
  double receding_goal_distance_m{60.0};
  double collision_radius_m{0.5};
  double critical_distance_m{1.0};
  double preferred_distance_m{6.0};
  double critical_cost_per_m{100000.0};
  double planning_cost_per_m{1000.0};
  double turn_cost{0.5};
  double heuristic_weight{2.0};
  std::size_t maximum_expansions{60000U};
};

struct RiskAwareLatticeResult {
  bool valid{false};
  bool reached_mission_goal{false};
  Point2 planning_goal{};
  std::vector<Point2> guide;
  std::size_t expansions{0U};
  double cost{0.0};
};

[[nodiscard]] RiskAwareLatticeResult
planRiskAwareMotionPrimitiveGuide(const mppi::EsdfGrid& grid,
                                  std::span<const float> esdf_m, Point2 start,
                                  double start_heading_rad, Point2 mission_goal,
                                  const RiskAwareLatticeConfig& config);

} // namespace drone_city_nav
