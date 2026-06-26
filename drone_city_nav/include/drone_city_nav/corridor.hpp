#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

struct CorridorConfig {
  double max_radius_m{40.0};
  double sample_step_m{1.0};
  double ray_step_m{0.0};
  double safety_margin_m{0.5};
  double center_recovery_max_m{3.0};
  double endpoint_anchor_distance_m{20.0};
  double lateral_limit_window_m{20.0};
  double lateral_limit_ratio{1.25};
  double lateral_limit_margin_m{1.0};
};

struct CorridorSample {
  double s_m{0.0};
  Point2 route_center{};
  Point2 center{};
  Point2 tangent{};
  Point2 normal{};
  double left_bound_m{0.0};
  double right_bound_m{0.0};
  double clearance_m{0.0};
  double center_recovery_m{0.0};
  double centering_shift_m{0.0};
};

struct CorridorStats {
  std::size_t input_points{0U};
  std::size_t samples{0U};
  std::size_t route_prohibited_samples{0U};
  std::size_t center_recovered_samples{0U};
  std::size_t center_unrecoverable_samples{0U};
  std::size_t centered_samples{0U};
  std::size_t endpoint_anchored_samples{0U};
  std::size_t outside_grid_samples{0U};
  std::size_t lateral_limited_samples{0U};
  double min_width_m{0.0};
  double mean_width_m{0.0};
  double max_width_m{0.0};
  double min_clearance_m{0.0};
  double mean_clearance_m{0.0};
  double max_clearance_m{0.0};
  double max_center_recovery_m{0.0};
  double max_centering_shift_m{0.0};
  double max_endpoint_anchor_reduction_m{0.0};
  double max_lateral_bound_reduction_m{0.0};
};

struct CorridorResult {
  std::vector<CorridorSample> samples;
  CorridorStats stats{};
  bool valid{false};
};

[[nodiscard]] CorridorResult buildCorridor(std::span<const Point2> route_points,
                                           const OccupancyGrid2D& prohibited_grid,
                                           const CorridorConfig& config);

} // namespace drone_city_nav
