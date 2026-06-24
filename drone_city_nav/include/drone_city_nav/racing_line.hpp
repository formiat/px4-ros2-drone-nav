#pragma once

#include "drone_city_nav/corridor.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

struct RacingLineConfig {
  bool enabled{true};
  std::size_t max_iterations{80U};
  double initial_offset_step_m{2.0};
  double min_offset_step_m{0.1};
  double cooling_ratio{0.5};
  double weight_length{1.0};
  double weight_curvature{25.0};
  double weight_curvature_change{10.0};
  double weight_offset_change{1.0};
  double weight_offset_second_change{10.0};
  double weight_center_bias{0.02};
  double max_length_ratio{1.25};
};

struct RacingLineStats {
  std::size_t input_samples{0U};
  std::size_t output_samples{0U};
  std::size_t iterations{0U};
  std::size_t candidate_evaluations{0U};
  std::size_t collision_rejections{0U};
  double initial_cost{0.0};
  double final_cost{0.0};
  double centerline_length_m{0.0};
  double final_length_m{0.0};
  double max_abs_offset_m{0.0};
  double max_abs_curvature_1pm{0.0};
  double mean_abs_curvature_1pm{0.0};
};

struct RacingLineResult {
  std::vector<TrajectoryPointSample> samples;
  RacingLineStats stats{};
  bool valid{false};
};

[[nodiscard]] RacingLineResult
optimizeRacingLine(std::span<const CorridorSample> corridor_samples,
                   const OccupancyGrid2D& prohibited_grid,
                   const RacingLineConfig& config);

} // namespace drone_city_nav
