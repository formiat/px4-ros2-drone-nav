#pragma once

#include "drone_city_nav/corridor.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_diagnostics.hpp"

#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace drone_city_nav {

struct TurnSmoothingConfig {
  double trigger_heading_delta_rad{0.65};
  double trigger_min_radius_m{12.0};
  double entry_distance_m{45.0};
  double exit_distance_m{45.0};
  double sample_step_m{1.0};
  double outer_bias_ratio{0.45};
  double min_outer_shift_m{2.0};
  double max_outer_shift_m{12.0};
  double min_corridor_margin_m{0.5};
  double max_length_ratio{1.25};
  double min_heading_improvement_rad{0.05};
  std::size_t max_passes{8U};
};

struct TurnSmoothingStats {
  std::size_t input_samples{0U};
  std::size_t output_samples{0U};
  std::size_t detected_corners{0U};
  std::size_t attempted_corners{0U};
  std::size_t candidate_attempts{0U};
  std::size_t relaxed_candidate_attempts{0U};
  std::size_t smoothed_corners{0U};
  std::size_t rejected_prohibited{0U};
  std::size_t rejected_corridor{0U};
  std::size_t rejected_length{0U};
  std::size_t rejected_not_improved{0U};
  double max_heading_delta_before_rad{0.0};
  double max_heading_delta_after_rad{0.0};
  double max_curvature_jump_before_1pm{0.0};
  double max_curvature_jump_after_1pm{0.0};
  double min_inner_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double max_applied_outer_shift_m{0.0};
  double accepted_entry_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double accepted_exit_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double accepted_shift_scale{std::numeric_limits<double>::quiet_NaN()};
  double accepted_relaxed_angle_deg{std::numeric_limits<double>::quiet_NaN()};
};

struct TurnSmoothingResult {
  std::vector<TrajectoryPointSample> samples;
  TurnSmoothingStats stats{};
  bool changed{false};
  bool valid{false};
};

[[nodiscard]] TurnSmoothingResult
smoothTrajectoryTurns(std::span<const TrajectoryPointSample> samples,
                      std::span<const CorridorSample> corridor_samples,
                      const OccupancyGrid2D& prohibited_grid,
                      const TurnSmoothingConfig& config);

} // namespace drone_city_nav
