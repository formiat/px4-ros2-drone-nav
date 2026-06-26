#pragma once

#include "drone_city_nav/corridor.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_diagnostics.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

struct TrajectoryStraighteningConfig {
  double min_segment_length_m{20.0};
  double validation_step_m{2.0};
  double min_corridor_margin_m{0.5};
  double max_path_length_ratio{1.01};
  double max_heading_error_rad{0.08726646259971647};
  double max_chord_deviation_m{1.0};
  double max_abs_curvature_1pm{0.0025};
  double max_edge_margin_loss_m{0.25};
};

struct TrajectoryStraighteningStats {
  std::size_t input_samples{0U};
  std::size_t output_samples{0U};
  std::size_t collapsed_segments{0U};
  std::size_t rejected_too_short{0U};
  std::size_t rejected_shape{0U};
  std::size_t rejected_prohibited{0U};
  std::size_t rejected_corridor{0U};
  std::size_t rejected_curvature{0U};
  std::size_t rejected_chord_deviation{0U};
  std::size_t rejected_edge_margin{0U};
  double max_heading_delta_before_rad{0.0};
  double max_heading_delta_after_rad{0.0};
  double max_curvature_jump_before_1pm{0.0};
  double max_curvature_jump_after_1pm{0.0};
};

struct TrajectoryStraighteningResult {
  std::vector<TrajectoryPointSample> samples;
  TrajectoryStraighteningStats stats{};
  bool changed{false};
  bool valid{false};
};

[[nodiscard]] TrajectoryStraighteningResult
straightenTrajectory(std::span<const TrajectoryPointSample> samples,
                     std::span<const CorridorSample> corridor_samples,
                     const OccupancyGrid2D& prohibited_grid,
                     const TrajectoryStraighteningConfig& config);

} // namespace drone_city_nav
