#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/types.hpp"

#include <limits>
#include <span>
#include <vector>

namespace drone_city_nav {

struct HorizontalTrajectoryHandoverConfig {
  bool enabled{true};
  bool require_validation_grid{true};
  double prefix_time_s{0.6};
  double min_prefix_distance_m{3.0};
  double max_prefix_distance_m{10.0};
  double candidate_lookahead_distance_m{12.0};
  double sample_step_m{0.5};
  double max_join_distance_m{15.0};
  double trigger_projection_jump_m{3.0};
  double trigger_tangent_jump_rad{0.52};
  double max_sample_heading_delta_rad{0.35};
  double max_abs_curvature_1pm{0.15};
};

struct HorizontalTrajectoryHandoverState {
  Point2 current_position{};
  double current_horizontal_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  bool current_position_valid{false};
  bool current_horizontal_speed_valid{false};
};

struct HorizontalTrajectoryHandoverResult {
  std::vector<TrajectoryPointSample> samples;
  bool applied{false};
  const char* reason{"not_applied"};
  double old_projection_s_m{std::numeric_limits<double>::quiet_NaN()};
  double old_join_s_m{std::numeric_limits<double>::quiet_NaN()};
  double candidate_projection_s_m{std::numeric_limits<double>::quiet_NaN()};
  double candidate_join_s_m{std::numeric_limits<double>::quiet_NaN()};
  double stitched_join_s_m{std::numeric_limits<double>::quiet_NaN()};
  double candidate_station_offset_m{std::numeric_limits<double>::quiet_NaN()};
  double projection_jump_m{std::numeric_limits<double>::quiet_NaN()};
  double tangent_jump_rad{std::numeric_limits<double>::quiet_NaN()};
  double join_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double prefix_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double max_sample_heading_delta_rad{0.0};
  double max_abs_curvature_1pm{0.0};
  std::size_t non_traversable_segment_index{0U};
};

[[nodiscard]] HorizontalTrajectoryHandoverResult buildHorizontalTrajectoryHandover(
    std::span<const TrajectoryPointSample> current_samples,
    std::span<const TrajectoryPointSample> candidate_samples,
    const HorizontalTrajectoryHandoverState& state,
    const HorizontalTrajectoryHandoverConfig& config = {},
    const OccupancyGrid2D* validation_grid = nullptr);

} // namespace drone_city_nav
