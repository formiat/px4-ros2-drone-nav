#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

struct CornerRoundingConfig {
  bool enabled{true};
  double min_radius_m{3.0};
  double max_radius_m{30.0};
  double min_segment_remainder_m{1.0};
  double collision_sample_step_m{0.25};
};

struct CornerRoundingStats {
  std::size_t input_points{0U};
  std::size_t output_segments{0U};
  std::size_t corners_seen{0U};
  std::size_t corners_rounded{0U};
  std::size_t skipped_straight{0U};
  std::size_t skipped_short_segments{0U};
  std::size_t skipped_collision{0U};
  std::size_t skipped_degenerate{0U};
  double min_radius_m{0.0};
  double max_radius_m{0.0};
  double mean_radius_m{0.0};
};

struct CornerRoundingResult {
  std::vector<TrajectorySegment> segments;
  CornerRoundingStats stats{};
};

[[nodiscard]] CornerRoundingResult
roundCorners(std::span<const Point2> path_points, const CornerRoundingConfig& config,
             const OccupancyGrid2D* prohibited_grid = nullptr);

} // namespace drone_city_nav
