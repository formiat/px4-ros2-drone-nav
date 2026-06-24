#pragma once

#include "drone_city_nav/trajectory.hpp"

#include <cstddef>
#include <limits>
#include <span>

namespace drone_city_nav {

struct TrajectoryShapeDiagnostics {
  std::size_t segment_count{0U};
  std::size_t segments_shorter_than_0_5m{0U};
  std::size_t segments_shorter_than_1m{0U};
  std::size_t segments_shorter_than_2m{0U};
  std::size_t max_heading_delta_index{0U};
  std::size_t max_curvature_jump_index{0U};
  std::size_t max_offset_delta_index{0U};
  std::size_t max_offset_second_delta_index{0U};
  double min_segment_length_m{std::numeric_limits<double>::quiet_NaN()};
  double mean_segment_length_m{std::numeric_limits<double>::quiet_NaN()};
  double max_segment_length_m{0.0};
  double max_heading_delta_rad{0.0};
  double max_curvature_jump_1pm{0.0};
  double max_offset_delta_m{0.0};
  double max_offset_second_delta_m{0.0};
  Point2 max_heading_delta_point{};
  Point2 max_curvature_jump_point{};
  Point2 max_offset_delta_point{};
  Point2 max_offset_second_delta_point{};
};

[[nodiscard]] TrajectoryShapeDiagnostics
computeTrajectoryShapeDiagnostics(std::span<const TrajectoryPointSample> samples);

} // namespace drone_city_nav
