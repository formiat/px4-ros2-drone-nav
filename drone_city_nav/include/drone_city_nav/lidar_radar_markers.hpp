#pragma once

#include "drone_city_nav/lidar_projection.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <span>
#include <string>

namespace drone_city_nav {

struct LidarRadarMarkerConfig {
  builtin_interfaces::msg::Time stamp{};
  std::string frame_id{"map"};
  Point2 drone_position{};
  Point2 heading_direction{1.0, 0.0};
  double scan_range_max_m{35.0};
  double marker_z_m{0.4};
};

[[nodiscard]] visualization_msgs::msg::MarkerArray
buildLidarRadarMarkers(const LidarRadarMarkerConfig& config,
                       std::span<const LidarBeamProjection> projections);

} // namespace drone_city_nav
