#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_speed_planner.hpp"
#include "drone_city_nav/types.hpp"

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <limits>
#include <span>

namespace drone_city_nav {

struct DroneDebugMarkerState {
  bool pose_fresh{false};
  Point2 position{};
  double altitude_m{std::numeric_limits<double>::quiet_NaN()};
  bool altitude_valid{false};
  double heading_rad{0.0};
};

[[nodiscard]] visualization_msgs::msg::MarkerArray
buildDroneDebugMarkers(const std_msgs::msg::Header& header,
                       const DroneDebugMarkerState& state);

[[nodiscard]] visualization_msgs::msg::MarkerArray
buildOffboardDebugMarkers(const std_msgs::msg::Header& header,
                          const DroneDebugMarkerState& state,
                          std::span<const TrajectoryPointSample> trajectory_samples,
                          const TrajectorySpeedProfile& speed_profile);

} // namespace drone_city_nav
