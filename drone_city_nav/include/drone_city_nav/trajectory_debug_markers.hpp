#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_speed_planner.hpp"

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <span>

namespace drone_city_nav {

[[nodiscard]] visualization_msgs::msg::MarkerArray
buildTrajectoryDebugMarkers(const std_msgs::msg::Header& header,
                            std::span<const TrajectoryPointSample> trajectory_samples,
                            const TrajectorySpeedProfile& speed_profile);

} // namespace drone_city_nav
