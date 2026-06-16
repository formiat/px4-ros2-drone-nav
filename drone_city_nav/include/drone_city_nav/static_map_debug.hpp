#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace drone_city_nav {

struct StaticMapDebugConfig {
  std_msgs::msg::Header header;
  float point_z_m{0.05F};
};

[[nodiscard]] nav_msgs::msg::OccupancyGrid
staticMapGridMessage(const OccupancyGrid2D& grid, const StaticMapDebugConfig& config);

[[nodiscard]] sensor_msgs::msg::PointCloud2
staticMapPointCloud(const OccupancyGrid2D& grid, const StaticMapDebugConfig& config);

} // namespace drone_city_nav
