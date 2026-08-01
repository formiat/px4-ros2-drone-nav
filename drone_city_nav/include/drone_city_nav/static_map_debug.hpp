#pragma once

#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace drone_city_nav {

struct StaticMapDebugConfig {
  std_msgs::msg::Header header;
  float point_z_m{0.05F};
  float building_alpha{0.62F};
};

[[nodiscard]] sensor_msgs::msg::PointCloud2
staticMapPointCloud3D(const OccupancyGrid3D& grid, const StaticMapDebugConfig& config);

[[nodiscard]] visualization_msgs::msg::MarkerArray
staticMapBuildingDeleteMarkers(const std_msgs::msg::Header& header);

} // namespace drone_city_nav
