#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/static_city_map.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace drone_city_nav {

struct StaticMapDebugConfig {
  std_msgs::msg::Header header;
  float point_z_m{0.05F};
  float building_alpha{0.62F};
};

[[nodiscard]] nav_msgs::msg::OccupancyGrid
staticMapGridMessage(const OccupancyGrid2D& grid, const StaticMapDebugConfig& config);

[[nodiscard]] sensor_msgs::msg::PointCloud2
staticMapPointCloud(const OccupancyGrid2D& grid, const StaticMapDebugConfig& config);

[[nodiscard]] visualization_msgs::msg::MarkerArray
staticMapBuildingMarkers(const StaticCityMap& map, const StaticMapDebugConfig& config);

[[nodiscard]] visualization_msgs::msg::MarkerArray
staticMapBuildingDeleteMarkers(const std_msgs::msg::Header& header);

} // namespace drone_city_nav
