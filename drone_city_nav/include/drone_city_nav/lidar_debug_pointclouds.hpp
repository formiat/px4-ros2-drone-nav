#pragma once

#include "drone_city_nav/types.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] std::vector<Point2>
collectOccupancyGridPoints(const nav_msgs::msg::OccupancyGrid& grid,
                           std::uint8_t min_value, std::uint8_t max_value);

[[nodiscard]] std::vector<Point2>
collectProhibitedGridPoints(const nav_msgs::msg::OccupancyGrid& grid);

[[nodiscard]] std::vector<Point2>
collectOccupiedGridPoints(const nav_msgs::msg::OccupancyGrid& grid);

[[nodiscard]] sensor_msgs::msg::PointCloud2
buildLidarDebugPointCloud(std::span<const Point2> points, double z_m,
                          const builtin_interfaces::msg::Time& stamp,
                          std::string_view frame_id);

} // namespace drone_city_nav
