#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/obstacle_memory.hpp"
#include "drone_city_nav/types.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <span>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] std::vector<Point2>
collectOccupancyGridPoints(const nav_msgs::msg::OccupancyGrid& grid,
                           std::uint8_t min_value, std::uint8_t max_value);

[[nodiscard]] std::vector<Point2>
collectOccupiedGridPoints(const nav_msgs::msg::OccupancyGrid& grid);

// `gazebo_aligned_axes_swapped` selects the RViz `gazebo_map` overlay
// convention; see visualization_marker_helpers.hpp.
[[nodiscard]] sensor_msgs::msg::PointCloud2
buildLidarDebugPointCloud(std::span<const Point2> points, double z_m,
                          const builtin_interfaces::msg::Time& stamp,
                          std::string_view frame_id, bool gazebo_aligned_axes_swapped);

[[nodiscard]] sensor_msgs::msg::PointCloud2
buildLidarDebugPointCloud(std::span<const Point3> points,
                          const builtin_interfaces::msg::Time& stamp,
                          std::string_view frame_id, bool gazebo_aligned_axes_swapped);

[[nodiscard]] sensor_msgs::msg::PointCloud2 buildObservedOccupancyPointCloud3D(
    const ObservedOccupancyGrid3D& grid, const builtin_interfaces::msg::Time& stamp,
    std::string_view frame_id, bool gazebo_aligned_axes_swapped,
    std::size_t stride = 1U);

[[nodiscard]] sensor_msgs::msg::PointCloud2 buildObstacleMemoryTriggerPointCloud(
    const std::unordered_map<std::size_t, MemoryCellProvenance>& active_provenance,
    const builtin_interfaces::msg::Time& stamp, std::string_view frame_id,
    bool gazebo_aligned_axes_swapped);

} // namespace drone_city_nav
