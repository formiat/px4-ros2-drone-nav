#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] sensor_msgs::msg::PointCloud2
buildLidarDebugPointCloud(std::span<const Point3> points,
                          const builtin_interfaces::msg::Time& stamp,
                          std::string_view frame_id, bool gazebo_aligned_axes_swapped);

// The occupied voxels of the grid: with `unobservable` false the surfaces
// (measured hits), with it true the space observed unobservable (darkness,
// specification K14), so that RViz draws the two apart.
[[nodiscard]] sensor_msgs::msg::PointCloud2 buildObservedOccupancyPointCloud3D(
    const ObservedOccupancyGrid3D& grid, const builtin_interfaces::msg::Time& stamp,
    std::string_view frame_id, bool gazebo_aligned_axes_swapped, std::size_t stride,
    bool unobservable);

} // namespace drone_city_nav
