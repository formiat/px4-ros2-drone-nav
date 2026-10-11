#include "drone_city_nav/lidar_debug_pointclouds.hpp"

#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] sensor_msgs::msg::PointCloud2
makeXyzPointCloud(const std::size_t point_count,
                  const builtin_interfaces::msg::Time& stamp,
                  const std::string_view frame_id) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = std::string{frame_id};
  cloud.height = 1U;
  cloud.width = static_cast<std::uint32_t>(point_count);
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12U;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(3U);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0U;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1U;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4U;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1U;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8U;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1U;
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
  return cloud;
}

void writeXyzPoint(sensor_msgs::msg::PointCloud2& cloud, const std::size_t index,
                   const Point3& point, const bool gazebo_aligned_axes_swapped) {
  const float x = static_cast<float>(point.x);
  const float y = static_cast<float>(point.y);
  const float z =
      static_cast<float>(gazeboAlignedRvizZ(point.z, gazebo_aligned_axes_swapped));
  const std::size_t offset = index * static_cast<std::size_t>(cloud.point_step);
  std::memcpy(&cloud.data[offset], &x, sizeof(float));
  std::memcpy(&cloud.data[offset + 4U], &y, sizeof(float));
  std::memcpy(&cloud.data[offset + 8U], &z, sizeof(float));
}

} // namespace

[[nodiscard]] sensor_msgs::msg::PointCloud2 buildLidarDebugPointCloud(
    const std::span<const Point3> points, const builtin_interfaces::msg::Time& stamp,
    const std::string_view frame_id, const bool gazebo_aligned_axes_swapped) {
  sensor_msgs::msg::PointCloud2 cloud =
      makeXyzPointCloud(points.size(), stamp, frame_id);
  for (std::size_t i = 0U; i < points.size(); ++i) {
    writeXyzPoint(cloud, i, points[i], gazebo_aligned_axes_swapped);
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 buildObservedOccupancyPointCloud3D(
    const ObservedOccupancyGrid3D& grid, const builtin_interfaces::msg::Time& stamp,
    const std::string_view frame_id, const bool gazebo_aligned_axes_swapped,
    const std::size_t stride, const bool unobservable) {
  const std::size_t effective_stride = std::max<std::size_t>(1U, stride);
  std::vector<Point3> points;
  points.reserve(grid.occupiedVoxelCount() / effective_stride + 1U);
  std::size_t occupied_index{0U};
  for (const auto& [chunk_index, storage] : grid.chunks()) {
    const ObservedOccupancyGrid3D::Chunk& chunk = storage.get();
    for (std::size_t word_index = 0U; word_index < chunk.occupied.size();
         ++word_index) {
      std::uint64_t word = unobservable ? chunk.unobservable.at(word_index)
                                        : chunk.occupied.at(word_index) &
                                              ~chunk.unobservable.at(word_index);
      while (word != 0U) {
        const std::size_t bit_offset = static_cast<std::size_t>(std::countr_zero(word));
        const std::size_t local_index = word_index * 64U + bit_offset;
        if (occupied_index % effective_stride == 0U) {
          const int local_x =
              static_cast<int>(local_index % ObservedOccupancyGrid3D::kChunkSize);
          const int local_y =
              static_cast<int>((local_index / ObservedOccupancyGrid3D::kChunkSize) %
                               ObservedOccupancyGrid3D::kChunkSize);
          const int local_z =
              static_cast<int>(local_index / static_cast<std::size_t>(
                                                 ObservedOccupancyGrid3D::kChunkSize *
                                                 ObservedOccupancyGrid3D::kChunkSize));
          const GridIndex3D index{
              chunk_index.x * ObservedOccupancyGrid3D::kChunkSize + local_x,
              chunk_index.y * ObservedOccupancyGrid3D::kChunkSize + local_y,
              chunk_index.z * ObservedOccupancyGrid3D::kChunkSize + local_z};
          if (grid.contains(index)) {
            points.push_back(grid.cellCenter(index));
          }
        }
        ++occupied_index;
        word &= word - 1U;
      }
    }
  }
  return buildLidarDebugPointCloud(points, stamp, frame_id,
                                   gazebo_aligned_axes_swapped);
}

} // namespace drone_city_nav
