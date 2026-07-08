#include "drone_city_nav/lidar_debug_pointclouds.hpp"

#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstring>

namespace drone_city_nav {

[[nodiscard]] std::vector<Point2>
collectOccupancyGridPoints(const nav_msgs::msg::OccupancyGrid& grid,
                           const std::uint8_t min_value, const std::uint8_t max_value) {
  std::vector<Point2> points;
  const double resolution = static_cast<double>(grid.info.resolution);
  if (!(resolution > 0.0)) {
    return points;
  }

  const auto width = static_cast<int>(grid.info.width);
  const auto height = static_cast<int>(grid.info.height);
  const std::size_t expected_cells =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (width <= 0 || height <= 0 || grid.data.size() < expected_cells) {
    return points;
  }

  points.reserve(expected_cells / 8U);
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const std::size_t index =
          static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(x);
      const std::int8_t raw_value = grid.data[index];
      if (raw_value < 0) {
        continue;
      }
      const auto value = static_cast<std::uint8_t>(raw_value);
      if (value >= min_value && value <= max_value) {
        points.push_back(
            Point2{origin_x + (static_cast<double>(x) + 0.5) * resolution,
                   origin_y + (static_cast<double>(y) + 0.5) * resolution});
      }
    }
  }
  return points;
}

[[nodiscard]] std::vector<Point2>
collectProhibitedGridPoints(const nav_msgs::msg::OccupancyGrid& grid) {
  return collectOccupancyGridPoints(grid, 80, 99);
}

[[nodiscard]] std::vector<Point2>
collectOccupiedGridPoints(const nav_msgs::msg::OccupancyGrid& grid) {
  return collectOccupancyGridPoints(grid, 100, 100);
}

[[nodiscard]] sensor_msgs::msg::PointCloud2
buildLidarDebugPointCloud(const std::span<const Point2> points, const double z_m,
                          const builtin_interfaces::msg::Time& stamp,
                          const std::string_view frame_id) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = std::string{frame_id};
  cloud.height = 1U;
  cloud.width = static_cast<std::uint32_t>(points.size());
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

  for (std::size_t i = 0U; i < points.size(); ++i) {
    const float x = static_cast<float>(points[i].x);
    const float y = static_cast<float>(points[i].y);
    const float z =
        std::isfinite(z_m) ? static_cast<float>(gazeboAlignedRvizZ(z_m)) : 0.0F;
    const std::size_t offset = i * static_cast<std::size_t>(cloud.point_step);
    std::memcpy(&cloud.data[offset], &x, sizeof(float));
    std::memcpy(&cloud.data[offset + 4U], &y, sizeof(float));
    std::memcpy(&cloud.data[offset + 8U], &z, sizeof(float));
  }
  return cloud;
}

} // namespace drone_city_nav
