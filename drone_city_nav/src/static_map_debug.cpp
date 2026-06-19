#include "drone_city_nav/static_map_debug.hpp"

#include "drone_city_nav/ros_conversions.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <cstdint>
#include <cstring>

namespace drone_city_nav {

nav_msgs::msg::OccupancyGrid staticMapGridMessage(const OccupancyGrid2D& grid,
                                                  const StaticMapDebugConfig& config) {
  return rawOccupancyGridToRos(grid, RawOccupancyGridToRosConfig{config.header});
}

sensor_msgs::msg::PointCloud2 staticMapPointCloud(const OccupancyGrid2D& grid,
                                                  const StaticMapDebugConfig& config) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = config.header;
  cloud.height = 1U;
  cloud.width = 0U;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12U;
  cloud.row_step = 0U;
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

  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex cell{x, y};
      if (!grid.isOccupied(cell)) {
        continue;
      }

      const Point2 center = grid.cellCenter(cell);
      const float point_x = static_cast<float>(center.x);
      const float point_y = static_cast<float>(center.y);
      const std::size_t offset = cloud.data.size();
      cloud.data.resize(offset + static_cast<std::size_t>(cloud.point_step));
      std::memcpy(&cloud.data[offset], &point_x, sizeof(float));
      std::memcpy(&cloud.data[offset + 4U], &point_y, sizeof(float));
      std::memcpy(&cloud.data[offset + 8U], &config.point_z_m, sizeof(float));
      ++cloud.width;
    }
  }

  cloud.row_step = cloud.point_step * cloud.width;
  return cloud;
}

} // namespace drone_city_nav
