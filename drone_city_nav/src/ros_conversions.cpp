#include "drone_city_nav/ros_conversions.hpp"

#include <cstdint>
#include <limits>
#include <utility>

namespace drone_city_nav {

OccupancyGridFromRosResult
occupancyGridFromRos(const nav_msgs::msg::OccupancyGrid& msg,
                     const OccupancyGridFromRosConfig& config) {
  OccupancyGridFromRosResult result{};
  if (!(msg.info.resolution > 0.0F) || msg.info.width == 0U || msg.info.height == 0U ||
      msg.info.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      msg.info.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    result.error = OccupancyGridFromRosError::kInvalidMetadata;
    return result;
  }

  const auto width = static_cast<int>(msg.info.width);
  const auto height = static_cast<int>(msg.info.height);
  result.expected_data_size =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  result.actual_data_size = msg.data.size();
  if (result.actual_data_size != result.expected_data_size) {
    result.error = OccupancyGridFromRosError::kMismatchedDataSize;
    return result;
  }

  OccupancyGrid2D grid{
      GridBounds{msg.info.origin.position.x, msg.info.origin.position.y,
                 static_cast<double>(msg.info.resolution), width, height}};
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const GridIndex cell{x, y};
      const std::size_t index = grid.linearIndex(cell);
      const auto raw_value = msg.data[index];
      if (raw_value < 0) {
        continue;
      }
      const int value = static_cast<int>(static_cast<unsigned char>(raw_value));
      if (value >= config.occupied_threshold) {
        grid.setOccupied(cell);
      } else if (value >= config.free_threshold) {
        grid.setFree(cell);
      }
    }
  }

  result.grid = std::move(grid);
  return result;
}

nav_msgs::msg::OccupancyGrid
occupancyGridToRos(const OccupancyGrid2D& grid,
                   const OccupancyGridToRosConfig& config) {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = config.header;
  msg.info.map_load_time = msg.header.stamp;
  msg.info.resolution = static_cast<float>(grid.resolution());
  msg.info.width = static_cast<std::uint32_t>(grid.width());
  msg.info.height = static_cast<std::uint32_t>(grid.height());
  msg.info.origin.position.x = grid.originX();
  msg.info.origin.position.y = grid.originY();
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(grid.cellCount(), static_cast<std::int8_t>(-1));

  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex cell{x, y};
      const std::size_t index = grid.linearIndex(cell);
      if (grid.isOccupied(cell)) {
        msg.data[index] = static_cast<std::int8_t>(100);
      } else if (config.include_inflation && grid.isInflated(cell)) {
        msg.data[index] = static_cast<std::int8_t>(80);
      } else if (grid.state(cell) == CellState::kFree) {
        msg.data[index] = static_cast<std::int8_t>(0);
      }
    }
  }

  return msg;
}

nav_msgs::msg::Path pathToRos(std::span<const Point2> points,
                              const std_msgs::msg::Header& header,
                              const double altitude_m) {
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(points.size());

  for (const Point2 point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = altitude_m;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

} // namespace drone_city_nav
