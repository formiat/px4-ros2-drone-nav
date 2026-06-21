#include "drone_city_nav/ros_conversions.hpp"

#include "drone_city_nav/grid_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::optional<int> finiteFloorToInt(const double value) noexcept {
  if (!std::isfinite(value)) {
    return std::nullopt;
  }

  const double floored = std::floor(value);
  if (!std::isfinite(floored) ||
      floored < static_cast<double>(std::numeric_limits<int>::min()) ||
      floored > static_cast<double>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }

  return static_cast<int>(floored);
}

[[nodiscard]] std::optional<int> finiteCeilToInt(const double value) noexcept {
  if (!std::isfinite(value)) {
    return std::nullopt;
  }

  const double ceiled = std::ceil(value);
  if (!std::isfinite(ceiled) ||
      ceiled < static_cast<double>(std::numeric_limits<int>::min()) ||
      ceiled > static_cast<double>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }

  return static_cast<int>(ceiled);
}

[[nodiscard]] std::optional<GridBounds>
rosGridBounds(const nav_msgs::msg::OccupancyGrid& msg) noexcept {
  if (!std::isfinite(msg.info.resolution) || !(msg.info.resolution > 0.0F) ||
      msg.info.width == 0U || msg.info.height == 0U ||
      msg.info.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      msg.info.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }

  const GridBounds bounds{msg.info.origin.position.x, msg.info.origin.position.y,
                          static_cast<double>(msg.info.resolution),
                          static_cast<int>(msg.info.width),
                          static_cast<int>(msg.info.height)};
  if (!gridBoundsUsable(bounds)) {
    return std::nullopt;
  }
  return bounds;
}

[[nodiscard]] std::optional<GridIndex> worldToRosGridCell(const GridBounds& bounds,
                                                          const Point2 point) noexcept {
  const auto x = finiteFloorToInt((point.x - bounds.origin_x) / bounds.resolution_m);
  const auto y = finiteFloorToInt((point.y - bounds.origin_y) / bounds.resolution_m);
  if (!x.has_value() || !y.has_value()) {
    return std::nullopt;
  }
  if (*x < 0 || *y < 0 || *x >= bounds.width_cells || *y >= bounds.height_cells) {
    return std::nullopt;
  }
  return GridIndex{*x, *y};
}

} // namespace

RawOccupancyGridFromRosResult
rawOccupancyGridFromRos(const nav_msgs::msg::OccupancyGrid& msg,
                        const RawOccupancyGridFromRosConfig& config) {
  RawOccupancyGridFromRosResult result{};
  const auto bounds = rosGridBounds(msg);
  if (!bounds.has_value()) {
    result.error = OccupancyGridFromRosError::kInvalidMetadata;
    return result;
  }

  const auto width = bounds->width_cells;
  const auto height = bounds->height_cells;
  result.expected_data_size =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  result.actual_data_size = msg.data.size();
  if (result.actual_data_size != result.expected_data_size) {
    result.error = OccupancyGridFromRosError::kMismatchedDataSize;
    return result;
  }

  OccupancyGrid2D grid{*bounds};
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const GridIndex cell{x, y};
      const std::size_t index = grid.linearIndex(cell);
      const auto raw_value = msg.data[index];
      if (raw_value < 0) {
        continue;
      }
      const int value = static_cast<int>(static_cast<unsigned char>(raw_value));
      if (value == config.occupied_value) {
        grid.setOccupied(cell);
      } else if (value == config.free_value) {
        grid.setFree(cell);
      } else {
        ++result.intermediate_value_cells;
      }
    }
  }

  result.grid = std::move(grid);
  return result;
}

nav_msgs::msg::OccupancyGrid
rawOccupancyGridToRos(const OccupancyGrid2D& grid,
                      const RawOccupancyGridToRosConfig& config) {
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
      } else if (grid.state(cell) == CellState::kFree) {
        msg.data[index] = static_cast<std::int8_t>(0);
      }
    }
  }

  return msg;
}

nav_msgs::msg::OccupancyGrid
prohibitedGridToRos(const OccupancyGrid2D& grid,
                    const ProhibitedGridToRosConfig& config) {
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
      } else if (grid.isInflated(cell)) {
        msg.data[index] = static_cast<std::int8_t>(80);
      } else if (grid.state(cell) == CellState::kFree) {
        msg.data[index] = static_cast<std::int8_t>(0);
      }
    }
  }

  return msg;
}

double occupancyGridClearanceM(const nav_msgs::msg::OccupancyGrid& msg,
                               const Point2 point, const double search_radius_m,
                               const std::int8_t min_occupancy_value) {
  const auto bounds = rosGridBounds(msg);
  if (!bounds.has_value()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto width = bounds->width_cells;
  const auto height = bounds->height_cells;
  const std::size_t expected_data_size =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (msg.data.size() != expected_data_size) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double resolution = bounds->resolution_m;
  const double origin_x = bounds->origin_x;
  const double origin_y = bounds->origin_y;
  const auto center = worldToRosGridCell(*bounds, point);
  if (!center.has_value()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double safe_search_radius_m =
      std::isfinite(search_radius_m) ? std::max(0.0, search_radius_m) : 0.0;
  const auto radius_cells = finiteCeilToInt(safe_search_radius_m / resolution);
  if (!radius_cells.has_value()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto radius = static_cast<long long>(*radius_cells);
  const auto center_x = static_cast<long long>(center->x);
  const auto center_y = static_cast<long long>(center->y);
  const int min_x = static_cast<int>(
      std::clamp(center_x - radius, 0LL, static_cast<long long>(width - 1)));
  const int max_x = static_cast<int>(
      std::clamp(center_x + radius, 0LL, static_cast<long long>(width - 1)));
  const int min_y = static_cast<int>(
      std::clamp(center_y - radius, 0LL, static_cast<long long>(height - 1)));
  const int max_y = static_cast<int>(
      std::clamp(center_y + radius, 0LL, static_cast<long long>(height - 1)));

  double nearest_clearance_m = std::numeric_limits<double>::infinity();
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      const std::size_t data_index =
          static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(x);
      if (msg.data[data_index] < min_occupancy_value) {
        continue;
      }
      const Point2 cell_center{origin_x + (static_cast<double>(x) + 0.5) * resolution,
                               origin_y + (static_cast<double>(y) + 0.5) * resolution};
      nearest_clearance_m = std::min(nearest_clearance_m, distance(point, cell_center));
    }
  }

  return nearest_clearance_m;
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
