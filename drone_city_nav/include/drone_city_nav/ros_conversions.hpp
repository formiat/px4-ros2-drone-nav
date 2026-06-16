#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/types.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>

#include <optional>
#include <span>
#include <string>

namespace drone_city_nav {

enum class OccupancyGridFromRosError {
  kInvalidMetadata,
  kMismatchedDataSize,
};

struct OccupancyGridFromRosConfig {
  int occupied_threshold{65};
  int free_threshold{0};
};

struct OccupancyGridFromRosResult {
  std::optional<OccupancyGrid2D> grid;
  std::optional<OccupancyGridFromRosError> error;
  std::size_t expected_data_size{0U};
  std::size_t actual_data_size{0U};
};

struct OccupancyGridToRosConfig {
  std_msgs::msg::Header header;
  bool include_inflation{true};
};

[[nodiscard]] OccupancyGridFromRosResult
occupancyGridFromRos(const nav_msgs::msg::OccupancyGrid& msg,
                     const OccupancyGridFromRosConfig& config);

[[nodiscard]] nav_msgs::msg::OccupancyGrid
occupancyGridToRos(const OccupancyGrid2D& grid, const OccupancyGridToRosConfig& config);

[[nodiscard]] nav_msgs::msg::Path pathToRos(std::span<const Point2> points,
                                            const std_msgs::msg::Header& header,
                                            double altitude_m);

} // namespace drone_city_nav
