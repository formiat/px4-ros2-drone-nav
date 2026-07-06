#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"
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

struct RawOccupancyGridFromRosConfig {
  int occupied_value{100};
  int free_value{0};
};

struct RawOccupancyGridFromRosResult {
  std::optional<OccupancyGrid2D> grid;
  std::optional<OccupancyGridFromRosError> error;
  std::size_t expected_data_size{0U};
  std::size_t actual_data_size{0U};
  std::size_t intermediate_value_cells{0U};
};

struct RawOccupancyGridToRosConfig {
  std_msgs::msg::Header header;
};

struct ProhibitedGridToRosConfig {
  std_msgs::msg::Header header;
};

[[nodiscard]] nav_msgs::msg::OccupancyGrid
rawOccupancyGridToRos(const OccupancyGrid2D& grid,
                      const RawOccupancyGridToRosConfig& config);

[[nodiscard]] RawOccupancyGridFromRosResult
rawOccupancyGridFromRos(const nav_msgs::msg::OccupancyGrid& msg,
                        const RawOccupancyGridFromRosConfig& config);

[[nodiscard]] nav_msgs::msg::OccupancyGrid
prohibitedGridToRos(const OccupancyGrid2D& grid,
                    const ProhibitedGridToRosConfig& config);

[[nodiscard]] double occupancyGridClearanceM(const nav_msgs::msg::OccupancyGrid& msg,
                                             Point2 point, double search_radius_m,
                                             std::int8_t min_occupancy_value);

[[nodiscard]] nav_msgs::msg::Path pathToRos(std::span<const Point2> points,
                                            const std_msgs::msg::Header& header,
                                            double altitude_m);

[[nodiscard]] nav_msgs::msg::Path
pathToRos(std::span<const TrajectoryPointSample> samples,
          const std_msgs::msg::Header& header);

} // namespace drone_city_nav
