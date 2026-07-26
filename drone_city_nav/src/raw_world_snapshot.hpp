#pragma once

#include "drone_city_nav/msg/obstacle_memory_snapshot.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

#include <filesystem>
#include <optional>
#include <string_view>

namespace rclcpp {
class Node;
}

namespace drone_city_nav {

[[nodiscard]] std::optional<OccupancyGrid2D>
declareStaticRawWorldGrid(rclcpp::Node& node, std::string_view frame_id,
                          const std::filesystem::path& package_share);

[[nodiscard]] std::optional<msg::RawObstacleSnapshot>
composeRawObstacleSnapshot(const msg::ObstacleMemorySnapshot& memory_snapshot,
                           const std::optional<OccupancyGrid2D>& static_grid,
                           double critical_distance_m, double preferred_distance_m);

} // namespace drone_city_nav
