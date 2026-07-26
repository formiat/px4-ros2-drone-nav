#include "raw_world_snapshot.hpp"

#include "drone_city_nav/grid_overlay.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/static_map_source.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace drone_city_nav {

std::optional<OccupancyGrid2D>
declareStaticRawWorldGrid(rclcpp::Node& node, const std::string_view frame_id,
                          const std::filesystem::path& package_share) {
  const StaticMapSourceResult source = loadStaticMapSource(StaticMapSourceConfig{
      .enabled = node.declare_parameter<bool>("use_static_map", true),
      .configured_path = node.declare_parameter<std::string>(
          "static_map_path", "worlds/generated_city.map2d"),
      .package_share_directory = package_share,
      .expected_frame_id = std::string{frame_id},
      .min_blocking_height_m =
          node.declare_parameter<double>("static_map_min_blocking_height_m", 0.0)});
  if (source.status == StaticMapSourceStatus::kLoadFailed) {
    throw std::runtime_error{"failed to load raw static world map: " +
                             source.error_message};
  }
  return source.grid;
}

std::optional<msg::RawObstacleSnapshot>
composeRawObstacleSnapshot(const msg::ObstacleMemorySnapshot& memory_snapshot,
                           const std::optional<OccupancyGrid2D>& static_grid,
                           const double critical_distance_m,
                           const double preferred_distance_m) {
  const RawOccupancyGridFromRosResult converted = rawOccupancyGridFromRos(
      memory_snapshot.grid, RawOccupancyGridFromRosConfig{100, 0});
  if (!converted.grid.has_value()) {
    return std::nullopt;
  }
  OccupancyGrid2D raw_world = *converted.grid;
  if (static_grid.has_value()) {
    if (!haveSameGridGeometry(raw_world, *static_grid)) {
      return std::nullopt;
    }
    OccupancyGrid2D combined = *static_grid;
    const GridOverlayStats overlay = overlayKnownMemoryCells(combined, raw_world);
    (void)overlay;
    raw_world = std::move(combined);
  }
  msg::RawObstacleSnapshot message;
  message.producer_instance_id = memory_snapshot.producer_instance_id;
  message.obstacle_snapshot_revision = memory_snapshot.sequence;
  message.risk_policy_fingerprint =
      (static_cast<std::uint64_t>(std::llround(critical_distance_m * 1000.0)) << 32U) ^
      static_cast<std::uint64_t>(std::llround(preferred_distance_m * 1000.0));
  message.risk_critical_distance_m = critical_distance_m;
  message.risk_preferred_distance_m = preferred_distance_m;
  message.grid = rawOccupancyGridToRos(
      raw_world, RawOccupancyGridToRosConfig{memory_snapshot.grid.header});
  return message;
}

} // namespace drone_city_nav
