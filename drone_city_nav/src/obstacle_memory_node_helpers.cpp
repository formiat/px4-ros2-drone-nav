#include "obstacle_memory_node_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace drone_city_nav {

std::optional<std::int64_t>
validRosStampNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  constexpr std::int64_t kNanosecondsPerSecond{1'000'000'000};
  if (stamp.sec < 0 ||
      stamp.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond) ||
      (stamp.sec == 0 && stamp.nanosec == 0U)) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(stamp.nanosec);
}

std::int8_t rawOccupancyValue(const OccupancyGrid2D& grid, const GridIndex cell) {
  if (grid.isOccupied(cell)) {
    return static_cast<std::int8_t>(100);
  }
  if (grid.state(cell) == CellState::kFree) {
    return static_cast<std::int8_t>(0);
  }
  return static_cast<std::int8_t>(-1);
}

const PassageStructure*
passageStructureNearPoint(const std::optional<KnownPassageMap>& map, const Point2 point,
                          const double margin_m) noexcept {
  if (!map.has_value()) {
    return nullptr;
  }
  for (const PassageStructure& structure : map->structures) {
    const double half_x = structure.size_x_m / 2.0;
    const double half_y = structure.size_y_m / 2.0;
    if (std::abs(point.x - structure.center.x) <= half_x + margin_m &&
        std::abs(point.y - structure.center.y) <= half_y + margin_m) {
      return &structure;
    }
  }
  return nullptr;
}

AmbiguousLidarHitTrackerConfig
declareAmbiguousLidarHitTrackerConfig(rclcpp::Node& node) {
  return AmbiguousLidarHitTrackerConfig{
      .required_independent_scans = static_cast<std::size_t>(std::clamp<std::int64_t>(
          node.declare_parameter<std::int64_t>(
              "ambiguous_lidar_hit_required_independent_scans", 3),
          1, 20)),
      .max_scan_gap_ns = static_cast<std::int64_t>(
          1'000'000.0 * std::clamp(node.declare_parameter<double>(
                                       "ambiguous_lidar_hit_max_scan_gap_ms", 500.0),
                                   1.0, 10'000.0)),
      .retention_ns = static_cast<std::int64_t>(
          1'000'000.0 * std::clamp(node.declare_parameter<double>(
                                       "ambiguous_lidar_hit_retention_ms", 2000.0),
                                   1.0, 60'000.0)),
      .endpoint_voxel_size_m = std::clamp(
          node.declare_parameter<double>("ambiguous_lidar_hit_voxel_size_m", 0.5), 0.1,
          5.0),
      .min_viewpoint_translation_m =
          std::clamp(node.declare_parameter<double>(
                         "ambiguous_lidar_hit_min_viewpoint_shift_m", 0.5),
                     0.0, 10.0),
      .min_viewpoint_direction_change_rad =
          std::clamp(node.declare_parameter<double>(
                         "ambiguous_lidar_hit_min_viewpoint_angle_deg", 4.0),
                     0.0, 180.0) *
          std::numbers::pi / 180.0,
  };
}

} // namespace drone_city_nav
