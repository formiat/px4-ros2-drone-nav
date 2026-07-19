#include "obstacle_memory_node_helpers.hpp"

#include <cmath>

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

} // namespace drone_city_nav
