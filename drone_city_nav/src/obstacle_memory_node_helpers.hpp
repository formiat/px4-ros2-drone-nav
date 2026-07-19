#pragma once

#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <optional>

namespace drone_city_nav {

[[nodiscard]] std::optional<std::int64_t>
validRosStampNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept;

[[nodiscard]] std::int8_t rawOccupancyValue(const OccupancyGrid2D& grid,
                                            GridIndex cell);

[[nodiscard]] const PassageStructure*
passageStructureNearPoint(const std::optional<KnownPassageMap>& map, Point2 point,
                          double margin_m) noexcept;

} // namespace drone_city_nav
