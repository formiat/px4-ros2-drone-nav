#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

namespace drone_city_nav {

[[nodiscard]] GridBounds selectLocalEsdfBounds(const GridBounds& world_bounds,
                                               Point2 center, double half_extent_m);

[[nodiscard]] OccupancyGrid2D cropOccupancyGrid(const OccupancyGrid2D& source,
                                                const GridBounds& local_bounds);

[[nodiscard]] bool localEsdfNeedsRecenter(const GridBounds& active_bounds,
                                          const GridBounds& world_bounds,
                                          Point2 position,
                                          double core_margin_m) noexcept;

} // namespace drone_city_nav
