#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>

namespace drone_city_nav {

struct GridOverlayStats {
  std::size_t source_occupied_cells{0U};
  std::size_t source_free_cells{0U};
  std::size_t occupied_cells_applied{0U};
  std::size_t free_cells_applied{0U};
  std::size_t occupied_cells_preserved{0U};
  std::size_t occupied_cells_excluded{0U};
  std::size_t free_cells_excluded{0U};
};

[[nodiscard]] bool haveSameGridGeometry(const OccupancyGrid2D& lhs,
                                        const OccupancyGrid2D& rhs) noexcept;

[[nodiscard]] GridOverlayStats overlayOccupiedCells(OccupancyGrid2D& destination,
                                                    const OccupancyGrid2D& source);

[[nodiscard]] GridOverlayStats
overlayOccupiedCellsExcludingProhibited(OccupancyGrid2D& destination,
                                        const OccupancyGrid2D& source,
                                        const OccupancyGrid2D& prohibited_exclusion);

[[nodiscard]] GridOverlayStats overlayKnownMemoryCells(OccupancyGrid2D& destination,
                                                       const OccupancyGrid2D& memory);

[[nodiscard]] GridOverlayStats
overlayKnownMemoryCellsExcludingProhibited(OccupancyGrid2D& destination,
                                           const OccupancyGrid2D& memory,
                                           const OccupancyGrid2D& prohibited_exclusion);

[[nodiscard]] GridOverlayStats
overlayCurrentLidarCells(OccupancyGrid2D& destination,
                         const OccupancyGrid2D& current_lidar);

[[nodiscard]] GridOverlayStats overlayCurrentLidarCellsExcludingProhibited(
    OccupancyGrid2D& destination, const OccupancyGrid2D& current_lidar,
    const OccupancyGrid2D& prohibited_exclusion);

} // namespace drone_city_nav
