#include "drone_city_nav/grid_overlay.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_city_nav {
namespace {

constexpr double kGeometryTolerance = 1.0e-9;

void requireSameGridGeometry(const OccupancyGrid2D& lhs, const OccupancyGrid2D& rhs) {
  if (!haveSameGridGeometry(lhs, rhs)) {
    throw std::invalid_argument{"Grid overlay requires matching grid geometry"};
  }
}

} // namespace

bool haveSameGridGeometry(const OccupancyGrid2D& lhs,
                          const OccupancyGrid2D& rhs) noexcept {
  return lhs.width() == rhs.width() && lhs.height() == rhs.height() &&
         std::abs(lhs.resolution() - rhs.resolution()) <= kGeometryTolerance &&
         std::abs(lhs.originX() - rhs.originX()) <= kGeometryTolerance &&
         std::abs(lhs.originY() - rhs.originY()) <= kGeometryTolerance;
}

GridOverlayStats overlayOccupiedCells(OccupancyGrid2D& destination,
                                      const OccupancyGrid2D& source) {
  requireSameGridGeometry(destination, source);
  GridOverlayStats stats{};
  for (int y = 0; y < source.height(); ++y) {
    for (int x = 0; x < source.width(); ++x) {
      const GridIndex cell{x, y};
      if (!source.isOccupied(cell)) {
        continue;
      }
      ++stats.source_occupied_cells;
      if (destination.isOccupied(cell)) {
        ++stats.occupied_cells_preserved;
        continue;
      }
      destination.setOccupied(cell);
      ++stats.occupied_cells_applied;
    }
  }
  return stats;
}

GridOverlayStats overlayKnownMemoryCells(OccupancyGrid2D& destination,
                                         const OccupancyGrid2D& memory) {
  requireSameGridGeometry(destination, memory);
  GridOverlayStats stats{};
  for (int y = 0; y < memory.height(); ++y) {
    for (int x = 0; x < memory.width(); ++x) {
      const GridIndex cell{x, y};
      if (memory.isOccupied(cell)) {
        ++stats.source_occupied_cells;
        if (destination.isOccupied(cell)) {
          ++stats.occupied_cells_preserved;
          continue;
        }
        destination.setOccupied(cell);
        ++stats.occupied_cells_applied;
        continue;
      }
      if (memory.state(cell) != CellState::kFree) {
        continue;
      }
      ++stats.source_free_cells;
      if (destination.isOccupied(cell)) {
        ++stats.occupied_cells_preserved;
        continue;
      }
      if (destination.state(cell) == CellState::kUnknown) {
        destination.setFree(cell);
        ++stats.free_cells_applied;
      }
    }
  }
  return stats;
}

GridOverlayStats overlayCurrentLidarCells(OccupancyGrid2D& destination,
                                          const OccupancyGrid2D& current_lidar) {
  return overlayOccupiedCells(destination, current_lidar);
}

} // namespace drone_city_nav
