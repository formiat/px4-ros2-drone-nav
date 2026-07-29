#include "drone_city_nav/semantic_portal_occupancy.hpp"

#include "drone_city_nav/known_passage_solid_volumes.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool isSideVolume(const KnownPassageSolidVolume& volume) noexcept {
  return volume.part_kind == KnownPassageSolidPartKind::kLeft ||
         volume.part_kind == KnownPassageSolidPartKind::kRight;
}

[[nodiscard]] std::array<Point2, 4U>
corners(const KnownPassageSolidVolume& volume) noexcept {
  const double half_depth = 0.5 * volume.depth_m;
  const double half_width = 0.5 * volume.width_m;
  const auto corner = [&volume](const double depth, const double lateral) {
    return Point2{
        volume.center.x + depth * volume.normal_xy.x + lateral * volume.lateral_xy.x,
        volume.center.y + depth * volume.normal_xy.y + lateral * volume.lateral_xy.y,
    };
  };
  return {
      corner(-half_depth, -half_width),
      corner(-half_depth, half_width),
      corner(half_depth, half_width),
      corner(half_depth, -half_width),
  };
}

} // namespace

SemanticPortalOccupancyResult
overlaySemanticPortalSideSolids(OccupancyGrid2D& grid,
                                const KnownPassageMap& passage_map) {
  SemanticPortalOccupancyResult result;
  for (const KnownPassageSolidVolume& volume : knownPassageSolidVolumes(passage_map)) {
    if (!isSideVolume(volume)) {
      continue;
    }
    ++result.side_volumes_considered;
    const std::array<Point2, 4U> volume_corners = corners(volume);
    double minimum_x = std::numeric_limits<double>::max();
    double maximum_x = std::numeric_limits<double>::lowest();
    double minimum_y = std::numeric_limits<double>::max();
    double maximum_y = std::numeric_limits<double>::lowest();
    for (const Point2 corner : volume_corners) {
      minimum_x = std::min(minimum_x, corner.x);
      maximum_x = std::max(maximum_x, corner.x);
      minimum_y = std::min(minimum_y, corner.y);
      maximum_y = std::max(maximum_y, corner.y);
    }
    const GridBounds& bounds = grid.bounds();
    const int first_x =
        std::max(0, static_cast<int>(std::floor((minimum_x - bounds.origin_x) /
                                                bounds.resolution_m)));
    const int last_x =
        std::min(bounds.width_cells - 1,
                 static_cast<int>(
                     std::floor((maximum_x - bounds.origin_x) / bounds.resolution_m)));
    const int first_y =
        std::max(0, static_cast<int>(std::floor((minimum_y - bounds.origin_y) /
                                                bounds.resolution_m)));
    const int last_y =
        std::min(bounds.height_cells - 1,
                 static_cast<int>(
                     std::floor((maximum_y - bounds.origin_y) / bounds.resolution_m)));
    const double cell_margin = 0.5 * std::numbers::sqrt2 * bounds.resolution_m;
    for (int y = first_y; y <= last_y; ++y) {
      for (int x = first_x; x <= last_x; ++x) {
        const GridIndex cell{x, y};
        const Point2 center = grid.cellCenter(cell);
        const double dx = center.x - volume.center.x;
        const double dy = center.y - volume.center.y;
        const double depth = dx * volume.normal_xy.x + dy * volume.normal_xy.y;
        const double lateral = dx * volume.lateral_xy.x + dy * volume.lateral_xy.y;
        if (std::abs(depth) > 0.5 * volume.depth_m + cell_margin ||
            std::abs(lateral) > 0.5 * volume.width_m + cell_margin) {
          continue;
        }
        if (!grid.isOccupied(cell)) {
          ++result.cells_marked_occupied;
        }
        grid.setOccupied(cell);
      }
    }
  }
  return result;
}

} // namespace drone_city_nav
