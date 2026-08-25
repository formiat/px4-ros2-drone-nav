#include "observed_esdf_3d_regions.hpp"

#include <algorithm>
#include <array>
#include <deque>
#include <unordered_set>
#include <utility>

namespace drone_city_nav {

std::vector<ChangedCellRegion3D>
splitChangedCellRegions3D(const ChangedCellRegion3D& changed,
                          const GridBounds3D& bounds) {
  const auto encode = [&bounds](const GridIndex3D cell) {
    return (static_cast<std::size_t>(cell.z) *
                static_cast<std::size_t>(bounds.height_cells) +
            static_cast<std::size_t>(cell.y)) *
               static_cast<std::size_t>(bounds.width_cells) +
           static_cast<std::size_t>(cell.x);
  };
  std::unordered_set<std::size_t> remaining;
  remaining.reserve(changed.cells.size());
  for (const GridIndex3D cell : changed.cells) {
    remaining.insert(encode(cell));
  }
  const std::array<GridIndex3D, 6U> directions{
      GridIndex3D{1, 0, 0},  GridIndex3D{-1, 0, 0}, GridIndex3D{0, 1, 0},
      GridIndex3D{0, -1, 0}, GridIndex3D{0, 0, 1},  GridIndex3D{0, 0, -1}};
  const auto in_bounds = [&bounds](const GridIndex3D cell) noexcept {
    return cell.x >= 0 && cell.x < bounds.width_cells && cell.y >= 0 &&
           cell.y < bounds.height_cells && cell.z >= 0 && cell.z < bounds.depth_cells;
  };
  std::vector<ChangedCellRegion3D> regions;
  for (const GridIndex3D seed : changed.cells) {
    if (!remaining.contains(encode(seed))) {
      continue;
    }
    ChangedCellRegion3D region;
    std::deque<GridIndex3D> pending{seed};
    remaining.erase(encode(seed));
    while (!pending.empty()) {
      const GridIndex3D cell = pending.front();
      pending.pop_front();
      if (region.count == 0U) {
        region.minimum = cell;
        region.maximum_exclusive = {cell.x + 1, cell.y + 1, cell.z + 1};
      } else {
        region.minimum.x = std::min(region.minimum.x, cell.x);
        region.minimum.y = std::min(region.minimum.y, cell.y);
        region.minimum.z = std::min(region.minimum.z, cell.z);
        region.maximum_exclusive.x = std::max(region.maximum_exclusive.x, cell.x + 1);
        region.maximum_exclusive.y = std::max(region.maximum_exclusive.y, cell.y + 1);
        region.maximum_exclusive.z = std::max(region.maximum_exclusive.z, cell.z + 1);
      }
      ++region.count;
      region.cells.push_back(cell);
      for (const GridIndex3D direction : directions) {
        const GridIndex3D neighbor{cell.x + direction.x, cell.y + direction.y,
                                   cell.z + direction.z};
        if (in_bounds(neighbor) && remaining.erase(encode(neighbor)) != 0U) {
          pending.push_back(neighbor);
        }
      }
    }
    regions.push_back(std::move(region));
  }
  return regions;
}

} // namespace drone_city_nav
