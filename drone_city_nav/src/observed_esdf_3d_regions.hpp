#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"

#include <cstddef>
#include <vector>

namespace drone_city_nav {

struct ChangedCellRegion3D {
  GridIndex3D minimum{};
  GridIndex3D maximum_exclusive{};
  std::size_t count{0U};
  std::vector<GridIndex3D> cells;
};

[[nodiscard]] std::vector<ChangedCellRegion3D>
splitChangedCellRegions3D(const ChangedCellRegion3D& changed,
                          const GridBounds3D& bounds);

} // namespace drone_city_nav
