#pragma once

#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>

namespace drone_city_nav {

struct SemanticPortalOccupancyResult {
  std::size_t side_volumes_considered{0U};
  std::size_t cells_marked_occupied{0U};
};

[[nodiscard]] SemanticPortalOccupancyResult
overlaySemanticPortalSideSolids(OccupancyGrid2D& grid,
                                const KnownPassageMap& passage_map);

} // namespace drone_city_nav
