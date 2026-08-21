#pragma once

#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] std::vector<IncrementalTopologyTileIndex3D>
selectIncrementalTopologyTiles3D(
    std::span<const IncrementalTopologyTileIndex3D> pending_tiles,
    std::size_t maximum_tiles, const GridBounds3D& bounds, int tile_size_cells,
    const std::optional<IncrementalTopologyBuildPriority3D>& priority);

} // namespace drone_city_nav
