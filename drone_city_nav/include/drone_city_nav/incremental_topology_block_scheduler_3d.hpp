#pragma once

#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class IncrementalTopologyBlockPriorityTier3D : std::uint8_t {
  kLocalSafety,
  kForwardCorridor,
  kBacklog,
};

[[nodiscard]] IncrementalTopologyBlockPriorityTier3D
incrementalTopologyBlockPriorityTier3D(
    IncrementalTopologyBlockIndex3D block, const GridBounds3D& bounds,
    int block_size_cells, const IncrementalTopologyBuildPriority3D& priority) noexcept;

[[nodiscard]] std::vector<IncrementalTopologyBlockIndex3D>
selectIncrementalTopologyBlocks3D(
    std::span<const IncrementalTopologyBlockIndex3D> pending_blocks,
    std::size_t maximum_blocks, const GridBounds3D& bounds, int block_size_cells,
    const std::optional<IncrementalTopologyBuildPriority3D>& priority);

} // namespace drone_city_nav
