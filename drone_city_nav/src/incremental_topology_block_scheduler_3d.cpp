#include "drone_city_nav/incremental_topology_block_scheduler_3d.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <stdexcept>
#include <tuple>

namespace drone_city_nav {
namespace {

[[nodiscard]] Point3 blockCenter(const IncrementalTopologyBlockIndex3D block,
                                 const GridBounds3D& bounds,
                                 const int block_size_cells) noexcept {
  const double extent_m = static_cast<double>(block_size_cells) * bounds.resolution_m;
  return {bounds.origin_x + (static_cast<double>(block.x) + 0.5) * extent_m,
          bounds.origin_y + (static_cast<double>(block.y) + 0.5) * extent_m,
          bounds.origin_z + (static_cast<double>(block.z) + 0.5) * extent_m};
}

[[nodiscard]] double
buildPriorityScore(const IncrementalTopologyBlockIndex3D block,
                   const GridBounds3D& bounds, const int block_size_cells,
                   const IncrementalTopologyBuildPriority3D& priority) noexcept {
  const Point3 center = blockCenter(block, bounds, block_size_cells);
  const Vec3 offset{center.x - priority.position.x, center.y - priority.position.y,
                    center.z - priority.position.z};
  const Vec3 direction{priority.target.x - priority.position.x,
                       priority.target.y - priority.position.y,
                       priority.target.z - priority.position.z};
  const double direction_m =
      std::sqrt(direction.x * direction.x + direction.y * direction.y +
                direction.z * direction.z);
  if (direction_m <= 1.0e-6) {
    return std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z);
  }
  const double offset_squared =
      offset.x * offset.x + offset.y * offset.y + offset.z * offset.z;
  const double distance_m = std::sqrt(offset_squared);
  if (distance_m <= 1.0e-6) {
    return 0.0;
  }
  const double forward_m =
      (offset.x * direction.x + offset.y * direction.y + offset.z * direction.z) /
      direction_m;
  const double alignment = std::clamp(forward_m / distance_m, -1.0, 1.0);
  return distance_m + 0.25 * (1.0 - alignment);
}

} // namespace

std::vector<IncrementalTopologyBlockIndex3D> selectIncrementalTopologyBlocks3D(
    const std::span<const IncrementalTopologyBlockIndex3D> pending_blocks,
    const std::size_t maximum_blocks, const GridBounds3D& bounds,
    const int block_size_cells,
    const std::optional<IncrementalTopologyBuildPriority3D>& priority) {
  if (block_size_cells <= 0) {
    throw std::invalid_argument{"topology block size must be positive"};
  }
  std::vector<IncrementalTopologyBlockIndex3D> ordered{pending_blocks.begin(),
                                                       pending_blocks.end()};
  std::ranges::sort(ordered, [&](const IncrementalTopologyBlockIndex3D first,
                                 const IncrementalTopologyBlockIndex3D second) {
    if (priority.has_value()) {
      const double first_score =
          buildPriorityScore(first, bounds, block_size_cells, *priority);
      const double second_score =
          buildPriorityScore(second, bounds, block_size_cells, *priority);
      if (std::abs(first_score - second_score) > 1.0e-9) {
        return first_score < second_score;
      }
    }
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  ordered.resize(std::min(maximum_blocks, ordered.size()));
  return ordered;
}

} // namespace drone_city_nav
