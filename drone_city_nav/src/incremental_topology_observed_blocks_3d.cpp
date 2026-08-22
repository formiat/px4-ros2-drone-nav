#include "incremental_topology_observed_blocks_3d.hpp"

#include "drone_city_nav/incremental_topology_block_scheduler_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <ranges>
#include <tuple>

namespace drone_city_nav::incremental_topology_detail {
namespace {

[[nodiscard]] IncrementalTopologyBlockIndex3D
blockForCell(const GridIndex3D cell, const int block_size_cells) noexcept {
  return {cell.x / block_size_cells, cell.y / block_size_cells,
          cell.z / block_size_cells};
}

template<typename Visitor>
void forEachSetBit(std::uint64_t bits, const std::size_t word_index, Visitor visitor) {
  while (bits != 0U) {
    const auto offset = static_cast<std::size_t>(std::countr_zero(bits));
    visitor(word_index * 64U + offset);
    bits &= bits - 1U;
  }
}

[[nodiscard]] GridIndex3D chunkCell(const OccupancyChunkIndex3D chunk,
                                    const std::size_t bit_index) noexcept {
  constexpr int chunk_size = ObservedOccupancyGrid3D::kChunkSize;
  const int local_x = static_cast<int>(bit_index % chunk_size);
  const int local_y = static_cast<int>((bit_index / chunk_size) % chunk_size);
  const int local_z =
      static_cast<int>(bit_index / static_cast<std::size_t>(chunk_size * chunk_size));
  return {chunk.x * chunk_size + local_x, chunk.y * chunk_size + local_y,
          chunk.z * chunk_size + local_z};
}

} // namespace

ObservedBlockLifecycle3D::ObservedBlockLifecycle3D(
    const IncrementalTopologyGraph3DConfig& config)
    : config_{config} {
}

std::vector<IncrementalTopologyBlockIndex3D>
ObservedBlockLifecycle3D::allObservedBlocks(
    const ObservedOccupancyGrid3D& occupancy) const {
  std::unordered_set<IncrementalTopologyBlockIndex3D,
                     IncrementalTopologyBlockIndex3DHash>
      unique;
  for (const auto& [chunk, data] : occupancy.chunks()) {
    for (std::size_t word = 0U; word < data.observed.size(); ++word) {
      const std::uint64_t known_free = data.observed.at(word) & ~data.occupied.at(word);
      forEachSetBit(known_free, word, [&](const std::size_t bit_index) {
        const GridIndex3D cell = chunkCell(chunk, bit_index);
        if (occupancy.contains(cell)) {
          unique.insert(blockForCell(cell, config_.block_size_cells));
        }
      });
    }
  }
  return sortedBlocks(unique);
}

std::vector<IncrementalTopologyBlockIndex3D>
ObservedBlockLifecycle3D::dirtyObservedBlocks(
    const ObservedOccupancyGrid3D& occupancy,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks) {
  std::unordered_set<IncrementalTopologyBlockIndex3D,
                     IncrementalTopologyBlockIndex3DHash>
      unique;
  const GridBounds3D& bounds = occupancy.bounds();
  const double maximum_extent_m =
      std::max({config_.footprint.radius_m, config_.footprint.lower_extent_m,
                config_.footprint.upper_extent_m});
  const int halo_cells =
      static_cast<int>(std::ceil(maximum_extent_m / bounds.resolution_m)) +
      config_.coarse_sample_stride_cells;
  for (const OccupancyChunkIndex3D chunk_index : dirty_chunks) {
    const auto previous = observed_chunks_.find(chunk_index);
    const auto current = occupancy.chunks().find(chunk_index);
    for (std::size_t word = 0U; word < OccupancyGrid3D::kWordsPerChunk; ++word) {
      const std::uint64_t previous_observed =
          previous == observed_chunks_.end() ? 0U : previous->second.observed.at(word);
      const std::uint64_t previous_occupied =
          previous == observed_chunks_.end() ? 0U : previous->second.occupied.at(word);
      const std::uint64_t current_observed =
          current == occupancy.chunks().end() ? 0U : current->second.observed.at(word);
      const std::uint64_t current_occupied =
          current == occupancy.chunks().end() ? 0U : current->second.occupied.at(word);
      const std::uint64_t changed = (previous_observed ^ current_observed) |
                                    (previous_occupied ^ current_occupied);
      forEachSetBit(changed, word, [&](const std::size_t bit_index) {
        const GridIndex3D cell = chunkCell(chunk_index, bit_index);
        if (!occupancy.contains(cell)) {
          return;
        }
        addBlockRange(unique,
                      {std::max(0, cell.x - halo_cells),
                       std::max(0, cell.y - halo_cells),
                       std::max(0, cell.z - halo_cells)},
                      {std::min(bounds.width_cells - 1, cell.x + halo_cells),
                       std::min(bounds.height_cells - 1, cell.y + halo_cells),
                       std::min(bounds.depth_cells - 1, cell.z + halo_cells)});
      });
    }
    if (current == occupancy.chunks().end()) {
      observed_chunks_.erase(chunk_index);
    } else {
      observed_chunks_[chunk_index] = current->second;
    }
  }
  return sortedBlocks(unique);
}

std::vector<OccupancyChunkIndex3D> ObservedBlockLifecycle3D::completeSnapshotChunks(
    const ObservedOccupancyGrid3D& occupancy) const {
  std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash> unique;
  unique.reserve(observed_chunks_.size() + occupancy.chunks().size());
  for (const auto& [index, unused] : observed_chunks_) {
    static_cast<void>(unused);
    unique.insert(index);
  }
  for (const auto& [index, unused] : occupancy.chunks()) {
    static_cast<void>(unused);
    unique.insert(index);
  }
  std::vector<OccupancyChunkIndex3D> result{unique.begin(), unique.end()};
  std::ranges::sort(result, [](const OccupancyChunkIndex3D first,
                               const OccupancyChunkIndex3D second) {
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  return result;
}

std::vector<IncrementalTopologyBlockIndex3D>
ObservedBlockLifecycle3D::allStaticBlocks(const GridBounds3D& bounds) const {
  std::unordered_set<IncrementalTopologyBlockIndex3D,
                     IncrementalTopologyBlockIndex3DHash>
      unique;
  addBlockRange(
      unique, {0, 0, 0},
      {bounds.width_cells - 1, bounds.height_cells - 1, bounds.depth_cells - 1});
  return sortedBlocks(unique);
}

void ObservedBlockLifecycle3D::addBlockRange(
    std::unordered_set<IncrementalTopologyBlockIndex3D,
                       IncrementalTopologyBlockIndex3DHash>& output,
    const GridIndex3D minimum, const GridIndex3D maximum) const {
  if (maximum.x < minimum.x || maximum.y < minimum.y || maximum.z < minimum.z) {
    return;
  }
  const IncrementalTopologyBlockIndex3D first =
      blockForCell(minimum, config_.block_size_cells);
  const IncrementalTopologyBlockIndex3D last =
      blockForCell(maximum, config_.block_size_cells);
  for (int z = first.z; z <= last.z; ++z) {
    for (int y = first.y; y <= last.y; ++y) {
      for (int x = first.x; x <= last.x; ++x) {
        output.insert({x, y, z});
      }
    }
  }
}

std::vector<IncrementalTopologyBlockIndex3D> ObservedBlockLifecycle3D::sortedBlocks(
    const std::unordered_set<IncrementalTopologyBlockIndex3D,
                             IncrementalTopologyBlockIndex3DHash>& blocks) {
  std::vector<IncrementalTopologyBlockIndex3D> result{blocks.begin(), blocks.end()};
  std::ranges::sort(result, [](const IncrementalTopologyBlockIndex3D first,
                               const IncrementalTopologyBlockIndex3D second) {
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  return result;
}

void ObservedBlockLifecycle3D::clearPending() {
  pending_blocks_.clear();
  pending_sequence_.clear();
}

void ObservedBlockLifecycle3D::enqueue(
    const std::span<const IncrementalTopologyBlockIndex3D> dirty_blocks) {
  for (const IncrementalTopologyBlockIndex3D block : dirty_blocks) {
    if (pending_blocks_.insert(block).second) {
      pending_sequence_.emplace(block, next_pending_sequence_++);
    }
  }
}

std::vector<IncrementalTopologyBlockIndex3D> ObservedBlockLifecycle3D::takePending(
    const GridBounds3D& bounds,
    const std::optional<IncrementalTopologyBuildPriority3D>& priority,
    const bool preserve_oldest_work) {
  const std::vector<IncrementalTopologyBlockIndex3D> pending{pending_blocks_.begin(),
                                                             pending_blocks_.end()};
  const std::size_t oldest_budget =
      preserve_oldest_work && priority.has_value()
          ? std::min(config_.minimum_oldest_blocks_per_update,
                     config_.maximum_observed_blocks_per_update - 1U)
          : 0U;
  const std::size_t priority_budget =
      config_.maximum_observed_blocks_per_update - oldest_budget;
  std::vector<IncrementalTopologyBlockIndex3D> ordered =
      selectIncrementalTopologyBlocks3D(pending, priority_budget, bounds,
                                        config_.block_size_cells, priority);
  std::unordered_set<IncrementalTopologyBlockIndex3D,
                     IncrementalTopologyBlockIndex3DHash>
      selected{ordered.begin(), ordered.end()};
  std::vector<IncrementalTopologyBlockIndex3D> oldest{pending.begin(), pending.end()};
  std::ranges::sort(oldest, [this](const auto first, const auto second) {
    return std::tie(pending_sequence_.at(first), first.z, first.y, first.x) <
           std::tie(pending_sequence_.at(second), second.z, second.y, second.x);
  });
  for (const IncrementalTopologyBlockIndex3D block : oldest) {
    if (ordered.size() >= config_.maximum_observed_blocks_per_update) {
      break;
    }
    if (selected.insert(block).second) {
      ordered.push_back(block);
    }
  }
  for (const IncrementalTopologyBlockIndex3D block : ordered) {
    pending_blocks_.erase(block);
    pending_sequence_.erase(block);
  }
  return ordered;
}

const std::unordered_set<IncrementalTopologyBlockIndex3D,
                         IncrementalTopologyBlockIndex3DHash>&
ObservedBlockLifecycle3D::pending() const noexcept {
  return pending_blocks_;
}

std::size_t ObservedBlockLifecycle3D::pendingCount() const noexcept {
  return pending_blocks_.size();
}

void ObservedBlockLifecycle3D::replaceObservedSnapshot(
    const ObservedOccupancyGrid3D::ChunkMap& chunks) {
  observed_chunks_ = chunks;
}

void ObservedBlockLifecycle3D::clearObservedSnapshot() {
  observed_chunks_.clear();
}

} // namespace drone_city_nav::incremental_topology_detail
