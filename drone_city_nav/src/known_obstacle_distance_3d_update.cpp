#include "drone_city_nav/known_obstacle_distance_3d.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "known_obstacle_distance_3d_internal.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool compatiblePreviousField(
    const KnownObstacleDistance3D& previous,
    const detail::KnownObstacleDistanceStorage3D& current_geometry) noexcept {
  return previous.valid() &&
         detail::sameKnownObstacleBounds3D(previous.bounds(),
                                           current_geometry.output_bounds) &&
         detail::sameKnownObstacleBounds3D(previous.sourceBounds(),
                                           current_geometry.source_bounds) &&
         std::abs(previous.maximumDistanceM() - current_geometry.maximum_distance_m) <=
             1.0e-9;
}

void appendSourceDifference(const detail::KnownObstacleSourceChunk3D& previous,
                            const detail::KnownObstacleSourceChunk3D& current,
                            std::vector<detail::KnownObstacleSourcePoint3D>& inserted,
                            std::vector<detail::KnownObstacleSourcePoint3D>& removed) {
  std::size_t previous_index{0U};
  std::size_t current_index{0U};
  while (previous_index < previous.size() || current_index < current.size()) {
    if (previous_index == previous.size()) {
      inserted.push_back(current[current_index++]);
      continue;
    }
    if (current_index == current.size()) {
      removed.push_back(previous[previous_index++]);
      continue;
    }
    if (previous[previous_index].key < current[current_index].key) {
      removed.push_back(previous[previous_index++]);
    } else if (current[current_index].key < previous[previous_index].key) {
      inserted.push_back(current[current_index++]);
    } else {
      ++previous_index;
      ++current_index;
    }
  }
}

[[nodiscard]] std::vector<OccupancyChunkIndex3D>
affectedSourceChunks(const detail::KnownObstacleDistanceStorage3D& previous,
                     const std::span<const OccupancyChunkIndex3D> dirty_chunks,
                     const std::span<const GridIndex3D> current_suppressed_cells) {
  std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash> result{
      dirty_chunks.begin(), dirty_chunks.end()};
  for (const GridIndex3D cell : previous.suppressed_source_cells) {
    result.insert(ObservedOccupancyGrid3D::chunkIndex(cell));
  }
  for (const GridIndex3D cell : current_suppressed_cells) {
    result.insert(ObservedOccupancyGrid3D::chunkIndex(cell));
  }
  std::vector<OccupancyChunkIndex3D> ordered{result.begin(), result.end()};
  std::ranges::sort(ordered, {}, [](const OccupancyChunkIndex3D index) {
    return std::tuple{index.z, index.y, index.x};
  });
  return ordered;
}

[[nodiscard]] std::size_t
affectedVoxelCount(const GridBounds3D& bounds,
                   const std::span<const OccupancyChunkIndex3D> chunks) noexcept {
  std::size_t result{0U};
  for (const OccupancyChunkIndex3D chunk : chunks) {
    result += detail::knownObstacleRegionVoxelCount3D(
        detail::knownObstacleChunkRegion3D(bounds, chunk));
  }
  return result;
}

[[nodiscard]] bool
chunksEqual(const std::shared_ptr<const detail::KnownObstacleDistanceChunk3D>& first,
            const std::shared_ptr<const detail::KnownObstacleDistanceChunk3D>& second) {
  if (first == nullptr || second == nullptr) {
    return first == second;
  }
  return *first == *second;
}

} // namespace

namespace detail {

bool knownObstacleRawChangesCovered3D(
    const ObservedOccupancyGrid3D& previous, const ObservedOccupancyGrid3D& current,
    const KnownObstacleDistanceStorage3D& storage,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks) {
  if (!sameKnownObstacleBounds3D(previous.bounds(), current.bounds())) {
    return false;
  }
  const std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash> dirty{
      dirty_chunks.begin(), dirty_chunks.end()};
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  const OccupancyChunkIndex3D first{storage.source_minimum_x / kChunkSize,
                                    storage.source_minimum_y / kChunkSize,
                                    storage.source_minimum_z / kChunkSize};
  const OccupancyChunkIndex3D last{
      (storage.source_maximum_x_exclusive - 1) / kChunkSize,
      (storage.source_maximum_y_exclusive - 1) / kChunkSize,
      (storage.source_maximum_z_exclusive - 1) / kChunkSize};
  for (int z = first.z; z <= last.z; ++z) {
    for (int y = first.y; y <= last.y; ++y) {
      for (int x = first.x; x <= last.x; ++x) {
        const OccupancyChunkIndex3D chunk_index{x, y, z};
        if (dirty.contains(chunk_index)) {
          continue;
        }
        const ObservedOccupancyGrid3D::Chunk* const before =
            previous.findChunk(chunk_index);
        const ObservedOccupancyGrid3D::Chunk* const after =
            current.findChunk(chunk_index);
        for (std::size_t word = 0U; word < OccupancyGrid3D::kWordsPerChunk; ++word) {
          const std::uint64_t before_word =
              before != nullptr ? before->occupied.at(word) : 0U;
          const std::uint64_t after_word =
              after != nullptr ? after->occupied.at(word) : 0U;
          std::uint64_t difference = before_word ^ after_word;
          while (difference != 0U) {
            const int bit_offset = std::countr_zero(difference);
            const std::size_t bit_index =
                word * 64U + static_cast<std::size_t>(bit_offset);
            const int local_z = static_cast<int>(
                bit_index / static_cast<std::size_t>(kChunkSize * kChunkSize));
            const int local_y = static_cast<int>((bit_index / kChunkSize) % kChunkSize);
            const int local_x =
                static_cast<int>(bit_index % static_cast<std::size_t>(kChunkSize));
            if (knownObstacleSourceCanInfluence3D(
                    storage,
                    GridIndex3D{x * kChunkSize + local_x, y * kChunkSize + local_y,
                                z * kChunkSize + local_z})) {
              return false;
            }
            difference &= difference - 1U;
          }
        }
      }
    }
  }
  return true;
}

} // namespace detail

KnownObstacleDistance3DBuildResult updateKnownObstacleDistance3D(
    const ObservedOccupancyGrid3D& occupancy, const GridBounds3D& local_bounds,
    const double maximum_distance_m,
    const std::shared_ptr<const KnownObstacleDistance3D>& previous,
    const ObservedOccupancyGrid3D* const previous_source_occupancy,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks, const bool full_reset,
    const double maximum_rebuild_ratio,
    const std::span<const GridIndex3D> suppressed_source_cells,
    BoundedWorkerPool* const worker_pool) {
  const auto started = std::chrono::steady_clock::now();
  if (!std::isfinite(maximum_rebuild_ratio) || !(maximum_rebuild_ratio > 0.0) ||
      maximum_rebuild_ratio > 1.0) {
    throw std::invalid_argument{
        "invalid known obstacle distance incremental rebuild ratio"};
  }
  auto current_storage = detail::makeKnownObstacleDistanceStorage3D(
      occupancy, local_bounds, maximum_distance_m, suppressed_source_cells);
  const bool previous_compatible = previous != nullptr &&
                                   previous_source_occupancy != nullptr &&
                                   compatiblePreviousField(*previous, *current_storage);
  if (full_reset || !previous_compatible) {
    KnownObstacleDistance3DBuildResult result =
        buildKnownObstacleDistance3D(occupancy, local_bounds, maximum_distance_m,
                                     suppressed_source_cells, worker_pool);
    result.incremental_fallback = !full_reset && previous != nullptr;
    return result;
  }

  const detail::KnownObstacleDistanceStorage3D& previous_storage = *previous->storage_;
  if (!detail::knownObstacleRawChangesCovered3D(*previous_source_occupancy, occupancy,
                                                previous_storage, dirty_chunks)) {
    KnownObstacleDistance3DBuildResult result =
        buildKnownObstacleDistance3D(occupancy, local_bounds, maximum_distance_m,
                                     suppressed_source_cells, worker_pool);
    result.incremental_fallback = true;
    return result;
  }

  const auto source_started = std::chrono::steady_clock::now();
  current_storage =
      std::make_shared<detail::KnownObstacleDistanceStorage3D>(previous_storage);
  current_storage->suppressed_source_cells.assign(suppressed_source_cells.begin(),
                                                  suppressed_source_cells.end());
  std::ranges::sort(current_storage->suppressed_source_cells,
                    [](const GridIndex3D first, const GridIndex3D second) {
                      return std::tuple{first.z, first.y, first.x} <
                             std::tuple{second.z, second.y, second.x};
                    });
  current_storage->suppressed_source_cells.erase(
      std::unique(current_storage->suppressed_source_cells.begin(),
                  current_storage->suppressed_source_cells.end()),
      current_storage->suppressed_source_cells.end());
  const std::unordered_set<std::uint64_t> suppressed_keys =
      detail::knownObstacleSuppressedKeys3D(occupancy.bounds(),
                                            suppressed_source_cells);
  const std::vector<OccupancyChunkIndex3D> source_chunks_to_update =
      affectedSourceChunks(previous_storage, dirty_chunks, suppressed_source_cells);
  std::vector<detail::KnownObstacleSourcePoint3D> inserted;
  std::vector<detail::KnownObstacleSourcePoint3D> removed;
  for (const OccupancyChunkIndex3D chunk_index : source_chunks_to_update) {
    const auto previous_found = previous_storage.source_chunks.find(chunk_index);
    const detail::KnownObstacleSourceChunk3D empty;
    const detail::KnownObstacleSourceChunk3D& previous_sources =
        previous_found != previous_storage.source_chunks.end() ? *previous_found->second
                                                               : empty;
    detail::KnownObstacleSourceChunk3D current_sources =
        detail::collectKnownObstacleSourceChunk3D(occupancy, *current_storage,
                                                  chunk_index, suppressed_keys);
    appendSourceDifference(previous_sources, current_sources, inserted, removed);
    if (current_sources.empty()) {
      current_storage->source_chunks.erase(chunk_index);
    } else if (previous_sources != current_sources) {
      current_storage->source_chunks.insert_or_assign(
          chunk_index, std::make_shared<const detail::KnownObstacleSourceChunk3D>(
                           std::move(current_sources)));
    }
  }

  if (inserted.empty() && removed.empty()) {
    return KnownObstacleDistance3DBuildResult{
        .field = previous,
        .dirty_regions = {},
        .stats =
            KnownObstacleDistance3DBuildStats{
                .source_voxels = previous->sourceVoxelCount(),
                .source_chunks = previous->sourceChunkCount(),
                .stored_distance_chunks = previous->storedDistanceChunkCount(),
                .finite_distance_voxels = previous->finiteDistanceVoxelCount(),
                .reused_chunks = previous->storedDistanceChunkCount(),
                .source_index_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - source_started)
                        .count(),
                .duration_ms = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - started)
                                   .count(),
            },
        .mode = KnownObstacleDistance3DBuildMode::kReused,
        .incremental_fallback = false,
    };
  }

  std::vector<detail::KnownObstacleSourcePoint3D> changed_sources = inserted;
  changed_sources.insert(changed_sources.end(), removed.begin(), removed.end());
  const std::vector<OccupancyChunkIndex3D> affected_output_chunks =
      detail::knownObstacleOutputChunksAffectedBySources3D(*current_storage,
                                                           changed_sources);
  const std::size_t total_voxels = detail::knownObstacleVoxelCount3D(local_bounds);
  const std::size_t affected_voxels =
      affectedVoxelCount(local_bounds, affected_output_chunks);
  if (affected_voxels >= total_voxels ||
      static_cast<double>(affected_voxels) / static_cast<double>(total_voxels) >
          maximum_rebuild_ratio) {
    KnownObstacleDistance3DBuildResult result =
        buildKnownObstacleDistance3D(occupancy, local_bounds, maximum_distance_m,
                                     suppressed_source_cells, worker_pool);
    result.stats.inserted_sources = inserted.size();
    result.stats.removed_sources = removed.size();
    result.incremental_fallback = true;
    return result;
  }

  const detail::KnownObstacleSourceIndex3D source_index{current_storage->source_chunks};
  current_storage->source_voxels = source_index.size();
  current_storage->source_fingerprint = detail::knownObstacleSourceFingerprint3D(
      *current_storage, current_storage->source_chunks);
  const double source_index_ms = std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - source_started)
                                     .count();
  const auto query_started = std::chrono::steady_clock::now();
  const auto computed = detail::computeKnownObstacleDistanceChunks3D(
      *current_storage, source_index, affected_output_chunks, worker_pool);
  std::vector<KnownObstacleDistanceRegion3D> dirty_regions;
  dirty_regions.reserve(affected_output_chunks.size());
  for (const OccupancyChunkIndex3D chunk_index : affected_output_chunks) {
    dirty_regions.push_back(
        detail::knownObstacleChunkRegion3D(local_bounds, chunk_index));
  }
  std::size_t changed_chunks{0U};
  for (std::size_t index = 0U; index < affected_output_chunks.size(); ++index) {
    const OccupancyChunkIndex3D chunk_index = affected_output_chunks[index];
    const auto old_found = previous_storage.distance_chunks.find(chunk_index);
    const std::shared_ptr<const detail::KnownObstacleDistanceChunk3D> old_chunk =
        old_found != previous_storage.distance_chunks.end() ? old_found->second
                                                            : nullptr;
    if (chunksEqual(old_chunk, computed[index])) {
      continue;
    }
    ++changed_chunks;
    current_storage->finite_distance_voxels -=
        old_chunk != nullptr ? old_chunk->finite_voxels : 0U;
    if (computed[index] != nullptr) {
      current_storage->distance_chunks.insert_or_assign(chunk_index, computed[index]);
      current_storage->finite_distance_voxels += computed[index]->finite_voxels;
    } else {
      current_storage->distance_chunks.erase(chunk_index);
    }
  }
  const double query_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - query_started)
                              .count();
  const std::size_t stored_chunks = current_storage->distance_chunks.size();
  const std::size_t reused_chunks = static_cast<std::size_t>(std::ranges::count_if(
      current_storage->distance_chunks, [&previous_storage](const auto& entry) {
        const auto previous_chunk = previous_storage.distance_chunks.find(entry.first);
        return previous_chunk != previous_storage.distance_chunks.end() &&
               previous_chunk->second == entry.second;
      }));
  KnownObstacleDistance3DBuildResult result{
      .field = KnownObstacleDistance3D::create(current_storage),
      .dirty_regions = std::move(dirty_regions),
      .stats =
          KnownObstacleDistance3DBuildStats{
              .source_voxels = current_storage->source_voxels,
              .source_chunks = current_storage->source_chunks.size(),
              .stored_distance_chunks = stored_chunks,
              .finite_distance_voxels = current_storage->finite_distance_voxels,
              .inserted_sources = inserted.size(),
              .removed_sources = removed.size(),
              .recomputed_chunks = affected_output_chunks.size(),
              .reused_chunks = reused_chunks,
              .changed_chunks = changed_chunks,
              .queried_voxels = affected_voxels,
              .source_index_ms = source_index_ms,
              .distance_query_ms = query_ms,
          },
      .mode = KnownObstacleDistance3DBuildMode::kIncremental,
      .incremental_fallback = false,
  };
  result.stats.duration_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
  return result;
}

} // namespace drone_city_nav
