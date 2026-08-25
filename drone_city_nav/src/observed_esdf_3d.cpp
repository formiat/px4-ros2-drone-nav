#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>

#include "observed_esdf_3d_regions.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool sameResolution(const GridBounds3D& first,
                                  const GridBounds3D& second) noexcept {
  return std::abs(first.resolution_m - second.resolution_m) <= 1.0e-9;
}

[[nodiscard]] int alignedCellOffset(const double local_origin,
                                    const double world_origin,
                                    const double resolution_m) {
  const double offset = (local_origin - world_origin) / resolution_m;
  const double rounded = std::round(offset);
  if (std::abs(offset - rounded) > 1.0e-6) {
    throw std::invalid_argument{"observed ESDF bounds are not cell aligned"};
  }
  return static_cast<int>(rounded);
}

struct SourceCellRegion {
  int minimum_x{0};
  int minimum_y{0};
  int minimum_z{0};
  int maximum_x_exclusive{0};
  int maximum_y_exclusive{0};
  int maximum_z_exclusive{0};
};

[[nodiscard]] SourceCellRegion sourceCellRegion(const GridBounds3D& world,
                                                const GridBounds3D& local) {
  if (!sameResolution(world, local) || local.width_cells <= 0 ||
      local.height_cells <= 0 || local.depth_cells <= 0) {
    throw std::invalid_argument{"invalid observed ESDF local bounds"};
  }
  const int minimum_x =
      alignedCellOffset(local.origin_x, world.origin_x, world.resolution_m);
  const int minimum_y =
      alignedCellOffset(local.origin_y, world.origin_y, world.resolution_m);
  const int minimum_z =
      alignedCellOffset(local.origin_z, world.origin_z, world.resolution_m);
  const SourceCellRegion region{
      .minimum_x = minimum_x,
      .minimum_y = minimum_y,
      .minimum_z = minimum_z,
      .maximum_x_exclusive = minimum_x + local.width_cells,
      .maximum_y_exclusive = minimum_y + local.height_cells,
      .maximum_z_exclusive = minimum_z + local.depth_cells,
  };
  if (region.minimum_x < 0 || region.minimum_y < 0 || region.minimum_z < 0 ||
      region.maximum_x_exclusive > world.width_cells ||
      region.maximum_y_exclusive > world.height_cells ||
      region.maximum_z_exclusive > world.depth_cells) {
    throw std::invalid_argument{"observed ESDF local bounds exceed world bounds"};
  }
  return region;
}

[[nodiscard]] bool overlaps(const OccupancyChunkIndex3D& chunk,
                            const SourceCellRegion& region) noexcept {
  const int chunk_minimum_x = chunk.x * ObservedOccupancyGrid3D::kChunkSize;
  const int chunk_minimum_y = chunk.y * ObservedOccupancyGrid3D::kChunkSize;
  const int chunk_minimum_z = chunk.z * ObservedOccupancyGrid3D::kChunkSize;
  const int chunk_maximum_x = chunk_minimum_x + ObservedOccupancyGrid3D::kChunkSize;
  const int chunk_maximum_y = chunk_minimum_y + ObservedOccupancyGrid3D::kChunkSize;
  const int chunk_maximum_z = chunk_minimum_z + ObservedOccupancyGrid3D::kChunkSize;
  return chunk_maximum_x > region.minimum_x &&
         chunk_minimum_x < region.maximum_x_exclusive &&
         chunk_maximum_y > region.minimum_y &&
         chunk_minimum_y < region.maximum_y_exclusive &&
         chunk_maximum_z > region.minimum_z &&
         chunk_minimum_z < region.maximum_z_exclusive;
}

[[nodiscard]] std::size_t localLinearIndex(const GridBounds3D& bounds,
                                           const GridIndex3D cell) noexcept {
  return (static_cast<std::size_t>(cell.z) *
              static_cast<std::size_t>(bounds.height_cells) +
          static_cast<std::size_t>(cell.y)) *
             static_cast<std::size_t>(bounds.width_cells) +
         static_cast<std::size_t>(cell.x);
}

template<typename Callback>
void forEachObservedVoxel(const ObservedOccupancyGrid3D& occupancy,
                          const SourceCellRegion& region, Callback callback) {
  for (const auto& [chunk_index, storage] : occupancy.chunks()) {
    const ObservedOccupancyGrid3D::Chunk& chunk = storage.get();
    if (!overlaps(chunk_index, region)) {
      continue;
    }
    for (std::size_t word_index = 0U; word_index < chunk.observed.size();
         ++word_index) {
      std::uint64_t observed_bits = chunk.observed.at(word_index);
      while (observed_bits != 0U) {
        const int bit_offset = std::countr_zero(observed_bits);
        const std::size_t bit_index =
            word_index * 64U + static_cast<std::size_t>(bit_offset);
        const int local_x = static_cast<int>(
            bit_index % static_cast<std::size_t>(ObservedOccupancyGrid3D::kChunkSize));
        const int local_y =
            static_cast<int>((bit_index / ObservedOccupancyGrid3D::kChunkSize) %
                             ObservedOccupancyGrid3D::kChunkSize);
        const int local_z = static_cast<int>(
            bit_index / static_cast<std::size_t>(ObservedOccupancyGrid3D::kChunkSize *
                                                 ObservedOccupancyGrid3D::kChunkSize));
        const GridIndex3D source{
            chunk_index.x * ObservedOccupancyGrid3D::kChunkSize + local_x,
            chunk_index.y * ObservedOccupancyGrid3D::kChunkSize + local_y,
            chunk_index.z * ObservedOccupancyGrid3D::kChunkSize + local_z,
        };
        if (source.x >= region.minimum_x && source.x < region.maximum_x_exclusive &&
            source.y >= region.minimum_y && source.y < region.maximum_y_exclusive &&
            source.z >= region.minimum_z && source.z < region.maximum_z_exclusive) {
          const bool occupied =
              (chunk.occupied.at(word_index) &
               (std::uint64_t{1U} << static_cast<unsigned int>(bit_offset))) != 0U;
          callback(source, occupied ? ObservedVoxelState::kOccupied
                                    : ObservedVoxelState::kFree);
        }
        observed_bits &= observed_bits - 1U;
      }
    }
  }
}

[[nodiscard]] std::uint64_t cellKey(const GridBounds3D& bounds,
                                    const GridIndex3D cell) noexcept {
  return (static_cast<std::uint64_t>(cell.z) *
              static_cast<std::uint64_t>(bounds.height_cells) +
          static_cast<std::uint64_t>(cell.y)) *
             static_cast<std::uint64_t>(bounds.width_cells) +
         static_cast<std::uint64_t>(cell.x);
}

[[nodiscard]] std::vector<GridIndex3D>
launchSupportCells(const ObservedOccupancyGrid3D& occupancy,
                   const LaunchSupportContact3D* const launch_support_contact) {
  std::vector<GridIndex3D> result;
  if (launch_support_contact == nullptr) {
    return result;
  }
  result.reserve(launch_support_contact->contact_cells.size());
  for (const AxisAlignedBox3D& box : launch_support_contact->contact_cells) {
    const Point3 center{0.5 * (box.minimum.x + box.maximum.x),
                        0.5 * (box.minimum.y + box.maximum.y),
                        0.5 * (box.minimum.z + box.maximum.z)};
    const std::optional<GridIndex3D> cell = occupancy.worldToCell(center);
    if (cell) {
      result.push_back(*cell);
    }
  }
  return result;
}

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  constexpr double kTolerance{1.0e-9};
  return std::abs(first.origin_x - second.origin_x) <= kTolerance &&
         std::abs(first.origin_y - second.origin_y) <= kTolerance &&
         std::abs(first.origin_z - second.origin_z) <= kTolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= kTolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

[[nodiscard]] std::size_t voxelCount(const GridBounds3D& bounds) noexcept {
  return static_cast<std::size_t>(bounds.width_cells) *
         static_cast<std::size_t>(bounds.height_cells) *
         static_cast<std::size_t>(bounds.depth_cells);
}

[[nodiscard]] mppi::EsdfGrid esdfGrid(const GridBounds3D& bounds) noexcept {
  return mppi::EsdfGrid{
      .width = bounds.width_cells,
      .height = bounds.height_cells,
      .resolution_m = static_cast<float>(bounds.resolution_m),
      .origin_x_m = static_cast<float>(bounds.origin_x),
      .origin_y_m = static_cast<float>(bounds.origin_y),
      .depth = bounds.depth_cells,
      .origin_z_m = static_cast<float>(bounds.origin_z),
      .outside_is_unknown = true,
  };
}

struct ClassifiedObservedGrid3D {
  std::shared_ptr<ObservedOccupancyGrid3D> occupancy;
  std::vector<GridIndex3D> override_cells;
  std::uint64_t fingerprint{0U};
  ObservedEsdf3DBuildStats stats{};
};

[[nodiscard]] ClassifiedObservedGrid3D
classifyObservedGrid3D(const ObservedOccupancyGrid3D& occupancy,
                       const GridBounds3D& local_bounds,
                       const LaunchSupportContact3D* const launch_support_contact) {
  const auto started = std::chrono::steady_clock::now();
  const SourceCellRegion source_region =
      sourceCellRegion(occupancy.bounds(), local_bounds);
  auto local_occupancy = std::make_shared<ObservedOccupancyGrid3D>(local_bounds);
  ObservedEsdf3DBuildStats stats;
  const std::vector<GridIndex3D> support_cells =
      launchSupportCells(occupancy, launch_support_contact);
  std::unordered_set<std::uint64_t> support_cell_keys;
  support_cell_keys.reserve(support_cells.size());
  for (const GridIndex3D cell : support_cells) {
    support_cell_keys.insert(cellKey(occupancy.bounds(), cell));
  }
  const auto sourceToLocal = [&source_region](const GridIndex3D source) {
    return GridIndex3D{source.x - source_region.minimum_x,
                       source.y - source_region.minimum_y,
                       source.z - source_region.minimum_z};
  };
  std::vector<GridIndex3D> override_cells;
  override_cells.reserve(support_cells.size());
  forEachObservedVoxel(
      occupancy, source_region,
      [&](const GridIndex3D source, const ObservedVoxelState state) {
        const bool launch_support_cell =
            support_cell_keys.contains(cellKey(occupancy.bounds(), source));
        static_cast<void>(local_occupancy->setState(
            sourceToLocal(source),
            launch_support_cell ? ObservedVoxelState::kFree : state));
        stats.launch_support_voxels +=
            launch_support_cell && state != ObservedVoxelState::kFree ? 1U : 0U;
      });

  for (const GridIndex3D source : support_cells) {
    if (source.x < source_region.minimum_x ||
        source.x >= source_region.maximum_x_exclusive ||
        source.y < source_region.minimum_y ||
        source.y >= source_region.maximum_y_exclusive ||
        source.z < source_region.minimum_z ||
        source.z >= source_region.maximum_z_exclusive) {
      continue;
    }
    const GridIndex3D local_cell = sourceToLocal(source);
    override_cells.push_back(local_cell);
    if (local_occupancy->state(local_cell) != ObservedVoxelState::kFree) {
      static_cast<void>(
          local_occupancy->setState(local_cell, ObservedVoxelState::kFree));
      ++stats.launch_support_voxels;
    }
  }

  stats.known_voxels = local_occupancy->knownVoxelCount();
  stats.free_voxels = local_occupancy->freeVoxelCount();
  stats.occupied_voxels = local_occupancy->occupiedVoxelCount();
  stats.unknown_voxels = voxelCount(local_bounds) - stats.known_voxels;
  stats.classified_voxels = voxelCount(local_bounds);
  stats.classification_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
  const std::uint64_t fingerprint =
      observedOccupancyFingerprint(*local_occupancy, local_bounds);
  return {.occupancy = std::move(local_occupancy),
          .override_cells = std::move(override_cells),
          .fingerprint = fingerprint,
          .stats = stats};
}

[[nodiscard]] GridIndex3D localCellFromLinear(const GridBounds3D& bounds,
                                              const std::size_t index) noexcept {
  const std::size_t width = static_cast<std::size_t>(bounds.width_cells);
  const std::size_t height = static_cast<std::size_t>(bounds.height_cells);
  const std::size_t plane = width * height;
  return GridIndex3D{
      .x = static_cast<int>(index % width),
      .y = static_cast<int>((index / width) % height),
      .z = static_cast<int>(index / plane),
  };
}

[[nodiscard]] ClassifiedObservedGrid3D classifyObservedGrid3DIncremental(
    const ObservedOccupancyGrid3D& occupancy, const GridBounds3D& local_bounds,
    const ObservedOccupancyGrid3D& previous_local_occupancy,
    const std::span<const GridIndex3D> previous_override_cells,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks,
    const LaunchSupportContact3D* const launch_support_contact) {
  const auto started = std::chrono::steady_clock::now();
  const SourceCellRegion source_region =
      sourceCellRegion(occupancy.bounds(), local_bounds);
  auto local_occupancy =
      std::make_shared<ObservedOccupancyGrid3D>(previous_local_occupancy);
  std::unordered_set<std::size_t> affected_cells;
  affected_cells.reserve(dirty_chunks.size() * 64U + previous_override_cells.size());
  for (const GridIndex3D cell : previous_override_cells) {
    if (local_occupancy->contains(cell)) {
      affected_cells.insert(localLinearIndex(local_bounds, cell));
    }
  }

  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  for (const OccupancyChunkIndex3D chunk : dirty_chunks) {
    const int minimum_x = std::max(source_region.minimum_x, chunk.x * kChunkSize);
    const int minimum_y = std::max(source_region.minimum_y, chunk.y * kChunkSize);
    const int minimum_z = std::max(source_region.minimum_z, chunk.z * kChunkSize);
    const int maximum_x =
        std::min(source_region.maximum_x_exclusive, (chunk.x + 1) * kChunkSize);
    const int maximum_y =
        std::min(source_region.maximum_y_exclusive, (chunk.y + 1) * kChunkSize);
    const int maximum_z =
        std::min(source_region.maximum_z_exclusive, (chunk.z + 1) * kChunkSize);
    for (int z = minimum_z; z < maximum_z; ++z) {
      for (int y = minimum_y; y < maximum_y; ++y) {
        for (int x = minimum_x; x < maximum_x; ++x) {
          affected_cells.insert(
              localLinearIndex(local_bounds, GridIndex3D{x - source_region.minimum_x,
                                                         y - source_region.minimum_y,
                                                         z - source_region.minimum_z}));
        }
      }
    }
  }

  std::vector<GridIndex3D> override_cells;
  const std::vector<GridIndex3D> support_cells =
      launchSupportCells(occupancy, launch_support_contact);
  override_cells.reserve(support_cells.size());
  std::unordered_set<std::size_t> override_indices;
  override_indices.reserve(support_cells.size());
  for (const GridIndex3D source : support_cells) {
    if (source.x < source_region.minimum_x ||
        source.x >= source_region.maximum_x_exclusive ||
        source.y < source_region.minimum_y ||
        source.y >= source_region.maximum_y_exclusive ||
        source.z < source_region.minimum_z ||
        source.z >= source_region.maximum_z_exclusive) {
      continue;
    }
    const GridIndex3D local{source.x - source_region.minimum_x,
                            source.y - source_region.minimum_y,
                            source.z - source_region.minimum_z};
    const std::size_t index = localLinearIndex(local_bounds, local);
    override_cells.push_back(local);
    override_indices.insert(index);
    affected_cells.insert(index);
  }

  ObservedEsdf3DBuildStats stats;
  for (const std::size_t index : affected_cells) {
    const GridIndex3D local = localCellFromLinear(local_bounds, index);
    const GridIndex3D source{local.x + source_region.minimum_x,
                             local.y + source_region.minimum_y,
                             local.z + source_region.minimum_z};
    const ObservedVoxelState raw_state = occupancy.state(source);
    const bool overridden = override_indices.contains(index);
    static_cast<void>(local_occupancy->setState(
        local, overridden ? ObservedVoxelState::kFree : raw_state));
    stats.launch_support_voxels +=
        overridden && raw_state != ObservedVoxelState::kFree ? 1U : 0U;
  }
  stats.known_voxels = local_occupancy->knownVoxelCount();
  stats.free_voxels = local_occupancy->freeVoxelCount();
  stats.occupied_voxels = local_occupancy->occupiedVoxelCount();
  const std::size_t total_voxels = voxelCount(local_bounds);
  stats.unknown_voxels = total_voxels - stats.known_voxels;
  stats.classified_voxels = affected_cells.size();
  stats.reused_classification_voxels = total_voxels - affected_cells.size();
  stats.classification_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
  const std::uint64_t fingerprint =
      observedOccupancyFingerprint(*local_occupancy, local_bounds);
  return ClassifiedObservedGrid3D{
      .occupancy = std::move(local_occupancy),
      .override_cells = std::move(override_cells),
      .fingerprint = fingerprint,
      .stats = stats,
  };
}

[[nodiscard]] ObservedEsdf3D buildFullObservedEsdf3D(
    const ClassifiedObservedGrid3D& classified, const double maximum_distance_m,
    BoundedWorkerPool* const worker_pool, const std::size_t dirty_chunks,
    const bool incremental_fallback) {
  const GridBounds3D& bounds = classified.occupancy->bounds();
  const OccupancyGrid3D occupied = classified.occupancy->occupiedSnapshot();
  const DistanceField3D field =
      DistanceField3D::build(occupied, maximum_distance_m, worker_pool);
  ObservedEsdf3D result{
      .grid = esdfGrid(bounds),
      .distances_m =
          std::vector<float>(voxelCount(bounds), mppi::kUnknownEsdfDistanceM),
      .nearest_obstacle_indices =
          std::vector<std::size_t>(field.nearestSourceLinearIndices().begin(),
                                   field.nearestSourceLinearIndices().end()),
      .local_occupancy = classified.occupancy,
      .classification_override_cells = classified.override_cells,
      .dirty_regions = {},
      .occupancy_fingerprint = classified.fingerprint,
      .maximum_distance_m = maximum_distance_m,
      .stats = classified.stats,
  };
  const SourceCellRegion region = sourceCellRegion(bounds, bounds);
  forEachObservedVoxel(*classified.occupancy, region,
                       [&](const GridIndex3D cell, const ObservedVoxelState) {
                         result.distances_m.at(localLinearIndex(bounds, cell)) =
                             field.distanceAt(cell);
                       });
  result.stats.distance_field = field.stats();
  result.stats.recomputed_voxels = result.distances_m.size();
  result.stats.dirty_chunks = dirty_chunks;
  result.stats.mode = ObservedEsdf3DBuildMode::kFull;
  result.stats.incremental_fallback = incremental_fallback;
  return result;
}

[[nodiscard]] GridIndex3D chunkCell(const OccupancyChunkIndex3D chunk_index,
                                    const std::size_t bit_index) noexcept {
  return GridIndex3D{
      chunk_index.x * ObservedOccupancyGrid3D::kChunkSize +
          static_cast<int>(bit_index % ObservedOccupancyGrid3D::kChunkSize),
      chunk_index.y * ObservedOccupancyGrid3D::kChunkSize +
          static_cast<int>((bit_index / ObservedOccupancyGrid3D::kChunkSize) %
                           ObservedOccupancyGrid3D::kChunkSize),
      chunk_index.z * ObservedOccupancyGrid3D::kChunkSize +
          static_cast<int>(bit_index / static_cast<std::size_t>(
                                           ObservedOccupancyGrid3D::kChunkSize *
                                           ObservedOccupancyGrid3D::kChunkSize)),
  };
}

[[nodiscard]] bool inside(const GridIndex3D cell,
                          const SourceCellRegion& region) noexcept {
  return cell.x >= region.minimum_x && cell.x < region.maximum_x_exclusive &&
         cell.y >= region.minimum_y && cell.y < region.maximum_y_exclusive &&
         cell.z >= region.minimum_z && cell.z < region.maximum_z_exclusive;
}

void includeChangedCell(ChangedCellRegion3D& region, const GridIndex3D cell) {
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
}

[[nodiscard]] ChangedCellRegion3D
changedCellRegion3D(const ObservedOccupancyGrid3D& previous,
                    const ObservedOccupancyGrid3D& current) {
  std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash> chunk_indices;
  chunk_indices.reserve(previous.chunks().size() + current.chunks().size());
  for (const auto& [index, storage] : previous.chunks()) {
    static_cast<void>(storage);
    chunk_indices.insert(index);
  }
  for (const auto& [index, storage] : current.chunks()) {
    static_cast<void>(storage);
    chunk_indices.insert(index);
  }
  ChangedCellRegion3D result;
  for (const OccupancyChunkIndex3D chunk_index : chunk_indices) {
    const ObservedOccupancyGrid3D::Chunk* const before =
        previous.findChunk(chunk_index);
    const ObservedOccupancyGrid3D::Chunk* const after = current.findChunk(chunk_index);
    for (std::size_t word = 0U; word < OccupancyGrid3D::kWordsPerChunk; ++word) {
      const std::uint64_t before_observed =
          before != nullptr ? before->observed.at(word) : 0U;
      const std::uint64_t before_occupied =
          before != nullptr ? before->occupied.at(word) : 0U;
      const std::uint64_t after_observed =
          after != nullptr ? after->observed.at(word) : 0U;
      const std::uint64_t after_occupied =
          after != nullptr ? after->occupied.at(word) : 0U;
      std::uint64_t changed =
          (before_observed ^ after_observed) | (before_occupied ^ after_occupied);
      while (changed != 0U) {
        const int bit_offset = std::countr_zero(changed);
        const std::size_t bit_index = word * 64U + static_cast<std::size_t>(bit_offset);
        const GridIndex3D cell = chunkCell(chunk_index, bit_index);
        if (current.contains(cell)) {
          includeChangedCell(result, cell);
        }
        changed &= changed - 1U;
      }
    }
  }
  return result;
}

[[nodiscard]] bool rawChangesCoveredByDirtyChunks(
    const ObservedOccupancyGrid3D& previous, const ObservedOccupancyGrid3D& current,
    const GridBounds3D& local_bounds,
    const std::span<const OccupancyChunkIndex3D> dirty_chunk_span) {
  if (!sameBounds(previous.bounds(), current.bounds())) {
    return false;
  }
  const SourceCellRegion region = sourceCellRegion(current.bounds(), local_bounds);
  const std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash>
      dirty_chunks{dirty_chunk_span.begin(), dirty_chunk_span.end()};
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  const OccupancyChunkIndex3D first{region.minimum_x / kChunkSize,
                                    region.minimum_y / kChunkSize,
                                    region.minimum_z / kChunkSize};
  const OccupancyChunkIndex3D last{(region.maximum_x_exclusive - 1) / kChunkSize,
                                   (region.maximum_y_exclusive - 1) / kChunkSize,
                                   (region.maximum_z_exclusive - 1) / kChunkSize};
  for (int z = first.z; z <= last.z; ++z) {
    for (int y = first.y; y <= last.y; ++y) {
      for (int x = first.x; x <= last.x; ++x) {
        const OccupancyChunkIndex3D chunk_index{x, y, z};
        if (dirty_chunks.contains(chunk_index)) {
          continue;
        }
        const ObservedOccupancyGrid3D::Chunk* const before =
            previous.findChunk(chunk_index);
        const ObservedOccupancyGrid3D::Chunk* const after =
            current.findChunk(chunk_index);
        for (std::size_t word = 0U; word < OccupancyGrid3D::kWordsPerChunk; ++word) {
          const std::uint64_t before_observed =
              before != nullptr ? before->observed.at(word) : 0U;
          const std::uint64_t before_occupied =
              before != nullptr ? before->occupied.at(word) : 0U;
          const std::uint64_t after_observed =
              after != nullptr ? after->observed.at(word) : 0U;
          const std::uint64_t after_occupied =
              after != nullptr ? after->occupied.at(word) : 0U;
          std::uint64_t changed =
              (before_observed ^ after_observed) | (before_occupied ^ after_occupied);
          while (changed != 0U) {
            const int bit_offset = std::countr_zero(changed);
            const std::size_t bit_index =
                word * 64U + static_cast<std::size_t>(bit_offset);
            if (inside(chunkCell(chunk_index, bit_index), region)) {
              return false;
            }
            changed &= changed - 1U;
          }
        }
      }
    }
  }
  return true;
}

[[nodiscard]] bool
previousObservedEsdfIsCompatible(const PreviousObservedEsdf3D& previous,
                                 const ObservedOccupancyGrid3D& source_occupancy,
                                 const GridBounds3D& bounds,
                                 const double maximum_distance_m) {
  if (!previous.source_occupancy || !previous.local_occupancy ||
      !sameBounds(previous.source_occupancy->bounds(), source_occupancy.bounds()) ||
      !sameBounds(previous.local_occupancy->bounds(), bounds) ||
      previous.distances_m.size() != voxelCount(bounds) ||
      previous.nearest_obstacle_indices.size() != voxelCount(bounds) ||
      previous.occupancy_fingerprint == 0U ||
      std::abs(previous.maximum_distance_m - maximum_distance_m) > 1.0e-9) {
    return false;
  }
  const mppi::EsdfGrid expected = esdfGrid(bounds);
  constexpr float kGridTolerance{1.0e-5F};
  return previous.grid.width == expected.width &&
         previous.grid.height == expected.height &&
         previous.grid.depth == expected.depth && previous.grid.outside_is_unknown &&
         std::abs(previous.grid.resolution_m - expected.resolution_m) <=
             kGridTolerance &&
         std::abs(previous.grid.origin_x_m - expected.origin_x_m) <= kGridTolerance &&
         std::abs(previous.grid.origin_y_m - expected.origin_y_m) <= kGridTolerance &&
         std::abs(previous.grid.origin_z_m - expected.origin_z_m) <= kGridTolerance &&
         observedOccupancyFingerprint(*previous.local_occupancy, bounds) ==
             previous.occupancy_fingerprint;
}

[[nodiscard]] SourceCellRegion expandedRegion(const SourceCellRegion& region,
                                              const int cells,
                                              const GridBounds3D& bounds) noexcept {
  return SourceCellRegion{
      .minimum_x = std::max(0, region.minimum_x - cells),
      .minimum_y = std::max(0, region.minimum_y - cells),
      .minimum_z = std::max(0, region.minimum_z - cells),
      .maximum_x_exclusive =
          std::min(bounds.width_cells, region.maximum_x_exclusive + cells),
      .maximum_y_exclusive =
          std::min(bounds.height_cells, region.maximum_y_exclusive + cells),
      .maximum_z_exclusive =
          std::min(bounds.depth_cells, region.maximum_z_exclusive + cells),
  };
}

} // namespace

ObservedEsdf3D
buildObservedEsdf3D(const ObservedOccupancyGrid3D& occupancy,
                    const GridBounds3D& local_bounds, const double maximum_distance_m,
                    BoundedWorkerPool* const worker_pool,
                    const LaunchSupportContact3D* const launch_support_contact) {
  return updateObservedEsdf3D(occupancy, local_bounds, maximum_distance_m, nullptr, {},
                              true, 1.0, worker_pool, launch_support_contact);
}

ObservedEsdf3D updateObservedEsdf3D(
    const ObservedOccupancyGrid3D& occupancy, const GridBounds3D& local_bounds,
    const double maximum_distance_m, const PreviousObservedEsdf3D* const previous,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks, const bool full_reset,
    const double maximum_rebuild_ratio, BoundedWorkerPool* const worker_pool,
    const LaunchSupportContact3D* const launch_support_contact) {
  static_cast<void>(sourceCellRegion(occupancy.bounds(), local_bounds));
  if (!std::isfinite(maximum_distance_m) || maximum_distance_m <= 0.0 ||
      !std::isfinite(maximum_rebuild_ratio) || maximum_rebuild_ratio <= 0.0 ||
      maximum_rebuild_ratio > 1.0) {
    throw std::invalid_argument{"invalid incremental observed ESDF request"};
  }
  const bool previous_compatible =
      previous != nullptr &&
      previousObservedEsdfIsCompatible(*previous, occupancy, local_bounds,
                                       maximum_distance_m);
  if (full_reset || !previous_compatible) {
    const ClassifiedObservedGrid3D classified =
        classifyObservedGrid3D(occupancy, local_bounds, launch_support_contact);
    return buildFullObservedEsdf3D(classified, maximum_distance_m, worker_pool,
                                   dirty_chunks.size(), previous != nullptr);
  }
  if (!rawChangesCoveredByDirtyChunks(*previous->source_occupancy, occupancy,
                                      local_bounds, dirty_chunks)) {
    const ClassifiedObservedGrid3D classified =
        classifyObservedGrid3D(occupancy, local_bounds, launch_support_contact);
    const ChangedCellRegion3D changed =
        changedCellRegion3D(*previous->local_occupancy, *classified.occupancy);
    ObservedEsdf3D result = buildFullObservedEsdf3D(
        classified, maximum_distance_m, worker_pool, dirty_chunks.size(), true);
    result.stats.changed_voxels = changed.count;
    return result;
  }
  const ClassifiedObservedGrid3D classified = classifyObservedGrid3DIncremental(
      occupancy, local_bounds, *previous->local_occupancy,
      previous->classification_override_cells, dirty_chunks, launch_support_contact);
  const ChangedCellRegion3D changed =
      changedCellRegion3D(*previous->local_occupancy, *classified.occupancy);
  if (changed.count == 0U) {
    ObservedEsdf3D result{
        .grid = previous->grid,
        .distances_m = std::vector<float>(previous->distances_m.begin(),
                                          previous->distances_m.end()),
        .nearest_obstacle_indices =
            std::vector<std::size_t>(previous->nearest_obstacle_indices.begin(),
                                     previous->nearest_obstacle_indices.end()),
        .local_occupancy = previous->local_occupancy,
        .classification_override_cells = classified.override_cells,
        .dirty_regions = {},
        .occupancy_fingerprint = classified.fingerprint,
        .maximum_distance_m = maximum_distance_m,
        .stats = classified.stats,
    };
    result.stats.reused_voxels = result.distances_m.size();
    result.stats.dirty_chunks = dirty_chunks.size();
    result.stats.mode = ObservedEsdf3DBuildMode::kReused;
    return result;
  }

  const double radius_cells_value =
      std::ceil(maximum_distance_m / local_bounds.resolution_m);
  if (radius_cells_value > static_cast<double>(std::numeric_limits<int>::max())) {
    throw std::overflow_error{"incremental observed ESDF radius exceeds int"};
  }
  const int radius_cells = static_cast<int>(radius_cells_value);
  const double maximum_squared_cells =
      std::pow(maximum_distance_m / local_bounds.resolution_m, 2.0);
  const std::size_t total_voxels = voxelCount(local_bounds);
  std::unordered_set<std::size_t> inserted_sources;
  std::unordered_set<std::size_t> removed_sources;
  std::vector<std::uint8_t> recompute_mask(total_voxels, 0U);
  std::size_t scheduled_voxels{0U};
  const auto schedule = [&](const std::size_t index) {
    if (recompute_mask[index] == 0U) {
      recompute_mask[index] = 1U;
      ++scheduled_voxels;
    }
  };
  for (const GridIndex3D cell : changed.cells) {
    const std::size_t index = localLinearIndex(local_bounds, cell);
    const ObservedVoxelState before = previous->local_occupancy->state(cell);
    const ObservedVoxelState after = classified.occupancy->state(cell);
    if (before == ObservedVoxelState::kOccupied &&
        after != ObservedVoxelState::kOccupied) {
      removed_sources.insert(index);
    }
    if (before != ObservedVoxelState::kOccupied &&
        after == ObservedVoxelState::kOccupied) {
      inserted_sources.insert(index);
    }
    schedule(index);
  }
  for (std::size_t index = 0U; index < total_voxels; ++index) {
    if (removed_sources.contains(previous->nearest_obstacle_indices[index])) {
      schedule(index);
    }
  }
  for (const std::size_t source_index : inserted_sources) {
    const GridIndex3D source = localCellFromLinear(local_bounds, source_index);
    const SourceCellRegion affected =
        expandedRegion(SourceCellRegion{.minimum_x = source.x,
                                        .minimum_y = source.y,
                                        .minimum_z = source.z,
                                        .maximum_x_exclusive = source.x + 1,
                                        .maximum_y_exclusive = source.y + 1,
                                        .maximum_z_exclusive = source.z + 1},
                       radius_cells, local_bounds);
    for (int z = affected.minimum_z; z < affected.maximum_z_exclusive; ++z) {
      for (int y = affected.minimum_y; y < affected.maximum_y_exclusive; ++y) {
        for (int x = affected.minimum_x; x < affected.maximum_x_exclusive; ++x) {
          const double dx = static_cast<double>(x - source.x);
          const double dy = static_cast<double>(y - source.y);
          const double dz = static_cast<double>(z - source.z);
          if (dx * dx + dy * dy + dz * dz <= maximum_squared_cells) {
            schedule(localLinearIndex(local_bounds, GridIndex3D{x, y, z}));
          }
        }
      }
    }
  }
  if (scheduled_voxels >= total_voxels ||
      static_cast<double>(scheduled_voxels) / static_cast<double>(total_voxels) >
          maximum_rebuild_ratio) {
    ObservedEsdf3D result = buildFullObservedEsdf3D(
        classified, maximum_distance_m, worker_pool, dirty_chunks.size(), true);
    result.stats.changed_voxels = changed.count;
    return result;
  }

  const auto dynamic_started = std::chrono::steady_clock::now();
  ObservedEsdf3D result{
      .grid = esdfGrid(local_bounds),
      .distances_m = std::vector<float>(previous->distances_m.begin(),
                                        previous->distances_m.end()),
      .nearest_obstacle_indices =
          std::vector<std::size_t>(previous->nearest_obstacle_indices.begin(),
                                   previous->nearest_obstacle_indices.end()),
      .local_occupancy = classified.occupancy,
      .classification_override_cells = classified.override_cells,
      .dirty_regions = {},
      .occupancy_fingerprint = classified.fingerprint,
      .maximum_distance_m = maximum_distance_m,
      .stats = classified.stats,
  };
  std::unordered_set<std::size_t> replacement_sources = inserted_sources;
  constexpr std::array<int, 3U> kOffsets{-1, 0, 1};
  for (std::size_t index = 0U; index < total_voxels; ++index) {
    if (recompute_mask[index] == 0U ||
        !removed_sources.contains(previous->nearest_obstacle_indices[index])) {
      continue;
    }
    const GridIndex3D cell = localCellFromLinear(local_bounds, index);
    if (classified.occupancy->isOccupied(cell)) {
      replacement_sources.insert(index);
    }
    for (const int dz : kOffsets) {
      for (const int dy : kOffsets) {
        for (const int dx : kOffsets) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridIndex3D neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
          if (!classified.occupancy->contains(neighbor)) {
            continue;
          }
          const std::size_t neighbor_index = localLinearIndex(local_bounds, neighbor);
          if (recompute_mask[neighbor_index] != 0U &&
              removed_sources.contains(
                  previous->nearest_obstacle_indices[neighbor_index])) {
            continue;
          }
          const std::size_t source = previous->nearest_obstacle_indices[neighbor_index];
          if (source != DistanceField3D::kNoNearestSource && source < total_voxels &&
              classified.occupancy->isOccupied(
                  localCellFromLinear(local_bounds, source))) {
            replacement_sources.insert(source);
          }
        }
      }
    }
  }

  std::vector<GridIndex3D> replacement_source_cells;
  replacement_source_cells.reserve(replacement_sources.size());
  for (const std::size_t source : replacement_sources) {
    replacement_source_cells.push_back(localCellFromLinear(local_bounds, source));
  }
  std::size_t dependency_invalidated_voxels{0U};
  for (std::size_t index = 0U; index < total_voxels; ++index) {
    if (recompute_mask[index] == 0U ||
        !removed_sources.contains(previous->nearest_obstacle_indices[index])) {
      continue;
    }
    ++dependency_invalidated_voxels;
    const GridIndex3D cell = localCellFromLinear(local_bounds, index);
    double best_squared_cells = std::numeric_limits<double>::infinity();
    std::size_t best_source = DistanceField3D::kNoNearestSource;
    for (const GridIndex3D source : replacement_source_cells) {
      const double dx = static_cast<double>(cell.x - source.x);
      const double dy = static_cast<double>(cell.y - source.y);
      const double dz = static_cast<double>(cell.z - source.z);
      const double squared_cells = dx * dx + dy * dy + dz * dz;
      if (squared_cells < best_squared_cells) {
        best_squared_cells = squared_cells;
        best_source = localLinearIndex(local_bounds, source);
      }
    }
    if (best_squared_cells <= maximum_squared_cells) {
      result.distances_m[index] =
          static_cast<float>(std::sqrt(best_squared_cells) * local_bounds.resolution_m);
      result.nearest_obstacle_indices[index] = best_source;
    } else {
      result.distances_m[index] = std::numeric_limits<float>::infinity();
      result.nearest_obstacle_indices[index] = DistanceField3D::kNoNearestSource;
    }
  }

  std::size_t lowered_voxels{0U};
  for (const std::size_t source_index : inserted_sources) {
    const GridIndex3D source = localCellFromLinear(local_bounds, source_index);
    const SourceCellRegion affected =
        expandedRegion(SourceCellRegion{.minimum_x = source.x,
                                        .minimum_y = source.y,
                                        .minimum_z = source.z,
                                        .maximum_x_exclusive = source.x + 1,
                                        .maximum_y_exclusive = source.y + 1,
                                        .maximum_z_exclusive = source.z + 1},
                       radius_cells, local_bounds);
    for (int z = affected.minimum_z; z < affected.maximum_z_exclusive; ++z) {
      for (int y = affected.minimum_y; y < affected.maximum_y_exclusive; ++y) {
        for (int x = affected.minimum_x; x < affected.maximum_x_exclusive; ++x) {
          const GridIndex3D cell{x, y, z};
          const double dx = static_cast<double>(x - source.x);
          const double dy = static_cast<double>(y - source.y);
          const double dz = static_cast<double>(z - source.z);
          const double squared_cells = dx * dx + dy * dy + dz * dz;
          if (squared_cells > maximum_squared_cells) {
            continue;
          }
          const std::size_t index = localLinearIndex(local_bounds, cell);
          double previous_squared_cells = std::numeric_limits<double>::infinity();
          if (result.nearest_obstacle_indices[index] !=
              DistanceField3D::kNoNearestSource) {
            const GridIndex3D previous_source = localCellFromLinear(
                local_bounds, result.nearest_obstacle_indices[index]);
            const double previous_dx = static_cast<double>(x - previous_source.x);
            const double previous_dy = static_cast<double>(y - previous_source.y);
            const double previous_dz = static_cast<double>(z - previous_source.z);
            previous_squared_cells = previous_dx * previous_dx +
                                     previous_dy * previous_dy +
                                     previous_dz * previous_dz;
          }
          const float distance_m =
              static_cast<float>(std::sqrt(squared_cells) * local_bounds.resolution_m);
          if (squared_cells < previous_squared_cells) {
            result.distances_m[index] = distance_m;
            result.nearest_obstacle_indices[index] = source_index;
            ++lowered_voxels;
          }
        }
      }
    }
  }

  ChangedCellRegion3D recomputed;
  for (std::size_t index = 0U; index < total_voxels; ++index) {
    if (recompute_mask[index] == 0U) {
      continue;
    }
    const GridIndex3D cell = localCellFromLinear(local_bounds, index);
    if (classified.occupancy->state(cell) == ObservedVoxelState::kUnknown) {
      result.distances_m[index] = mppi::kUnknownEsdfDistanceM;
    } else if (result.nearest_obstacle_indices[index] !=
               DistanceField3D::kNoNearestSource) {
      const GridIndex3D source =
          localCellFromLinear(local_bounds, result.nearest_obstacle_indices[index]);
      const double dx = static_cast<double>(cell.x - source.x);
      const double dy = static_cast<double>(cell.y - source.y);
      const double dz = static_cast<double>(cell.z - source.z);
      result.distances_m[index] = static_cast<float>(
          std::hypot(std::hypot(dx, dy), dz) * local_bounds.resolution_m);
    } else {
      result.distances_m[index] = std::numeric_limits<float>::infinity();
    }
    includeChangedCell(recomputed, cell);
  }
  result.stats.distance_field.voxel_count = recomputed.count;
  result.stats.distance_field.duration_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                dynamic_started)
          .count();
  result.stats.changed_voxels = changed.count;
  result.stats.recomputed_voxels = recomputed.count;
  result.stats.reused_voxels =
      total_voxels > recomputed.count ? total_voxels - recomputed.count : 0U;
  result.stats.dependency_invalidated_voxels = dependency_invalidated_voxels;
  result.stats.lowered_voxels = lowered_voxels;
  result.stats.dirty_chunks = dirty_chunks.size();
  result.stats.mode = ObservedEsdf3DBuildMode::kIncremental;
  const std::vector<ChangedCellRegion3D> dirty_regions =
      splitChangedCellRegions3D(recomputed, local_bounds);
  result.dirty_regions.reserve(dirty_regions.size());
  for (const ChangedCellRegion3D& region : dirty_regions) {
    result.dirty_regions.push_back(ObservedEsdfDirtyRegion3D{
        .minimum_x = region.minimum.x,
        .minimum_y = region.minimum.y,
        .minimum_z = region.minimum.z,
        .maximum_x_exclusive = region.maximum_exclusive.x,
        .maximum_y_exclusive = region.maximum_exclusive.y,
        .maximum_z_exclusive = region.maximum_exclusive.z,
    });
  }
  return result;
}

} // namespace drone_city_nav
