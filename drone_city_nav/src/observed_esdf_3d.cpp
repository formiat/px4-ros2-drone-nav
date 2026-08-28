#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kGeometryTolerance{1.0e-9};

struct SourceCellRegion3D {
  int minimum_x{0};
  int minimum_y{0};
  int minimum_z{0};
  int maximum_x_exclusive{0};
  int maximum_y_exclusive{0};
  int maximum_z_exclusive{0};
};

struct ClassifiedObservedGrid3D {
  std::shared_ptr<ObservedOccupancyGrid3D> occupancy;
  std::vector<GridIndex3D> override_cells;
  std::vector<GridIndex3D> suppressed_source_cells;
  ObservedEsdf3DBuildStats stats{};
};

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  return std::abs(first.origin_x - second.origin_x) <= kGeometryTolerance &&
         std::abs(first.origin_y - second.origin_y) <= kGeometryTolerance &&
         std::abs(first.origin_z - second.origin_z) <= kGeometryTolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= kGeometryTolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

[[nodiscard]] int alignedCellOffset(const double local_origin,
                                    const double world_origin,
                                    const double resolution_m) {
  const double offset = (local_origin - world_origin) / resolution_m;
  const double rounded = std::round(offset);
  if (std::abs(offset - rounded) > 1.0e-6) {
    throw std::invalid_argument{"observed distance bounds are not cell aligned"};
  }
  return static_cast<int>(rounded);
}

[[nodiscard]] SourceCellRegion3D sourceCellRegion(const GridBounds3D& world,
                                                  const GridBounds3D& local) {
  if (std::abs(world.resolution_m - local.resolution_m) > kGeometryTolerance ||
      local.width_cells <= 0 || local.height_cells <= 0 || local.depth_cells <= 0) {
    throw std::invalid_argument{"invalid observed distance local bounds"};
  }
  const SourceCellRegion3D region{
      .minimum_x =
          alignedCellOffset(local.origin_x, world.origin_x, world.resolution_m),
      .minimum_y =
          alignedCellOffset(local.origin_y, world.origin_y, world.resolution_m),
      .minimum_z =
          alignedCellOffset(local.origin_z, world.origin_z, world.resolution_m),
      .maximum_x_exclusive =
          alignedCellOffset(local.origin_x, world.origin_x, world.resolution_m) +
          local.width_cells,
      .maximum_y_exclusive =
          alignedCellOffset(local.origin_y, world.origin_y, world.resolution_m) +
          local.height_cells,
      .maximum_z_exclusive =
          alignedCellOffset(local.origin_z, world.origin_z, world.resolution_m) +
          local.depth_cells,
  };
  if (region.minimum_x < 0 || region.minimum_y < 0 || region.minimum_z < 0 ||
      region.maximum_x_exclusive > world.width_cells ||
      region.maximum_y_exclusive > world.height_cells ||
      region.maximum_z_exclusive > world.depth_cells) {
    throw std::invalid_argument{"observed distance local bounds exceed world"};
  }
  return region;
}

[[nodiscard]] bool inside(const GridIndex3D cell,
                          const SourceCellRegion3D& region) noexcept {
  return cell.x >= region.minimum_x && cell.x < region.maximum_x_exclusive &&
         cell.y >= region.minimum_y && cell.y < region.maximum_y_exclusive &&
         cell.z >= region.minimum_z && cell.z < region.maximum_z_exclusive;
}

[[nodiscard]] std::size_t voxelCount(const GridBounds3D& bounds) noexcept {
  return static_cast<std::size_t>(bounds.width_cells) *
         static_cast<std::size_t>(bounds.height_cells) *
         static_cast<std::size_t>(bounds.depth_cells);
}

[[nodiscard]] std::size_t localLinearIndex(const GridBounds3D& bounds,
                                           const GridIndex3D cell) noexcept {
  return (static_cast<std::size_t>(cell.z) *
              static_cast<std::size_t>(bounds.height_cells) +
          static_cast<std::size_t>(cell.y)) *
             static_cast<std::size_t>(bounds.width_cells) +
         static_cast<std::size_t>(cell.x);
}

[[nodiscard]] GridIndex3D localCellFromLinear(const GridBounds3D& bounds,
                                              const std::size_t index) noexcept {
  const std::size_t width = static_cast<std::size_t>(bounds.width_cells);
  const std::size_t height = static_cast<std::size_t>(bounds.height_cells);
  return GridIndex3D{
      .x = static_cast<int>(index % width),
      .y = static_cast<int>((index / width) % height),
      .z = static_cast<int>(index / (width * height)),
  };
}

[[nodiscard]] bool overlaps(const OccupancyChunkIndex3D chunk,
                            const SourceCellRegion3D& region) noexcept {
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  const int minimum_x = chunk.x * kChunkSize;
  const int minimum_y = chunk.y * kChunkSize;
  const int minimum_z = chunk.z * kChunkSize;
  return minimum_x + kChunkSize > region.minimum_x &&
         minimum_x < region.maximum_x_exclusive &&
         minimum_y + kChunkSize > region.minimum_y &&
         minimum_y < region.maximum_y_exclusive &&
         minimum_z + kChunkSize > region.minimum_z &&
         minimum_z < region.maximum_z_exclusive;
}

template<typename Callback>
void forEachObservedVoxel(const ObservedOccupancyGrid3D& occupancy,
                          const SourceCellRegion3D& region, Callback callback) {
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
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
        const GridIndex3D source{
            chunk_index.x * kChunkSize + static_cast<int>(bit_index % kChunkSize),
            chunk_index.y * kChunkSize +
                static_cast<int>((bit_index / kChunkSize) % kChunkSize),
            chunk_index.z * kChunkSize +
                static_cast<int>(bit_index /
                                 static_cast<std::size_t>(kChunkSize * kChunkSize)),
        };
        if (inside(source, region)) {
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
    if (const std::optional<GridIndex3D> cell = occupancy.worldToCell(center)) {
      result.push_back(*cell);
    }
  }
  std::ranges::sort(result, [](const GridIndex3D first, const GridIndex3D second) {
    return std::tuple{first.z, first.y, first.x} <
           std::tuple{second.z, second.y, second.x};
  });
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

void finalizeClassificationStats(ObservedEsdf3DBuildStats& stats,
                                 const ObservedOccupancyGrid3D& local_occupancy,
                                 const std::size_t total_voxels) {
  stats.known_voxels = local_occupancy.knownVoxelCount();
  stats.free_voxels = local_occupancy.freeVoxelCount();
  stats.occupied_voxels = local_occupancy.occupiedVoxelCount();
  stats.unknown_voxels = total_voxels - stats.known_voxels;
}

[[nodiscard]] ClassifiedObservedGrid3D
classifyObservedGrid3D(const ObservedOccupancyGrid3D& occupancy,
                       const GridBounds3D& local_bounds,
                       const LaunchSupportContact3D* const launch_support_contact) {
  const auto started = std::chrono::steady_clock::now();
  const SourceCellRegion3D region = sourceCellRegion(occupancy.bounds(), local_bounds);
  auto local_occupancy = std::make_shared<ObservedOccupancyGrid3D>(local_bounds);
  const std::vector<GridIndex3D> support_cells =
      launchSupportCells(occupancy, launch_support_contact);
  std::unordered_set<std::size_t> support_indices;
  for (const GridIndex3D source : support_cells) {
    if (inside(source, region)) {
      support_indices.insert(
          localLinearIndex(local_bounds, GridIndex3D{source.x - region.minimum_x,
                                                     source.y - region.minimum_y,
                                                     source.z - region.minimum_z}));
    }
  }
  forEachObservedVoxel(
      occupancy, region, [&](const GridIndex3D source, const ObservedVoxelState state) {
        const GridIndex3D local{source.x - region.minimum_x,
                                source.y - region.minimum_y,
                                source.z - region.minimum_z};
        const bool suppressed =
            support_indices.contains(localLinearIndex(local_bounds, local));
        static_cast<void>(local_occupancy->setState(
            local, suppressed ? ObservedVoxelState::kFree : state));
      });
  std::vector<GridIndex3D> override_cells;
  override_cells.reserve(support_indices.size());
  ObservedEsdf3DBuildStats stats;
  for (const std::size_t index : support_indices) {
    const GridIndex3D local = localCellFromLinear(local_bounds, index);
    override_cells.push_back(local);
    const ObservedVoxelState before = local_occupancy->state(local);
    static_cast<void>(local_occupancy->setState(local, ObservedVoxelState::kFree));
    stats.launch_support_voxels += before != ObservedVoxelState::kFree ? 1U : 0U;
  }
  const std::size_t total_voxels = voxelCount(local_bounds);
  finalizeClassificationStats(stats, *local_occupancy, total_voxels);
  stats.classified_voxels = total_voxels;
  stats.classification_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
  return ClassifiedObservedGrid3D{
      .occupancy = std::move(local_occupancy),
      .override_cells = std::move(override_cells),
      .suppressed_source_cells = support_cells,
      .stats = stats,
  };
}

[[nodiscard]] ClassifiedObservedGrid3D classifyObservedGrid3DIncremental(
    const ObservedOccupancyGrid3D& occupancy, const GridBounds3D& local_bounds,
    const ObservedOccupancyGrid3D& previous_local_occupancy,
    const std::span<const GridIndex3D> previous_override_cells,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks,
    const LaunchSupportContact3D* const launch_support_contact) {
  const auto started = std::chrono::steady_clock::now();
  const SourceCellRegion3D region = sourceCellRegion(occupancy.bounds(), local_bounds);
  auto local_occupancy =
      std::make_shared<ObservedOccupancyGrid3D>(previous_local_occupancy);
  std::unordered_set<std::size_t> affected;
  for (const GridIndex3D local : previous_override_cells) {
    affected.insert(localLinearIndex(local_bounds, local));
  }
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  for (const OccupancyChunkIndex3D chunk : dirty_chunks) {
    const int minimum_x = std::max(region.minimum_x, chunk.x * kChunkSize);
    const int minimum_y = std::max(region.minimum_y, chunk.y * kChunkSize);
    const int minimum_z = std::max(region.minimum_z, chunk.z * kChunkSize);
    const int maximum_x =
        std::min(region.maximum_x_exclusive, (chunk.x + 1) * kChunkSize);
    const int maximum_y =
        std::min(region.maximum_y_exclusive, (chunk.y + 1) * kChunkSize);
    const int maximum_z =
        std::min(region.maximum_z_exclusive, (chunk.z + 1) * kChunkSize);
    for (int z = minimum_z; z < maximum_z; ++z) {
      for (int y = minimum_y; y < maximum_y; ++y) {
        for (int x = minimum_x; x < maximum_x; ++x) {
          affected.insert(localLinearIndex(
              local_bounds, GridIndex3D{x - region.minimum_x, y - region.minimum_y,
                                        z - region.minimum_z}));
        }
      }
    }
  }
  const std::vector<GridIndex3D> support_cells =
      launchSupportCells(occupancy, launch_support_contact);
  std::unordered_set<std::size_t> override_indices;
  std::vector<GridIndex3D> override_cells;
  for (const GridIndex3D source : support_cells) {
    if (!inside(source, region)) {
      continue;
    }
    const GridIndex3D local{source.x - region.minimum_x, source.y - region.minimum_y,
                            source.z - region.minimum_z};
    const std::size_t index = localLinearIndex(local_bounds, local);
    override_indices.insert(index);
    affected.insert(index);
    override_cells.push_back(local);
  }
  ObservedEsdf3DBuildStats stats;
  for (const std::size_t index : affected) {
    const GridIndex3D local = localCellFromLinear(local_bounds, index);
    const GridIndex3D source{local.x + region.minimum_x, local.y + region.minimum_y,
                             local.z + region.minimum_z};
    const ObservedVoxelState raw_state = occupancy.state(source);
    const bool suppressed = override_indices.contains(index);
    static_cast<void>(local_occupancy->setState(
        local, suppressed ? ObservedVoxelState::kFree : raw_state));
    stats.launch_support_voxels +=
        suppressed && raw_state != ObservedVoxelState::kFree ? 1U : 0U;
  }
  const std::size_t total_voxels = voxelCount(local_bounds);
  finalizeClassificationStats(stats, *local_occupancy, total_voxels);
  stats.classified_voxels = affected.size();
  stats.reused_classification_voxels = total_voxels - affected.size();
  stats.classification_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
  return ClassifiedObservedGrid3D{
      .occupancy = std::move(local_occupancy),
      .override_cells = std::move(override_cells),
      .suppressed_source_cells = support_cells,
      .stats = stats,
  };
}

[[nodiscard]] bool rawClassificationChangesCoveredByDirtyChunks(
    const ObservedOccupancyGrid3D& previous, const ObservedOccupancyGrid3D& current,
    const GridBounds3D& local_bounds,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks) {
  if (!sameBounds(previous.bounds(), current.bounds())) {
    return false;
  }
  const SourceCellRegion3D region = sourceCellRegion(current.bounds(), local_bounds);
  const std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash> dirty{
      dirty_chunks.begin(), dirty_chunks.end()};
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
        if (dirty.contains(chunk_index)) {
          continue;
        }
        const ObservedOccupancyGrid3D::Chunk* const before =
            previous.findChunk(chunk_index);
        const ObservedOccupancyGrid3D::Chunk* const after =
            current.findChunk(chunk_index);
        for (std::size_t word = 0U; word < OccupancyGrid3D::kWordsPerChunk; ++word) {
          const std::uint64_t before_observed =
              before != nullptr ? before->observed.at(word) : 0U;
          const std::uint64_t after_observed =
              after != nullptr ? after->observed.at(word) : 0U;
          const std::uint64_t before_occupied =
              before != nullptr ? before->occupied.at(word) : 0U;
          const std::uint64_t after_occupied =
              after != nullptr ? after->occupied.at(word) : 0U;
          if (before_observed != after_observed || before_occupied != after_occupied) {
            return false;
          }
        }
      }
    }
  }
  return true;
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

[[nodiscard]] bool
previousObservedEsdfIsCompatible(const PreviousObservedEsdf3D& previous,
                                 const ObservedOccupancyGrid3D& source_occupancy,
                                 const GridBounds3D& local_bounds,
                                 const double maximum_distance_m) {
  return previous.source_occupancy && previous.local_occupancy &&
         previous.distances_m && previous.known_obstacle_distance &&
         previous.known_obstacle_distance->valid() &&
         sameBounds(previous.source_occupancy->bounds(), source_occupancy.bounds()) &&
         sameBounds(previous.local_occupancy->bounds(), local_bounds) &&
         sameBounds(previous.known_obstacle_distance->bounds(), local_bounds) &&
         previous.distances_m->size() == voxelCount(local_bounds) &&
         previous.occupancy_fingerprint ==
             previous.known_obstacle_distance->sourceFingerprint() &&
         std::abs(previous.maximum_distance_m - maximum_distance_m) <=
             kGeometryTolerance;
}

[[nodiscard]] ObservedEsdf3DBuildMode
observedMode(const KnownObstacleDistance3DBuildMode mode) noexcept {
  switch (mode) {
    case KnownObstacleDistance3DBuildMode::kFull:
      return ObservedEsdf3DBuildMode::kFull;
    case KnownObstacleDistance3DBuildMode::kIncremental:
      return ObservedEsdf3DBuildMode::kIncremental;
    case KnownObstacleDistance3DBuildMode::kReused:
      return ObservedEsdf3DBuildMode::kReused;
  }
  return ObservedEsdf3DBuildMode::kFull;
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
    throw std::invalid_argument{"invalid incremental observed distance request"};
  }
  const bool previous_compatible =
      previous != nullptr &&
      previousObservedEsdfIsCompatible(*previous, occupancy, local_bounds,
                                       maximum_distance_m);
  const bool classification_lineage_complete =
      previous_compatible &&
      rawClassificationChangesCoveredByDirtyChunks(
          *previous->source_occupancy, occupancy, local_bounds, dirty_chunks);
  ClassifiedObservedGrid3D classified =
      previous_compatible && classification_lineage_complete && !full_reset
          ? classifyObservedGrid3DIncremental(occupancy, local_bounds,
                                              *previous->local_occupancy,
                                              previous->classification_override_cells,
                                              dirty_chunks, launch_support_contact)
          : classifyObservedGrid3D(occupancy, local_bounds, launch_support_contact);

  const std::shared_ptr<const KnownObstacleDistance3D> previous_distance =
      previous_compatible ? previous->known_obstacle_distance : nullptr;
  const ObservedOccupancyGrid3D* const previous_source_occupancy =
      previous_compatible ? previous->source_occupancy.get() : nullptr;
  KnownObstacleDistance3DBuildResult distance_update = updateKnownObstacleDistance3D(
      occupancy, local_bounds, maximum_distance_m, previous_distance,
      previous_source_occupancy, dirty_chunks, full_reset || !previous_compatible,
      maximum_rebuild_ratio, classified.suppressed_source_cells, worker_pool);
  if (!distance_update.field || !distance_update.field->valid()) {
    throw std::runtime_error{"known obstacle distance build produced no field"};
  }

  std::shared_ptr<const std::vector<float>> dense_distances;
  if (distance_update.mode == KnownObstacleDistance3DBuildMode::kReused &&
      previous_compatible) {
    dense_distances = previous->distances_m;
  } else {
    dense_distances = distance_update.field->materializeDense();
  }
  if (!dense_distances || dense_distances->size() != voxelCount(local_bounds)) {
    throw std::runtime_error{"known obstacle distance projection is incomplete"};
  }

  ObservedEsdf3D result{
      .grid = esdfGrid(local_bounds),
      .distances_m = std::move(dense_distances),
      .known_obstacle_distance = distance_update.field,
      .local_occupancy = std::move(classified.occupancy),
      .classification_override_cells = std::move(classified.override_cells),
      .dirty_regions = {},
      .occupancy_fingerprint = distance_update.field->sourceFingerprint(),
      .maximum_distance_m = maximum_distance_m,
      .stats = classified.stats,
  };
  result.stats.distance_cache = distance_update.stats;
  result.stats.changed_voxels =
      distance_update.stats.inserted_sources + distance_update.stats.removed_sources;
  result.stats.dirty_chunks = dirty_chunks.size();
  result.stats.mode = observedMode(distance_update.mode);
  result.stats.incremental_fallback = distance_update.incremental_fallback;
  const std::size_t total_voxels = voxelCount(local_bounds);
  switch (result.stats.mode) {
    case ObservedEsdf3DBuildMode::kFull:
      result.stats.recomputed_voxels = total_voxels;
      break;
    case ObservedEsdf3DBuildMode::kIncremental:
      result.stats.recomputed_voxels =
          std::min(total_voxels, distance_update.stats.queried_voxels);
      result.stats.reused_voxels = total_voxels - result.stats.recomputed_voxels;
      break;
    case ObservedEsdf3DBuildMode::kReused:
      result.stats.reused_voxels = total_voxels;
      break;
  }
  result.dirty_regions.reserve(distance_update.dirty_regions.size());
  for (const KnownObstacleDistanceRegion3D& region : distance_update.dirty_regions) {
    result.dirty_regions.push_back(ObservedEsdfDirtyRegion3D{
        .minimum_x = region.minimum_x,
        .minimum_y = region.minimum_y,
        .minimum_z = region.minimum_z,
        .maximum_x_exclusive = region.maximum_x_exclusive,
        .maximum_y_exclusive = region.maximum_y_exclusive,
        .maximum_z_exclusive = region.maximum_z_exclusive,
    });
  }
  return result;
}

} // namespace drone_city_nav
