#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <tuple>
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

[[nodiscard]] bool chunkAlignedCopyPossible(const SourceCellRegion3D& region,
                                            const OccupancyChunkIndex3D chunk,
                                            const GridBounds3D& world) noexcept {
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  // A chunk may be copied verbatim when the local window starts on a chunk
  // boundary and the chunk lies fully inside the window or ends exactly at the
  // world edge, where the world itself has no cells past the window.
  const int chunk_minimum_x = chunk.x * kChunkSize;
  const int chunk_minimum_y = chunk.y * kChunkSize;
  const int chunk_minimum_z = chunk.z * kChunkSize;
  const auto axis_copyable = [](const int chunk_minimum, const int region_minimum,
                                const int region_maximum_exclusive,
                                const int world_cells) {
    return region_minimum % kChunkSize == 0 && chunk_minimum >= region_minimum &&
           (chunk_minimum + kChunkSize <= region_maximum_exclusive ||
            region_maximum_exclusive == world_cells);
  };
  return axis_copyable(chunk_minimum_x, region.minimum_x, region.maximum_x_exclusive,
                       world.width_cells) &&
         axis_copyable(chunk_minimum_y, region.minimum_y, region.maximum_y_exclusive,
                       world.height_cells) &&
         axis_copyable(chunk_minimum_z, region.minimum_z, region.maximum_z_exclusive,
                       world.depth_cells);
}

void finalizeClassificationStats(ObservedEsdf3DBuildStats& stats,
                                 const ObservedOccupancyGrid3D& local_occupancy,
                                 const std::size_t total_voxels) {
  stats.known_voxels = local_occupancy.knownVoxelCount();
  stats.free_voxels = local_occupancy.freeVoxelCount();
  stats.occupied_voxels = local_occupancy.occupiedVoxelCount();
  stats.unknown_voxels = total_voxels - stats.known_voxels;
}

// Copies the observed labels of the local window out of the world grid. Chunks
// aligned with the window are copied as whole bitsets; only boundary chunks of
// an unaligned window are walked per voxel.
[[nodiscard]] ClassifiedObservedGrid3D
classifyObservedGrid3D(const ObservedOccupancyGrid3D& occupancy,
                       const GridBounds3D& local_bounds,
                       const LaunchSupportContact3D* const launch_support_contact) {
  const auto started = std::chrono::steady_clock::now();
  const SourceCellRegion3D region = sourceCellRegion(occupancy.bounds(), local_bounds);
  auto local_occupancy = std::make_shared<ObservedOccupancyGrid3D>(local_bounds);
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  const bool window_chunk_aligned = region.minimum_x % kChunkSize == 0 &&
                                    region.minimum_y % kChunkSize == 0 &&
                                    region.minimum_z % kChunkSize == 0;
  const OccupancyChunkIndex3D first{region.minimum_x / kChunkSize,
                                    region.minimum_y / kChunkSize,
                                    region.minimum_z / kChunkSize};
  const OccupancyChunkIndex3D last{(region.maximum_x_exclusive - 1) / kChunkSize,
                                   (region.maximum_y_exclusive - 1) / kChunkSize,
                                   (region.maximum_z_exclusive - 1) / kChunkSize};
  for (int chunk_z = first.z; chunk_z <= last.z; ++chunk_z) {
    for (int chunk_y = first.y; chunk_y <= last.y; ++chunk_y) {
      for (int chunk_x = first.x; chunk_x <= last.x; ++chunk_x) {
        const OccupancyChunkIndex3D chunk_index{chunk_x, chunk_y, chunk_z};
        const ObservedOccupancyGrid3D::Chunk* const chunk =
            occupancy.findChunk(chunk_index);
        if (chunk == nullptr) {
          continue;
        }
        if (window_chunk_aligned &&
            chunkAlignedCopyPossible(region, chunk_index, occupancy.bounds())) {
          static_cast<void>(local_occupancy->replaceChunk(
              OccupancyChunkIndex3D{chunk_x - first.x, chunk_y - first.y,
                                    chunk_z - first.z},
              *chunk));
          continue;
        }
        const std::span<const std::uint64_t> occupied_words{chunk->occupied};
        std::size_t word_index{0U};
        for (const std::uint64_t observed_word : chunk->observed) {
          std::uint64_t observed_bits = observed_word;
          const std::size_t word_offset = word_index * 64U;
          const std::uint64_t occupied_word = occupied_words[word_index++];
          while (observed_bits != 0U) {
            const int bit_offset = std::countr_zero(observed_bits);
            observed_bits &= observed_bits - 1U;
            const std::size_t bit_index =
                word_offset + static_cast<std::size_t>(bit_offset);
            const GridIndex3D source{
                chunk_x * kChunkSize + static_cast<int>(bit_index % kChunkSize),
                chunk_y * kChunkSize +
                    static_cast<int>((bit_index / kChunkSize) % kChunkSize),
                chunk_z * kChunkSize +
                    static_cast<int>(bit_index /
                                     static_cast<std::size_t>(kChunkSize * kChunkSize)),
            };
            if (!inside(source, region)) {
              continue;
            }
            const bool occupied =
                (occupied_word &
                 (std::uint64_t{1U} << static_cast<unsigned int>(bit_offset))) != 0U;
            static_cast<void>(local_occupancy->setState(
                GridIndex3D{source.x - region.minimum_x, source.y - region.minimum_y,
                            source.z - region.minimum_z},
                occupied ? ObservedVoxelState::kOccupied : ObservedVoxelState::kFree));
          }
        }
      }
    }
  }
  const std::vector<GridIndex3D> support_cells =
      launchSupportCells(occupancy, launch_support_contact);
  std::vector<GridIndex3D> override_cells;
  override_cells.reserve(support_cells.size());
  ObservedEsdf3DBuildStats stats;
  for (const GridIndex3D source : support_cells) {
    if (!inside(source, region)) {
      continue;
    }
    const GridIndex3D local{source.x - region.minimum_x, source.y - region.minimum_y,
                            source.z - region.minimum_z};
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

[[nodiscard]] EsdfGrid3D esdfGrid(const GridBounds3D& bounds) noexcept {
  return EsdfGrid3D{
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
                                 const GridBounds3D& local_bounds,
                                 const double maximum_distance_m) {
  return previous.distances_m && previous.known_obstacle_distance &&
         previous.known_obstacle_distance->valid() &&
         sameBounds(previous.known_obstacle_distance->bounds(), local_bounds) &&
         previous.distances_m->size() == voxelCount(local_bounds) &&
         previous.distances_m == previous.known_obstacle_distance->denseDistances() &&
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
  return updateObservedEsdf3D(occupancy, local_bounds, maximum_distance_m, nullptr,
                              true, worker_pool, launch_support_contact);
}

ObservedEsdf3D
updateObservedEsdf3D(const ObservedOccupancyGrid3D& occupancy,
                     const GridBounds3D& local_bounds, const double maximum_distance_m,
                     const PreviousObservedEsdf3D* const previous,
                     const bool full_reset, BoundedWorkerPool* const worker_pool,
                     const LaunchSupportContact3D* const launch_support_contact) {
  static_cast<void>(sourceCellRegion(occupancy.bounds(), local_bounds));
  if (!std::isfinite(maximum_distance_m) || maximum_distance_m <= 0.0) {
    throw std::invalid_argument{"invalid observed distance request"};
  }
  ClassifiedObservedGrid3D classified =
      classifyObservedGrid3D(occupancy, local_bounds, launch_support_contact);
  const bool previous_compatible =
      !full_reset && previous != nullptr &&
      previousObservedEsdfIsCompatible(*previous, local_bounds, maximum_distance_m);
  KnownObstacleDistance3DBuildResult distance_update = updateKnownObstacleDistance3D(
      occupancy, local_bounds, maximum_distance_m,
      previous_compatible ? previous->known_obstacle_distance : nullptr,
      classified.suppressed_source_cells, worker_pool);
  if (!distance_update.field || !distance_update.field->valid()) {
    throw std::runtime_error{"known obstacle distance build produced no field"};
  }

  ObservedEsdf3D result{
      .grid = esdfGrid(local_bounds),
      .distances_m = distance_update.field->denseDistances(),
      .known_obstacle_distance = distance_update.field,
      .local_occupancy = std::move(classified.occupancy),
      .classification_override_cells = std::move(classified.override_cells),
      .occupancy_fingerprint = distance_update.field->sourceFingerprint(),
      .maximum_distance_m = maximum_distance_m,
      .stats = classified.stats,
  };
  result.stats.distance_field = distance_update.stats;
  result.stats.mode = observedMode(distance_update.mode);
  const std::size_t total_voxels = voxelCount(local_bounds);
  switch (result.stats.mode) {
    case ObservedEsdf3DBuildMode::kFull:
      result.stats.recomputed_voxels = total_voxels;
      break;
    case ObservedEsdf3DBuildMode::kReused:
      result.stats.reused_voxels = total_voxels;
      break;
  }
  return result;
}

} // namespace drone_city_nav
