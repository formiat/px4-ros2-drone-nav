#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
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

[[nodiscard]] int clampedCell(const double coordinate, const double origin,
                              const double resolution_m,
                              const int cell_count) noexcept {
  return std::clamp(static_cast<int>(std::floor((coordinate - origin) / resolution_m)),
                    0, cell_count - 1);
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

struct LaunchSupportCellCandidate {
  GridIndex3D index{};
  AxisAlignedBox3D bounds{};
};

[[nodiscard]] bool launchSupportEnvelopeIntersectsCell(
    const ProprioceptiveFreeSpaceSeed3D& seed, const double resolution_m,
    const Point3& cell_minimum, const Point3& cell_maximum) noexcept {
  const double axis_norm =
      std::hypot(std::hypot(seed.body_axis.x, seed.body_axis.y), seed.body_axis.z);
  if (!(axis_norm > 1.0e-9) || !(resolution_m > 0.0)) {
    return false;
  }
  const FootprintBodyAxis axis{seed.body_axis.x / axis_norm,
                               seed.body_axis.y / axis_norm,
                               seed.body_axis.z / axis_norm};
  const double lower_extent_m = std::max(0.0, seed.footprint.lower_extent_m);
  const double maximum_axial_extent_m =
      std::max(lower_extent_m, std::max(0.0, seed.footprint.upper_extent_m));
  const Point3 contact_center{seed.position.x - lower_extent_m * axis.x,
                              seed.position.y - lower_extent_m * axis.y,
                              seed.position.z - lower_extent_m * axis.z};
  const SweptFootprintConfig contact_envelope{
      .radius_m =
          std::hypot(std::max(0.0, seed.footprint.radius_m), maximum_axial_extent_m) +
          resolution_m,
      .lower_extent_m = resolution_m,
      .upper_extent_m = resolution_m,
  };
  return footprintIntersectsAxisAlignedBox(contact_center, axis, contact_envelope,
                                           cell_minimum, cell_maximum);
}

[[nodiscard]] std::vector<LaunchSupportCellCandidate>
launchSupportCellCandidates(const GridBounds3D& bounds,
                            const ProprioceptiveFreeSpaceSeed3D& seed) {
  const double broad_extent_m = std::max(0.0, seed.footprint.radius_m) +
                                std::max(std::max(0.0, seed.footprint.lower_extent_m),
                                         std::max(0.0, seed.footprint.upper_extent_m)) +
                                bounds.resolution_m;
  const int minimum_x = clampedCell(seed.position.x - broad_extent_m, bounds.origin_x,
                                    bounds.resolution_m, bounds.width_cells);
  const int maximum_x = clampedCell(seed.position.x + broad_extent_m, bounds.origin_x,
                                    bounds.resolution_m, bounds.width_cells);
  const int minimum_y = clampedCell(seed.position.y - broad_extent_m, bounds.origin_y,
                                    bounds.resolution_m, bounds.height_cells);
  const int maximum_y = clampedCell(seed.position.y + broad_extent_m, bounds.origin_y,
                                    bounds.resolution_m, bounds.height_cells);
  const int minimum_z = clampedCell(seed.position.z - broad_extent_m, bounds.origin_z,
                                    bounds.resolution_m, bounds.depth_cells);
  const int maximum_z = clampedCell(seed.position.z + broad_extent_m, bounds.origin_z,
                                    bounds.resolution_m, bounds.depth_cells);

  std::vector<LaunchSupportCellCandidate> result;
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        const Point3 minimum{
            bounds.origin_x + static_cast<double>(x) * bounds.resolution_m,
            bounds.origin_y + static_cast<double>(y) * bounds.resolution_m,
            bounds.origin_z + static_cast<double>(z) * bounds.resolution_m};
        const Point3 maximum{minimum.x + bounds.resolution_m,
                             minimum.y + bounds.resolution_m,
                             minimum.z + bounds.resolution_m};
        if (launchSupportEnvelopeIntersectsCell(seed, bounds.resolution_m, minimum,
                                                maximum)) {
          result.push_back(LaunchSupportCellCandidate{
              .index = GridIndex3D{x, y, z},
              .bounds = AxisAlignedBox3D{.minimum = minimum, .maximum = maximum},
          });
        }
      }
    }
  }
  return result;
}

[[nodiscard]] LaunchSupportContact3D
makeLaunchSupportContact3D(const GridBounds3D& bounds,
                           const ProprioceptiveFreeSpaceSeed3D& seed,
                           const LaunchSupportEvidenceSource evidence_source,
                           const std::size_t occupied_evidence_cells,
                           const std::vector<LaunchSupportCellCandidate>& candidates) {
  LaunchSupportContact3D contact{
      .seed = seed,
      .contact_cells = {},
      .occupied_evidence_cells = occupied_evidence_cells,
      .evidence_source = evidence_source,
      .maximum_lateral_departure_m = bounds.resolution_m,
      .minimum_axial_departure_m = 0.0,
      .maximum_axial_settling_m = bounds.resolution_m,
  };
  contact.contact_cells.reserve(candidates.size());
  std::ranges::transform(
      candidates, std::back_inserter(contact.contact_cells),
      [](const LaunchSupportCellCandidate& candidate) { return candidate.bounds; });
  return contact;
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
  std::uint64_t fingerprint{0U};
  ObservedEsdf3DBuildStats stats{};
};

[[nodiscard]] ClassifiedObservedGrid3D
classifyObservedGrid3D(const ObservedOccupancyGrid3D& occupancy,
                       const GridBounds3D& local_bounds,
                       const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
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
    if (local_occupancy->state(local_cell) != ObservedVoxelState::kFree) {
      static_cast<void>(
          local_occupancy->setState(local_cell, ObservedVoxelState::kFree));
      ++stats.launch_support_voxels;
    }
  }

  if (free_space_seed != nullptr) {
    const double seed_extent_m =
        std::max(0.0, free_space_seed->footprint.radius_m) +
        std::max(std::max(0.0, free_space_seed->footprint.lower_extent_m),
                 std::max(0.0, free_space_seed->footprint.upper_extent_m)) +
        local_bounds.resolution_m;
    const int minimum_x =
        clampedCell(free_space_seed->position.x - seed_extent_m, local_bounds.origin_x,
                    local_bounds.resolution_m, local_bounds.width_cells);
    const int maximum_x =
        clampedCell(free_space_seed->position.x + seed_extent_m, local_bounds.origin_x,
                    local_bounds.resolution_m, local_bounds.width_cells);
    const int minimum_y =
        clampedCell(free_space_seed->position.y - seed_extent_m, local_bounds.origin_y,
                    local_bounds.resolution_m, local_bounds.height_cells);
    const int maximum_y =
        clampedCell(free_space_seed->position.y + seed_extent_m, local_bounds.origin_y,
                    local_bounds.resolution_m, local_bounds.height_cells);
    const int minimum_z =
        clampedCell(free_space_seed->position.z - seed_extent_m, local_bounds.origin_z,
                    local_bounds.resolution_m, local_bounds.depth_cells);
    const int maximum_z =
        clampedCell(free_space_seed->position.z + seed_extent_m, local_bounds.origin_z,
                    local_bounds.resolution_m, local_bounds.depth_cells);
    for (int z = minimum_z; z <= maximum_z; ++z) {
      for (int y = minimum_y; y <= maximum_y; ++y) {
        for (int x = minimum_x; x <= maximum_x; ++x) {
          const GridIndex3D local_cell{x, y, z};
          if (local_occupancy->state(local_cell) != ObservedVoxelState::kUnknown) {
            continue;
          }
          const Point3 cell_minimum{
              local_bounds.origin_x +
                  static_cast<double>(x) * local_bounds.resolution_m,
              local_bounds.origin_y +
                  static_cast<double>(y) * local_bounds.resolution_m,
              local_bounds.origin_z +
                  static_cast<double>(z) * local_bounds.resolution_m,
          };
          const Point3 cell_maximum{
              cell_minimum.x + local_bounds.resolution_m,
              cell_minimum.y + local_bounds.resolution_m,
              cell_minimum.z + local_bounds.resolution_m,
          };
          if (!footprintIntersectsAxisAlignedBox(
                  free_space_seed->position, free_space_seed->body_axis,
                  free_space_seed->footprint, cell_minimum, cell_maximum)) {
            continue;
          }
          static_cast<void>(
              local_occupancy->setState(local_cell, ObservedVoxelState::kFree));
          ++stats.proprioceptive_free_voxels;
        }
      }
    }
  }

  stats.known_voxels = local_occupancy->knownVoxelCount();
  stats.free_voxels = local_occupancy->freeVoxelCount();
  stats.occupied_voxels = local_occupancy->occupiedVoxelCount();
  stats.unknown_voxels = voxelCount(local_bounds) - stats.known_voxels;
  stats.classification_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
  const std::uint64_t fingerprint =
      observedOccupancyFingerprint(*local_occupancy, local_bounds);
  return {.occupancy = std::move(local_occupancy),
          .fingerprint = fingerprint,
          .stats = stats};
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
      .local_occupancy = classified.occupancy,
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

[[nodiscard]] std::size_t regionVoxelCount(const SourceCellRegion& region) noexcept {
  return static_cast<std::size_t>(region.maximum_x_exclusive - region.minimum_x) *
         static_cast<std::size_t>(region.maximum_y_exclusive - region.minimum_y) *
         static_cast<std::size_t>(region.maximum_z_exclusive - region.minimum_z);
}

[[nodiscard]] GridBounds3D boundsForRegion(const GridBounds3D& bounds,
                                           const SourceCellRegion& region) noexcept {
  return GridBounds3D{
      .origin_x = bounds.origin_x + region.minimum_x * bounds.resolution_m,
      .origin_y = bounds.origin_y + region.minimum_y * bounds.resolution_m,
      .origin_z = bounds.origin_z + region.minimum_z * bounds.resolution_m,
      .resolution_m = bounds.resolution_m,
      .width_cells = region.maximum_x_exclusive - region.minimum_x,
      .height_cells = region.maximum_y_exclusive - region.minimum_y,
      .depth_cells = region.maximum_z_exclusive - region.minimum_z,
  };
}

[[nodiscard]] bool regionsTouchOrOverlap(const SourceCellRegion& first,
                                         const SourceCellRegion& second) noexcept {
  return first.minimum_x <= second.maximum_x_exclusive &&
         second.minimum_x <= first.maximum_x_exclusive &&
         first.minimum_y <= second.maximum_y_exclusive &&
         second.minimum_y <= first.maximum_y_exclusive &&
         first.minimum_z <= second.maximum_z_exclusive &&
         second.minimum_z <= first.maximum_z_exclusive;
}

[[nodiscard]] SourceCellRegion unionRegion(const SourceCellRegion& first,
                                           const SourceCellRegion& second) noexcept {
  return SourceCellRegion{
      .minimum_x = std::min(first.minimum_x, second.minimum_x),
      .minimum_y = std::min(first.minimum_y, second.minimum_y),
      .minimum_z = std::min(first.minimum_z, second.minimum_z),
      .maximum_x_exclusive =
          std::max(first.maximum_x_exclusive, second.maximum_x_exclusive),
      .maximum_y_exclusive =
          std::max(first.maximum_y_exclusive, second.maximum_y_exclusive),
      .maximum_z_exclusive =
          std::max(first.maximum_z_exclusive, second.maximum_z_exclusive),
  };
}

void mergeTouchingRegions(std::vector<SourceCellRegion>& regions) {
  bool merged{true};
  while (merged) {
    merged = false;
    for (std::size_t first = 0U; first < regions.size() && !merged; ++first) {
      for (std::size_t second = first + 1U; second < regions.size(); ++second) {
        if (!regionsTouchOrOverlap(regions[first], regions[second])) {
          continue;
        }
        regions[first] = unionRegion(regions[first], regions[second]);
        regions.erase(regions.begin() + static_cast<std::ptrdiff_t>(second));
        merged = true;
        break;
      }
    }
  }
}

} // namespace

GridBounds3D selectLocalObservedEsdfBounds(const GridBounds3D& world_bounds,
                                           const Point3& position,
                                           const LocalObservedEsdfWindow3D& window) {
  if (!(world_bounds.resolution_m > 0.0) || world_bounds.width_cells <= 0 ||
      world_bounds.height_cells <= 0 || world_bounds.depth_cells <= 0 ||
      !localObservedEsdfWindow3DIsValid(window) || !std::isfinite(position.x) ||
      !std::isfinite(position.y) || !std::isfinite(position.z)) {
    throw std::invalid_argument{"invalid local observed ESDF bounds request"};
  }
  const int minimum_x =
      clampedCell(position.x - window.horizontal_half_extent_m, world_bounds.origin_x,
                  world_bounds.resolution_m, world_bounds.width_cells);
  const int maximum_x =
      clampedCell(position.x + window.horizontal_half_extent_m, world_bounds.origin_x,
                  world_bounds.resolution_m, world_bounds.width_cells);
  const int minimum_y =
      clampedCell(position.y - window.horizontal_half_extent_m, world_bounds.origin_y,
                  world_bounds.resolution_m, world_bounds.height_cells);
  const int maximum_y =
      clampedCell(position.y + window.horizontal_half_extent_m, world_bounds.origin_y,
                  world_bounds.resolution_m, world_bounds.height_cells);
  const int minimum_z =
      clampedCell(position.z - window.vertical_half_extent_m, world_bounds.origin_z,
                  world_bounds.resolution_m, world_bounds.depth_cells);
  const int maximum_z =
      clampedCell(position.z + window.vertical_half_extent_m, world_bounds.origin_z,
                  world_bounds.resolution_m, world_bounds.depth_cells);
  return GridBounds3D{
      .origin_x = world_bounds.origin_x +
                  static_cast<double>(minimum_x) * world_bounds.resolution_m,
      .origin_y = world_bounds.origin_y +
                  static_cast<double>(minimum_y) * world_bounds.resolution_m,
      .origin_z = world_bounds.origin_z +
                  static_cast<double>(minimum_z) * world_bounds.resolution_m,
      .resolution_m = world_bounds.resolution_m,
      .width_cells = maximum_x - minimum_x + 1,
      .height_cells = maximum_y - minimum_y + 1,
      .depth_cells = maximum_z - minimum_z + 1,
  };
}

bool localObservedEsdfNeedsRecenter(const GridBounds3D& local_bounds,
                                    const GridBounds3D& world_bounds,
                                    const Point3& position,
                                    const LocalObservedEsdfWindow3D& window) noexcept {
  if (!sameResolution(local_bounds, world_bounds) ||
      !localObservedEsdfWindow3DIsValid(window) || !std::isfinite(position.x) ||
      !std::isfinite(position.y) || !std::isfinite(position.z)) {
    return true;
  }
  const double local_maximum_x =
      local_bounds.origin_x + local_bounds.width_cells * local_bounds.resolution_m;
  const double local_maximum_y =
      local_bounds.origin_y + local_bounds.height_cells * local_bounds.resolution_m;
  const double local_maximum_z =
      local_bounds.origin_z + local_bounds.depth_cells * local_bounds.resolution_m;
  const double world_maximum_x =
      world_bounds.origin_x + world_bounds.width_cells * world_bounds.resolution_m;
  const double world_maximum_y =
      world_bounds.origin_y + world_bounds.height_cells * world_bounds.resolution_m;
  const double world_maximum_z =
      world_bounds.origin_z + world_bounds.depth_cells * world_bounds.resolution_m;
  const bool room_left = local_bounds.origin_x > world_bounds.origin_x + 1.0e-9;
  const bool room_right = local_maximum_x < world_maximum_x - 1.0e-9;
  const bool room_down = local_bounds.origin_y > world_bounds.origin_y + 1.0e-9;
  const bool room_up = local_maximum_y < world_maximum_y - 1.0e-9;
  const bool room_below = local_bounds.origin_z > world_bounds.origin_z + 1.0e-9;
  const bool room_above = local_maximum_z < world_maximum_z - 1.0e-9;
  return (room_left &&
          position.x - local_bounds.origin_x < window.horizontal_recenter_margin_m) ||
         (room_right &&
          local_maximum_x - position.x < window.horizontal_recenter_margin_m) ||
         (room_down &&
          position.y - local_bounds.origin_y < window.horizontal_recenter_margin_m) ||
         (room_up &&
          local_maximum_y - position.y < window.horizontal_recenter_margin_m) ||
         (room_below &&
          position.z - local_bounds.origin_z < window.vertical_recenter_margin_m) ||
         (room_above &&
          local_maximum_z - position.z < window.vertical_recenter_margin_m);
}

bool localObservedEsdfWindow3DIsValid(
    const LocalObservedEsdfWindow3D& window) noexcept {
  return std::isfinite(window.horizontal_half_extent_m) &&
         window.horizontal_half_extent_m > 0.0 &&
         std::isfinite(window.vertical_half_extent_m) &&
         window.vertical_half_extent_m > 0.0 &&
         std::isfinite(window.horizontal_recenter_margin_m) &&
         window.horizontal_recenter_margin_m >= 0.0 &&
         window.horizontal_recenter_margin_m < window.horizontal_half_extent_m &&
         std::isfinite(window.vertical_recenter_margin_m) &&
         window.vertical_recenter_margin_m >= 0.0 &&
         window.vertical_recenter_margin_m < window.vertical_half_extent_m;
}

std::optional<LaunchSupportContact3D>
detectLaunchSupportContact3D(const ObservedOccupancyGrid3D& occupancy,
                             const ProprioceptiveFreeSpaceSeed3D& seed) {
  const GridBounds3D& bounds = occupancy.bounds();
  const std::vector<LaunchSupportCellCandidate> candidates =
      launchSupportCellCandidates(bounds, seed);
  const std::size_t occupied_evidence_cells = static_cast<std::size_t>(
      std::ranges::count_if(candidates, [&](const LaunchSupportCellCandidate& cell) {
        return occupancy.state(cell.index) == ObservedVoxelState::kOccupied;
      }));
  if (occupied_evidence_cells == 0U) {
    return std::nullopt;
  }
  return makeLaunchSupportContact3D(bounds, seed,
                                    LaunchSupportEvidenceSource::kObservedOccupancy,
                                    occupied_evidence_cells, candidates);
}

LaunchSupportContact3D
makeVehicleLandedSupportContact3D(const GridBounds3D& bounds,
                                  const ProprioceptiveFreeSpaceSeed3D& seed) {
  const std::vector<LaunchSupportCellCandidate> candidates =
      launchSupportCellCandidates(bounds, seed);
  return makeLaunchSupportContact3D(
      bounds, seed, LaunchSupportEvidenceSource::kVehicleLandDetector, 0U, candidates);
}

LaunchSupportDeparture3D planLaunchSupportDeparture3D(
    const ObservedOccupancyGrid3D& occupancy, const Point3& current_position,
    const LaunchSupportContact3D& contact, const double minimum_departure_m) {
  LaunchSupportDeparture3D result;
  const GridBounds3D& bounds = occupancy.bounds();
  const FootprintBodyAxis& requested_axis = contact.seed.body_axis;
  const double axis_norm =
      std::hypot(std::hypot(requested_axis.x, requested_axis.y), requested_axis.z);
  if (!(bounds.resolution_m > 0.0) || !(axis_norm > 1.0e-9) ||
      !std::isfinite(minimum_departure_m) || minimum_departure_m < 0.0 ||
      contact.contact_cells.empty()) {
    return result;
  }
  const FootprintBodyAxis axis{requested_axis.x / axis_norm,
                               requested_axis.y / axis_norm,
                               requested_axis.z / axis_norm};
  double contact_maximum_axial_m{-std::numeric_limits<double>::infinity()};
  for (const AxisAlignedBox3D& cell : contact.contact_cells) {
    for (const double x : {cell.minimum.x, cell.maximum.x}) {
      for (const double y : {cell.minimum.y, cell.maximum.y}) {
        for (const double z : {cell.minimum.z, cell.maximum.z}) {
          const Point3 offset{x - contact.seed.position.x, y - contact.seed.position.y,
                              z - contact.seed.position.z};
          contact_maximum_axial_m =
              std::max(contact_maximum_axial_m,
                       offset.x * axis.x + offset.y * axis.y + offset.z * axis.z);
        }
      }
    }
  }
  const double required_axial_departure_m = std::max(
      minimum_departure_m, contact_maximum_axial_m +
                               std::max(0.0, contact.seed.footprint.lower_extent_m) +
                               bounds.resolution_m);
  const double maximum_search_departure_m =
      required_axial_departure_m +
      std::max({bounds.resolution_m * 4.0,
                std::max(0.0, contact.seed.footprint.lower_extent_m) +
                    std::max(0.0, contact.seed.footprint.upper_extent_m),
                std::max(0.0, contact.seed.footprint.radius_m)});
  const Point3 seed_offset{current_position.x - contact.seed.position.x,
                           current_position.y - contact.seed.position.y,
                           current_position.z - contact.seed.position.z};
  const double current_axial_departure_m =
      seed_offset.x * axis.x + seed_offset.y * axis.y + seed_offset.z * axis.z;
  const int first_step =
      std::max(1, static_cast<int>(std::ceil(
                      (required_axial_departure_m - current_axial_departure_m) /
                      bounds.resolution_m)));
  const int maximum_step =
      std::max(first_step, static_cast<int>(std::ceil((maximum_search_departure_m -
                                                       current_axial_departure_m) /
                                                      bounds.resolution_m)));
  for (int step = first_step; step <= maximum_step; ++step) {
    const double displacement_m = static_cast<double>(step) * bounds.resolution_m;
    const Point3 target{current_position.x + axis.x * displacement_m,
                        current_position.y + axis.y * displacement_m,
                        current_position.z + axis.z * displacement_m};
    const SweptFootprintResult target_validation = validateRawFootprintAt(
        occupancy, target, axis, contact.seed.footprint, nullptr, nullptr);
    if (!target_validation.accepted()) {
      result.validation = target_validation;
      continue;
    }
    const SweptFootprintResult path_validation =
        validateRawSweptFootprint(occupancy, current_position, axis, target, axis,
                                  contact.seed.footprint, &contact.seed, &contact);
    if (!path_validation.accepted()) {
      result.validation = path_validation;
      continue;
    }
    result.target = target;
    result.validation = path_validation;
    result.axial_departure_m = current_axial_departure_m + displacement_m;
    result.executable = true;
    return result;
  }
  return result;
}

ObservedEsdf3D
buildObservedEsdf3D(const ObservedOccupancyGrid3D& occupancy,
                    const GridBounds3D& local_bounds, const double maximum_distance_m,
                    BoundedWorkerPool* const worker_pool,
                    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
                    const LaunchSupportContact3D* const launch_support_contact) {
  return updateObservedEsdf3D(occupancy, local_bounds, maximum_distance_m, nullptr, {},
                              true, 1.0, worker_pool, free_space_seed,
                              launch_support_contact);
}

ObservedEsdf3D updateObservedEsdf3D(
    const ObservedOccupancyGrid3D& occupancy, const GridBounds3D& local_bounds,
    const double maximum_distance_m, const PreviousObservedEsdf3D* const previous,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks, const bool full_reset,
    const double maximum_rebuild_ratio, BoundedWorkerPool* const worker_pool,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
    const LaunchSupportContact3D* const launch_support_contact) {
  static_cast<void>(sourceCellRegion(occupancy.bounds(), local_bounds));
  if (!std::isfinite(maximum_distance_m) || maximum_distance_m <= 0.0 ||
      !std::isfinite(maximum_rebuild_ratio) || maximum_rebuild_ratio <= 0.0 ||
      maximum_rebuild_ratio > 1.0) {
    throw std::invalid_argument{"invalid incremental observed ESDF request"};
  }
  const ClassifiedObservedGrid3D classified = classifyObservedGrid3D(
      occupancy, local_bounds, free_space_seed, launch_support_contact);
  const bool previous_compatible =
      previous != nullptr &&
      previousObservedEsdfIsCompatible(*previous, occupancy, local_bounds,
                                       maximum_distance_m);
  if (full_reset || !previous_compatible) {
    return buildFullObservedEsdf3D(classified, maximum_distance_m, worker_pool,
                                   dirty_chunks.size(), previous != nullptr);
  }

  const ChangedCellRegion3D changed =
      changedCellRegion3D(*previous->local_occupancy, *classified.occupancy);
  if (!rawChangesCoveredByDirtyChunks(*previous->source_occupancy, occupancy,
                                      local_bounds, dirty_chunks)) {
    ObservedEsdf3D result = buildFullObservedEsdf3D(
        classified, maximum_distance_m, worker_pool, dirty_chunks.size(), true);
    result.stats.changed_voxels = changed.count;
    return result;
  }
  if (changed.count == 0U) {
    ObservedEsdf3D result{
        .grid = previous->grid,
        .distances_m = std::vector<float>(previous->distances_m.begin(),
                                          previous->distances_m.end()),
        .local_occupancy = previous->local_occupancy,
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
  const std::vector<ChangedCellRegion3D> changed_regions =
      splitChangedCellRegions3D(changed, local_bounds);
  const std::size_t total_voxels = voxelCount(local_bounds);
  std::vector<SourceCellRegion> target_regions;
  target_regions.reserve(changed_regions.size());
  for (const ChangedCellRegion3D& changed_region : changed_regions) {
    const SourceCellRegion source_region{
        .minimum_x = changed_region.minimum.x,
        .minimum_y = changed_region.minimum.y,
        .minimum_z = changed_region.minimum.z,
        .maximum_x_exclusive = changed_region.maximum_exclusive.x,
        .maximum_y_exclusive = changed_region.maximum_exclusive.y,
        .maximum_z_exclusive = changed_region.maximum_exclusive.z,
    };
    const SourceCellRegion target_region =
        expandedRegion(source_region, radius_cells, local_bounds);
    target_regions.push_back(target_region);
  }
  // Each region needs a maximum-distance halo.  Merge first so overlapping
  // halos are built once and the budget reflects their global unique work.
  mergeTouchingRegions(target_regions);
  std::vector<std::pair<SourceCellRegion, SourceCellRegion>> regions;
  regions.reserve(target_regions.size());
  std::size_t global_patch_voxels{0U};
  for (const SourceCellRegion target_region : target_regions) {
    const SourceCellRegion patch_region =
        expandedRegion(target_region, radius_cells, local_bounds);
    global_patch_voxels += regionVoxelCount(patch_region);
    regions.emplace_back(target_region, patch_region);
  }
  if (global_patch_voxels >= total_voxels ||
      static_cast<double>(global_patch_voxels) / static_cast<double>(total_voxels) >
          maximum_rebuild_ratio) {
    ObservedEsdf3D result = buildFullObservedEsdf3D(
        classified, maximum_distance_m, worker_pool, dirty_chunks.size(), true);
    result.stats.changed_voxels = changed.count;
    return result;
  }

  const OccupancyGrid3D occupied = classified.occupancy->occupiedSnapshot();
  ObservedEsdf3D result{
      .grid = esdfGrid(local_bounds),
      .distances_m = std::vector<float>(previous->distances_m.begin(),
                                        previous->distances_m.end()),
      .local_occupancy = classified.occupancy,
      .occupancy_fingerprint = classified.fingerprint,
      .maximum_distance_m = maximum_distance_m,
      .stats = classified.stats,
  };
  std::size_t recomputed_voxels{0U};
  double patch_build_ms{0.0};
  for (const auto& [target_region, patch_region] : regions) {
    const auto patch_started = std::chrono::steady_clock::now();
    const DistanceField3D patch = DistanceField3D::buildLocal(
        occupied, boundsForRegion(local_bounds, patch_region), maximum_distance_m,
        worker_pool);
    patch_build_ms += std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - patch_started)
                          .count();
    recomputed_voxels += regionVoxelCount(target_region);
    for (int z = target_region.minimum_z; z < target_region.maximum_z_exclusive; ++z) {
      for (int y = target_region.minimum_y; y < target_region.maximum_y_exclusive;
           ++y) {
        for (int x = target_region.minimum_x; x < target_region.maximum_x_exclusive;
             ++x) {
          const GridIndex3D local_cell{x, y, z};
          const std::size_t output_index = localLinearIndex(local_bounds, local_cell);
          if (classified.occupancy->state(local_cell) == ObservedVoxelState::kUnknown) {
            result.distances_m[output_index] = mppi::kUnknownEsdfDistanceM;
            continue;
          }
          result.distances_m[output_index] = patch.distanceAt(
              GridIndex3D{x - patch_region.minimum_x, y - patch_region.minimum_y,
                          z - patch_region.minimum_z});
        }
      }
    }
  }
  result.stats.distance_field.duration_ms = patch_build_ms;
  result.stats.changed_voxels = changed.count;
  result.stats.recomputed_voxels = recomputed_voxels;
  result.stats.reused_voxels =
      total_voxels > recomputed_voxels ? total_voxels - recomputed_voxels : 0U;
  result.stats.dirty_chunks = dirty_chunks.size();
  result.stats.mode = ObservedEsdf3DBuildMode::kIncremental;
  result.dirty_regions.reserve(regions.size());
  for (const auto& [target_region, unused_patch_region] : regions) {
    static_cast<void>(unused_patch_region);
    result.dirty_regions.push_back(ObservedEsdfDirtyRegion3D{
        .minimum_x = target_region.minimum_x,
        .minimum_y = target_region.minimum_y,
        .minimum_z = target_region.minimum_z,
        .maximum_x_exclusive = target_region.maximum_x_exclusive,
        .maximum_y_exclusive = target_region.maximum_y_exclusive,
        .maximum_z_exclusive = target_region.maximum_z_exclusive,
    });
  }
  return result;
}

} // namespace drone_city_nav
