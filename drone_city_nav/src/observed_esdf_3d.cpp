#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffsetBasis{14695981039346656037ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

void hashWord(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
    hash ^= (value >> shift) & 0xFFU;
    hash *= kFnvPrime;
  }
}

void hashInteger(std::uint64_t& hash, const int value) noexcept {
  hashWord(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(value)));
}

void hashBounds(std::uint64_t& hash, const GridBounds3D& bounds) noexcept {
  hashWord(hash, std::bit_cast<std::uint64_t>(bounds.origin_x));
  hashWord(hash, std::bit_cast<std::uint64_t>(bounds.origin_y));
  hashWord(hash, std::bit_cast<std::uint64_t>(bounds.origin_z));
  hashWord(hash, std::bit_cast<std::uint64_t>(bounds.resolution_m));
  hashInteger(hash, bounds.width_cells);
  hashInteger(hash, bounds.height_cells);
  hashInteger(hash, bounds.depth_cells);
}

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

[[nodiscard]] int clampedCell(const double coordinate, const double origin,
                              const double resolution_m,
                              const int cell_count) noexcept {
  return std::clamp(static_cast<int>(std::floor((coordinate - origin) / resolution_m)),
                    0, cell_count - 1);
}

} // namespace

GridBounds3D selectLocalObservedEsdfBounds(const GridBounds3D& world_bounds,
                                           const Point3& position,
                                           const double half_extent_m) {
  if (!(world_bounds.resolution_m > 0.0) || world_bounds.width_cells <= 0 ||
      world_bounds.height_cells <= 0 || world_bounds.depth_cells <= 0 ||
      !(half_extent_m > 0.0) || !std::isfinite(position.x) ||
      !std::isfinite(position.y)) {
    throw std::invalid_argument{"invalid local observed ESDF bounds request"};
  }
  const int minimum_x =
      clampedCell(position.x - half_extent_m, world_bounds.origin_x,
                  world_bounds.resolution_m, world_bounds.width_cells);
  const int maximum_x =
      clampedCell(position.x + half_extent_m, world_bounds.origin_x,
                  world_bounds.resolution_m, world_bounds.width_cells);
  const int minimum_y =
      clampedCell(position.y - half_extent_m, world_bounds.origin_y,
                  world_bounds.resolution_m, world_bounds.height_cells);
  const int maximum_y =
      clampedCell(position.y + half_extent_m, world_bounds.origin_y,
                  world_bounds.resolution_m, world_bounds.height_cells);
  return GridBounds3D{
      .origin_x = world_bounds.origin_x +
                  static_cast<double>(minimum_x) * world_bounds.resolution_m,
      .origin_y = world_bounds.origin_y +
                  static_cast<double>(minimum_y) * world_bounds.resolution_m,
      .origin_z = world_bounds.origin_z,
      .resolution_m = world_bounds.resolution_m,
      .width_cells = maximum_x - minimum_x + 1,
      .height_cells = maximum_y - minimum_y + 1,
      .depth_cells = world_bounds.depth_cells,
  };
}

bool localObservedEsdfNeedsRecenter(const GridBounds3D& local_bounds,
                                    const GridBounds3D& world_bounds,
                                    const Point3& position,
                                    const double recenter_margin_m) noexcept {
  if (!sameResolution(local_bounds, world_bounds) || !(recenter_margin_m >= 0.0) ||
      !std::isfinite(position.x) || !std::isfinite(position.y)) {
    return true;
  }
  const double local_maximum_x =
      local_bounds.origin_x + local_bounds.width_cells * local_bounds.resolution_m;
  const double local_maximum_y =
      local_bounds.origin_y + local_bounds.height_cells * local_bounds.resolution_m;
  const double world_maximum_x =
      world_bounds.origin_x + world_bounds.width_cells * world_bounds.resolution_m;
  const double world_maximum_y =
      world_bounds.origin_y + world_bounds.height_cells * world_bounds.resolution_m;
  const bool room_left = local_bounds.origin_x > world_bounds.origin_x + 1.0e-9;
  const bool room_right = local_maximum_x < world_maximum_x - 1.0e-9;
  const bool room_down = local_bounds.origin_y > world_bounds.origin_y + 1.0e-9;
  const bool room_up = local_maximum_y < world_maximum_y - 1.0e-9;
  return (room_left && position.x - local_bounds.origin_x < recenter_margin_m) ||
         (room_right && local_maximum_x - position.x < recenter_margin_m) ||
         (room_down && position.y - local_bounds.origin_y < recenter_margin_m) ||
         (room_up && local_maximum_y - position.y < recenter_margin_m);
}

std::uint64_t observedOccupancyFingerprint(const ObservedOccupancyGrid3D& occupancy,
                                           const GridBounds3D& local_bounds) {
  const SourceCellRegion region = sourceCellRegion(occupancy.bounds(), local_bounds);
  using ChunkEntry = std::pair<OccupancyChunkIndex3D, const ObservedOccupancyChunk3D*>;
  std::vector<ChunkEntry> chunks;
  chunks.reserve(occupancy.chunks().size());
  for (const auto& [index, chunk] : occupancy.chunks()) {
    if (overlaps(index, region)) {
      chunks.emplace_back(index, &chunk);
    }
  }
  std::ranges::sort(chunks, {}, [](const ChunkEntry& entry) {
    return std::tuple{entry.first.z, entry.first.y, entry.first.x};
  });

  std::uint64_t hash = kFnvOffsetBasis;
  hashBounds(hash, local_bounds);
  for (const auto& [index, chunk] : chunks) {
    hashInteger(hash, index.x);
    hashInteger(hash, index.y);
    hashInteger(hash, index.z);
    for (const std::uint64_t word : chunk->observed) {
      hashWord(hash, word);
    }
    for (const std::uint64_t word : chunk->occupied) {
      hashWord(hash, word);
    }
  }
  return hash;
}

ObservedEsdf3D buildObservedEsdf3D(const ObservedOccupancyGrid3D& occupancy,
                                   const GridBounds3D& local_bounds,
                                   const double maximum_distance_m,
                                   BoundedWorkerPool* const worker_pool) {
  static_cast<void>(sourceCellRegion(occupancy.bounds(), local_bounds));
  const OccupancyGrid3D occupied = occupancy.occupiedSnapshot();
  const DistanceField3D field = DistanceField3D::buildLocal(
      occupied, local_bounds, maximum_distance_m, worker_pool);
  ObservedEsdf3D result{
      .grid =
          mppi::EsdfGrid{
              .width = local_bounds.width_cells,
              .height = local_bounds.height_cells,
              .resolution_m = static_cast<float>(local_bounds.resolution_m),
              .origin_x_m = static_cast<float>(local_bounds.origin_x),
              .origin_y_m = static_cast<float>(local_bounds.origin_y),
              .depth = local_bounds.depth_cells,
              .origin_z_m = static_cast<float>(local_bounds.origin_z),
              .outside_is_unknown = true,
          },
      .distances_m = {field.distancesM().begin(), field.distancesM().end()},
      .occupancy_fingerprint = observedOccupancyFingerprint(occupancy, local_bounds),
  };
  result.stats.distance_field = field.stats();

  const auto classification_started = std::chrono::steady_clock::now();
  std::size_t linear_index{0U};
  for (int z = 0; z < local_bounds.depth_cells; ++z) {
    for (int y = 0; y < local_bounds.height_cells; ++y) {
      for (int x = 0; x < local_bounds.width_cells; ++x, ++linear_index) {
        const Point3 center{
            local_bounds.origin_x +
                (static_cast<double>(x) + 0.5) * local_bounds.resolution_m,
            local_bounds.origin_y +
                (static_cast<double>(y) + 0.5) * local_bounds.resolution_m,
            local_bounds.origin_z +
                (static_cast<double>(z) + 0.5) * local_bounds.resolution_m,
        };
        const std::optional<GridIndex3D> source = occupancy.worldToCell(center);
        const ObservedVoxelState state = source.has_value()
                                             ? occupancy.state(*source)
                                             : ObservedVoxelState::kUnknown;
        switch (state) {
          case ObservedVoxelState::kUnknown:
            result.distances_m.at(linear_index) = mppi::kUnknownEsdfDistanceM;
            ++result.stats.unknown_voxels;
            break;
          case ObservedVoxelState::kFree:
            ++result.stats.known_voxels;
            ++result.stats.free_voxels;
            break;
          case ObservedVoxelState::kOccupied:
            ++result.stats.known_voxels;
            ++result.stats.occupied_voxels;
            break;
        }
      }
    }
  }
  result.stats.classification_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                classification_started)
          .count();
  return result;
}

} // namespace drone_city_nav
