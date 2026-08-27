#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <stdexcept>

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

struct CellRegion3D {
  int minimum_x{0};
  int minimum_y{0};
  int minimum_z{0};
  int maximum_x_exclusive{0};
  int maximum_y_exclusive{0};
  int maximum_z_exclusive{0};
};

[[nodiscard]] int alignedOffset(const double local_origin, const double world_origin,
                                const double resolution_m) {
  const double offset = (local_origin - world_origin) / resolution_m;
  const double rounded = std::round(offset);
  if (std::abs(offset - rounded) > 1.0e-6) {
    throw std::invalid_argument{"observed ESDF bounds are not cell aligned"};
  }
  return static_cast<int>(rounded);
}

[[nodiscard]] CellRegion3D localRegion(const GridBounds3D& world,
                                       const GridBounds3D& local) {
  if (std::abs(world.resolution_m - local.resolution_m) > 1.0e-9 ||
      local.width_cells <= 0 || local.height_cells <= 0 || local.depth_cells <= 0) {
    throw std::invalid_argument{"invalid observed ESDF local bounds"};
  }
  const int minimum_x =
      alignedOffset(local.origin_x, world.origin_x, world.resolution_m);
  const int minimum_y =
      alignedOffset(local.origin_y, world.origin_y, world.resolution_m);
  const int minimum_z =
      alignedOffset(local.origin_z, world.origin_z, world.resolution_m);
  const CellRegion3D region{
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

[[nodiscard]] bool inside(const GridIndex3D cell, const CellRegion3D& region) noexcept {
  return cell.x >= region.minimum_x && cell.x < region.maximum_x_exclusive &&
         cell.y >= region.minimum_y && cell.y < region.maximum_y_exclusive &&
         cell.z >= region.minimum_z && cell.z < region.maximum_z_exclusive;
}

[[nodiscard]] GridIndex3D chunkCell(const OccupancyChunkIndex3D chunk,
                                    const std::size_t bit_index) noexcept {
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  return GridIndex3D{
      chunk.x * kChunkSize + static_cast<int>(bit_index % kChunkSize),
      chunk.y * kChunkSize + static_cast<int>((bit_index / kChunkSize) % kChunkSize),
      chunk.z * kChunkSize + static_cast<int>(bit_index / static_cast<std::size_t>(
                                                              kChunkSize * kChunkSize)),
  };
}

} // namespace

std::uint64_t knownObstacleFingerprint3D(const ObservedOccupancyGrid3D& occupancy,
                                         const GridBounds3D& local_bounds) {
  const CellRegion3D region = localRegion(occupancy.bounds(), local_bounds);
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  const OccupancyChunkIndex3D first{region.minimum_x / kChunkSize,
                                    region.minimum_y / kChunkSize,
                                    region.minimum_z / kChunkSize};
  const OccupancyChunkIndex3D last{(region.maximum_x_exclusive - 1) / kChunkSize,
                                   (region.maximum_y_exclusive - 1) / kChunkSize,
                                   (region.maximum_z_exclusive - 1) / kChunkSize};
  std::uint64_t hash = kFnvOffsetBasis;
  hashBounds(hash, local_bounds);
  for (int z = first.z; z <= last.z; ++z) {
    for (int y = first.y; y <= last.y; ++y) {
      for (int x = first.x; x <= last.x; ++x) {
        const OccupancyChunkIndex3D index{x, y, z};
        const ObservedOccupancyChunk3D* const chunk = occupancy.findChunk(index);
        if (chunk == nullptr) {
          continue;
        }
        OccupancyGrid3D::Chunk local_occupied{};
        for (std::size_t word_index = 0U; word_index < chunk->occupied.size();
             ++word_index) {
          std::uint64_t occupied_bits = chunk->occupied.at(word_index);
          while (occupied_bits != 0U) {
            const int bit_offset = std::countr_zero(occupied_bits);
            const std::size_t bit_index =
                word_index * 64U + static_cast<std::size_t>(bit_offset);
            if (inside(chunkCell(index, bit_index), region)) {
              const std::uint64_t bit = std::uint64_t{1U}
                                        << static_cast<unsigned int>(bit_offset);
              local_occupied.at(word_index) |= bit;
            }
            occupied_bits &= occupied_bits - 1U;
          }
        }
        if (std::ranges::all_of(local_occupied,
                                [](const std::uint64_t word) { return word == 0U; })) {
          continue;
        }
        hashInteger(hash, index.x);
        hashInteger(hash, index.y);
        hashInteger(hash, index.z);
        for (const std::uint64_t word : local_occupied) {
          hashWord(hash, word);
        }
      }
    }
  }
  return hash;
}

} // namespace drone_city_nav
