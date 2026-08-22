#pragma once

#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>

namespace drone_city_nav::incremental_topology_detail {

inline constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
inline constexpr std::uint64_t kFnvPrime{1099511628211ULL};

inline void hashInteger(std::uint64_t& hash, const int value) noexcept {
  const auto encoded = static_cast<std::uint32_t>(value);
  for (std::size_t shift = 0U; shift < sizeof(encoded); ++shift) {
    hash ^= static_cast<std::uint8_t>(encoded >> (shift * 8U));
    hash *= kFnvPrime;
  }
}

inline void hashUnsigned(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t shift = 0U; shift < sizeof(value); ++shift) {
    hash ^= static_cast<std::uint8_t>(value >> (shift * 8U));
    hash *= kFnvPrime;
  }
}

[[nodiscard]] inline std::uint64_t sampleCellKey(const GridBounds3D& bounds,
                                                 const GridIndex3D cell) noexcept {
  if (cell.x < 0 || cell.y < 0 || cell.z < 0 || cell.x >= bounds.width_cells ||
      cell.y >= bounds.height_cells || cell.z >= bounds.depth_cells) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return (static_cast<std::uint64_t>(cell.z) *
              static_cast<std::uint64_t>(bounds.height_cells) +
          static_cast<std::uint64_t>(cell.y)) *
             static_cast<std::uint64_t>(bounds.width_cells) +
         static_cast<std::uint64_t>(cell.x);
}

[[nodiscard]] inline GridIndex3D sampleCellForKey(const GridBounds3D& bounds,
                                                  const std::uint64_t key) noexcept {
  const std::uint64_t width = static_cast<std::uint64_t>(bounds.width_cells);
  const std::uint64_t height = static_cast<std::uint64_t>(bounds.height_cells);
  const int x = static_cast<int>(key % width);
  const std::uint64_t yz = key / width;
  return GridIndex3D{x, static_cast<int>(yz % height), static_cast<int>(yz / height)};
}

} // namespace drone_city_nav::incremental_topology_detail
