#pragma once

#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

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

[[nodiscard]] inline bool cellLess(const GridIndex3D first,
                                   const GridIndex3D second) noexcept {
  return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
}

[[nodiscard]] inline bool sameBounds(const GridBounds3D& first,
                                     const GridBounds3D& second) noexcept {
  constexpr double tolerance{1.0e-9};
  return std::abs(first.origin_x - second.origin_x) <= tolerance &&
         std::abs(first.origin_y - second.origin_y) <= tolerance &&
         std::abs(first.origin_z - second.origin_z) <= tolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= tolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

[[nodiscard]] inline IncrementalTopologyBlockIndex3D
blockForCell(const GridIndex3D cell, const int block_size_cells) noexcept {
  return IncrementalTopologyBlockIndex3D{
      cell.x / block_size_cells, cell.y / block_size_cells, cell.z / block_size_cells};
}

[[nodiscard]] inline int firstAlignedCell(const int minimum,
                                          const int stride) noexcept {
  const int remainder = minimum % stride;
  return remainder == 0 ? minimum : minimum + stride - remainder;
}

[[nodiscard]] inline Point3 cellCenter(const GridBounds3D& bounds,
                                       const GridIndex3D cell) noexcept {
  return Point3{
      .x = bounds.origin_x + (static_cast<double>(cell.x) + 0.5) * bounds.resolution_m,
      .y = bounds.origin_y + (static_cast<double>(cell.y) + 0.5) * bounds.resolution_m,
      .z = bounds.origin_z + (static_cast<double>(cell.z) + 0.5) * bounds.resolution_m};
}

[[nodiscard]] inline std::uint64_t
makeNodeIdValue(const IncrementalTopologyBlockIndex3D block, const GridIndex3D anchor,
                const std::uint64_t salt) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashInteger(hash, block.x);
  hashInteger(hash, block.y);
  hashInteger(hash, block.z);
  hashInteger(hash, anchor.x);
  hashInteger(hash, anchor.y);
  hashInteger(hash, anchor.z);
  hashUnsigned(hash, salt);
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] inline IncrementalTopologyEdgeId
makeEdgeId(IncrementalTopologyNodeId first, IncrementalTopologyNodeId second) noexcept {
  if (second < first) {
    std::swap(first, second);
  }
  std::uint64_t hash{kFnvOffset};
  hashUnsigned(hash, first.value);
  hashUnsigned(hash, second.value);
  return IncrementalTopologyEdgeId{hash == 0U ? 1U : hash};
}

[[nodiscard]] inline std::array<GridIndex3D, 6U>
cardinalNeighbors(const GridIndex3D cell, const int stride) noexcept {
  return {
      GridIndex3D{cell.x - stride, cell.y, cell.z},
      GridIndex3D{cell.x + stride, cell.y, cell.z},
      GridIndex3D{cell.x, cell.y - stride, cell.z},
      GridIndex3D{cell.x, cell.y + stride, cell.z},
      GridIndex3D{cell.x, cell.y, cell.z - stride},
      GridIndex3D{cell.x, cell.y, cell.z + stride},
  };
}

template<typename Occupancy>
[[nodiscard]] bool navigableAt(const Occupancy& occupancy, const Point3& position,
                               const SweptFootprintConfig& footprint,
                               const bool require_known_free_space) noexcept {
  if constexpr (std::is_same_v<Occupancy, ObservedOccupancyGrid3D>) {
    if (!require_known_free_space) {
      return rawOccupiedFootprintIsClearAt(occupancy, position, FootprintBodyAxis{},
                                           footprint);
    }
    const SweptFootprintResult evidence =
        validateRawFootprintAt(occupancy, position, FootprintBodyAxis{}, footprint);
    return !evidence.evidence.raw_collision &&
           !evidence.evidence.outside_grid_exposure &&
           !evidence.evidence.unknown_exposure;
  }
  static_cast<void>(require_known_free_space);
  return rawFootprintIsNavigableAt(occupancy, position, FootprintBodyAxis{}, footprint);
}

template<typename Occupancy>
[[nodiscard]] bool navigableBetween(const Occupancy& occupancy, const Point3& first,
                                    const Point3& second,
                                    const SweptFootprintConfig& footprint,
                                    const bool require_known_free_space) noexcept {
  if constexpr (std::is_same_v<Occupancy, ObservedOccupancyGrid3D>) {
    if (!require_known_free_space) {
      return rawOccupiedSweptFootprintIsClear(occupancy, first, FootprintBodyAxis{},
                                              second, FootprintBodyAxis{}, footprint);
    }
    const SweptFootprintResult evidence = validateRawSweptFootprint(
        occupancy, first, FootprintBodyAxis{}, second, FootprintBodyAxis{}, footprint);
    return !evidence.evidence.raw_collision &&
           !evidence.evidence.outside_grid_exposure &&
           !evidence.evidence.unknown_exposure;
  }
  static_cast<void>(require_known_free_space);
  return rawSweptFootprintIsNavigable(occupancy, first, FootprintBodyAxis{}, second,
                                      FootprintBodyAxis{}, footprint);
}

inline void appendUniquePoint(std::vector<Point3>& output, const Point3& point) {
  if (output.empty() || distance3D(output.back(), point) > 1.0e-9) {
    output.push_back(point);
  }
}

[[nodiscard]] inline double
polylineLength(const std::span<const Point3> polyline) noexcept {
  double result = 0.0;
  for (std::size_t index = 1U; index < polyline.size(); ++index) {
    result += distance3D(polyline[index - 1U], polyline[index]);
  }
  return result;
}

} // namespace drone_city_nav::incremental_topology_detail
