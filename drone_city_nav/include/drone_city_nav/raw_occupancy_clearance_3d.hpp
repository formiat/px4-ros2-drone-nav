#pragma once

#include "drone_city_nav/footprint_geometry_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace drone_city_nav {

// Exact, capped clearance queries against raw occupied voxel boxes. They are
// derived evidence for ranking and for speed ceilings: nothing here decides
// traversability, and unknown cells contribute nothing, exactly like free ones.

// A vertical cylinder body: horizontal radius plus axial extents below and
// above the reference point.
struct RawClearanceBody3D {
  double radius_m{0.0};
  double lower_extent_m{0.0};
  double upper_extent_m{0.0};
};

namespace raw_occupancy_clearance_detail {

[[nodiscard]] inline const OccupancyGrid3D::Chunk&
occupiedWords(const OccupancyGrid3D::Chunk& chunk) noexcept {
  return chunk;
}

[[nodiscard]] inline const OccupancyGrid3D::Chunk&
occupiedWords(const ObservedOccupancyChunk3D& chunk) noexcept {
  return chunk.occupied;
}

[[nodiscard]] inline int cellFloor(const double coordinate, const double origin,
                                   const double resolution_m) noexcept {
  return static_cast<int>(std::floor((coordinate - origin) / resolution_m));
}

// Distance from a closed interval to a coordinate; zero inside the interval.
[[nodiscard]] inline double intervalGap(const double minimum, const double maximum,
                                        const double coordinate) noexcept {
  return std::max({minimum - coordinate, 0.0, coordinate - maximum});
}

// The cell box of every axis within reach_m of a center, clipped to the grid.
struct CellReach3D {
  int minimum_x{0};
  int maximum_x{-1};
  int minimum_y{0};
  int maximum_y{-1};
  int minimum_z{0};
  int maximum_z{-1};

  [[nodiscard]] bool empty() const noexcept {
    return minimum_x > maximum_x || minimum_y > maximum_y || minimum_z > maximum_z;
  }
};

[[nodiscard]] inline CellReach3D cellReach3D(const GridBounds3D& bounds,
                                             const Point3& center,
                                             const double reach_m) noexcept {
  const double resolution_m = bounds.resolution_m;
  return CellReach3D{
      .minimum_x =
          std::max(0, cellFloor(center.x - reach_m, bounds.origin_x, resolution_m)),
      .maximum_x =
          std::min(bounds.width_cells - 1,
                   cellFloor(center.x + reach_m, bounds.origin_x, resolution_m)),
      .minimum_y =
          std::max(0, cellFloor(center.y - reach_m, bounds.origin_y, resolution_m)),
      .maximum_y =
          std::min(bounds.height_cells - 1,
                   cellFloor(center.y + reach_m, bounds.origin_y, resolution_m)),
      .minimum_z =
          std::max(0, cellFloor(center.z - reach_m, bounds.origin_z, resolution_m)),
      .maximum_z =
          std::min(bounds.depth_cells - 1,
                   cellFloor(center.z + reach_m, bounds.origin_z, resolution_m)),
  };
}

// Squared distance from a point to the world-space box of one chunk.
[[nodiscard]] inline double chunkDistanceSquared3D(const GridBounds3D& bounds,
                                                   const OccupancyChunkIndex3D& index,
                                                   const Point3& point) noexcept {
  constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  const double chunk_extent_m = kChunkSize * bounds.resolution_m;
  const double minimum_x = bounds.origin_x + index.x * chunk_extent_m;
  const double minimum_y = bounds.origin_y + index.y * chunk_extent_m;
  const double minimum_z = bounds.origin_z + index.z * chunk_extent_m;
  const double dx = intervalGap(minimum_x, minimum_x + chunk_extent_m, point.x);
  const double dy = intervalGap(minimum_y, minimum_y + chunk_extent_m, point.y);
  const double dz = intervalGap(minimum_z, minimum_z + chunk_extent_m, point.z);
  return dx * dx + dy * dy + dz * dz;
}

// Visits the minimum and maximum corners of every raw occupied voxel box of one
// chunk whose cell lies inside the reach.
template<typename Chunk, typename Visitor>
void forEachOccupiedVoxelInChunk3D(const GridBounds3D& bounds, const Chunk& chunk,
                                   const OccupancyChunkIndex3D& index,
                                   const CellReach3D& reach,
                                   const bool suppress_contact_cells,
                                   const LaunchSupportContact3D* const launch_support,
                                   Visitor&& visitor) {
  constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  const double resolution_m = bounds.resolution_m;
  const int clip_minimum_x = std::max(reach.minimum_x, index.x * kChunkSize);
  const int clip_maximum_x =
      std::min(reach.maximum_x, index.x * kChunkSize + kChunkSize - 1);
  const int clip_minimum_y = std::max(reach.minimum_y, index.y * kChunkSize);
  const int clip_maximum_y =
      std::min(reach.maximum_y, index.y * kChunkSize + kChunkSize - 1);
  const int clip_minimum_z = std::max(reach.minimum_z, index.z * kChunkSize);
  const int clip_maximum_z =
      std::min(reach.maximum_z, index.z * kChunkSize + kChunkSize - 1);
  const auto visit_cell = [&](const int cell_x, const int cell_y, const int cell_z) {
    const Point3 box_minimum{bounds.origin_x + cell_x * resolution_m,
                             bounds.origin_y + cell_y * resolution_m,
                             bounds.origin_z + cell_z * resolution_m};
    const Point3 box_maximum{box_minimum.x + resolution_m, box_minimum.y + resolution_m,
                             box_minimum.z + resolution_m};
    if (suppress_contact_cells &&
        launchSupportContactContainsCell3D(*launch_support, box_minimum, box_maximum)) {
      return;
    }
    visitor(box_minimum, box_maximum);
  };
  const auto clipped_cells =
      static_cast<std::size_t>(clip_maximum_x - clip_minimum_x + 1) *
      static_cast<std::size_t>(clip_maximum_y - clip_minimum_y + 1) *
      static_cast<std::size_t>(clip_maximum_z - clip_minimum_z + 1);
  const OccupancyGrid3D::Chunk& words = occupiedWords(chunk);
  if (clipped_cells < OccupancyGrid3D::kVoxelsPerChunk / 2U) {
    // The reach covers a small part of the chunk: testing the clipped cells
    // directly is cheaper than scanning every occupied bit of a dense chunk
    // and discarding those outside the reach.
    for (int cell_z = clip_minimum_z; cell_z <= clip_maximum_z; ++cell_z) {
      for (int cell_y = clip_minimum_y; cell_y <= clip_maximum_y; ++cell_y) {
        for (int cell_x = clip_minimum_x; cell_x <= clip_maximum_x; ++cell_x) {
          const std::size_t bit =
              OccupancyGrid3D::localBitIndex(GridIndex3D{cell_x, cell_y, cell_z});
          if ((words[bit / 64U] & (std::uint64_t{1U} << (bit % 64U))) != 0U) {
            visit_cell(cell_x, cell_y, cell_z);
          }
        }
      }
    }
    return;
  }
  std::size_t word_offset{0U};
  for (const std::uint64_t word : words) {
    std::uint64_t occupied_bits = word;
    const std::size_t current_word_offset = word_offset;
    word_offset += 64U;
    while (occupied_bits != 0U) {
      const int bit_offset = std::countr_zero(occupied_bits);
      occupied_bits &= occupied_bits - 1U;
      const std::size_t bit_index =
          current_word_offset + static_cast<std::size_t>(bit_offset);
      const int cell_x =
          index.x * kChunkSize + static_cast<int>(bit_index % kChunkSize);
      const int cell_y = index.y * kChunkSize +
                         static_cast<int>((bit_index / kChunkSize) % kChunkSize);
      const int cell_z = index.z * kChunkSize +
                         static_cast<int>(bit_index / static_cast<std::size_t>(
                                                          kChunkSize * kChunkSize));
      if (cell_x < clip_minimum_x || cell_x > clip_maximum_x ||
          cell_y < clip_minimum_y || cell_y > clip_maximum_y ||
          cell_z < clip_minimum_z || cell_z > clip_maximum_z) {
        continue;
      }
      visit_cell(cell_x, cell_y, cell_z);
    }
  }
}

struct ChunkVisit3D {
  OccupancyChunkIndex3D index{};
  double distance_squared{0.0};
};

// The present chunks touching the reach, nearest to the center first.
template<typename Occupancy>
[[nodiscard]] std::vector<ChunkVisit3D> orderedChunksNear3D(const Occupancy& occupancy,
                                                            const Point3& center,
                                                            const CellReach3D& reach) {
  constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  std::vector<ChunkVisit3D> chunks;
  for (int chunk_z = reach.minimum_z / kChunkSize;
       chunk_z <= reach.maximum_z / kChunkSize; ++chunk_z) {
    for (int chunk_y = reach.minimum_y / kChunkSize;
         chunk_y <= reach.maximum_y / kChunkSize; ++chunk_y) {
      for (int chunk_x = reach.minimum_x / kChunkSize;
           chunk_x <= reach.maximum_x / kChunkSize; ++chunk_x) {
        const OccupancyChunkIndex3D index{chunk_x, chunk_y, chunk_z};
        if (occupancy.findChunk(index) == nullptr) {
          continue;
        }
        chunks.push_back(ChunkVisit3D{
            .index = index,
            .distance_squared =
                chunkDistanceSquared3D(occupancy.bounds(), index, center),
        });
      }
    }
  }
  std::ranges::sort(chunks, {}, &ChunkVisit3D::distance_squared);
  return chunks;
}

} // namespace raw_occupancy_clearance_detail

// Visits the minimum and maximum corners of every raw occupied voxel box whose
// cell lies within reach_m of the center along every axis. Contact cells of
// the launch support are skipped while the center is inside its envelope.
template<typename Occupancy, typename Visitor>
void forEachRawOccupiedVoxelNear3D(const Occupancy& occupancy, const Point3& center,
                                   const double reach_m,
                                   const LaunchSupportContact3D* const launch_support,
                                   Visitor&& visitor) {
  using raw_occupancy_clearance_detail::cellReach3D;
  using raw_occupancy_clearance_detail::forEachOccupiedVoxelInChunk3D;
  constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  const GridBounds3D& bounds = occupancy.bounds();
  const raw_occupancy_clearance_detail::CellReach3D reach =
      cellReach3D(bounds, center, reach_m);
  if (reach.empty()) {
    return;
  }
  const bool suppress_contact_cells =
      launch_support != nullptr &&
      launchSupportEnvelopeContains3D(*launch_support, center);
  for (int chunk_z = reach.minimum_z / kChunkSize;
       chunk_z <= reach.maximum_z / kChunkSize; ++chunk_z) {
    for (int chunk_y = reach.minimum_y / kChunkSize;
         chunk_y <= reach.maximum_y / kChunkSize; ++chunk_y) {
      for (int chunk_x = reach.minimum_x / kChunkSize;
           chunk_x <= reach.maximum_x / kChunkSize; ++chunk_x) {
        const OccupancyChunkIndex3D index{chunk_x, chunk_y, chunk_z};
        const auto* const chunk = occupancy.findChunk(index);
        if (chunk == nullptr) {
          continue;
        }
        forEachOccupiedVoxelInChunk3D(bounds, *chunk, index, reach,
                                      suppress_contact_cells, launch_support, visitor);
      }
    }
  }
}

// Like forEachRawOccupiedVoxelNear3D, but chunks are visited nearest first and
// `chunk_gate(distance_squared)` is consulted before each chunk with the
// squared distance from the center to that chunk's box; a false answer ends
// the visit, since every later chunk lies at least as far. Nearest-box
// searches stop as soon as no unvisited chunk can beat their best box.
template<typename Occupancy, typename ChunkGate, typename Visitor>
void forEachRawOccupiedVoxelNearestFirst3D(
    const Occupancy& occupancy, const Point3& center, const double reach_m,
    const LaunchSupportContact3D* const launch_support, ChunkGate&& chunk_gate,
    Visitor&& visitor) {
  using raw_occupancy_clearance_detail::cellReach3D;
  using raw_occupancy_clearance_detail::forEachOccupiedVoxelInChunk3D;
  using raw_occupancy_clearance_detail::orderedChunksNear3D;
  const GridBounds3D& bounds = occupancy.bounds();
  const raw_occupancy_clearance_detail::CellReach3D reach =
      cellReach3D(bounds, center, reach_m);
  if (reach.empty()) {
    return;
  }
  const bool suppress_contact_cells =
      launch_support != nullptr &&
      launchSupportEnvelopeContains3D(*launch_support, center);
  for (const raw_occupancy_clearance_detail::ChunkVisit3D& visit :
       orderedChunksNear3D(occupancy, center, reach)) {
    if (!chunk_gate(visit.distance_squared)) {
      return;
    }
    const auto* const chunk = occupancy.findChunk(visit.index);
    if (chunk == nullptr) {
      continue;
    }
    forEachOccupiedVoxelInChunk3D(bounds, *chunk, visit.index, reach,
                                  suppress_contact_cells, launch_support, visitor);
  }
}

// Euclidean distance from a point to the nearest raw occupied voxel box,
// capped at cap_m. The cap is returned when nothing occupied lies within it.
template<typename Occupancy>
[[nodiscard]] double
rawEuclideanClearance3D(const Occupancy& occupancy, const Point3& point,
                        const double cap_m,
                        const LaunchSupportContact3D* const launch_support = nullptr) {
  using raw_occupancy_clearance_detail::intervalGap;
  double best_squared = cap_m * cap_m;
  forEachRawOccupiedVoxelNearestFirst3D(
      occupancy, point, cap_m, launch_support,
      [&](const double chunk_distance_squared) {
        return chunk_distance_squared < best_squared;
      },
      [&](const Point3& box_minimum, const Point3& box_maximum) {
        const double dx = intervalGap(box_minimum.x, box_maximum.x, point.x);
        const double dy = intervalGap(box_minimum.y, box_maximum.y, point.y);
        const double dz = intervalGap(box_minimum.z, box_maximum.z, point.z);
        best_squared = std::min(best_squared, dx * dx + dy * dy + dz * dz);
      });
  return std::sqrt(best_squared);
}

// The largest uniform inflation of the body (radius and both axial extents
// grown by the same margin) that still clears every raw occupied voxel box,
// capped at cap_m. This is the exact quantity a swept-footprint bisection over
// inflated footprints converges to for a single body pose. Zero means the
// body already touches occupied evidence.
template<typename Occupancy>
[[nodiscard]] double
rawBodyInflationMargin3D(const Occupancy& occupancy, const Point3& position,
                         const RawClearanceBody3D& body, const double cap_m,
                         const LaunchSupportContact3D* const launch_support = nullptr) {
  using raw_occupancy_clearance_detail::intervalGap;
  const double radius_m = std::max(0.0, body.radius_m);
  const double lower_extent_m = std::max(0.0, body.lower_extent_m);
  const double upper_extent_m = std::max(0.0, body.upper_extent_m);
  const double axial_extent_m = std::max(lower_extent_m, upper_extent_m);
  const double reach_m = cap_m + std::max(radius_m, axial_extent_m);
  double margin_m = cap_m;
  forEachRawOccupiedVoxelNearestFirst3D(
      occupancy, position, reach_m, launch_support,
      [&](const double chunk_distance_squared) {
        // A box lowers the margin only when both its horizontal and its
        // vertical gap fall short of it, which bounds the box's distance by
        // the inflated radius and extent; a chunk at least that far away
        // holds no such box.
        const double inflated_radius_m = radius_m + margin_m;
        const double inflated_extent_m = axial_extent_m + margin_m;
        return chunk_distance_squared < inflated_radius_m * inflated_radius_m +
                                            inflated_extent_m * inflated_extent_m;
      },
      [&](const Point3& box_minimum, const Point3& box_maximum) {
        const double dx = intervalGap(box_minimum.x, box_maximum.x, position.x);
        const double dy = intervalGap(box_minimum.y, box_maximum.y, position.y);
        const double horizontal_gap_m = std::hypot(dx, dy) - radius_m;
        const double vertical_gap_m =
            std::max(box_minimum.z - (position.z + upper_extent_m),
                     (position.z - lower_extent_m) - box_maximum.z);
        // The inflated cylinder misses the box while either gap exceeds the
        // margin, so this box bounds the margin by the larger of the two.
        margin_m =
            std::min(margin_m, std::max({horizontal_gap_m, vertical_gap_m, 0.0}));
      });
  return margin_m;
}

} // namespace drone_city_nav
