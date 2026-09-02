#pragma once

#include "drone_city_nav/footprint_geometry_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>

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

} // namespace raw_occupancy_clearance_detail

// Visits the minimum and maximum corners of every raw occupied voxel box whose
// cell lies within reach_m of the center along every axis. Contact cells of
// the launch support are skipped while the center is inside its envelope.
template<typename Occupancy, typename Visitor>
void forEachRawOccupiedVoxelNear3D(const Occupancy& occupancy, const Point3& center,
                                   const double reach_m,
                                   const LaunchSupportContact3D* const launch_support,
                                   Visitor&& visitor) {
  using raw_occupancy_clearance_detail::cellFloor;
  using raw_occupancy_clearance_detail::occupiedWords;
  constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  const GridBounds3D& bounds = occupancy.bounds();
  const double resolution_m = bounds.resolution_m;
  const int minimum_x =
      std::max(0, cellFloor(center.x - reach_m, bounds.origin_x, resolution_m));
  const int maximum_x =
      std::min(bounds.width_cells - 1,
               cellFloor(center.x + reach_m, bounds.origin_x, resolution_m));
  const int minimum_y =
      std::max(0, cellFloor(center.y - reach_m, bounds.origin_y, resolution_m));
  const int maximum_y =
      std::min(bounds.height_cells - 1,
               cellFloor(center.y + reach_m, bounds.origin_y, resolution_m));
  const int minimum_z =
      std::max(0, cellFloor(center.z - reach_m, bounds.origin_z, resolution_m));
  const int maximum_z =
      std::min(bounds.depth_cells - 1,
               cellFloor(center.z + reach_m, bounds.origin_z, resolution_m));
  if (minimum_x > maximum_x || minimum_y > maximum_y || minimum_z > maximum_z) {
    return;
  }
  const bool suppress_contact_cells =
      launch_support != nullptr &&
      launchSupportEnvelopeContains3D(*launch_support, center);
  for (int chunk_z = minimum_z / kChunkSize; chunk_z <= maximum_z / kChunkSize;
       ++chunk_z) {
    for (int chunk_y = minimum_y / kChunkSize; chunk_y <= maximum_y / kChunkSize;
         ++chunk_y) {
      for (int chunk_x = minimum_x / kChunkSize; chunk_x <= maximum_x / kChunkSize;
           ++chunk_x) {
        const auto* const chunk =
            occupancy.findChunk(OccupancyChunkIndex3D{chunk_x, chunk_y, chunk_z});
        if (chunk == nullptr) {
          continue;
        }
        std::size_t word_offset{0U};
        for (const std::uint64_t word : occupiedWords(*chunk)) {
          std::uint64_t occupied_bits = word;
          const std::size_t current_word_offset = word_offset;
          word_offset += 64U;
          while (occupied_bits != 0U) {
            const int bit_offset = std::countr_zero(occupied_bits);
            occupied_bits &= occupied_bits - 1U;
            const std::size_t bit_index =
                current_word_offset + static_cast<std::size_t>(bit_offset);
            const int cell_x =
                chunk_x * kChunkSize + static_cast<int>(bit_index % kChunkSize);
            const int cell_y = chunk_y * kChunkSize +
                               static_cast<int>((bit_index / kChunkSize) % kChunkSize);
            const int cell_z =
                chunk_z * kChunkSize +
                static_cast<int>(bit_index /
                                 static_cast<std::size_t>(kChunkSize * kChunkSize));
            if (cell_x < minimum_x || cell_x > maximum_x || cell_y < minimum_y ||
                cell_y > maximum_y || cell_z < minimum_z || cell_z > maximum_z) {
              continue;
            }
            const Point3 box_minimum{bounds.origin_x + cell_x * resolution_m,
                                     bounds.origin_y + cell_y * resolution_m,
                                     bounds.origin_z + cell_z * resolution_m};
            const Point3 box_maximum{box_minimum.x + resolution_m,
                                     box_minimum.y + resolution_m,
                                     box_minimum.z + resolution_m};
            if (suppress_contact_cells &&
                launchSupportContactContainsCell3D(*launch_support, box_minimum,
                                                   box_maximum)) {
              continue;
            }
            visitor(box_minimum, box_maximum);
          }
        }
      }
    }
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
  forEachRawOccupiedVoxelNear3D(
      occupancy, point, cap_m, launch_support,
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
  const double reach_m = cap_m + std::max({radius_m, lower_extent_m, upper_extent_m});
  double margin_m = cap_m;
  forEachRawOccupiedVoxelNear3D(
      occupancy, position, reach_m, launch_support,
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
