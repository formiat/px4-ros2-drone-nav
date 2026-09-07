#pragma once

#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <array>
#include <cmath>
#include <cstddef>

// Level-zero edge bookkeeping shared by the lattice sources: the canonical
// edge offsets, the per-edge state codes and the grid geometry comparison.

namespace drone_city_nav::detail {

inline constexpr double kGeometryTolerance{1.0e-9};

[[nodiscard]] inline bool sameBounds(const GridBounds3D& first,
                                     const GridBounds3D& second) noexcept {
  return std::abs(first.origin_x - second.origin_x) <= kGeometryTolerance &&
         std::abs(first.origin_y - second.origin_y) <= kGeometryTolerance &&
         std::abs(first.origin_z - second.origin_z) <= kGeometryTolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= kGeometryTolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

inline constexpr unsigned kLevelZeroEdgeUnknown{0U};
inline constexpr unsigned kLevelZeroEdgeClear{1U};
inline constexpr unsigned kLevelZeroEdgeBlocked{2U};
// The straight segment between two node centres does not clear the body, but a
// short step off it does: the edge is traversable through a stored waypoint.
// The lattice steps 2 m horizontally against a 0.25 m map, so a doorway can be
// wide enough for the body and still admit no straight node-to-node segment
// unless its centre happens to fall on the grid.
//
// The state is the single source of truth. The waypoint map is consulted only
// while the state says refined, because a bulk clear or an occupied change can
// overwrite the state without the map knowing, and a waypoint validated
// against an older world must never reach a path.
inline constexpr unsigned kLevelZeroEdgeRefined{3U};

struct LevelZeroOffset3D {
  int x{0};
  int y{0};
  int z{0};
};

// The thirteen offsets that are lexicographically positive in (z, y, x)
// order: exactly the second endpoints of canonical level-zero edges.
inline constexpr std::array<LevelZeroOffset3D, 13U> kLevelZeroOffsets{{
    {1, 0, 0},
    {-1, 1, 0},
    {0, 1, 0},
    {1, 1, 0},
    {-1, -1, 1},
    {0, -1, 1},
    {1, -1, 1},
    {-1, 0, 1},
    {0, 0, 1},
    {1, 0, 1},
    {-1, 1, 1},
    {0, 1, 1},
    {1, 1, 1},
}};

// Index of a canonical offset in kLevelZeroOffsets: the offsets enumerate the
// codes 14..26 of (z + 1) * 9 + (y + 1) * 3 + (x + 1).
[[nodiscard]] constexpr std::size_t levelZeroDirection(const int x, const int y,
                                                       const int z) noexcept {
  return static_cast<std::size_t>((z + 1) * 9 + (y + 1) * 3 + (x + 1) - 14);
}

} // namespace drone_city_nav::detail
