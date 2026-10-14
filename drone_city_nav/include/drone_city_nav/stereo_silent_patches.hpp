#pragma once

// Roadmap item 18 stage 0b: a direction the stereo pair is silent in. The
// pair gives a depth where it matched and nothing elsewhere, and its silence
// is one for every cause: too far for the light, a shadow, no texture, smoke,
// a surface that swallows the light. A frame as a whole is read by the
// braking contract (K9) and marked when blind (K14); a patch of silence
// inside a healthy frame is unknown, and unknown is free (I1). What tells a
// silent patch that is far from one that may be a surface:
//
// - its rim: the matched cells around it hold surfaces within the light's
//   reach (the pair's confident depth), with a sharp edge, all at one depth
//   (a rim that recedes into the patch is an opening, a corridor or a door
//   whose walls run on), and the rim goes round the patch: a surface the pair
//   cannot match stands against what is seen behind and beside it, where the
//   dark beyond the light's reach is bounded by the lit floor below it alone
//   and runs out of the frame (every frame of the dark location, r1496);
// - a second sensor: a time-of-flight zone that reads free through the patch
//   beyond the rim says the direction is open, whatever the pair sees.
//
// Such a patch bounds the speed of a motion into it: the braking contract
// reads the rim's depth as the range along the heading (K9, specification
// F13), as it reads a surface seen there. It is never marked in the memory:
// in the dark location lit by the vehicle's lamp alone the silence inside a
// seeing frame is shadow and darkness far more often than a surface, and
// marking it observed unobservable from one frame closed every way the
// vehicle looked at (r1496, r1497: 5 million marks, the mission lost); a
// surface the pair cannot match enters the memory by the hits the light's
// falloff and its edges still give, two to a voxel, and by the
// time-of-flight ring (r1491, r1495).

#include "drone_city_nav/stereo_depth_returns.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

struct SilentPatchConfig {
  // Cells of the stride grid a patch must hold: 200 at the stride of 4 is a
  // patch 0.45 m across at 3 m.
  std::size_t least_cells{200U};
  // Hits on the rim the judgment needs.
  std::size_t least_rim_hits{20U};
  // The rim's depths within this spread, from their first to their third
  // quartile, are one surface's edge.
  double rim_flat_m{0.5};
  // Of the eight sectors round the patch's centre, how many must hold rim
  // hits for the rim to go round it; a patch touching the frame's border has
  // no rim there and is not judged.
  std::size_t least_rim_sectors{6U};
  double confident_depth_m{5.2};
};

struct SilentPatch {
  std::size_t cells{0U};
  // The patch's cells on the stride grid, row-major indices.
  std::vector<std::size_t> cell_indices;
  std::size_t rim_hits{0U};
  double rim_depth_m{0.0};
  double rim_spread_m{0.0};
  // Whether the patch may be a surface at its rim's depth.
  bool closed{false};
  // Why not: rim beyond reach, rim recedes, too few rim hits, the rim not
  // round it, the patch at the frame's border, a time-of-flight zone reads
  // free through it.
  const char* reason{""};
};

// `returns` is the frame's returns at `returns_config.pixel_stride` (hits and
// free rays), `free_through` directions (the left camera's forward-left-up
// frame) that an active sensor read free to `free_through_range_m`. Returns
// every patch found of `config.least_cells` or more.
[[nodiscard]] std::vector<SilentPatch>
silentPatches(std::span<const StereoDepthReturn> returns,
              const StereoPairGeometry& geometry,
              const StereoDepthReturnsConfig& returns_config, std::size_t width,
              std::size_t height, const SilentPatchConfig& config,
              std::span<const Point3> free_through, double free_through_range_m);

// The depth a motion along the camera's axis is bounded by: the rim's depth
// of the closed patch whose cells hold the frame's centre, within
// `cone_half_angle_rad` of it (the heading follows the motion, K-gaze; the
// motion lies within the cone while it does), or NaN where no closed patch
// does. `stride_columns`/`stride_rows` are the stride grid's size.
[[nodiscard]] double silentRangeAheadM(std::span<const SilentPatch> patches,
                                       const StereoPairGeometry& geometry,
                                       std::size_t stride, std::size_t stride_columns,
                                       std::size_t stride_rows,
                                       double cone_half_angle_rad);

} // namespace drone_city_nav
