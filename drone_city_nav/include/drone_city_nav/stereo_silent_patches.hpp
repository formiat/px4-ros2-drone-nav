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
#include <cstdint>
#include <optional>
#include <span>
#include <string>
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
  // Free rays beside the patch's cells: matches deeper than the confident
  // depth the silence encloses or borders. A closed patch the pair still
  // matches through is thin smoke, not a face (the smoke face, below).
  std::size_t free_neighbours{0U};
  // The cells of the free rays a region of lit silence encloses (smokeFaces).
  std::vector<std::size_t> enclosed_free_cells;
  double rim_depth_m{0.0};
  double rim_spread_m{0.0};
  // The share of the rim depths' variance a plane over the rim explains (a
  // face's judgment, K44): nought for a scatter, one for a recession.
  double rim_trend{0.0};
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

// A face of dense smoke, or a surface the pair cannot texture, in the frame
// (roadmap item 18, the smoke face detector, specification K44; the owner,
// 2026-10-18: dense smoke is an obstacle the vehicle sees and never enters).
// The silent cells that are LIT (raw grey, before the driver's gain, at
// `lit_raw_grey` or more) are the pair's silence on something the lamp
// reaches: a plume, or a surface the pair cannot texture; the dark silence
// is shadow or the dark beyond the lamp (F13) and is left alone. A
// connected region of lit silence the pair does not see through (free rays
// it encloses, matches deeper than the confident depth with no hit beside
// them, under `see_through_share` of its cells) is a face in one of two
// ways:
// - rimmed: its rim of hits lies at one depth within the lamp's reach and
//   goes round it from `least_rim_sectors` of the eight sectors (a closed
//   patch's rim, the top allowed missing: a plume fills a corridor to the
//   frame's top, r1745, and the dark above it gives no rim; the veil of thin
//   smoke in a hall is lit silence rimmed by the floor's fade below alone,
//   r1769, as the dark beyond the lamp is). The face is at the rim's depth.
// - a lit body in the dark: no rim (a plume floating above the horizon in a
//   dark hall, r1788, 0.4 m over the vehicle's path), the region inside the
//   frame, `dark_ring_share` or more of its neighbours dark silence, and a
//   SHARP edge: the silence three cells outside it reads `shell_ratio` of
//   the region's median or less. The lamp's cone on the veil of a thin
//   hall is a lit body in the dark too, with a soft edge: the bell of the
//   cone, 1 2 4 9 15 22 26 31 34 36 across the frame on the rig, against a
//   plume's plateau, 1 6 35 37 38 ... 35 21 1 (tools/smoke_rig.py
//   height=8). The face has no rim to measure: it is placed by its
//   brightness, nearer than it can be (the rig: 49 to 58 raw grey at 2 m,
//   33 to 39 at 4 m, under 10 at 6 m), `near_m` at `near_grey` or more,
//   `mid_m` at `mid_grey` or more, `far_m` else.
// The face enters the memory as observed unobservable from its depth over
// `face_slab_m` (K14's frustum rays) and bounds the motion into it as a
// closed patch does. Measured on the smoke rig (tools/smoke_rig.py,
// tools/smoke_signature_report.py, log/tools/smoke_detector/part0): in a
// 4.4 m corridor the face of 3, 1 and 0.5 per metre within 4 m reads 35 to
// 58 raw grey, the dark beyond the lamp 0 to 10; the far end of a corridor
// of 0.1 per metre is lit silence too (rim 4.8 m, raw 37 to 39) but the pair
// matches through it, 1.6 to 3.8 percent of its cells deeper than the rim,
// against 0 to 0.1 percent for 0.5 per metre and more; a plume floating
// before a carrier 8 m up reads 34 to 38 with the shell three cells out at
// 0.13 to 0.26 of it, the veil of 0.1 per metre there 28 with the shell at
// 0.61. The rim's median overshoots the true face by 0.6 to 1.3 m (the pair
// sees into the face's soft edge).
struct SmokeFaceConfig {
  double lit_raw_grey{20.0};
  // The free rays a region encloses are dropped before the memory while the
  // region is a face or near one, its see-through under this share: a
  // plume's region flickers between face and seen_through as its chance far
  // matches cross the 1 percent (r1847), and each such ray carved the
  // memory along its 5.2 m and lifted the slab. The halls' open regions
  // carry 10 to 17 percent, real far surfaces, and keep theirs (r1866: every
  // enclosed ray dropped, 215 000 a flight, the stale marks never lifted,
  // the routes closed, B not reached).
  double carve_share{0.03};
  // Lit silence brighter than this is a surface under the lamp at close
  // range, saturated past its texture, not smoke: a plume reads 49 to 58
  // raw grey at 2 m and 43 to 71 inside (the rig), the rig's floor blocks
  // 70 to 110 at 1.4 m, a matte panel 171 to 231 at 3 m. r1837 and r1822:
  // such a surface 1.2 m ahead was a face every frame, its marks closed the
  // way, and the vehicle hovered 400 to 545 s.
  double surface_raw_grey{80.0};
  // The enclosed free rays a face may hold: thin smoke of 0.1 per metre
  // leaves 1.6 to 3.8 percent on the rig; a dense plume 4 to 6 m off, dim,
  // leaves 0.5 to 4 percent of spurious far matches on its own churning
  // texture (r1806), 0 to 0.1 percent nearer.
  double see_through_share{0.01};
  double face_slab_m{1.5};
  std::size_t least_rim_sectors{5U};
  // A plume's edge is soft: the pair sees 0.6 to 1.3 m into it, and the rim's
  // depths SCATTER by up to 1 m across the face (r1806: 0.56 at 1.8 m, r1807:
  // 0.57 to 0.91 on the approach, no face under K36's 0.5 m), with no trend
  // along the rim: a plane fitted to the rim's depths over the cells'
  // rows and columns explains 0.00 to 0.16 of their variance on the rig. A
  // hall's walls running on RECEDE along the rim, the plane explaining 0.46
  // to 0.95 (r1822: lit silences in the halls with spreads of 0.6 to 0.8
  // judged faces, the routes closed, B not reached; r1873: hundreds of
  // untextured patches of floor and wall under the lamp, seen obliquely,
  // with flat rims of 0.01 to 0.48 m and trends of 0.90 to 0.99, their slabs
  // narrowing every way). A face's rim spreads by up to `rim_scatter_m` and
  // the trend explains less than `rim_trend_share` of it, whatever the
  // spread. The face is at the rim's nearest quartile.
  double rim_scatter_m{1.0};
  double rim_trend_share{0.3};
  double dark_ring_share{0.7};
  double shell_ratio{0.35};
  double near_grey{45.0};
  double near_m{1.5};
  double mid_grey{30.0};
  double mid_m{2.5};
  double far_m{3.5};
  // A rimmed face stands in the world, not with the lamp: as the vehicle
  // travels toward it, its rim comes nearer by the travel. The veil of thin
  // smoke keeps its rim at the lamp's reach however the vehicle moves
  // (r1867: in the west hall of 0.1 per metre, 5 m over its floor, the veil
  // read a face at 2.1 m frame after frame, and its marks held the vehicle
  // high and blind). A rimmed face is confirmed once the vehicle has
  // travelled `least_travel_m` along the camera's axis since the face was
  // first seen and the rim's depth, regressed on the travel over every
  // frame since, falls by `shrink_slope` or steeper per metre of it (a
  // plume's by one; a veil's by nought: its rim's quartile scatters by up to
  // 0.8 m from frame to frame, and the first depth against the least seen
  // confirmed the veil of the west hall, r1871, 70 000 marks in 13 s). A lit
  // body in the dark, whose depth is a brightness step, is confirmed by
  // frames instead.
  double least_travel_m{1.2};
  double shrink_slope{0.5};
};

// The confirmation of a rimmed face by the vehicle's travel toward it
// (SmokeFaceConfig). `observe` takes each frame: whether a rimmed face
// stands in it, the nearest rim's depth, and the vehicle's travel along the
// camera's axis so far (the autopilot's position projected on its heading,
// summed); a frame without a rimmed face starts over.
class FaceApproach {
public:
  void observe(bool face, double rim_depth_m, double along_m,
               const SmokeFaceConfig& config);

  [[nodiscard]] bool confirmed() const noexcept {
    return confirmed_;
  }

private:
  // The regression of the rim's depth on the travel: the sums of the frames
  // since the face was first seen.
  std::optional<double> first_along_m_;
  double count_{0.0};
  double sum_a_{0.0};
  double sum_d_{0.0};
  double sum_aa_{0.0};
  double sum_ad_{0.0};
  bool confirmed_{false};
};

// The frame's raw grey on the stride grid, one byte a cell, row-major: the
// grey at the cell's pixel (the frame's rows `row_step` bytes apart) over
// the driver's gain, saturating at 255.
[[nodiscard]] std::vector<std::uint8_t> rawCells(std::span<const std::uint8_t> grey,
                                                 std::size_t row_step,
                                                 std::size_t width, std::size_t height,
                                                 std::size_t stride, double gain);

// Seen where a return, a hit or a free ray, fell in the cell.
[[nodiscard]] std::vector<std::uint8_t>
seenCells(std::span<const StereoDepthReturn> returns,
          const StereoPairGeometry& geometry, std::size_t stride, std::size_t width,
          std::size_t height);

// The stride cell a direction (the rectified forward-left-up frame) falls
// in, or none outside the frame.
[[nodiscard]] std::optional<std::size_t>
strideCellOf(const Point3& direction, const StereoPairGeometry& geometry,
             std::size_t stride, std::size_t width, std::size_t height);

// The faces' rays for the memory: through the cells of each face (the closed
// regions of `regions` whose reason is `reason`) every `pitch_cells` cells each way,
// from the face's depth over `slab_m` in steps of `step_m`; the rectified
// forward-left-up frame, as every return. The slab starts at the face, not before it: a
// lit surface the pair cannot texture is a face too, and marks before its rim would
// stand in free air (r1746: 48 670 marks over a flight, a metre before every such
// surface). A face nearer than `nearest_m` gives no rays: that is the ring's reach and
// the frustum's own nearest (K14), and a slab pushed out to it from a surface 1.2 m off
// boxed the vehicle in (r1837).
[[nodiscard]] std::vector<Point3>
smokeFaceRays(std::span<const SilentPatch> regions, const char* reason,
              const StereoPairGeometry& geometry, std::size_t stride, std::size_t width,
              double nearest_m, double slab_m, double step_m, std::size_t pitch_cells);

// Drops the pair's free rays the regions of lit silence enclose where the
// region's see-through is under `carve_share` (a face or near one): a match
// far beyond the lamp's reach inside a plume's silence is the smoke's own
// texture matched at a chance disparity, and published it carved the memory
// along its 5.2 m and lifted the face's marks (r1847). Returns how many were
// dropped.
[[nodiscard]] std::size_t dropEnclosedFreeRays(std::vector<StereoDepthReturn>& returns,
                                               std::span<const SilentPatch> regions,
                                               const StereoPairGeometry& geometry,
                                               std::size_t stride, std::size_t width,
                                               std::size_t height, double carve_share);

// A patch or a face in a log line:
// cells@column,row:reason:rimN@depth+spread:trend:freeN.
[[nodiscard]] std::string silentPatchSummary(const SilentPatch& patch);

// The summaries of the closed regions of `reason` at `nearest_m` or beyond,
// one space before each: the faces whose rays are laid (smokeFaceRays).
[[nodiscard]] std::string smokeFacesLaid(std::span<const SilentPatch> regions,
                                         const char* reason, double nearest_m);

// The faces of the frame: `raw_cells` is the frame's raw grey a stride cell
// (rawCells); the rest as silentPatches. Each face is a SilentPatch with
// `closed` true, its depth in `rim_depth_m`, and the reason "face" or
// "body"; regions of lit silence that are not faces are returned too, with
// their reason.
[[nodiscard]] std::vector<SilentPatch>
smokeFaces(std::span<const StereoDepthReturn> returns,
           const StereoPairGeometry& geometry,
           const StereoDepthReturnsConfig& returns_config, std::size_t width,
           std::size_t height, std::span<const std::uint8_t> raw_cells,
           const SilentPatchConfig& config, const SmokeFaceConfig& face_config);

} // namespace drone_city_nav
