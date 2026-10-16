#pragma once

// Roadmap item 18 stage 0: the patterns in which a lidar fails, read from the
// scan as a whole. A single ray with no return is ambiguous: nothing within
// range, or something that absorbed or deflected the beam (black smoke, a
// matte black surface, glass at an angle, water, a grazing wall); one ray
// cannot tell them apart. Two patterns are failure, and both are the
// memory's fail evidence (roadmap item 17 stage 8, specification I2):
//
// - an aerosol: weak returns across neighbouring rays, near and scattered,
//   where the beam gave back a little of itself from a volume rather than a
//   surface; the ray is free to the return and the return is observed
//   unobservable, a surface never measured;
// - a dropout: a bundle of rays with no return surrounded by rays that return
//   at a moderate range, from one surface (its ranges flat round the
//   bundle): a patch that swallowed the beams. The bundle's rays are free to
//   the border's range and observed unobservable there.
//
// A ray with no return whose neighbours have none either is open space and
// stays a free ray to the maximum range, as before; a no-return bundle whose
// border recedes (a corridor's walls running on past the range) is an
// opening. The dropout cannot be checked in the simulator: gpu_lidar returns
// geometry whatever the material, so the rule is designed and not verified
// until a dropout by material is modelled.

#include "drone_city_nav/lidar_scan_3d.hpp"

#include <cstddef>
#include <span>

namespace drone_city_nav {

struct LidarScanPatternsConfig {
  // The intensity the driver reports for a matte surface (the simulation's
  // relay reports one): the aerosol's reference is the scan's own surfaces,
  // never under it, so that a scan of smoke alone (every hit weak: r1546,
  // inside the plume, aerosol 0 with 14 309 weak hits) still reads as one.
  double matte_surface_intensity{1.0};
  // An aerosol's evidence lies no nearer than this: the lidar returns from
  // dense smoke round it at 0.25 to 0.5 m (tools/smoke_rig.py, 3/m), within
  // the memory's self filter (0.9 m), where the evidence would be the
  // vehicle's own and dropped (r1546: the scan read as open space).
  double aerosol_least_range_m{1.2};
  // An aerosol return under this intensity (over a matte surface's at its
  // range) is thin smoke the vehicle flies through (roadmap item 23): the
  // beam is a miss to the return and nothing is marked there; at or over
  // it the smoke is dense and its return fail evidence (item 18). The
  // relay's echo of a plume's face is 0.057 at 3/m and 0.019 at 1/m, of
  // thin smoke 0.003 at 0.1/m and 0.006 at 0.3/m; the line falls at about
  // 0.5/m, a visibility of 6 m. Read as dense, the room of 0.1/m closed in
  // front of the lidar at the range its smoke returned from, 7 m (r1661).
  double aerosol_dense_intensity{0.01};
  // A hit is weak under this share of the scan's surface intensity (the
  // median of its hits' intensities: most hits are surfaces); an aerosol
  // needs this many of its eight neighbours weak hits within the range band.
  double weak_intensity_share{0.5};
  std::size_t aerosol_least_neighbours{3U};
  double aerosol_range_band_m{1.0};
  // A dropout bundle has at most this many beams (a 2 x 2 m patch at 8 m is
  // 14 x 14 beams of the lidar's degree), hits on this share of its border
  // at least, their median range within this reach and their spread from
  // the first to the third quartile within this.
  std::size_t dropout_most_beams{400U};
  double dropout_border_hit_share{0.75};
  double dropout_border_range_m{8.0};
  double dropout_border_spread_m{1.0};
};

struct LidarScanPatternsResult {
  std::size_t aerosol_beams{0U};
  // Of them, the returns of thin smoke: misses to the return, no evidence.
  std::size_t thin_aerosol_beams{0U};
  std::size_t dropout_bundles{0U};
  std::size_t dropout_beams{0U};
};

// Rewrites `scan` in place: an aerosol hit becomes a miss to its range with
// an unobservable sample at it; a dropout bundle's beams become misses to the
// border's median range with unobservable samples there. `intensities` are
// the beams' return intensities in the beams' order, or empty (no aerosol is
// then read). Beams are organized `scan_config.horizontal_samples` a row.
LidarScanPatternsResult
classifyLidarScanPatterns3D(OrganizedLidarScan3DResult& scan,
                            std::span<const float> intensities,
                            const OrganizedLidarScan3DConfig& scan_config,
                            const LidarScanPatternsConfig& config);

} // namespace drone_city_nav
