#pragma once

#include "drone_city_nav/stereo_depth_returns.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

// The thermal core's frame read for a heat source (roadmap item 32,
// specification K49): longwave infrared crosses smoke almost untouched (K39),
// so the core sees a fire's body through the plume that carries it, which
// the pair, the ring and the lidar read by density alone. The frame is the
// core's 16 bits a pixel in hundredths of a kelvin.
struct ThermalReadingConfig {
  // The core's horizontal field of view (the FLIR Lepton 3.5 class, 57
  // degrees over 160 pixels).
  double horizontal_fov_rad{0.9948376736367679};
  // A pixel this much over the frame's median is hot. The rig (2026-10-26):
  // the location's surfaces read the atmosphere's 288 K and a wall's 311,
  // a spread of 23 K that 5 to 20 K flag and 30 K do not.
  double above_median_k{30.0};
  // Fewer hot pixels are no source: a 0.3 m body gives 6 at 30 m and 16 at
  // 20 (the rig), a 0.1 m one 20 at 5.7.
  std::size_t least_hot_pixels{4U};
};

struct ThermalReading {
  // The share of the frame's pixels over the median by the threshold.
  double hot_share{0.0};
  // The hot pixels when they make a source, else nought.
  std::size_t hot_pixels{0U};
  // The hot pixels' centre as a unit direction in the core's forward-left-up
  // frame, and the angular radius of the region round it (half a pixel added).
  Point3 direction{1.0, 0.0, 0.0};
  double half_angle_rad{0.0};

  [[nodiscard]] bool hot() const noexcept {
    return hot_pixels > 0U;
  }
};

[[nodiscard]] ThermalReading
readThermalFrame(std::span<const std::uint16_t> kelvin_hundredths, std::size_t width,
                 std::size_t height, const ThermalReadingConfig& config);

// The range to the source: the median range of the hits (the ring's or the
// pair's, whichever has them) within the hot region's cone, widened by
// `cone_margin_rad`; without one, `conservative_range_m` (K7's 5.2 m: the rig
// read no range through 1/m and none beyond 3.7 m through 0.1/m).
[[nodiscard]] double heatSourceRangeM(const ThermalReading& reading,
                                      std::span<const StereoDepthReturn> returns,
                                      double cone_margin_rad,
                                      double conservative_range_m);

// The heat mark (K49): the near half of the standoff ball round the range
// point, as the memory's observed-unobservable rays at `spacing_m`, nothing
// nearer than `nearest_m` (K44's: a mark pushed to the vehicle boxed it in,
// r1837). From every stand the half facing the vehicle keeps its straight
// approaches out; the halves laid from the stands round a source make its
// ball.
[[nodiscard]] std::vector<Point3> heatStandoffCap(const ThermalReading& reading,
                                                  double range_m, double standoff_m,
                                                  double spacing_m, double nearest_m);

// Whether a point lies inside the standoff ball round the range point: the
// pair's returns there are the plume's own texture and are dropped, as K44
// drops a face's (a match published inside carved the marks, r1847).
[[nodiscard]] bool withinHeatStandoff(const Point3& point,
                                      const ThermalReading& reading, double range_m,
                                      double standoff_m) noexcept;

} // namespace drone_city_nav
