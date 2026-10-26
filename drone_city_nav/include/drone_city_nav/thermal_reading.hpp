#pragma once

#include "drone_city_nav/stereo_depth_returns.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
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
// `cone_margin_rad`, and never over `ceiling_range_m` (the source's fix over
// the vehicle's motion when there is one, else K7's 5.2 m: the rig read no
// range through 1/m and none beyond 3.7 m through 0.1/m); without hits, the
// ceiling. A hit in the cone is the plume's face or its body, nearer than
// the source, or the smoke's texture matched at a chance disparity, far
// beyond it (r2237: 4.7 m read while the source stood at 2.3): the nearer
// of the two readings is the one to keep.
[[nodiscard]] double heatSourceRangeM(const ThermalReading& reading,
                                      std::span<const StereoDepthReturn> returns,
                                      double cone_margin_rad, double ceiling_range_m);

// The source fixed in the world from its bearings over the vehicle's motion
// (a bearing-only fix): a range point laid at a guessed range ahead moves with
// the vehicle, and a bound before a moving point stops nothing (r2236: the
// vehicle crept to 1.5 m of the source at the conservative range). The fix
// is the point nearest the bearing rays of the last `window_s`, taken once
// the rays' origins spread `least_baseline_m` across the mean bearing, and
// kept while the rays pass it within `most_residual_rad` of their length on
// average: a fan of bearings from one stand, turned by the vehicle's yaw
// (r2238: the pad, the heading's errors at takeoff), has a point near the
// stand within a metre of every ray and reads as a source 2.5 m off; by
// angle it is 20 degrees off every ray.
struct HeatSourceFixConfig {
  double window_s{10.0};
  double least_baseline_m{1.0};
  // Five degrees: the core's frame against the pose of a turning vehicle (a
  // tenth of a second at 30 degrees a second is 3) and the estimate's yaw.
  double most_residual_rad{0.09};
};

class HeatSourceFix {
public:
  explicit HeatSourceFix(const HeatSourceFixConfig& config = {})
      : config_{config} {
  }

  // A bearing from `origin` along the unit `direction`, both in the world frame.
  void observe(const Point3& origin, const Point3& direction, double stamp_s);
  // The fix, when the bearings so far give one.
  [[nodiscard]] std::optional<Point3> point() const;

  [[nodiscard]] std::size_t bearings() const noexcept {
    return rays_.size();
  }

  [[nodiscard]] double baselineM() const noexcept;
  // The bearings' mean angle off the point nearest them (NaN without one).
  [[nodiscard]] double residualRad() const;

private:
  struct Ray {
    Point3 origin;
    Point3 direction;
    double stamp_s;
  };

  HeatSourceFixConfig config_;
  std::deque<Ray> rays_;
};

// The heat mark (K49): the near half of the standoff ball round the range
// point, filled on a lattice of `spacing_m`, as the memory's
// observed-unobservable rays. A shell had a hole wherever the vehicle stood
// at it: cut at 1.5 m (r2237) or by the memory's own filter of 0.9 m
// (r2239), and the vehicle flew through; a filled half is solid behind
// whatever the filter takes. From every stand the half facing the vehicle
// keeps its straight approaches out; the halves laid from the stands round a
// source make its ball. With the vehicle inside the ball (the range under
// the standoff) the whole ball is laid: the planner's escape search leads out
// of it by the nearest way, which is away from the centre.
[[nodiscard]] std::vector<Point3> heatStandoffCap(const ThermalReading& reading,
                                                  double range_m, double standoff_m,
                                                  double spacing_m);

// Whether a return's ray from the sensor crosses the standoff ball round the
// range point before its end, or ends inside it: such returns are the plume's
// own texture, or the surfaces beyond it, and are dropped, as K44 drops a
// face's chance matches (a match published through the marks carved them,
// r1847; the memory scores misses against the marks).
[[nodiscard]] bool crossesHeatStandoff(const Point3& point,
                                       const ThermalReading& reading, double range_m,
                                       double standoff_m) noexcept;

} // namespace drone_city_nav
