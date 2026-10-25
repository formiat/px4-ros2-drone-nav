#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

// A rectified pinhole pair: the focal length in pixels, the principal point
// and the distance between the two optical centres.
struct StereoPairGeometry {
  double focal_px{0.0};
  double principal_x_px{0.0};
  double principal_y_px{0.0};
  double baseline_m{0.0};
};

struct StereoDepthReturnsConfig {
  // One return per `pixel_stride` pixels along each image axis.
  std::size_t pixel_stride{4U};
  double minimum_range_m{0.3};
  // Depth error grows with the square of the depth: for a disparity error
  // `e`, `z^2 * e / (baseline * focal)`. A pixel whose error may exceed the
  // allowed one is not evidence.
  double disparity_error_px{0.45};
  double allowed_depth_error_m{0.25};
};

[[nodiscard]] bool
stereoDepthReturnsConfigIsValid(const StereoPairGeometry& geometry,
                                const StereoDepthReturnsConfig& config) noexcept;

// The depth within which a pixel's depth error stays inside the allowed one.
[[nodiscard]] double
stereoConfidentDepthM(const StereoPairGeometry& geometry,
                      const StereoDepthReturnsConfig& config) noexcept;

// One ray of the pair: a surface at `point`, or free space along the ray to
// `point` with nothing said about what lies beyond it.
struct StereoDepthReturn {
  Point3 point{};
  bool hit{false};
  // A hit the sensor looked at and could not see through, smoke on a
  // time-of-flight zone (roadmap item 18 stage 2): the memory's fail
  // evidence, not a surface.
  bool unobservable{false};
};

// The returns of a disparity image in the left camera's forward-left-up frame.
// `disparity_16` holds sixteenths of a pixel row by row, non-positive where the
// matcher found no depth; such a pixel yields no return at all. A matched
// pixel within the confident depth is a hit. One matched deeper than that is
// no surface measurement, but the match still says the surface is no nearer
// than the disparity plus its error allows: the ray is free up to that depth,
// or up to the confident depth if that is nearer.
[[nodiscard]] std::vector<StereoDepthReturn>
stereoDepthReturns(std::span<const std::int16_t> disparity_16, std::size_t width,
                   std::size_t height, const StereoPairGeometry& geometry,
                   const StereoDepthReturnsConfig& config);

// K14 (roadmap item 17 stage 8): a frame blind for longer than the moderate
// flicker's dark is a pair that looked and could not see. Dark is what the
// braking contract reads as blind (I2; r815: read by the light alone, nothing
// was marked); the dark ends after half a second without a dark frame (r812:
// a frame a step above the line every second reset a minute's darkness); the
// frustum is laid once a second while the dark goes on.
class DarknessWatch final {
public:
  static constexpr double kDarknessS{2.5};
  static constexpr double kConfirmationPeriodS{1.0};
  static constexpr double kDarknessGapS{0.5};
  // Whether this frame, blind or not, at `stamp_s`, lays the frustum.
  [[nodiscard]] bool observe(double stamp_s, bool blind) noexcept;

private:
  double collapse_started_s_{-1.0};
  double last_dark_s_{-1.0};
  double unobservable_confirmed_s_{-1.0};
};

// The frustum the pair looked into and could not see: points along the rays
// of every `pitch_px`-th pixel from `nearest_m` to `far_m` at `step_m`, in the
// left camera's forward-left-up frame, as every return.
[[nodiscard]] std::vector<Point3>
unobservableFrustumRays(const StereoPairGeometry& geometry, std::size_t width,
                        std::size_t height, int pitch_px, double nearest_m,
                        double step_m, double far_m);

} // namespace drone_city_nav
