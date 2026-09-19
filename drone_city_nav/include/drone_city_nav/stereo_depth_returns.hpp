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

// The returns of a disparity image in the left camera's forward-left-up frame.
// `disparity_16` holds sixteenths of a pixel row by row, non-positive where the
// matcher found no depth; such a pixel, and one beyond the confident depth,
// yields no return at all.
[[nodiscard]] std::vector<Point3>
stereoDepthReturns(std::span<const std::int16_t> disparity_16, std::size_t width,
                   std::size_t height, const StereoPairGeometry& geometry,
                   const StereoDepthReturnsConfig& config);

} // namespace drone_city_nav
