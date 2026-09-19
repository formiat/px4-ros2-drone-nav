#include "drone_city_nav/stereo_depth_returns.hpp"

#include <cmath>

namespace drone_city_nav {

bool stereoDepthReturnsConfigIsValid(const StereoPairGeometry& geometry,
                                     const StereoDepthReturnsConfig& config) noexcept {
  return std::isfinite(geometry.focal_px) && geometry.focal_px > 0.0 &&
         std::isfinite(geometry.principal_x_px) &&
         std::isfinite(geometry.principal_y_px) && std::isfinite(geometry.baseline_m) &&
         geometry.baseline_m > 0.0 && config.pixel_stride > 0U &&
         std::isfinite(config.minimum_range_m) && config.minimum_range_m >= 0.0 &&
         std::isfinite(config.disparity_error_px) && config.disparity_error_px > 0.0 &&
         std::isfinite(config.allowed_depth_error_m) &&
         config.allowed_depth_error_m > 0.0;
}

double stereoConfidentDepthM(const StereoPairGeometry& geometry,
                             const StereoDepthReturnsConfig& config) noexcept {
  return std::sqrt(config.allowed_depth_error_m * geometry.baseline_m *
                   geometry.focal_px / config.disparity_error_px);
}

std::vector<Point3> stereoDepthReturns(const std::span<const std::int16_t> disparity_16,
                                       const std::size_t width,
                                       const std::size_t height,
                                       const StereoPairGeometry& geometry,
                                       const StereoDepthReturnsConfig& config) {
  std::vector<Point3> returns;
  if (!stereoDepthReturnsConfigIsValid(geometry, config) ||
      disparity_16.size() != width * height) {
    return returns;
  }
  const double confident_depth_m = stereoConfidentDepthM(geometry, config);
  returns.reserve((width / config.pixel_stride + 1U) *
                  (height / config.pixel_stride + 1U));
  for (std::size_t row = 0U; row < height; row += config.pixel_stride) {
    for (std::size_t column = 0U; column < width; column += config.pixel_stride) {
      const std::int16_t disparity = disparity_16[row * width + column];
      if (disparity <= 0) {
        continue;
      }
      const double depth_m = geometry.focal_px * geometry.baseline_m * 16.0 /
                             static_cast<double>(disparity);
      if (depth_m > confident_depth_m) {
        continue;
      }
      // Optical frame (x right, y down, z forward) to forward-left-up.
      const double right_m = (static_cast<double>(column) - geometry.principal_x_px) *
                             depth_m / geometry.focal_px;
      const double down_m = (static_cast<double>(row) - geometry.principal_y_px) *
                            depth_m / geometry.focal_px;
      const Point3 point{depth_m, -right_m, -down_m};
      if (std::hypot(std::hypot(point.x, point.y), point.z) < config.minimum_range_m) {
        continue;
      }
      returns.push_back(point);
    }
  }
  return returns;
}

} // namespace drone_city_nav
