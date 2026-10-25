#include "drone_city_nav/stereo_depth_returns.hpp"

#include <algorithm>
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

std::vector<StereoDepthReturn>
stereoDepthReturns(const std::span<const std::int16_t> disparity_16,
                   const std::size_t width, const std::size_t height,
                   const StereoPairGeometry& geometry,
                   const StereoDepthReturnsConfig& config) {
  std::vector<StereoDepthReturn> returns;
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
      const double disparity_px = static_cast<double>(disparity) / 16.0;
      const double matched_depth_m =
          geometry.focal_px * geometry.baseline_m / disparity_px;
      const bool hit = matched_depth_m <= confident_depth_m;
      const double depth_m =
          hit ? matched_depth_m
              : std::min(confident_depth_m,
                         geometry.focal_px * geometry.baseline_m /
                             (disparity_px + config.disparity_error_px));
      // Optical frame (x right, y down, z forward) to forward-left-up.
      const double right_m = (static_cast<double>(column) - geometry.principal_x_px) *
                             depth_m / geometry.focal_px;
      const double down_m = (static_cast<double>(row) - geometry.principal_y_px) *
                            depth_m / geometry.focal_px;
      const Point3 point{depth_m, -right_m, -down_m};
      if (std::hypot(std::hypot(point.x, point.y), point.z) < config.minimum_range_m) {
        continue;
      }
      returns.push_back(StereoDepthReturn{.point = point, .hit = hit});
    }
  }
  return returns;
}

bool DarknessWatch::observe(const double stamp_s, const bool blind) noexcept {
  if (!blind) {
    if (stamp_s - last_dark_s_ > kDarknessGapS || stamp_s < last_dark_s_) {
      collapse_started_s_ = -1.0;
    }
    return false;
  }
  last_dark_s_ = stamp_s;
  if (collapse_started_s_ < 0.0 || stamp_s < collapse_started_s_) {
    collapse_started_s_ = stamp_s;
  }
  if (stamp_s - collapse_started_s_ < kDarknessS ||
      stamp_s - unobservable_confirmed_s_ < kConfirmationPeriodS) {
    return false;
  }
  unobservable_confirmed_s_ = stamp_s;
  return true;
}

std::vector<Point3> unobservableFrustumRays(const StereoPairGeometry& geometry,
                                            const std::size_t width,
                                            const std::size_t height,
                                            const int pitch_px, const double nearest_m,
                                            const double step_m, const double far_m) {
  std::vector<Point3> points;
  for (int row = pitch_px / 2; row < static_cast<int>(height); row += pitch_px) {
    for (int column = pitch_px / 2; column < static_cast<int>(width);
         column += pitch_px) {
      const double left = -(column - geometry.principal_x_px) / geometry.focal_px;
      const double up = -(row - geometry.principal_y_px) / geometry.focal_px;
      const double norm = std::sqrt(1.0 + left * left + up * up);
      for (double range_m = nearest_m; range_m <= far_m; range_m += step_m) {
        const double along = range_m / norm;
        points.push_back(Point3{along, along * left, along * up});
      }
    }
  }
  return points;
}

} // namespace drone_city_nav
