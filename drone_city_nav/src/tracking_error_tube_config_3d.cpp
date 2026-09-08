#include "drone_city_nav/tracking_error_tube_config_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace drone_city_nav {

bool trackingErrorTubeConfig3DIsValid(
    const TrackingErrorTubeConfig3D& config) noexcept {
  return std::isfinite(config.response_time_s) && config.response_time_s > 0.0 &&
         std::isfinite(config.minimum_progress_speed_mps) &&
         config.minimum_progress_speed_mps >= 0.0 &&
         std::isfinite(config.maximum_body_tilt_rad) &&
         config.maximum_body_tilt_rad >= 0.0 &&
         config.maximum_body_tilt_rad <= std::numbers::pi / 2.0;
}

double trackingErrorTubeRadiusM(const TrackingErrorTubeConfig3D& config,
                                const double speed_mps) noexcept {
  if (!trackingErrorTubeConfig3DIsValid(config) || !std::isfinite(speed_mps) ||
      speed_mps < 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return config.response_time_s * speed_mps;
}

double trackingErrorTubeSpeedLimitMps(const TrackingErrorTubeConfig3D& config,
                                      const double clearance_m,
                                      const double maximum_speed_mps) noexcept {
  if (!trackingErrorTubeConfig3DIsValid(config) || !std::isfinite(maximum_speed_mps) ||
      maximum_speed_mps < 0.0) {
    return 0.0;
  }
  if (!std::isfinite(clearance_m)) {
    return clearance_m > 0.0 ? maximum_speed_mps : 0.0;
  }
  const double clearance_limit_mps = std::clamp(
      std::max(0.0, clearance_m) / config.response_time_s, 0.0, maximum_speed_mps);
  return std::clamp(std::max(clearance_limit_mps, config.minimum_progress_speed_mps),
                    0.0, maximum_speed_mps);
}

} // namespace drone_city_nav
