#include "drone_city_nav/observed_esdf_footprint_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace drone_city_nav {

double requiredObservedEsdfMaximumDistanceM(const double preferred_distance_m,
                                            const SweptFootprintConfig& footprint,
                                            const double resolution_m) noexcept {
  if (!std::isfinite(preferred_distance_m) || preferred_distance_m < 0.0 ||
      !std::isfinite(footprint.radius_m) || footprint.radius_m < 0.0 ||
      !std::isfinite(footprint.lower_extent_m) || footprint.lower_extent_m < 0.0 ||
      !std::isfinite(footprint.upper_extent_m) || footprint.upper_extent_m < 0.0 ||
      !std::isfinite(resolution_m) || resolution_m <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double bounding_radius_m = std::hypot(
      footprint.radius_m, std::max(footprint.lower_extent_m, footprint.upper_extent_m));
  return preferred_distance_m + bounding_radius_m + std::numbers::sqrt3 * resolution_m;
}

} // namespace drone_city_nav
