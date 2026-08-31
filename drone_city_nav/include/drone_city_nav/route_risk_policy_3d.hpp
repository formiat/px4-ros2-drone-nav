#pragma once

#include <cmath>

namespace drone_city_nav {

// Controller-neutral thresholds used to annotate route samples from derived
// clearance evidence. This policy never grants hard collision authority.
struct RouteRiskPolicy3D {
  double critical_distance_m{0.0};
  double preferred_distance_m{0.0};

  [[nodiscard]] bool valid() const noexcept {
    return std::isfinite(critical_distance_m) && critical_distance_m > 0.0 &&
           std::isfinite(preferred_distance_m) &&
           preferred_distance_m >= critical_distance_m;
  }
};

} // namespace drone_city_nav
