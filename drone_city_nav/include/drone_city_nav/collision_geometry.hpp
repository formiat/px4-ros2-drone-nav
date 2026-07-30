#pragma once

#include <cmath>
#include <stdexcept>

namespace drone_city_nav {

struct VehicleFootprintGeometry {
  double rotor_center_x_m{0.48};
  double rotor_center_y_m{0.48};
  double rotor_radius_m{0.14};
};

[[nodiscard]] inline double
effectiveCollisionRadiusM(const VehicleFootprintGeometry& geometry) {
  if (!std::isfinite(geometry.rotor_center_x_m) ||
      !std::isfinite(geometry.rotor_center_y_m) ||
      !std::isfinite(geometry.rotor_radius_m) || geometry.rotor_radius_m < 0.0) {
    throw std::invalid_argument{"invalid vehicle footprint geometry"};
  }
  return std::hypot(geometry.rotor_center_x_m, geometry.rotor_center_y_m) +
         geometry.rotor_radius_m;
}

} // namespace drone_city_nav
