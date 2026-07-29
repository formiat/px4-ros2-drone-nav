#include "drone_city_nav/mapping_lifecycle.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_city_nav {

MappingLifecycle::MappingLifecycle(const double activation_altitude_m)
    : activation_altitude_m_{activation_altitude_m} {
  if (!std::isfinite(activation_altitude_m_) || activation_altitude_m_ < 0.0) {
    throw std::invalid_argument{
        "mapping activation altitude must be finite and nonnegative"};
  }
}

void MappingLifecycle::updateArmed(const bool armed) noexcept {
  if (armed_seen_ && armed_ && !armed) {
    active_ = false;
  }
  armed_ = armed;
  armed_seen_ = true;
}

bool MappingLifecycle::updateAltitude(const double altitude_m,
                                      const bool valid) noexcept {
  if (!active_ && valid && std::isfinite(altitude_m) &&
      altitude_m >= activation_altitude_m_) {
    active_ = true;
  }
  return active_;
}

bool MappingLifecycle::active() const noexcept {
  return active_;
}

} // namespace drone_city_nav
