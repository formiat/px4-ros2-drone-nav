#pragma once

#include "drone_city_nav/types.hpp"

#include <cstdint>

namespace drone_city_nav {

// A stamped external vehicle observation with explicit per-field validity.
// Radar modelling, truth alignment, guidance, and mission refereeing all sample
// the same shape, so it is a base model contract rather than mission state.
struct TimedVehicleState {
  Point3 position{};
  Vec3 velocity{};
  std::int64_t stamp_ns{0};
  double heading_rad{0.0};
  bool position_valid{false};
  bool velocity_valid{false};
  bool heading_valid{false};
  bool armed{false};
  bool airborne{false};
  bool navigation_ready{false};
};

} // namespace drone_city_nav
