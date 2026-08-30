#pragma once

#include <cstdint>

namespace drone_city_nav {

// Controller-neutral risk annotation carried by spatial routes and compiled
// trajectories. Derived clearance may select a softer controller policy, but
// this tier never has hard-collision authority.
enum class RouteRiskTier3D : std::uint8_t {
  kPreferred = 0,
  kPlanning = 1,
  kCritical = 2,
  kCollision = 3,
};

} // namespace drone_city_nav
