#pragma once

#include "drone_city_nav/swept_footprint.hpp"

namespace drone_city_nav {

// Collision-owned sizing rule for the observed ESDF. The observed world publishes
// distances; deciding how far those distances must reach for a given physical
// footprint is a collision concern, so the world contract stays free of it.
[[nodiscard]] double
requiredObservedEsdfMaximumDistanceM(double preferred_distance_m,
                                     const SweptFootprintConfig& footprint,
                                     double resolution_m) noexcept;

} // namespace drone_city_nav
