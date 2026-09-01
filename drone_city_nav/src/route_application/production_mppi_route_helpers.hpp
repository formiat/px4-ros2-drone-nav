#pragma once

#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_progress_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <span>

namespace drone_city_nav {

// Bridges the full-3D route geometry into the remaining progress-tracker API.
// Station and cross-track distance are always measured in XYZ; the Point2
// members are visualization-only projections of the measured 3D result.
[[nodiscard]] RouteProgressProjection3D
projectOntoRouteProgress3D(std::span<const RouteSample3D> route, const Point3& position,
                           double minimum_station_m = 0.0) noexcept;

} // namespace drone_city_nav
