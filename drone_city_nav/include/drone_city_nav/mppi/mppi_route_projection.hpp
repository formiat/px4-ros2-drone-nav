#pragma once

#include "drone_city_nav/control_route_projection_3d.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"

#include <cstddef>
#include <span>

namespace drone_city_nav::mppi {

#if defined(__CUDACC__)
#define DRONE_CITY_NAV_MPPI_HOST_DEVICE __host__ __device__
#else
#define DRONE_CITY_NAV_MPPI_HOST_DEVICE
#endif

using MppiRouteProjection3D = drone_city_nav::ControlRouteProjection3D;

[[nodiscard]] DRONE_CITY_NAV_MPPI_HOST_DEVICE inline float
creditedRouteProgressM(const float projected_station_m, const float initial_station_m,
                       const float traveled_distance_m) noexcept {
  return creditedControlRouteProgressM3D(projected_station_m, initial_station_m,
                                         traveled_distance_m);
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_HOST_DEVICE inline float
routeTrackingSpeedMps(const State& state,
                      const MppiRouteProjection3D& projection) noexcept {
  return controlRouteTrackingSpeedMps3D(state, projection);
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_HOST_DEVICE inline MppiRouteProjection3D
projectOntoMppiRoute3D(const State& state, const RouteSample3D* route_points,
                       const std::size_t route_point_count,
                       const float minimum_station_m) noexcept {
  return projectOntoControlRoute3D(state, route_points, route_point_count,
                                   minimum_station_m);
}

[[nodiscard]] inline MppiRouteProjection3D
projectOntoMppiRoute3D(const State& state, const std::span<const RouteSample3D> route,
                       const float minimum_station_m) noexcept {
  return projectOntoControlRoute3D(state, route, minimum_station_m);
}

#undef DRONE_CITY_NAV_MPPI_HOST_DEVICE

} // namespace drone_city_nav::mppi
