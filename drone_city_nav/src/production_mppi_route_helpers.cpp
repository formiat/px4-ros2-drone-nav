#include "production_mppi_route_helpers.hpp"

namespace drone_city_nav {

RouteProgressProjection3D
projectOntoRouteProgress3D(const std::span<const RouteSample3D> route,
                           const Point3& position,
                           const double minimum_station_m) noexcept {
  RouteProgressProjection3D projection;
  const RouteProjection3D measured =
      projectOntoRoute3D(route, position, minimum_station_m);
  if (!measured.valid || route.empty()) {
    return projection;
  }

  const RouteSample3D sample = sampleRoute3DAtStation(route, measured.station_m);
  projection.valid = true;
  projection.station_m = measured.station_m;
  projection.total_length_m = route.back().station_m;
  projection.remaining_m = measured.remaining_m;
  projection.cross_track_m = measured.distance_m;
  projection.point = {measured.point.x, measured.point.y};
  projection.tangent = {sample.tangent.x, sample.tangent.y};
  return projection;
}

} // namespace drone_city_nav
