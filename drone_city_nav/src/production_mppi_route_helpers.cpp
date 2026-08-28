#include "production_mppi_route_helpers.hpp"

#include "drone_city_nav/route_time_parameterization.hpp"

#include <algorithm>

namespace drone_city_nav {

std::shared_ptr<const std::vector<mppi::RouteSample3D>>
makeMppiRoute2D(const std::span<const Point2> route, const double z_m,
                const double reference_speed_mps,
                const RouteEndpointSemantics3D endpoint_semantics,
                const MppiSpeedPolicyConfig& speed_policy_config,
                const mppi::DynamicsConfig& dynamics) {
  std::vector<Point3> points;
  points.reserve(route.size());
  for (const Point2 point : route) {
    points.push_back(Point3{point.x, point.y, z_m});
  }
  return makeMppiRoute3D(sampleRoute3D(points, 0.5, reference_speed_mps), {},
                         reference_speed_mps, reference_speed_mps, endpoint_semantics,
                         speed_policy_config, dynamics);
}

std::shared_ptr<const std::vector<mppi::RouteSample3D>>
makeMppiRoute3D(const std::span<const RouteSample3D> route,
                const std::span<const ConstrainedRouteSpan> spans,
                const double unconstrained_speed_mps,
                const double constrained_speed_mps,
                const RouteEndpointSemantics3D endpoint_semantics,
                const MppiSpeedPolicyConfig& speed_policy_config,
                const mppi::DynamicsConfig& dynamics) {
  auto points = std::make_shared<std::vector<mppi::RouteSample3D>>();
  points->reserve(route.size());
  const RouteTimeParameterization3D parameterization = parameterizeRouteTime3D(
      route, spans, unconstrained_speed_mps, constrained_speed_mps, endpoint_semantics,
      speed_policy_config, dynamics);
  if (!parameterization.valid ||
      parameterization.reference_speeds_mps.size() != route.size()) {
    return points;
  }
  for (std::size_t index = 0U; index < route.size(); ++index) {
    const RouteSample3D& sample = route[index];
    points->push_back(mppi::RouteSample3D{
        .x_m = static_cast<float>(sample.position.x),
        .y_m = static_cast<float>(sample.position.y),
        .z_m = static_cast<float>(sample.position.z),
        .tangent_x = static_cast<float>(sample.tangent.x),
        .tangent_y = static_cast<float>(sample.tangent.y),
        .tangent_z = static_cast<float>(sample.tangent.z),
        .station_m = static_cast<float>(sample.station_m),
        .reference_speed_mps =
            static_cast<float>(parameterization.reference_speeds_mps[index]),
        .required_risk_tier = sample.required_risk_tier,
    });
  }
  return points;
}

std::shared_ptr<const std::vector<Point2>>
projectRouteTo2D(const std::span<const RouteSample3D> route) {
  auto points = std::make_shared<std::vector<Point2>>();
  points->reserve(route.size());
  for (const RouteSample3D& sample : route) {
    points->push_back(Point2{sample.position.x, sample.position.y});
  }
  return points;
}

GlobalGuideProjection
projectOntoRouteProgress3D(const std::span<const RouteSample3D> route,
                           const Point3& position,
                           const double minimum_station_m) noexcept {
  GlobalGuideProjection projection;
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
