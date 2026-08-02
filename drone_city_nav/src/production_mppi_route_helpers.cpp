#include "production_mppi_route_helpers.hpp"

#include <algorithm>

namespace drone_city_nav {

std::shared_ptr<const std::vector<mppi::RouteSample3D>>
makeMppiRoute2D(const std::span<const Point2> route, const double z_m,
                const double reference_speed_mps) {
  std::vector<Point3> points;
  points.reserve(route.size());
  for (const Point2 point : route) {
    points.push_back(Point3{point.x, point.y, z_m});
  }
  return makeMppiRoute3D(sampleRoute3D(points, 0.5, reference_speed_mps), {},
                         reference_speed_mps, reference_speed_mps);
}

std::shared_ptr<const std::vector<mppi::RouteSample3D>>
makeMppiRoute3D(const std::span<const RouteSample3D> route,
                const std::span<const ConstrainedRouteSpan> spans,
                const double unconstrained_speed_mps,
                const double constrained_speed_mps) {
  auto points = std::make_shared<std::vector<mppi::RouteSample3D>>();
  points->reserve(route.size());
  for (const RouteSample3D& sample : route) {
    const auto constrained =
        std::ranges::find_if(spans, [&sample](const ConstrainedRouteSpan& span) {
          return sample.station_m >= span.begin_station_m &&
                 sample.station_m <= span.end_station_m;
        });
    double reference_speed_mps = unconstrained_speed_mps;
    if (constrained != spans.end()) {
      reference_speed_mps = constrained->envelope.empty()
                                ? constrained_speed_mps
                                : constrained->envelope.front().reference_speed_mps;
    }
    points->push_back(mppi::RouteSample3D{
        .x_m = static_cast<float>(sample.position.x),
        .y_m = static_cast<float>(sample.position.y),
        .z_m = static_cast<float>(sample.position.z),
        .tangent_x = static_cast<float>(sample.tangent.x),
        .tangent_y = static_cast<float>(sample.tangent.y),
        .tangent_z = static_cast<float>(sample.tangent.z),
        .station_m = static_cast<float>(sample.station_m),
        .reference_speed_mps = static_cast<float>(reference_speed_mps),
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

} // namespace drone_city_nav
