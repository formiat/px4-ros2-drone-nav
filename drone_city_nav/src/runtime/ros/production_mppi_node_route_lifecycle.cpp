#include "production_mppi_node.hpp"

namespace drone_city_nav {

RouteSegmentCompletionAssessment3D
ProductionMppiNode::assessActiveRouteCompletion3D(const Point3& position) {
  const std::shared_ptr<const ExecutionPlan3D> snapshot = execution_supervisor_.plan();
  const CertifiedRouteSuffix3D* const route =
      snapshot != nullptr ? snapshot->route() : nullptr;
  if (route == nullptr || route->geometry == nullptr ||
      route->geometry->route == nullptr) {
    return {};
  }
  return assessRouteSegmentCompletion3D(
      *route->geometry->route, route->identity.generation,
      RouteSegmentCompletionObservation3D{
          .route_generation = route->identity.generation,
          .position = position,
          .minimum_station_m = route->progress.station_m,
      },
      RouteSegmentCompletionConfig3D{
          .capture_radius_m = config_.planning.route_completion_tolerance_m,
      });
}

} // namespace drone_city_nav
