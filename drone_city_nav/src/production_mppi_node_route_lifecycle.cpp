#include <limits>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

RouteSegmentCompletionAssessment3D
ProductionMppiNode::assessActiveRouteCompletion3D(const Point3& position) {
  const std::shared_ptr<const ExecutionPlan3D> snapshot =
      route_execution_manager_.plan();
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
          .capture_radius_m = route_completion_tolerance_m_,
      });
}

std::uint64_t ProductionMppiNode::nextRouteGeneration3D() {
  const std::shared_ptr<const ExecutionPlan3D> snapshot =
      route_execution_manager_.plan();
  const std::uint64_t current_generation =
      snapshot != nullptr ? snapshot->routeGenerationHighWater() : 0U;
  return current_generation == std::numeric_limits<std::uint64_t>::max()
             ? 0U
             : current_generation + 1U;
}

} // namespace drone_city_nav
