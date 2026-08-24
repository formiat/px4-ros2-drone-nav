#include <limits>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

RouteSegmentCompletionAssessment3D
ProductionMppiNode::assessActiveRouteCompletion3D(const ProductionMppiPreparedEsdf&,
                                                  const Point3& position) {
  const std::shared_ptr<const ExecutionRouteSnapshot3D> snapshot =
      execution_route_store_.snapshot();
  if (snapshot == nullptr || !snapshot->route.has_value() ||
      snapshot->route->geometry == nullptr ||
      snapshot->route->geometry->route == nullptr) {
    return {};
  }
  return assessRouteSegmentCompletion3D(
      *snapshot->route->geometry->route, snapshot->route->identity.generation,
      RouteSegmentCompletionObservation3D{
          .route_generation = snapshot->route->identity.generation,
          .position = position,
          .minimum_station_m = snapshot->route->progress.station_m,
      },
      RouteSegmentCompletionConfig3D{
          .capture_radius_m =
              topological_lattice_adapter_3d_config_.segment_capture_radius_m,
      });
}

std::uint64_t ProductionMppiNode::nextRouteGeneration3D() {
  const std::shared_ptr<const ExecutionRouteSnapshot3D> snapshot =
      execution_route_store_.snapshot();
  const std::uint64_t current_generation =
      snapshot != nullptr ? snapshot->routeGenerationHighWater() : 0U;
  return current_generation == std::numeric_limits<std::uint64_t>::max()
             ? 0U
             : current_generation + 1U;
}

} // namespace drone_city_nav
