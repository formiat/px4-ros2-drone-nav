#include <limits>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

RouteSegmentCompletionAssessment3D ProductionMppiNode::assessActiveRouteCompletion3D(
    const ProductionMppiPreparedEsdf& world, const Point3& position) {
  if (!world.route_3d || !world.activated_route_3d) {
    return {};
  }

  const std::scoped_lock lock{route_supervisor_mutex_};
  RouteSegmentCompletionAssessment3D assessment = route_supervisor_.assessCompletion(
      *world.route_3d,
      RouteSegmentCompletionObservation3D{
          .route_generation = world.activated_route_3d->identity.generation,
          .position = position,
      },
      RouteSegmentCompletionConfig3D{
          .capture_radius_m =
              topological_lattice_adapter_3d_config_.segment_capture_radius_m,
      });
  if (assessment.captured) {
    static_cast<void>(route_supervisor_.applyEvent(RouteLifecycleEvent3D{
        .kind = RouteLifecycleEventKind3D::kCompleted,
        .generation = world.activated_route_3d->identity.generation,
    }));
  }
  return assessment;
}

std::uint64_t ProductionMppiNode::nextRouteGeneration3D() {
  const std::scoped_lock lock{route_supervisor_mutex_};
  if (route_supervisor_.lastAllocatedGeneration() ==
      std::numeric_limits<std::uint64_t>::max()) {
    return 0U;
  }
  return route_supervisor_.lastAllocatedGeneration() + 1U;
}

} // namespace drone_city_nav
