#include "drone_city_nav/compiled_trajectory_views_3d.hpp"

namespace drone_city_nav {

std::vector<Point2>
projectCompiledTrajectoryTo2D(const CompiledTrajectory3D& trajectory) {
  std::vector<Point2> result;
  if (trajectory.route == nullptr || trajectory.compiled_trajectory_revision == 0U ||
      compiledTrajectoryRevision3D(trajectory) !=
          trajectory.compiled_trajectory_revision) {
    return result;
  }
  result.reserve(trajectory.route->size());
  for (const RouteSample3D& sample : *trajectory.route) {
    result.push_back(Point2{sample.position.x, sample.position.y});
  }
  return result;
}

} // namespace drone_city_nav
