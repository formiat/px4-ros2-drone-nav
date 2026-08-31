#include "drone_city_nav/trajectory_control_reference_3d.hpp"

namespace drone_city_nav {

ControlRouteRiskTier3D toControlRouteRiskTier3D(const RouteRiskTier3D tier) noexcept {
  switch (tier) {
    case RouteRiskTier3D::kPreferred:
      return ControlRouteRiskTier3D::kPreferred;
    case RouteRiskTier3D::kPlanning:
      return ControlRouteRiskTier3D::kPlanning;
    case RouteRiskTier3D::kCritical:
      return ControlRouteRiskTier3D::kCritical;
    case RouteRiskTier3D::kCollision:
      return ControlRouteRiskTier3D::kCollision;
  }
  return ControlRouteRiskTier3D::kCollision;
}

std::shared_ptr<const std::vector<ControlRouteSample3D>>
adaptRouteControlVisualization3D(const std::span<const RouteSample3D> route) {
  auto result = std::make_shared<std::vector<ControlRouteSample3D>>();
  result->reserve(route.size());
  for (const RouteSample3D& sample : route) {
    result->push_back(ControlRouteSample3D{
        .x_m = static_cast<float>(sample.position.x),
        .y_m = static_cast<float>(sample.position.y),
        .z_m = static_cast<float>(sample.position.z),
        .tangent_x = static_cast<float>(sample.tangent.x),
        .tangent_y = static_cast<float>(sample.tangent.y),
        .tangent_z = static_cast<float>(sample.tangent.z),
        .station_m = static_cast<float>(sample.station_m),
        .reference_speed_mps = static_cast<float>(sample.reference_speed_mps),
        .required_risk_tier = toControlRouteRiskTier3D(sample.required_risk_tier),
    });
  }
  return result;
}

std::shared_ptr<const std::vector<ControlRouteSample3D>>
adaptTrajectoryControlReference3D(const CompiledTrajectory3D& trajectory) {
  if (trajectory.compiled_trajectory_revision == 0U || trajectory.route == nullptr ||
      compiledTrajectoryRevision3D(trajectory) !=
          trajectory.compiled_trajectory_revision) {
    return nullptr;
  }
  return adaptRouteControlVisualization3D(*trajectory.route);
}

} // namespace drone_city_nav
