#include "drone_city_nav/mppi/trajectory_reference_adapter_3d.hpp"

namespace drone_city_nav::mppi {

RiskTier toMppiRiskTier(const RouteRiskTier3D tier) noexcept {
  switch (tier) {
    case RouteRiskTier3D::kPreferred:
      return RiskTier::kPreferred;
    case RouteRiskTier3D::kPlanning:
      return RiskTier::kPlanning;
    case RouteRiskTier3D::kCritical:
      return RiskTier::kCritical;
    case RouteRiskTier3D::kCollision:
      return RiskTier::kCollision;
  }
  return RiskTier::kCollision;
}

std::shared_ptr<const std::vector<RouteSample3D>>
adaptRouteVisualization3D(const std::span<const drone_city_nav::RouteSample3D> route) {
  auto result = std::make_shared<std::vector<RouteSample3D>>();
  result->reserve(route.size());
  for (const drone_city_nav::RouteSample3D& sample : route) {
    result->push_back(RouteSample3D{
        .x_m = static_cast<float>(sample.position.x),
        .y_m = static_cast<float>(sample.position.y),
        .z_m = static_cast<float>(sample.position.z),
        .tangent_x = static_cast<float>(sample.tangent.x),
        .tangent_y = static_cast<float>(sample.tangent.y),
        .tangent_z = static_cast<float>(sample.tangent.z),
        .station_m = static_cast<float>(sample.station_m),
        .reference_speed_mps = static_cast<float>(sample.reference_speed_mps),
        .required_risk_tier = toMppiRiskTier(sample.required_risk_tier),
    });
  }
  return result;
}

std::shared_ptr<const std::vector<RouteSample3D>>
adaptTrajectoryReference3D(const CompiledTrajectory3D& trajectory) {
  if (trajectory.compiled_trajectory_revision == 0U || trajectory.route == nullptr ||
      compiledTrajectoryRevision3D(trajectory) !=
          trajectory.compiled_trajectory_revision) {
    return nullptr;
  }
  return adaptRouteVisualization3D(*trajectory.route);
}

std::shared_ptr<const std::vector<RouteSample3D>> TrajectoryReferenceAdapter3D::adapt(
    const std::shared_ptr<const CompiledTrajectory3D>& trajectory) {
  const std::scoped_lock lock{mutex_};
  if (trajectory == nullptr) {
    source_.reset();
    reference_.reset();
    return nullptr;
  }
  if (source_.lock() == trajectory && reference_ != nullptr) {
    return reference_;
  }
  reference_ = adaptTrajectoryReference3D(*trajectory);
  if (reference_ != nullptr) {
    source_ = trajectory;
  } else {
    source_.reset();
  }
  return reference_;
}

void TrajectoryReferenceAdapter3D::clear() noexcept {
  const std::scoped_lock lock{mutex_};
  source_.reset();
  reference_.reset();
}

} // namespace drone_city_nav::mppi
