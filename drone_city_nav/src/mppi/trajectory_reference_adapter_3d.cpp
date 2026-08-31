#include "drone_city_nav/mppi/trajectory_reference_adapter_3d.hpp"

#include "drone_city_nav/trajectory_control_reference_3d.hpp"

namespace drone_city_nav::mppi {

RiskTier toMppiRiskTier(const RouteRiskTier3D tier) noexcept {
  return toControlRouteRiskTier3D(tier);
}

std::shared_ptr<const std::vector<RouteSample3D>>
adaptRouteVisualization3D(const std::span<const drone_city_nav::RouteSample3D> route) {
  return adaptRouteControlVisualization3D(route);
}

std::shared_ptr<const std::vector<RouteSample3D>>
adaptTrajectoryReference3D(const CompiledTrajectory3D& trajectory) {
  return adaptTrajectoryControlReference3D(trajectory);
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
