#pragma once

#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"

#include <memory>
#include <mutex>
#include <span>
#include <vector>

namespace drone_city_nav::mppi {

[[nodiscard]] RiskTier toMppiRiskTier(RouteRiskTier3D tier) noexcept;

// Derives the controller representation from the sealed canonical trajectory.
// The result has no independent identity or lifecycle authority.
[[nodiscard]] std::shared_ptr<const std::vector<RouteSample3D>>
adaptTrajectoryReference3D(const CompiledTrajectory3D& trajectory);

// Visualization-only adapter for synthetic direct-tracking segments that are
// not route-execution owners.
[[nodiscard]] std::shared_ptr<const std::vector<RouteSample3D>>
adaptRouteVisualization3D(std::span<const drone_city_nav::RouteSample3D> route);

// Controller-owned cache keyed by the immutable trajectory object. It avoids
// rebuilding the float/CUDA representation on every control tick without
// placing controller state inside the trajectory artifact.
class TrajectoryReferenceAdapter3D final {
public:
  [[nodiscard]] std::shared_ptr<const std::vector<RouteSample3D>>
  adapt(const std::shared_ptr<const CompiledTrajectory3D>& trajectory);

  void clear() noexcept;

private:
  std::mutex mutex_;
  std::weak_ptr<const CompiledTrajectory3D> source_;
  std::shared_ptr<const std::vector<RouteSample3D>> reference_;
};

} // namespace drone_city_nav::mppi
