#pragma once

#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/control_contracts_3d.hpp"

#include <memory>
#include <span>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] ControlRouteRiskTier3D
toControlRouteRiskTier3D(RouteRiskTier3D tier) noexcept;

// Derives a controller-neutral sampled view from the sealed trajectory. The
// result has no independent identity or lifecycle authority.
[[nodiscard]] std::shared_ptr<const std::vector<ControlRouteSample3D>>
adaptTrajectoryControlReference3D(const CompiledTrajectory3D& trajectory);

[[nodiscard]] std::shared_ptr<const std::vector<ControlRouteSample3D>>
adaptRouteControlVisualization3D(std::span<const RouteSample3D> route);

} // namespace drone_city_nav
