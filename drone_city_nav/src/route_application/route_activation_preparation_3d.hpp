#pragma once

#include "route_activation_coordinator_3d.hpp"
#include "route_trajectory_compiler_3d.hpp"

namespace drone_city_nav {

// Runs one ROS-free activation preparation transaction through the ordered,
// side-effect-free stages owned by RouteActivationCoordinator3D.
[[nodiscard]] PreparedRouteActivation3D
prepareRouteActivation3D(RouteActivationPreparationRequest3D request,
                         const RouteActivationCoordinatorConfig3D& config,
                         const RouteTrajectoryCompiler3D& trajectory_compiler);

} // namespace drone_city_nav
