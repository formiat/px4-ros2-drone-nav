#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_execution_contract_3d.hpp"
#include "drone_city_nav/route_progress_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <memory>
#include <span>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] std::shared_ptr<const std::vector<mppi::RouteSample3D>>
makeMppiRoute2D(std::span<const Point2> route, double z_m, double reference_speed_mps,
                RouteEndpointSemantics3D endpoint_semantics,
                const MppiSpeedPolicyConfig& speed_policy_config = {},
                const mppi::DynamicsConfig& dynamics = {});

[[nodiscard]] std::shared_ptr<const std::vector<mppi::RouteSample3D>>
makeMppiRoute3D(std::span<const RouteSample3D> route,
                std::span<const ConstrainedRouteSpan> spans,
                double unconstrained_speed_mps, double constrained_speed_mps,
                RouteEndpointSemantics3D endpoint_semantics,
                const MppiSpeedPolicyConfig& speed_policy_config = {},
                const mppi::DynamicsConfig& dynamics = {});

[[nodiscard]] std::shared_ptr<const std::vector<Point2>>
projectRouteTo2D(std::span<const RouteSample3D> route);

// Bridges the full-3D route geometry into the remaining progress-tracker API.
// Station and cross-track distance are always measured in XYZ; the Point2
// members are visualization-only projections of the measured 3D result.
[[nodiscard]] RouteProgressProjection3D
projectOntoRouteProgress3D(std::span<const RouteSample3D> route, const Point3& position,
                           double minimum_station_m = 0.0) noexcept;

} // namespace drone_city_nav
