#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <memory>
#include <span>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] std::shared_ptr<const std::vector<mppi::RouteSample3D>>
makeMppiRoute2D(std::span<const Point2> route, double z_m, double reference_speed_mps);

[[nodiscard]] std::shared_ptr<const std::vector<mppi::RouteSample3D>>
makeMppiRoute3D(std::span<const RouteSample3D> route,
                std::span<const ConstrainedRouteSpan> spans,
                double unconstrained_speed_mps, double constrained_speed_mps);

[[nodiscard]] std::shared_ptr<const std::vector<Point2>>
projectRouteTo2D(std::span<const RouteSample3D> route);

} // namespace drone_city_nav
