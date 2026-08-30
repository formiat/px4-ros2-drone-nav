#pragma once

#include "drone_city_nav/flight_time_model_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_execution_contract_3d.hpp"

#include <span>
#include <vector>

namespace drone_city_nav {

// The materialized route is the single authority for both the execution speed
// references and strategic travel-time comparison.  This keeps post-geometry
// arbitration from ranking a route by a lattice-only surrogate.
struct RouteTimeParameterization3D {
  bool valid{false};
  double travel_time_s{0.0};
  double translation_time_s{0.0};
  double stationary_turn_time_s{0.0};
  std::vector<double> reference_speeds_mps;
};

[[nodiscard]] RouteTimeParameterization3D
parameterizeRouteTime3D(std::span<const RouteSample3D> route,
                        std::span<const ConstrainedRouteSpan> constrained_spans,
                        double unconstrained_speed_mps, double constrained_speed_mps,
                        RouteEndpointSemantics3D endpoint_semantics,
                        double maximum_lateral_acceleration_mps2,
                        const FlightTimeModel3D& time_model,
                        const Vec3& initial_velocity,
                        std::span<const double> tracking_speed_limits_mps = {});

} // namespace drone_city_nav
