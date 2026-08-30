#pragma once

#include "production_mppi_node.hpp"

namespace drone_city_nav {

struct ProductionRouteMaterialization3D {
  MaterializedRoute3D route{};
  ProductionRoutePipelineTelemetry3D telemetry{};
  StaticRouteCandidateValidation validation{};
  StaticRouteReplacementPolicy replacement_policy{
      StaticRouteReplacementPolicy::kRequireEndpointImprovement};
};

} // namespace drone_city_nav
