#include "drone_city_nav/mppi/route_risk_adapter_3d.hpp"

#include "drone_city_nav/esdf_query.hpp"

#include <cmath>

namespace drone_city_nav {

RouteRiskTierAssignmentResult3D assignRouteRiskTiersFromMppiEsdf3D(
    const std::span<RouteSample3D> route, const mppi::EsdfGrid& grid,
    const std::span<const float> esdf_m, const double critical_distance_m,
    const double preferred_distance_m) noexcept {
  if (!std::isfinite(critical_distance_m) || !std::isfinite(preferred_distance_m) ||
      critical_distance_m < 0.0 || preferred_distance_m < critical_distance_m) {
    return {};
  }
  for (std::size_t index = 0U; index < route.size(); ++index) {
    RouteSample3D& sample = route[index];
    if (!std::isfinite(sample.position.x) || !std::isfinite(sample.position.y) ||
        !std::isfinite(sample.position.z)) {
      return {.status = RouteRiskTierAssignmentStatus3D::kInvalidInput,
              .failure_sample_index = index,
              .failure_point = sample.position};
    }
    const EsdfQueryResult query = queryConservativeEsdf3D(
        grid, esdf_m, static_cast<float>(sample.position.x),
        static_cast<float>(sample.position.y), static_cast<float>(sample.position.z));
    if (query.status != EsdfQueryStatus::kValid) {
      sample.required_risk_tier = RouteRiskTier3D::kPreferred;
      continue;
    }
    if (query.clearance_m < critical_distance_m) {
      sample.required_risk_tier = RouteRiskTier3D::kCritical;
    } else if (query.clearance_m < preferred_distance_m) {
      sample.required_risk_tier = RouteRiskTier3D::kPlanning;
    } else {
      sample.required_risk_tier = RouteRiskTier3D::kPreferred;
    }
  }
  return {.status = RouteRiskTierAssignmentStatus3D::kAccepted};
}

std::string_view routeRiskTierAssignmentStatus3DName(
    const RouteRiskTierAssignmentStatus3D status) noexcept {
  switch (status) {
    case RouteRiskTierAssignmentStatus3D::kAccepted:
      return "accepted";
    case RouteRiskTierAssignmentStatus3D::kInvalidInput:
      return "invalid_input";
  }
  return "invalid_status";
}

} // namespace drone_city_nav
