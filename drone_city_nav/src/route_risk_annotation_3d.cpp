#include "drone_city_nav/route_risk_annotation_3d.hpp"

#include "drone_city_nav/esdf_query.hpp"

#include <cmath>

namespace drone_city_nav {

RouteRiskAnnotationResult3D annotateRouteRiskTiersFromDerivedEsdf3D(
    const std::span<RouteSample3D> route, const EsdfGrid3D& grid,
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
      return {.status = RouteRiskAnnotationStatus3D::kInvalidInput,
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
  return {.status = RouteRiskAnnotationStatus3D::kAccepted};
}

std::string_view
routeRiskAnnotationStatus3DName(const RouteRiskAnnotationStatus3D status) noexcept {
  switch (status) {
    case RouteRiskAnnotationStatus3D::kAccepted:
      return "accepted";
    case RouteRiskAnnotationStatus3D::kInvalidInput:
      return "invalid_input";
  }
  return "invalid_status";
}

} // namespace drone_city_nav
