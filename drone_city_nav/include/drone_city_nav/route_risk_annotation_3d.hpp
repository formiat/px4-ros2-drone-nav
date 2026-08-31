#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/route_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace drone_city_nav {

enum class RouteRiskAnnotationStatus3D : std::uint8_t {
  kAccepted,
  kInvalidInput,
};

struct RouteRiskAnnotationResult3D {
  RouteRiskAnnotationStatus3D status{RouteRiskAnnotationStatus3D::kInvalidInput};
  std::size_t failure_sample_index{0U};
  Point3 failure_point{};

  [[nodiscard]] bool accepted() const noexcept {
    return status == RouteRiskAnnotationStatus3D::kAccepted;
  }
};

// Derived ESDF evidence may annotate a route with a softer controller risk
// tier. Invalid or unavailable distance evidence is deliberately neutral and
// never acquires hard collision authority.
[[nodiscard]] RouteRiskAnnotationResult3D annotateRouteRiskTiersFromDerivedEsdf3D(
    std::span<RouteSample3D> route, const mppi::EsdfGrid& grid,
    std::span<const float> esdf_m, double critical_distance_m,
    double preferred_distance_m) noexcept;

[[nodiscard]] std::string_view
routeRiskAnnotationStatus3DName(RouteRiskAnnotationStatus3D status) noexcept;

} // namespace drone_city_nav
