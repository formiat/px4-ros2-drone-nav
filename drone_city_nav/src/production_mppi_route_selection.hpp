#pragma once

namespace drone_city_nav {

enum class ProductionRouteSearchContinuity3D {
  kCurrentState,
  kCertifiedStitch,
};

// A normal extension preserves route continuity through a certified overlap.
// A replacement search starts at the current vehicle state because its resident
// route may be the physical obstruction that requested the replacement.
[[nodiscard]] constexpr ProductionRouteSearchContinuity3D
productionRouteSearchContinuity3D(const bool extension_request,
                                  const bool replacement_request) noexcept {
  return extension_request && !replacement_request
             ? ProductionRouteSearchContinuity3D::kCertifiedStitch
             : ProductionRouteSearchContinuity3D::kCurrentState;
}

} // namespace drone_city_nav
