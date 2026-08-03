#pragma once

#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

class BoundedWorkerPool;

struct StaticRouteGeometryConfig {
  double sample_step_m{0.5};
  double maximum_shortcut_length_m{30.0};
  double corner_smoothing_distance_m{2.0};
  std::size_t corner_curve_samples{4U};
};

struct StaticRouteGeometryResult {
  std::vector<RouteSample3D> route;
  std::vector<ConstrainedRouteSpan> constrained_spans;
  std::size_t shortcuts_applied{0U};
  std::size_t corners_smoothed{0U};
  std::size_t shortcut_candidates{0U};
  std::size_t parallel_shortcut_candidates{0U};
  std::size_t corner_candidates{0U};
  std::size_t parallel_corner_candidates{0U};
  double shortcut_validation_ms{0.0};
  double corner_validation_ms{0.0};
};

[[nodiscard]] StaticRouteGeometryResult
optimizeStaticRouteGeometry(std::span<const RouteSample3D> route,
                            std::span<const ConstrainedRouteSpan> constrained_spans,
                            const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                            const SweptFootprintConfig& footprint_config,
                            const StaticRouteGeometryConfig& geometry_config,
                            const RouteEnvelopeConfig& envelope_config,
                            BoundedWorkerPool* worker_pool = nullptr);

} // namespace drone_city_nav
