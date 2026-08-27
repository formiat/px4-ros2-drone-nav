#pragma once

#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

class BoundedWorkerPool;

struct StaticRouteGeometryConfig {
  bool enabled{false};
  bool shortcut_optimization_enabled{true};
  double sample_step_m{0.5};
  double maximum_shortcut_length_m{30.0};
  double sparse_deviation_tolerance_m{0.05};
  double maximum_shortcut_turn_increase_rad{0.35};
  std::size_t shortcut_validation_batch_size{4U};
  double corner_smoothing_distance_m{2.0};
  std::size_t corner_curve_samples{4U};
  // An activated route prefix is already executable evidence. Geometry
  // optimization may only operate after this station.
  std::optional<double> frozen_prefix_end_station_m;
};

struct StaticRouteGeometryRawValidation {
  const ObservedOccupancyGrid3D* occupancy{nullptr};
  const OccupancyGrid3D* static_occupancy{nullptr};
  const ProprioceptiveFreeSpaceSeed3D* proprioceptive_free_space_seed{nullptr};
  const LaunchSupportContact3D* launch_support_contact{nullptr};
  ObservedSpaceValidationPolicy policy{ObservedSpaceValidationPolicy::kAllowUnknown};
};

struct StaticRouteGeometryResult {
  std::vector<RouteSample3D> route;
  std::vector<ConstrainedRouteSpan> constrained_spans;
  std::size_t shortcuts_applied{0U};
  std::size_t corners_smoothed{0U};
  std::size_t sparse_anchor_count{0U};
  std::size_t sparse_samples_removed{0U};
  std::size_t shortcut_validation_batches{0U};
  std::size_t shortcut_turn_budget_rejections{0U};
  std::size_t shortcut_candidates{0U};
  std::size_t parallel_shortcut_candidates{0U};
  std::size_t corner_candidates{0U};
  std::size_t parallel_corner_candidates{0U};
  double shortcut_validation_ms{0.0};
  double corner_validation_ms{0.0};
};

[[nodiscard]] StaticRouteGeometryResult optimizeStaticRouteGeometry(
    std::span<const RouteSample3D> route,
    std::span<const ConstrainedRouteSpan> constrained_spans, const mppi::EsdfGrid& grid,
    std::span<const float> esdf_m, const SweptFootprintConfig& footprint_config,
    const StaticRouteGeometryConfig& geometry_config,
    const RouteEnvelopeConfig& envelope_config,
    BoundedWorkerPool* worker_pool = nullptr,
    const StaticRouteGeometryRawValidation* raw_validation = nullptr);

} // namespace drone_city_nav
