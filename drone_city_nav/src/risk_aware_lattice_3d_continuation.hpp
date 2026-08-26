#pragma once

#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <span>
#include <vector>

namespace drone_city_nav {

class BoundedWorkerPool;

namespace detail {

struct Lattice3DContinuationMetrics {
  std::size_t immediate_successors{0U};
  std::size_t reachable_states{0U};
  double reachable_depth_m{0.0};
  std::vector<Point3> path;
  Lattice3DSuccessorBatchProfile successor_profile{};
};

// Returns a materializable path only when the validated continuation moves its
// endpoint farther from route_origin than terminal. Reachable space that curls
// back along the incumbent remains diagnostic evidence, not a route extension.
[[nodiscard]] Lattice3DContinuationMetrics evaluateLattice3DContinuation(
    const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
    const Point3& route_origin, const Point3& terminal, const Vec3& incoming_direction,
    const Point3& planning_goal, Lattice3DRiskStage stage,
    const RiskAwareLattice3DConfig& config, BoundedWorkerPool* worker_pool);

} // namespace detail
} // namespace drone_city_nav
