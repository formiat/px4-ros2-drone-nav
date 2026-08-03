#pragma once

#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <span>

namespace drone_city_nav {

class BoundedWorkerPool;

namespace detail {

struct Lattice3DContinuationMetrics {
  std::size_t immediate_successors{0U};
  std::size_t reachable_states{0U};
  double reachable_depth_m{0.0};
  Lattice3DSuccessorBatchProfile successor_profile{};
};

[[nodiscard]] Lattice3DContinuationMetrics evaluateLattice3DContinuation(
    const mppi::EsdfGrid& grid, std::span<const float> esdf_m, const Point3& terminal,
    const Vec3& incoming_direction, Lattice3DRiskStage stage,
    const RiskAwareLattice3DConfig& config, BoundedWorkerPool* worker_pool);

} // namespace detail
} // namespace drone_city_nav
