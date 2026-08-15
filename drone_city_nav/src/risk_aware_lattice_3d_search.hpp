#pragma once

#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <cstdint>
#include <span>

namespace drone_city_nav::detail {

enum class Lattice3DTopologyRequirement : std::uint8_t {
  kUnconstrained,
  kRequirePassageTraversal,
};

[[nodiscard]] RiskAwareLattice3DResult searchRiskAwareLattice3DStage(
    const mppi::EsdfGrid& grid, std::span<const float> esdf_m, const Point3& start,
    const Vec3& preferred_direction, const Point3& planning_goal,
    const Point3& mission_goal,
    std::span<const PassageTraversalEdge> passage_traversals, Lattice3DRiskStage stage,
    Lattice3DTopologyRequirement topology_requirement,
    const RiskAwareLattice3DConfig& config, BoundedWorkerPool* worker_pool);

} // namespace drone_city_nav::detail
