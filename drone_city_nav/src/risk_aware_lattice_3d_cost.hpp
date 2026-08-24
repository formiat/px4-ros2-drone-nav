#pragma once

#include "risk_aware_lattice_3d_geometry.hpp"

namespace drone_city_nav::detail {

struct Lattice3DCostMetrics {
  double objective_cost{0.0};
  double route_length_m{0.0};
  double travel_time_s{0.0};
  double vertical_alignment_time_s{0.0};
  double planning_exposure_m{0.0};
  double critical_exposure_m{0.0};
  double turn_cost{0.0};
};

[[nodiscard]] Vec3 lattice3DUnitDirection(const Point3& first,
                                          const Point3& second) noexcept;

[[nodiscard]] Lattice3DCostMetrics evaluateLattice3DEdgeCost(
    const Point3& first, const Point3& second, const Vec3& incoming_direction,
    const Vec3& preferred_direction, const Lattice3DEdgeEvaluation& exposure,
    const RiskAwareLattice3DConfig& config, bool charge_shape_turn = true) noexcept;

void accumulateLattice3DCost(Lattice3DCostMetrics& target,
                             const Lattice3DCostMetrics& addition) noexcept;

[[nodiscard]] double
lattice3DTravelHeuristic(const Point3& point, const Point3& goal,
                         const RiskAwareLattice3DConfig& config) noexcept;

} // namespace drone_city_nav::detail
