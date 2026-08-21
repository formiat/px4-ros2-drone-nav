#pragma once

#include "drone_city_nav/incremental_topological_planner_3d.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <cstddef>
#include <optional>

namespace drone_city_nav {

struct IncrementalTopologicalLatticeAdapter3DConfig {
  double maximum_lookahead_m{30.0};
  double minimum_target_displacement_m{0.25};
};

struct IncrementalTopologicalLatticeDirective3D {
  Lattice3DStrategicDirective lattice{};
  std::size_t source_segment_index{0U};
  double source_station_m{0.0};
  double target_station_m{0.0};
  double projection_distance_m{0.0};
  bool reaches_topological_target{false};
};

[[nodiscard]] bool incrementalTopologicalLatticeAdapter3DConfigIsValid(
    const IncrementalTopologicalLatticeAdapter3DConfig& config) noexcept;

[[nodiscard]] std::optional<IncrementalTopologicalLatticeDirective3D>
makeIncrementalTopologicalLatticeDirective3D(
    const IncrementalTopologicalPlan3D& plan, const Point3& position,
    const IncrementalTopologicalLatticeAdapter3DConfig& config = {});

} // namespace drone_city_nav
