#pragma once

#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav::detail {

struct Lattice3DStageSelection {
  std::size_t selected_index{0U};
  std::vector<Lattice3DTopologyCandidate> diagnostics;
};

[[nodiscard]] Lattice3DStageSelection
selectLattice3DStageResult(std::span<const RiskAwareLattice3DResult> stage_results,
                           std::size_t passage_count);

} // namespace drone_city_nav::detail
