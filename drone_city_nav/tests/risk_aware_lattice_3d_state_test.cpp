#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(RiskAwareLattice3DStateTest,
     RetainsDistinctIncomingDirectionsAtTheSameSpatialCell) {
  const mppi::EsdfGrid grid{5, 5, 1.0F, 0.0F, 0.0F, 3, 0.0F};
  const std::size_t spatial_cell_count = static_cast<std::size_t>(grid.width) *
                                         static_cast<std::size_t>(grid.height) *
                                         static_cast<std::size_t>(grid.depth);
  std::vector<float> esdf(spatial_cell_count, 20.0F);
  const std::size_t blocked_goal =
      (std::size_t{1U} * static_cast<std::size_t>(grid.height) + 4U) *
          static_cast<std::size_t>(grid.width) +
      4U;
  esdf.at(blocked_goal) = 0.0F;

  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.sample_step_m = 0.25;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_lower_extent_m = 0.0;
  config.physical_footprint_upper_extent_m = 0.0;
  config.physical_footprint_samples = 0U;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.goal_tolerance_m = 1.0;
  config.maximum_expansions = 10000U;
  config.maximum_search_time_ms = 3000.0;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, esdf, Point3{2.5, 2.5, 1.5}, Vec3{1.0, 0.0, 0.0},
                             Point3{4.5, 4.5, 1.5}, {}, config);

  // With no passage nodes or topology-progress variants, exceeding the spatial
  // cell count proves that arrivals with different incoming motion primitives
  // coexist instead of overwriting one another.
  EXPECT_GT(result.records_peak, spatial_cell_count);
  EXPECT_GT(result.successor_diagnostics.lattice_rejected_no_cost_improvement, 0U);
}

} // namespace
} // namespace drone_city_nav
