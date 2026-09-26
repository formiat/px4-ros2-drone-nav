#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

// The proof of roadmap item 19: whether the goal is provably unreachable on
// the obstacle memory as it stands. A flood from the vehicle through every
// voxel that is not occupied, unknown included, because unknown is
// traversable at no penalty; the goal is unreachable only when the flood
// neither reaches the goal nor the edge of the grid, which is no measurement
// and therefore counts as unknown beyond it. The flood is a check of the map
// and never a reading of the planner: the planner finds no route for reasons
// of its own (r596 stood 106 s without a route in an open world). It stops as
// soon as it reaches the goal or the edge, and it stops undecided when it has
// visited more voxels than its budget, so that an open world costs the caller
// a bounded time.
struct GoalReachabilityProof3D {
  bool vehicle_inside_grid{false};
  bool goal_inside{false};
  bool touches_grid_edge{false};
  bool budget_exhausted{false};
  std::size_t component_voxels{0U};

  [[nodiscard]] bool provenUnreachable() const noexcept {
    return vehicle_inside_grid && !goal_inside && !touches_grid_edge &&
           !budget_exhausted;
  }
};

[[nodiscard]] GoalReachabilityProof3D
proveGoalUnreachable3D(const ObservedOccupancyGrid3D& grid, const Point3& vehicle,
                       const Point3& goal, std::size_t voxel_budget);

[[nodiscard]] const char*
goalReachabilityProofVerdict(const GoalReachabilityProof3D& proof) noexcept;

} // namespace drone_city_nav
