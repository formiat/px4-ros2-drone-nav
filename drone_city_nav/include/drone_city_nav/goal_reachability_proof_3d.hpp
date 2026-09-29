#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

// The proof of roadmap item 19: whether the goal is provably unreachable on
// the obstacle memory as it stands. A flood from the vehicle through every
// voxel that is not occupied, unknown included, because unknown is
// traversable at no penalty, within the space the planner flies in: the grid,
// whose edge its lattice does not cross, and the voxels the flight envelope's
// band of heights touches (roadmap item 17 stage 8). Beyond them no route
// exists for the vehicle, measured or not, so they bound the flood as walls
// do. The goal, at the height the planner flies to it (clamped into the
// band), is unreachable when the flood does not reach it. The flood is a
// check of the map and never a reading of the planner: the planner finds no
// route for reasons of its own (r596 stood 106 s without a route in an open
// world). It stops as soon as it reaches the goal, and it stops undecided when
// it has visited more voxels than its budget, so that an open world costs the
// caller a bounded time. A vehicle outside the band proves nothing.
struct GoalReachabilityProof3D {
  bool vehicle_inside_grid{false};
  bool goal_inside{false};
  bool budget_exhausted{false};
  std::size_t component_voxels{0U};

  [[nodiscard]] bool provenUnreachable() const noexcept {
    return vehicle_inside_grid && !goal_inside && !budget_exhausted;
  }
};

[[nodiscard]] GoalReachabilityProof3D
proveGoalUnreachable3D(const ObservedOccupancyGrid3D& grid, const Point3& vehicle,
                       const Point3& goal, const FlightEnvelopeConfig& envelope,
                       std::size_t voxel_budget);

[[nodiscard]] const char*
goalReachabilityProofVerdict(const GoalReachabilityProof3D& proof) noexcept;

} // namespace drone_city_nav
