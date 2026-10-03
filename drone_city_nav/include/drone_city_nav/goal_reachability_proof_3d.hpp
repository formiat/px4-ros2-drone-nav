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
//
// A second flood starts at the goal (roadmap item 17 stage 8): through the
// voxels the body fits in, none of whose 26 neighbours is occupied, from every
// such voxel within the capture radius of the goal. A goal in a dark the
// carried light does not reach is surrounded by the unobservable
// evidence of the frames that looked at it (specification K14), a sieve of
// points a ray apart that a single voxel passes and the body does not; the
// goal is unreachable when this flood closes without coming within a metre of
// the vehicle, or when no voxel near the goal fits the body at all. It never
// closes on an opening the body passes: one free voxel on each side of the
// body's axis is less than the body's 0.55 m radius. The same budget bounds it.
struct GoalReachabilityProof3D {
  bool vehicle_inside_grid{false};
  bool goal_inside{false};
  bool budget_exhausted{false};
  bool goal_region_closed{false};
  std::size_t component_voxels{0U};
  std::size_t goal_region_voxels{0U};

  [[nodiscard]] bool provenUnreachable() const noexcept {
    return vehicle_inside_grid &&
           ((!goal_inside && !budget_exhausted) || goal_region_closed);
  }
};

[[nodiscard]] GoalReachabilityProof3D
proveGoalUnreachable3D(const ObservedOccupancyGrid3D& grid, const Point3& vehicle,
                       const Point3& goal, const FlightEnvelopeConfig& envelope,
                       double capture_radius_m, std::size_t voxel_budget);

[[nodiscard]] const char*
goalReachabilityProofVerdict(const GoalReachabilityProof3D& proof) noexcept;

// Roadmap item 19: a goal is given up for the start only with a position
// source, the autopilot's position valid and under a second old; without one
// nothing is substituted (GOAL_UNREACHABLE_HELD).
[[nodiscard]] bool returnHomePositionSourceFresh(bool position_valid,
                                                 std::int64_t position_stamp_ns,
                                                 std::int64_t now_ns) noexcept;

} // namespace drone_city_nav
