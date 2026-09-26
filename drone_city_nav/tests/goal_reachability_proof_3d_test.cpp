#include "drone_city_nav/goal_reachability_proof_3d.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

constexpr GridBounds3D kBounds{
    .origin_x = 0.0,
    .origin_y = 0.0,
    .origin_z = 0.0,
    .resolution_m = 1.0,
    .width_cells = 40,
    .height_cells = 40,
    .depth_cells = 40,
};

// A hollow box of occupied voxels from 8 to 24 on every axis, one voxel
// thick, its inside unknown; the vehicle stands inside it.
ObservedOccupancyGrid3D closedBox() {
  ObservedOccupancyGrid3D grid{kBounds};
  for (int x = 8; x <= 24; ++x) {
    for (int y = 8; y <= 24; ++y) {
      for (int z = 8; z <= 24; ++z) {
        const bool shell = x == 8 || x == 24 || y == 8 || y == 24 || z == 8 || z == 24;
        if (shell) {
          static_cast<void>(
              grid.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kOccupied));
        }
      }
    }
  }
  return grid;
}

constexpr Point3 kVehicle{16.5, 16.5, 16.5};
constexpr Point3 kGoalOutside{36.5, 36.5, 36.5};

TEST(GoalReachabilityProof3D, AClosedBoxProvesTheGoalOutsideItUnreachable) {
  const GoalReachabilityProof3D proof =
      proveGoalUnreachable3D(closedBox(), kVehicle, kGoalOutside, 1'000'000U);
  EXPECT_TRUE(proof.vehicle_inside_grid);
  EXPECT_FALSE(proof.goal_inside);
  EXPECT_FALSE(proof.touches_grid_edge);
  EXPECT_FALSE(proof.budget_exhausted);
  EXPECT_EQ(15U * 15U * 15U, proof.component_voxels);
  EXPECT_TRUE(proof.provenUnreachable());
  EXPECT_STREQ("proven_unreachable", goalReachabilityProofVerdict(proof));
}

TEST(GoalReachabilityProof3D, OneOpenVoxelInTheShellLeavesTheProofOpen) {
  ObservedOccupancyGrid3D grid = closedBox();
  static_cast<void>(grid.setState(GridIndex3D{24, 16, 16}, ObservedVoxelState::kFree));
  // The goal just outside the hole: the flood reaches it through the hole.
  const GoalReachabilityProof3D proof =
      proveGoalUnreachable3D(grid, kVehicle, Point3{26.5, 16.5, 16.5}, 1'000'000U);
  EXPECT_TRUE(proof.goal_inside);
  EXPECT_FALSE(proof.provenUnreachable());
  EXPECT_STREQ("goal_reachable", goalReachabilityProofVerdict(proof));
}

TEST(GoalReachabilityProof3D, TheGridEdgeIsNoMeasurement) {
  ObservedOccupancyGrid3D grid = closedBox();
  static_cast<void>(grid.setState(GridIndex3D{24, 16, 16}, ObservedVoxelState::kFree));
  // The goal lies outside the grid altogether: the flood leaving the box
  // reaches the edge and the proof is open there.
  const GoalReachabilityProof3D proof =
      proveGoalUnreachable3D(grid, kVehicle, Point3{80.0, 80.0, 80.0}, 1'000'000U);
  EXPECT_TRUE(proof.touches_grid_edge);
  EXPECT_FALSE(proof.provenUnreachable());
  EXPECT_STREQ("open_at_grid_edge", goalReachabilityProofVerdict(proof));
}

TEST(GoalReachabilityProof3D, AGoalInsideTheBoxIsReachable) {
  const GoalReachabilityProof3D proof = proveGoalUnreachable3D(
      closedBox(), kVehicle, Point3{20.5, 20.5, 20.5}, 1'000'000U);
  EXPECT_TRUE(proof.goal_inside);
  EXPECT_FALSE(proof.provenUnreachable());
}

TEST(GoalReachabilityProof3D, TheBudgetEndsTheFloodUndecided) {
  const GoalReachabilityProof3D proof =
      proveGoalUnreachable3D(closedBox(), kVehicle, kGoalOutside, 100U);
  EXPECT_TRUE(proof.budget_exhausted);
  EXPECT_FALSE(proof.provenUnreachable());
  EXPECT_STREQ("budget_exhausted", goalReachabilityProofVerdict(proof));
}

TEST(GoalReachabilityProof3D, AVehicleOutsideTheGridProvesNothing) {
  const GoalReachabilityProof3D proof = proveGoalUnreachable3D(
      closedBox(), Point3{-5.0, 16.5, 16.5}, kGoalOutside, 1'000'000U);
  EXPECT_FALSE(proof.vehicle_inside_grid);
  EXPECT_FALSE(proof.provenUnreachable());
  EXPECT_STREQ("vehicle_outside_grid", goalReachabilityProofVerdict(proof));
}

} // namespace
} // namespace drone_city_nav
