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
// The whole grid's heights.
constexpr FlightEnvelopeConfig kEnvelope{.minimum_target_z_m = 0.0,
                                         .maximum_target_z_m = 40.0};

TEST(GoalReachabilityProof3D, AClosedBoxProvesTheGoalOutsideItUnreachable) {
  const GoalReachabilityProof3D proof = proveGoalUnreachable3D(
      closedBox(), kVehicle, kGoalOutside, kEnvelope, 1'000'000U);
  EXPECT_TRUE(proof.vehicle_inside_grid);
  EXPECT_FALSE(proof.goal_inside);
  EXPECT_FALSE(proof.budget_exhausted);
  EXPECT_EQ(15U * 15U * 15U, proof.component_voxels);
  EXPECT_TRUE(proof.provenUnreachable());
  EXPECT_STREQ("proven_unreachable", goalReachabilityProofVerdict(proof));
}

TEST(GoalReachabilityProof3D, OneOpenVoxelInTheShellLeavesTheProofOpen) {
  ObservedOccupancyGrid3D grid = closedBox();
  static_cast<void>(grid.setState(GridIndex3D{24, 16, 16}, ObservedVoxelState::kFree));
  // The goal just outside the hole: the flood reaches it through the hole.
  const GoalReachabilityProof3D proof = proveGoalUnreachable3D(
      grid, kVehicle, Point3{26.5, 16.5, 16.5}, kEnvelope, 1'000'000U);
  EXPECT_TRUE(proof.goal_inside);
  EXPECT_FALSE(proof.provenUnreachable());
  EXPECT_STREQ("goal_reachable", goalReachabilityProofVerdict(proof));
}

TEST(GoalReachabilityProof3D, TheGridEdgeBoundsTheFloodAsAWallDoes) {
  // No shell at all: the unknown grid is one component, and a goal outside
  // it, where the planner's lattice does not reach, is not in it.
  const GoalReachabilityProof3D proof =
      proveGoalUnreachable3D(ObservedOccupancyGrid3D{kBounds}, kVehicle,
                             Point3{80.0, 16.5, 16.5}, kEnvelope, 1'000'000U);
  EXPECT_EQ(40U * 40U * 40U, proof.component_voxels);
  EXPECT_TRUE(proof.provenUnreachable());
}

TEST(GoalReachabilityProof3D, TheEnvelopeBandBoundsTheFloodAsAWallDoes) {
  // The shell open at its roof, the band ending under the roof's layer: the
  // space above it is not flown, so it is no opening.
  ObservedOccupancyGrid3D grid = closedBox();
  for (int x = 9; x <= 23; ++x) {
    for (int y = 9; y <= 23; ++y) {
      static_cast<void>(
          grid.setState(GridIndex3D{x, y, 24}, ObservedVoxelState::kFree));
    }
  }
  const FlightEnvelopeConfig band{.minimum_target_z_m = 9.0,
                                  .maximum_target_z_m = 23.5};
  const GoalReachabilityProof3D closed =
      proveGoalUnreachable3D(grid, kVehicle, kGoalOutside, band, 1'000'000U);
  EXPECT_EQ(15U * 15U * 15U, closed.component_voxels);
  EXPECT_TRUE(closed.provenUnreachable());
  const GoalReachabilityProof3D open =
      proveGoalUnreachable3D(grid, kVehicle, kGoalOutside, kEnvelope, 1'000'000U);
  EXPECT_TRUE(open.goal_inside);
}

TEST(GoalReachabilityProof3D, TheGoalIsFlownToAtItsHeightClampedIntoTheBand) {
  // A goal above the band over the open roof: the planner flies to it at the
  // band's ceiling, inside the box's opening, which the flood reaches.
  ObservedOccupancyGrid3D grid = closedBox();
  static_cast<void>(grid.setState(GridIndex3D{16, 16, 24}, ObservedVoxelState::kFree));
  const FlightEnvelopeConfig band{.minimum_target_z_m = 9.0,
                                  .maximum_target_z_m = 24.5};
  const GoalReachabilityProof3D proof = proveGoalUnreachable3D(
      grid, kVehicle, Point3{16.5, 16.5, 36.5}, band, 1'000'000U);
  EXPECT_TRUE(proof.goal_inside);
}

TEST(GoalReachabilityProof3D, AStartShutOffWithTheGoalIsProvenUnreachableAsWell) {
  // After a return, the proof runs to the start: a vehicle shut in where
  // neither is reached holds (the monitor's START_UNREACHABLE).
  const GoalReachabilityProof3D proof = proveGoalUnreachable3D(
      closedBox(), kVehicle, Point3{2.5, 2.5, 2.5}, kEnvelope, 1'000'000U);
  EXPECT_FALSE(proof.goal_inside);
  EXPECT_TRUE(proof.provenUnreachable());
}

TEST(GoalReachabilityProof3D, AGoalInsideTheBoxIsReachable) {
  const GoalReachabilityProof3D proof = proveGoalUnreachable3D(
      closedBox(), kVehicle, Point3{20.5, 20.5, 20.5}, kEnvelope, 1'000'000U);
  EXPECT_TRUE(proof.goal_inside);
  EXPECT_FALSE(proof.provenUnreachable());
}

TEST(GoalReachabilityProof3D, TheBudgetEndsTheFloodUndecided) {
  const GoalReachabilityProof3D proof =
      proveGoalUnreachable3D(closedBox(), kVehicle, kGoalOutside, kEnvelope, 100U);
  EXPECT_TRUE(proof.budget_exhausted);
  EXPECT_FALSE(proof.provenUnreachable());
  EXPECT_STREQ("budget_exhausted", goalReachabilityProofVerdict(proof));
}

TEST(GoalReachabilityProof3D, AVehicleOutsideTheGridProvesNothing) {
  const GoalReachabilityProof3D proof = proveGoalUnreachable3D(
      closedBox(), Point3{-5.0, 16.5, 16.5}, kGoalOutside, kEnvelope, 1'000'000U);
  EXPECT_FALSE(proof.vehicle_inside_grid);
  EXPECT_FALSE(proof.provenUnreachable());
  EXPECT_STREQ("vehicle_outside_grid", goalReachabilityProofVerdict(proof));
}

TEST(GoalReachabilityProof3D, AVehicleOutsideTheBandProvesNothing) {
  const FlightEnvelopeConfig band{.minimum_target_z_m = 18.0,
                                  .maximum_target_z_m = 30.0};
  const GoalReachabilityProof3D proof =
      proveGoalUnreachable3D(closedBox(), kVehicle, kGoalOutside, band, 1'000'000U);
  EXPECT_FALSE(proof.vehicle_inside_grid);
  EXPECT_FALSE(proof.provenUnreachable());
}

} // namespace
} // namespace drone_city_nav
