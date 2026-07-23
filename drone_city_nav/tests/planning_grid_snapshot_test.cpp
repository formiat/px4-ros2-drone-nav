#include "drone_city_nav/planning_grid_snapshot.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] GridBounds testBounds() {
  return GridBounds{0.0, 0.0, 1.0, 8, 8};
}

[[nodiscard]] PlanningGridBuildResult readyBuild() {
  OccupancyGrid2D raw{testBounds()};
  raw.setOccupied(GridIndex{3, 3});
  OccupancyGrid2D runtime = raw;
  runtime.rebuildInflation(2.0);
  OccupancyGrid2D planning = raw;
  planning.rebuildInflation(3.0);

  PlanningGridBuildResult result{};
  result.status = PlanningGridStatus::kReady;
  result.raw_grid = std::move(raw);
  result.grid = std::move(runtime);
  result.planning_grid = std::move(planning);
  return result;
}

[[nodiscard]] bool sameFingerprint(const OccupancyGridFingerprint& lhs,
                                   const OccupancyGridFingerprint& rhs) {
  return lhs.bounds.origin_x == rhs.bounds.origin_x &&
         lhs.bounds.origin_y == rhs.bounds.origin_y &&
         lhs.bounds.resolution_m == rhs.bounds.resolution_m &&
         lhs.bounds.width_cells == rhs.bounds.width_cells &&
         lhs.bounds.height_cells == rhs.bounds.height_cells &&
         lhs.cells_hash == rhs.cells_hash && lhs.inflated_hash == rhs.inflated_hash;
}

} // namespace

TEST(PlanningGridSnapshot, FailedBuildDoesNotConsumeRevision) {
  PlanningGridSnapshotBuilder builder;
  PlanningGridBuildResult failed{};
  failed.status = PlanningGridStatus::kNoReadySourceData;

  EXPECT_FALSE(
      builder
          .prepare(PlanningGridPreparationInput{.build_result = &failed,
                                                .relaxation_center = Point2{4.5, 3.5},
                                                .relaxation_radius_m = 1.1,
                                                .clearance_max_distance_m = 10.0})
          .has_value());
  EXPECT_EQ(builder.nextRevision(), 1U);

  PlanningGridBuildResult ready = readyBuild();
  const auto prepared = builder.prepare(
      PlanningGridPreparationInput{.build_result = &ready,
                                   .relaxation_center = Point2{4.5, 3.5},
                                   .relaxation_radius_m = 1.1,
                                   .clearance_max_distance_m = 10.0});
  ASSERT_TRUE(prepared.has_value());
  EXPECT_EQ(prepared->version.build_revision, 1U);
  EXPECT_EQ(builder.nextRevision(), 2U);
}

TEST(PlanningGridSnapshot, RevisionAndSourceIdentityMatchSuccessfulBuilds) {
  PlanningGridSnapshotBuilder builder;
  PlanningGridBuildResult ready = readyBuild();
  const PlanningGridPreparationInput input{
      .build_result = &ready,
      .relaxation_center = Point2{4.5, 3.5},
      .relaxation_radius_m = 1.1,
      .clearance_max_distance_m = 10.0,
      .sources =
          PlanningGridSourceIdentity{
              .memory_producer_instance_id = 17U,
              .memory_sequence = 42U,
              .lidar_update_ns = 123456,
              .config_fingerprint = 99U,
          },
  };

  const auto first = builder.prepare(input);
  const auto second = builder.prepare(input);

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->version.build_revision, 1U);
  EXPECT_EQ(second->version.build_revision, 2U);
  EXPECT_EQ(first->version.memory_producer_instance_id, 17U);
  EXPECT_EQ(first->version.memory_sequence, 42U);
  EXPECT_EQ(first->version.lidar_update_ns, 123456);
  EXPECT_EQ(first->version.config_fingerprint, 99U);
}

TEST(PlanningGridSnapshot, FingerprintsAndClearanceDescribeRelaxedGrids) {
  PlanningGridSnapshotBuilder builder;
  PlanningGridBuildResult ready = readyBuild();
  ASSERT_TRUE(ready.grid.has_value());
  ASSERT_TRUE(ready.planning_grid.has_value());
  const OccupancyGridFingerprint runtime_before = ready.grid->prohibitedFingerprint();
  const OccupancyGridFingerprint planning_before =
      ready.planning_grid->prohibitedFingerprint();

  const auto prepared = builder.prepare(
      PlanningGridPreparationInput{.build_result = &ready,
                                   .relaxation_center = Point2{4.5, 3.5},
                                   .relaxation_radius_m = 1.1,
                                   .clearance_max_distance_m = 10.0});

  ASSERT_TRUE(prepared.has_value());
  EXPECT_TRUE(prepared->runtime_prohibited_grid.isOccupied(GridIndex{3, 3}));
  EXPECT_TRUE(prepared->planning_clearance_grid.isOccupied(GridIndex{3, 3}));
  EXPECT_FALSE(prepared->runtime_prohibited_grid.isInflated(GridIndex{4, 3}));
  EXPECT_FALSE(prepared->planning_clearance_grid.isInflated(GridIndex{4, 3}));
  EXPECT_GT(prepared->runtime_relaxation.inflated_cells_cleared, 0U);
  EXPECT_GT(prepared->planning_relaxation.inflated_cells_cleared, 0U);
  EXPECT_TRUE(
      sameFingerprint(prepared->version.runtime_prohibited,
                      prepared->runtime_prohibited_grid.prohibitedFingerprint()));
  EXPECT_TRUE(
      sameFingerprint(prepared->version.planning_clearance,
                      prepared->planning_clearance_grid.prohibitedFingerprint()));
  EXPECT_NE(prepared->version.runtime_prohibited.inflated_hash,
            runtime_before.inflated_hash);
  EXPECT_NE(prepared->version.planning_clearance.inflated_hash,
            planning_before.inflated_hash);
  EXPECT_DOUBLE_EQ(prepared->runtime_clearance.distanceAt(GridIndex{4, 3}), 1.0);
  EXPECT_DOUBLE_EQ(prepared->planning_clearance.distanceAt(GridIndex{4, 3}), 1.0);
}

} // namespace drone_city_nav
