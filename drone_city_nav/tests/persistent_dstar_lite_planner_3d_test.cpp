#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] PersistentPlannerConfig3D testConfig() {
  PersistentPlannerConfig3D config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.time_model.maximum_horizontal_speed_mps = 5.0;
  config.time_model.maximum_vertical_speed_mps = 2.0;
  config.goal_tolerance_m = 0.01;
  config.maximum_compute_time_ms = 1000.0;
  config.maximum_expansions_per_update = 100000U;
  config.physical_footprint.radius_m = 0.0;
  config.physical_footprint.lower_extent_m = 0.0;
  config.physical_footprint.upper_extent_m = 0.0;
  config.physical_footprint.perimeter_samples = 0U;
  config.physical_footprint.radial_rings = 0U;
  config.physical_footprint.axial_samples = 0U;
  config.physical_footprint.sweep_step_m = 0.2;
  config.flight_envelope.minimum_target_z_m = 0.0;
  config.flight_envelope.maximum_target_z_m = 20.0;
  return config;
}

[[nodiscard]] PersistentPlannerWorld3D
world(std::shared_ptr<const ObservedOccupancyGrid3D> occupancy,
      const std::uint64_t revision,
      std::vector<OccupancyChunkIndex3D> dirty_chunks = {},
      const bool full_reset = false) {
  const OccupancyGrid3D occupied = occupancy->occupiedSnapshot();
  return PersistentPlannerWorld3D{
      .observed_occupancy = std::move(occupancy),
      .static_occupancy = nullptr,
      .proprioceptive_free_space_seed = std::nullopt,
      .launch_support_contact = std::nullopt,
      .dirty_chunks = std::move(dirty_chunks),
      .producer_instance_id = 17U,
      .revision = revision,
      .occupied_fingerprint = occupied.contentFingerprint(),
      .full_reset = full_reset,
  };
}

[[nodiscard]] PersistentPlannerRequest3D
request(const Point3& start, const Point3& goal, PersistentPlannerWorld3D raw_world,
        const std::uint64_t mission_epoch = 3U) {
  return PersistentPlannerRequest3D{
      .start = start,
      .velocity = {},
      .mission_goal = goal,
      .mission_epoch = mission_epoch,
      .world = std::move(raw_world),
  };
}

void expectSamePath(const std::vector<Point3>& first,
                    const std::vector<Point3>& second) {
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0U; index < first.size(); ++index) {
    EXPECT_NEAR(first[index].x, second[index].x, 1.0e-12);
    EXPECT_NEAR(first[index].y, second[index].y, 1.0e-12);
    EXPECT_NEAR(first[index].z, second[index].z, 1.0e-12);
  }
}

void expectRawValid(const std::vector<Point3>& path,
                    const ObservedOccupancyGrid3D& occupancy,
                    const SweptFootprintConfig& footprint) {
  ASSERT_GE(path.size(), 2U);
  for (std::size_t index = 1U; index < path.size(); ++index) {
    EXPECT_TRUE(
        validateObservedSweptFootprint(occupancy, path[index - 1U], FootprintBodyAxis{},
                                       path[index], FootprintBodyAxis{}, footprint,
                                       ObservedSpaceValidationPolicy::kAllowUnknown)
            .accepted());
  }
}

TEST(PersistentDStarLitePlanner3DTest,
     SolvesOneWorldFixedTwentySixConnectedMissionToTheExactGoal) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 12, 8});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{10.5, 10.5, 5.5};

  const PersistentPlannerResult3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(result.executable());
  ASSERT_EQ(result.status, PersistentPlannerStatus3D::kReachedMissionGoal);
  EXPECT_TRUE(result.search_complete);
  EXPECT_FALSE(result.search_state_reused);
  EXPECT_EQ(result.search_generation, 1U);
  EXPECT_DOUBLE_EQ(result.points.front().x, start.x);
  EXPECT_DOUBLE_EQ(result.points.front().y, start.y);
  EXPECT_DOUBLE_EQ(result.points.front().z, start.z);
  EXPECT_DOUBLE_EQ(result.points.back().x, goal.x);
  EXPECT_DOUBLE_EQ(result.points.back().y, goal.y);
  EXPECT_DOUBLE_EQ(result.points.back().z, goal.z);
  EXPECT_GT(result.estimated_translation_time_s, 0.0);
  EXPECT_GE(result.estimated_execution_time_s, result.estimated_translation_time_s);
  expectRawValid(result.points, *occupancy, planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     FreeUnknownRelabelPreservesPathCostAndIncrementalSearchState) {
  auto first = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 8, 6});
  static_cast<void>(first->setState(GridIndex3D{3, 3, 2}, ObservedVoxelState::kFree));
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 3.5, 2.5};
  const Point3 goal{10.5, 3.5, 2.5};
  const PersistentPlannerResult3D initial =
      planner.plan(request(start, goal, world(first, 1U)));
  ASSERT_TRUE(initial.executable());

  auto relabeled = std::make_shared<ObservedOccupancyGrid3D>(*first);
  static_cast<void>(
      relabeled->setState(GridIndex3D{3, 3, 2}, ObservedVoxelState::kUnknown));
  static_cast<void>(
      relabeled->setState(GridIndex3D{8, 4, 2}, ObservedVoxelState::kFree));
  const PersistentPlannerResult3D updated = planner.plan(
      request(start, goal,
              world(relabeled, 2U,
                    {ObservedOccupancyGrid3D::chunkIndex(GridIndex3D{3, 3, 2})})));

  ASSERT_TRUE(updated.executable());
  EXPECT_TRUE(updated.search_state_reused);
  EXPECT_TRUE(updated.occupied_world_unchanged);
  EXPECT_EQ(updated.changed_occupied_voxels, 0U);
  EXPECT_EQ(updated.search_generation, initial.search_generation);
  EXPECT_DOUBLE_EQ(updated.path_length_m, initial.path_length_m);
  EXPECT_DOUBLE_EQ(updated.estimated_translation_time_s,
                   initial.estimated_translation_time_s);
  expectSamePath(updated.points, initial.points);
}

TEST(PersistentDStarLitePlanner3DTest,
     RepairsTheResidentSearchAfterAnOccupiedCellAppearsOnItsPath) {
  auto initial_occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 10, 6});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 5.5, 2.5};
  const Point3 goal{12.5, 5.5, 2.5};
  const PersistentPlannerResult3D initial =
      planner.plan(request(start, goal, world(initial_occupancy, 1U)));
  ASSERT_TRUE(initial.executable());

  auto changed = std::make_shared<ObservedOccupancyGrid3D>(*initial_occupancy);
  const GridIndex3D obstacle{7, 5, 2};
  ASSERT_TRUE(changed->setState(obstacle, ObservedVoxelState::kOccupied));
  const PersistentPlannerResult3D repaired = planner.plan(
      request(start, goal,
              world(changed, 2U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));

  ASSERT_TRUE(repaired.executable());
  EXPECT_TRUE(repaired.search_state_reused);
  EXPECT_EQ(repaired.changed_occupied_voxels, 1U);
  EXPECT_GT(repaired.affected_lattice_states, 0U);
  EXPECT_EQ(repaired.search_generation, initial.search_generation);
  EXPECT_EQ(repaired.repair_generation, 1U);
  EXPECT_GT(repaired.path_length_m, initial.path_length_m);
  expectRawValid(repaired.points, *changed, planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     PreparatoryVerticalMotionPassesThroughTheOnlyLowerOpening) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 10, 8});
  for (int z = 3; z < 8; ++z) {
    for (int y = 0; y < 10; ++y) {
      static_cast<void>(
          occupancy->setState(GridIndex3D{6, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{2.5, 5.5, 5.5};
  const Point3 goal{11.5, 5.5, 5.5};

  const PersistentPlannerResult3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(result.executable());
  EXPECT_LT(std::ranges::min_element(result.points, {}, &Point3::z)->z, start.z);
  expectRawValid(result.points, *occupancy, planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     MovingStartReusesTheSameBackwardSearchAndReturnsACurrentConnector) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 16, 8, 6});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 first_start{1.5, 3.5, 2.5};
  const Point3 moved_start{5.5, 3.5, 2.5};
  const Point3 goal{14.5, 3.5, 2.5};
  const PersistentPlannerResult3D initial =
      planner.plan(request(first_start, goal, world(occupancy, 1U)));
  ASSERT_TRUE(initial.executable());

  const PersistentPlannerResult3D moved =
      planner.plan(request(moved_start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(moved.executable());
  EXPECT_TRUE(moved.search_state_reused);
  EXPECT_EQ(moved.search_generation, initial.search_generation);
  EXPECT_DOUBLE_EQ(moved.points.front().x, moved_start.x);
  EXPECT_DOUBLE_EQ(moved.points.front().y, moved_start.y);
  EXPECT_DOUBLE_EQ(moved.points.front().z, moved_start.z);
  EXPECT_LT(moved.path_length_m, initial.path_length_m);
}

TEST(PersistentDStarLitePlanner3DTest,
     BoundedSearchResumesInsteadOfPublishingAGreedyFrontier) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 18, 18, 8});
  PersistentPlannerConfig3D config = testConfig();
  config.maximum_expansions_per_update = 4U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{16.5, 16.5, 6.5};

  PersistentPlannerResult3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));
  ASSERT_EQ(result.status, PersistentPlannerStatus3D::kSearchInProgress);
  EXPECT_TRUE(result.points.empty());
  const std::uint64_t generation = result.search_generation;
  for (std::size_t attempt = 0U; attempt < 200U && !result.executable(); ++attempt) {
    result = planner.plan(request(start, goal, world(occupancy, 1U)));
  }

  ASSERT_TRUE(result.executable());
  EXPECT_TRUE(result.search_state_reused);
  EXPECT_EQ(result.search_generation, generation);
}

} // namespace
} // namespace drone_city_nav
