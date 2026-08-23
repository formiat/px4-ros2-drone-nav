#include "drone_city_nav/incremental_topological_navigation_3d.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] IncrementalTopologyGraph3DConfig graphConfig() {
  IncrementalTopologyGraph3DConfig config;
  config.block_size_cells = 4;
  config.coarse_sample_stride_cells = 1;
  config.refined_sample_stride_cells = 1;
  config.maximum_observed_blocks_per_update = 4096U;
  config.footprint = SweptFootprintConfig{.radius_m = 0.1,
                                          .lower_extent_m = 0.1,
                                          .upper_extent_m = 0.1,
                                          .perimeter_samples = 4,
                                          .radial_rings = 1,
                                          .axial_samples = 2,
                                          .sweep_step_m = 0.25};
  return config;
}

[[nodiscard]] SensorObservabilityConfig
observabilityFor(const IncrementalTopologyGraph3DConfig& graph_config) {
  SensorObservabilityConfig config;
  config.footprint = graph_config.footprint;
  config.minimum_supporting_rays = 1U;
  config.minimum_information_gain_voxels = 1U;
  config.minimum_known_free_ray_m = 0.0;
  return config;
}

void fillFree(ObservedOccupancyGrid3D& occupancy) {
  const GridBounds3D& bounds = occupancy.bounds();
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        ASSERT_TRUE(occupancy.setState({x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
}

void expectSamePolyline(const std::vector<Point3>& expected,
                        const std::vector<Point3>& actual) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t index = 0U; index < expected.size(); ++index) {
    EXPECT_NEAR(actual[index].x, expected[index].x, 1.0e-9);
    EXPECT_NEAR(actual[index].y, expected[index].y, 1.0e-9);
    EXPECT_NEAR(actual[index].z, expected[index].z, 1.0e-9);
  }
}

class IncrementalTopologicalNavigation3DLifecycleTest : public ::testing::Test {
protected:
  IncrementalTopologicalNavigation3DLifecycleTest()
      : graph_config_{graphConfig()},
        navigation_{graph_config_,
                    {},
                    {},
                    observabilityFor(graph_config_),
                    IncrementalTopologicalNavigation3DConfig{
                        .active_route_completion_tolerance_m = 0.5}} {
  }

  void SetUp() override {
    fillFree(occupancy_);
    world_ = navigation_.updateObserved(occupancy_, 17U, 1U, {}, true);
  }

  void advanceWorldRevision() {
    world_ = navigation_.updateObserved(occupancy_, 17U, world_.graph.revision + 1U, {},
                                        true);
  }

  [[nodiscard]] IncrementalTopologicalPlan3D makeAcceptedMissionPlan() {
    IncrementalTopologicalPlan3D plan =
        navigation_.planObserved(world_.snapshot, occupancy_, start_, goal_);
    EXPECT_TRUE(plan.executableTargetSelected());
    EXPECT_GE(plan.guidance_points.size(), 2U);
    EXPECT_TRUE(navigation_.commitAcceptedPlan(plan).accepted);
    return plan;
  }

  [[nodiscard]] const Point3& start() const noexcept {
    return start_;
  }

  [[nodiscard]] const Point3& goal() const noexcept {
    return goal_;
  }

  [[nodiscard]] ObservedOccupancyGrid3D& occupancy() noexcept {
    return occupancy_;
  }

  [[nodiscard]] IncrementalTopologicalNavigation3D& navigation() noexcept {
    return navigation_;
  }

  [[nodiscard]] const IncrementalTopologicalWorldUpdate3D& world() const noexcept {
    return world_;
  }

private:
  const Point3 start_{3.5, 9.5, 5.5};
  const Point3 goal_{34.5, 9.5, 5.5};
  IncrementalTopologyGraph3DConfig graph_config_;
  ObservedOccupancyGrid3D occupancy_{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 20, 12}};
  IncrementalTopologicalNavigation3D navigation_;
  IncrementalTopologicalWorldUpdate3D world_;
};

TEST_F(IncrementalTopologicalNavigation3DLifecycleTest,
       ContinuesAcceptedPolylineAcrossNewerWorldRevision) {
  const IncrementalTopologicalPlan3D accepted = makeAcceptedMissionPlan();
  advanceWorldRevision();

  const IncrementalTopologicalPlan3D continued =
      navigation().planObserved(world().snapshot, occupancy(), {8.5, 9.5, 5.5}, goal());

  EXPECT_TRUE(continued.continued_from_active_plan);
  EXPECT_EQ(continued.planned_on_revision, accepted.planned_on_revision);
  EXPECT_LT(continued.planned_on_revision, world().graph.revision);
  expectSamePolyline(accepted.guidance_points, continued.guidance_points);
}

TEST_F(IncrementalTopologicalNavigation3DLifecycleTest,
       MissionTargetChangeReplacesAcceptedPolyline) {
  const IncrementalTopologicalPlan3D accepted = makeAcceptedMissionPlan();
  advanceWorldRevision();
  const Point3 replacement_goal{34.5, 4.5, 5.5};

  const IncrementalTopologicalPlan3D replacement = navigation().planObserved(
      world().snapshot, occupancy(), {8.5, 9.5, 5.5}, replacement_goal);

  EXPECT_FALSE(replacement.continued_from_active_plan);
  EXPECT_EQ(replacement.planned_on_revision, world().graph.revision);
  EXPECT_GT(distance3D(replacement.mission_target, accepted.mission_target), 1.0);
}

TEST_F(IncrementalTopologicalNavigation3DLifecycleTest,
       ReachingAcceptedPolylineEndpointStartsAFreshPlan) {
  const IncrementalTopologicalPlan3D accepted = makeAcceptedMissionPlan();
  advanceWorldRevision();

  const IncrementalTopologicalPlan3D replacement = navigation().planObserved(
      world().snapshot, occupancy(), accepted.guidance_points.back(), goal());

  EXPECT_FALSE(replacement.continued_from_active_plan);
  EXPECT_EQ(replacement.planned_on_revision, world().graph.revision);
}

TEST_F(IncrementalTopologicalNavigation3DLifecycleTest,
       RawCollisionInvalidationDropsOnlyTheAcceptedPlan) {
  const IncrementalTopologicalPlan3D accepted = makeAcceptedMissionPlan();
  advanceWorldRevision();
  const Point3 advanced_start{8.5, 9.5, 5.5};
  IncrementalTopologicalPlan3D continued =
      navigation().planObserved(world().snapshot, occupancy(), advanced_start, goal());
  ASSERT_TRUE(continued.continued_from_active_plan);
  IncrementalTopologicalPlan3D unrelated = continued;
  unrelated.target_node.value += 1U;

  EXPECT_FALSE(navigation().invalidateAcceptedPlan(unrelated));
  EXPECT_TRUE(navigation().invalidateAcceptedPlan(continued));
  const IncrementalTopologicalPlan3D replacement =
      navigation().planObserved(world().snapshot, occupancy(), advanced_start, goal());
  EXPECT_FALSE(replacement.continued_from_active_plan);
}

TEST_F(IncrementalTopologicalNavigation3DLifecycleTest,
       FrontierWithoutRemainingUnknownVolumeIsNotContinued) {
  IncrementalTopologicalPlan3D frontier_plan;
  frontier_plan.status = IncrementalTopologicalPlanStatus3D::kFrontierRoute;
  frontier_plan.purpose = IncrementalTopologicalRoutePurpose3D::kObservationFrontier;
  frontier_plan.planned_on_revision = world().graph.revision;
  frontier_plan.validated_through_revision = world().graph.revision;
  frontier_plan.mission_target = goal();
  frontier_plan.target_node = IncrementalTopologyNodeId{42U};
  frontier_plan.guidance_points = {start(), {20.5, 9.5, 5.5}};
  frontier_plan.selected_frontier = ObservationFrontier{
      .id = ObservationFrontierId{29U},
      .observation_pose = frontier_plan.guidance_points.back(),
  };
  ASSERT_TRUE(navigation().commitAcceptedPlan(frontier_plan).accepted);
  advanceWorldRevision();

  const IncrementalTopologicalPlan3D replacement =
      navigation().planObserved(world().snapshot, occupancy(), {8.5, 9.5, 5.5}, goal());

  EXPECT_FALSE(replacement.continued_from_active_plan);
  EXPECT_EQ(replacement.planned_on_revision, world().graph.revision);
}

TEST(IncrementalTopologicalNavigation3DConfigTest,
     RejectsNonFiniteCompletionTolerance) {
  IncrementalTopologicalNavigation3DConfig config;
  config.active_route_completion_tolerance_m = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(incrementalTopologicalNavigation3DConfigIsValid(config));
  EXPECT_THROW((IncrementalTopologicalNavigation3D{{}, {}, {}, {}, config}),
               std::invalid_argument);
}

} // namespace
} // namespace drone_city_nav
