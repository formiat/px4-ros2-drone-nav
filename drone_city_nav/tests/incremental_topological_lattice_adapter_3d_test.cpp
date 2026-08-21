#include "drone_city_nav/incremental_topological_lattice_adapter_3d.hpp"

#include <gtest/gtest.h>

#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

IncrementalTopologicalPlan3D
executablePlan(const IncrementalTopologicalRoutePurpose3D purpose,
               std::vector<Point3> guidance_points) {
  IncrementalTopologicalPlan3D result;
  result.status = purpose == IncrementalTopologicalRoutePurpose3D::kMissionTransit
                      ? IncrementalTopologicalPlanStatus3D::kMissionRoute
                      : IncrementalTopologicalPlanStatus3D::kBacktrackRoute;
  result.purpose = purpose;
  result.guidance_points = std::move(guidance_points);
  result.selection_score = 12.5;
  result.reaches_mission_goal =
      purpose == IncrementalTopologicalRoutePurpose3D::kMissionTransit;
  return result;
}

TEST(IncrementalTopologicalLatticeAdapter3DTest,
     SelectsLookaheadAlongBentPolylineInsteadOfStraightChord) {
  const IncrementalTopologicalPlan3D plan =
      executablePlan(IncrementalTopologicalRoutePurpose3D::kMissionTransit,
                     {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}});

  const auto directive_result = makeIncrementalTopologicalLatticeDirective3D(
      plan, {0.0, 0.0, 0.0},
      {.maximum_lookahead_m = 15.0, .minimum_target_displacement_m = 0.1});

  ASSERT_TRUE(directive_result.has_value());
  const IncrementalTopologicalLatticeDirective3D directive =
      directive_result.value_or(IncrementalTopologicalLatticeDirective3D{});
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.x, 10.0);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.y, 5.0);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.z, 0.0);
  EXPECT_DOUBLE_EQ(directive.lattice.preferred_direction.x, 10.0);
  EXPECT_DOUBLE_EQ(directive.lattice.preferred_direction.y, 0.0);
  EXPECT_FALSE(directive.reaches_topological_target);
  EXPECT_FALSE(directive.lattice.reaches_mission_goal);
}

TEST(IncrementalTopologicalLatticeAdapter3DTest,
     ProjectsCurrentPositionAndContinuesForwardAlongRoute) {
  const IncrementalTopologicalPlan3D plan =
      executablePlan(IncrementalTopologicalRoutePurpose3D::kMissionTransit,
                     {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}});

  const auto directive_result = makeIncrementalTopologicalLatticeDirective3D(
      plan, {6.0, 1.0, 0.0},
      {.maximum_lookahead_m = 8.0, .minimum_target_displacement_m = 0.1});

  ASSERT_TRUE(directive_result.has_value());
  const IncrementalTopologicalLatticeDirective3D directive =
      directive_result.value_or(IncrementalTopologicalLatticeDirective3D{});
  EXPECT_DOUBLE_EQ(directive.source_station_m, 6.0);
  EXPECT_DOUBLE_EQ(directive.projection_distance_m, 1.0);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.x, 10.0);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.y, 4.0);
  EXPECT_DOUBLE_EQ(directive.target_station_m, 14.0);
}

TEST(IncrementalTopologicalLatticeAdapter3DTest,
     PreservesBacktrackPurposeAndAllowsMovementAwayFromMissionGoal) {
  const IncrementalTopologicalPlan3D plan =
      executablePlan(IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack,
                     {{10.0, 0.0, 4.0}, {2.0, 0.0, 4.0}});

  const auto directive_result = makeIncrementalTopologicalLatticeDirective3D(
      plan, {10.0, 0.0, 4.0},
      {.maximum_lookahead_m = 30.0, .minimum_target_displacement_m = 0.1});

  ASSERT_TRUE(directive_result.has_value());
  const IncrementalTopologicalLatticeDirective3D directive =
      directive_result.value_or(IncrementalTopologicalLatticeDirective3D{});
  EXPECT_EQ(directive.lattice.route_purpose,
            Lattice3DRoutePurpose::kTopologicalBacktrack);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.x, 2.0);
  EXPECT_TRUE(directive.reaches_topological_target);
  EXPECT_FALSE(directive.lattice.reaches_mission_goal);
}

TEST(IncrementalTopologicalLatticeAdapter3DTest,
     RejectsNonExecutableAndDegeneratePlans) {
  IncrementalTopologicalPlan3D unavailable;
  EXPECT_FALSE(
      makeIncrementalTopologicalLatticeDirective3D(unavailable, {0.0, 0.0, 0.0})
          .has_value());

  IncrementalTopologicalPlan3D degenerate =
      executablePlan(IncrementalTopologicalRoutePurpose3D::kMissionTransit,
                     {{1.0, 2.0, 3.0}, {1.0, 2.0, 3.0}});
  EXPECT_FALSE(makeIncrementalTopologicalLatticeDirective3D(degenerate, {1.0, 2.0, 3.0})
                   .has_value());
}

} // namespace
} // namespace drone_city_nav
