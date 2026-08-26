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
     StopsAtTheNextStrategicBoundaryInsteadOfCrossingAJunction) {
  IncrementalTopologicalPlan3D plan =
      executablePlan(IncrementalTopologicalRoutePurpose3D::kMissionTransit,
                     {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}});
  plan.strategic_boundary_stations_m = {10.0, 20.0};

  const auto directive_result = makeIncrementalTopologicalLatticeDirective3D(
      plan, {0.0, 0.0, 0.0},
      {.maximum_lookahead_m = 15.0, .segment_capture_radius_m = 0.1});

  ASSERT_TRUE(directive_result.has_value());
  const IncrementalTopologicalLatticeDirective3D directive =
      directive_result.value_or(IncrementalTopologicalLatticeDirective3D{});
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.x, 10.0);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.y, 0.0);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.z, 0.0);
  EXPECT_DOUBLE_EQ(directive.lattice.preferred_direction.x, 10.0);
  EXPECT_DOUBLE_EQ(directive.lattice.preferred_direction.y, 0.0);
  EXPECT_FALSE(directive.reaches_topological_target);
  EXPECT_FALSE(directive.lattice.reaches_mission_goal);
}

TEST(IncrementalTopologicalLatticeAdapter3DTest,
     ProjectsCurrentPositionAndContinuesForwardAlongRoute) {
  IncrementalTopologicalPlan3D plan =
      executablePlan(IncrementalTopologicalRoutePurpose3D::kMissionTransit,
                     {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}});
  plan.strategic_boundary_stations_m = {10.0, 20.0};

  const auto directive_result = makeIncrementalTopologicalLatticeDirective3D(
      plan, {6.0, 1.0, 0.0},
      {.maximum_lookahead_m = 8.0, .segment_capture_radius_m = 0.1});

  ASSERT_TRUE(directive_result.has_value());
  const IncrementalTopologicalLatticeDirective3D directive =
      directive_result.value_or(IncrementalTopologicalLatticeDirective3D{});
  EXPECT_DOUBLE_EQ(directive.source_station_m, 6.0);
  EXPECT_DOUBLE_EQ(directive.projection_distance_m, 1.0);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.x, 10.0);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.y, 0.0);
  EXPECT_DOUBLE_EQ(directive.target_station_m, 10.0);
}

TEST(IncrementalTopologicalLatticeAdapter3DTest,
     ProjectionFloorDisambiguatesASelfOverlappingRoute) {
  const std::vector<Point3> points{
      {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}};

  const auto initial = projectOntoTopologicalPolyline3D(points, {2.0, 0.0, 0.0});
  const auto continued = projectOntoTopologicalPolyline3D(points, {2.0, 0.0, 0.0}, 9.0);

  ASSERT_TRUE(initial.has_value());
  ASSERT_TRUE(continued.has_value());
  const TopologicalPolylineProjection3D initial_projection =
      initial.value_or(TopologicalPolylineProjection3D{});
  const TopologicalPolylineProjection3D continued_projection =
      continued.value_or(TopologicalPolylineProjection3D{});
  EXPECT_EQ(initial_projection.segment_index, 0U);
  EXPECT_DOUBLE_EQ(initial_projection.station_m, 2.0);
  EXPECT_EQ(continued_projection.segment_index, 1U);
  EXPECT_DOUBLE_EQ(continued_projection.station_m, 18.0);
  EXPECT_NEAR(continued_projection.distance_m, 0.0, 1.0e-12);
}

TEST(IncrementalTopologicalLatticeAdapter3DTest,
     PreservesBacktrackPurposeAndAllowsMovementAwayFromMissionGoal) {
  const IncrementalTopologicalPlan3D plan =
      executablePlan(IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack,
                     {{10.0, 0.0, 4.0}, {2.0, 0.0, 4.0}});

  const auto directive_result = makeIncrementalTopologicalLatticeDirective3D(
      plan, {10.0, 0.0, 4.0},
      {.maximum_lookahead_m = 30.0, .segment_capture_radius_m = 0.1});

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
     MissionContinuationRemainsExecutableWithoutClaimingGoalArrival) {
  IncrementalTopologicalPlan3D plan;
  plan.status = IncrementalTopologicalPlanStatus3D::kMissionContinuationRoute;
  plan.purpose = IncrementalTopologicalRoutePurpose3D::kMissionTransit;
  plan.strategic_plan_id = 27U;
  plan.guidance_points = {{0.0, 0.0, 4.0}, {0.0, 8.0, 4.0}, {8.0, 8.0, 4.0}};
  plan.strategic_boundary_stations_m = {16.0};

  const auto directive_result = makeIncrementalTopologicalLatticeDirective3D(
      plan, {0.0, 0.0, 4.0},
      {.maximum_lookahead_m = 30.0, .segment_capture_radius_m = 0.1});

  ASSERT_TRUE(directive_result.has_value());
  const IncrementalTopologicalLatticeDirective3D directive =
      directive_result.value_or(IncrementalTopologicalLatticeDirective3D{});
  EXPECT_EQ(directive.lattice.route_purpose, Lattice3DRoutePurpose::kMissionTransit);
  EXPECT_EQ(directive.strategic_plan_id, plan.strategic_plan_id);
  EXPECT_TRUE(directive.reaches_topological_target);
  EXPECT_FALSE(directive.lattice.reaches_mission_goal);
  EXPECT_FALSE(isExplicitTopologicalBacktrack3D(plan));
}

TEST(IncrementalTopologicalLatticeAdapter3DTest,
     SkipsAStrategicBoundaryInsideGoalCaptureRadius) {
  IncrementalTopologicalPlan3D plan =
      executablePlan(IncrementalTopologicalRoutePurpose3D::kMissionTransit,
                     {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 10.0, 0.0}});
  plan.strategic_boundary_stations_m = {0.5, 10.5};

  const auto directive_result = makeIncrementalTopologicalLatticeDirective3D(
      plan, {0.0, 0.0, 0.0},
      {.maximum_lookahead_m = 6.0, .segment_capture_radius_m = 2.0});

  ASSERT_TRUE(directive_result.has_value());
  const IncrementalTopologicalLatticeDirective3D directive =
      directive_result.value_or(IncrementalTopologicalLatticeDirective3D{});
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.x, 0.5);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.y, 5.5);
  EXPECT_DOUBLE_EQ(directive.lattice.preferred_direction.x, 0.0);
  EXPECT_DOUBLE_EQ(directive.lattice.preferred_direction.y, 10.0);
  EXPECT_EQ(directive.captured_boundaries_skipped, 1U);
  EXPECT_FALSE(directive.reaches_topological_target);
}

TEST(IncrementalTopologicalLatticeAdapter3DTest,
     CoalescesStrategicBoundariesUntilExecutableLookahead) {
  IncrementalTopologicalPlan3D plan = executablePlan(
      IncrementalTopologicalRoutePurpose3D::kMissionTransit,
      {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}});
  plan.strategic_boundary_stations_m = {5.0, 10.0, 20.0};

  const auto directive_result = makeIncrementalTopologicalLatticeDirective3D(
      plan, {0.0, 0.0, 0.0},
      {.maximum_lookahead_m = 15.0,
       .minimum_executable_lookahead_m = 12.0,
       .segment_capture_radius_m = 0.1});

  ASSERT_TRUE(directive_result.has_value());
  const IncrementalTopologicalLatticeDirective3D directive =
      directive_result.value_or(IncrementalTopologicalLatticeDirective3D{});
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.x, 15.0);
  EXPECT_DOUBLE_EQ(directive.target_station_m, 15.0);
  EXPECT_EQ(directive.captured_boundaries_skipped, 0U);
  EXPECT_EQ(directive.executable_lookahead_boundaries_skipped, 2U);
  EXPECT_FALSE(directive.reaches_topological_target);
}

TEST(IncrementalTopologicalLatticeAdapter3DTest,
     TraversesGeometryBendsWithinOneStrategicEdge) {
  IncrementalTopologicalPlan3D plan = executablePlan(
      IncrementalTopologicalRoutePurpose3D::kMissionTransit,
      {{0.0, 0.0, 0.0}, {4.0, 0.0, 0.0}, {4.0, 4.0, 0.0}, {10.0, 4.0, 0.0}});
  plan.strategic_boundary_stations_m = {14.0};

  const auto directive_result = makeIncrementalTopologicalLatticeDirective3D(
      plan, {0.0, 0.0, 0.0},
      {.maximum_lookahead_m = 12.0, .segment_capture_radius_m = 0.1});

  ASSERT_TRUE(directive_result.has_value());
  const IncrementalTopologicalLatticeDirective3D directive =
      directive_result.value_or(IncrementalTopologicalLatticeDirective3D{});
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.x, 8.0);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.y, 4.0);
  EXPECT_DOUBLE_EQ(directive.lattice.planning_goal.z, 0.0);
  EXPECT_DOUBLE_EQ(directive.lattice.preferred_direction.x, 4.0);
  EXPECT_DOUBLE_EQ(directive.lattice.preferred_direction.y, 0.0);
  EXPECT_DOUBLE_EQ(directive.target_station_m, 12.0);
  EXPECT_EQ(directive.captured_boundaries_skipped, 0U);
  EXPECT_FALSE(directive.reaches_topological_target);
}

TEST(IncrementalTopologicalLatticeAdapter3DTest,
     DoesNotReacquireACompletedPolylineFromCrossTrack) {
  const IncrementalTopologicalPlan3D plan =
      executablePlan(IncrementalTopologicalRoutePurpose3D::kMissionTransit,
                     {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}});
  const Point3 position{15.0, 8.0, 0.0};

  const auto projection =
      projectOntoTopologicalPolyline3D(plan.guidance_points, position);
  const auto directive = makeIncrementalTopologicalLatticeDirective3D(
      plan, position, {.maximum_lookahead_m = 15.0, .segment_capture_radius_m = 0.1});

  ASSERT_TRUE(projection.has_value());
  const auto completed_projection =
      projection.value_or(TopologicalPolylineProjection3D{});
  EXPECT_DOUBLE_EQ(completed_projection.station_m, 10.0);
  EXPECT_DOUBLE_EQ(completed_projection.remaining_m, 0.0);
  EXPECT_FALSE(directive.has_value());
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

  const IncrementalTopologicalPlan3D route =
      executablePlan(IncrementalTopologicalRoutePurpose3D::kMissionTransit,
                     {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}});
  EXPECT_FALSE(
      makeIncrementalTopologicalLatticeDirective3D(route, {1.0, 0.0, 0.0}, {}, -1.0)
          .has_value());
}

} // namespace
} // namespace drone_city_nav
