#include "drone_city_nav/strategic_route_manager_3d.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] IncrementalTopologicalPlan3D missionPlan() {
  IncrementalTopologicalPlan3D plan;
  plan.status = IncrementalTopologicalPlanStatus3D::kMissionContinuationRoute;
  plan.purpose = IncrementalTopologicalRoutePurpose3D::kMissionTransit;
  plan.planned_on_revision = 7U;
  plan.validated_through_revision = 9U;
  plan.mission_target = {30.0, 10.0, 4.0};
  plan.start_node = IncrementalTopologyNodeId{1U};
  plan.target_node = IncrementalTopologyNodeId{3U};
  plan.route_nodes = {IncrementalTopologyNodeId{1U}, IncrementalTopologyNodeId{2U},
                      IncrementalTopologyNodeId{3U}};
  plan.guidance_points = {{0.0, 0.0, 4.0}, {10.0, 0.0, 4.0}, {10.0, 10.0, 4.0}};
  return plan;
}

TEST(StrategicRouteManager3DTest, OwnsPlanIdentityCorridorAndMonotonicCursor) {
  StrategicRouteManager3D manager;
  IncrementalTopologicalPlan3D plan = missionPlan();

  EXPECT_EQ(manager.previewPlanId(plan), 1U);
  const StrategicRouteCommit3D initial = manager.accept(plan);

  ASSERT_TRUE(initial.accepted);
  EXPECT_EQ(initial.plan_id, 1U);
  ASSERT_NE(manager.active(), nullptr);
  EXPECT_EQ(manager.active()->plan.strategic_plan_id, initial.plan_id);
  ASSERT_EQ(manager.active()->plan.guidance_points.size(), plan.guidance_points.size());
  for (std::size_t index = 0U; index < plan.guidance_points.size(); ++index) {
    EXPECT_NEAR(distance3D(manager.active()->plan.guidance_points[index],
                           plan.guidance_points[index]),
                0.0, 1.0e-12);
  }

  EXPECT_TRUE(manager.advance(plan, 12.0, 1U));
  EXPECT_TRUE(manager.advance(plan, 12.0, 0U));
  EXPECT_TRUE(manager.advance(plan, 3.0, 0U));
  ASSERT_NE(manager.active(), nullptr);
  EXPECT_DOUBLE_EQ(manager.active()->cursor.station_m, 12.0);
  EXPECT_EQ(manager.active()->cursor.segment_index, 1U);

  plan.continued_from_active_plan = true;
  plan.strategic_plan_id = initial.plan_id;
  const StrategicRouteCommit3D continued = manager.accept(plan);
  EXPECT_TRUE(continued.accepted);
  EXPECT_TRUE(continued.cursor_preserved);
  EXPECT_EQ(continued.plan_id, initial.plan_id);
  ASSERT_NE(manager.active(), nullptr);
  EXPECT_DOUBLE_EQ(manager.active()->cursor.station_m, 12.0);
}

TEST(StrategicRouteManager3DTest, ReplacementGetsNewPlanIdAndFreshCursor) {
  StrategicRouteManager3D manager;
  const IncrementalTopologicalPlan3D first = missionPlan();
  ASSERT_TRUE(manager.accept(first).accepted);
  ASSERT_TRUE(manager.advance(first, 8.0, 0U));
  IncrementalTopologicalPlan3D replacement = first;
  replacement.mission_target = {30.0, -10.0, 4.0};

  EXPECT_EQ(manager.previewPlanId(replacement), 2U);
  const StrategicRouteCommit3D committed = manager.accept(replacement);

  EXPECT_TRUE(committed.accepted);
  EXPECT_FALSE(committed.cursor_preserved);
  EXPECT_EQ(committed.plan_id, 2U);
  ASSERT_NE(manager.active(), nullptr);
  EXPECT_DOUBLE_EQ(manager.active()->cursor.station_m, 0.0);
}

TEST(StrategicRouteManager3DTest, InvalidationIsScopedToMatchingPlan) {
  StrategicRouteManager3D manager;
  const IncrementalTopologicalPlan3D plan = missionPlan();
  ASSERT_TRUE(manager.accept(plan).accepted);
  IncrementalTopologicalPlan3D unrelated = plan;
  unrelated.target_node.value += 1U;

  EXPECT_FALSE(manager.invalidate(unrelated));
  EXPECT_NE(manager.active(), nullptr);
  EXPECT_TRUE(manager.invalidate(plan));
  EXPECT_EQ(manager.active(), nullptr);
  EXPECT_FALSE(manager.supersede());
}

TEST(StrategicRouteManager3DTest, ResetDoesNotReuseAnAllocatedPlanId) {
  StrategicRouteManager3D manager;
  const IncrementalTopologicalPlan3D first = missionPlan();
  ASSERT_EQ(manager.accept(first).plan_id, 1U);
  manager.reset();
  IncrementalTopologicalPlan3D second = first;
  second.target_node.value += 1U;

  EXPECT_EQ(manager.previewPlanId(second), 2U);
  EXPECT_EQ(manager.accept(second).plan_id, 2U);
  EXPECT_EQ(manager.lastAllocatedPlanId(), 2U);
}

} // namespace
} // namespace drone_city_nav
