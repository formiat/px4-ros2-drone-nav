#include <variant>

#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

static_assert(std::variant_size_v<ExecutionPlanTransitionCommand3D> == 14U,
              "every execution transition must remain a tagged reducer command");

TEST(ExecutionPlanReducer3DTest,
     TaggedCommandProducesANewPlanWithoutMutatingItsPredecessor) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_EQ(active->phase(), ExecutionRoutePhase3D::kFollowing);
  ASSERT_NE(active->route(), nullptr);
  const std::uint64_t active_version = active->version;
  const std::uint64_t active_owner_epoch = active->execution_owner_epoch;
  const RouteInstanceId3D active_route_instance = active->route()->route_instance_id;

  const ExecutionRouteTransitionResult3D revoked =
      reduceExecutionPlan3D(*active, RevokeExecutionCommand3D{
                                         .expected_snapshot_version = active_version,
                                     });

  ASSERT_TRUE(revoked.applied());
  ASSERT_EQ(revoked.predecessor, active.get());
  ASSERT_NE(revoked.next, nullptr);
  EXPECT_EQ(revoked.next->phase(), ExecutionRoutePhase3D::kRevoked);
  EXPECT_EQ(revoked.next->version, active_version + 1U);
  EXPECT_EQ(revoked.next->execution_owner_epoch, active_owner_epoch + 1U);
  EXPECT_EQ(revoked.next->routeGenerationHighWater(),
            active->routeGenerationHighWater());

  EXPECT_EQ(active->phase(), ExecutionRoutePhase3D::kFollowing);
  EXPECT_EQ(active->version, active_version);
  ASSERT_NE(active->route(), nullptr);
  EXPECT_EQ(active->route()->route_instance_id, active_route_instance);
}

TEST(ExecutionPlanReducer3DTest,
     CompatibilityAdapterDelegatesToTheSameTaggedTransition) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);

  const ExecutionRouteTransitionResult3D reduced =
      reduceExecutionPlan3D(*active, SuspendFiniteExecutionCommand3D{
                                         .expected_snapshot_version = active->version,
                                     });
  const ExecutionRouteTransitionResult3D adapted =
      suspendFiniteExecution3D(*active, active->version);

  ASSERT_TRUE(reduced.applied());
  ASSERT_TRUE(adapted.applied());
  ASSERT_NE(reduced.next, nullptr);
  ASSERT_NE(adapted.next, nullptr);
  EXPECT_EQ(reduced.next->phase(), adapted.next->phase());
  EXPECT_EQ(reduced.next->version, adapted.next->version);
  EXPECT_EQ(reduced.next->execution_owner_epoch, adapted.next->execution_owner_epoch);
  EXPECT_EQ(reduced.next->routeGenerationHighWater(),
            adapted.next->routeGenerationHighWater());
  ASSERT_NE(reduced.next->route(), nullptr);
  ASSERT_NE(adapted.next->route(), nullptr);
  EXPECT_EQ(reduced.next->route()->route_instance_id,
            adapted.next->route()->route_instance_id);
  EXPECT_EQ(reduced.next->finiteExecution(), nullptr);
  EXPECT_EQ(adapted.next->finiteExecution(), nullptr);
}

TEST(ExecutionPlanReducer3DTest, MalformedTaggedCommandFailsClosed) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);

  const ExecutionRouteTransitionResult3D rejected =
      reduceExecutionPlan3D(*active, ReplaceCertifiedRouteCommand3D{
                                         .guard = SnapshotFixture3D::guard(*active),
                                         .splice = nullptr,
                                     });

  EXPECT_EQ(rejected.status, ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  EXPECT_EQ(rejected.detail,
            ExecutionRouteTransitionDetail3D::kReplacementWithoutSplice);
  EXPECT_FALSE(rejected.applied());
  EXPECT_EQ(rejected.predecessor, nullptr);
  EXPECT_EQ(rejected.next, nullptr);
  EXPECT_EQ(active->phase(), ExecutionRoutePhase3D::kFollowing);
  EXPECT_TRUE(active->publishable());
}

} // namespace
} // namespace drone_city_nav
