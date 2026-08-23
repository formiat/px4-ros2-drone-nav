#include "drone_city_nav/execution_arbiter_3d.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

struct TestFiniteTrajectory {
  int identity{0};
};

TEST(ExecutionArbiter3DTest, ControlCandidateRejectionPreservesFiniteTrajectory) {
  ExecutionArbiter3D<TestFiniteTrajectory> arbiter;
  arbiter.activate(7U, TestFiniteTrajectory{.identity = 42});

  EXPECT_TRUE(arbiter.observe(RouteLifecycleEvent3D{
      .kind = RouteLifecycleEventKind3D::kControlCandidateRejected,
      .generation = 7U,
  }));

  ASSERT_NE(arbiter.activeTrajectory(), nullptr);
  EXPECT_EQ(arbiter.activeTrajectory()->identity, 42);
  EXPECT_FALSE(arbiter.trajectoryRevalidationRequired());
}

TEST(ExecutionArbiter3DTest, RawInvalidationRequiresValidationBeforeRetention) {
  ExecutionArbiter3D<TestFiniteTrajectory> arbiter;
  arbiter.activate(7U, TestFiniteTrajectory{.identity = 42});

  EXPECT_TRUE(arbiter.observe(RouteLifecycleEvent3D{
      .kind = RouteLifecycleEventKind3D::kRawInvalidated,
      .generation = 7U,
  }));

  ASSERT_NE(arbiter.activeTrajectory(), nullptr);
  EXPECT_TRUE(arbiter.trajectoryRevalidationRequired());
  arbiter.confirmRetainedTrajectory();
  EXPECT_FALSE(arbiter.trajectoryRevalidationRequired());
}

TEST(ExecutionArbiter3DTest, StaleEventCannotAffectANewerFiniteTrajectory) {
  ExecutionArbiter3D<TestFiniteTrajectory> arbiter;
  arbiter.activate(8U, TestFiniteTrajectory{.identity = 43});

  EXPECT_FALSE(arbiter.observe(RouteLifecycleEvent3D{
      .kind = RouteLifecycleEventKind3D::kRawInvalidated,
      .generation = 7U,
  }));

  ASSERT_NE(arbiter.activeTrajectory(), nullptr);
  EXPECT_EQ(arbiter.activeTrajectory()->identity, 43);
  EXPECT_FALSE(arbiter.trajectoryRevalidationRequired());
}

TEST(ExecutionArbiter3DTest, RejectedValidationLeavesOnlyExplicitHoldState) {
  ExecutionArbiter3D<TestFiniteTrajectory> arbiter;
  arbiter.activate(7U, TestFiniteTrajectory{.identity = 42});
  arbiter.rejectTrajectory();
  arbiter.enterNoExecutableHold(Point3{1.0, 2.0, 3.0});

  EXPECT_EQ(arbiter.activeTrajectory(), nullptr);
  ASSERT_TRUE(arbiter.noExecutableHoldPosition().has_value());
  EXPECT_DOUBLE_EQ(arbiter.noExecutableHoldPosition().value_or(Point3{}).z, 3.0);

  arbiter.leaveNoExecutableHold();
  EXPECT_FALSE(arbiter.noExecutableHoldPosition().has_value());
}

} // namespace
} // namespace drone_city_nav
