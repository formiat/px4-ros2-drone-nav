#include "offboard_velocity_follower_test_helpers.hpp"

namespace drone_city_nav {

using offboard_velocity_follower_test_helpers::lineTrajectory;
using offboard_velocity_follower_test_helpers::normalizedTestVector;
using offboard_velocity_follower_test_helpers::testConfig;
using offboard_velocity_follower_test_helpers::trajectoryWithArc;

TEST(OffboardVelocityFollower, EmptyTrajectoryReturnsInvalidPlan) {
  const std::vector<TrajectorySegment> trajectory{};
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{0.0, 0.0}, Point2{}, false, 0.1,
                           VelocityFollowerState{}, testConfig());

  EXPECT_FALSE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kInvalidPath);
}

TEST(OffboardVelocityFollower, NonFinitePositionReturnsInvalidPlan) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      trajectory, profile, Point2{std::numeric_limits<double>::quiet_NaN(), 0.0},
      Point2{}, false, 0.1, VelocityFollowerState{}, testConfig());

  EXPECT_FALSE(plan.valid);
}

} // namespace drone_city_nav
