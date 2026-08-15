#include "drone_city_nav/mppi_speed_policy.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace drone_city_nav {
namespace {

TEST(MppiSpeedPolicyTest, ObservationRangeLimitsStoppingSpeed) {
  MppiSpeedPolicyConfig config;
  config.observation_distance_m = 30.0;
  config.observation_margin_m = 3.0;
  config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 = 8.0;
  config.stopping_capability.reaction_latency_s = 0.1;

  const MppiSpeedPolicyResult result =
      evaluateMppiSpeedPolicy(config, MppiSpeedPolicyInput{});

  EXPECT_NEAR(result.observation_limit_mps, 20.00, 0.01);
  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 0.0);
}

TEST(MppiSpeedPolicyTest, StraightGuideUsesCruiseAndHundredMeterLookahead) {
  MppiSpeedPolicyConfig config;
  const std::array<Point2, 4> guide{Point2{0.0, 0.0}, Point2{40.0, 0.0},
                                    Point2{80.0, 0.0}, Point2{180.0, 0.0}};
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{300.0, 0.0, 18.0};
  input.guide = guide;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 20.0);
  EXPECT_DOUBLE_EQ(result.target_lookahead_m, 100.0);
  EXPECT_DOUBLE_EQ(result.maximum_preview_curvature_1pm, 0.0);
}

TEST(MppiSpeedPolicyTest, UpcomingTurnReducesReferenceSpeedBeforeTurn) {
  MppiSpeedPolicyConfig config;
  const std::array<Point2, 4> guide{Point2{0.0, 0.0}, Point2{8.0, 0.0},
                                    Point2{8.0, 8.0}, Point2{8.0, 40.0}};
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{8.0, 100.0, 18.0};
  input.guide = guide;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_GT(result.maximum_preview_curvature_1pm, 0.19);
  EXPECT_LT(result.curvature_limit_mps, 14.0);
  EXPECT_LT(result.reference_speed_mps, config.cruise_speed_mps);
}

TEST(MppiSpeedPolicyTest, RouteConstraintAndGoalApplyIndependentCaps) {
  MppiSpeedPolicyConfig config;
  MppiSpeedPolicyInput passage_input;
  passage_input.mission_goal = Point3{200.0, 0.0, 18.0};
  passage_input.route_constraint_speed_limit_mps = 10.0;
  const MppiSpeedPolicyResult passage = evaluateMppiSpeedPolicy(config, passage_input);
  EXPECT_DOUBLE_EQ(passage.reference_speed_mps, 10.0);

  MppiSpeedPolicyInput goal_input;
  goal_input.mission_goal = Point3{10.0, 0.0, 18.0};
  const MppiSpeedPolicyResult goal = evaluateMppiSpeedPolicy(config, goal_input);
  EXPECT_LT(goal.goal_limit_mps, 12.0);
  EXPECT_DOUBLE_EQ(goal.reference_speed_mps, goal.goal_limit_mps);
  EXPECT_EQ(goal.active_limiter, MppiSpeedLimiter::kGoal);
}

TEST(MppiSpeedPolicyTest, ContinuousTrackingDoesNotBrakeForMovingGoal) {
  MppiSpeedPolicyConfig config;
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{1.0, 0.0, 18.0};
  input.terminal_goal_limit_enabled = false;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_FALSE(result.terminal_goal_limit_enabled);
  EXPECT_TRUE(std::isinf(result.goal_limit_mps));
  EXPECT_DOUBLE_EQ(result.reference_speed_mps, config.cruise_speed_mps);
  EXPECT_NE(result.active_limiter, MppiSpeedLimiter::kGoal);
}

TEST(MppiSpeedPolicyTest, FrontierRouteCanStopBeforeUnextendedEndpoint) {
  MppiSpeedPolicyConfig config;
  config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 = 8.0;
  config.stopping_capability.reaction_latency_s = 0.1;
  config.goal_margin_m = 2.0;
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{300.0, 0.0, 18.0};
  input.route_endpoint_remaining_m = 6.0;

  const MppiSpeedPolicyResult approaching = evaluateMppiSpeedPolicy(config, input);

  EXPECT_NEAR(approaching.route_endpoint_limit_mps, 7.23, 0.01);
  EXPECT_DOUBLE_EQ(approaching.reference_speed_mps,
                   approaching.route_endpoint_limit_mps);
  EXPECT_EQ(approaching.active_limiter, MppiSpeedLimiter::kRouteEndpoint);

  input.route_endpoint_remaining_m = 2.0;
  const MppiSpeedPolicyResult at_margin = evaluateMppiSpeedPolicy(config, input);
  EXPECT_DOUBLE_EQ(at_margin.route_endpoint_limit_mps, 0.0);
  EXPECT_DOUBLE_EQ(at_margin.reference_speed_mps, 0.0);
}

TEST(MppiSpeedPolicyTest, TerminalRouteUsesMissionGoalLimitOnly) {
  MppiSpeedPolicyConfig config;
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{100.0, 0.0, 18.0};

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_TRUE(std::isinf(result.route_endpoint_limit_mps));
  EXPECT_NE(result.active_limiter, MppiSpeedLimiter::kRouteEndpoint);
}

TEST(MppiSpeedPolicyTest, NoStaticProfileTracksTenMetersPerSecondAtFixedLookahead) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 10.0;
  config.absolute_speed_limit_mps = 10.0;
  config.maximum_lateral_acceleration_mps2 = 4.0;
  config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 = 4.0;
  config.horizon_duration_s = 4.0;
  config.minimum_target_lookahead_m = 30.0;
  config.maximum_target_lookahead_m = 30.0;
  const std::array<Point2, 3> guide{Point2{0.0, 0.0}, Point2{30.0, 0.0},
                                    Point2{60.0, 0.0}};
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{300.0, 0.0, 18.0};
  input.guide = guide;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_TRUE(result.enabled);
  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 10.0);
  EXPECT_DOUBLE_EQ(result.absolute_limit_mps, 10.0);
  EXPECT_DOUBLE_EQ(result.target_lookahead_m, 30.0);
}

TEST(MppiSpeedPolicyTest, RouteConstraintLimitOverridesCruiseSpeed) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 10.0;
  config.absolute_speed_limit_mps = 10.0;
  config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 = 4.0;
  config.minimum_target_lookahead_m = 30.0;
  config.maximum_target_lookahead_m = 30.0;
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{300.0, 0.0, 18.0};
  input.route_constraint_speed_limit_mps = 5.0;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 5.0);
  EXPECT_DOUBLE_EQ(result.route_constraint_limit_mps, 5.0);
}

} // namespace
} // namespace drone_city_nav
