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
  config.maximum_braking_acceleration_mps2 = 8.0;
  config.reaction_latency_s = 0.1;

  const MppiSpeedPolicyResult result =
      evaluateStaticMppiSpeedPolicy(config, MppiSpeedPolicyInput{});

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

  const MppiSpeedPolicyResult result = evaluateStaticMppiSpeedPolicy(config, input);

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

  const MppiSpeedPolicyResult result = evaluateStaticMppiSpeedPolicy(config, input);

  EXPECT_GT(result.maximum_preview_curvature_1pm, 0.19);
  EXPECT_LT(result.curvature_limit_mps, 14.0);
  EXPECT_LT(result.reference_speed_mps, config.cruise_speed_mps);
}

TEST(MppiSpeedPolicyTest, PassageAndGoalApplyIndependentCaps) {
  MppiSpeedPolicyConfig config;
  MppiSpeedPolicyInput passage_input;
  passage_input.mission_goal = Point3{200.0, 0.0, 18.0};
  passage_input.passage_speed_limit_mps = 10.0;
  const MppiSpeedPolicyResult passage =
      evaluateStaticMppiSpeedPolicy(config, passage_input);
  EXPECT_DOUBLE_EQ(passage.reference_speed_mps, 10.0);

  MppiSpeedPolicyInput goal_input;
  goal_input.mission_goal = Point3{10.0, 0.0, 18.0};
  const MppiSpeedPolicyResult goal = evaluateStaticMppiSpeedPolicy(config, goal_input);
  EXPECT_LT(goal.goal_limit_mps, 12.0);
  EXPECT_DOUBLE_EQ(goal.reference_speed_mps, goal.goal_limit_mps);
}

} // namespace
} // namespace drone_city_nav
