#include "drone_city_nav/mppi_speed_policy.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace drone_city_nav {
namespace {

void allowHighSensorBrakingSpeed(MppiSpeedPolicyConfig& config) {
  config.sensor_braking_contract.guaranteed_detection_range_m = 1000.0;
  config.sensor_braking_contract.maximum_evidence_age_s = 0.0;
  config.sensor_braking_contract.physical_margin_m = 0.0;
}

TEST(MppiSpeedPolicyTest, SensorBrakingContractLimitsReferenceSpeed) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  config.sensor_braking_contract.guaranteed_detection_range_m = 30.0;
  config.sensor_braking_contract.maximum_evidence_age_s = 1.0;
  config.sensor_braking_contract.physical_margin_m = 3.0;
  config.sensor_braking_contract.maximum_forward_acceleration_mps2 = 5.0;
  config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 = 4.0;
  config.stopping_capability.reaction_latency_s = 0.1;
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_DOUBLE_EQ(result.reference_speed_mps, result.sensor_braking_limit_mps);
  EXPECT_EQ(result.active_limiter, MppiSpeedLimiter::kSensorBraking);
  EXPECT_STREQ(mppiSpeedLimiterName(result.active_limiter), "sensor_braking");
  EXPECT_TRUE(result.sensor_braking_assessment.accepted());
  EXPECT_NEAR(result.sensor_braking_assessment.required_detection_range_m,
              config.sensor_braking_contract.guaranteed_detection_range_m, 1.0e-10);
}

TEST(MppiSpeedPolicyTest, MeasuredOverspeedRequestsBrakingInsteadOfNewMotion) {
  MppiSpeedPolicyConfig config;
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  input.state.vx = 20.0F;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 0.0);
  EXPECT_EQ(result.active_limiter, MppiSpeedLimiter::kSensorBraking);
  EXPECT_DOUBLE_EQ(result.sensor_braking_assessment.speed_mps, 20.0);
  EXPECT_FALSE(result.sensor_braking_assessment.accepted());
}

TEST(MppiSpeedPolicyTest, StraightGuideUsesCruiseAndHundredMeterLookahead) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 = 8.0;
  config.stopping_capability.reaction_latency_s = 0.1;
  allowHighSensorBrakingSpeed(config);
  const std::array<RouteSample3D, 4> route{
      RouteSample3D{.position = {0.0, 0.0, 0.0}},
      RouteSample3D{.position = {40.0, 0.0, 0.0}},
      RouteSample3D{.position = {80.0, 0.0, 0.0}},
      RouteSample3D{.position = {180.0, 0.0, 0.0}}};
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{300.0, 0.0, 18.0};
  input.route = route;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 20.0);
  EXPECT_DOUBLE_EQ(result.target_lookahead_m, 100.0);
  EXPECT_DOUBLE_EQ(result.maximum_preview_curvature_1pm, 0.0);
}

TEST(MppiSpeedPolicyTest, UpcomingTurnReducesReferenceSpeedBeforeTurn) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  allowHighSensorBrakingSpeed(config);
  const std::array<RouteSample3D, 4> route{RouteSample3D{.position = {0.0, 0.0, 0.0}},
                                           RouteSample3D{.position = {8.0, 0.0, 0.0}},
                                           RouteSample3D{.position = {8.0, 8.0, 0.0}},
                                           RouteSample3D{.position = {8.0, 40.0, 0.0}}};
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{8.0, 100.0, 18.0};
  input.route = route;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_GT(result.maximum_preview_curvature_1pm, 0.19);
  EXPECT_LT(result.curvature_limit_mps, 14.0);
  EXPECT_LT(result.reference_speed_mps, config.cruise_speed_mps);
}

TEST(MppiSpeedPolicyTest, RouteConstraintAndGoalApplyIndependentCaps) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  allowHighSensorBrakingSpeed(config);
  MppiSpeedPolicyInput passage_input;
  passage_input.mission_goal = Point3{200.0, 0.0, 18.0};
  passage_input.route_constraint_speed_limit_mps = 10.0;
  const MppiSpeedPolicyResult passage = evaluateMppiSpeedPolicy(config, passage_input);
  EXPECT_DOUBLE_EQ(passage.reference_speed_mps, 10.0);

  MppiSpeedPolicyInput goal_input;
  goal_input.mission_goal = Point3{10.0, 0.0, 0.0};
  const MppiSpeedPolicyResult goal = evaluateMppiSpeedPolicy(config, goal_input);
  EXPECT_LT(goal.goal_limit_mps, 12.0);
  EXPECT_DOUBLE_EQ(goal.reference_speed_mps, goal.goal_limit_mps);
  EXPECT_EQ(goal.active_limiter, MppiSpeedLimiter::kGoal);
}

TEST(MppiSpeedPolicyTest, PureVerticalMissionUsesTheSameGoalBrakingMetric) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  allowHighSensorBrakingSpeed(config);
  MppiSpeedPolicyInput input;
  input.state.z = 10.0F;
  input.mission_goal = Point3{0.0, 0.0, 20.0};

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_LT(result.goal_limit_mps, config.cruise_speed_mps);
  EXPECT_DOUBLE_EQ(result.reference_speed_mps, result.goal_limit_mps);
  EXPECT_EQ(result.active_limiter, MppiSpeedLimiter::kGoal);
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

TEST(MppiSpeedPolicyTest, ContinuationIgnoresItsLocalEndpointDistance) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  allowHighSensorBrakingSpeed(config);
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{300.0, 0.0, 18.0};
  input.route_endpoint_remaining_m = 0.0;
  input.route_endpoint_semantics = RouteEndpointSemantics3D::kContinuation;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_FALSE(result.route_endpoint_stop_required);
  EXPECT_EQ(result.route_endpoint_semantics, RouteEndpointSemantics3D::kContinuation);
  EXPECT_TRUE(std::isinf(result.route_endpoint_limit_mps));
  EXPECT_DOUBLE_EQ(result.reference_speed_mps, config.cruise_speed_mps);
  EXPECT_NE(result.active_limiter, MppiSpeedLimiter::kRouteEndpoint);
}

TEST(MppiSpeedPolicyTest, LocalStopBrakesAtItsFiniteEndpoint) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 = 8.0;
  config.stopping_capability.reaction_latency_s = 0.1;
  config.goal_margin_m = 2.0;
  allowHighSensorBrakingSpeed(config);
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{300.0, 0.0, 18.0};
  input.route_endpoint_remaining_m = 6.0;
  input.route_endpoint_semantics = RouteEndpointSemantics3D::kLocalStop;

  const MppiSpeedPolicyResult approaching = evaluateMppiSpeedPolicy(config, input);

  EXPECT_NEAR(approaching.route_endpoint_limit_mps, 9.03, 0.01);
  EXPECT_TRUE(approaching.route_endpoint_stop_required);
  EXPECT_EQ(approaching.route_endpoint_semantics, RouteEndpointSemantics3D::kLocalStop);
  EXPECT_DOUBLE_EQ(approaching.reference_speed_mps,
                   approaching.route_endpoint_limit_mps);
  EXPECT_EQ(approaching.active_limiter, MppiSpeedLimiter::kRouteEndpoint);

  input.route_endpoint_remaining_m = 1.0;
  const MppiSpeedPolicyResult near_endpoint = evaluateMppiSpeedPolicy(config, input);
  EXPECT_GT(near_endpoint.route_endpoint_limit_mps, 0.0);
  EXPECT_DOUBLE_EQ(near_endpoint.reference_speed_mps,
                   near_endpoint.route_endpoint_limit_mps);

  input.route_endpoint_remaining_m = 0.0;
  const MppiSpeedPolicyResult at_endpoint = evaluateMppiSpeedPolicy(config, input);
  EXPECT_DOUBLE_EQ(at_endpoint.route_endpoint_limit_mps, 0.0);
  EXPECT_DOUBLE_EQ(at_endpoint.reference_speed_mps, 0.0);
}

TEST(MppiSpeedPolicyTest, MissionStopAlsoHonorsItsCertifiedRouteEndpoint) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{300.0, 0.0, 18.0};
  input.route_endpoint_remaining_m = 0.0;
  input.route_endpoint_semantics = RouteEndpointSemantics3D::kMissionStop;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_TRUE(result.route_endpoint_stop_required);
  EXPECT_DOUBLE_EQ(result.route_endpoint_limit_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 0.0);
  EXPECT_EQ(result.active_limiter, MppiSpeedLimiter::kRouteEndpoint);
}

TEST(MppiSpeedPolicyTest, TerminalRouteUsesMissionGoalLimitOnly) {
  MppiSpeedPolicyConfig config;
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{100.0, 0.0, 18.0};

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_TRUE(std::isinf(result.route_endpoint_limit_mps));
  EXPECT_NE(result.active_limiter, MppiSpeedLimiter::kRouteEndpoint);
}

TEST(MppiSpeedPolicyTest, ExplicitProfileTracksTenMetersPerSecondAtFixedLookahead) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 10.0;
  config.absolute_speed_limit_mps = 10.0;
  config.maximum_lateral_acceleration_mps2 = 4.0;
  config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 = 4.0;
  config.horizon_duration_s = 4.0;
  config.minimum_target_lookahead_m = 30.0;
  config.maximum_target_lookahead_m = 30.0;
  allowHighSensorBrakingSpeed(config);
  const std::array<RouteSample3D, 3> route{RouteSample3D{.position = {0.0, 0.0, 0.0}},
                                           RouteSample3D{.position = {30.0, 0.0, 0.0}},
                                           RouteSample3D{.position = {60.0, 0.0, 0.0}}};
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{300.0, 0.0, 18.0};
  input.route = route;

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
  allowHighSensorBrakingSpeed(config);
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{300.0, 0.0, 18.0};
  input.route_constraint_speed_limit_mps = 5.0;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_DOUBLE_EQ(result.reference_speed_mps, 5.0);
  EXPECT_DOUBLE_EQ(result.route_constraint_limit_mps, 5.0);
}

} // namespace
} // namespace drone_city_nav
