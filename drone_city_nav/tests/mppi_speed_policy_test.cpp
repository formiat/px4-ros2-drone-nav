#include "drone_city_nav/mppi_speed_policy.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

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

TEST(MppiSpeedPolicyTest, MeasuredOverspeedKeepsTheReferenceAtTheSensorBrakingLimit) {
  // The excess above the reference is priced as overspeed by the optimizer,
  // so the policy names the sensor limiter and keeps the reference at the
  // speed the sensor range can stop within instead of dropping it to zero
  // and releasing it again once the vehicle dips under the limit.
  MppiSpeedPolicyConfig config;
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  input.state.vx = 20.0F;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_GT(result.reference_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.reference_speed_mps,
                   std::min(result.cruise_limit_mps, result.sensor_braking_limit_mps));
  EXPECT_EQ(result.active_limiter, MppiSpeedLimiter::kSensorBraking);
  EXPECT_DOUBLE_EQ(result.sensor_braking_assessment.speed_mps, 20.0);
  EXPECT_FALSE(result.sensor_braking_assessment.accepted());
}

TEST(MppiSpeedPolicyTest, ABlockedRouteLimitsSpeedToStopBeforeTheBlock) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 = 4.0;
  config.stopping_capability.guaranteed_horizontal_deceleration_mps2 = 4.0;
  config.stopping_capability.reaction_latency_s = 0.0;
  allowHighSensorBrakingSpeed(config);
  config.sensor_braking_contract.physical_margin_m = 3.0;
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  input.blocked_route_remaining_m = 8.0;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  // The stop lands the body margin before the block: 8 m less the 3 m margin.
  EXPECT_EQ(result.active_limiter, MppiSpeedLimiter::kBlockedRoute);
  EXPECT_STREQ(mppiSpeedLimiterName(result.active_limiter), "blocked_route");
  EXPECT_NEAR(result.blocked_route_limit_mps, std::sqrt(2.0 * 4.0 * 5.0), 1.0e-6);
  EXPECT_DOUBLE_EQ(result.reference_speed_mps, result.blocked_route_limit_mps);

  input.blocked_route_remaining_m = std::nullopt;
  const MppiSpeedPolicyResult open = evaluateMppiSpeedPolicy(config, input);
  EXPECT_NE(open.active_limiter, MppiSpeedLimiter::kBlockedRoute);
  EXPECT_DOUBLE_EQ(open.reference_speed_mps, 20.0);
}

MppiSpeedPolicyConfig clearanceLimiterConfig() {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 = 4.0;
  config.stopping_capability.guaranteed_horizontal_deceleration_mps2 = 4.0;
  config.stopping_capability.reaction_latency_s = 0.0;
  config.clearance_response_time_s = 0.5;
  config.clearance_minimum_progress_speed_mps = 1.0;
  allowHighSensorBrakingSpeed(config);
  return config;
}

ExecutedHorizonClearance3D executedClearance(const double distance_m,
                                             const double clearance_m) {
  return ExecutedHorizonClearance3D{
      .available = true,
      .constrained_samples = {ConstrainedHorizonSample3D{.distance_m = distance_m,
                                                         .clearance_m = clearance_m}},
      .minimum_clearance_m = clearance_m,
  };
}

TEST(MppiSpeedPolicyTest,
     TheExecutedHorizonClearanceCapsTheReferenceSpeedAtTheTightPoint) {
  const MppiSpeedPolicyConfig config = clearanceLimiterConfig();
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  input.executed_horizon_clearance = executedClearance(0.0, 2.0);

  const MppiSpeedPolicyResult beside = evaluateMppiSpeedPolicy(config, input);
  EXPECT_EQ(beside.active_limiter, MppiSpeedLimiter::kClearance);
  EXPECT_STREQ(mppiSpeedLimiterName(beside.active_limiter), "clearance");
  // Evidence beside the motion answers to the tube law alone: the error the
  // controller can accumulate within 0.5 s must fit inside 2 m, so 4 m/s. No
  // braking distance is owed to a wall the vehicle flies along.
  EXPECT_NEAR(beside.clearance_limit_mps, 2.0 / 0.5, 1.0e-6);
  EXPECT_DOUBLE_EQ(beside.reference_speed_mps, beside.clearance_limit_mps);

  input.executed_horizon_clearance = executedClearance(0.0, 0.0);
  const MppiSpeedPolicyResult touching = evaluateMppiSpeedPolicy(config, input);
  EXPECT_EQ(touching.active_limiter, MppiSpeedLimiter::kClearance);
  EXPECT_DOUBLE_EQ(touching.reference_speed_mps, 1.0);

  input.executed_horizon_clearance = std::nullopt;
  const MppiSpeedPolicyResult open = evaluateMppiSpeedPolicy(config, input);
  EXPECT_NE(open.active_limiter, MppiSpeedLimiter::kClearance);
  EXPECT_DOUBLE_EQ(open.reference_speed_mps, 20.0);
}

TEST(MppiSpeedPolicyTest, ATightPointFarAheadOnlyHasToBeReachedSlowly) {
  const MppiSpeedPolicyConfig config = clearanceLimiterConfig();
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  // The tube admits 0.5 / 0.5 = 1 m/s where the horizon grazes an obstacle
  // fifteen metres ahead, which is also the floor. Braking at 4 m/s^2 the
  // vehicle may still be doing 11 m/s now.
  input.executed_horizon_clearance = executedClearance(15.0, 0.5);

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_NEAR(result.clearance_limit_mps, std::sqrt(1.0 + 2.0 * 4.0 * 15.0), 1.0e-6);
  EXPECT_GT(result.clearance_limit_mps, 10.0);
}

TEST(MppiSpeedPolicyTest, ATightPointBehindAMildOneStillBindsTheReference) {
  // A mild constraint nearby must not hide a tight one a few metres behind
  // it: every constrained sample is folded, and the tightest answer wins.
  const MppiSpeedPolicyConfig config = clearanceLimiterConfig();
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  ExecutedHorizonClearance3D clearance = executedClearance(0.0, 5.0);
  clearance.constrained_samples.push_back(
      ConstrainedHorizonSample3D{.distance_m = 2.0, .clearance_m = 0.5});
  input.executed_horizon_clearance = clearance;

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  // The mild point alone admits 10 m/s; the tight one two metres on admits
  // 1 m/s and, braking at 4 m/s^2, sqrt(1 + 2 * 4 * 2) now.
  EXPECT_NEAR(result.clearance_limit_mps, std::sqrt(1.0 + 16.0), 1.0e-6);
  EXPECT_EQ(result.active_limiter, MppiSpeedLimiter::kClearance);
}

TEST(MppiSpeedPolicyTest, TheReferenceSpeedRisesNoFasterThanTheAirframeFollows) {
  MppiSpeedPolicyConfig config = clearanceLimiterConfig();
  config.reference_speed_rise_mps2 = 4.0;
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  input.previous_reference_speed_mps = 1.0;
  input.elapsed_since_previous_reference_s = 0.05;

  const MppiSpeedPolicyResult rising = evaluateMppiSpeedPolicy(config, input);
  EXPECT_TRUE(rising.reference_speed_rise_limited);
  EXPECT_DOUBLE_EQ(rising.unslewed_reference_speed_mps, 20.0);
  EXPECT_DOUBLE_EQ(rising.reference_speed_mps, 1.0 + 4.0 * 0.05);

  // A cap is always allowed to bite at once, so a fall is never slewed.
  input.previous_reference_speed_mps = 20.0;
  input.executed_horizon_clearance = executedClearance(0.0, 0.0);
  const MppiSpeedPolicyResult falling = evaluateMppiSpeedPolicy(config, input);
  EXPECT_FALSE(falling.reference_speed_rise_limited);
  EXPECT_DOUBLE_EQ(falling.reference_speed_mps, 1.0);
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

TEST(MppiSpeedPolicyTest, LocalStopBesideItsEndpointKeepsTheStraightDistanceAllowance) {
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 = 8.0;
  config.stopping_capability.reaction_latency_s = 0.1;
  allowHighSensorBrakingSpeed(config);
  // The vehicle projects onto the route's last sample (no station left) while
  // the endpoint itself is still half a metre away.
  std::array<RouteSample3D, 1U> route{};
  route.front().position = Point3{0.5, 0.0, 0.0};
  route.front().station_m = 3.5;
  MppiSpeedPolicyInput input;
  input.mission_goal = Point3{300.0, 0.0, 18.0};
  input.route = route;
  input.route_endpoint_remaining_m = 0.0;
  input.route_endpoint_semantics = RouteEndpointSemantics3D::kLocalStop;

  const MppiSpeedPolicyResult beside_endpoint = evaluateMppiSpeedPolicy(config, input);

  EXPECT_GT(beside_endpoint.route_endpoint_limit_mps, 1.0);
  EXPECT_TRUE(beside_endpoint.route_endpoint_stop_required);

  route.front().position = Point3{0.0, 0.0, 0.0};
  const MppiSpeedPolicyResult at_endpoint = evaluateMppiSpeedPolicy(config, input);
  EXPECT_DOUBLE_EQ(at_endpoint.route_endpoint_limit_mps, 0.0);
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
