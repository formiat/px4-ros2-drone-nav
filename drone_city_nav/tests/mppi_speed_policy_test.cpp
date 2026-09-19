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
  config.sensor_braking_contract.maximum_horizontal_acceleration_mps2 = 5.0;
  config.sensor_braking_contract.maximum_vertical_acceleration_mps2 = 5.0;
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

TEST(MppiSpeedPolicyTest, MemoryAnswersForAMotionAForwardSensorDoesNotFace) {
  // A pair that sees 60 degrees either side of the heading. Flying where it
  // looks the contract's range answers; flying sideways nothing looks, and
  // the contract is read with what memory has observed along the motion.
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  allowHighSensorBrakingSpeed(config);
  config.sensor_braking_contract.forward_horizontal_half_angle_rad = 1.0471975511965976;
  config.sensor_braking_contract.unobserved_speed_mps = 1.0;
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  input.state.vx = 3.0F;
  input.state.yaw = 0.0F;
  const MppiSpeedPolicyResult facing = evaluateMppiSpeedPolicy(config, input);
  EXPECT_GT(facing.sensor_braking_limit_mps, 10.0);

  // Nothing observed along it (r500: a wall 1 m away that no sensor had
  // looked at): nothing is admitted until the gaze has turned the vehicle.
  input.state.yaw = 1.5707964F;
  const MppiSpeedPolicyResult blind = evaluateMppiSpeedPolicy(config, input);
  EXPECT_DOUBLE_EQ(blind.sensor_braking_limit_mps, 0.0);
  EXPECT_EQ(blind.active_limiter, MppiSpeedLimiter::kSensorBraking);
  input.unfaced_observed_range_m = config.sensor_braking_contract.physical_margin_m;
  EXPECT_DOUBLE_EQ(evaluateMppiSpeedPolicy(config, input).sensor_braking_limit_mps,
                   0.0);

  // Space seen before is flown through without facing it, at what its depth
  // lets the vehicle stop within, and never faster than facing it would.
  input.unfaced_observed_range_m = 8.0;
  const double through_short =
      evaluateMppiSpeedPolicy(config, input).sensor_braking_limit_mps;
  input.unfaced_observed_range_m = 16.0;
  const double through_long =
      evaluateMppiSpeedPolicy(config, input).sensor_braking_limit_mps;
  EXPECT_GT(through_short, 0.0);
  EXPECT_GT(through_long, through_short);
  input.unfaced_observed_range_m = 1000.0;
  EXPECT_DOUBLE_EQ(evaluateMppiSpeedPolicy(config, input).sensor_braking_limit_mps,
                   facing.sensor_braking_limit_mps);
  input.unfaced_observed_range_m.reset();

  // A climb inside the cone of the sensors that look up and down is not the
  // pair's to face, whatever its horizontal remnant points at.
  config.sensor_braking_contract.forward_vertical_half_angle_rad = 0.9;
  config.sensor_braking_contract.vertical_detection_range_m = 30.0;
  config.sensor_braking_contract.vertical_cone_half_angle_rad = 0.5;
  config.sensor_braking_contract.vertical_physical_margin_m = 1.0;
  input.state.vx = 0.1F;
  input.state.vz = 2.0F;
  EXPECT_GT(evaluateMppiSpeedPolicy(config, input).sensor_braking_limit_mps, 1.0);
  input.state.vx = 3.0F;
  input.state.vz = 0.0F;
  config.sensor_braking_contract.forward_vertical_half_angle_rad = 1.5707963267948966;

  // A lidar that sees the whole sphere does not care where the vehicle looks.
  config.sensor_braking_contract.forward_horizontal_half_angle_rad = 3.141592653589793;
  EXPECT_GT(evaluateMppiSpeedPolicy(config, input).sensor_braking_limit_mps, 10.0);
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
  EXPECT_NEAR(result.blocked_route_limit_mps,
              stoppingLimitedSpeed(5.0, 0.0, config.stopping_capability,
                                   config.sensor_braking_contract, 0.0),
              1.0e-9);
  EXPECT_LT(result.blocked_route_limit_mps, std::sqrt(2.0 * 4.0 * 5.0));
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

TEST(MppiSpeedPolicyTest, TheRouteClearanceBoundsTheSpeedTheHorizonCannotYetSee) {
  // A vehicle at rest publishes a horizon that ends where it can stop, so the
  // horizon sees only the space beside the vehicle and admits cruise. The
  // route is the geometry the vehicle is committed to, and the tight spot it
  // enters bounds the reference before the vehicle accelerates into it.
  const MppiSpeedPolicyConfig config = clearanceLimiterConfig();
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  input.executed_horizon_clearance = ExecutedHorizonClearance3D{.available = true};

  const MppiSpeedPolicyResult horizon_only = evaluateMppiSpeedPolicy(config, input);
  EXPECT_NE(horizon_only.active_limiter, MppiSpeedLimiter::kRouteClearance);
  EXPECT_DOUBLE_EQ(horizon_only.reference_speed_mps, 20.0);

  // A 1 m clearance 10 m along the route: the tube law admits 2 m/s there, and
  // the stopping law decides what may be carried on the way to it.
  input.route_clearance = executedClearance(10.0, 1.0);
  const MppiSpeedPolicyResult ahead = evaluateMppiSpeedPolicy(config, input);
  EXPECT_EQ(ahead.active_limiter, MppiSpeedLimiter::kRouteClearance);
  EXPECT_STREQ(mppiSpeedLimiterName(ahead.active_limiter), "route_clearance");
  EXPECT_NEAR(ahead.route_clearance_limit_mps,
              stoppingLimitedSpeed(10.0, 1.0 / 0.5, config.stopping_capability,
                                   config.sensor_braking_contract, 0.0),
              1.0e-9);
  EXPECT_DOUBLE_EQ(ahead.reference_speed_mps, ahead.route_clearance_limit_mps);

  // Standing at the tight point, the tube law alone answers.
  input.route_clearance = executedClearance(0.0, 1.0);
  const MppiSpeedPolicyResult beside = evaluateMppiSpeedPolicy(config, input);
  EXPECT_EQ(beside.active_limiter, MppiSpeedLimiter::kRouteClearance);
  EXPECT_NEAR(beside.route_clearance_limit_mps, 1.0 / 0.5, 1.0e-9);

  // The horizon keeps its own authority: the tighter of the two answers wins.
  input.executed_horizon_clearance = executedClearance(0.0, 0.25);
  const MppiSpeedPolicyResult both = evaluateMppiSpeedPolicy(config, input);
  EXPECT_EQ(both.active_limiter, MppiSpeedLimiter::kClearance);
  EXPECT_DOUBLE_EQ(both.reference_speed_mps, 1.0);
}

TEST(MppiSpeedPolicyTest, TheLagBehindAFallingReferenceIsLatencyInTheDistanceLaws) {
  // The same tight spot 10 m ahead and the same block 8 m ahead: a vehicle
  // that follows a falling reference half a second late is owed half a
  // second of travel more, and the sensor-braking limit is not touched.
  MppiSpeedPolicyConfig config = clearanceLimiterConfig();
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  input.executed_horizon_clearance = executedClearance(10.0, 1.0);
  input.blocked_route_remaining_m = 8.0;
  const MppiSpeedPolicyResult prompt = evaluateMppiSpeedPolicy(config, input);
  config.reference_tracking_lag_s = 0.5;
  const MppiSpeedPolicyResult late = evaluateMppiSpeedPolicy(config, input);
  StoppingCapability delayed = config.stopping_capability;
  delayed.reaction_latency_s += 0.5;
  EXPECT_NEAR(
      late.clearance_limit_mps,
      stoppingLimitedSpeed(10.0, 2.0, delayed, config.sensor_braking_contract, 0.0),
      1.0e-9);
  EXPECT_LT(late.clearance_limit_mps, prompt.clearance_limit_mps);
  EXPECT_LT(late.blocked_route_limit_mps, prompt.blocked_route_limit_mps);
  EXPECT_DOUBLE_EQ(late.sensor_braking_limit_mps, prompt.sensor_braking_limit_mps);
}

TEST(MppiSpeedPolicyTest, TheAgeOfTheClearanceEvidenceIsLatencyInTheStoppingLaw) {
  // The same tight spot 10 m ahead: read off evidence half a second old, the
  // free path to it is owed half a second of travel more than the reaction
  // latency alone, on the horizon and on the route alike.
  const MppiSpeedPolicyConfig config = clearanceLimiterConfig();
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  input.executed_horizon_clearance = executedClearance(10.0, 1.0);
  const MppiSpeedPolicyResult fresh = evaluateMppiSpeedPolicy(config, input);
  input.esdf_evidence_age_s = 0.5;
  const MppiSpeedPolicyResult aged = evaluateMppiSpeedPolicy(config, input);
  StoppingCapability delayed = config.stopping_capability;
  delayed.reaction_latency_s += 0.5;
  EXPECT_NEAR(fresh.clearance_limit_mps,
              stoppingLimitedSpeed(10.0, 2.0, config.stopping_capability,
                                   config.sensor_braking_contract, 0.0),
              1.0e-9);
  EXPECT_NEAR(
      aged.clearance_limit_mps,
      stoppingLimitedSpeed(10.0, 2.0, delayed, config.sensor_braking_contract, 0.0),
      1.0e-9);
  EXPECT_LT(aged.clearance_limit_mps, fresh.clearance_limit_mps);

  input.executed_horizon_clearance.reset();
  input.route_clearance = executedClearance(10.0, 1.0);
  const MppiSpeedPolicyResult aged_route = evaluateMppiSpeedPolicy(config, input);
  EXPECT_NEAR(
      aged_route.route_clearance_limit_mps,
      stoppingLimitedSpeed(10.0, 2.0, delayed, config.sensor_braking_contract, 0.0),
      1.0e-9);
  // Beside the tight spot the tube law alone answers, whatever the age.
  input.route_clearance = executedClearance(0.0, 1.0);
  EXPECT_NEAR(evaluateMppiSpeedPolicy(config, input).route_clearance_limit_mps, 2.0,
              1.0e-9);
}

TEST(MppiSpeedPolicyTest,
     TheObservedRangeAlongTheMotionBoundsTheSpeedLikeTheSensorRange) {
  // The sensor-braking contract bounds the speed by the range the sensor is
  // guaranteed to have seen. Where the motion under execution enters space the
  // evidence has not observed, the same contract is read with the range the
  // evidence actually covers along that motion.
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  config.sensor_braking_contract.guaranteed_detection_range_m = 30.0;
  config.sensor_braking_contract.maximum_evidence_age_s = 0.25;
  config.sensor_braking_contract.physical_margin_m = 3.0;
  config.clearance_minimum_progress_speed_mps = 1.0;
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  ExecutedHorizonClearance3D clearance;
  clearance.available = true;

  clearance.unobserved_distance_m = 10.0;
  input.executed_horizon_clearance = clearance;
  const MppiSpeedPolicyResult near = evaluateMppiSpeedPolicy(config, input);
  SensorBrakingContract3D observed = config.sensor_braking_contract;
  observed.guaranteed_detection_range_m = 10.0;
  const double expected_mps = sensorBrakingMaximumSpeedMps(
      observed, config.stopping_capability, config.absolute_speed_limit_mps, Vec3{});
  EXPECT_EQ(near.active_limiter, MppiSpeedLimiter::kUnobservedFrontier);
  EXPECT_STREQ(mppiSpeedLimiterName(near.active_limiter), "unobserved_frontier");
  EXPECT_NEAR(near.unobserved_frontier_limit_mps, expected_mps, 1.0e-9);
  EXPECT_LT(near.unobserved_frontier_limit_mps, near.sensor_braking_limit_mps);
  EXPECT_DOUBLE_EQ(near.reference_speed_mps, near.unobserved_frontier_limit_mps);

  // Inside the physical margin the contract admits nothing; unobserved space
  // stays enterable at the progress floor.
  clearance.unobserved_distance_m = 2.0;
  input.executed_horizon_clearance = clearance;
  const MppiSpeedPolicyResult touching = evaluateMppiSpeedPolicy(config, input);
  EXPECT_EQ(touching.active_limiter, MppiSpeedLimiter::kUnobservedFrontier);
  EXPECT_DOUBLE_EQ(touching.unobserved_frontier_limit_mps, 1.0);
  EXPECT_DOUBLE_EQ(touching.reference_speed_mps, 1.0);

  // Beyond the guaranteed range the observed range adds nothing.
  clearance.unobserved_distance_m = 60.0;
  input.executed_horizon_clearance = clearance;
  const MppiSpeedPolicyResult far = evaluateMppiSpeedPolicy(config, input);
  EXPECT_NE(far.active_limiter, MppiSpeedLimiter::kUnobservedFrontier);
  EXPECT_GE(far.unobserved_frontier_limit_mps, far.sensor_braking_limit_mps);

  clearance.unobserved_distance_m.reset();
  input.executed_horizon_clearance = clearance;
  const MppiSpeedPolicyResult observed_throughout =
      evaluateMppiSpeedPolicy(config, input);
  EXPECT_TRUE(std::isinf(observed_throughout.unobserved_frontier_limit_mps));
}

TEST(MppiSpeedPolicyTest, TheRouteAheadBoundsTheSpeedWhereTheHorizonSeesNoFrontier) {
  // The executed horizon ends where the vehicle can rest, so it reports no
  // frontier while the vehicle can still stop before one; the route ahead
  // does, and the contract is read with the shorter of the two ranges.
  MppiSpeedPolicyConfig config;
  config.cruise_speed_mps = 20.0;
  config.absolute_speed_limit_mps = 20.0;
  config.sensor_braking_contract.guaranteed_detection_range_m = 30.0;
  config.sensor_braking_contract.maximum_evidence_age_s = 0.25;
  config.sensor_braking_contract.physical_margin_m = 3.0;
  config.clearance_minimum_progress_speed_mps = 1.0;
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  ExecutedHorizonClearance3D clearance;
  clearance.available = true;
  input.executed_horizon_clearance = clearance;

  input.route_observed_range_m = 10.0;
  const MppiSpeedPolicyResult route_only = evaluateMppiSpeedPolicy(config, input);
  SensorBrakingContract3D observed = config.sensor_braking_contract;
  observed.guaranteed_detection_range_m = 10.0;
  const double expected_mps = sensorBrakingMaximumSpeedMps(
      observed, config.stopping_capability, config.absolute_speed_limit_mps, Vec3{});
  EXPECT_EQ(route_only.active_limiter, MppiSpeedLimiter::kUnobservedFrontier);
  EXPECT_NEAR(route_only.unobserved_frontier_limit_mps, expected_mps, 1.0e-9);
  EXPECT_DOUBLE_EQ(route_only.unobserved_frontier_range_m, 10.0);

  clearance.unobserved_distance_m = 6.0;
  input.executed_horizon_clearance = clearance;
  const MppiSpeedPolicyResult nearer_horizon = evaluateMppiSpeedPolicy(config, input);
  EXPECT_DOUBLE_EQ(nearer_horizon.unobserved_frontier_range_m, 6.0);
  EXPECT_LT(nearer_horizon.unobserved_frontier_limit_mps,
            route_only.unobserved_frontier_limit_mps);

  input.route_observed_range_m.reset();
  input.executed_horizon_clearance.reset();
  const MppiSpeedPolicyResult unbounded = evaluateMppiSpeedPolicy(config, input);
  EXPECT_TRUE(std::isinf(unbounded.unobserved_frontier_limit_mps));
  EXPECT_TRUE(std::isinf(unbounded.unobserved_frontier_range_m));
}

TEST(MppiSpeedPolicyTest, ATightPointFarAheadOnlyHasToBeReachedSlowly) {
  const MppiSpeedPolicyConfig config = clearanceLimiterConfig();
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;
  // The tube admits 0.5 / 0.5 = 1 m/s where the horizon grazes an obstacle
  // fifteen metres ahead, which is also the floor. The vehicle may still be
  // fast now: less than the 11 m/s of an instantaneous 4 m/s^2, by the ramp
  // the jerk limit takes to get there.
  input.executed_horizon_clearance = executedClearance(15.0, 0.5);

  const MppiSpeedPolicyResult result = evaluateMppiSpeedPolicy(config, input);

  EXPECT_LT(result.clearance_limit_mps, std::sqrt(1.0 + 2.0 * 4.0 * 15.0));
  EXPECT_GT(result.clearance_limit_mps, 7.0);
}

TEST(MppiSpeedPolicyTest, TheStoppingLawHoldsForAVehicleStillAccelerating) {
  // Whatever the distance, and at the few metres a camera resolves as much as
  // at a lidar's fourteen: a vehicle at the admitted speed and the measured
  // forward acceleration, integrated under the jerk limit, is down to the
  // terminal speed within the distance, and one 5 percent faster is not.
  const MppiSpeedPolicyConfig config = clearanceLimiterConfig();
  const double jerk_mps3 = config.sensor_braking_contract.maximum_control_jerk_mps3;
  const double deceleration_mps2 =
      config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2;
  const auto slowdownDistanceM = [&](const double speed_mps,
                                     const double terminal_speed_mps,
                                     const double forward_acceleration_mps2) {
    constexpr double kStepS{1.0e-4};
    double acceleration_mps2 = forward_acceleration_mps2;
    double speed = speed_mps;
    double distance_m{0.0};
    for (double elapsed_s = 0.0; speed > terminal_speed_mps; elapsed_s += kStepS) {
      if (elapsed_s >= config.stopping_capability.reaction_latency_s) {
        acceleration_mps2 =
            std::max(-deceleration_mps2, acceleration_mps2 - jerk_mps3 * kStepS);
      }
      distance_m += speed * kStepS;
      speed += acceleration_mps2 * kStepS;
    }
    return distance_m;
  };
  for (const double distance_m : {3.0, 4.3, 5.7, 14.0}) {
    for (const double terminal_speed_mps : {0.0, 1.0, 3.0}) {
      for (const double forward_acceleration_mps2 : {0.0, 2.0, 4.0}) {
        const double admitted_mps = stoppingLimitedSpeed(
            distance_m, terminal_speed_mps, config.stopping_capability,
            config.sensor_braking_contract, forward_acceleration_mps2);
        ASSERT_GE(admitted_mps, terminal_speed_mps);
        if (admitted_mps > terminal_speed_mps) {
          EXPECT_LE(slowdownDistanceM(admitted_mps, terminal_speed_mps,
                                      forward_acceleration_mps2),
                    distance_m + 1.0e-2);
          EXPECT_GT(slowdownDistanceM(1.05 * admitted_mps, terminal_speed_mps,
                                      forward_acceleration_mps2),
                    distance_m);
        }
      }
    }
  }
}

TEST(MppiSpeedPolicyTest, TheBodyClearanceBoundsTheProgressFloor) {
  MppiSpeedPolicyConfig config = clearanceLimiterConfig();
  config.clearance_minimum_progress_speed_mps = 3.0;
  MppiSpeedPolicyInput input;
  input.terminal_goal_limit_enabled = false;

  // The envelope stands in evidence, the body keeps 0.1 m: the floor would
  // admit 3 m/s, the body's tube 0.2 m/s, and a contact is left at 1 m/s.
  ExecutedHorizonClearance3D grazing = executedClearance(0.0, 0.0);
  grazing.constrained_samples.front().body_clearance_m = 0.1;
  input.executed_horizon_clearance = grazing;
  EXPECT_NEAR(evaluateMppiSpeedPolicy(config, input).clearance_limit_mps, 1.0, 1.0e-6);

  // The body keeps 2 m: its tube admits 4 m/s and the floor stands.
  grazing.constrained_samples.front().body_clearance_m = 2.0;
  input.executed_horizon_clearance = grazing;
  EXPECT_NEAR(evaluateMppiSpeedPolicy(config, input).clearance_limit_mps, 3.0, 1.0e-6);

  // Unmeasured, the body's clearance bounds nothing.
  input.executed_horizon_clearance = executedClearance(0.0, 0.0);
  EXPECT_NEAR(evaluateMppiSpeedPolicy(config, input).clearance_limit_mps, 3.0, 1.0e-6);
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
  // 1 m/s and what the stopping law carries over two metres to it.
  EXPECT_NEAR(result.clearance_limit_mps,
              stoppingLimitedSpeed(2.0, 1.0, config.stopping_capability,
                                   config.sensor_braking_contract, 0.0),
              1.0e-9);
  EXPECT_LT(result.clearance_limit_mps, std::sqrt(1.0 + 16.0));
  EXPECT_GT(result.clearance_limit_mps, 1.0);
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

  EXPECT_NEAR(approaching.route_endpoint_limit_mps,
              stoppingLimitedSpeed(6.0, 0.0, config.stopping_capability,
                                   config.sensor_braking_contract, 0.0),
              1.0e-9);
  EXPECT_LT(approaching.route_endpoint_limit_mps, 9.03);
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
