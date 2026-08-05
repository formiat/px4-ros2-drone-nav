#include "drone_city_nav/intercept_guidance.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] TimedVehicleState state(const Point3 position, const Vec3 velocity,
                                      const std::int64_t stamp_ns) {
  return TimedVehicleState{
      .position = position,
      .velocity = velocity,
      .stamp_ns = stamp_ns,
      .position_valid = true,
      .velocity_valid = true,
      .armed = true,
      .airborne = true,
      .navigation_ready = true,
  };
}

TEST(InterceptGuidance, SolvesAnalyticInterceptAndCompensatesMeasurementAge) {
  InterceptGuidance guidance;
  const TimedVehicleState interceptor =
      state(Point3{0.0, 20.0, 5.0}, Vec3{}, 800'000'000LL);
  const TimedVehicleState target =
      state(Point3{10.0, 20.0, 5.0}, Vec3{10.0, 0.0, 0.0}, 800'000'000LL);

  const InterceptGuidanceResult result =
      guidance.update(interceptor, target, 1'000'000'000LL);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.mode, InterceptGuidanceMode::kAnalyticIntercept);
  EXPECT_NEAR(result.analytic_intercept_time_s, 1.2, 1.0e-9);
  EXPECT_NEAR(result.prediction_horizon_s, 1.2, 1.0e-9);
  EXPECT_NEAR(result.prediction_age_s, 0.2, 1.0e-9);
  EXPECT_NEAR(result.predicted_position.x, 24.0, 1.0e-9);
}

TEST(InterceptGuidance, UsesAnalyticMeetingPointWhenInterceptorIsAheadInCorridor) {
  InterceptGuidance guidance;
  const TimedVehicleState interceptor =
      state(Point3{6.0, 0.0, 0.0}, Vec3{}, 1'000'000'000LL);
  const TimedVehicleState target =
      state(Point3{}, Vec3{10.0, 0.0, 0.0}, 1'000'000'000LL);

  const InterceptGuidanceResult result =
      guidance.update(interceptor, target, 1'100'000'000LL);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.mode, InterceptGuidanceMode::kAheadIntercept);
  EXPECT_NEAR(result.prediction_horizon_s, 1.0 / 6.0, 1.0e-9);
  EXPECT_NEAR(result.ahead_m, 5.0, 1.0e-9);
  EXPECT_NEAR(result.cross_track_m, 0.0, 1.0e-9);
  EXPECT_NEAR(result.predicted_position.x, 8.0 / 3.0, 1.0e-9);
}

TEST(InterceptGuidance, AppliesModeHysteresisAndSmoothsHorizonChanges) {
  InterceptGuidance guidance;
  const TimedVehicleState target =
      state(Point3{}, Vec3{10.0, 0.0, 0.0}, 1'000'000'000LL);

  const auto far = guidance.update(
      state(Point3{-10.0, 0.0, 0.0}, Vec3{}, 1'000'000'000LL), target, 1'000'000'000LL);
  const auto entered = guidance.update(
      state(Point3{6.0, 0.0, 0.0}, Vec3{}, 1'100'000'000LL), target, 1'100'000'000LL);
  const auto retained = guidance.update(
      state(Point3{8.0, 18.0, 0.0}, Vec3{}, 1'200'000'000LL), target, 1'200'000'000LL);
  const auto exited = guidance.update(
      state(Point3{-1.0, 0.0, 0.0}, Vec3{}, 1'500'000'000LL), target, 1'500'000'000LL);

  EXPECT_EQ(far.mode, InterceptGuidanceMode::kAnalyticIntercept);
  EXPECT_DOUBLE_EQ(far.prediction_horizon_s, 1.0);
  EXPECT_EQ(entered.mode, InterceptGuidanceMode::kAheadIntercept);
  EXPECT_GT(entered.prediction_horizon_s, entered.analytic_intercept_time_s);
  EXPECT_LT(entered.prediction_horizon_s, far.prediction_horizon_s);
  EXPECT_EQ(retained.mode, InterceptGuidanceMode::kAheadIntercept);
  EXPECT_EQ(exited.mode, InterceptGuidanceMode::kAnalyticIntercept);
  EXPECT_GT(exited.prediction_horizon_s, exited.analytic_intercept_time_s);
  EXPECT_LT(exited.prediction_horizon_s, far.prediction_horizon_s);
}

TEST(InterceptGuidance, InvalidVelocityFallsBackToFiniteDirectObjective) {
  InterceptGuidance guidance;
  TimedVehicleState target =
      state(Point3{10.0, 2.0, 3.0}, Vec3{1.0, 0.0, 0.0}, 900'000'000LL);
  target.velocity_valid = false;
  target.velocity.x = std::numeric_limits<double>::quiet_NaN();

  const InterceptGuidanceResult result =
      guidance.update(state(Point3{}, Vec3{}, 900'000'000LL), target, 1'000'000'000LL);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.mode, InterceptGuidanceMode::kDirect);
  EXPECT_DOUBLE_EQ(result.observed_velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(result.predicted_position.x, target.position.x);
}

TEST(InterceptGuidance, FallsBackToCurrentPositionAtLowTargetSpeed) {
  InterceptGuidance guidance;
  const TimedVehicleState interceptor =
      state(Point3{20.0, 0.0, 0.0}, Vec3{}, 1'000'000'000LL);
  const TimedVehicleState target =
      state(Point3{10.0, 2.0, 3.0}, Vec3{0.2, 0.0, 0.0}, 900'000'000LL);

  const InterceptGuidanceResult result =
      guidance.update(interceptor, target, 1'000'000'000LL);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.mode, InterceptGuidanceMode::kDirect);
  EXPECT_DOUBLE_EQ(result.prediction_horizon_s, 0.0);
  EXPECT_NEAR(result.predicted_position.x, 10.02, 1.0e-9);
  EXPECT_DOUBLE_EQ(result.predicted_position.y, target.position.y);
  EXPECT_DOUBLE_EQ(result.predicted_position.z, target.position.z);
}

TEST(InterceptGuidance, LooksFarEnoughAheadToMeetDistantTarget) {
  InterceptGuidance guidance;
  const TimedVehicleState interceptor =
      state(Point3{0.0, 0.0, 10.0}, Vec3{}, 1'000'000'000LL);
  const TimedVehicleState target =
      state(Point3{100.0, 0.0, 10.0}, Vec3{10.0, 0.0, 0.0}, 1'000'000'000LL);

  const InterceptGuidanceResult result =
      guidance.update(interceptor, target, 1'000'000'000LL);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.mode, InterceptGuidanceMode::kAnalyticIntercept);
  EXPECT_NEAR(result.analytic_intercept_time_s, 10.0, 1.0e-9);
  EXPECT_NEAR(result.predicted_position.x, 200.0, 1.0e-9);
}

TEST(InterceptGuidance, RejectsInvalidTargetPosition) {
  InterceptGuidance guidance;
  TimedVehicleState target;
  target.position_valid = false;

  EXPECT_FALSE(guidance.update(TimedVehicleState{}, target, 1'000'000'000LL).valid);
}

} // namespace
} // namespace drone_city_nav
