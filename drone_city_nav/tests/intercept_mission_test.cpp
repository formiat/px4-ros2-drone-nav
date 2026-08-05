#include "drone_city_nav/intercept_mission.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] TimedVehicleState state(const Point3 position, const Vec3 velocity,
                                      const std::int64_t stamp_ns) {
  return TimedVehicleState{.position = position,
                           .velocity = velocity,
                           .stamp_ns = stamp_ns,
                           .position_valid = true,
                           .velocity_valid = true,
                           .armed = true,
                           .airborne = true,
                           .navigation_ready = true};
}

TEST(InterceptMissionEvaluatorTest, DetectsCaptureBetweenSamples) {
  InterceptMissionEvaluator evaluator{Point3{100.0, 0.0, 10.0}};
  EXPECT_EQ(evaluator
                .update(state(Point3{-10.0, 0.0, 10.0}, {}, 1),
                        state(Point3{0.0, 0.0, 10.0}, {}, 1))
                .outcome,
            InterceptMissionOutcome::kRunning);
  const InterceptMissionUpdate update = evaluator.update(
      state(Point3{10.0, 0.0, 10.0}, {}, 2), state(Point3{0.0, 0.0, 10.0}, {}, 2));
  EXPECT_TRUE(update.newly_terminal);
  EXPECT_EQ(update.outcome, InterceptMissionOutcome::kIntercepted);
  EXPECT_NEAR(update.separation_m, 0.0, 1.0e-9);
}

TEST(InterceptMissionEvaluatorTest, GoalTriggersAtFirstEntry) {
  InterceptMissionEvaluator evaluator{
      Point3{100.0, 0.0, 10.0},
      InterceptMissionConfig{.capture_radius_m = 5.0, .evader_goal_radius_m = 2.0}};
  const TimedVehicleState interceptor =
      state(Point3{0.0, 0.0, 10.0}, {}, 1'000'000'000LL);
  const InterceptMissionUpdate update =
      evaluator.update(interceptor, state(Point3{100.0, 0.0, 10.0},
                                          Vec3{10.0, 0.0, 0.0}, 1'000'000'000LL));
  EXPECT_TRUE(update.newly_terminal);
  EXPECT_EQ(update.outcome, InterceptMissionOutcome::kEvaderReachedGoal);
}

TEST(InterceptMissionEvaluatorTest, DoesNotCaptureBeforeBothVehiclesAreAirborne) {
  InterceptMissionEvaluator evaluator{Point3{100.0, 0.0, 10.0}};
  TimedVehicleState interceptor = state(Point3{0.0, 0.0, 0.0}, {}, 1);
  interceptor.airborne = false;
  TimedVehicleState evader = state(Point3{1.0, 0.0, 0.0}, {}, 1);
  evader.airborne = false;
  EXPECT_EQ(evaluator.update(interceptor, evader).outcome,
            InterceptMissionOutcome::kRunning);
}

TEST(InterceptMissionEvaluatorTest, FirstTerminalOutcomeRemainsLatched) {
  InterceptMissionEvaluator evaluator{
      Point3{100.0, 0.0, 10.0},
      InterceptMissionConfig{.capture_radius_m = 5.0, .evader_goal_radius_m = 2.0}};
  EXPECT_EQ(evaluator
                .update(state(Point3{50.0, 0.0, 10.0}, {}, 1'000'000'000LL),
                        state(Point3{100.0, 0.0, 10.0}, {}, 1'000'000'000LL))
                .outcome,
            InterceptMissionOutcome::kEvaderReachedGoal);

  const InterceptMissionUpdate late_interceptor =
      evaluator.update(state(Point3{100.0, 0.0, 10.0}, {}, 3'200'000'000LL),
                       state(Point3{100.0, 0.0, 10.0}, {}, 3'200'000'000LL));
  EXPECT_FALSE(late_interceptor.newly_terminal);
  EXPECT_TRUE(late_interceptor.newly_captured);
  EXPECT_TRUE(late_interceptor.capture_detected);
  EXPECT_EQ(late_interceptor.outcome, InterceptMissionOutcome::kEvaderReachedGoal);
}

TEST(InterceptorHoldConfirmationTest, RequiresStablePositionAndSpeed) {
  InterceptorHoldConfirmation confirmation{
      Point3{10.0, 20.0, 15.0}, InterceptorHoldConfig{.position_tolerance_m = 2.0,
                                                      .maximum_speed_mps = 0.8,
                                                      .confirmation_duration_s = 1.0}};
  EXPECT_FALSE(
      confirmation
          .update(state(Point3{10.0, 20.0, 15.0}, Vec3{2.0, 0.0, 0.0}, 1'000'000'000LL))
          .confirmed);
  EXPECT_FALSE(
      confirmation
          .update(state(Point3{10.5, 20.0, 15.0}, Vec3{0.2, 0.0, 0.0}, 2'000'000'000LL))
          .confirmed);
  const InterceptorHoldUpdate confirmed = confirmation.update(
      state(Point3{10.4, 20.0, 15.0}, Vec3{0.1, 0.0, 0.0}, 3'100'000'000LL));
  EXPECT_TRUE(confirmed.confirmed);
  EXPECT_TRUE(confirmed.newly_confirmed);
  EXPECT_NEAR(confirmed.position_error_m, 0.4, 1.0e-9);
  EXPECT_NEAR(confirmed.speed_mps, 0.1, 1.0e-9);
}

TEST(InterceptMissionReadinessTest, RequiresBothWorldsAndFirstTargetTrack) {
  InterceptMissionReadiness readiness{
      .interceptor_navigation_ready = true,
      .evader_navigation_ready = true,
      .interceptor_world_ready = true,
      .evader_world_ready = true,
      .target_track_ready = false,
  };

  EXPECT_FALSE(interceptMissionReady(readiness));
  readiness.target_track_ready = true;
  EXPECT_TRUE(interceptMissionReady(readiness));
}

TEST(InterceptMissionReadinessTest, RejectsAnyMissingPlannerReadiness) {
  InterceptMissionReadiness readiness{
      .interceptor_navigation_ready = true,
      .evader_navigation_ready = true,
      .interceptor_world_ready = false,
      .evader_world_ready = true,
      .target_track_ready = true,
  };

  EXPECT_FALSE(interceptMissionReady(readiness));
  readiness.interceptor_world_ready = true;
  readiness.evader_world_ready = false;
  EXPECT_FALSE(interceptMissionReady(readiness));
}

} // namespace
} // namespace drone_city_nav
