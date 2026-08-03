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

TEST(InterceptMissionEvaluatorTest, GoalRequiresStableStoppedEvader) {
  InterceptMissionEvaluator evaluator{
      Point3{100.0, 0.0, 10.0},
      InterceptMissionConfig{.capture_radius_m = 5.0,
                             .evader_goal_radius_m = 2.0,
                             .evader_goal_stop_speed_mps = 0.8,
                             .evader_goal_hold_s = 2.0}};
  const TimedVehicleState interceptor =
      state(Point3{0.0, 0.0, 10.0}, {}, 1'000'000'000LL);
  EXPECT_EQ(
      evaluator
          .update(interceptor, state(Point3{100.0, 0.0, 10.0}, {}, 1'000'000'000LL))
          .outcome,
      InterceptMissionOutcome::kRunning);
  const InterceptMissionUpdate update =
      evaluator.update(state(Point3{0.0, 0.0, 10.0}, {}, 3'100'000'000LL),
                       state(Point3{100.0, 0.0, 10.0}, {}, 3'100'000'000LL));
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

} // namespace
} // namespace drone_city_nav
