#include "drone_city_nav/intercept_mission.hpp"

#include <gtest/gtest.h>

#include <array>

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
  EXPECT_NEAR(update.current_separation_m, 10.0, 1.0e-9);
  EXPECT_NEAR(update.interpolation_fraction, 0.5, 1.0e-9);
}

TEST(InterceptMissionEvaluatorTest, DoesNotSweepAcrossTelemetryGapAfterReset) {
  InterceptMissionEvaluator evaluator{Point3{100.0, 0.0, 10.0}};
  EXPECT_EQ(evaluator
                .update(state(Point3{-10.0, 0.0, 10.0}, {}, 1),
                        state(Point3{0.0, 0.0, 10.0}, {}, 1))
                .outcome,
            InterceptMissionOutcome::kRunning);

  evaluator.resetTemporalContinuity();
  const InterceptMissionUpdate update = evaluator.update(
      state(Point3{10.0, 0.0, 10.0}, {}, 2), state(Point3{0.0, 0.0, 10.0}, {}, 2));

  EXPECT_EQ(update.outcome, InterceptMissionOutcome::kRunning);
  EXPECT_FALSE(update.newly_captured);
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

TEST(MultiInterceptMissionEvaluatorTest, SelectsClosestCapturingInterceptor) {
  MultiInterceptMissionEvaluator evaluator{Point3{100.0, 0.0, 10.0}, 3U};
  const std::array interceptors{
      state(Point3{20.0, 0.0, 10.0}, {}, 1),
      state(Point3{3.0, 0.0, 10.0}, {}, 1),
      state(Point3{4.0, 0.0, 10.0}, {}, 1),
  };

  const MultiInterceptMissionUpdate update =
      evaluator.update(interceptors, state(Point3{0.0, 0.0, 10.0}, {}, 1));

  EXPECT_EQ(update.capturing_interceptor_index, std::optional<std::size_t>{1U});
  EXPECT_EQ(update.outcome, InterceptMissionOutcome::kIntercepted);
  EXPECT_NEAR(update.separation_m, 3.0, 1.0e-9);
}

TEST(MultiInterceptMissionEvaluatorTest, IgnoresDisarmedNearbyInterceptor) {
  MultiInterceptMissionEvaluator evaluator{Point3{100.0, 0.0, 10.0}, 2U};
  std::array interceptors{
      state(Point3{1.0, 0.0, 10.0}, {}, 1),
      state(Point3{20.0, 0.0, 10.0}, {}, 1),
  };
  interceptors[0].armed = false;

  const MultiInterceptMissionUpdate update =
      evaluator.update(interceptors, state(Point3{}, {}, 1));

  EXPECT_FALSE(update.capturing_interceptor_index.has_value());
  EXPECT_EQ(update.outcome, InterceptMissionOutcome::kRunning);
}

TEST(MultiInterceptMissionEvaluatorTest, CaptureWinsOverGoalAtSameSample) {
  MultiInterceptMissionEvaluator evaluator{
      Point3{100.0, 0.0, 10.0}, 2U,
      InterceptMissionConfig{.capture_radius_m = 5.0, .evader_goal_radius_m = 2.0}};
  const std::array interceptors{
      state(Point3{104.0, 0.0, 10.0}, {}, 1),
      state(Point3{}, {}, 1),
  };

  const MultiInterceptMissionUpdate update =
      evaluator.update(interceptors, state(Point3{100.0, 0.0, 10.0}, {}, 1));

  EXPECT_TRUE(update.newly_terminal);
  EXPECT_EQ(update.outcome, InterceptMissionOutcome::kIntercepted);
  EXPECT_EQ(update.capturing_interceptor_index, std::optional<std::size_t>{0U});
  EXPECT_NEAR(update.separation_m, 4.0, 1.0e-9);
  EXPECT_NEAR(update.current_separation_m, 4.0, 1.0e-9);
  EXPECT_NEAR(update.interpolation_fraction, 1.0, 1.0e-9);
}

TEST(MultiInterceptMissionEvaluatorTest, DetectsCaptureBySweptRelativeMotion) {
  MultiInterceptMissionEvaluator evaluator{Point3{100.0, 0.0, 10.0}, 2U};
  std::array interceptors{
      state(Point3{-10.0, 0.0, 10.0}, {}, 1),
      state(Point3{40.0, 0.0, 10.0}, {}, 1),
  };
  const TimedVehicleState evader = state(Point3{0.0, 0.0, 10.0}, {}, 1);
  EXPECT_EQ(evaluator.update(interceptors, evader).outcome,
            InterceptMissionOutcome::kRunning);
  interceptors[0] = state(Point3{10.0, 0.0, 10.0}, {}, 2);

  const MultiInterceptMissionUpdate update =
      evaluator.update(interceptors, state(Point3{0.0, 0.0, 10.0}, {}, 2));

  EXPECT_EQ(update.outcome, InterceptMissionOutcome::kIntercepted);
  EXPECT_EQ(update.capturing_interceptor_index, std::optional<std::size_t>{0U});
  EXPECT_NEAR(update.separation_m, 0.0, 1.0e-9);
  EXPECT_NEAR(update.current_separation_m, 10.0, 1.0e-9);
  EXPECT_NEAR(update.interpolation_fraction, 0.5, 1.0e-9);
}

TEST(MultiInterceptMissionEvaluatorTest, ResolvesEqualCaptureByLowestIndex) {
  MultiInterceptMissionEvaluator evaluator{Point3{100.0, 0.0, 10.0}, 3U};
  const std::array interceptors{
      state(Point3{-3.0, 0.0, 10.0}, {}, 1),
      state(Point3{3.0, 0.0, 10.0}, {}, 1),
      state(Point3{20.0, 0.0, 10.0}, {}, 1),
  };

  const MultiInterceptMissionUpdate update =
      evaluator.update(interceptors, state(Point3{0.0, 0.0, 10.0}, {}, 1));

  EXPECT_EQ(update.capturing_interceptor_index, std::optional<std::size_t>{0U});
}

TEST(MultiInterceptMissionEvaluatorTest, LateCapturePreservesGoalOutcome) {
  MultiInterceptMissionEvaluator evaluator{
      Point3{100.0, 0.0, 10.0}, 2U,
      InterceptMissionConfig{.capture_radius_m = 5.0, .evader_goal_radius_m = 2.0}};
  std::array interceptors{
      state(Point3{50.0, 0.0, 10.0}, {}, 1),
      state(Point3{}, {}, 1),
  };
  EXPECT_EQ(
      evaluator.update(interceptors, state(Point3{100.0, 0.0, 10.0}, {}, 1)).outcome,
      InterceptMissionOutcome::kEvaderReachedGoal);
  interceptors[0] = state(Point3{100.0, 0.0, 10.0}, {}, 2);

  const MultiInterceptMissionUpdate update =
      evaluator.update(interceptors, state(Point3{100.0, 0.0, 10.0}, {}, 2));

  EXPECT_EQ(update.outcome, InterceptMissionOutcome::kEvaderReachedGoal);
  EXPECT_FALSE(update.newly_terminal);
  EXPECT_TRUE(update.newly_captured);
  EXPECT_EQ(update.capturing_interceptor_index, std::optional<std::size_t>{0U});
}

TEST(InterceptorHoldConfirmationTest, RequiresStablePositionAndSpeed) {
  InterceptorHoldConfirmation confirmation{
      InterceptorHoldConfig{.position_tolerance_m = 2.0,
                            .maximum_speed_mps = 0.8,
                            .confirmation_duration_s = 1.0}};
  EXPECT_FALSE(
      confirmation
          .update(state(Point3{10.0, 20.0, 15.0}, {}, 500'000'000LL), std::nullopt)
          .confirmed);
  EXPECT_FALSE(
      confirmation
          .update(state(Point3{10.0, 20.0, 15.0}, Vec3{2.0, 0.0, 0.0}, 1'000'000'000LL),
                  Point3{10.0, 20.0, 15.0})
          .confirmed);
  EXPECT_FALSE(
      confirmation
          .update(state(Point3{10.5, 20.0, 15.0}, Vec3{0.2, 0.0, 0.0}, 2'000'000'000LL),
                  Point3{10.0, 20.0, 15.0})
          .confirmed);
  const InterceptorHoldUpdate confirmed = confirmation.update(
      state(Point3{10.4, 20.0, 15.0}, Vec3{0.1, 0.0, 0.0}, 3'100'000'000LL),
      Point3{10.0, 20.0, 15.0});
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

TEST(InterceptStateAdjudicationLifecycleTest, RecoversFromTransientStaleness) {
  InterceptStateAdjudicationLifecycle lifecycle{InterceptStateAdjudicationConfig{
      .maximum_state_age_s = 1.0, .maximum_degraded_duration_s = 5.0}};
  const TimedVehicleState interceptor = state(Point3{}, {}, 1'000'000'000LL);
  const TimedVehicleState stale_evader = state(Point3{}, {}, 1'000'000'000LL);

  const InterceptStateAdjudicationUpdate degraded =
      lifecycle.update(2'100'000'000LL, interceptor, stale_evader);
  EXPECT_EQ(degraded.status, InterceptStateAdjudicationStatus::kDegraded);
  EXPECT_TRUE(degraded.newly_degraded);

  const InterceptStateAdjudicationUpdate recovered =
      lifecycle.update(2'200'000'000LL, state(Point3{}, {}, 2'200'000'000LL),
                       state(Point3{}, {}, 2'200'000'000LL));
  EXPECT_EQ(recovered.status, InterceptStateAdjudicationStatus::kHealthy);
  EXPECT_TRUE(recovered.newly_recovered);
}

TEST(InterceptStateAdjudicationLifecycleTest, ReportsProlongedFailureOnce) {
  InterceptStateAdjudicationLifecycle lifecycle{InterceptStateAdjudicationConfig{
      .maximum_state_age_s = 1.0, .maximum_degraded_duration_s = 2.0}};
  const TimedVehicleState stale = state(Point3{}, {}, 1'000'000'000LL);

  EXPECT_EQ(lifecycle.update(2'100'000'000LL, stale, stale).status,
            InterceptStateAdjudicationStatus::kDegraded);
  const InterceptStateAdjudicationUpdate failed =
      lifecycle.update(4'100'000'000LL, stale, stale);
  EXPECT_EQ(failed.status, InterceptStateAdjudicationStatus::kProlongedFailure);
  EXPECT_TRUE(failed.newly_prolonged_failure);
  EXPECT_FALSE(lifecycle.update(4'200'000'000LL, stale, stale).newly_prolonged_failure);
}

} // namespace
} // namespace drone_city_nav
