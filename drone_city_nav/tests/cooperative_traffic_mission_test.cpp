#include "drone_city_nav/cooperative_traffic_mission.hpp"

#include <gtest/gtest.h>

#include <array>
#include <limits>

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

TEST(CooperativeGoalHoldConfirmation, RequiresPlannerHoldAtOwnGoal) {
  CooperativeGoalHoldConfirmation confirmation;
  const Point3 goal{10.0, 20.0, 18.0};

  EXPECT_FALSE(confirmation.update(state(goal, {}, 1'000'000'000LL), goal, std::nullopt)
                   .confirmed);
  EXPECT_FALSE(
      confirmation.update(state(goal, Vec3{2.0, 0.0, 0.0}, 2'000'000'000LL), goal, goal)
          .confirmed);
  EXPECT_FALSE(
      confirmation.update(state(goal, Vec3{0.1, 0.0, 0.0}, 3'000'000'000LL), goal, goal)
          .confirmed);
  const CooperativeGoalHoldUpdate confirmed = confirmation.update(
      state(Point3{10.2, 20.0, 18.0}, Vec3{0.1, 0.0, 0.0}, 4'100'000'000LL), goal,
      goal);

  EXPECT_TRUE(confirmed.confirmed);
  EXPECT_TRUE(confirmed.newly_confirmed);
  EXPECT_NEAR(confirmed.goal_distance_m, 0.2, 1.0e-9);
}

TEST(CooperativeSeparationMonitor, DetectsSweptDesiredSeparationViolation) {
  CooperativeSeparationMonitor monitor{2U};
  std::array states{
      state(Point3{-10.0, 0.0, 18.0}, Vec3{10.0, 0.0, 0.0}, 1'000'000'000LL),
      state(Point3{10.0, 0.0, 18.0}, Vec3{-10.0, 0.0, 0.0}, 1'000'000'000LL),
  };
  EXPECT_EQ(monitor.update(states).desired_violation_event_count, 0U);
  states[0] = state(Point3{10.0, 0.0, 18.0}, Vec3{10.0, 0.0, 0.0}, 1'200'000'000LL);
  states[1] = state(Point3{-10.0, 0.0, 18.0}, Vec3{-10.0, 0.0, 0.0}, 1'200'000'000LL);

  const CooperativeSeparationUpdate update = monitor.update(states);

  ASSERT_EQ(update.pairs.size(), 1U);
  EXPECT_NEAR(update.minimum_separation_m, 0.0, 1.0e-9);
  EXPECT_TRUE(update.pairs.front().newly_entered_desired_violation);
  EXPECT_EQ(update.active_desired_violation_count, 1U);
  EXPECT_EQ(update.desired_violation_event_count, 1U);
}

TEST(CooperativeSeparationMonitor, ReleasesOnlyAfterSeparationAndHysteresis) {
  CooperativeSeparationMonitor monitor{2U};
  std::array states{
      state(Point3{0.0, 0.0, 18.0}, Vec3{-1.0, 0.0, 0.0}, 1'000'000'000LL),
      state(Point3{4.0, 0.0, 18.0}, Vec3{1.0, 0.0, 0.0}, 1'000'000'000LL),
  };
  EXPECT_EQ(monitor.update(states).active_desired_violation_count, 1U);
  states[0] = state(Point3{0.0, 0.0, 18.0}, Vec3{1.0, 0.0, 0.0}, 1'100'000'000LL);
  states[1] = state(Point3{7.5, 0.0, 18.0}, Vec3{-1.0, 0.0, 0.0}, 1'100'000'000LL);
  EXPECT_EQ(monitor.update(states).active_desired_violation_count, 1U);
  states[0].velocity = Vec3{-1.0, 0.0, 0.0};
  states[1].velocity = Vec3{1.0, 0.0, 0.0};
  states[0].stamp_ns = 1'200'000'000LL;
  states[1].stamp_ns = 1'200'000'000LL;

  const CooperativeSeparationUpdate released = monitor.update(states);

  EXPECT_EQ(released.active_desired_violation_count, 0U);
  EXPECT_TRUE(released.pairs.front().newly_released_desired_violation);
  EXPECT_EQ(released.desired_violation_event_count, 1U);
}

TEST(CooperativeSeparationMonitor, DoesNotSweepAcrossAStateGap) {
  CooperativeSeparationMonitor monitor{
      2U, CooperativeSeparationConfig{.desired_minimum_separation_m = 5.0,
                                      .release_separation_m = 7.0,
                                      .maximum_continuity_gap_s = 0.25}};
  std::array states{
      state(Point3{-10.0, 0.0, 18.0}, {}, 1'000'000'000LL),
      state(Point3{10.0, 0.0, 18.0}, {}, 1'000'000'000LL),
  };
  static_cast<void>(monitor.update(states));
  states[0] = state(Point3{10.0, 0.0, 18.0}, {}, 2'000'000'000LL);
  states[1] = state(Point3{-10.0, 0.0, 18.0}, {}, 2'000'000'000LL);

  const CooperativeSeparationUpdate update = monitor.update(states);

  EXPECT_NEAR(update.minimum_separation_m, 20.0, 1.0e-9);
  EXPECT_EQ(update.desired_violation_event_count, 0U);
  EXPECT_TRUE(std::isfinite(monitor.minimumObservedSeparationM()));
}

} // namespace
} // namespace drone_city_nav
