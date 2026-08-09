#include "drone_city_nav/target_assignment.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] TimedVehicleState state(const double x, const double y,
                                      const std::int64_t stamp_ns) {
  return TimedVehicleState{
      .position = Point3{x, y, 18.0},
      .velocity = Vec3{},
      .stamp_ns = stamp_ns,
      .position_valid = true,
      .velocity_valid = true,
      .navigation_ready = true,
  };
}

[[nodiscard]] TargetAssignmentTrack track(const std::uint64_t id, const double x,
                                          const double y, const std::int64_t stamp_ns) {
  return TargetAssignmentTrack{
      .state = state(x, y, stamp_ns),
      .detection_id = id,
      .track_id = id,
  };
}

[[nodiscard]] const TargetAssignmentDecision&
decisionFor(const TargetAssignmentUpdate& update, const std::string& id) {
  const auto iterator = std::ranges::find(update.decisions, id,
                                          &TargetAssignmentDecision::interceptor_id);
  if (iterator == update.decisions.end()) {
    ADD_FAILURE() << "No assignment for " << id;
    static const TargetAssignmentDecision missing{};
    return missing;
  }
  return *iterator;
}

TEST(TargetAssignment, AssignsDistinctNearestTargetsInTwoByTwoScenario) {
  AdaptiveTargetAssignment assignment;
  constexpr std::int64_t now_ns{1'000'000'000LL};
  const std::vector<TargetAssignmentTrack> tracks{track(1U, 10.0, 0.0, now_ns),
                                                  track(2U, 90.0, 0.0, now_ns)};
  const TargetAssignmentUpdate update = assignment.update(
      now_ns,
      {
          TargetAssignmentAgent{"interceptor_0", state(0.0, 0.0, now_ns), tracks},
          TargetAssignmentAgent{"interceptor_1", state(100.0, 0.0, now_ns), tracks},
      });

  ASSERT_TRUE(update.changed);
  ASSERT_EQ(update.decisions.size(), 2U);
  EXPECT_EQ(decisionFor(update, "interceptor_0").detection_id, 1U);
  EXPECT_EQ(decisionFor(update, "interceptor_1").detection_id, 2U);
}

TEST(TargetAssignment, AssignsSurplusInterceptorsAfterCoveringEveryTarget) {
  AdaptiveTargetAssignment assignment;
  constexpr std::int64_t now_ns{1'000'000'000LL};
  const std::vector<TargetAssignmentTrack> tracks{track(1U, 50.0, 0.0, now_ns),
                                                  track(2U, 150.0, 0.0, now_ns)};
  const TargetAssignmentUpdate update = assignment.update(
      now_ns,
      {
          TargetAssignmentAgent{"interceptor_0", state(0.0, 0.0, now_ns), tracks},
          TargetAssignmentAgent{"interceptor_1", state(100.0, 0.0, now_ns), tracks},
          TargetAssignmentAgent{"interceptor_2", state(200.0, 0.0, now_ns), tracks},
      });

  ASSERT_EQ(update.decisions.size(), 3U);
  const auto assigned = [&update](const std::uint64_t target_id) {
    return std::ranges::count(update.decisions, target_id,
                              &TargetAssignmentDecision::detection_id);
  };
  EXPECT_GE(assigned(1U), 1);
  EXPECT_GE(assigned(2U), 1);
}

TEST(TargetAssignment, RequiresStableCostImprovementBeforeSwitching) {
  AdaptiveTargetAssignment assignment{TargetAssignmentConfig{
      .interceptor_speed_mps = 20.0,
      .maximum_track_age_s = 10.0,
      .switch_penalty_s = 0.0,
      .minimum_switch_improvement_s = 0.1,
      .minimum_switch_improvement_ratio = 0.0,
      .minimum_assignment_hold_s = 0.0,
      .switch_confirmation_s = 0.5,
      .no_intercept_solution_penalty_s = 30.0,
  }};
  const auto agents = [](const std::int64_t stamp_ns, const bool swapped) {
    const std::vector<TargetAssignmentTrack> tracks{
        track(1U, swapped ? 90.0 : 10.0, 0.0, stamp_ns),
        track(2U, swapped ? 10.0 : 90.0, 0.0, stamp_ns),
    };
    return std::vector<TargetAssignmentAgent>{
        {"interceptor_0", state(0.0, 0.0, stamp_ns), tracks},
        {"interceptor_1", state(100.0, 0.0, stamp_ns), tracks},
    };
  };

  const TargetAssignmentUpdate initial =
      assignment.update(1'000'000'000LL, agents(1'000'000'000LL, false));
  ASSERT_TRUE(initial.changed);
  const TargetAssignmentUpdate pending =
      assignment.update(2'000'000'000LL, agents(2'000'000'000LL, true));
  EXPECT_FALSE(pending.changed);
  EXPECT_EQ(decisionFor(pending, "interceptor_0").detection_id, 1U);
  const TargetAssignmentUpdate switched =
      assignment.update(2'600'000'000LL, agents(2'600'000'000LL, true));
  EXPECT_TRUE(switched.changed);
  EXPECT_EQ(decisionFor(switched, "interceptor_0").detection_id, 2U);
}

TEST(TargetAssignment, ReassignsImmediatelyWhenCurrentTargetDisappears) {
  AdaptiveTargetAssignment assignment;
  constexpr std::int64_t first_ns{1'000'000'000LL};
  const TargetAssignmentUpdate initial = assignment.update(
      first_ns, {TargetAssignmentAgent{
                    "interceptor_0",
                    state(0.0, 0.0, first_ns),
                    {track(1U, 10.0, 0.0, first_ns), track(2U, 20.0, 0.0, first_ns)}}});
  ASSERT_EQ(decisionFor(initial, "interceptor_0").detection_id, 1U);

  constexpr std::int64_t next_ns{1'100'000'000LL};
  const TargetAssignmentUpdate reassigned = assignment.update(
      next_ns,
      {TargetAssignmentAgent{
          "interceptor_0", state(0.0, 0.0, next_ns), {track(2U, 20.0, 0.0, next_ns)}}});

  EXPECT_TRUE(reassigned.changed);
  EXPECT_EQ(reassigned.reason, TargetAssignmentReason::kTargetSetChanged);
  EXPECT_EQ(decisionFor(reassigned, "interceptor_0").detection_id, 2U);
}

} // namespace
} // namespace drone_city_nav
