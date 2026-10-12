#include "drone_city_nav/main_battery_estimate.hpp"
#include "drone_city_nav/operator_command.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {
namespace {

TEST(OperatorCommand, TheFourCommandsAreReadAndNothingElse) {
  EXPECT_EQ(parseOperatorCommand("stop")->kind, OperatorCommandKind::kStop);
  EXPECT_EQ(parseOperatorCommand("land")->kind, OperatorCommandKind::kLand);
  EXPECT_EQ(parseOperatorCommand("home")->kind, OperatorCommandKind::kHome);
  const std::optional<OperatorCommand> goal =
      parseOperatorCommand("goal 63.0 23.9 12.6");
  ASSERT_TRUE(goal.has_value());
  EXPECT_EQ(goal->kind, OperatorCommandKind::kGoal);
  EXPECT_DOUBLE_EQ(goal->goal.y, 23.9);
  EXPECT_FALSE(parseOperatorCommand("goal 1 2").has_value());
  EXPECT_FALSE(parseOperatorCommand("goal 1 2 nan").has_value());
  EXPECT_FALSE(parseOperatorCommand("land now").has_value());
  EXPECT_FALSE(parseOperatorCommand("arm").has_value());
  EXPECT_FALSE(parseOperatorCommand("").has_value());
}

// Specification K32: the autopilot's own time left is taken when it has one;
// without it the charge's fall over the window gives it, and a charge that
// does not fall gives none (the simulation's full battery).
TEST(MainBatteryEstimate, TimeLeftFromTheAutopilotOrFromTheFall) {
  MainBatteryEstimate full;
  for (int second = 0; second <= 60; ++second) {
    full.observe(second, 1.0, std::nan(""));
  }
  EXPECT_TRUE(std::isinf(full.timeLeftS()));

  MainBatteryEstimate draining;
  for (int second = 0; second <= 30; ++second) {
    draining.observe(second, 1.0 - 0.002 * second, std::nan(""));
  }
  // 0.94 left, falling 0.002 a second: 470 s.
  EXPECT_NEAR(draining.timeLeftS(), 470.0, 1.0);
  draining.observe(31.0, 0.938, 120.0);
  EXPECT_DOUBLE_EQ(draining.timeLeftS(), 120.0);

  MainBatteryEstimate young;
  young.observe(0.0, 1.0, std::nan(""));
  young.observe(5.0, 0.9, std::nan(""));
  EXPECT_TRUE(std::isinf(young.timeLeftS()));
}

} // namespace
} // namespace drone_city_nav
