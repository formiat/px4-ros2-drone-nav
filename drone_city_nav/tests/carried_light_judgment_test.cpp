#include "drone_city_nav/carried_light_judgment.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {
namespace {

TEST(LightReliabilityJudgment, AModerateFlickerIsNotUnreliable) {
  // An outage of 3 s every 20 s for ten minutes: 15 percent, no stretch
  // near 5 s.
  LightReliabilityJudgment judgment;
  for (int tick = 0; tick < 6000; ++tick) {
    const double stamp_s = 0.1 * tick;
    const bool dark = std::fmod(stamp_s, 20.0) < 3.0;
    judgment.observe(stamp_s, dark ? 2.0 : 6.4, 6.4);
  }
  EXPECT_FALSE(judgment.unreliable());
  EXPECT_NEAR(judgment.outageShare(), 0.15, 0.01);
}

TEST(LightReliabilityJudgment, ALongOutageIsUnreliableAndStaysSo) {
  LightReliabilityJudgment judgment;
  for (int tick = 0; tick <= 45; ++tick) {
    judgment.observe(0.1 * tick, 2.0, 6.4);
  }
  EXPECT_FALSE(judgment.unreliable());
  judgment.observe(5.1, 2.0, 6.4);
  EXPECT_TRUE(judgment.unreliable());
  judgment.observe(10.2, 6.4, 6.4);
  EXPECT_TRUE(judgment.unreliable());
  EXPECT_EQ(judgment.currentOutageS(), 0.0);
}

TEST(LightReliabilityJudgment, FrequentOutagesAreUnreliable) {
  // 4.5 s dark in every 6.5 s: no single stretch reaches 5 s, the share
  // does.
  LightReliabilityJudgment judgment;
  bool reached{false};
  for (int tick = 0; tick < 1200 && !reached; ++tick) {
    const double stamp_s = 0.1 * tick;
    judgment.observe(stamp_s, std::fmod(stamp_s, 6.5) < 4.5 ? 2.0 : 6.4, 6.4);
    reached = judgment.unreliable();
  }
  EXPECT_TRUE(reached);
}

TEST(CarriedLightGoalEstimate, TheWayToBIsMarginedAndTheStartFloored) {
  EXPECT_DOUBLE_EQ(carriedLightGoalEstimateS(85.0, 0.0), 9.0 * 85.0 / 0.5 + 20.0);
  EXPECT_DOUBLE_EQ(carriedLightGoalEstimateS(60.0, 1.6), 9.0 * 60.0 / 1.6 + 20.0);
}

} // namespace
} // namespace drone_city_nav
