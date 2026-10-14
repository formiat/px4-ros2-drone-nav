#include "drone_city_nav/carried_light_judgment.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr double kNoReading = std::numeric_limits<double>::quiet_NaN();
constexpr double kNoSurface = std::numeric_limits<double>::quiet_NaN();

TEST(LightReliabilityJudgment, AModerateFlickerIsNotUnreliable) {
  // An outage of 3 s every 20 s for ten minutes: 15 percent, no stretch
  // of 3.5 s.
  LightReliabilityJudgment judgment;
  for (int tick = 0; tick < 6000; ++tick) {
    const double stamp_s = 0.1 * tick;
    const bool dark = std::fmod(stamp_s, 20.0) < 3.0;
    judgment.observe(stamp_s, dark ? 0.0 : 1.0, kNoReading, kNoSurface);
  }
  EXPECT_FALSE(judgment.unreliable());
  EXPECT_NEAR(judgment.outageShare(), 0.15, 0.01);
}

TEST(LightReliabilityJudgment, ALongOutageIsUnreliableAndStaysSo) {
  LightReliabilityJudgment judgment;
  for (int tick = 0; tick <= 30; ++tick) {
    judgment.observe(0.1 * tick, 0.0, kNoReading, kNoSurface);
  }
  EXPECT_FALSE(judgment.unreliable());
  judgment.observe(3.6, 0.0, kNoReading, kNoSurface);
  EXPECT_TRUE(judgment.unreliable());
  judgment.observe(3.7, 1.0, kNoReading, kNoSurface);
  EXPECT_TRUE(judgment.unreliable());
  EXPECT_EQ(judgment.currentOutageS(), 0.0);
}

TEST(LightReliabilityJudgment, ADimFrameIsNoOutageABlindOneIs) {
  // A frame the contract still grants half its range sees; one it grants a
  // tenth, a light all but gone, is blind.
  LightReliabilityJudgment dim;
  for (int tick = 0; tick < 600; ++tick) {
    dim.observe(0.1 * tick, 0.5, kNoReading, kNoSurface);
  }
  EXPECT_FALSE(dim.unreliable());
  LightReliabilityJudgment blind;
  for (int tick = 0; tick <= 41; ++tick) {
    blind.observe(0.1 * tick, 0.1, kNoReading, kNoSurface);
  }
  EXPECT_TRUE(blind.unreliable());
}

TEST(LightReliabilityJudgment, FrequentOutagesAreUnreliable) {
  // 3.2 s dark in every 5 s: no single stretch reaches 3.5 s, the share
  // does.
  LightReliabilityJudgment judgment;
  bool reached{false};
  for (int tick = 0; tick < 1200 && !reached; ++tick) {
    const double stamp_s = 0.1 * tick;
    judgment.observe(stamp_s, std::fmod(stamp_s, 5.0) < 3.2 ? 0.0 : 1.0, kNoReading,
                     kNoSurface);
    reached = judgment.unreliable();
  }
  EXPECT_TRUE(reached);
}

TEST(LightReliabilityJudgment, DarknessWithASoundLightIsNoOutage) {
  // Roadmap item 29: the frames blind for ten minutes in a large dark space,
  // the light's reading full, nothing within the ring's range.
  LightReliabilityJudgment judgment;
  for (int tick = 0; tick < 6000; ++tick) {
    judgment.observe(0.1 * tick, 0.0, 1.0, kNoSurface);
  }
  EXPECT_FALSE(judgment.unreliable());
  EXPECT_EQ(judgment.outageShare(), 0.0);
}

TEST(LightReliabilityJudgment, TheLightsReadingDownIsAnOutage) {
  // Its frames still see (a lit neighbour), its reading under a quarter.
  LightReliabilityJudgment judgment;
  for (int tick = 0; tick <= 35; ++tick) {
    judgment.observe(0.1 * tick, 1.0, 0.1, kNoSurface);
  }
  EXPECT_TRUE(judgment.unreliable());
  LightReliabilityJudgment dimmed;
  for (int tick = 0; tick < 600; ++tick) {
    dimmed.observe(0.1 * tick, 1.0, 0.3, kNoSurface);
  }
  EXPECT_FALSE(dimmed.unreliable());
}

TEST(LightReliabilityJudgment, SurfacesTheRingSeesDarkAreTheLightsFailure) {
  // The reading full, the frame blind, the ring's surfaces in its field dark:
  // a soiled lens or a lamp dimming on its current.
  LightReliabilityJudgment judgment;
  for (int tick = 0; tick <= 35; ++tick) {
    judgment.observe(0.1 * tick, 0.0, 1.0, 1.0);
  }
  EXPECT_TRUE(judgment.unreliable());
  // A few of them dark beside lit ones is a shadow, not the light.
  LightReliabilityJudgment shadow;
  for (int tick = 0; tick < 600; ++tick) {
    shadow.observe(0.1 * tick, 0.0, 1.0, 0.2);
  }
  EXPECT_FALSE(shadow.unreliable());
}

} // namespace
} // namespace drone_city_nav
