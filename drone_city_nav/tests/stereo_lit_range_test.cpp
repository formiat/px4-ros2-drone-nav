#include "drone_city_nav/stereo_lit_range.hpp"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace drone_city_nav {
namespace {

// The carried light's falloff as the records show it: each metre a fixed
// share of the 2 to 3 m metre.
constexpr std::array<double, 8> kLit{-1.0, 170.0, 140.0, 110.0, 90.0, 70.0, 55.0, 45.0};

LitRangeTracker learned() {
  LitRangeTracker tracker;
  for (std::size_t frame = 0U; frame < LitBaseline::kFrames; ++frame) {
    static_cast<void>(tracker.observe(kLit, true));
  }
  return tracker;
}

TEST(LitBaseline, TheMedianOfTheLastFramesOnceTenSecondsOfThem) {
  LitBaseline baseline;
  for (std::size_t frame = 0U; frame < LitBaseline::kMinimumFrames - 1U; ++frame) {
    baseline.observe(100.0);
  }
  EXPECT_FALSE(baseline.median().has_value());
  baseline.observe(100.0);
  EXPECT_DOUBLE_EQ(baseline.median().value_or(0.0), 100.0);
  // Thirty seconds of lower frames replace the window.
  for (std::size_t frame = 0U; frame < LitBaseline::kFrames; ++frame) {
    baseline.observe(40.0);
  }
  EXPECT_DOUBLE_EQ(baseline.median().value_or(0.0), 40.0);
}

TEST(LitRangeTracker, ThePlainFrameLightsTheWholeSpan) {
  LitRangeTracker tracker = learned();
  EXPECT_DOUBLE_EQ(tracker.observe(kLit, true), 8.0);
}

TEST(LitRangeTracker, AFlickerDimsEveryMetreAlikeAndLightsTheWholeSpan) {
  LitRangeTracker tracker = learned();
  std::array<double, 8> dimmed = kLit;
  for (double& value : dimmed) {
    value = value > 0.0 ? value / 6.0 : value;
  }
  EXPECT_DOUBLE_EQ(tracker.observe(dimmed, false), 8.0);
}

TEST(LitRangeTracker, AFadeOverDistanceEndsTheRangeAtTheFirstDarkenedMetre) {
  LitRangeTracker tracker = learned();
  // The light reaches 4 m as before and the metres beyond read a tenth.
  const std::array<double, 8> fade{-1.0, 170.0, 140.0, 110.0, 9.0, 5.0, 3.0, 2.0};
  EXPECT_DOUBLE_EQ(tracker.observe(fade, true), 4.0);
}

TEST(LitRangeTracker, AMetreNothingMatchedGivesNoVerdict) {
  LitRangeTracker tracker = learned();
  const std::array<double, 8> gap{-1.0, 170.0, 140.0, -1.0, -1.0, 70.0, 55.0, 45.0};
  EXPECT_DOUBLE_EQ(tracker.observe(gap, true), 8.0);
}

TEST(LitRangeTracker, WithoutABaselineTheRangeIsTheWholeSpan) {
  LitRangeTracker tracker;
  const std::array<double, 8> fade{-1.0, 170.0, 140.0, 110.0, 9.0, 5.0, 3.0, 2.0};
  EXPECT_DOUBLE_EQ(tracker.observe(fade, true), 8.0);
}

TEST(LitRangeTracker, WithoutTheReferenceMetreTheRangeIsTheWholeSpan) {
  LitRangeTracker tracker = learned();
  const std::array<double, 8> blind{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
  EXPECT_DOUBLE_EQ(tracker.observe(blind, false), 8.0);
}

} // namespace
} // namespace drone_city_nav
