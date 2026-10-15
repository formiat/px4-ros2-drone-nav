#include "drone_city_nav/smoke_ladder.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecond{1'000'000'000};

SmokeLadderInput at(const std::int64_t tenths, const bool blind, const double smoke,
                    const bool fresh = true, const bool lit = true) {
  return SmokeLadderInput{.blind = blind,
                          .smoke_share = smoke,
                          .lit = lit,
                          .position_fresh = fresh,
                          .position = Point3{},
                          .now_ns = tenths * kSecond / 10};
}

TEST(SmokeLadder, OutOfTheFirstPlumeTheVehicleHoldsThenTakesItsGoalAgain) {
  SmokeLadder ladder;
  EXPECT_EQ(ladder.observe(at(100, false, 0.0)), SmokeLadderAction::kNone);
  // Inside: blind, the ring smoky, the position gone with the frame; an
  // entry once that has lasted a second.
  EXPECT_EQ(ladder.observe(at(120, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.entries(), 0U);
  EXPECT_EQ(ladder.observe(at(130, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.entries(), 1U);
  // Backed out, seeing, the position back: the hold.
  EXPECT_EQ(ladder.observe(at(140, false, 0.0)), SmokeLadderAction::kHold);
  EXPECT_TRUE(ladder.holding());
  EXPECT_EQ(ladder.observe(at(300, false, 0.0)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(450, false, 0.0)), SmokeLadderAction::kRestore);
  EXPECT_FALSE(ladder.holding());
}

TEST(SmokeLadder, OutOfASecondPlumeTheGoalIsGivenUp) {
  SmokeLadder ladder;
  EXPECT_EQ(ladder.observe(at(120, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(130, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(140, false, 0.0)), SmokeLadderAction::kHold);
  EXPECT_EQ(ladder.observe(at(450, false, 0.0)), SmokeLadderAction::kRestore);
  // At a plume's edge: blind and dead reckoning under a reliable light, the
  // ring returning smoke from a few zones only (r1534).
  EXPECT_EQ(ladder.observe(at(600, true, 0.1, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(611, true, 0.1, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.entries(), 2U);
  EXPECT_EQ(ladder.observe(at(630, false, 0.0)), SmokeLadderAction::kGiveUp);
  EXPECT_EQ(ladder.observe(at(700, false, 0.0)), SmokeLadderAction::kNone);
}

TEST(SmokeLadder, TheDarkAndAGlitchAreNotSmoke) {
  SmokeLadder ladder;
  // Blind with a clear ring under a light reading an outage: the dark.
  for (std::int64_t tenths = 10; tenths < 200; tenths += 10) {
    EXPECT_EQ(ladder.observe(at(tenths, true, 0.0, false, false)),
              SmokeLadderAction::kNone);
  }
  EXPECT_EQ(ladder.entries(), 0U);
  // A dead-reckoning glitch under a reliable light shorter than a second.
  EXPECT_EQ(ladder.observe(at(200, true, 0.0, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(205, true, 0.0, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(210, false, 0.0)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.entries(), 0U);
  // Smoke reaching a holding vehicle counts as an entry.
  EXPECT_EQ(ladder.observe(at(300, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(310, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(320, false, 0.0)), SmokeLadderAction::kHold);
  EXPECT_EQ(ladder.observe(at(330, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(340, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.entries(), 2U);
}

} // namespace
} // namespace drone_city_nav
