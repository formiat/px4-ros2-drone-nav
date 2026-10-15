#include "drone_city_nav/smoke_ladder.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecond{1'000'000'000};

SmokeLadderInput at(const std::int64_t second, const bool blind, const double smoke,
                    const bool fresh = true) {
  return SmokeLadderInput{.blind = blind,
                          .smoke_share = smoke,
                          .position_fresh = fresh,
                          .position = Point3{},
                          .now_ns = second * kSecond};
}

TEST(SmokeLadder, OutOfTheFirstPlumeTheVehicleHoldsThenTakesItsGoalAgain) {
  SmokeLadder ladder;
  EXPECT_EQ(ladder.observe(at(10, false, 0.0)), SmokeLadderAction::kNone);
  // Inside: blind, the ring smoky, the position gone with the frame.
  EXPECT_EQ(ladder.observe(at(12, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.entries(), 1U);
  // Backed out, seeing, the position back: the hold.
  EXPECT_EQ(ladder.observe(at(14, false, 0.0)), SmokeLadderAction::kHold);
  EXPECT_TRUE(ladder.holding());
  EXPECT_EQ(ladder.observe(at(30, false, 0.0)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(45, false, 0.0)), SmokeLadderAction::kRestore);
  EXPECT_FALSE(ladder.holding());
}

TEST(SmokeLadder, OutOfASecondPlumeTheGoalIsGivenUp) {
  SmokeLadder ladder;
  EXPECT_EQ(ladder.observe(at(12, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(14, false, 0.0)), SmokeLadderAction::kHold);
  EXPECT_EQ(ladder.observe(at(45, false, 0.0)), SmokeLadderAction::kRestore);
  EXPECT_EQ(ladder.observe(at(60, true, 0.8, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.entries(), 2U);
  // Still blind and without a position: nothing yet.
  EXPECT_EQ(ladder.observe(at(61, true, 0.8, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(63, false, 0.0)), SmokeLadderAction::kGiveUp);
  EXPECT_EQ(ladder.observe(at(70, false, 0.0)), SmokeLadderAction::kNone);
}

TEST(SmokeLadder, ABlindFrameWithAClearRingIsTheDarkNotSmoke) {
  SmokeLadder ladder;
  for (std::int64_t second = 1; second < 20; ++second) {
    EXPECT_EQ(ladder.observe(at(second, true, 0.0)), SmokeLadderAction::kNone);
  }
  EXPECT_EQ(ladder.entries(), 0U);
  // Smoke reaching a holding vehicle counts as an entry.
  EXPECT_EQ(ladder.observe(at(20, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(22, false, 0.0)), SmokeLadderAction::kHold);
  EXPECT_EQ(ladder.observe(at(25, true, 0.9, false)), SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.entries(), 2U);
}

} // namespace
} // namespace drone_city_nav
