#include "drone_city_nav/smoke_ladder.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecond{1'000'000'000};

SmokeLadderInput at(const std::int64_t second, const bool blind, const double smoke,
                    const Point3& position) {
  return SmokeLadderInput{.blind = blind,
                          .smoke_share = smoke,
                          .position_fresh = true,
                          .position = position,
                          .now_ns = second * kSecond};
}

TEST(SmokeLadder, InsideAPlumeTheVehicleRetreatsHoldsAndGivesTheGoalUp) {
  SmokeLadder ladder;
  // Seeing on the way: the seeing place follows the vehicle.
  EXPECT_EQ(ladder.observe(at(10, false, 0.0, Point3{5.0, 0.0, 2.0})),
            SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(11, false, 0.05, Point3{6.0, 0.0, 2.0})),
            SmokeLadderAction::kNone);
  ASSERT_TRUE(ladder.seeingPlace().has_value());
  EXPECT_DOUBLE_EQ(ladder.seeingPlace()->x, 6.0);
  // Inside the plume: blind, the ring smoky all round; two seconds before the retreat.
  EXPECT_EQ(ladder.observe(at(12, true, 0.9, Point3{8.0, 0.0, 2.0})),
            SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(13, true, 0.9, Point3{8.5, 0.0, 2.0})),
            SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(14, true, 0.9, Point3{9.0, 0.0, 2.0})),
            SmokeLadderAction::kRetreat);
  EXPECT_TRUE(ladder.active());
  // Back within 2 m of the seeing place, still blind: the hold.
  EXPECT_EQ(ladder.observe(at(20, true, 0.9, Point3{7.0, 0.0, 2.0})),
            SmokeLadderAction::kHold);
  EXPECT_EQ(ladder.observe(at(40, true, 0.9, Point3{6.5, 0.0, 2.0})),
            SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(51, true, 0.9, Point3{6.5, 0.0, 2.0})),
            SmokeLadderAction::kGiveUp);
}

TEST(SmokeLadder, TheRangeBackRestoresTheGoalAndABlindFrameAloneIsNoSmoke) {
  SmokeLadder ladder;
  EXPECT_EQ(ladder.observe(at(1, false, 0.0, Point3{})), SmokeLadderAction::kNone);
  // Blind with the ring clear (the dark, not smoke): nothing.
  for (std::int64_t second = 2; second < 10; ++second) {
    EXPECT_EQ(ladder.observe(at(second, true, 0.0, Point3{1.0, 0.0, 0.0})),
              SmokeLadderAction::kNone);
  }
  EXPECT_EQ(ladder.observe(at(10, true, 0.6, Point3{2.0, 0.0, 0.0})),
            SmokeLadderAction::kNone);
  EXPECT_EQ(ladder.observe(at(13, true, 0.6, Point3{2.0, 0.0, 0.0})),
            SmokeLadderAction::kRetreat);
  // Seeing again on the way back: the goal restored, the ladder at rest.
  EXPECT_EQ(ladder.observe(at(15, false, 0.0, Point3{1.0, 0.0, 0.0})),
            SmokeLadderAction::kRestore);
  EXPECT_FALSE(ladder.active());
  // No position source: no retreat.
  SmokeLadderInput lost = at(20, true, 0.9, Point3{3.0, 0.0, 0.0});
  lost.position_fresh = false;
  EXPECT_EQ(ladder.observe(lost), SmokeLadderAction::kNone);
  lost.now_ns = 23 * kSecond;
  EXPECT_EQ(ladder.observe(lost), SmokeLadderAction::kNone);
}

} // namespace
} // namespace drone_city_nav
