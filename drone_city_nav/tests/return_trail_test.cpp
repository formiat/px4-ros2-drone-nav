#include "drone_city_nav/return_trail.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {

namespace {

// A straight way out along x, a point every metre recorded, every 5 m kept.
ReturnTrail straightTrail(const double length_m) {
  ReturnTrail trail;
  for (double x = 0.0; x <= length_m; x += 1.0) {
    trail.record(Point3{x, 0.0, 10.0});
  }
  return trail;
}

} // namespace

TEST(ReturnTrail, KeepsAPointEveryFiveMetres) {
  const ReturnTrail trail = straightTrail(100.0);
  ASSERT_EQ(trail.points().size(), 21U);
  EXPECT_DOUBLE_EQ(trail.points()[4].x, 20.0);
}

TEST(ReturnTrail, TheObjectiveLiesTenMetresBackAndPassesOnWhenReached) {
  ReturnTrail trail = straightTrail(100.0);
  // At the far end: the point 10 m back along the trail.
  EXPECT_EQ(trail.next(Point3{100.0, 0.0, 10.0}), 18U);
  // Flown 15 m back: 10 m back from where the vehicle now is.
  EXPECT_EQ(trail.next(Point3{85.0, 0.0, 10.0}), 15U);
  // Never later than before, whatever the vehicle does.
  EXPECT_EQ(trail.next(Point3{95.0, 0.0, 10.0}), 15U);
  EXPECT_EQ(trail.next(Point3{66.0, 0.0, 10.0}), 11U);
  // Near the start the start itself.
  EXPECT_EQ(trail.next(Point3{20.0, 0.0, 10.0}), 2U);
  EXPECT_EQ(trail.next(Point3{12.0, 0.0, 10.0}), 0U);
  EXPECT_EQ(trail.next(Point3{3.0, 0.0, 10.0}), 0U);
}

TEST(ReturnTrail, ATrailCrossedAgainIsTakenFromTheEarlierPass) {
  // Out along x to 60 m, a loop away, and back beside the way out.
  ReturnTrail trail;
  for (double x = 0.0; x <= 60.0; x += 1.0) {
    trail.record(Point3{x, 0.0, 10.0});
  }
  for (double y = 1.0; y <= 40.0; y += 1.0) {
    trail.record(Point3{60.0, y, 10.0});
  }
  for (double x = 59.0; x >= 30.0; x -= 1.0) {
    trail.record(Point3{x, 40.0, 10.0});
  }
  for (double y = 39.0; y >= 2.0; y -= 1.0) {
    trail.record(Point3{30.0, y, 10.0});
  }
  // Standing over the way out at x = 30: the objective is 10 m back along the
  // first pass, not the whole loop.
  EXPECT_EQ(trail.next(Point3{30.0, 1.0, 10.0}), 4U);
}

TEST(ReturnTrail, NoTrailNamesTheStart) {
  ReturnTrail trail;
  EXPECT_EQ(trail.next(Point3{5.0, 5.0, 10.0}), 0U);
}

} // namespace drone_city_nav
