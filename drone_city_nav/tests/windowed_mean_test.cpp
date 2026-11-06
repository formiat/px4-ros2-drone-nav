#include <gtest/gtest.h>

#include "windowed_mean.hpp"

namespace drone_city_nav {
namespace {

// The window's mean, and nothing of what came before the window: two
// smoothings that began at different readings agree, to the bit, once each
// holds a full window of the same values.
TEST(WindowedMean, HoldsNothingOlderThanItsWindow) {
  WindowedMean early{4U};
  WindowedMean late{4U};
  EXPECT_DOUBLE_EQ(7.0, early.push(7.0));
  EXPECT_DOUBLE_EQ(5.0, early.push(3.0));
  for (const double before : {100.0, -40.0, 12.5}) {
    early.push(before);
  }
  double from_early = 0.0;
  double from_late = 0.0;
  for (const double value : {1.0, 2.0, 4.0, 8.0}) {
    from_early = early.push(value);
    from_late = late.push(value);
  }
  EXPECT_EQ(from_early, from_late);
  EXPECT_DOUBLE_EQ(3.75, from_late);
  EXPECT_DOUBLE_EQ(5.75, late.push(9.0));
}

} // namespace
} // namespace drone_city_nav
