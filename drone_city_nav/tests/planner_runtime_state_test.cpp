#include "drone_city_nav/planner_runtime_state.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {

TEST(PlannerRuntimeState, ComputesFiniteAgeAfterStamp) {
  EXPECT_DOUBLE_EQ(ageSecondsFromStamp(1'000'000'000LL, 2'500'000'000LL), 1.5);
}

TEST(PlannerRuntimeState, ReturnsInfinityForMissingOrFutureStamp) {
  EXPECT_TRUE(std::isinf(ageSecondsFromStamp(0, 2'000'000'000LL)));
  EXPECT_TRUE(std::isinf(ageSecondsFromStamp(3'000'000'000LL, 2'000'000'000LL)));
}

} // namespace drone_city_nav
