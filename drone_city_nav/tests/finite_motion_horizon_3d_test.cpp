#include "drone_city_nav/finite_motion_horizon_3d.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] FiniteMotionHorizon3D horizonSlidingIntoRest() {
  FiniteMotionHorizon3D horizon;
  // Three moving states, then four resting states at the terminal position.
  horizon.states = {
      MotionState3D{.x = 0.0F, .vx = 2.0F},
      MotionState3D{.x = 1.0F, .vx = 1.0F},
      MotionState3D{.x = 1.8F, .vx = 0.4F},
      MotionState3D{.x = 2.0F, .vx = 0.0F},
      MotionState3D{.x = 2.0F},
      MotionState3D{.x = 2.0F},
      MotionState3D{.x = 2.0F},
  };
  horizon.controls.assign(horizon.states.size() - 1U, MotionControl3D{});
  return horizon;
}

TEST(FiniteMotionHorizon3DTest, RestTailStartsWhereEveryLaterStateRestsAtTheTerminal) {
  const FiniteMotionHorizon3D horizon = horizonSlidingIntoRest();
  constexpr double kPositionToleranceM{0.25};
  constexpr double kSpeedToleranceMps{0.25};
  EXPECT_FALSE(finiteMotionHorizonRestsFromState3D(horizon, 0U, kPositionToleranceM,
                                                   kSpeedToleranceMps));
  EXPECT_FALSE(finiteMotionHorizonRestsFromState3D(horizon, 2U, kPositionToleranceM,
                                                   kSpeedToleranceMps));
  EXPECT_TRUE(finiteMotionHorizonRestsFromState3D(horizon, 3U, kPositionToleranceM,
                                                  kSpeedToleranceMps));
  EXPECT_TRUE(finiteMotionHorizonRestsFromState3D(
      horizon, horizon.states.size() - 1U, kPositionToleranceM, kSpeedToleranceMps));
}

TEST(FiniteMotionHorizon3DTest, RestTailRejectsResidualDriftAndOutOfRangeIndices) {
  FiniteMotionHorizon3D horizon = horizonSlidingIntoRest();
  constexpr double kPositionToleranceM{0.25};
  constexpr double kSpeedToleranceMps{0.25};
  EXPECT_FALSE(finiteMotionHorizonRestsFromState3D(
      horizon, horizon.states.size(), kPositionToleranceM, kSpeedToleranceMps));
  horizon.states[5U].yaw_rate = 0.5F;
  EXPECT_FALSE(finiteMotionHorizonRestsFromState3D(horizon, 3U, kPositionToleranceM,
                                                   kSpeedToleranceMps));
  horizon.states[5U].yaw_rate = 0.0F;
  horizon.states[4U].z = 0.5F;
  EXPECT_FALSE(finiteMotionHorizonRestsFromState3D(horizon, 3U, kPositionToleranceM,
                                                   kSpeedToleranceMps));
  EXPECT_TRUE(finiteMotionHorizonRestsFromState3D(horizon, 5U, kPositionToleranceM,
                                                  kSpeedToleranceMps));
  EXPECT_FALSE(finiteMotionHorizonRestsFromState3D(
      FiniteMotionHorizon3D{}, 0U, kPositionToleranceM, kSpeedToleranceMps));
}

} // namespace
} // namespace drone_city_nav
