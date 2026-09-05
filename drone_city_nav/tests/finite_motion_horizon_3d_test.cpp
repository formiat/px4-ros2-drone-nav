#include "drone_city_nav/finite_motion_horizon_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <optional>
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

TEST(FiniteMotionHorizon3DTest, ABrakingHorizonRestsFromAboveTheTranslationalSpeedCap) {
  // The sensor-braking cap sits below cruise overshoot: a vehicle observed at
  // 8 m/s under a 6.5 m/s cap is shed at the maximum deceleration whatever
  // the control commands, and the braking profile has to rest under that
  // model, not under the drag model alone.
  MotionDynamicsConfig3D dynamics;
  dynamics.dt_s = 0.05F;
  dynamics.linear_drag_1ps = 0.08F;
  dynamics.maximum_horizontal_acceleration_mps2 = 4.0F;
  dynamics.maximum_vertical_acceleration_mps2 = 4.0F;
  dynamics.maximum_control_jerk_mps3 = 12.0F;
  dynamics.maximum_translational_speed_mps = 6.5F;
  const FiniteMotionHorizonConfig3D config = makeFiniteMotionHorizonConfig3D(
      StoppingCapability{.maximum_commanded_horizontal_deceleration_mps2 = 4.0,
                         .guaranteed_horizontal_deceleration_mps2 = 4.0,
                         .guaranteed_vertical_deceleration_mps2 = 2.0,
                         .reaction_latency_s = 0.1});
  const MotionState3D state{.vx = 4.4F, .vy = 6.6F, .vz = 0.57F};
  constexpr std::size_t kControls{80U};

  const std::optional<FiniteMotionHorizon3D> horizon = buildFiniteBrakingHorizon3D(
      state, kControls, dynamics, MotionControl3D{.ax = 1.6F, .ay = 0.5F}, config);

  ASSERT_TRUE(horizon.has_value());
  const FiniteMotionHorizon3D& built =
      horizon.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_EQ(built.controls.size(), kControls);
  EXPECT_TRUE(finiteMotionHorizonHasTerminalRestState3D(built));
  // Every state is reachable under the same integrator the profile was
  // shaped against; the terminal state is pinned to exact rest from a
  // velocity inside the tolerance.
  MotionState3D simulated = state;
  for (std::size_t index = 0U; index < built.controls.size(); ++index) {
    simulated = integrateMotionState3D(simulated, built.controls[index], dynamics);
    EXPECT_NEAR(simulated.x, built.states[index + 1U].x, 1.0e-4F);
    EXPECT_NEAR(simulated.vx, built.states[index + 1U].vx,
                config.terminal_velocity_tolerance_mps);
  }
  EXPECT_LE(std::hypot(std::hypot(simulated.vx, simulated.vy), simulated.vz),
            config.terminal_velocity_tolerance_mps);
}

} // namespace
} // namespace drone_city_nav
