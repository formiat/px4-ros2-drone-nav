#include "drone_city_nav/finite_motion_horizon_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
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

TEST(FiniteMotionHorizon3DTest, ABrakingHorizonContinuesTheDecelerationItStartsFrom) {
  // Rebuilt every tick under a receding horizon, only the first control of a
  // braking profile is ever flown. A profile that released the applied
  // deceleration and ramped in again from zero would never brake harder than
  // its first ramp step; the profile has to continue from the applied control.
  MotionDynamicsConfig3D dynamics;
  dynamics.dt_s = 0.05F;
  dynamics.maximum_horizontal_acceleration_mps2 = 4.0F;
  dynamics.maximum_vertical_acceleration_mps2 = 4.0F;
  dynamics.maximum_control_jerk_mps3 = 12.0F;
  const FiniteMotionHorizonConfig3D config = makeFiniteMotionHorizonConfig3D(
      StoppingCapability{.maximum_commanded_horizontal_deceleration_mps2 = 4.0,
                         .guaranteed_horizontal_deceleration_mps2 = 4.0,
                         .guaranteed_vertical_deceleration_mps2 = 2.0,
                         .reaction_latency_s = 0.1});
  const float jerk_step = dynamics.maximum_control_jerk_mps3 * dynamics.dt_s;
  constexpr std::size_t kControls{80U};
  MotionState3D state{.vx = 2.0F};
  MotionControl3D applied{.ax = -1.8F};

  // Three consecutive rebuilds, each from the state the previous first control
  // leaves and with that control as the applied one.
  float previous_first_ax = applied.ax;
  for (int rebuild = 0; rebuild < 3; ++rebuild) {
    const std::optional<FiniteMotionHorizon3D> horizon =
        buildFiniteBrakingHorizon3D(state, kControls, dynamics, applied, config);
    ASSERT_TRUE(horizon.has_value());
    const FiniteMotionHorizon3D& built =
        horizon.value(); // NOLINT(bugprone-unchecked-optional-access)
    EXPECT_TRUE(finiteMotionHorizonHasTerminalRestState3D(built));
    const MotionControl3D& first = built.controls.front();
    // Never weaker than the deceleration already applied, and within one jerk
    // step of it.
    EXPECT_LE(first.ax, previous_first_ax + 1.0e-4F);
    EXPECT_LE(std::abs(first.ax - applied.ax), jerk_step + 1.0e-4F);
    for (std::size_t index = 1U; index < built.controls.size(); ++index) {
      EXPECT_LE(std::abs(built.controls[index].ax - built.controls[index - 1U].ax),
                jerk_step + 1.0e-4F);
      EXPECT_LE(std::hypot(built.controls[index].ax, built.controls[index].ay),
                dynamics.maximum_horizontal_acceleration_mps2 + 1.0e-4F);
    }
    previous_first_ax = first.ax;
    state = integrateMotionState3D(state, first, dynamics);
    applied = first;
  }
  // Each rebuild took the deceleration one full jerk step further: the
  // profile is on its way to the full deceleration, never back to zero.
  EXPECT_NEAR(applied.ax, -1.8F - 3.0F * jerk_step, 1.0e-3F);
}

TEST(FiniteMotionHorizon3DTest, AnArrivalCancelsTheVelocityItsOwnRampInjects) {
  // The applied control need not point along the velocity. The ramp from it
  // to the deceleration injects velocity across the motion, and the profile
  // still has to end at rest, not merely along the initial direction.
  MotionDynamicsConfig3D dynamics;
  dynamics.dt_s = 0.05F;
  dynamics.maximum_horizontal_acceleration_mps2 = 4.0F;
  dynamics.maximum_vertical_acceleration_mps2 = 4.0F;
  dynamics.maximum_control_jerk_mps3 = 12.0F;
  const MotionState3D state{.vx = 1.5F, .vz = -0.4F};
  constexpr std::size_t kControls{60U};

  const std::optional<FiniteMotionHorizon3D> horizon = buildFiniteBrakingHorizon3D(
      state, kControls, dynamics, MotionControl3D{.ax = 0.5F, .ay = 3.5F, .az = 1.0F});

  ASSERT_TRUE(horizon.has_value());
  const FiniteMotionHorizon3D& built =
      horizon.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_TRUE(finiteMotionHorizonHasTerminalRestState3D(built));
  MotionState3D simulated = state;
  for (const MotionControl3D& control : built.controls) {
    simulated = integrateMotionState3D(simulated, control, dynamics);
  }
  EXPECT_LE(std::hypot(std::hypot(simulated.vx, simulated.vy), simulated.vz),
            FiniteMotionHorizonConfig3D{}.terminal_velocity_tolerance_mps);
}

TEST(FiniteMotionHorizon3DTest, ABrakingHorizonExistsFromEveryAdmissibleControl) {
  // A stop is the last thing the planner has for a vehicle without a plan.
  // The profile has to rest from every state the vehicle can be in, whatever
  // control it was applying: braking harder than the stop needs, accelerating
  // along the motion, or pushing across it.
  MotionDynamicsConfig3D dynamics;
  dynamics.dt_s = 0.05F;
  dynamics.linear_drag_1ps = 0.08F;
  dynamics.maximum_horizontal_acceleration_mps2 = 4.0F;
  dynamics.maximum_vertical_acceleration_mps2 = 4.0F;
  dynamics.maximum_control_jerk_mps3 = 12.0F;
  dynamics.maximum_translational_speed_mps = 6.567F;
  const FiniteMotionHorizonConfig3D config = makeFiniteMotionHorizonConfig3D(
      StoppingCapability{.maximum_commanded_horizontal_deceleration_mps2 = 4.0,
                         .guaranteed_horizontal_deceleration_mps2 = 4.0,
                         .guaranteed_vertical_deceleration_mps2 = 2.0,
                         .reaction_latency_s = 0.1});
  const std::vector<MotionState3D> states{
      MotionState3D{.vx = 0.362F, .vy = 0.569F, .vz = 0.235F},
      MotionState3D{.vx = 0.05F},
      MotionState3D{.vx = 0.7F, .vy = -0.7F},
      MotionState3D{.vx = 2.0F, .vz = -1.0F},
      MotionState3D{.vx = -4.0F, .vy = 3.0F, .vz = 0.5F},
      MotionState3D{.vx = 6.0F, .vy = 2.5F},
      MotionState3D{.vx = 7.0F, .vy = 7.0F, .vz = 1.0F},
      MotionState3D{.yaw_rate = 0.8F},
  };
  const std::vector<MotionControl3D> applied{
      MotionControl3D{},
      MotionControl3D{.ax = -0.049F, .ay = -2.286F, .az = -0.549F},
      MotionControl3D{.ax = 0.755F, .ay = -2.539F, .az = -1.298F},
      MotionControl3D{.ax = 2.5F, .ay = 3.0F},
      MotionControl3D{.ax = -3.9F, .ay = 0.5F, .az = 3.5F},
      MotionControl3D{.ax = 0.0F, .ay = 0.0F, .az = -4.0F},
      MotionControl3D{.ax = 1.0F, .ay = -1.0F, .yaw_accel = -1.5F},
  };
  constexpr std::size_t kControls{200U};
  const auto expect_rests = [&](const MotionState3D& state,
                                const MotionControl3D& control) {
    const std::optional<FiniteMotionHorizon3D> horizon =
        buildFiniteBrakingHorizon3D(state, kControls, dynamics, control, config);
    ASSERT_TRUE(horizon.has_value())
        << "v=(" << state.vx << "," << state.vy << "," << state.vz
        << ") yaw_rate=" << state.yaw_rate << " applied=(" << control.ax << ","
        << control.ay << "," << control.az << "," << control.yaw_accel << ")";
    EXPECT_TRUE(finiteMotionHorizonHasTerminalRestState3D(*horizon));
  };
  for (const MotionState3D& state : states) {
    for (const MotionControl3D& control : applied) {
      expect_rests(state, control);
    }
  }
  // A deterministic sweep over the whole admissible envelope: any speed up
  // to the cruise overshoot, any direction, any applied control inside the
  // acceleration limits.
  std::uint32_t seed{20260905U};
  const auto unit = [&seed]() {
    seed = seed * 1664525U + 1013904223U;
    return static_cast<float>(seed >> 8U) / static_cast<float>(1U << 24U);
  };
  for (int sample = 0; sample < 500; ++sample) {
    const float speed = 10.0F * unit();
    const float azimuth = 6.2831853F * unit();
    const float climb = (unit() - 0.5F) * 1.2F;
    MotionState3D state;
    state.vx = speed * std::cos(azimuth) * std::cos(climb);
    state.vy = speed * std::sin(azimuth) * std::cos(climb);
    state.vz = speed * std::sin(climb);
    state.yaw_rate = (unit() - 0.5F) * 2.0F;
    const float control_magnitude = 4.0F * unit();
    const float control_azimuth = 6.2831853F * unit();
    const MotionControl3D control{
        .ax = control_magnitude * std::cos(control_azimuth),
        .ay = control_magnitude * std::sin(control_azimuth),
        .az = (unit() - 0.5F) * 8.0F,
        .yaw_accel = (unit() - 0.5F) * 4.0F,
    };
    expect_rests(state, control);
  }
}

} // namespace
} // namespace drone_city_nav
