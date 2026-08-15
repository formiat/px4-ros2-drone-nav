#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace drone_city_nav::mppi {
namespace {

[[nodiscard]] std::vector<State> simulate(const State& initial,
                                          const std::vector<Control>& controls,
                                          const DynamicsConfig& dynamics) {
  std::vector<State> states{initial};
  states.reserve(controls.size() + 1U);
  for (const Control& control : controls) {
    states.push_back(integrateReference(states.back(), control, dynamics));
  }
  return states;
}

TEST(MppiFiniteHorizon, EmbedsArrivalProfileInsidePlannedDuration) {
  DynamicsConfig dynamics;
  std::vector<Control> controls(120U, Control{});
  const State initial{.vx = 8.0F, .vy = 2.0F, .vz = -1.0F, .yaw_rate = 0.4F};
  const std::vector<State> states = simulate(initial, controls, dynamics);
  constexpr std::size_t kNominalPrefixControls{20U};

  const std::optional<FiniteHorizon> finite =
      buildFiniteHorizon(states, controls, kNominalPrefixControls, dynamics, Control{});

  ASSERT_TRUE(finite.has_value());
  const FiniteHorizon path = finite.value_or(FiniteHorizon{});
  EXPECT_EQ(path.nominal_prefix_control_count, kNominalPrefixControls);
  EXPECT_EQ(path.arrival_control_count, controls.size() - kNominalPrefixControls);
  EXPECT_EQ(path.controls.size(), controls.size());
  EXPECT_EQ(path.states.size(), states.size());
  ASSERT_EQ(path.states.size(), path.controls.size() + 1U);
  EXPECT_TRUE(finiteHorizonHasTerminalRestState(path));
  for (std::size_t index = 0U; index <= kNominalPrefixControls; ++index) {
    EXPECT_FLOAT_EQ(path.states[index].x, states[index].x);
    EXPECT_FLOAT_EQ(path.states[index].y, states[index].y);
    EXPECT_FLOAT_EQ(path.states[index].z, states[index].z);
  }
}

TEST(MppiFiniteHorizon, ArrivalControlsRespectAccelerationAndJerkLimits) {
  DynamicsConfig dynamics;
  dynamics.maximum_horizontal_acceleration_mps2 = 4.0F;
  dynamics.maximum_vertical_acceleration_mps2 = 2.0F;
  dynamics.maximum_control_jerk_mps3 = 12.0F;
  const Control previous{.ax = 3.0F, .ay = -2.0F, .az = 1.5F};
  const std::vector<Control> controls(160U, previous);
  const std::vector<State> states =
      simulate(State{.vx = 10.0F, .vy = -3.0F, .vz = 2.0F}, controls, dynamics);
  constexpr std::size_t kNominalPrefixControls{10U};

  const std::optional<FiniteHorizon> finite =
      buildFiniteHorizon(states, controls, kNominalPrefixControls, dynamics, Control{});

  ASSERT_TRUE(finite.has_value());
  const FiniteHorizon path = finite.value_or(FiniteHorizon{});
  Control last = controls[kNominalPrefixControls - 1U];
  for (std::size_t index = path.nominal_prefix_control_count;
       index < path.controls.size(); ++index) {
    const Control& control = path.controls[index];
    EXPECT_LE(std::hypot(control.ax, control.ay),
              dynamics.maximum_horizontal_acceleration_mps2 + 1.0e-4F);
    EXPECT_LE(std::abs(control.az),
              dynamics.maximum_vertical_acceleration_mps2 + 1.0e-4F);
    EXPECT_LE(std::abs(control.ax - last.ax),
              dynamics.maximum_control_jerk_mps3 * dynamics.dt_s + 1.0e-4F);
    EXPECT_LE(std::abs(control.ay - last.ay),
              dynamics.maximum_control_jerk_mps3 * dynamics.dt_s + 1.0e-4F);
    EXPECT_LE(std::abs(control.az - last.az),
              dynamics.maximum_control_jerk_mps3 * dynamics.dt_s + 1.0e-4F);
    last = control;
  }
  EXPECT_TRUE(finiteHorizonHasTerminalRestState(path));
}

TEST(MppiFiniteHorizon, KeepsNominalPrefixAndStopsBeforeOriginalEndpoint) {
  DynamicsConfig dynamics;
  const std::vector<Control> controls(120U, Control{.ax = 0.5F, .ay = 0.25F});
  const std::vector<State> states = simulate(State{.vx = 5.0F}, controls, dynamics);
  constexpr std::size_t kNominalPrefixControls{12U};

  const std::optional<FiniteHorizon> finite =
      buildFiniteHorizon(states, controls, kNominalPrefixControls, dynamics, Control{});

  ASSERT_TRUE(finite.has_value());
  const FiniteHorizon path = finite.value_or(FiniteHorizon{});
  EXPECT_EQ(path.nominal_prefix_control_count, kNominalPrefixControls);
  EXPECT_FLOAT_EQ(path.states[kNominalPrefixControls].x,
                  states[kNominalPrefixControls].x);
  EXPECT_FLOAT_EQ(path.states[kNominalPrefixControls].y,
                  states[kNominalPrefixControls].y);
  EXPECT_LT(path.states.back().x, states.back().x);
  EXPECT_TRUE(finiteHorizonHasTerminalRestState(path));
}

TEST(MppiFiniteHorizon, UsesConservativeHorizontalDecelerationInArrivalProfile) {
  DynamicsConfig dynamics;
  dynamics.maximum_horizontal_acceleration_mps2 = 8.0F;
  const std::vector<Control> controls(120U);
  const std::vector<State> states = simulate(State{.vx = 10.0F}, controls, dynamics);
  FiniteHorizonConfig config;
  config.maximum_horizontal_deceleration_mps2 = 4.0F;

  const std::optional<FiniteHorizon> finite =
      buildFiniteHorizon(states, controls, 0U, dynamics, Control{}, config);

  ASSERT_TRUE(finite.has_value());
  const FiniteHorizon path = finite.value_or(FiniteHorizon{});
  for (const Control& control : path.controls) {
    EXPECT_LE(std::hypot(control.ax, control.ay),
              config.maximum_horizontal_deceleration_mps2 + 1.0e-4F);
  }
  EXPECT_TRUE(finiteHorizonHasTerminalRestState(path));
}

TEST(MppiFiniteHorizon, FitsNoStaticCruiseArrivalInsideFourSecondPath) {
  DynamicsConfig dynamics;
  dynamics.maximum_horizontal_acceleration_mps2 = 4.0F;
  dynamics.maximum_vertical_acceleration_mps2 = 4.0F;
  dynamics.maximum_control_jerk_mps3 = 12.0F;
  std::vector<Control> controls(80U);
  const State initial{
      .vx = 8.83F,
      .vy = 4.74F,
      .vz = 3.53F,
  };
  const std::vector<State> states = simulate(initial, controls, dynamics);
  FiniteHorizonConfig config;
  config.maximum_horizontal_deceleration_mps2 = 4.0F;
  const Control previous{.ax = 2.0F, .ay = 1.0F, .az = -2.8F};

  const std::optional<FiniteHorizon> finite =
      buildFiniteHorizon(states, controls, 0U, dynamics, previous, config);

  ASSERT_TRUE(finite.has_value());
  const FiniteHorizon horizon = finite.value_or(FiniteHorizon{});
  EXPECT_TRUE(finiteHorizonHasTerminalRestState(horizon));
  EXPECT_EQ(horizon.controls.size(), controls.size());
}

TEST(MppiFiniteHorizon, RejectsPathWithoutRoomForArrivalProfile) {
  DynamicsConfig dynamics;
  const std::vector<Control> controls(2U);
  const std::vector<State> states = simulate(State{.vx = 20.0F}, controls, dynamics);

  EXPECT_FALSE(
      buildFiniteHorizon(states, controls, controls.size(), dynamics, Control{})
          .has_value());
}

TEST(MppiFiniteHorizon, ConvertsConfiguredControlPeriodWithoutFloatDrift) {
  EXPECT_EQ(finitePathControlIntervalNanoseconds(0.05F), 50'000'000LL);
  EXPECT_EQ(finitePathControlIntervalNanoseconds(0.1F), 100'000'000LL);
}

} // namespace
} // namespace drone_city_nav::mppi
