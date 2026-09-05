#include "drone_city_nav/motion_dynamics_3d.hpp"
#include "drone_city_nav/mppi/mppi_config.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "production_mppi_node_execution_internal.hpp"

namespace drone_city_nav {
namespace {

using production_mppi_execution_detail::appendFiniteExecutionPoints;

[[nodiscard]] std::vector<mppi::State>
integrate(const mppi::State& initial, const std::vector<mppi::Control>& controls,
          const mppi::DynamicsConfig& dynamics) {
  std::vector<mppi::State> states{initial};
  for (const mppi::Control& control : controls) {
    states.push_back(integrateMotionState3D(states.back(), control, dynamics));
  }
  return states;
}

TEST(ProductionMppiExecutionPointsTest, PointsCarryTheCommandWhereNothingClampedIt) {
  mppi::DynamicsConfig dynamics;
  dynamics.dt_s = 0.05F;
  dynamics.linear_drag_1ps = 0.08F;
  dynamics.maximum_translational_speed_mps = 6.567F;
  const std::vector<mppi::Control> controls(
      4U, mppi::Control{.ax = 1.5F, .ay = -0.5F, .az = 0.2F, .yaw_accel = 0.3F});
  const std::vector<mppi::State> states =
      integrate(mppi::State{.vx = 2.0F, .vy = 1.0F}, controls, dynamics);
  const mppi::Control applied{.ax = 1.2F, .ay = -0.4F};

  msg::MppiTrajectoryHorizon horizon;
  ASSERT_TRUE(appendFiniteExecutionPoints(horizon, states, controls, applied,
                                          50'000'000, dynamics));

  ASSERT_EQ(horizon.points.size(), states.size());
  EXPECT_FLOAT_EQ(static_cast<float>(horizon.points.front().acceleration.x),
                  applied.ax);
  EXPECT_FLOAT_EQ(static_cast<float>(horizon.points.front().acceleration.y),
                  applied.ay);
  for (std::size_t index = 1U; index < horizon.points.size(); ++index) {
    // Exactly the command: the applied-control witness has to match it.
    EXPECT_EQ(static_cast<float>(horizon.points[index].acceleration.x), 1.5F);
    EXPECT_EQ(static_cast<float>(horizon.points[index].acceleration.y), -0.5F);
    EXPECT_EQ(static_cast<float>(horizon.points[index].acceleration.z), 0.2F);
    EXPECT_EQ(horizon.points[index].yaw_acceleration_radps2, 0.3F);
  }
}

TEST(ProductionMppiExecutionPointsTest, PointsCarryTheBrakingTheCapImposesAboveIt) {
  // Above the translational cap the integrator sheds the excess at the
  // maximum deceleration whatever the command says. The command here pushes
  // along the motion; the vehicle must be told to brake, not to push.
  mppi::DynamicsConfig dynamics;
  dynamics.dt_s = 0.05F;
  dynamics.linear_drag_1ps = 0.08F;
  dynamics.maximum_horizontal_acceleration_mps2 = 4.0F;
  dynamics.maximum_horizontal_speed_mps = 10.0F;
  dynamics.maximum_translational_speed_mps = 6.567F;
  const std::vector<mppi::Control> controls(6U, mppi::Control{.ax = 2.0F, .ay = 2.0F});
  const mppi::State initial{.vx = 6.0F, .vy = 6.0F};
  const std::vector<mppi::State> states = integrate(initial, controls, dynamics);

  msg::MppiTrajectoryHorizon horizon;
  ASSERT_TRUE(appendFiniteExecutionPoints(horizon, states, controls, mppi::Control{},
                                          50'000'000, dynamics));

  ASSERT_EQ(horizon.points.size(), states.size());
  for (std::size_t index = 1U; index < horizon.points.size(); ++index) {
    const mppi::State& previous = states[index - 1U];
    const mppi::State& state = states[index];
    const double speed_before = std::hypot(previous.vx, previous.vy);
    const double speed_after = std::hypot(state.vx, state.vy);
    ASSERT_LT(speed_after, speed_before);
    // The published acceleration reproduces the clamped step exactly.
    const double drag = 1.0 - dynamics.linear_drag_1ps * dynamics.dt_s;
    const double ax = horizon.points[index].acceleration.x;
    const double ay = horizon.points[index].acceleration.y;
    EXPECT_NEAR(previous.vx * drag + ax * dynamics.dt_s, state.vx, 1.0e-4);
    EXPECT_NEAR(previous.vy * drag + ay * dynamics.dt_s, state.vy, 1.0e-4);
    // And it brakes: opposite to the motion, never the command's push.
    EXPECT_LT(ax * previous.vx + ay * previous.vy, 0.0);
  }
}

} // namespace
} // namespace drone_city_nav
