#include "drone_city_nav/vertical_capture_profile.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {
namespace {

TEST(VerticalCaptureProfile, DescendsWithinConfiguredLimits) {
  VerticalCaptureProfileState state{};
  VerticalCaptureProfileConfig config{};
  config.max_descent_speed_mps = 3.2;
  config.max_accel_mps2 = 3.0;
  config.max_jerk_mps3 = 9.0;
  constexpr double dt_s = 0.05;
  double previous_accel_mps2 = 0.0;

  for (int step_index = 0; step_index < 400; ++step_index) {
    const VerticalCaptureProfileStep step =
        advanceVerticalCaptureProfile(state, 23.5, 0.0, true, 6.5, dt_s, config);
    ASSERT_TRUE(step.valid);
    EXPECT_LE(std::abs(step.state.commanded_vz_mps), 3.2 + 1.0e-9);
    EXPECT_LE(std::abs(step.state.commanded_accel_mps2), 3.0 + 1.0e-9);
    EXPECT_LE(std::abs(step.state.commanded_accel_mps2 - previous_accel_mps2),
              9.0 * dt_s + 1.0e-9);
    previous_accel_mps2 = step.state.commanded_accel_mps2;
    state = step.state;
    if (step.target_reached) {
      break;
    }
  }

  EXPECT_NEAR(state.commanded_z_m, 6.5, 1.0e-9);
  EXPECT_DOUBLE_EQ(state.commanded_vz_mps, 0.0);
}

TEST(VerticalCaptureProfile, RejectsNonFiniteInput) {
  const VerticalCaptureProfileStep step = advanceVerticalCaptureProfile(
      VerticalCaptureProfileState{}, 10.0, 0.0, true,
      std::numeric_limits<double>::quiet_NaN(), 0.05, VerticalCaptureProfileConfig{});
  EXPECT_FALSE(step.valid);
}

} // namespace
} // namespace drone_city_nav
