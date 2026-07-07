#include "drone_city_nav/velocity_control_config.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {

TEST(VelocityControlConfig, SeparatesConstructionAndRuntimeSpeedPolicyFingerprints) {
  VelocityFollowerConfig config{};
  const std::uint64_t construction = speedProfileConstructionConfigFingerprint(config);
  const std::uint64_t speed_policy = runtimeSpeedPolicyConfigFingerprint(config);
  const std::uint64_t velocity_control =
      runtimeVelocityControlConfigFingerprint(config);

  config.speed_profile_lookahead_max_m += 10.0;

  EXPECT_EQ(speedProfileConstructionConfigFingerprint(config), construction);
  EXPECT_NE(runtimeSpeedPolicyConfigFingerprint(config), speed_policy);
  EXPECT_EQ(runtimeVelocityControlConfigFingerprint(config), velocity_control);
}

TEST(VelocityControlConfig, ConstructionFingerprintTracksProfileBuildInputs) {
  VelocityFollowerConfig config{};
  const std::uint64_t construction = speedProfileConstructionConfigFingerprint(config);

  config.speed_profile_sample_step_m = 0.5;

  EXPECT_NE(speedProfileConstructionConfigFingerprint(config), construction);
}

TEST(VelocityControlConfig, ConstructionFingerprintTracksVerticalProfileCaps) {
  VelocityFollowerConfig config{};
  const std::uint64_t construction = speedProfileConstructionConfigFingerprint(config);

  config.vertical_profile_max_vertical_speed_mps += 0.5;

  EXPECT_NE(speedProfileConstructionConfigFingerprint(config), construction);
}

TEST(VelocityControlConfig, RuntimeSpeedPolicyTracksVerticalTrackability) {
  VelocityFollowerConfig config{};
  const std::uint64_t construction = speedProfileConstructionConfigFingerprint(config);
  const std::uint64_t speed_policy = runtimeSpeedPolicyConfigFingerprint(config);
  const std::uint64_t velocity_control =
      runtimeVelocityControlConfigFingerprint(config);

  config.vertical_trackability_altitude_tolerance_m += 0.1;

  EXPECT_EQ(speedProfileConstructionConfigFingerprint(config), construction);
  EXPECT_NE(runtimeSpeedPolicyConfigFingerprint(config), speed_policy);
  EXPECT_EQ(runtimeVelocityControlConfigFingerprint(config), velocity_control);
}

TEST(VelocityControlConfig, RuntimeVelocityControlTracksLateralAndTerminalInputs) {
  VelocityFollowerConfig config{};
  const std::uint64_t construction = speedProfileConstructionConfigFingerprint(config);
  const std::uint64_t speed_policy = runtimeSpeedPolicyConfigFingerprint(config);
  const std::uint64_t velocity_control =
      runtimeVelocityControlConfigFingerprint(config);

  config.curvature_feedforward_time_s += 0.1;
  config.terminal_capture_radius_m += 1.0;

  EXPECT_EQ(speedProfileConstructionConfigFingerprint(config), construction);
  EXPECT_EQ(runtimeSpeedPolicyConfigFingerprint(config), speed_policy);
  EXPECT_NE(runtimeVelocityControlConfigFingerprint(config), velocity_control);
}

TEST(VelocityControlConfig, ForwardSetpointDynamicsAffectBothRuntimeLayers) {
  VelocityFollowerConfig config{};
  const std::uint64_t construction = speedProfileConstructionConfigFingerprint(config);
  const std::uint64_t speed_policy = runtimeSpeedPolicyConfigFingerprint(config);
  const std::uint64_t velocity_control =
      runtimeVelocityControlConfigFingerprint(config);

  config.setpoint_forward_accel_mps2 += 1.0;

  EXPECT_EQ(speedProfileConstructionConfigFingerprint(config), construction);
  EXPECT_NE(runtimeSpeedPolicyConfigFingerprint(config), speed_policy);
  EXPECT_NE(runtimeVelocityControlConfigFingerprint(config), velocity_control);
}

} // namespace drone_city_nav
