#include "drone_city_nav/px4_offboard_node_config.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace drone_city_nav {

TEST(Px4OffboardNodeConfig, BoundsScalarHelpers) {
  EXPECT_DOUBLE_EQ(boundedFiniteDouble(2.5, 1.0, 0.0, 2.0), 2.0);
  EXPECT_DOUBLE_EQ(
      boundedFiniteDouble(std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0, 2.0),
      1.0);
  EXPECT_EQ(boundedUint8(-1), 0U);
  EXPECT_EQ(boundedUint8(300), 255U);
  EXPECT_EQ(boundedUint16(-1), 0U);
  EXPECT_EQ(boundedUint16(70000), 65535U);
}

TEST(Px4OffboardNodeConfig, SanitizesTrajectoryRelatedConfig) {
  Px4OffboardNodeConfig config;
  config.cruise_altitude_m = 12.0;
  config.min_navigation_altitude_m = 100.0;
  config.takeoff_hover_s = -5.0;
  config.acceptance_radius_m = std::numeric_limits<double>::infinity();
  config.turn_preview_distance_m = 600.0;
  config.command_resend_period_s = 0.0;
  config.velocity_follower.cruise_speed_mps = 10.0;
  config.velocity_follower.min_turn_speed_mps = 20.0;
  config.velocity_follower.speed_profile_lookahead_min_m = 8.0;
  config.velocity_follower.speed_profile_lookahead_max_m = 3.0;

  sanitizePx4OffboardNodeConfig(config);

  EXPECT_DOUBLE_EQ(config.min_navigation_altitude_m, 12.0);
  EXPECT_DOUBLE_EQ(config.takeoff_hover_s, 0.0);
  EXPECT_DOUBLE_EQ(config.acceptance_radius_m, 1.5);
  EXPECT_DOUBLE_EQ(config.turn_preview_distance_m, 500.0);
  EXPECT_DOUBLE_EQ(config.command_resend_period_s, 0.05);
  EXPECT_DOUBLE_EQ(config.velocity_follower.min_turn_speed_mps, 10.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.speed_profile_lookahead_max_m, 8.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.final_acceptance_radius_m, 1.5);
}

} // namespace drone_city_nav
