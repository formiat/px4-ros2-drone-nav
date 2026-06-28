#include "drone_city_nav/lidar_debug_node_config.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {

TEST(LidarDebugNodeConfig, KeepsDefaultsUsable) {
  LidarDebugNodeConfig config;

  sanitizeLidarDebugNodeConfig(config);

  EXPECT_EQ(config.output_dir, "log/lidar_debug");
  EXPECT_DOUBLE_EQ(config.snapshot_period_s, 1.0);
  EXPECT_DOUBLE_EQ(config.view_radius_m, 45.0);
  EXPECT_DOUBLE_EQ(config.max_lidar_range_m, 35.0);
  EXPECT_EQ(config.beam_csv_stride, 1U);
  EXPECT_EQ(config.max_remembered_hit_points, 50000U);
}

TEST(LidarDebugNodeConfig, AppliesRuntimeClampRules) {
  LidarDebugNodeConfig config;
  config.snapshot_period_s = 0.01;
  config.view_radius_m = 1.0;
  config.max_lidar_range_m = 0.0;
  config.range_hit_epsilon_m = -1.0;
  config.hit_memory_resolution_m = 0.01;
  config.beam_csv_stride = 0U;
  config.max_logged_hit_points = 200000U;
  config.max_remembered_hit_points = 0U;

  sanitizeLidarDebugNodeConfig(config);

  EXPECT_DOUBLE_EQ(config.snapshot_period_s, 0.1);
  EXPECT_DOUBLE_EQ(config.view_radius_m, 5.0);
  EXPECT_DOUBLE_EQ(config.max_lidar_range_m, 1.0);
  EXPECT_DOUBLE_EQ(config.range_hit_epsilon_m, 0.0);
  EXPECT_DOUBLE_EQ(config.hit_memory_resolution_m, 0.05);
  EXPECT_EQ(config.beam_csv_stride, 1U);
  EXPECT_EQ(config.max_logged_hit_points, 100000U);
  EXPECT_EQ(config.max_remembered_hit_points, 1U);
}

} // namespace drone_city_nav
