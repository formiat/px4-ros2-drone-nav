#include "drone_city_nav/lidar_debug_node_config.hpp"

#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

class LidarDebugNodeConfigTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  [[nodiscard]] static std::shared_ptr<rclcpp::Node>
  makeNode(const std::string& name,
           const std::vector<rclcpp::Parameter>& parameters = {}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    return std::make_shared<rclcpp::Node>(name, options);
  }
};

} // namespace

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

TEST_F(LidarDebugNodeConfigTest, LoadsDocumentedDefaults) {
  const auto node = makeNode("lidar_debug_node_config_defaults");

  const LidarDebugNodeConfig config = loadLidarDebugNodeConfig(*node);

  EXPECT_EQ(config.output_dir, "log/lidar_debug");
  EXPECT_DOUBLE_EQ(config.snapshot_period_s, 1.0);
  EXPECT_EQ(config.image_size_px, 900);
  EXPECT_DOUBLE_EQ(config.view_radius_m, 45.0);
  EXPECT_DOUBLE_EQ(config.max_lidar_range_m, 35.0);
  EXPECT_TRUE(config.motion_compensate_lidar_pose);
  EXPECT_DOUBLE_EQ(config.lidar_pose_latency_s, 0.05);
  EXPECT_DOUBLE_EQ(config.hit_memory_resolution_m, 0.25);
  EXPECT_EQ(config.topics.lidar, "/scan");
  EXPECT_EQ(config.topics.path, "/drone_city_nav/final_trajectory_path");
  EXPECT_EQ(config.topics.prohibited_pointcloud,
            "/drone_city_nav/prohibited_obstacle_points");
  EXPECT_EQ(config.topics.px4_local_position, "/fmu/out/vehicle_local_position_v1");
}

TEST_F(LidarDebugNodeConfigTest, LoadsCustomTopicsAndProjectionParams) {
  const auto node = makeNode(
      "lidar_debug_node_config_custom",
      {rclcpp::Parameter{"output_dir", "log/custom_lidar"},
       rclcpp::Parameter{"lidar_topic", "/custom/scan"},
       rclcpp::Parameter{"prohibited_grid_topic", "/custom/prohibited"},
       rclcpp::Parameter{"memory_grid_topic", "/custom/memory"},
       rclcpp::Parameter{"path_topic", "/custom/path"},
       rclcpp::Parameter{"pointcloud_topic", "/custom/current_points"},
       rclcpp::Parameter{"remembered_pointcloud_topic", "/custom/remembered"},
       rclcpp::Parameter{"prohibited_pointcloud_topic", "/custom/prohibited_points"},
       rclcpp::Parameter{"raw_memory_pointcloud_topic", "/custom/raw_memory"},
       rclcpp::Parameter{"marker_topic", "/custom/markers"},
       rclcpp::Parameter{"px4_local_position_topic", "/custom/local_position"},
       rclcpp::Parameter{"px4_vehicle_attitude_topic", "/custom/attitude"},
       rclcpp::Parameter{"scan_yaw_offset_rad", 0.25},
       rclcpp::Parameter{"lidar_mount_roll_rad", 0.1},
       rclcpp::Parameter{"lidar_mount_pitch_rad", 0.2},
       rclcpp::Parameter{"lidar_mount_yaw_rad", 0.3},
       rclcpp::Parameter{"compensate_lidar_attitude", true},
       rclcpp::Parameter{"use_px4_heading_for_scan", true}});

  const LidarDebugNodeConfig config = loadLidarDebugNodeConfig(*node);

  EXPECT_EQ(config.output_dir, "log/custom_lidar");
  EXPECT_EQ(config.topics.lidar, "/custom/scan");
  EXPECT_EQ(config.topics.prohibited_grid, "/custom/prohibited");
  EXPECT_EQ(config.topics.memory_grid, "/custom/memory");
  EXPECT_EQ(config.topics.path, "/custom/path");
  EXPECT_EQ(config.topics.pointcloud, "/custom/current_points");
  EXPECT_EQ(config.topics.remembered_pointcloud, "/custom/remembered");
  EXPECT_EQ(config.topics.prohibited_pointcloud, "/custom/prohibited_points");
  EXPECT_EQ(config.topics.raw_memory_pointcloud, "/custom/raw_memory");
  EXPECT_EQ(config.topics.marker, "/custom/markers");
  EXPECT_EQ(config.topics.px4_local_position, "/custom/local_position");
  EXPECT_EQ(config.topics.px4_vehicle_attitude, "/custom/attitude");
  EXPECT_DOUBLE_EQ(config.scan_yaw_offset_rad, 0.25);
  EXPECT_DOUBLE_EQ(config.lidar_mount_roll_rad, 0.1);
  EXPECT_DOUBLE_EQ(config.lidar_mount_pitch_rad, 0.2);
  EXPECT_DOUBLE_EQ(config.lidar_mount_yaw_rad, 0.3);
  EXPECT_TRUE(config.compensate_lidar_attitude);
  EXPECT_TRUE(config.use_px4_heading_for_scan);
}

TEST_F(LidarDebugNodeConfigTest, ClampsLoaderValues) {
  const auto node = makeNode("lidar_debug_node_config_clamps",
                             {rclcpp::Parameter{"snapshot_period_s", 0.0},
                              rclcpp::Parameter{"image_size_px", 99},
                              rclcpp::Parameter{"view_radius_m", 1.0},
                              rclcpp::Parameter{"max_lidar_range_m", -5.0},
                              rclcpp::Parameter{"range_hit_epsilon_m", -1.0},
                              rclcpp::Parameter{"lidar_pose_latency_s", 4.0},
                              rclcpp::Parameter{"lidar_scan_duration_override_s", 4.0},
                              rclcpp::Parameter{"beam_csv_stride", 0},
                              rclcpp::Parameter{"max_logged_hit_points", 999999},
                              rclcpp::Parameter{"hit_memory_resolution_m", 0.01},
                              rclcpp::Parameter{"min_remember_altitude_m", -2.0},
                              rclcpp::Parameter{"max_remembered_hit_points", 0}});

  const LidarDebugNodeConfig config = loadLidarDebugNodeConfig(*node);

  EXPECT_DOUBLE_EQ(config.snapshot_period_s, 0.1);
  EXPECT_EQ(config.image_size_px, 200);
  EXPECT_DOUBLE_EQ(config.view_radius_m, 5.0);
  EXPECT_DOUBLE_EQ(config.max_lidar_range_m, 1.0);
  EXPECT_DOUBLE_EQ(config.range_hit_epsilon_m, 0.0);
  EXPECT_DOUBLE_EQ(config.lidar_pose_latency_s, 1.0);
  EXPECT_DOUBLE_EQ(config.lidar_scan_duration_override_s, 1.0);
  EXPECT_EQ(config.beam_csv_stride, 1U);
  EXPECT_EQ(config.max_logged_hit_points, 100000U);
  EXPECT_DOUBLE_EQ(config.hit_memory_resolution_m, 0.05);
  EXPECT_DOUBLE_EQ(config.min_remember_altitude_m, 0.0);
  EXPECT_EQ(config.max_remembered_hit_points, 1U);
}

} // namespace drone_city_nav
