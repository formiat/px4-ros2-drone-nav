#include "drone_city_nav/px4_offboard_node_config.hpp"

#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

class Px4OffboardNodeConfigTest : public ::testing::Test {
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
  config.trajectory_update_max_start_cross_track_m =
      std::numeric_limits<double>::infinity();
  config.velocity_follower.cruise_speed_mps = 10.0;
  config.velocity_follower.min_turn_speed_mps = 20.0;
  config.velocity_follower.speed_profile_lookahead_min_m = 8.0;
  config.velocity_follower.speed_profile_lookahead_max_m = 3.0;
  config.velocity_follower.control_tangent_smoothing_back_m =
      std::numeric_limits<double>::infinity();
  config.velocity_follower.control_tangent_smoothing_forward_m =
      std::numeric_limits<double>::quiet_NaN();
  config.velocity_follower.control_tangent_smoothing_max_heading_span_rad =
      std::numeric_limits<double>::infinity();
  config.velocity_follower.control_tangent_smoothing_max_abs_curvature_1pm =
      std::numeric_limits<double>::quiet_NaN();

  sanitizePx4OffboardNodeConfig(config);

  EXPECT_DOUBLE_EQ(config.min_navigation_altitude_m, 12.0);
  EXPECT_DOUBLE_EQ(config.takeoff_hover_s, 0.0);
  EXPECT_DOUBLE_EQ(config.acceptance_radius_m, 1.5);
  EXPECT_DOUBLE_EQ(config.turn_preview_distance_m, 500.0);
  EXPECT_DOUBLE_EQ(config.command_resend_period_s, 0.05);
  EXPECT_DOUBLE_EQ(config.trajectory_update_max_start_cross_track_m, 8.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.min_turn_speed_mps, 10.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.speed_profile_lookahead_max_m, 8.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.control_tangent_smoothing_back_m, 8.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.control_tangent_smoothing_forward_m, 18.0);
  EXPECT_DOUBLE_EQ(
      config.velocity_follower.control_tangent_smoothing_max_heading_span_rad,
      12.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(
      config.velocity_follower.control_tangent_smoothing_max_abs_curvature_1pm, 0.015);
  EXPECT_DOUBLE_EQ(config.velocity_follower.final_acceptance_radius_m, 1.5);
}

TEST_F(Px4OffboardNodeConfigTest, LoadsDocumentedDefaults) {
  const auto node = makeNode("px4_offboard_node_config_defaults");

  const Px4OffboardNodeConfig config = loadPx4OffboardNodeConfig(*node);

  EXPECT_DOUBLE_EQ(config.cruise_altitude_m, 12.0);
  EXPECT_DOUBLE_EQ(config.acceptance_radius_m, 1.5);
  EXPECT_DOUBLE_EQ(config.velocity_follower.cruise_speed_mps, 12.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.min_turn_speed_mps, 2.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.cross_track_derivative_gain, 0.5);
  EXPECT_DOUBLE_EQ(config.velocity_follower.max_lateral_control_angle_rad,
                   55.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.max_lateral_control_rate_mps2, 5.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.velocity_lateral_response_accel_mps2, 5.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.curvature_feedforward_time_s, 0.25);
  EXPECT_DOUBLE_EQ(config.velocity_follower.curvature_feedforward_deadband_angle_rad,
                   2.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.curvature_feedforward_full_angle_rad,
                   8.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.max_curvature_feedforward_angle_rad,
                   30.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.max_lateral_velocity_jerk_mps3, 14.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.lateral_smoothing_min_speed_mps, 8.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.lateral_smoothing_full_speed_mps, 20.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.lateral_smoothing_max_factor, 1.6);
  EXPECT_DOUBLE_EQ(config.velocity_follower.max_velocity_heading_rate_rad_s, 1.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.min_velocity_heading_rate_rad_s, 0.8);
  EXPECT_DOUBLE_EQ(config.velocity_follower.lateral_zero_crossing_max_cross_track_m,
                   1.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.lateral_zero_crossing_max_growth_m, 0.1);
  EXPECT_DOUBLE_EQ(
      config.velocity_follower.speed_aware_derivative_damping_min_speed_mps, 8.0);
  EXPECT_DOUBLE_EQ(
      config.velocity_follower.speed_aware_derivative_damping_full_speed_mps, 20.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.speed_aware_derivative_damping_max_factor,
                   1.5);
  EXPECT_DOUBLE_EQ(config.velocity_follower.control_tangent_smoothing_back_m, 8.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.control_tangent_smoothing_forward_m, 18.0);
  EXPECT_DOUBLE_EQ(
      config.velocity_follower.control_tangent_smoothing_max_heading_span_rad,
      12.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(
      config.velocity_follower.control_tangent_smoothing_max_abs_curvature_1pm, 0.015);
  EXPECT_DOUBLE_EQ(config.velocity_follower.adaptive_lateral_response_max_factor, 1.2);
  EXPECT_EQ(config.flight_blackbox_path, "log/offboard_blackbox.jsonl");
  EXPECT_TRUE(config.flight_blackbox_enabled);
  EXPECT_DOUBLE_EQ(config.trajectory_update_max_start_cross_track_m, 8.0);
  EXPECT_EQ(config.topics.path, "/drone_city_nav/path");
  EXPECT_EQ(config.topics.trajectory_diagnostics,
            "/drone_city_nav/trajectory_diagnostics");
  EXPECT_EQ(config.topics.px4_local_position, "/fmu/out/vehicle_local_position");
  EXPECT_EQ(config.topics.offboard_control_mode, "/fmu/in/offboard_control_mode");
}

TEST_F(Px4OffboardNodeConfigTest, LoadsCustomTopicsAndBlackboxPath) {
  const auto node = makeNode(
      "px4_offboard_node_config_custom",
      {rclcpp::Parameter{"path_topic", "/custom/path"},
       rclcpp::Parameter{"path_id_topic", "/custom/path_id"},
       rclcpp::Parameter{"trajectory_diagnostics_topic", "/custom/diagnostics"},
       rclcpp::Parameter{"px4_local_position_topic", "/custom/local_position"},
       rclcpp::Parameter{"px4_vehicle_attitude_topic", "/custom/attitude"},
       rclcpp::Parameter{"px4_vehicle_status_topic", "/custom/status"},
       rclcpp::Parameter{"emergency_stop_topic", "/custom/estop"},
       rclcpp::Parameter{"prohibited_grid_topic", "/custom/prohibited"},
       rclcpp::Parameter{"offboard_control_mode_topic", "/custom/offboard_mode"},
       rclcpp::Parameter{"trajectory_setpoint_topic", "/custom/setpoint"},
       rclcpp::Parameter{"vehicle_command_topic", "/custom/command"},
       rclcpp::Parameter{"final_trajectory_debug_topic", "/custom/final_path"},
       rclcpp::Parameter{"offboard_debug_marker_topic", "/custom/markers"},
       rclcpp::Parameter{"flight_blackbox_path", "log/custom_blackbox.jsonl"},
       rclcpp::Parameter{"flight_blackbox_enabled", false}});

  const Px4OffboardNodeConfig config = loadPx4OffboardNodeConfig(*node);

  EXPECT_EQ(config.topics.path, "/custom/path");
  EXPECT_EQ(config.topics.path_id, "/custom/path_id");
  EXPECT_EQ(config.topics.trajectory_diagnostics, "/custom/diagnostics");
  EXPECT_EQ(config.topics.px4_local_position, "/custom/local_position");
  EXPECT_EQ(config.topics.px4_vehicle_attitude, "/custom/attitude");
  EXPECT_EQ(config.topics.px4_vehicle_status, "/custom/status");
  EXPECT_EQ(config.topics.emergency_stop, "/custom/estop");
  EXPECT_EQ(config.topics.prohibited_grid, "/custom/prohibited");
  EXPECT_EQ(config.topics.offboard_control_mode, "/custom/offboard_mode");
  EXPECT_EQ(config.topics.trajectory_setpoint, "/custom/setpoint");
  EXPECT_EQ(config.topics.vehicle_command, "/custom/command");
  EXPECT_EQ(config.topics.final_trajectory_debug, "/custom/final_path");
  EXPECT_EQ(config.topics.offboard_debug_marker, "/custom/markers");
  EXPECT_EQ(config.flight_blackbox_path, "log/custom_blackbox.jsonl");
  EXPECT_FALSE(config.flight_blackbox_enabled);
}

TEST_F(Px4OffboardNodeConfigTest, ClampsLoaderValues) {
  const auto node = makeNode(
      "px4_offboard_node_config_clamps",
      {rclcpp::Parameter{"min_navigation_altitude_m", 100.0},
       rclcpp::Parameter{"takeoff_hover_s", -10.0},
       rclcpp::Parameter{"acceptance_radius_m", 500.0},
       rclcpp::Parameter{"turn_preview_distance_m", 999.0},
       rclcpp::Parameter{"max_clearance_grid_staleness_s", 9999.0},
       rclcpp::Parameter{"max_pose_staleness_s", -1.0},
       rclcpp::Parameter{"cruise_speed_mps", 6.0},
       rclcpp::Parameter{"min_turn_speed_mps", 10.0},
       rclcpp::Parameter{"speed_profile_lookahead_min_m", 20.0},
       rclcpp::Parameter{"speed_profile_lookahead_max_m", 5.0},
       rclcpp::Parameter{"max_lateral_control_angle_deg", 500.0},
       rclcpp::Parameter{"curvature_feedforward_deadband_angle_deg", 20.0},
       rclcpp::Parameter{"curvature_feedforward_full_angle_deg", 10.0},
       rclcpp::Parameter{"max_curvature_feedforward_angle_deg", 500.0},
       rclcpp::Parameter{"speed_aware_derivative_damping_min_speed_mps", 30.0},
       rclcpp::Parameter{"speed_aware_derivative_damping_full_speed_mps", 10.0},
       rclcpp::Parameter{"speed_aware_derivative_damping_max_factor", 0.5},
       rclcpp::Parameter{"control_tangent_smoothing_back_m", -1.0},
       rclcpp::Parameter{"control_tangent_smoothing_forward_m", 2000.0},
       rclcpp::Parameter{"control_tangent_smoothing_max_heading_span_deg", 500.0},
       rclcpp::Parameter{"control_tangent_smoothing_max_abs_curvature_1pm", -1.0},
       rclcpp::Parameter{"final_trajectory_debug_sample_step_m", 100.0},
       rclcpp::Parameter{"trajectory_update_max_start_cross_track_m", 2000.0},
       rclcpp::Parameter{"telemetry_log_period_s", 0.01},
       rclcpp::Parameter{"command_resend_period_s", 0.0},
       rclcpp::Parameter{"target_system", 999},
       rclcpp::Parameter{"source_component", 999999}});

  const Px4OffboardNodeConfig config = loadPx4OffboardNodeConfig(*node);

  EXPECT_DOUBLE_EQ(config.min_navigation_altitude_m, 12.0);
  EXPECT_DOUBLE_EQ(config.takeoff_hover_s, 0.0);
  EXPECT_DOUBLE_EQ(config.acceptance_radius_m, 100.0);
  EXPECT_DOUBLE_EQ(config.turn_preview_distance_m, 500.0);
  EXPECT_EQ(config.max_clearance_grid_staleness_ns, 3'600'000'000'000LL);
  EXPECT_EQ(config.max_pose_staleness_ns, 0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.min_turn_speed_mps, 6.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.speed_profile_lookahead_max_m, 20.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.max_lateral_control_angle_rad,
                   std::numbers::pi / 2.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.curvature_feedforward_deadband_angle_rad,
                   20.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.curvature_feedforward_full_angle_rad,
                   20.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.max_curvature_feedforward_angle_rad,
                   std::numbers::pi / 2.0);
  EXPECT_DOUBLE_EQ(
      config.velocity_follower.speed_aware_derivative_damping_min_speed_mps, 30.0);
  EXPECT_DOUBLE_EQ(
      config.velocity_follower.speed_aware_derivative_damping_full_speed_mps, 30.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.speed_aware_derivative_damping_max_factor,
                   1.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.control_tangent_smoothing_back_m, 0.0);
  EXPECT_DOUBLE_EQ(config.velocity_follower.control_tangent_smoothing_forward_m,
                   1000.0);
  EXPECT_DOUBLE_EQ(
      config.velocity_follower.control_tangent_smoothing_max_heading_span_rad,
      std::numbers::pi);
  EXPECT_DOUBLE_EQ(
      config.velocity_follower.control_tangent_smoothing_max_abs_curvature_1pm, 0.0);
  EXPECT_DOUBLE_EQ(config.final_trajectory_debug_sample_step_m, 20.0);
  EXPECT_DOUBLE_EQ(config.trajectory_update_max_start_cross_track_m, 1000.0);
  EXPECT_EQ(config.telemetry_log_period_ns, 100'000'000LL);
  EXPECT_DOUBLE_EQ(config.command_resend_period_s, 0.05);
  EXPECT_EQ(config.target_system, 255U);
  EXPECT_EQ(config.source_component, 65535U);
}

} // namespace drone_city_nav
