#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace drone_city_nav {

TEST(Px4OffboardConfig, DefaultYamlKeepsPlannerOwnedRacingLineParameters) {
  const std::string config_path =
      std::string{DRONE_CITY_NAV_SOURCE_DIR} + "/config/urban_mvp.yaml";
  std::ifstream stream{config_path};
  ASSERT_TRUE(stream.is_open()) << config_path;

  std::ostringstream buffer;
  buffer << stream.rdbuf();
  const std::string yaml = buffer.str();

  EXPECT_NE(yaml.find("racing_line_weight_time:"), std::string::npos);
  EXPECT_NE(yaml.find("racing_line_weight_length: 0.02"), std::string::npos);
  EXPECT_EQ(yaml.find("racing_line_weight_edge_margin:"), std::string::npos);
  EXPECT_EQ(yaml.find("racing_line_desired_edge_margin_m:"), std::string::npos);
  EXPECT_NE(yaml.find("racing_line_regularization_iterations:"), std::string::npos);
  EXPECT_NE(yaml.find("racing_line_regularization_max_time_regression_s:"),
            std::string::npos);
  EXPECT_NE(yaml.find("turn_smoothing_trigger_heading_delta_deg: 37.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("turn_smoothing_entry_distance_m: 45.0"), std::string::npos);
  EXPECT_NE(yaml.find("turn_smoothing_outer_bias_ratio: 0.45"), std::string::npos);
  EXPECT_NE(yaml.find("turn_smoothing_max_passes: 8"), std::string::npos);
  EXPECT_NE(yaml.find("speed_profile_decel_mps2: 2.0"), std::string::npos);
  EXPECT_NE(yaml.find("speed_profile_lookahead_time_s: 1.0"), std::string::npos);
  EXPECT_NE(yaml.find("speed_profile_lookahead_min_m: 5.0"), std::string::npos);
  EXPECT_NE(yaml.find("speed_profile_lookahead_max_m: 35.0"), std::string::npos);
  EXPECT_NE(yaml.find("tracking_prediction_horizon_s: 0.35"), std::string::npos);
  EXPECT_NE(yaml.find("trajectory_diagnostics_topic: "
                      "/drone_city_nav/trajectory_diagnostics"),
            std::string::npos);
  EXPECT_NE(yaml.find("cross_track_gain: 0.5"), std::string::npos);
  EXPECT_NE(yaml.find("cross_track_derivative_gain: 0.5"), std::string::npos);
  EXPECT_NE(yaml.find("max_lateral_control_angle_deg: 55.0"), std::string::npos);
  EXPECT_NE(yaml.find("max_lateral_control_rate_mps2: 5.0"), std::string::npos);
  EXPECT_NE(yaml.find("velocity_lateral_response_accel_mps2: 5.0"), std::string::npos);
  EXPECT_EQ(yaml.find("cross_track_speed_guard_start_m:"), std::string::npos);
  EXPECT_EQ(yaml.find("cross_track_speed_guard_full_m:"), std::string::npos);
  EXPECT_EQ(yaml.find("cross_track_speed_guard_min_factor:"), std::string::npos);
  EXPECT_NE(yaml.find("curvature_feedforward_time_s: 0.25"), std::string::npos);
  EXPECT_NE(yaml.find("curvature_feedforward_deadband_angle_deg: 2.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("curvature_feedforward_full_angle_deg: 8.0"), std::string::npos);
  EXPECT_NE(yaml.find("max_curvature_feedforward_angle_deg: 30.0"), std::string::npos);
  EXPECT_NE(yaml.find("max_velocity_jerk_mps3: 12.0"), std::string::npos);
  EXPECT_NE(yaml.find("max_lateral_velocity_jerk_mps3: 14.0"), std::string::npos);
  EXPECT_NE(yaml.find("speed_aware_derivative_damping_min_speed_mps: 8.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("speed_aware_derivative_damping_full_speed_mps: 20.0"),
            std::string::npos);
  EXPECT_NE(yaml.find("speed_aware_derivative_damping_max_factor: 1.5"),
            std::string::npos);
  EXPECT_NE(yaml.find("adaptive_lateral_response_scale_m: 3.0"), std::string::npos);
  EXPECT_NE(yaml.find("adaptive_lateral_response_max_factor: 1.2"), std::string::npos);
  EXPECT_NE(yaml.find("trajectory_update_max_start_cross_track_m: 8.0"),
            std::string::npos);
  EXPECT_EQ(yaml.find("max_cross_track_correction_angle_deg:"), std::string::npos);
  EXPECT_EQ(yaml.find("max_cross_track_correction_rate_mps2:"), std::string::npos);
  EXPECT_EQ(yaml.find("curvature_velocity_anticipation_time_s:"), std::string::npos);
  EXPECT_EQ(yaml.find("max_curvature_velocity_anticipation_angle_deg:"),
            std::string::npos);
  EXPECT_EQ(yaml.find("max_curvature_velocity_anticipation_rate_mps2:"),
            std::string::npos);
  EXPECT_EQ(yaml.find("max_feedforward_accel_mps2:"), std::string::npos);
  EXPECT_EQ(yaml.find("max_feedforward_jerk_mps3:"), std::string::npos);
  EXPECT_EQ(yaml.find("acceleration_feedforward_scale:"), std::string::npos);
  EXPECT_EQ(yaml.find("executable_trajectory_max_step_m:"), std::string::npos);
  EXPECT_EQ(yaml.find("trajectory_result_stale_cross_track_m:"), std::string::npos);
}

} // namespace drone_city_nav
