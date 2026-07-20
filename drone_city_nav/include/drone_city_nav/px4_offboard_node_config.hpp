#pragma once

#include "drone_city_nav/offboard_vertical_follower.hpp"
#include "drone_city_nav/trajectory_horizontal_handover.hpp"
#include "drone_city_nav/trajectory_update_continuity.hpp"
#include "drone_city_nav/types.hpp"
#include "drone_city_nav/velocity_control_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace rclcpp {
class Node;
} // namespace rclcpp

namespace drone_city_nav {

struct Px4OffboardNodeTopics {
  std::string path{"/drone_city_nav/path"};
  std::string path_id{"/drone_city_nav/path_id"};
  std::string trajectory_diagnostics{"/drone_city_nav/trajectory_diagnostics"};
  std::string replan_blocker{"/drone_city_nav/replan_blocker"};
  std::string px4_local_position{"/fmu/out/vehicle_local_position"};
  std::string px4_vehicle_attitude{"/fmu/out/vehicle_attitude"};
  std::string px4_vehicle_status{"/fmu/out/vehicle_status"};
  std::string prohibited_grid{"/drone_city_nav/prohibited_grid"};
  std::string offboard_control_mode{"/fmu/in/offboard_control_mode"};
  std::string trajectory_setpoint{"/fmu/in/trajectory_setpoint"};
  std::string vehicle_command{"/fmu/in/vehicle_command"};
  std::string final_trajectory_debug{"/drone_city_nav/final_trajectory_path"};
  std::string offboard_debug_marker{"/drone_city_nav/offboard_debug_markers"};
};

struct Px4OffboardNodeConfig {
  double initial_altitude_m{12.0};
  double min_navigation_altitude_m{0.0};
  double takeoff_hover_s{2.0};
  double acceptance_radius_m{1.5};
  double diagnostic_turn_preview_distance_m{32.0};
  std::int64_t max_clearance_grid_staleness_ns{1'500'000'000};
  double command_resend_period_s{2.0};
  std::int64_t max_pose_staleness_ns{1'000'000'000};
  double final_trajectory_debug_sample_step_m{1.0};
  double trajectory_update_max_start_cross_track_m{8.0};
  bool safe_trajectory_truncation_enabled{true};
  double safe_trajectory_truncation_margin_m{10.0};
  HorizontalTrajectoryHandoverConfig trajectory_handover{};
  TrajectoryContinuityThresholds trajectory_continuity{};
  double altitude_hold_kp{0.5};
  std::int64_t telemetry_log_period_ns{500'000'000};
  bool flight_blackbox_enabled{true};
  int warmup_setpoints{20};
  bool auto_arm{true};
  bool auto_offboard{true};
  Point2 px4_local_origin{};
  Point2 mission_goal{85.0, 0.0};
  double hold_x_m{0.0};
  double hold_y_m{0.0};
  bool rviz_drone_follow_tf_enabled{true};
  std::string rviz_drone_follow_parent_frame{"gazebo_map"};
  std::string rviz_drone_follow_frame{"drone_follow"};
  std::uint8_t target_system{1U};
  std::uint8_t target_component{1U};
  std::uint8_t source_system{1U};
  std::uint16_t source_component{1U};
  VelocityFollowerConfig velocity_follower{};
  VerticalFollowerConfig vertical_follower{};
  std::string flight_blackbox_path{"log/offboard_blackbox.jsonl"};
  Px4OffboardNodeTopics topics{};
};

[[nodiscard]] double boundedFiniteDouble(double value, double fallback,
                                         double min_value, double max_value) noexcept;

[[nodiscard]] std::uint8_t boundedUint8(std::int64_t value);

[[nodiscard]] std::uint16_t boundedUint16(std::int64_t value);

void sanitizePx4OffboardNodeConfig(Px4OffboardNodeConfig& config);

[[nodiscard]] Px4OffboardNodeConfig loadPx4OffboardNodeConfig(rclcpp::Node& node);

} // namespace drone_city_nav
