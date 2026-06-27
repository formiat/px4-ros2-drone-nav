#include "px4_offboard_node.hpp"

namespace drone_city_nav {

Px4OffboardNode::Px4OffboardNode()
    : Node{"px4_offboard_node"} {
  cruise_altitude_m_ = declare_parameter<double>("cruise_altitude_m", 12.0);
  min_navigation_altitude_m_ =
      std::clamp(declare_parameter<double>("min_navigation_altitude_m", 0.0), 0.0,
                 std::abs(cruise_altitude_m_));
  takeoff_hover_s_ =
      std::clamp(declare_parameter<double>("takeoff_hover_s", 2.0), 0.0, 30.0);
  face_target_yaw_ = declare_parameter<bool>("face_target_yaw", false);
  acceptance_radius_m_ = declare_parameter<double>("acceptance_radius_m", 1.5);
  turn_preview_distance_m_ = std::clamp(
      declare_parameter<double>("turn_preview_distance_m", 32.0), 0.0, 500.0);
  max_clearance_grid_staleness_ns_ = static_cast<std::int64_t>(
      std::clamp<double>(
          declare_parameter<double>("max_clearance_grid_staleness_s", 1.5), 0.0,
          3600.0) *
      1.0e9);
  max_pose_staleness_ns_ = static_cast<std::int64_t>(
      boundedFiniteDouble(declare_parameter<double>("max_pose_staleness_s", 1.0), 1.0,
                          0.0, 3600.0) *
      1.0e9);
  velocity_follower_config_.cruise_speed_mps =
      std::clamp(declare_parameter<double>("cruise_speed_mps", 12.0), 0.0, 100.0);
  velocity_follower_config_.min_turn_speed_mps =
      std::clamp(declare_parameter<double>("min_turn_speed_mps", 2.0), 0.0,
                 velocity_follower_config_.cruise_speed_mps);
  velocity_follower_config_.max_accel_mps2 =
      std::clamp(declare_parameter<double>("max_accel_mps2", 3.0), 0.0, 100.0);
  velocity_follower_config_.max_decel_mps2 =
      std::clamp(declare_parameter<double>("max_decel_mps2", 4.0), 0.0, 100.0);
  velocity_follower_config_.max_lateral_accel_mps2 =
      std::clamp(declare_parameter<double>("max_lateral_accel_mps2", 3.0), 0.0, 100.0);
  velocity_follower_config_.speed_profile_decel_mps2 = std::clamp(
      declare_parameter<double>("speed_profile_decel_mps2", 2.0), 0.0, 100.0);
  velocity_follower_config_.speed_profile_sample_step_m = std::clamp(
      declare_parameter<double>("speed_profile_sample_step_m", 1.0), 0.1, 10.0);
  velocity_follower_config_.speed_profile_lookahead_time_s = std::clamp(
      declare_parameter<double>("speed_profile_lookahead_time_s", 1.0), 0.0, 30.0);
  velocity_follower_config_.speed_profile_lookahead_min_m = std::clamp(
      declare_parameter<double>("speed_profile_lookahead_min_m", 5.0), 0.0, 500.0);
  const double requested_speed_profile_lookahead_max_m = std::clamp(
      declare_parameter<double>("speed_profile_lookahead_max_m", 35.0), 0.0, 5000.0);
  velocity_follower_config_.speed_profile_lookahead_max_m =
      std::max(requested_speed_profile_lookahead_max_m,
               velocity_follower_config_.speed_profile_lookahead_min_m);
  velocity_follower_config_.cross_track_gain =
      std::clamp(declare_parameter<double>("cross_track_gain", 0.5), 0.0, 10.0);
  velocity_follower_config_.cross_track_derivative_gain = std::clamp(
      declare_parameter<double>("cross_track_derivative_gain", 0.8), 0.0, 10.0);
  velocity_follower_config_.tracking_prediction_horizon_s = std::clamp(
      declare_parameter<double>("tracking_prediction_horizon_s", 0.45), 0.0, 2.0);
  velocity_follower_config_.max_lateral_control_angle_rad =
      std::clamp(radiansFromDegrees(
                     declare_parameter<double>("max_lateral_control_angle_deg", 55.0)),
                 0.0, std::numbers::pi / 2.0);
  velocity_follower_config_.max_lateral_control_rate_mps2 = std::clamp(
      declare_parameter<double>("max_lateral_control_rate_mps2", 8.0), 0.0, 100.0);
  velocity_follower_config_.curvature_feedforward_time_s = std::clamp(
      declare_parameter<double>("curvature_feedforward_time_s", 0.5), 0.0, 10.0);
  velocity_follower_config_.max_curvature_feedforward_angle_rad =
      std::clamp(radiansFromDegrees(declare_parameter<double>(
                     "max_curvature_feedforward_angle_deg", 40.0)),
                 0.0, std::numbers::pi / 2.0);
  velocity_follower_config_.max_velocity_jerk_mps3 = std::clamp(
      declare_parameter<double>("max_velocity_jerk_mps3", 12.0), 0.0, 1000.0);
  velocity_follower_config_.final_acceptance_radius_m = acceptance_radius_m_;
  velocity_follower_config_.final_hold_max_speed_mps = std::clamp(
      declare_parameter<double>("final_hold_max_speed_mps", 0.8), 0.0, 100.0);
  final_trajectory_debug_topic_ = declare_parameter<std::string>(
      "final_trajectory_debug_topic", "/drone_city_nav/final_trajectory_path");
  final_trajectory_debug_sample_step_m_ =
      std::clamp(declare_parameter<double>("final_trajectory_debug_sample_step_m", 1.0),
                 0.1, 20.0);
  offboard_debug_marker_topic_ = declare_parameter<std::string>(
      "offboard_debug_marker_topic", "/drone_city_nav/offboard_debug_markers");
  altitude_hold_kp_ =
      std::clamp(declare_parameter<double>("altitude_hold_kp", 0.5), 0.0, 10.0);
  max_vertical_speed_mps_ =
      std::clamp(declare_parameter<double>("max_vertical_speed_mps", 2.0), 0.0, 20.0);
  telemetry_log_period_ns_ = static_cast<std::int64_t>(
      std::clamp(declare_parameter<double>("telemetry_log_period_s", 0.5), 0.1, 60.0) *
      1.0e9);
  flight_blackbox_enabled_ = declare_parameter<bool>("flight_blackbox_enabled", true);
  flight_blackbox_path_ = declare_parameter<std::string>("flight_blackbox_path",
                                                         "log/offboard_blackbox.jsonl");
  warmup_setpoints_ = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("warmup_setpoints", 20), 1, 100000));
  auto_arm_ = declare_parameter<bool>("auto_arm", true);
  auto_offboard_ = declare_parameter<bool>("auto_offboard", true);
  const double requested_command_resend_period_s =
      declare_parameter<double>("command_resend_period_s", 2.0);
  command_resend_period_s_ =
      boundedFiniteDouble(requested_command_resend_period_s, 2.0, 0.05, 60.0);
  if (!std::isfinite(requested_command_resend_period_s) ||
      requested_command_resend_period_s != command_resend_period_s_) {
    RCLCPP_WARN(get_logger(),
                "Sanitized command_resend_period_s: requested=%.3f final=%.3f "
                "allowed_range=[0.050, 60.000]",
                requested_command_resend_period_s, command_resend_period_s_);
  }
  px4_local_origin_ = Point2{declare_parameter<double>("px4_local_origin_x_m", 0.0),
                             declare_parameter<double>("px4_local_origin_y_m", 0.0)};
  mission_goal_ = Point2{declare_parameter<double>("goal_x_m", 85.0),
                         declare_parameter<double>("goal_y_m", 0.0)};
  hold_x_m_ = declare_parameter<double>("hold_x_m", 0.0);
  hold_y_m_ = declare_parameter<double>("hold_y_m", 0.0);
  target_system_ = boundedUint8(declare_parameter<std::int64_t>("target_system", 1));
  target_component_ =
      boundedUint8(declare_parameter<std::int64_t>("target_component", 1));
  source_system_ = boundedUint8(declare_parameter<std::int64_t>("source_system", 1));
  source_component_ =
      boundedUint16(declare_parameter<std::int64_t>("source_component", 1));

  const std::string path_topic =
      declare_parameter<std::string>("path_topic", "/drone_city_nav/path");
  const std::string path_id_topic =
      declare_parameter<std::string>("path_id_topic", "/drone_city_nav/path_id");
  const std::string trajectory_diagnostics_topic = declare_parameter<std::string>(
      "trajectory_diagnostics_topic", "/drone_city_nav/trajectory_diagnostics");
  const std::string local_position_topic = declare_parameter<std::string>(
      "px4_local_position_topic", "/fmu/out/vehicle_local_position");
  const std::string attitude_topic = declare_parameter<std::string>(
      "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
  const std::string vehicle_status_topic = declare_parameter<std::string>(
      "px4_vehicle_status_topic", "/fmu/out/vehicle_status");
  const std::string emergency_stop_topic = declare_parameter<std::string>(
      "emergency_stop_topic", "/drone_city_nav/emergency_stop");
  const std::string prohibited_grid_topic = declare_parameter<std::string>(
      "prohibited_grid_topic", "/drone_city_nav/prohibited_grid");

  const auto px4_qos =
      rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
  const auto emergency_stop_qos = rclcpp::QoS{1}.reliable().durability_volatile();
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS{1}.reliable(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) { onPath(*msg); });
  path_id_sub_ = create_subscription<std_msgs::msg::UInt64>(
      path_id_topic, rclcpp::QoS{1}.reliable(),
      [this](const std_msgs::msg::UInt64::SharedPtr msg) { onPathId(*msg); });
  trajectory_diagnostics_sub_ = create_subscription<std_msgs::msg::String>(
      trajectory_diagnostics_topic, rclcpp::QoS{1}.reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        onTrajectoryDiagnostics(*msg);
      });
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      local_position_topic, px4_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        onLocalPosition(*msg);
      });
  attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      attitude_topic, px4_qos,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        onAttitude(*msg);
      });
  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic, px4_qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        onVehicleStatus(*msg);
      });
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic, emergency_stop_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { onEmergencyStop(*msg); });
  prohibited_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      prohibited_grid_topic, rclcpp::QoS{1}.transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        onProhibitedGrid(*msg);
      });

  offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      declare_parameter<std::string>("offboard_control_mode_topic",
                                     "/fmu/in/offboard_control_mode"),
      px4_qos);
  trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      declare_parameter<std::string>("trajectory_setpoint_topic",
                                     "/fmu/in/trajectory_setpoint"),
      px4_qos);
  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      declare_parameter<std::string>("vehicle_command_topic",
                                     "/fmu/in/vehicle_command"),
      px4_qos);
  final_trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
      final_trajectory_debug_topic_, rclcpp::QoS{1}.transient_local());
  offboard_debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      offboard_debug_marker_topic_, rclcpp::QoS{1}.transient_local());

  timer_ = create_wall_timer(kControllerPeriod, [this]() { onTimer(); });
  last_command_time_ = now() - rclcpp::Duration::from_seconds(command_resend_period_s_);
  openFlightBlackbox();

  RCLCPP_INFO(
      get_logger(),
      "PX4 offboard node ready: altitude=%.1fm acceptance=%.1fm auto_arm=%s "
      "auto_offboard=%s min_navigation_altitude=%.1fm face_target_yaw=%s "
      "takeoff_hover=%.1fs "
      "turn_preview_distance=%.1fm "
      "velocity_cruise=final_trajectory_only cruise_speed=%.2fmps "
      "min_turn_speed=%.2fmps "
      "max_accel=%.2fmps2 max_decel=%.2fmps2 max_lateral_accel=%.2fmps2 "
      "speed_profile_decel=%.2fmps2 speed_profile_sample_step=%.2fm "
      "speed_profile_lookahead[time=%.2fs min=%.2fm max=%.2fm] "
      "final_hold_max_speed=%.2fmps cross_track_gain=%.2f "
      "tracking_prediction_horizon=%.2fs "
      "max_lateral_control_angle=%.1fdeg "
      "max_lateral_control_rate=%.2fmps2 "
      "curvature_feedforward[time=%.2fs max_angle=%.1fdeg] "
      "max_velocity_jerk=%.2fmps3 "
      "altitude_hold_kp=%.2f "
      "max_vertical_speed=%.2fmps "
      "executable_trajectory[source=planner_final_path final_topic='%s' "
      "debug_sample_step=%.2fm "
      "marker_topic='%s'] "
      "mission_goal=(%.1f, %.1f) "
      "px4_local_origin=(%.1f, %.1f) telemetry_log_period=%.2fs "
      "flight_blackbox=%s flight_blackbox_path='%s' "
      "max_pose_staleness=%.2fs command_resend_period=%.2fs",
      cruise_altitude_m_, acceptance_radius_m_, auto_arm_ ? "true" : "false",
      auto_offboard_ ? "true" : "false", min_navigation_altitude_m_,
      face_target_yaw_ ? "true" : "false", takeoff_hover_s_, turn_preview_distance_m_,
      velocity_follower_config_.cruise_speed_mps,
      velocity_follower_config_.min_turn_speed_mps,
      velocity_follower_config_.max_accel_mps2,
      velocity_follower_config_.max_decel_mps2,
      velocity_follower_config_.max_lateral_accel_mps2,
      velocity_follower_config_.speed_profile_decel_mps2,
      velocity_follower_config_.speed_profile_sample_step_m,
      velocity_follower_config_.speed_profile_lookahead_time_s,
      velocity_follower_config_.speed_profile_lookahead_min_m,
      velocity_follower_config_.speed_profile_lookahead_max_m,
      velocity_follower_config_.final_hold_max_speed_mps,
      velocity_follower_config_.cross_track_gain,
      velocity_follower_config_.tracking_prediction_horizon_s,
      radiansToDegrees(velocity_follower_config_.max_lateral_control_angle_rad),
      velocity_follower_config_.max_lateral_control_rate_mps2,
      velocity_follower_config_.curvature_feedforward_time_s,
      radiansToDegrees(velocity_follower_config_.max_curvature_feedforward_angle_rad),
      velocity_follower_config_.max_velocity_jerk_mps3, altitude_hold_kp_,
      max_vertical_speed_mps_, final_trajectory_debug_topic_.c_str(),
      final_trajectory_debug_sample_step_m_, offboard_debug_marker_topic_.c_str(),
      mission_goal_.x, mission_goal_.y, px4_local_origin_.x, px4_local_origin_.y,
      static_cast<double>(telemetry_log_period_ns_) / 1.0e9,
      flight_blackbox_enabled_ ? "true" : "false", flight_blackbox_path_.c_str(),
      static_cast<double>(max_pose_staleness_ns_) / 1.0e9, command_resend_period_s_);
  RCLCPP_INFO(get_logger(),
              "PX4 offboard subscriptions: path='%s' path_id='%s' local_position='%s' "
              "attitude='%s' vehicle_status='%s' emergency_stop='%s' "
              "prohibited_grid='%s'",
              path_topic.c_str(), path_id_topic.c_str(), local_position_topic.c_str(),
              attitude_topic.c_str(), vehicle_status_topic.c_str(),
              emergency_stop_topic.c_str(), prohibited_grid_topic.c_str());
}

} // namespace drone_city_nav
