#include "px4_offboard_node.hpp"

namespace drone_city_nav {

void Px4OffboardNode::applyConfig(const Px4OffboardNodeConfig& config) {
  initial_altitude_m_ = config.initial_altitude_m;
  min_navigation_altitude_m_ = config.min_navigation_altitude_m;
  takeoff_hover_s_ = config.takeoff_hover_s;
  acceptance_radius_m_ = config.acceptance_radius_m;
  diagnostic_turn_preview_distance_m_ = config.diagnostic_turn_preview_distance_m;
  max_clearance_grid_staleness_ns_ = config.max_clearance_grid_staleness_ns;
  max_pose_staleness_ns_ = config.max_pose_staleness_ns;
  velocity_follower_config_ = config.velocity_follower;
  vertical_follower_config_ = config.vertical_follower;
  final_trajectory_debug_topic_ = config.topics.final_trajectory_debug;
  final_trajectory_debug_sample_step_m_ = config.final_trajectory_debug_sample_step_m;
  trajectory_update_max_start_cross_track_m_ =
      config.trajectory_update_max_start_cross_track_m;
  safe_trajectory_truncation_enabled_ = config.safe_trajectory_truncation_enabled;
  safe_trajectory_truncation_margin_m_ = config.safe_trajectory_truncation_margin_m;
  trajectory_handover_config_ = config.trajectory_handover;
  trajectory_continuity_thresholds_ = config.trajectory_continuity;
  offboard_debug_marker_topic_ = config.topics.offboard_debug_marker;
  altitude_hold_kp_ = config.vertical_follower.altitude_feedback_kp_1ps;
  telemetry_log_period_ns_ = config.telemetry_log_period_ns;
  flight_blackbox_enabled_ = config.flight_blackbox_enabled;
  flight_blackbox_path_ = config.flight_blackbox_path;
  warmup_setpoints_ = config.warmup_setpoints;
  auto_arm_ = config.auto_arm;
  auto_offboard_ = config.auto_offboard;
  command_resend_period_s_ = config.command_resend_period_s;
  px4_local_origin_ = config.px4_local_origin;
  mission_goal_ = config.mission_goal;
  hold_x_m_ = config.hold_x_m;
  hold_y_m_ = config.hold_y_m;
  rviz_drone_follow_tf_enabled_ = config.rviz_drone_follow_tf_enabled;
  rviz_drone_follow_parent_frame_ = config.rviz_drone_follow_parent_frame;
  rviz_drone_follow_frame_ = config.rviz_drone_follow_frame;
  target_system_ = config.target_system;
  target_component_ = config.target_component;
  source_system_ = config.source_system;
  source_component_ = config.source_component;
}

Px4OffboardNode::Px4OffboardNode()
    : Node{"px4_offboard_node"} {
  const Px4OffboardNodeConfig config = loadPx4OffboardNodeConfig(*this);
  applyConfig(config);
  const Px4OffboardNodeTopics& topics = config.topics;

  const auto px4_qos =
      rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
  executable_trajectory_sub_ = create_subscription<msg::ExecutableTrajectory>(
      topics.executable_trajectory, rclcpp::QoS{1}.reliable(),
      [this](const msg::ExecutableTrajectory::SharedPtr msg) {
        onExecutableTrajectory(*msg);
      });
  path_id_sub_ = create_subscription<std_msgs::msg::UInt64>(
      topics.path_id, rclcpp::QoS{1}.reliable(),
      [this](const std_msgs::msg::UInt64::SharedPtr msg) { onPathId(*msg); });
  trajectory_diagnostics_sub_ = create_subscription<std_msgs::msg::String>(
      topics.trajectory_diagnostics, rclcpp::QoS{1}.reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        onTrajectoryDiagnostics(*msg);
      });
  replan_blocker_sub_ = create_subscription<msg::ReplanBlockerEvent>(
      topics.replan_blocker, rclcpp::QoS{1}.reliable(),
      [this](const msg::ReplanBlockerEvent::SharedPtr msg) { onReplanBlocker(*msg); });
  replan_truncation_pub_ = create_publisher<msg::ReplanTruncation>(
      topics.replan_truncation, rclcpp::QoS{1}.reliable());
  truncation_suffix_ack_pub_ = create_publisher<msg::TruncationSuffixAck>(
      topics.truncation_suffix_ack, rclcpp::QoS{10}.reliable());
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      topics.px4_local_position, px4_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        onLocalPosition(*msg);
      });
  attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      topics.px4_vehicle_attitude, px4_qos,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        onAttitude(*msg);
      });
  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      topics.px4_vehicle_status, px4_qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        onVehicleStatus(*msg);
      });
  crash_state_sub_ = create_subscription<msg::CrashState>(
      "/drone_city_nav/crash_state",
      rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local(),
      [this](const msg::CrashState::SharedPtr msg) { onCrashState(*msg); });
  prohibited_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      topics.prohibited_grid, rclcpp::QoS{1}.transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        onProhibitedGrid(*msg);
      });

  offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      topics.offboard_control_mode, px4_qos);
  trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      topics.trajectory_setpoint, px4_qos);
  vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(topics.vehicle_command, px4_qos);
  final_trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
      final_trajectory_debug_topic_, rclcpp::QoS{1}.transient_local());
  offboard_debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      offboard_debug_marker_topic_, rclcpp::QoS{1}.transient_local());
  if (rviz_drone_follow_tf_enabled_) {
    rviz_drone_follow_tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  timer_ = create_wall_timer(kControllerPeriod, [this]() { onTimer(); });
  last_command_time_ = now() - rclcpp::Duration::from_seconds(command_resend_period_s_);
  openFlightBlackbox();

  RCLCPP_INFO(
      get_logger(),
      "PX4 offboard node ready: altitude=%.1fm acceptance=%.1fm auto_arm=%s "
      "auto_offboard=%s min_navigation_altitude=%.1fm "
      "takeoff_hover=%.1fs "
      "diagnostic_turn_preview_distance=%.1fm "
      "velocity_cruise=final_trajectory_only cruise_speed=%.2fmps "
      "min_turn_speed=%.2fmps "
      "speed_profile_accel=%.2fmps2 speed_profile_decel=%.2fmps2 "
      "turn_speed_lateral_accel=%.2fmps2 "
      "setpoint_forward_accel=%.2fmps2 setpoint_forward_decel=%.2fmps2 "
      "speed_profile_sample_step=%.2fm "
      "speed_profile_lookahead[time=%.2fs min=%.2fm max=%.2fm] "
      "final_hold_max_speed=%.2fmps "
      "terminal_capture[radius=%.2fm gain=%.2f max_speed=%.2fmps "
      "position_max_entry=%.2fmps stuck_speed=%.2fmps] "
      "cross_track_gain=%.2f "
      "cross_track_p_gain_schedule[start=%.2fm full=%.2fm min=%.2f max=%.2f] "
      "tracking_prediction_horizon=%.2fs "
      "max_lateral_control_angle=%.1fdeg "
      "setpoint_lateral_response_accel=%.2fmps2 "
      "curvature_feedforward[time=%.2fs deadband=%.1fdeg full=%.1fdeg "
      "max_angle=%.1fdeg] "
      "cross_track_d_gain_schedule[min_speed=%.2fmps full_speed=%.2fmps "
      "max_factor=%.2f] "
      "control_tangent_smoothing[back=%.2fm forward=%.2fm "
      "max_heading_span=%.1fdeg max_abs_curvature=%.4f] "
      "velocity_jerk[longitudinal=%.2fmps3 lateral=%.2fmps3] "
      "trajectory_update_max_start_cross_track=%.2fm "
      "vertical_follower[kp=%.2f max_climb_speed=%.2fmps "
      "max_descent_speed=%.2fmps max_accel=%.2fmps2 "
      "max_jerk=%.2fmps3 target_vz_ff_scale=%.2f] "
      "executable_trajectory[source=planner_final_path final_topic='%s' "
      "debug_sample_step=%.2fm "
      "marker_topic='%s'] "
      "rviz_drone_follow_tf[enabled=%s parent='%s' frame='%s'] "
      "mission_goal=(%.1f, %.1f) "
      "px4_local_origin=(%.1f, %.1f) telemetry_log_period=%.2fs "
      "flight_blackbox=%s flight_blackbox_path='%s' "
      "max_pose_staleness=%.2fs command_resend_period=%.2fs",
      initial_altitude_m_, acceptance_radius_m_, auto_arm_ ? "true" : "false",
      auto_offboard_ ? "true" : "false", min_navigation_altitude_m_, takeoff_hover_s_,
      diagnostic_turn_preview_distance_m_, velocity_follower_config_.cruise_speed_mps,
      velocity_follower_config_.min_turn_speed_mps,
      velocity_follower_config_.speed_profile_accel_mps2,
      velocity_follower_config_.speed_profile_decel_mps2,
      velocity_follower_config_.turn_speed_lateral_accel_mps2,
      velocity_follower_config_.setpoint_forward_accel_mps2,
      velocity_follower_config_.setpoint_forward_decel_mps2,
      velocity_follower_config_.speed_profile_sample_step_m,
      velocity_follower_config_.speed_profile_lookahead_time_s,
      velocity_follower_config_.speed_profile_lookahead_min_m,
      velocity_follower_config_.speed_profile_lookahead_max_m,
      velocity_follower_config_.final_hold_max_speed_mps,
      velocity_follower_config_.terminal_capture_radius_m,
      velocity_follower_config_.terminal_capture_gain_1ps,
      velocity_follower_config_.terminal_capture_max_speed_mps,
      velocity_follower_config_.terminal_position_capture_max_entry_speed_mps,
      velocity_follower_config_.terminal_stuck_speed_mps,
      velocity_follower_config_.cross_track_gain,
      velocity_follower_config_.cross_track_p_gain_schedule_start_m,
      velocity_follower_config_.cross_track_p_gain_schedule_full_m,
      velocity_follower_config_.cross_track_p_gain_schedule_min_factor,
      velocity_follower_config_.cross_track_p_gain_schedule_max_factor,
      velocity_follower_config_.tracking_prediction_horizon_s,
      radiansToDegrees(velocity_follower_config_.max_lateral_control_angle_rad),
      velocity_follower_config_.setpoint_lateral_response_accel_mps2,
      velocity_follower_config_.curvature_feedforward_time_s,
      radiansToDegrees(
          velocity_follower_config_.curvature_feedforward_deadband_angle_rad),
      radiansToDegrees(velocity_follower_config_.curvature_feedforward_full_angle_rad),
      radiansToDegrees(velocity_follower_config_.max_curvature_feedforward_angle_rad),
      velocity_follower_config_.cross_track_d_gain_schedule_min_speed_mps,
      velocity_follower_config_.cross_track_d_gain_schedule_full_speed_mps,
      velocity_follower_config_.cross_track_d_gain_schedule_max_factor,
      velocity_follower_config_.control_tangent_smoothing_back_m,
      velocity_follower_config_.control_tangent_smoothing_forward_m,
      radiansToDegrees(
          velocity_follower_config_.control_tangent_smoothing_max_heading_span_rad),
      velocity_follower_config_.control_tangent_smoothing_max_abs_curvature_1pm,
      velocity_follower_config_.max_velocity_jerk_mps3,
      velocity_follower_config_.max_lateral_velocity_jerk_mps3,
      trajectory_update_max_start_cross_track_m_,
      vertical_follower_config_.altitude_feedback_kp_1ps,
      vertical_follower_config_.max_climb_speed_mps,
      vertical_follower_config_.max_descent_speed_mps,
      vertical_follower_config_.max_vertical_accel_mps2,
      vertical_follower_config_.max_vertical_jerk_mps3,
      vertical_follower_config_.target_vz_feedforward_scale,
      final_trajectory_debug_topic_.c_str(), final_trajectory_debug_sample_step_m_,
      offboard_debug_marker_topic_.c_str(),
      rviz_drone_follow_tf_enabled_ ? "true" : "false",
      rviz_drone_follow_parent_frame_.c_str(), rviz_drone_follow_frame_.c_str(),
      mission_goal_.x, mission_goal_.y, px4_local_origin_.x, px4_local_origin_.y,
      static_cast<double>(telemetry_log_period_ns_) / 1.0e9,
      flight_blackbox_enabled_ ? "true" : "false", flight_blackbox_path_.c_str(),
      static_cast<double>(max_pose_staleness_ns_) / 1.0e9, command_resend_period_s_);
  RCLCPP_INFO(get_logger(),
              "No-static speed policy: enabled=%s max_speed=%.2fmps "
              "passage_speed_limit=%.2fmps braking_decel=%.2fmps2",
              velocity_follower_config_.no_static_speed_policy.enabled ? "true"
                                                                       : "false",
              velocity_follower_config_.no_static_speed_policy.max_speed_mps,
              velocity_follower_config_.no_static_speed_policy.passage_speed_limit_mps,
              velocity_follower_config_.no_static_speed_policy.braking_decel_mps2);
  RCLCPP_INFO(get_logger(),
              "Safe trajectory truncation: enabled=%s margin=%.2fm blocker_topic='%s' "
              "truncation_topic='%s' ack_topic='%s'",
              safe_trajectory_truncation_enabled_ ? "true" : "false",
              safe_trajectory_truncation_margin_m_, topics.replan_blocker.c_str(),
              topics.replan_truncation.c_str(), topics.truncation_suffix_ack.c_str());
  RCLCPP_INFO(
      get_logger(),
      "Trajectory handover: enabled=%s require_grid=%s prefix_time=%.2fs "
      "prefix_distance=[%.2f, %.2f]m candidate_lookahead=%.2fm "
      "hard_window_settle=%.2fm sample_step=%.2fm "
      "max_join_distance=%.2fm max_heading_delta=%.1fdeg max_curvature=%.3f "
      "defer[min_speed=%.2fmps projection_jump=%.2fm tangent_jump=%.1fdeg "
      "command_jump=%.2fmps]",
      trajectory_handover_config_.enabled ? "true" : "false",
      trajectory_handover_config_.require_validation_grid ? "true" : "false",
      trajectory_handover_config_.prefix_time_s,
      trajectory_handover_config_.min_prefix_distance_m,
      trajectory_handover_config_.max_prefix_distance_m,
      trajectory_handover_config_.candidate_lookahead_distance_m,
      trajectory_handover_config_.hard_window_exit_settle_distance_m,
      trajectory_handover_config_.sample_step_m,
      trajectory_handover_config_.max_join_distance_m,
      radiansToDegrees(trajectory_handover_config_.max_sample_heading_delta_rad),
      trajectory_handover_config_.max_abs_curvature_1pm,
      trajectory_continuity_thresholds_.defer_min_reference_speed_mps,
      trajectory_continuity_thresholds_.defer_projection_jump_m,
      radiansToDegrees(trajectory_continuity_thresholds_.defer_tangent_jump_rad),
      trajectory_continuity_thresholds_.defer_tangent_speed_command_jump_mps);
  RCLCPP_INFO(get_logger(),
              "PX4 offboard subscriptions: executable_trajectory='%s' path_id='%s' "
              "local_position='%s' "
              "attitude='%s' vehicle_status='%s' prohibited_grid='%s' "
              "replan_blocker='%s'",
              topics.executable_trajectory.c_str(), topics.path_id.c_str(),
              topics.px4_local_position.c_str(), topics.px4_vehicle_attitude.c_str(),
              topics.px4_vehicle_status.c_str(), topics.prohibited_grid.c_str(),
              topics.replan_blocker.c_str());
}

} // namespace drone_city_nav
