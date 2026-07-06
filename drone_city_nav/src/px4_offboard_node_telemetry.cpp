#include "px4_offboard_node.hpp"

namespace drone_city_nav {

void Px4OffboardNode::logTelemetry() {
  if (!local_position_valid_) {
    return;
  }

  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (last_telemetry_log_ns_ > 0 &&
      now_ns - last_telemetry_log_ns_ < telemetry_log_period_ns_) {
    return;
  }
  last_telemetry_log_ns_ = now_ns;

  const Point2 target = loggedTarget();
  const double target_distance = distance(current_position_, target);
  const double mission_goal_distance = distance(current_position_, mission_goal_);
  const double path_goal_distance = trajectoryGoalReady()
                                        ? distance(current_position_, trajectory_goal_)
                                        : std::numeric_limits<double>::quiet_NaN();
  const NearestProhibitedCellDiagnostic nearest_prohibited_cell =
      nearestProhibitedCellDiagnostic(current_position_, kInflatedOccupancyValue);
  const double prohibited_grid_clearance_m =
      nearest_prohibited_cell.valid
          ? nearest_prohibited_cell.clearance_m
          : estimateProhibitedGridClearanceM(current_position_);
  const bool hold_position = shouldHoldPosition();
  const bool pose_fresh = localPositionFresh();
  const double pose_age_s = localPositionAgeSeconds();
  const double attitude_age_s = attitudeAgeSeconds();
  const double roll_deg = radiansToDegrees(current_attitude_.roll_rad);
  const double pitch_deg = radiansToDegrees(current_attitude_.pitch_rad);
  const double attitude_yaw_deg = radiansToDegrees(current_attitude_.yaw_rad);
  const double tilt_deg = radiansToDegrees(
      std::hypot(current_attitude_.roll_rad, current_attitude_.pitch_rad));
  const UpcomingTurn upcoming_turn = upcomingTurnAtWaypoint(waypoint_index_);
  const double turn_angle_rad = upcoming_turn.angle_rad;
  const PathTrackingDiagnostics path_tracking = pathTrackingDiagnostics();

  RCLCPP_INFO(
      get_logger(),
      "Drone telemetry: current=(%.2f, %.2f) pose_fresh=%s pose_age_s=%.2f "
      "altitude=%.2f heading=%.3f "
      "attitude[valid=%s age_s=%.2f roll=%.3frad pitch=%.3frad yaw=%.3frad "
      "roll_deg=%.1f pitch_deg=%.1f yaw_deg=%.1f tilt_deg=%.1f] "
      "velocity=(%.2f, %.2f) velocity_valid=%s actual_speed=%.2f "
      "target=(%.2f, %.2f) "
      "distance_to_target=%.2f distance_to_path_goal=%.2f "
      "distance_to_mission_goal=%.2f waypoint=%zu/%zu motion_phase=%s "
      "final_trajectory_segment=%s prohibited_grid_clearance=%.2f "
      "diagnostic_rough_route_turn[valid=%s index=%zu distance=%.2f angle=%.3f] "
      "final_goal_hold=%s",
      current_position_.x, current_position_.y, pose_fresh ? "true" : "false",
      pose_age_s, current_altitude_m_, current_heading_rad_,
      attitude_valid_ ? "true" : "false", attitude_age_s, current_attitude_.roll_rad,
      current_attitude_.pitch_rad, current_attitude_.yaw_rad, roll_deg, pitch_deg,
      attitude_yaw_deg, tilt_deg, current_velocity_.x, current_velocity_.y,
      current_velocity_valid_ ? "true" : "false", current_speed_mps_, target.x,
      target.y, target_distance, path_goal_distance, mission_goal_distance,
      path_valid_ ? waypoint_index_ + 1U : 0U, path_points_.size(),
      motionPhaseName(hold_position), pathSegmentTypeName(turn_angle_rad),
      prohibited_grid_clearance_m, upcoming_turn.valid ? "true" : "false",
      upcoming_turn.valid ? upcoming_turn.waypoint_index + 1U : 0U,
      upcoming_turn.distance_to_turn_m, turn_angle_rad,
      final_goal_hold_active_ ? "true" : "false");
  RCLCPP_INFO(get_logger(),
              "Drone path diagnostics: path_id[local_update=%" PRIu64
              " planner=%" PRIu64 " planner_seen=%s stamp_ns=%" PRIu64
              "] tracking[valid=%s cross_track=%.2f signed_cross_track=%.2f "
              "heading_error=%.3f path_heading=%.3f segment=%zu t=%.2f "
              "projection=(%.2f, %.2f, %.2f)]",
              received_path_update_id_, accepted_planner_path_id_,
              accepted_planner_path_id_seen_ ? "true" : "false",
              last_received_path_stamp_ns_, path_tracking.valid ? "true" : "false",
              path_tracking.cross_track_error_m,
              path_tracking.signed_cross_track_error_m, path_tracking.heading_error_rad,
              path_tracking.path_heading_rad, path_tracking.segment_start_index,
              path_tracking.segment_t, path_tracking.projection.x,
              path_tracking.projection.y, path_tracking.projection_z_m);
  RCLCPP_INFO(get_logger(),
              "Drone command diagnostics: command[target_delta=%.2f "
              "target_distance=%.2f yaw=%.3f]",
              last_commanded_target_delta_m_, last_commanded_target_distance_m_,
              last_commanded_yaw_rad_);
  RCLCPP_INFO(
      get_logger(),
      "Drone velocity command diagnostics: control_mode=%s "
      "velocity_setpoint=(%.2f, %.2f, %.2f) velocity_setpoint_speed=%.2f "
      "desired_velocity=(%.2f, %.2f) desired_speed=%.2f "
      "velocity_tracking_error=%.2f desired_limiter_delta=%.2f "
      "velocity_setpoint_accel=(%.2f, %.2f) velocity_setpoint_accel_norm=%.2f "
      "velocity_setpoint_jerk=%.2f "
      "lateral_control[feedback=(%.2f, %.2f) derivative=(%.2f, %.2f) "
      "curvature_ff=(%.2f, %.2f) raw=(%.2f, %.2f) final=(%.2f, %.2f) "
      "final_norm=%.2f curvature_angle=%.1fdeg curvature_context=%.2f "
      "p_gain_factor=%.2f] "
      "speed_limit_reason=%s "
      "terminal_capture[active=%s goal_distance=%.2f signed_along=%.2f "
      "remaining_s=%.2f "
      "speed_limit=%.2f gain_limit=%.2f max_speed=%.2f "
      "brake_limit=%.2f activation=%.2f decel=%.2f margin=%.2f "
      "hold_distance_met=%s hold_speed_met=%s trigger_goal=%s "
      "trigger_remaining=%s] "
      "terminal_state=%s "
      "terminal_position_capture[active=%s reason=%s goal_distance=%.2f "
      "remaining_s=%.2f speed=%.2f radius=%.2f max_entry_speed=%.2f "
      "stuck_speed=%.2f] "
      "raw_speed_limit=%.2f profile_speed_limit=%.2f "
      "lookahead_distance=%.2f lookahead_speed_limit=%.2f "
      "speed_after_lookahead=%.2f lookahead_constraint[type=%s index=%zu "
      "distance=%.2f] "
      "final_command_speed=%.2f accel_limited_speed=%.2f "
      "limiting_constraint[type=%s index=%zu distance=%.2f speed=%.2f "
      "allowed=%.2f curve_radius=%.2f] "
      "final_stop_distance=%.2f final_stop_braking_distance=%.2f "
      "velocity_delta=%.2f trajectory_cross_track=%.2f "
      "cross_track_lateral_velocity=%.2f "
      "control_tangent[smoothed=%s mode=%s raw=(%.2f, %.2f) heading_span=%.1fdeg "
      "max_abs_curvature=%.4f window=(%.2f, %.2f)] "
      "velocity_basis[current_tangent=%.2f current_normal=%.2f "
      "desired_tangent=%.2f desired_normal=%.2f "
      "setpoint_tangent=%.2f setpoint_normal=%.2f "
      "desired_to_setpoint_tangent=%.2f desired_to_setpoint_normal=%.2f "
      "setpoint_to_actual_tangent=%.2f setpoint_to_actual_normal=%.2f "
      "desired_to_actual_tangent=%.2f desired_to_actual_normal=%.2f] "
      "smoother[reset_reason=%s path_update_resets=%" PRIu64
      " path_frame=%s lateral_accel=%.2f] "
      "altitude[target_z=%.2f actual_z=%.2f z_error=%.2f target_vz=%.2f "
      "feedback_vz=%.2f desired_vz=%.2f commanded_vz=%.2f "
      "commanded_vz_ned=%.2f trajectory_target_valid=%s passage_mode=%s "
      "passage_id=%s slope=%.4f constraint=%s reason=%s] "
      "tangent=(%.2f, %.2f) projection=(%.2f, %.2f) "
      "trajectory[valid=%s s=%.2f segment=%zu type=%s curvature=%.4f "
      "arc_radius=%.2f lines=%zu arcs=%zu length=%.2f samples=%zu "
      "status=%.*s corridor_width_min=%.2f lateral_offset_max=%.2f] "
      "tracking_prediction[horizon=%.2fs distance=%.2f predicted=(%.2f, %.2f) "
      "current_projection=(%.2f, %.2f) predicted_projection=(%.2f, %.2f) "
      "current_cross=%.2f predicted_cross=%.2f response_delay_distance=%.2f]",
      offboardSetpointModeName(last_offboard_setpoint_mode_), last_velocity_setpoint_.x,
      last_velocity_setpoint_.y, last_vertical_velocity_setpoint_mps_,
      last_velocity_setpoint_speed_mps_, last_velocity_plan_.desired_velocity_xy.x,
      last_velocity_plan_.desired_velocity_xy.y, last_velocity_plan_.desired_speed_mps,
      last_velocity_plan_.velocity_tracking_error_mps,
      last_velocity_plan_.desired_velocity_delta_mps,
      last_velocity_plan_.velocity_setpoint_acceleration_xy.x,
      last_velocity_plan_.velocity_setpoint_acceleration_xy.y,
      last_velocity_plan_.velocity_setpoint_acceleration_mps2,
      last_velocity_plan_.velocity_setpoint_jerk_mps3,
      last_velocity_plan_.cross_track_feedback_velocity.x,
      last_velocity_plan_.cross_track_feedback_velocity.y,
      last_velocity_plan_.cross_track_derivative_damping_velocity.x,
      last_velocity_plan_.cross_track_derivative_damping_velocity.y,
      last_velocity_plan_.curvature_feedforward_velocity.x,
      last_velocity_plan_.curvature_feedforward_velocity.y,
      last_velocity_plan_.raw_lateral_control_velocity.x,
      last_velocity_plan_.raw_lateral_control_velocity.y,
      last_velocity_plan_.lateral_control_velocity.x,
      last_velocity_plan_.lateral_control_velocity.y,
      last_velocity_plan_.lateral_control_mps,
      radiansToDegrees(last_velocity_plan_.curvature_feedforward_angle_rad),
      last_velocity_plan_.curvature_feedforward_context_scale,
      last_velocity_plan_.cross_track_p_gain_factor,
      velocitySetpointReasonName(last_velocity_plan_.reason),
      last_velocity_plan_.terminal_capture_active ? "true" : "false",
      last_velocity_plan_.terminal_goal_distance_m,
      last_velocity_plan_.terminal_signed_along_track_distance_m,
      last_velocity_plan_.terminal_remaining_trajectory_distance_m,
      last_velocity_plan_.terminal_capture_speed_limit_mps,
      last_velocity_plan_.terminal_capture_gain_speed_limit_mps,
      last_velocity_plan_.terminal_capture_max_speed_mps,
      last_velocity_plan_.terminal_capture_braking_speed_limit_mps,
      last_velocity_plan_.terminal_capture_activation_distance_m,
      last_velocity_plan_.terminal_capture_decel_mps2,
      last_velocity_plan_.terminal_capture_braking_margin_m,
      last_velocity_plan_.terminal_hold_distance_met ? "true" : "false",
      last_velocity_plan_.terminal_hold_speed_met ? "true" : "false",
      last_velocity_plan_.terminal_capture_goal_distance_triggered ? "true" : "false",
      last_velocity_plan_.terminal_capture_remaining_distance_triggered ? "true"
                                                                        : "false",
      terminalFlightStateName(terminal_capture_state_.state),
      last_terminal_position_capture_active_ ? "true" : "false",
      last_terminal_position_capture_reason_.c_str(),
      last_terminal_position_capture_goal_distance_m_,
      last_terminal_position_capture_remaining_s_m_,
      last_terminal_position_capture_speed_mps_,
      last_terminal_position_capture_activation_radius_m_,
      last_terminal_position_capture_max_entry_speed_mps_,
      last_terminal_position_capture_stuck_speed_mps_,
      last_velocity_plan_.raw_speed_limit_mps,
      last_velocity_plan_.profile_speed_limit_mps,
      last_velocity_plan_.speed_lookahead_distance_m,
      last_velocity_plan_.lookahead_speed_limit_mps,
      last_velocity_plan_.speed_after_lookahead_mps,
      speedConstraintTypeName(last_velocity_plan_.lookahead_limiting_constraint_type),
      last_velocity_plan_.lookahead_limiting_constraint_index,
      last_velocity_plan_.lookahead_limiting_constraint_distance_m,
      last_velocity_plan_.final_command_speed_mps,
      last_velocity_plan_.accel_limited_speed_mps,
      speedConstraintTypeName(last_velocity_plan_.limiting_constraint_type),
      last_velocity_plan_.limiting_constraint_index,
      last_velocity_plan_.limiting_constraint_distance_m,
      last_velocity_plan_.limiting_constraint_speed_mps,
      last_velocity_plan_.limiting_allowed_speed_now_mps,
      last_velocity_plan_.limiting_curve_radius_m,
      last_velocity_plan_.final_stop.distance_to_stop_m,
      last_velocity_plan_.final_stop.braking_distance_m,
      last_velocity_plan_.velocity_delta_mps,
      last_velocity_plan_.trajectory_cross_track_error_m,
      last_velocity_plan_.cross_track_lateral_velocity_mps,
      last_velocity_plan_.control_tangent_smoothed ? "true" : "false",
      controlProjectionSmoothingModeName(
          last_velocity_plan_.control_projection_smoothing_mode),
      last_velocity_plan_.control_tangent_raw.x,
      last_velocity_plan_.control_tangent_raw.y,
      radiansToDegrees(last_velocity_plan_.control_tangent_smoothing_heading_span_rad),
      last_velocity_plan_.control_tangent_smoothing_max_abs_curvature_1pm,
      last_velocity_plan_.control_tangent_smoothing_window_start_s_m,
      last_velocity_plan_.control_tangent_smoothing_window_end_s_m,
      last_velocity_plan_.current_velocity_tangent_mps,
      last_velocity_plan_.current_velocity_normal_mps,
      last_velocity_plan_.desired_velocity_tangent_mps,
      last_velocity_plan_.desired_velocity_normal_mps,
      last_velocity_plan_.setpoint_velocity_tangent_mps,
      last_velocity_plan_.setpoint_velocity_normal_mps,
      last_velocity_plan_.desired_to_setpoint_tangent_error_mps,
      last_velocity_plan_.desired_to_setpoint_normal_error_mps,
      last_velocity_plan_.setpoint_to_actual_tangent_error_mps,
      last_velocity_plan_.setpoint_to_actual_normal_error_mps,
      last_velocity_plan_.desired_to_actual_tangent_error_mps,
      last_velocity_plan_.desired_to_actual_normal_error_mps,
      last_velocity_smoother_reset_reason_.c_str(),
      path_update_velocity_smoother_reset_count_,
      last_velocity_plan_.path_frame_lateral_smoothing_applied ? "true" : "false",
      last_velocity_plan_.smoother_lateral_response_accel_mps2,
      last_vertical_plan_.target_z_m, last_vertical_plan_.actual_z_m,
      last_vertical_plan_.z_error_m, last_vertical_plan_.target_vz_mps,
      last_vertical_plan_.feedback_vz_mps, last_vertical_plan_.desired_vz_mps,
      last_vertical_plan_.commanded_vz_mps, last_vertical_plan_.commanded_vz_ned_mps,
      last_vertical_plan_.trajectory_target_valid ? "true" : "false",
      last_vertical_plan_.passage_mode ? "true" : "false",
      last_vertical_plan_.passage_id.c_str(), last_vertical_plan_.vertical_slope_dz_ds,
      last_vertical_plan_.vertical_constraint_active ? "true" : "false",
      last_vertical_plan_.reason.c_str(), last_velocity_plan_.path_tangent.x,
      last_velocity_plan_.path_tangent.y, last_velocity_plan_.projection.x,
      last_velocity_plan_.projection.y, trajectory_valid_ ? "true" : "false",
      last_velocity_plan_.trajectory_s_m, last_velocity_plan_.trajectory_segment_index,
      trajectorySegmentKindName(last_velocity_plan_.trajectory_segment_kind),
      last_velocity_plan_.trajectory_curvature_1pm,
      last_velocity_plan_.trajectory_arc_radius_m,
      last_trajectory_metrics_.line_segments, last_trajectory_metrics_.arc_segments,
      last_trajectory_metrics_.length_m, final_trajectory_samples_.size(),
      static_cast<int>(
          trajectoryPlannerStatusName(last_trajectory_planner_stats_.status).size()),
      trajectoryPlannerStatusName(last_trajectory_planner_stats_.status).data(),
      last_trajectory_planner_stats_.corridor.min_width_m,
      last_trajectory_planner_stats_.trajectory_optimizer.max_abs_offset_m,
      last_velocity_plan_.prediction_horizon_s,
      last_velocity_plan_.prediction_distance_m,
      last_velocity_plan_.predicted_position.x,
      last_velocity_plan_.predicted_position.y,
      last_velocity_plan_.current_projection.x,
      last_velocity_plan_.current_projection.y,
      last_velocity_plan_.predicted_projection.x,
      last_velocity_plan_.predicted_projection.y,
      last_velocity_plan_.current_cross_track_error_m,
      last_velocity_plan_.predicted_cross_track_error_m,
      last_velocity_plan_.response_delay_distance_m);
  RCLCPP_INFO(get_logger(),
              "Drone obstacle diagnostics: nearest_prohibited_cell[valid=%s "
              "prohibited_grid_clearance=%.2f "
              "bearing_map=%.3f bearing_body=%.3f bearing_body_deg=%.1f "
              "point=(%.2f, %.2f)]",
              nearest_prohibited_cell.valid ? "true" : "false",
              nearest_prohibited_cell.clearance_m,
              nearest_prohibited_cell.bearing_map_rad,
              nearest_prohibited_cell.bearing_body_rad,
              nearest_prohibited_cell.bearing_body_deg, nearest_prohibited_cell.point.x,
              nearest_prohibited_cell.point.y);
  writeFlightBlackbox(now_ns, target, target_distance, path_goal_distance,
                      mission_goal_distance, prohibited_grid_clearance_m, pose_fresh,
                      pose_age_s, attitude_age_s, upcoming_turn, hold_position,
                      path_tracking, nearest_prohibited_cell);
}

void Px4OffboardNode::writeFlightBlackbox(
    const std::int64_t now_ns, const Point2 target, const double target_distance_m,
    const double path_goal_distance_m, const double mission_goal_distance_m,
    const double prohibited_grid_clearance_m, const bool pose_fresh,
    const double pose_age_s, const double attitude_age_s,
    const UpcomingTurn& upcoming_turn, const bool hold_position,
    const PathTrackingDiagnostics& path_tracking,
    const NearestProhibitedCellDiagnostic& nearest_prohibited_cell) {
  if (!flight_blackbox_enabled_ || !flight_blackbox_stream_.is_open()) {
    return;
  }

  const OffboardBlackboxRecord record{
      now_ns,
      OffboardBlackboxPathId{received_path_update_id_, accepted_planner_path_id_,
                             accepted_planner_path_id_seen_,
                             last_received_path_stamp_ns_},
      pose_fresh,
      pose_age_s,
      current_position_,
      current_altitude_m_,
      current_heading_rad_,
      attitude_valid_,
      attitude_age_s,
      current_attitude_,
      current_velocity_valid_,
      current_velocity_,
      current_speed_mps_,
      target,
      target_distance_m,
      last_commanded_target_delta_m_,
      last_commanded_yaw_rad_,
      offboardSetpointModeName(last_offboard_setpoint_mode_),
      last_velocity_setpoint_,
      last_vertical_velocity_setpoint_mps_,
      last_velocity_setpoint_speed_mps_,
      last_velocity_plan_,
      last_vertical_plan_,
      last_velocity_smoother_reset_reason_,
      path_update_velocity_smoother_reset_count_,
      last_target_altitude_m_,
      last_trajectory_altitude_target_valid_,
      last_altitude_error_m_,
      trajectory_valid_,
      last_trajectory_metrics_,
      final_trajectory_samples_.size(),
      last_trajectory_planner_stats_,
      last_trajectory_shape_diagnostics_,
      path_valid_,
      waypoint_index_,
      path_points_.size(),
      path_goal_distance_m,
      mission_goal_distance_m,
      upcoming_turn,
      pathSegmentTypeName(upcoming_turn.angle_rad),
      OffboardBlackboxPathTracking{
          path_tracking.valid, path_tracking.segment_start_index,
          path_tracking.segment_t, path_tracking.cross_track_error_m,
          path_tracking.signed_cross_track_error_m, path_tracking.path_heading_rad,
          path_tracking.heading_error_rad, path_tracking.projection,
          path_tracking.projection_z_m},
      motionPhaseName(hold_position),
      final_goal_hold_active_,
      terminalFlightStateName(terminal_capture_state_.state),
      last_terminal_position_capture_active_,
      last_terminal_position_capture_reason_,
      last_terminal_position_capture_goal_distance_m_,
      last_terminal_position_capture_remaining_s_m_,
      last_terminal_position_capture_speed_mps_,
      last_terminal_position_capture_activation_radius_m_,
      last_terminal_position_capture_max_entry_speed_mps_,
      last_terminal_position_capture_stuck_speed_mps_,
      prohibited_grid_clearance_m,
      OffboardBlackboxNearestProhibitedCell{
          nearest_prohibited_cell.valid, nearest_prohibited_cell.clearance_m,
          nearest_prohibited_cell.bearing_map_rad,
          nearest_prohibited_cell.bearing_body_rad,
          nearest_prohibited_cell.bearing_body_deg, nearest_prohibited_cell.point}};
  writeOffboardBlackboxRecord(flight_blackbox_stream_, record);
}

[[nodiscard]] Point2 Px4OffboardNode::loggedTarget() const {
  if (last_published_target_valid_) {
    return last_published_target_;
  }
  if (commanded_target_valid_) {
    return commanded_target_;
  }
  return currentTarget();
}

} // namespace drone_city_nav
