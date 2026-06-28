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
  const double path_goal_distance =
      path_valid_ ? distance(current_position_, path_points_.back())
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

  RCLCPP_INFO(get_logger(),
              "Drone telemetry: current=(%.2f, %.2f) pose_fresh=%s pose_age_s=%.2f "
              "altitude=%.2f heading=%.3f "
              "attitude[valid=%s age_s=%.2f roll=%.3frad pitch=%.3frad yaw=%.3frad "
              "roll_deg=%.1f pitch_deg=%.1f yaw_deg=%.1f tilt_deg=%.1f] "
              "velocity=(%.2f, %.2f) velocity_valid=%s actual_speed=%.2f "
              "target=(%.2f, %.2f) "
              "distance_to_target=%.2f distance_to_path_goal=%.2f "
              "distance_to_mission_goal=%.2f waypoint=%zu/%zu motion_phase=%s "
              "final_trajectory_segment=%s prohibited_grid_clearance=%.2f "
              "final_trajectory_turn[valid=%s index=%zu distance=%.2f angle=%.3f] "
              "final_goal_hold=%s",
              current_position_.x, current_position_.y, pose_fresh ? "true" : "false",
              pose_age_s, current_altitude_m_, current_heading_rad_,
              attitude_valid_ ? "true" : "false", attitude_age_s,
              current_attitude_.roll_rad, current_attitude_.pitch_rad,
              current_attitude_.yaw_rad, roll_deg, pitch_deg, attitude_yaw_deg,
              tilt_deg, current_velocity_.x, current_velocity_.y,
              current_velocity_valid_ ? "true" : "false", current_speed_mps_, target.x,
              target.y, target_distance, path_goal_distance, mission_goal_distance,
              path_valid_ ? waypoint_index_ + 1U : 0U, path_points_.size(),
              motionPhaseName(hold_position), pathSegmentTypeName(turn_angle_rad),
              prohibited_grid_clearance_m, upcoming_turn.valid ? "true" : "false",
              upcoming_turn.valid ? upcoming_turn.waypoint_index + 1U : 0U,
              upcoming_turn.distance_to_turn_m, turn_angle_rad,
              final_goal_hold_active_ ? "true" : "false");
  RCLCPP_INFO(
      get_logger(),
      "Drone path diagnostics: path_id[local_update=%" PRIu64 " planner=%" PRIu64
      " planner_seen=%s stamp_ns=%" PRIu64
      "] tracking[valid=%s cross_track=%.2f signed_cross_track=%.2f "
      "heading_error=%.3f path_heading=%.3f segment=%zu t=%.2f "
      "projection=(%.2f, %.2f)]",
      received_path_update_id_, latest_planner_path_id_,
      latest_planner_path_id_seen_ ? "true" : "false", last_received_path_stamp_ns_,
      path_tracking.valid ? "true" : "false", path_tracking.cross_track_error_m,
      path_tracking.signed_cross_track_error_m, path_tracking.heading_error_rad,
      path_tracking.path_heading_rad, path_tracking.segment_start_index,
      path_tracking.segment_t, path_tracking.projection.x, path_tracking.projection.y);
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
      "final_norm=%.2f delta=%.2f curvature_angle=%.1fdeg] "
      "speed_limit_reason=%s raw_speed_limit=%.2f profile_speed_limit=%.2f "
      "lookahead_distance=%.2f lookahead_speed_limit=%.2f "
      "speed_after_lookahead=%.2f lookahead_constraint[type=%s index=%zu "
      "distance=%.2f] "
      "cross_track_speed_factor=%.2f cross_track_limited_speed=%.2f "
      "final_command_speed=%.2f accel_limited_speed=%.2f "
      "limiting_constraint[type=%s index=%zu distance=%.2f speed=%.2f "
      "allowed=%.2f curve_radius=%.2f] "
      "final_stop_distance=%.2f final_stop_braking_distance=%.2f "
      "velocity_delta=%.2f trajectory_cross_track=%.2f "
      "cross_track_lateral_velocity=%.2f "
      "velocity_basis[current_tangent=%.2f current_normal=%.2f "
      "desired_tangent=%.2f desired_normal=%.2f "
      "setpoint_tangent=%.2f setpoint_normal=%.2f] "
      "smoother[reset_reason=%s path_update_resets=%" PRIu64 "] "
      "altitude_error=%.2f tangent=(%.2f, %.2f) projection=(%.2f, %.2f) "
      "trajectory[valid=%s s=%.2f segment=%zu type=%s curvature=%.4f "
      "arc_radius=%.2f lines=%zu arcs=%zu length=%.2f samples=%zu "
      "status=%.*s corridor_width_min=%.2f racing_offset_max=%.2f] "
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
      last_velocity_plan_.lateral_control_delta_mps,
      radiansToDegrees(last_velocity_plan_.curvature_feedforward_angle_rad),
      velocitySetpointReasonName(last_velocity_plan_.reason),
      last_velocity_plan_.raw_speed_limit_mps,
      last_velocity_plan_.profile_speed_limit_mps,
      last_velocity_plan_.speed_lookahead_distance_m,
      last_velocity_plan_.lookahead_speed_limit_mps,
      last_velocity_plan_.speed_after_lookahead_mps,
      speedConstraintTypeName(last_velocity_plan_.lookahead_limiting_constraint_type),
      last_velocity_plan_.lookahead_limiting_constraint_index,
      last_velocity_plan_.lookahead_limiting_constraint_distance_m,
      last_velocity_plan_.cross_track_speed_factor,
      last_velocity_plan_.cross_track_limited_speed_mps,
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
      last_velocity_plan_.current_velocity_tangent_mps,
      last_velocity_plan_.current_velocity_normal_mps,
      last_velocity_plan_.desired_velocity_tangent_mps,
      last_velocity_plan_.desired_velocity_normal_mps,
      last_velocity_plan_.setpoint_velocity_tangent_mps,
      last_velocity_plan_.setpoint_velocity_normal_mps,
      last_velocity_smoother_reset_reason_.c_str(),
      path_update_velocity_smoother_reset_count_, last_altitude_error_m_,
      last_velocity_plan_.path_tangent.x, last_velocity_plan_.path_tangent.y,
      last_velocity_plan_.projection.x, last_velocity_plan_.projection.y,
      trajectory_valid_ ? "true" : "false", last_velocity_plan_.trajectory_s_m,
      last_velocity_plan_.trajectory_segment_index,
      trajectorySegmentKindName(last_velocity_plan_.trajectory_segment_kind),
      last_velocity_plan_.trajectory_curvature_1pm,
      last_velocity_plan_.trajectory_arc_radius_m,
      last_trajectory_metrics_.line_segments, last_trajectory_metrics_.arc_segments,
      last_trajectory_metrics_.length_m, final_trajectory_samples_.size(),
      static_cast<int>(
          trajectoryPlannerStatusName(last_trajectory_planner_stats_.status).size()),
      trajectoryPlannerStatusName(last_trajectory_planner_stats_.status).data(),
      last_trajectory_planner_stats_.corridor.min_width_m,
      last_trajectory_planner_stats_.racing_line.max_abs_offset_m,
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

  flight_blackbox_stream_ << "{\"time_ns\":" << now_ns << ",";
  writeBlackboxPathId(flight_blackbox_stream_,
                      OffboardBlackboxPathId{
                          received_path_update_id_, latest_planner_path_id_,
                          latest_planner_path_id_seen_, last_received_path_stamp_ns_});
  flight_blackbox_stream_ << ",\"pose\":{\"fresh\":";
  writeJsonBool(flight_blackbox_stream_, pose_fresh);
  flight_blackbox_stream_ << ",\"age_s\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, pose_age_s);
  flight_blackbox_stream_ << ",\"x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, current_position_.x);
  flight_blackbox_stream_ << ",\"y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, current_position_.y);
  flight_blackbox_stream_ << ",\"altitude_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, current_altitude_m_);
  flight_blackbox_stream_ << ",\"heading_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, current_heading_rad_);
  flight_blackbox_stream_ << "}";
  flight_blackbox_stream_ << ",\"attitude\":{\"valid\":";
  writeJsonBool(flight_blackbox_stream_, attitude_valid_);
  flight_blackbox_stream_ << ",\"age_s\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, attitude_age_s);
  flight_blackbox_stream_ << ",\"roll_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, current_attitude_.roll_rad);
  flight_blackbox_stream_ << ",\"pitch_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, current_attitude_.pitch_rad);
  flight_blackbox_stream_ << ",\"yaw_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, current_attitude_.yaw_rad);
  flight_blackbox_stream_ << ",\"tilt_deg\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        radiansToDegrees(std::hypot(current_attitude_.roll_rad,
                                                    current_attitude_.pitch_rad)));
  flight_blackbox_stream_ << "}";
  flight_blackbox_stream_ << ",\"velocity\":{\"valid\":";
  writeJsonBool(flight_blackbox_stream_, current_velocity_valid_);
  flight_blackbox_stream_ << ",\"x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, current_velocity_.x);
  flight_blackbox_stream_ << ",\"y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, current_velocity_.y);
  flight_blackbox_stream_ << ",\"speed_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, current_speed_mps_);
  flight_blackbox_stream_ << "}";
  flight_blackbox_stream_ << ",\"target\":{\"x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, target.x);
  flight_blackbox_stream_ << ",\"y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, target.y);
  flight_blackbox_stream_ << ",\"distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, target_distance_m);
  flight_blackbox_stream_ << ",\"delta_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_commanded_target_delta_m_);
  flight_blackbox_stream_ << "}";
  flight_blackbox_stream_ << ",\"command\":{\"yaw_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_commanded_yaw_rad_);
  flight_blackbox_stream_ << "}";
  flight_blackbox_stream_ << ",\"velocity_command\":{\"control_mode\":\""
                          << offboardSetpointModeName(last_offboard_setpoint_mode_)
                          << "\",\"setpoint_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_setpoint_.x);
  flight_blackbox_stream_ << ",\"setpoint_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_setpoint_.y);
  flight_blackbox_stream_ << ",\"setpoint_z\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_vertical_velocity_setpoint_mps_);
  flight_blackbox_stream_ << ",\"setpoint_speed_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_setpoint_speed_mps_);
  flight_blackbox_stream_ << ",\"final_command_speed_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.final_command_speed_mps);
  flight_blackbox_stream_ << ",\"smoother_reset_reason\":\""
                          << last_velocity_smoother_reset_reason_ << "\"";
  flight_blackbox_stream_ << ",\"path_update_reset_count\":"
                          << path_update_velocity_smoother_reset_count_;
  flight_blackbox_stream_ << ",\"desired_setpoint_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.desired_velocity_xy.x);
  flight_blackbox_stream_ << ",\"desired_setpoint_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.desired_velocity_xy.y);
  flight_blackbox_stream_ << ",\"desired_setpoint_speed_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_plan_.desired_speed_mps);
  flight_blackbox_stream_ << ",\"cross_track_feedback_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.cross_track_feedback_velocity.x);
  flight_blackbox_stream_ << ",\"cross_track_feedback_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.cross_track_feedback_velocity.y);
  flight_blackbox_stream_ << ",\"cross_track_feedback_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.cross_track_feedback_mps);
  flight_blackbox_stream_ << ",\"cross_track_derivative_damping_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.cross_track_derivative_damping_velocity.x);
  flight_blackbox_stream_ << ",\"cross_track_derivative_damping_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.cross_track_derivative_damping_velocity.y);
  flight_blackbox_stream_ << ",\"cross_track_derivative_damping_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.cross_track_derivative_damping_mps);
  flight_blackbox_stream_ << ",\"curvature_feedforward_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.curvature_feedforward_velocity.x);
  flight_blackbox_stream_ << ",\"curvature_feedforward_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.curvature_feedforward_velocity.y);
  flight_blackbox_stream_ << ",\"curvature_feedforward_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.curvature_feedforward_mps);
  flight_blackbox_stream_ << ",\"curvature_feedforward_angle_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.curvature_feedforward_angle_rad);
  flight_blackbox_stream_ << ",\"raw_lateral_control_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.raw_lateral_control_velocity.x);
  flight_blackbox_stream_ << ",\"raw_lateral_control_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.raw_lateral_control_velocity.y);
  flight_blackbox_stream_ << ",\"raw_lateral_control_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.raw_lateral_control_mps);
  flight_blackbox_stream_ << ",\"lateral_control_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.lateral_control_velocity.x);
  flight_blackbox_stream_ << ",\"lateral_control_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.lateral_control_velocity.y);
  flight_blackbox_stream_ << ",\"lateral_control_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.lateral_control_mps);
  flight_blackbox_stream_ << ",\"lateral_control_delta_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.lateral_control_delta_mps);
  flight_blackbox_stream_ << ",\"velocity_setpoint_accel_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.velocity_setpoint_acceleration_xy.x);
  flight_blackbox_stream_ << ",\"velocity_setpoint_accel_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.velocity_setpoint_acceleration_xy.y);
  flight_blackbox_stream_ << ",\"velocity_setpoint_accel_norm_mps2\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.velocity_setpoint_acceleration_mps2);
  flight_blackbox_stream_ << ",\"velocity_setpoint_jerk_mps3\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.velocity_setpoint_jerk_mps3);
  flight_blackbox_stream_ << ",\"speed_limit_reason\":\""
                          << velocitySetpointReasonName(last_velocity_plan_.reason)
                          << "\",\"raw_speed_limit_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.raw_speed_limit_mps);
  flight_blackbox_stream_ << ",\"profile_speed_limit_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.profile_speed_limit_mps);
  flight_blackbox_stream_ << ",\"lookahead_distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.speed_lookahead_distance_m);
  flight_blackbox_stream_ << ",\"lookahead_speed_limit_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.lookahead_speed_limit_mps);
  flight_blackbox_stream_ << ",\"speed_after_lookahead_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.speed_after_lookahead_mps);
  flight_blackbox_stream_ << ",\"lookahead_limiting_constraint_type\":\""
                          << speedConstraintTypeName(
                                 last_velocity_plan_.lookahead_limiting_constraint_type)
                          << "\",\"lookahead_limiting_constraint_index\":"
                          << last_velocity_plan_.lookahead_limiting_constraint_index;
  flight_blackbox_stream_ << ",\"lookahead_limiting_constraint_distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.lookahead_limiting_constraint_distance_m);
  flight_blackbox_stream_ << ",\"cross_track_speed_factor\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.cross_track_speed_factor);
  flight_blackbox_stream_ << ",\"cross_track_limited_speed_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.cross_track_limited_speed_mps);
  flight_blackbox_stream_ << ",\"accel_limited_speed_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.accel_limited_speed_mps);
  flight_blackbox_stream_ << ",\"limiting_constraint_type\":\""
                          << speedConstraintTypeName(
                                 last_velocity_plan_.limiting_constraint_type)
                          << "\",\"limiting_constraint_index\":"
                          << last_velocity_plan_.limiting_constraint_index;
  flight_blackbox_stream_ << ",\"limiting_constraint_distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.limiting_constraint_distance_m);
  flight_blackbox_stream_ << ",\"limiting_constraint_speed_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.limiting_constraint_speed_mps);
  flight_blackbox_stream_ << ",\"limiting_allowed_speed_now_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.limiting_allowed_speed_now_mps);
  flight_blackbox_stream_ << ",\"limiting_curve_radius_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.limiting_curve_radius_m);
  flight_blackbox_stream_ << ",\"final_stop_distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.final_stop.distance_to_stop_m);
  flight_blackbox_stream_ << ",\"final_stop_braking_distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.final_stop.braking_distance_m);
  flight_blackbox_stream_ << ",\"velocity_delta_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.velocity_delta_mps);
  flight_blackbox_stream_ << ",\"desired_velocity_delta_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.desired_velocity_delta_mps);
  flight_blackbox_stream_ << ",\"velocity_tracking_error_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.velocity_tracking_error_mps);
  flight_blackbox_stream_ << ",\"cross_track_lateral_velocity_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.cross_track_lateral_velocity_mps);
  flight_blackbox_stream_ << ",\"current_velocity_tangent_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.current_velocity_tangent_mps);
  flight_blackbox_stream_ << ",\"current_velocity_normal_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.current_velocity_normal_mps);
  flight_blackbox_stream_ << ",\"desired_velocity_tangent_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.desired_velocity_tangent_mps);
  flight_blackbox_stream_ << ",\"desired_velocity_normal_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.desired_velocity_normal_mps);
  flight_blackbox_stream_ << ",\"setpoint_velocity_tangent_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.setpoint_velocity_tangent_mps);
  flight_blackbox_stream_ << ",\"setpoint_velocity_normal_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.setpoint_velocity_normal_mps);
  flight_blackbox_stream_ << ",\"trajectory_cross_track_error_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.trajectory_cross_track_error_m);
  flight_blackbox_stream_ << ",\"altitude_error_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_altitude_error_m_);
  flight_blackbox_stream_ << ",\"path_tangent_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_plan_.path_tangent.x);
  flight_blackbox_stream_ << ",\"path_tangent_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_plan_.path_tangent.y);
  flight_blackbox_stream_ << ",\"projection_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_plan_.projection.x);
  flight_blackbox_stream_ << ",\"projection_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_plan_.projection.y);
  flight_blackbox_stream_ << ",\"tracking_prediction_horizon_s\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.prediction_horizon_s);
  flight_blackbox_stream_ << ",\"tracking_prediction_distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.prediction_distance_m);
  flight_blackbox_stream_ << ",\"tracking_predicted_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.predicted_position.x);
  flight_blackbox_stream_ << ",\"tracking_predicted_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.predicted_position.y);
  flight_blackbox_stream_ << ",\"current_projection_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.current_projection.x);
  flight_blackbox_stream_ << ",\"current_projection_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.current_projection.y);
  flight_blackbox_stream_ << ",\"predicted_projection_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.predicted_projection.x);
  flight_blackbox_stream_ << ",\"predicted_projection_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.predicted_projection.y);
  flight_blackbox_stream_ << ",\"current_cross_track_error_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.current_cross_track_error_m);
  flight_blackbox_stream_ << ",\"predicted_cross_track_error_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.predicted_cross_track_error_m);
  flight_blackbox_stream_ << ",\"response_delay_distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.response_delay_distance_m);
  flight_blackbox_stream_ << ",\"trajectory_valid\":";
  writeJsonBool(flight_blackbox_stream_, trajectory_valid_);
  flight_blackbox_stream_ << ",\"trajectory_s_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_plan_.trajectory_s_m);
  flight_blackbox_stream_ << ",\"trajectory_segment_index\":"
                          << last_velocity_plan_.trajectory_segment_index;
  flight_blackbox_stream_ << ",\"trajectory_segment_type\":\""
                          << trajectorySegmentKindName(
                                 last_velocity_plan_.trajectory_segment_kind)
                          << "\"";
  flight_blackbox_stream_ << ",\"trajectory_curvature_1pm\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.trajectory_curvature_1pm);
  flight_blackbox_stream_ << ",\"trajectory_arc_radius_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.trajectory_arc_radius_m);
  flight_blackbox_stream_ << ",\"trajectory_total_length_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, last_trajectory_metrics_.length_m);
  flight_blackbox_stream_ << ",\"trajectory_line_segments\":"
                          << last_trajectory_metrics_.line_segments;
  flight_blackbox_stream_ << ",\"trajectory_arc_segments\":"
                          << last_trajectory_metrics_.arc_segments;
  flight_blackbox_stream_ << ",\"speed_profile_limit_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.profile_speed_limit_mps);
  flight_blackbox_stream_ << ",\"speed_profile_lookahead_distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.speed_lookahead_distance_m);
  flight_blackbox_stream_ << ",\"speed_profile_lookahead_limit_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.lookahead_speed_limit_mps);
  flight_blackbox_stream_ << ",\"speed_profile_reason\":\""
                          << speedConstraintTypeName(
                                 last_velocity_plan_.limiting_constraint_type)
                          << "\"";
  flight_blackbox_stream_ << ",\"speed_profile_distance_to_constraint_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_velocity_plan_.limiting_constraint_distance_m);
  flight_blackbox_stream_ << ",\"final_trajectory_samples\":"
                          << final_trajectory_samples_.size();
  flight_blackbox_stream_ << ",\"trajectory_planner_status\":\""
                          << trajectoryPlannerStatusName(
                                 last_trajectory_planner_stats_.status)
                          << "\"";
  flight_blackbox_stream_ << ","
                          << trajectoryTimingDiagnosticsJsonFields(
                                 last_trajectory_planner_stats_);
  flight_blackbox_stream_ << ",\"corridor_samples\":"
                          << last_trajectory_planner_stats_.corridor.samples;
  flight_blackbox_stream_ << ",\"corridor_width_min_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.corridor.min_width_m);
  flight_blackbox_stream_ << ",\"corridor_width_mean_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.corridor.mean_width_m);
  flight_blackbox_stream_ << ",\"corridor_width_max_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.corridor.max_width_m);
  flight_blackbox_stream_
      << ",\"corridor_lateral_limited_samples\":"
      << last_trajectory_planner_stats_.corridor.lateral_limited_samples;
  flight_blackbox_stream_
      << ",\"corridor_center_recovered_samples\":"
      << last_trajectory_planner_stats_.corridor.center_recovered_samples;
  flight_blackbox_stream_
      << ",\"corridor_center_unrecoverable_samples\":"
      << last_trajectory_planner_stats_.corridor.center_unrecoverable_samples;
  flight_blackbox_stream_ << ",\"corridor_center_recovery_max_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.corridor.max_center_recovery_m);
  flight_blackbox_stream_ << ",\"corridor_lateral_reduction_max_m\":";
  writeJsonNumberOrNull(
      flight_blackbox_stream_,
      last_trajectory_planner_stats_.corridor.max_lateral_bound_reduction_m);
  flight_blackbox_stream_ << ",\"racing_line_iterations\":"
                          << last_trajectory_planner_stats_.racing_line.iterations;
  flight_blackbox_stream_
      << ",\"racing_line_optimizer_samples\":"
      << last_trajectory_planner_stats_.racing_line.optimizer_samples;
  flight_blackbox_stream_ << ",\"racing_line_cost_initial\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.racing_line.initial_cost);
  flight_blackbox_stream_ << ",\"racing_line_cost_final\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.racing_line.final_cost);
  flight_blackbox_stream_ << ",\"racing_line_max_offset_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.racing_line.max_abs_offset_m);
  flight_blackbox_stream_ << ","
                          << racingLineDiagnosticsJsonFields(
                                 last_trajectory_planner_stats_);
  flight_blackbox_stream_ << ","
                          << turnSmoothingDiagnosticsJsonFields(
                                 last_trajectory_planner_stats_);
  flight_blackbox_stream_ << ",\"racing_line_time_final_s\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.racing_line.estimated_time_s);
  flight_blackbox_stream_ << ",\"racing_line_time_centerline_s\":";
  writeJsonNumberOrNull(
      flight_blackbox_stream_,
      last_trajectory_planner_stats_.racing_line.centerline_estimated_time_s);
  flight_blackbox_stream_ << ",\"racing_line_time_gain_s\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.racing_line.time_gain_s);
  flight_blackbox_stream_ << ",\"racing_line_time_best_candidate_s\":";
  writeJsonNumberOrNull(
      flight_blackbox_stream_,
      last_trajectory_planner_stats_.racing_line.best_candidate_estimated_time_s);
  flight_blackbox_stream_ << ",\"racing_line_best_candidate_score\":";
  writeJsonNumberOrNull(
      flight_blackbox_stream_,
      last_trajectory_planner_stats_.racing_line.best_candidate_score);
  flight_blackbox_stream_ << ",\"racing_line_speed_limit_min_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.racing_line.min_speed_limit_mps);
  flight_blackbox_stream_ << ",\"racing_line_speed_limit_max_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.racing_line.max_speed_limit_mps);
  flight_blackbox_stream_
      << ",\"racing_line_curvature_limited_samples\":"
      << last_trajectory_planner_stats_.racing_line.curvature_limited_samples;
  flight_blackbox_stream_ << ",\"racing_line_regularization_applied\":";
  writeJsonBool(flight_blackbox_stream_,
                last_trajectory_planner_stats_.racing_line.regularization_applied);
  flight_blackbox_stream_
      << ",\"racing_line_regularization_iterations\":"
      << last_trajectory_planner_stats_.racing_line.regularization_iterations;
  flight_blackbox_stream_ << ",\"racing_line_regularization_time_delta_s\":";
  writeJsonNumberOrNull(
      flight_blackbox_stream_,
      last_trajectory_planner_stats_.racing_line.regularization_time_delta_s);
  flight_blackbox_stream_ << ",\"racing_line_pre_regularization_curvature_jump_1pm\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.racing_line
                            .pre_regularization_max_curvature_jump_1pm);
  flight_blackbox_stream_ << ",\"racing_line_post_regularization_curvature_jump_1pm\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.racing_line
                            .post_regularization_max_curvature_jump_1pm);
  flight_blackbox_stream_ << ",\"curvature_min_1pm\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.curvature_min_1pm);
  flight_blackbox_stream_ << ",\"curvature_max_1pm\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.curvature_max_1pm);
  flight_blackbox_stream_ << ",\"curvature_mean_abs_1pm\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.curvature_mean_abs_1pm);
  flight_blackbox_stream_ << ",\"speed_profile_min_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.speed_profile_min_mps);
  flight_blackbox_stream_ << ",\"speed_profile_max_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.speed_profile_max_mps);
  flight_blackbox_stream_ << ",\"speed_profile_mean_mps\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_planner_stats_.speed_profile_mean_mps);
  flight_blackbox_stream_
      << ",\"speed_profile_limited_by_curvature_count\":"
      << last_trajectory_planner_stats_.speed_profile_curvature_limited_samples;
  flight_blackbox_stream_ << ",\"trajectory_shape_segment_count\":"
                          << last_trajectory_shape_diagnostics_.segment_count;
  flight_blackbox_stream_ << ",\"trajectory_shape_segment_len_min_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_shape_diagnostics_.min_segment_length_m);
  flight_blackbox_stream_ << ",\"trajectory_shape_segment_len_mean_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_shape_diagnostics_.mean_segment_length_m);
  flight_blackbox_stream_ << ",\"trajectory_shape_segment_len_max_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_shape_diagnostics_.max_segment_length_m);
  flight_blackbox_stream_
      << ",\"trajectory_shape_segments_lt_0_5m\":"
      << last_trajectory_shape_diagnostics_.segments_shorter_than_0_5m;
  flight_blackbox_stream_
      << ",\"trajectory_shape_segments_lt_1m\":"
      << last_trajectory_shape_diagnostics_.segments_shorter_than_1m;
  flight_blackbox_stream_
      << ",\"trajectory_shape_segments_lt_2m\":"
      << last_trajectory_shape_diagnostics_.segments_shorter_than_2m;
  flight_blackbox_stream_ << ",\"trajectory_shape_max_heading_delta_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_shape_diagnostics_.max_heading_delta_rad);
  flight_blackbox_stream_ << ",\"trajectory_shape_max_heading_delta_index\":"
                          << last_trajectory_shape_diagnostics_.max_heading_delta_index;
  flight_blackbox_stream_ << ",\"trajectory_shape_max_heading_delta_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_shape_diagnostics_.max_heading_delta_point.x);
  flight_blackbox_stream_ << ",\"trajectory_shape_max_heading_delta_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_shape_diagnostics_.max_heading_delta_point.y);
  flight_blackbox_stream_ << ",\"trajectory_shape_max_curvature_jump_1pm\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_shape_diagnostics_.max_curvature_jump_1pm);
  flight_blackbox_stream_
      << ",\"trajectory_shape_max_curvature_jump_index\":"
      << last_trajectory_shape_diagnostics_.max_curvature_jump_index;
  flight_blackbox_stream_ << ",\"trajectory_shape_max_curvature_jump_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_shape_diagnostics_.max_curvature_jump_point.x);
  flight_blackbox_stream_ << ",\"trajectory_shape_max_curvature_jump_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_shape_diagnostics_.max_curvature_jump_point.y);
  flight_blackbox_stream_ << ",\"trajectory_shape_max_offset_delta_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_shape_diagnostics_.max_offset_delta_m);
  flight_blackbox_stream_ << ",\"trajectory_shape_max_offset_delta_index\":"
                          << last_trajectory_shape_diagnostics_.max_offset_delta_index;
  flight_blackbox_stream_ << ",\"trajectory_shape_max_offset_second_delta_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        last_trajectory_shape_diagnostics_.max_offset_second_delta_m);
  flight_blackbox_stream_
      << ",\"trajectory_shape_max_offset_second_delta_index\":"
      << last_trajectory_shape_diagnostics_.max_offset_second_delta_index;
  flight_blackbox_stream_ << "}";
  flight_blackbox_stream_ << ",\"path\":{\"valid\":";
  writeJsonBool(flight_blackbox_stream_, path_valid_);
  flight_blackbox_stream_ << ",\"waypoint_index\":"
                          << (path_valid_ ? waypoint_index_ + 1U : 0U)
                          << ",\"waypoint_count\":" << path_points_.size()
                          << ",\"path_goal_distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, path_goal_distance_m);
  flight_blackbox_stream_ << ",\"mission_goal_distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, mission_goal_distance_m);
  flight_blackbox_stream_ << ",\"final_trajectory_debug_turn_angle_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, upcoming_turn.angle_rad);
  flight_blackbox_stream_ << ",\"final_trajectory_debug_turn_valid\":";
  writeJsonBool(flight_blackbox_stream_, upcoming_turn.valid);
  flight_blackbox_stream_ << ",\"final_trajectory_debug_turn_waypoint_index\":"
                          << (upcoming_turn.valid ? upcoming_turn.waypoint_index + 1U
                                                  : 0U);
  flight_blackbox_stream_ << ",\"final_trajectory_debug_turn_distance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, upcoming_turn.distance_to_turn_m);
  flight_blackbox_stream_ << ",\"final_trajectory_debug_turn_point_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, upcoming_turn.turn_point.x);
  flight_blackbox_stream_ << ",\"final_trajectory_debug_turn_point_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, upcoming_turn.turn_point.y);
  flight_blackbox_stream_ << ",\"final_trajectory_debug_segment_type\":\""
                          << pathSegmentTypeName(upcoming_turn.angle_rad) << "\"";
  flight_blackbox_stream_ << ",\"tracking\":{\"valid\":";
  writeJsonBool(flight_blackbox_stream_, path_tracking.valid);
  flight_blackbox_stream_ << ",\"cross_track_error_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.cross_track_error_m);
  flight_blackbox_stream_ << ",\"signed_cross_track_error_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        path_tracking.signed_cross_track_error_m);
  flight_blackbox_stream_ << ",\"heading_error_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.heading_error_rad);
  flight_blackbox_stream_ << ",\"path_heading_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.path_heading_rad);
  flight_blackbox_stream_ << ",\"segment_start_index\":"
                          << path_tracking.segment_start_index << ",\"segment_t\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.segment_t);
  flight_blackbox_stream_ << ",\"projection_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.projection.x);
  flight_blackbox_stream_ << ",\"projection_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.projection.y);
  flight_blackbox_stream_ << "}}";
  flight_blackbox_stream_ << ",\"control\":{\"motion_phase\":\""
                          << motionPhaseName(hold_position)
                          << "\",\"final_goal_hold_active\":";
  writeJsonBool(flight_blackbox_stream_, final_goal_hold_active_);
  flight_blackbox_stream_ << "}";
  flight_blackbox_stream_ << ",\"obstacle\":{\"prohibited_grid_clearance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, prohibited_grid_clearance_m);
  flight_blackbox_stream_ << ",\"nearest_prohibited_cell_valid\":";
  writeJsonBool(flight_blackbox_stream_, nearest_prohibited_cell.valid);
  flight_blackbox_stream_ << ",\"nearest_prohibited_grid_clearance_m\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, nearest_prohibited_cell.clearance_m);
  flight_blackbox_stream_ << ",\"nearest_prohibited_cell_bearing_map_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        nearest_prohibited_cell.bearing_map_rad);
  flight_blackbox_stream_ << ",\"nearest_prohibited_cell_bearing_body_rad\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        nearest_prohibited_cell.bearing_body_rad);
  flight_blackbox_stream_ << ",\"nearest_prohibited_cell_bearing_body_deg\":";
  writeJsonNumberOrNull(flight_blackbox_stream_,
                        nearest_prohibited_cell.bearing_body_deg);
  flight_blackbox_stream_ << ",\"nearest_prohibited_cell_x\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, nearest_prohibited_cell.point.x);
  flight_blackbox_stream_ << ",\"nearest_prohibited_cell_y\":";
  writeJsonNumberOrNull(flight_blackbox_stream_, nearest_prohibited_cell.point.y);
  flight_blackbox_stream_ << "}}\n";
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
