#include "px4_offboard_node.hpp"

namespace drone_city_nav {

void Px4OffboardNode::onTimer() {
  const auto callback_wall_time = std::chrono::steady_clock::now();
  if (last_control_timer_callback_wall_time_.has_value()) {
    const double callback_gap_s =
        std::chrono::duration<double>(callback_wall_time -
                                      *last_control_timer_callback_wall_time_)
            .count();
    const double max_pose_staleness_s =
        static_cast<double>(max_pose_staleness_ns_) / 1.0e9;
    if (max_pose_staleness_s > 0.0 && callback_gap_s > max_pose_staleness_s) {
      const double local_position_callback_age_s =
          last_local_position_callback_wall_time_.has_value()
              ? std::chrono::duration<double>(callback_wall_time -
                                              *last_local_position_callback_wall_time_)
                    .count()
              : std::numeric_limits<double>::infinity();
      RCLCPP_WARN(get_logger(),
                  "Offboard timer callback gap: wall_gap_s=%.3f "
                  "local_position_callback_age_s=%.3f message_pose_age_s=%.3f",
                  callback_gap_s, local_position_callback_age_s,
                  localPositionAgeSeconds());
    }
  }
  last_control_timer_callback_wall_time_ = callback_wall_time;

  publishRvizDroneFollowTransform();

  if (crashed_) {
    handleCrashedVehicle();
    return;
  }

  updateFinalGoalHold();
  updateTemporaryReplanHold();
  advanceWaypointIfNeeded();
  updateTerminalCaptureState();
  publishOffboardControlMode();
  publishTrajectorySetpoint();
  publishOffboardDebugMarkers();
  logTelemetry();
  logControlSummary();

  if (setpoint_counter_ < warmup_setpoints_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Offboard warmup: sent %d/%d setpoints", setpoint_counter_,
                         warmup_setpoints_);
    ++setpoint_counter_;
    return;
  }

  const rclcpp::Time current_time = now();
  if ((current_time - last_command_time_).seconds() < command_resend_period_s_) {
    return;
  }

  const bool start_command_needed =
      (auto_offboard_ && !isOffboard()) || (auto_arm_ && !isArmed());
  if (start_command_needed && !missionStartReady()) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for mission start prerequisites before arming/offboard: "
        "pose_fresh=%s path_valid=%s waypoint=%zu/%zu trajectory_valid=%s "
        "trajectory_status=%.*s grid_seen=%s",
        localPositionFresh() ? "true" : "false", path_valid_ ? "true" : "false",
        waypoint_index_, path_points_.size(), finalTrajectoryReady() ? "true" : "false",
        static_cast<int>(
            trajectoryPlannerStatusName(last_trajectory_planner_stats_.status).size()),
        trajectoryPlannerStatusName(last_trajectory_planner_stats_.status).data(),
        prohibited_grid_valid_ ? "true" : "false");
    return;
  }

  if (auto_offboard_ && !isOffboard()) {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F,
                          6.0F);
    last_command_time_ = current_time;
    return;
  }

  if (auto_arm_ && !isArmed()) {
    publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
    last_command_time_ = current_time;
  }
}

void Px4OffboardNode::publishOffboardControlMode() {
  const OffboardSetpointMode mode = currentSetpointMode();
  const px4_msgs::msg::OffboardControlMode msg =
      buildOffboardControlMode(nowMicros(), mode);
  offboard_control_mode_pub_->publish(msg);
}

void Px4OffboardNode::publishTrajectorySetpoint() {
  const OffboardSetpointMode requested_mode = currentSetpointMode();
  const bool velocity_cruise_requested =
      requested_mode == OffboardSetpointMode::kVelocityCruise;
  const bool terminal_position_capture_requested =
      requested_mode == OffboardSetpointMode::kTerminalPositionCapture;
  last_terminal_position_capture_active_ = terminal_position_capture_requested;
  if (velocity_cruise_requested) {
    if (publishVelocityTrajectorySetpoint()) {
      return;
    }
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Velocity cruise requested but no valid velocity setpoint was produced; "
        "holding current position; rough route waypoints are not executable "
        "setpoints");
  }

  const bool had_previous_target = last_published_target_valid_;
  const Point2 previous_target = last_published_target_;
  Point2 desired_target = currentTarget();
  if (velocity_cruise_requested && local_position_valid_) {
    desired_target = current_position_;
  }
  if (terminal_position_capture_requested && trajectoryGoalReady()) {
    desired_target = trajectory_goal_;
  }
  const bool hold_position = (velocity_cruise_requested || shouldHoldPosition()) &&
                             !terminal_position_capture_requested;
  const Point2 target = selectCommandTarget(desired_target, hold_position);
  const double position_target_altitude_m =
      positionSetpointAltitudeM(terminal_position_capture_requested);
  commanded_target_ = target;
  commanded_target_valid_ = local_position_valid_;
  last_published_target_ = target;
  last_published_target_valid_ = true;

  const Point2 px4_local_target = mapToPx4Local(target);
  const px4_msgs::msg::TrajectorySetpoint msg = buildPositionTrajectorySetpoint(
      nowMicros(), px4_local_target, position_target_altitude_m, current_heading_rad_);
  updateCommandDiagnostics(target, previous_target, had_previous_target,
                           static_cast<double>(msg.yaw));
  resetVelocityDiagnostics();
  last_target_altitude_m_ = position_target_altitude_m;
  last_trajectory_altitude_target_valid_ =
      positionSetpointAltitudeValid(terminal_position_capture_requested);
  last_altitude_error_m_ = altitude_valid_
                               ? position_target_altitude_m - current_altitude_m_
                               : std::numeric_limits<double>::quiet_NaN();
  if (terminal_position_capture_requested) {
    last_offboard_setpoint_mode_ = OffboardSetpointMode::kTerminalPositionCapture;
  }

  trajectory_setpoint_pub_->publish(msg);
}

[[nodiscard]] OffboardSetpointMode Px4OffboardNode::currentSetpointMode() const {
  if (terminal_capture_state_.state == TerminalFlightState::kPositionCapture) {
    return OffboardSetpointMode::kTerminalPositionCapture;
  }
  return velocityCruiseReady() ? OffboardSetpointMode::kVelocityCruise
                               : OffboardSetpointMode::kPositionHold;
}

[[nodiscard]] bool Px4OffboardNode::velocityCruiseReady() const {
  return localPositionFresh() && navigationAllowed() && pathFollowingReady() &&
         !finalPathGoalReached() && !no_path_hold_target_valid_ &&
         !temporary_replan_hold_active_;
}

[[nodiscard]] Px4OffboardNode::TerminalCaptureState
Px4OffboardNode::computeTerminalCaptureState() const {
  TerminalStateMachineInput input{};
  input.final_goal_hold_active = final_goal_hold_active_;
  input.temporary_replan_hold_active = temporary_replan_hold_active_;
  input.no_path_hold_active = no_path_hold_target_valid_;
  input.prerequisites_valid = localPositionFresh() && navigationAllowed() &&
                              pathFollowingReady() && trajectoryGoalReady();
  input.previous_position_capture_latched = terminal_position_capture_latched_;
  input.velocity_terminal_capture_active = last_velocity_plan_.terminal_capture_active;
  input.current_speed_mps = current_speed_mps_;
  input.acceptance_radius_m = acceptance_radius_m_;
  input.terminal_capture_radius_m = velocity_follower_config_.terminal_capture_radius_m;
  input.terminal_capture_max_speed_mps =
      velocity_follower_config_.terminal_capture_max_speed_mps;
  input.terminal_position_capture_max_entry_speed_mps =
      velocity_follower_config_.terminal_position_capture_max_entry_speed_mps;
  input.terminal_stuck_speed_mps = velocity_follower_config_.terminal_stuck_speed_mps;

  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectorySamples(final_trajectory_samples_, current_position_);
  if (input.prerequisites_valid && projection.has_value()) {
    input.goal_distance_m = distance(current_position_, trajectory_goal_);
    input.remaining_trajectory_distance_m = std::max(
        0.0, final_trajectory_samples_.back().s_m - std::max(0.0, projection->s_m));
  } else if (input.prerequisites_valid) {
    input.prerequisites_valid = false;
  }

  return evaluateTerminalStateMachine(input);
}

void Px4OffboardNode::updateTerminalCaptureState() {
  terminal_capture_state_ = computeTerminalCaptureState();
  if (terminal_capture_state_.position_capture_latched) {
    latchTerminalPositionCaptureAltitude(terminal_capture_state_.reason);
  } else if (!final_goal_hold_active_ && !temporary_replan_hold_active_) {
    clearTerminalPositionCaptureAltitude();
  }
  terminal_position_capture_latched_ = terminal_capture_state_.position_capture_latched;
  last_terminal_position_capture_active_ =
      terminal_capture_state_.position_capture_active;
  last_terminal_position_capture_reason_ = terminal_capture_state_.reason;
  last_terminal_position_capture_goal_distance_m_ =
      terminal_capture_state_.goal_distance_m;
  last_terminal_position_capture_remaining_s_m_ =
      terminal_capture_state_.remaining_trajectory_distance_m;
  last_terminal_position_capture_speed_mps_ = terminal_capture_state_.current_speed_mps;
  last_terminal_position_capture_activation_radius_m_ =
      terminal_capture_state_.activation_radius_m;
  last_terminal_position_capture_max_entry_speed_mps_ =
      terminal_capture_state_.max_entry_speed_mps;
  last_terminal_position_capture_stuck_speed_mps_ =
      terminal_capture_state_.stuck_speed_mps;
}

[[nodiscard]] bool Px4OffboardNode::finalPathGoalReached() const {
  if (final_goal_hold_active_) {
    return true;
  }
  if (!localPositionFresh() || !trajectoryGoalReady()) {
    return false;
  }
  const bool geometrically_reached =
      distance(current_position_, trajectory_goal_) <= acceptance_radius_m_;
  if (!geometrically_reached) {
    return false;
  }
  if (!current_velocity_valid_ || !std::isfinite(current_speed_mps_)) {
    return true;
  }
  return current_speed_mps_ <= velocity_follower_config_.final_hold_max_speed_mps;
}

[[nodiscard]] bool Px4OffboardNode::finalPathGoalPassed() const {
  if (!localPositionFresh() || !path_valid_ || path_points_.size() < 2U ||
      !trajectoryGoalReady()) {
    return false;
  }

  const Point2 segment_start = path_points_[path_points_.size() - 2U];
  const Point2 segment_end = trajectory_goal_;
  const Point2 segment{segment_end.x - segment_start.x,
                       segment_end.y - segment_start.y};
  const double segment_length_sq = squaredDistance(segment_start, segment_end);
  if (segment_length_sq <= kTinyDistanceM * kTinyDistanceM) {
    return false;
  }

  const Point2 current_from_start{current_position_.x - segment_start.x,
                                  current_position_.y - segment_start.y};
  const double segment_t =
      (current_from_start.x * segment.x + current_from_start.y * segment.y) /
      segment_length_sq;
  if (segment_t < 1.0) {
    return false;
  }

  const double segment_length = std::sqrt(segment_length_sq);
  const double cross_track_m =
      std::abs(segment.x * current_from_start.y - segment.y * current_from_start.x) /
      segment_length;
  const double final_plane_cross_track_tolerance_m =
      std::max(2.0 * acceptance_radius_m_, 2.0);
  return cross_track_m <= final_plane_cross_track_tolerance_m;
}

[[nodiscard]] double Px4OffboardNode::consumeVelocityPlanDtS() {
  const rclcpp::Time current_time = get_clock()->now();
  double dt_s = static_cast<double>(kControllerPeriod.count()) / 1000.0;
  if (last_velocity_plan_time_.nanoseconds() > 0 &&
      current_time > last_velocity_plan_time_) {
    dt_s = std::clamp((current_time - last_velocity_plan_time_).seconds(), 0.001, 1.0);
  }
  last_velocity_plan_time_ = current_time;
  return dt_s;
}

bool Px4OffboardNode::publishVelocityTrajectorySetpoint() {
  const bool had_previous_target = last_published_target_valid_;
  const Point2 previous_target = last_published_target_;
  const Point2 target = currentTarget();
  const double dt_s = consumeVelocityPlanDtS();
  const VelocitySetpointPlan plan = planVelocitySetpoint(
      final_trajectory_samples_, trajectory_speed_profile_, current_position_,
      current_velocity_, current_velocity_valid_, current_altitude_m_, altitude_valid_,
      current_vertical_velocity_up_mps_, current_vertical_velocity_valid_, dt_s,
      velocity_follower_state_, velocity_follower_config_);
  last_velocity_plan_ = plan;
  last_velocity_plan_valid_ = plan.valid;
  if (!plan.valid || plan.final_goal_reached) {
    resetVelocityDiagnostics();
    return false;
  }

  const VerticalSetpointPlan vertical_plan =
      planVerticalSetpointForCurrentTrajectory(plan, dt_s);
  last_vertical_plan_ = vertical_plan;
  last_vertical_plan_valid_ = vertical_plan.valid;
  last_target_altitude_m_ = vertical_plan.target_z_m;
  last_altitude_error_m_ = vertical_plan.z_error_m;
  last_trajectory_altitude_target_valid_ = vertical_plan.trajectory_target_valid;
  const double vz_ned = vertical_plan.commanded_vz_ned_mps;
  const px4_msgs::msg::TrajectorySetpoint msg = buildVelocityTrajectorySetpoint(
      nowMicros(), plan.velocity_xy, vz_ned, current_heading_rad_);

  velocity_follower_state_.previous_velocity_setpoint = plan.velocity_xy;
  velocity_follower_state_.previous_velocity_setpoint_valid = true;
  velocity_follower_state_.previous_velocity_acceleration_setpoint =
      plan.velocity_setpoint_acceleration_xy;
  velocity_follower_state_.previous_velocity_acceleration_setpoint_valid =
      std::isfinite(plan.velocity_setpoint_acceleration_mps2);
  velocity_follower_state_.previous_scalar_speed_command_mps =
      plan.accel_limited_speed_mps;
  velocity_follower_state_.previous_scalar_speed_command_valid =
      std::isfinite(plan.accel_limited_speed_mps);
  velocity_follower_state_.previous_terminal_capture_active =
      plan.terminal_capture_active;
  velocity_follower_state_.previous_terminal_capture_speed_limit_mps =
      plan.terminal_capture_speed_limit_mps;
  velocity_follower_state_.previous_terminal_capture_speed_limit_valid =
      plan.terminal_capture_active &&
      std::isfinite(plan.terminal_capture_speed_limit_mps);
  if (vertical_plan.valid) {
    vertical_follower_state_.previous_command_valid = true;
    vertical_follower_state_.previous_commanded_vz_mps = vertical_plan.commanded_vz_mps;
    vertical_follower_state_.previous_vertical_accel_mps2 =
        std::isfinite(vertical_plan.vertical_accel_mps2)
            ? vertical_plan.vertical_accel_mps2
            : 0.0;
  }
  last_velocity_setpoint_ = plan.velocity_xy;
  last_vertical_velocity_setpoint_mps_ = vz_ned;
  last_velocity_setpoint_speed_mps_ = plan.speed_mps;
  last_offboard_setpoint_mode_ = OffboardSetpointMode::kVelocityCruise;
  commanded_target_ = target;
  commanded_target_valid_ = true;
  last_published_target_ = target;
  last_published_target_valid_ = true;
  updateCommandDiagnostics(target, previous_target, had_previous_target,
                           static_cast<double>(msg.yaw));
  trajectory_setpoint_pub_->publish(msg);
  return true;
}

void Px4OffboardNode::publishVehicleCommand(const std::uint32_t command,
                                            const float param1, const float param2) {
  const VehicleCommandEndpoint endpoint{target_system_, target_component_,
                                        source_system_, source_component_};
  const px4_msgs::msg::VehicleCommand msg =
      buildVehicleCommand(nowMicros(), command, param1, param2, endpoint);
  vehicle_command_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Sent PX4 command: %s (%u) param1=%.2f param2=%.2f",
              commandName(command), static_cast<unsigned int>(command),
              static_cast<double>(param1), static_cast<double>(param2));
}

void Px4OffboardNode::advanceWaypointIfNeeded() {
  if (!pathFollowingReady() || !localPositionFresh() || final_goal_hold_active_ ||
      temporary_replan_hold_active_) {
    return;
  }
  const std::size_t previous_waypoint_index = waypoint_index_;
  const std::size_t next_waypoint_index = drone_city_nav::advanceWaypointIndex(
      path_points_, current_position_, waypoint_index_, pathFollowerConfig());

  if (next_waypoint_index == previous_waypoint_index) {
    return;
  }

  waypoint_index_ = next_waypoint_index;
  if (waypoint_index_ != previous_waypoint_index) {
    const Point2 target = path_points_[waypoint_index_];
    RCLCPP_INFO(get_logger(),
                "Waypoint advanced: index=%zu/%zu current=(%.2f, %.2f) "
                "target=(%.2f, %.2f)",
                waypoint_index_ + 1U, path_points_.size(), current_position_.x,
                current_position_.y, target.x, target.y);
  }
}

[[nodiscard]] Point2 Px4OffboardNode::currentTarget() const {
  if (!navigationAllowed()) {
    if (takeoff_hold_target_valid_) {
      return takeoff_hold_target_;
    }
    if (local_position_valid_) {
      return current_position_;
    }
  }

  if (final_goal_hold_active_) {
    return final_goal_hold_target_;
  }
  if (temporary_replan_hold_active_) {
    return temporary_replan_hold_target_;
  }

  if (localPositionFresh() && pathFollowingReady() && !path_points_.empty()) {
    return path_points_[std::min(waypoint_index_, path_points_.size() - 1U)];
  }

  if (no_path_hold_target_valid_) {
    return no_path_hold_target_;
  }

  if (local_position_valid_) {
    return current_position_;
  }

  return Point2{hold_x_m_, hold_y_m_};
}

[[nodiscard]] Point2 Px4OffboardNode::selectCommandTarget(const Point2 desired_target,
                                                          const bool hold_position) {
  if (!local_position_valid_) {
    return desired_target;
  }
  if (!localPositionFresh()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Holding last known position because PX4 local position is stale: "
        "age_s=%.2f max_age_s=%.2f target=(%.2f, %.2f)",
        localPositionAgeSeconds(), static_cast<double>(max_pose_staleness_ns_) / 1.0e9,
        current_position_.x, current_position_.y);
    return current_position_;
  }
  if (final_goal_hold_active_ || temporary_replan_hold_active_) {
    return desired_target;
  }
  if (hold_position) {
    return current_position_;
  }

  return desired_target;
}

[[nodiscard]] bool Px4OffboardNode::shouldHoldPosition() const {
  return final_goal_hold_active_ || temporary_replan_hold_active_ ||
         !localPositionFresh() || !navigationAllowed() || !pathFollowingReady() ||
         finalPathGoalReached();
}

[[nodiscard]] bool Px4OffboardNode::finalTrajectoryReady() const {
  return trajectory_valid_ && trajectorySamplesAreUsable(final_trajectory_samples_) &&
         trajectory_speed_profile_.valid && final_trajectory_samples_.size() >= 2U;
}

[[nodiscard]] bool Px4OffboardNode::trajectoryGoalReady() const {
  return trajectory_goal_valid_ && std::isfinite(trajectory_goal_.x) &&
         std::isfinite(trajectory_goal_.y);
}

[[nodiscard]] bool Px4OffboardNode::pathFollowingReady() const {
  return path_valid_ && trajectoryGoalReady() && finalTrajectoryReady();
}

[[nodiscard]] bool Px4OffboardNode::missionStartReady() const {
  return localPositionFresh() && pathFollowingReady();
}

[[nodiscard]] UpcomingTurn
Px4OffboardNode::upcomingTurnAtWaypoint(const std::size_t index) const {
  if (!path_valid_ || !localPositionFresh()) {
    return UpcomingTurn{};
  }
  return drone_city_nav::upcomingTurnAtWaypoint(path_points_, index, current_position_,
                                                true, pathFollowerConfig());
}

[[nodiscard]] const char*
Px4OffboardNode::pathSegmentTypeName(const double turn_angle_rad) const {
  if (!path_valid_) {
    return "no_path";
  }
  if (turn_angle_rad < 0.15) {
    return "straight";
  }
  if (turn_angle_rad < std::numbers::pi / 2.0) {
    return "gentle_turn";
  }
  return "sharp_turn";
}

[[nodiscard]] const char*
Px4OffboardNode::motionPhaseName(const bool hold_position) const noexcept {
  if (final_goal_hold_active_) {
    return "final_goal_hold";
  }
  if (temporary_replan_hold_active_) {
    return "temporary_replan_hold";
  }
  if (no_path_hold_target_valid_) {
    return "hold_no_path";
  }
  if (path_valid_ && !finalTrajectoryReady()) {
    return "hold_invalid_trajectory";
  }
  if (last_terminal_position_capture_active_) {
    return "terminal_position_capture";
  }
  if (hold_position) {
    return "hold";
  }
  return "path_following";
}

[[nodiscard]] bool Px4OffboardNode::prohibitedGridFresh() const {
  if (!prohibited_grid_valid_ || last_prohibited_grid_update_ns_ <= 0) {
    return false;
  }
  if (max_clearance_grid_staleness_ns_ <= 0) {
    return true;
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  return now_ns >= last_prohibited_grid_update_ns_ &&
         now_ns - last_prohibited_grid_update_ns_ <= max_clearance_grid_staleness_ns_;
}

[[nodiscard]] bool Px4OffboardNode::localPositionFresh() const {
  if (!local_position_valid_ || last_local_position_update_ns_ <= 0) {
    return false;
  }
  if (max_pose_staleness_ns_ <= 0) {
    return true;
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  return now_ns >= last_local_position_update_ns_ &&
         now_ns - last_local_position_update_ns_ <= max_pose_staleness_ns_;
}

[[nodiscard]] std::optional<OccupancyGrid2D>
Px4OffboardNode::currentProhibitedGrid() const {
  if (!prohibited_grid_valid_ || !(prohibited_grid_.info.resolution > 0.0F) ||
      prohibited_grid_.info.width == 0U || prohibited_grid_.info.height == 0U ||
      prohibited_grid_.info.width >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      prohibited_grid_.info.height >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }

  const auto width = static_cast<int>(prohibited_grid_.info.width);
  const auto height = static_cast<int>(prohibited_grid_.info.height);
  const std::size_t expected_data_size =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (prohibited_grid_.data.size() != expected_data_size) {
    return std::nullopt;
  }

  OccupancyGrid2D grid{GridBounds{
      prohibited_grid_.info.origin.position.x, prohibited_grid_.info.origin.position.y,
      static_cast<double>(prohibited_grid_.info.resolution), width, height}};
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const GridIndex cell{x, y};
      const std::size_t index =
          static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(x);
      if (prohibited_grid_.data[index] >= kInflatedOccupancyValue) {
        grid.setOccupied(cell);
      } else if (prohibited_grid_.data[index] == 0) {
        grid.setFree(cell);
      }
    }
  }
  return grid;
}

[[nodiscard]] double Px4OffboardNode::localPositionAgeSeconds() const {
  if (last_local_position_update_ns_ <= 0) {
    return std::numeric_limits<double>::infinity();
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (now_ns <= last_local_position_update_ns_) {
    return 0.0;
  }
  return static_cast<double>(now_ns - last_local_position_update_ns_) / 1.0e9;
}

[[nodiscard]] double Px4OffboardNode::attitudeAgeSeconds() const {
  if (last_attitude_update_ns_ <= 0) {
    return std::numeric_limits<double>::infinity();
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (now_ns <= last_attitude_update_ns_) {
    return 0.0;
  }
  return static_cast<double>(now_ns - last_attitude_update_ns_) / 1.0e9;
}

[[nodiscard]] PathTrackingDiagnostics Px4OffboardNode::pathTrackingDiagnostics() const {
  PathTrackingDiagnostics diagnostics{};
  const auto projection =
      projectOnTrajectorySamples(final_trajectory_samples_, current_position_);
  if (!projection.has_value()) {
    return diagnostics;
  }

  if (!(std::hypot(projection->tangent.x, projection->tangent.y) > kTinyDistanceM)) {
    return diagnostics;
  }

  const Point2 left_normal{-projection->tangent.y, projection->tangent.x};
  const Point2 relative{current_position_.x - projection->point.x,
                        current_position_.y - projection->point.y};
  const double signed_error_m = relative.x * left_normal.x + relative.y * left_normal.y;
  const double path_heading_rad =
      std::atan2(projection->tangent.y, projection->tangent.x);

  diagnostics.valid = true;
  diagnostics.segment_start_index = projection->segment_index;
  diagnostics.segment_t = projection->segment_t;
  diagnostics.cross_track_error_m = std::sqrt(projection->distance_sq);
  diagnostics.signed_cross_track_error_m = signed_error_m;
  diagnostics.path_heading_rad = path_heading_rad;
  diagnostics.heading_error_rad =
      normalizeAngle(current_heading_rad_ - path_heading_rad);
  diagnostics.projection = projection->point;
  diagnostics.projection_z_m =
      trajectorySampleAltitudeAtS(final_trajectory_samples_, projection->s_m);
  return diagnostics;
}

[[nodiscard]] double
Px4OffboardNode::estimateProhibitedGridClearanceM(const Point2 point) const {
  return estimateGridClearanceM(point, kInflatedOccupancyValue);
}

[[nodiscard]] NearestProhibitedCellDiagnostic
Px4OffboardNode::nearestProhibitedCellDiagnostic(
    const Point2 point, const std::int8_t min_occupancy_value) const {
  NearestProhibitedCellDiagnostic diagnostic{};
  if (!prohibitedGridFresh() || !(prohibited_grid_.info.resolution > 0.0F) ||
      prohibited_grid_.info.width == 0U || prohibited_grid_.info.height == 0U) {
    return diagnostic;
  }

  const auto width = static_cast<int>(prohibited_grid_.info.width);
  const auto height = static_cast<int>(prohibited_grid_.info.height);
  const std::size_t expected_data_size =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (prohibited_grid_.data.size() != expected_data_size) {
    return diagnostic;
  }

  const double resolution = static_cast<double>(prohibited_grid_.info.resolution);
  const double origin_x = prohibited_grid_.info.origin.position.x;
  const double origin_y = prohibited_grid_.info.origin.position.y;
  const GridIndex center{
      static_cast<int>(std::floor((point.x - origin_x) / resolution)),
      static_cast<int>(std::floor((point.y - origin_y) / resolution))};
  if (center.x < 0 || center.y < 0 || center.x >= width || center.y >= height) {
    return diagnostic;
  }

  const int radius_cells = static_cast<int>(
      std::ceil(kProhibitedGridClearanceDiagnosticRadiusM / resolution));
  const int min_x = std::max(center.x - radius_cells, 0);
  const int max_x = std::min(center.x + radius_cells, width - 1);
  const int min_y = std::max(center.y - radius_cells, 0);
  const int max_y = std::min(center.y + radius_cells, height - 1);

  double nearest_distance_m = std::numeric_limits<double>::infinity();
  Point2 nearest_point{};
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      const std::size_t data_index =
          static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(x);
      if (prohibited_grid_.data[data_index] < min_occupancy_value) {
        continue;
      }
      const Point2 cell_center{origin_x + (static_cast<double>(x) + 0.5) * resolution,
                               origin_y + (static_cast<double>(y) + 0.5) * resolution};
      const double candidate_distance_m = distance(point, cell_center);
      if (candidate_distance_m < nearest_distance_m) {
        nearest_distance_m = candidate_distance_m;
        nearest_point = cell_center;
      }
    }
  }

  if (!std::isfinite(nearest_distance_m)) {
    return diagnostic;
  }

  const double bearing_map_rad =
      std::atan2(nearest_point.y - point.y, nearest_point.x - point.x);
  diagnostic.valid = true;
  diagnostic.clearance_m = nearest_distance_m;
  diagnostic.bearing_map_rad = bearing_map_rad;
  diagnostic.bearing_body_rad = normalizeAngle(bearing_map_rad - current_heading_rad_);
  diagnostic.bearing_body_deg = radiansToDegrees(diagnostic.bearing_body_rad);
  diagnostic.point = nearest_point;
  return diagnostic;
}

[[nodiscard]] double
Px4OffboardNode::estimateGridClearanceM(const Point2 point,
                                        const std::int8_t min_occupancy_value) const {
  if (!prohibitedGridFresh()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return occupancyGridClearanceM(prohibited_grid_, point,
                                 kProhibitedGridClearanceDiagnosticRadiusM,
                                 min_occupancy_value);
}

void Px4OffboardNode::updateCommandDiagnostics(const Point2 target,
                                               const Point2 previous_target,
                                               const bool had_previous_target,
                                               const double commanded_yaw_rad) {
  last_commanded_target_distance_m_ = local_position_valid_
                                          ? distance(current_position_, target)
                                          : std::numeric_limits<double>::quiet_NaN();
  last_commanded_target_delta_m_ = had_previous_target
                                       ? distance(previous_target, target)
                                       : std::numeric_limits<double>::quiet_NaN();
  last_commanded_yaw_rad_ = commanded_yaw_rad;
}

[[nodiscard]] Point2 Px4OffboardNode::mapToPx4Local(const Point2 point) const noexcept {
  return Point2{point.x - px4_local_origin_.x, point.y - px4_local_origin_.y};
}

void Px4OffboardNode::updateNavigationStartState() {
  if (navigation_started_ || min_navigation_altitude_m_ <= 0.0 || !altitude_valid_) {
    return;
  }
  if (current_altitude_m_ < min_navigation_altitude_m_) {
    return;
  }

  const rclcpp::Time now_time = get_clock()->now();
  if (!navigation_altitude_reached_) {
    navigation_altitude_reached_ = true;
    navigation_altitude_reached_time_ = now_time;
    commanded_target_valid_ = false;
    last_published_target_valid_ = false;
    RCLCPP_INFO(get_logger(),
                "Navigation altitude reached; holding before horizontal flight: "
                "altitude=%.2f required=%.2f hover_s=%.2f",
                current_altitude_m_, min_navigation_altitude_m_, takeoff_hover_s_);
  }

  const double hover_elapsed_s =
      (now_time - navigation_altitude_reached_time_).seconds();
  if (hover_elapsed_s + kTinyDistanceM < takeoff_hover_s_) {
    return;
  }

  navigation_started_ = true;
  commanded_target_valid_ = false;
  last_published_target_valid_ = false;
  RCLCPP_INFO(get_logger(),
              "Takeoff hover complete; horizontal navigation enabled: "
              "altitude=%.2f hover_elapsed=%.2f required_hover=%.2f",
              current_altitude_m_, hover_elapsed_s, takeoff_hover_s_);
}

[[nodiscard]] bool Px4OffboardNode::navigationAllowed() const {
  if (min_navigation_altitude_m_ <= 0.0) {
    return true;
  }
  return navigation_started_;
}

[[nodiscard]] bool Px4OffboardNode::isArmed() const {
  return vehicle_status_valid_ && vehicle_status_.arming_state ==
                                      px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

[[nodiscard]] bool Px4OffboardNode::isOffboard() const {
  return vehicle_status_valid_ &&
         vehicle_status_.nav_state ==
             px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

[[nodiscard]] std::uint64_t Px4OffboardNode::nowMicros() const {
  return static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
}

void Px4OffboardNode::logControlSummary() {
  const Point2 target = loggedTarget();
  const double target_distance = local_position_valid_
                                     ? distance(current_position_, target)
                                     : std::numeric_limits<double>::quiet_NaN();
  const double mission_goal_distance = local_position_valid_
                                           ? distance(current_position_, mission_goal_)
                                           : std::numeric_limits<double>::quiet_NaN();
  const double path_goal_distance = local_position_valid_ && trajectoryGoalReady()
                                        ? distance(current_position_, trajectory_goal_)
                                        : std::numeric_limits<double>::quiet_NaN();
  const double prohibited_grid_clearance_m =
      local_position_valid_ ? estimateProhibitedGridClearanceM(current_position_)
                            : std::numeric_limits<double>::quiet_NaN();
  const bool hold_position = shouldHoldPosition();
  const bool pose_fresh = localPositionFresh();
  const double pose_age_s = localPositionAgeSeconds();
  const UpcomingTurn upcoming_turn = upcomingTurnAtWaypoint(waypoint_index_);
  const double turn_angle_rad = upcoming_turn.angle_rad;
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Offboard summary: local_position=%s pose_fresh=%s pose_age_s=%.2f "
      "altitude=%.2f nav_allowed=%s "
      "status=%s armed=%s offboard=%s path=%s hold=%s waypoint=%zu/%zu "
      "motion_phase=%s control_mode=%s final_trajectory_segment=%s "
      "trajectory[valid=%s line_segments=%zu arc_segments=%zu length=%.2f "
      "s=%.2f segment=%zu type=%s curvature=%.4f arc_radius=%.2f "
      "samples=%zu debug_samples=%zu status=%.*s "
      "corridor_width[min=%.2f mean=%.2f] lateral_offset_max=%.2f] "
      "tracking_prediction[horizon=%.2fs distance=%.2f "
      "predicted=(%.2f, %.2f) current_cross=%.2f predicted_cross=%.2f "
      "response_delay_distance=%.2f] "
      "current=(%.2f, %.2f) target=(%.2f, %.2f) "
      "distance_to_target=%.2f distance_to_path_goal=%.2f "
      "distance_to_mission_goal=%.2f actual_speed=%.2f "
      "velocity_setpoint=(%.2f, %.2f, %.2f) velocity_setpoint_speed=%.2f "
      "velocity_setpoint_accel=%.2f velocity_setpoint_jerk=%.2f "
      "lateral_control[feedback=%.2f derivative=%.2f curvature_ff=%.2f "
      "raw=%.2f final=%.2f curvature_angle=%.1fdeg "
      "curvature_context=%.2f p_gain_factor=%.2f] "
      "speed_limit_reason=%s "
      "terminal_capture[active=%s goal_distance=%.2f signed_along=%.2f "
      "remaining_s=%.2f "
      "speed_limit=%.2f gain_limit=%.2f max_speed=%.2f "
      "brake_limit=%.2f activation=%.2f decel=%.2f margin=%.2f "
      "hold_distance_met=%s hold_speed_met=%s trigger_goal=%s "
      "trigger_remaining=%s] "
      "terminal_position_capture[active=%s reason=%s goal_distance=%.2f "
      "remaining_s=%.2f speed=%.2f radius=%.2f max_entry_speed=%.2f "
      "stuck_speed=%.2f] "
      "raw_speed_limit=%.2f profile_speed_limit=%.2f "
      "lookahead_distance=%.2f lookahead_speed_limit=%.2f "
      "speed_after_lookahead=%.2f lookahead_constraint[type=%s index=%zu "
      "distance=%.2f] "
      "final_command_speed=%.2f accel_limited_speed=%.2f "
      "constraint[type=%s index=%zu distance=%.2f speed=%.2f allowed=%.2f "
      "curve_radius=%.2f] "
      "final_stop[distance=%.2f braking_distance=%.2f] "
      "velocity_delta=%.2f trajectory_cross_track=%.2f "
      "lateral_control_velocity=(%.2f, %.2f) raw_lateral_control=(%.2f, %.2f) "
      "control_tangent[smoothed=%s mode=%s raw=(%.2f, %.2f) heading_span=%.1fdeg "
      "max_abs_curvature=%.4f window=(%.2f, %.2f)] "
      "cross_track_lateral_velocity=%.2f "
      "smoother[reset_reason=%s path_update_resets=%" PRIu64
      " path_frame=%s lateral_accel=%.2f] "
      "altitude[target_z=%.2f actual_z=%.2f z_error=%.2f target_vz=%.2f "
      "feedback_vz=%.2f desired_vz=%.2f commanded_vz=%.2f "
      "commanded_vz_ned=%.2f trajectory_target_valid=%s passage_mode=%s "
      "passage_id=%s slope=%.4f constraint=%s hard_window=%s "
      "safe=[%.2f, %.2f] gate_z=%.2f safe_error=%.2f reason=%s] "
      "diagnostic_rough_route_turn[valid=%s index=%zu distance=%.2f angle=%.2f] "
      "final_goal_hold=%s "
      "prohibited_grid_clearance=%.2f",
      local_position_valid_ ? "true" : "false", pose_fresh ? "true" : "false",
      pose_age_s, current_altitude_m_, navigationAllowed() ? "true" : "false",
      vehicle_status_valid_ ? "true" : "false", isArmed() ? "true" : "false",
      isOffboard() ? "true" : "false", path_valid_ ? "true" : "false",
      hold_position ? "true" : "false", path_valid_ ? waypoint_index_ + 1U : 0U,
      path_points_.size(), motionPhaseName(hold_position),
      offboardSetpointModeName(last_offboard_setpoint_mode_),
      pathSegmentTypeName(turn_angle_rad), trajectory_valid_ ? "true" : "false",
      last_trajectory_metrics_.line_segments, last_trajectory_metrics_.arc_segments,
      last_trajectory_metrics_.length_m, last_velocity_plan_.trajectory_s_m,
      last_velocity_plan_.trajectory_segment_index,
      trajectorySegmentKindName(last_velocity_plan_.trajectory_segment_kind),
      last_velocity_plan_.trajectory_curvature_1pm,
      last_velocity_plan_.trajectory_arc_radius_m, final_trajectory_samples_.size(),
      last_final_trajectory_debug_samples_,
      static_cast<int>(
          trajectoryPlannerStatusName(last_trajectory_planner_stats_.status).size()),
      trajectoryPlannerStatusName(last_trajectory_planner_stats_.status).data(),
      last_trajectory_planner_stats_.corridor.min_width_m,
      last_trajectory_planner_stats_.corridor.mean_width_m,
      last_trajectory_planner_stats_.trajectory_optimizer.max_abs_offset_m,
      last_velocity_plan_.prediction_horizon_s,
      last_velocity_plan_.prediction_distance_m,
      last_velocity_plan_.predicted_position.x,
      last_velocity_plan_.predicted_position.y,
      last_velocity_plan_.current_cross_track_error_m,
      last_velocity_plan_.predicted_cross_track_error_m,
      last_velocity_plan_.response_delay_distance_m, current_position_.x,
      current_position_.y, target.x, target.y, target_distance, path_goal_distance,
      mission_goal_distance, current_speed_mps_, last_velocity_setpoint_.x,
      last_velocity_setpoint_.y, last_vertical_velocity_setpoint_mps_,
      last_velocity_setpoint_speed_mps_,
      last_velocity_plan_.velocity_setpoint_acceleration_mps2,
      last_velocity_plan_.velocity_setpoint_jerk_mps3,
      last_velocity_plan_.cross_track_feedback_mps,
      last_velocity_plan_.cross_track_derivative_damping_mps,
      last_velocity_plan_.curvature_feedforward_mps,
      last_velocity_plan_.raw_lateral_control_mps,
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
      last_velocity_plan_.lateral_control_velocity.x,
      last_velocity_plan_.lateral_control_velocity.y,
      last_velocity_plan_.raw_lateral_control_velocity.x,
      last_velocity_plan_.raw_lateral_control_velocity.y,
      last_velocity_plan_.control_tangent_smoothed ? "true" : "false",
      controlProjectionSmoothingModeName(
          last_velocity_plan_.control_projection_smoothing_mode),
      last_velocity_plan_.control_tangent_raw.x,
      last_velocity_plan_.control_tangent_raw.y,
      radiansToDegrees(last_velocity_plan_.control_tangent_smoothing_heading_span_rad),
      last_velocity_plan_.control_tangent_smoothing_max_abs_curvature_1pm,
      last_velocity_plan_.control_tangent_smoothing_window_start_s_m,
      last_velocity_plan_.control_tangent_smoothing_window_end_s_m,
      last_velocity_plan_.cross_track_lateral_velocity_mps,
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
      last_vertical_plan_.vertical_hard_window_active ? "true" : "false",
      last_vertical_plan_.vertical_safe_min_z_m,
      last_vertical_plan_.vertical_safe_max_z_m, last_vertical_plan_.vertical_gate_z_m,
      last_vertical_plan_.vertical_safe_error_m, last_vertical_plan_.reason.c_str(),
      upcoming_turn.valid ? "true" : "false",
      upcoming_turn.valid ? upcoming_turn.waypoint_index + 1U : 0U,
      upcoming_turn.distance_to_turn_m, turn_angle_rad,
      final_goal_hold_active_ ? "true" : "false", prohibited_grid_clearance_m);
}

} // namespace drone_city_nav
