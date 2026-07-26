#include "px4_offboard_node.hpp"

namespace drone_city_nav {

void Px4OffboardNode::publishTruncationSuffixAck(
    const msg::ExecutableTrajectory& command,
    const TruncationSuffixAckDecision decision, const std::string_view reason) {
  const std::optional<TrajectoryEndpointSemantics> endpoint_semantics =
      trajectoryEndpointSemanticsFromWire(command.endpoint_semantics);
  const bool explicitly_required = command.requires_activation_ack ||
                                   command.truncation_suffix ||
                                   command.activate_after_terminal_hold;
  const bool ack_required =
      explicitly_required ||
      (endpoint_semantics.has_value() &&
       trajectoryActivationAckRequired(TrajectoryActivationAckContract{
           .endpoint_semantics = *endpoint_semantics,
       }));
  if (!ack_required || !truncation_suffix_ack_pub_) {
    return;
  }
  msg::TruncationSuffixAck ack;
  ack.header = makeDebugHeader();
  ack.path_id = command.path_id;
  ack.truncation_generation = command.truncation_generation;
  ack.temporary_prefix_fingerprint = command.temporary_prefix_fingerprint;
  ack.decision = static_cast<std::uint8_t>(decision);
  ack.reason = reason;
  truncation_suffix_ack_pub_->publish(ack);
  const std::optional<TruncationSuffixActivationMode> activation_mode =
      truncationSuffixActivationModeFromValue(
          command.truncation_suffix_activation_mode);
  RCLCPP_INFO(get_logger(),
              "%s suffix_activation=%s activation_ack decision=%s reason=%s "
              "path_id=%" PRIu64 " generation=%" PRIu64 " prefix_fingerprint=%" PRIu64,
              command.truncation_suffix ? "REPLAN_TRUNCATION" : "ROLLOUT",
              activation_mode.has_value()
                  ? truncationSuffixActivationModeName(*activation_mode)
                  : "invalid",
              truncationSuffixAckDecisionName(decision), ack.reason.c_str(),
              ack.path_id, ack.truncation_generation, ack.temporary_prefix_fingerprint);
}

void Px4OffboardNode::publishReplanTruncation(
    const msg::ReplanBlockerEvent& blocker,
    const TrajectoryPointSample& terminal_sample,
    const std::uint64_t prefix_fingerprint, const bool immediate_hold) {
  msg::ReplanTruncation confirmation;
  confirmation.header = makeDebugHeader();
  confirmation.blocked_path_id = blocker.blocked_path_id;
  confirmation.truncation_generation = blocker.truncation_generation;
  confirmation.truncation_position.x = terminal_sample.point.x;
  confirmation.truncation_position.y = terminal_sample.point.y;
  confirmation.truncation_position.z = terminal_sample.z_m;
  confirmation.truncation_tangent.x = terminal_sample.tangent.x;
  confirmation.truncation_tangent.y = terminal_sample.tangent.y;
  confirmation.truncation_tangent.z = terminal_sample.vertical_slope_dz_ds;
  confirmation.truncation_altitude_m = terminal_sample.z_m;
  confirmation.temporary_prefix_fingerprint = prefix_fingerprint;
  confirmation.immediate_hold = immediate_hold;
  replan_truncation_pub_->publish(confirmation);
  RCLCPP_WARN(get_logger(),
              "REPLAN_TRUNCATION confirmation_published=true blocked_path_id=%" PRIu64
              " generation=%" PRIu64 " point=(%.2f,%.2f,%.2f) tangent=(%.3f,%.3f) "
              "prefix_fingerprint=%" PRIu64 " immediate_hold=%s",
              confirmation.blocked_path_id, confirmation.truncation_generation,
              confirmation.truncation_position.x, confirmation.truncation_position.y,
              confirmation.truncation_altitude_m, confirmation.truncation_tangent.x,
              confirmation.truncation_tangent.y,
              confirmation.temporary_prefix_fingerprint,
              immediate_hold ? "true" : "false");
}

void Px4OffboardNode::updateFinalGoalHold() {
  if (final_goal_hold_active_ || temporary_replan_truncation_active_ ||
      active_trajectory_endpoint_semantics_ !=
          TrajectoryEndpointSemantics::kMissionGoal ||
      !finalPathGoalReached()) {
    return;
  }

  final_goal_hold_active_ = true;
  final_goal_hold_target_ = trajectory_goal_;
  latchTerminalPositionCaptureAltitude("final_goal_hold");
  no_path_hold_target_valid_ = false;
  resetVelocityDiagnostics();
  RCLCPP_INFO(get_logger(),
              "Final goal hold latched: target=(%.2f, %.2f) current=(%.2f, %.2f) "
              "distance=%.2f actual_speed=%.2f crossed_final_plane=%s",
              final_goal_hold_target_.x, final_goal_hold_target_.y, current_position_.x,
              current_position_.y, distance(current_position_, final_goal_hold_target_),
              current_speed_mps_, finalPathGoalPassed() ? "true" : "false");
}

void Px4OffboardNode::updateLocalHorizonExhaustionHold() {
  if (active_trajectory_endpoint_semantics_ !=
          TrajectoryEndpointSemantics::kLocalHorizon ||
      !trajectorySamplesAreUsable(final_trajectory_samples_) || !localPositionFresh()) {
    local_horizon_low_buffer_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    local_horizon_exhaustion_active_ = false;
    return;
  }
  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectorySamples(final_trajectory_samples_, current_position_);
  if (!projection.has_value()) {
    return;
  }
  const double remaining_s_m =
      std::max(0.0, final_trajectory_samples_.back().s_m - projection->s_m);
  const rclcpp::Time now_time = now();
  if (remaining_s_m > local_horizon_execution_config_.minimum_buffer_m) {
    local_horizon_low_buffer_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    local_horizon_exhaustion_active_ = false;
    return;
  }
  if (local_horizon_low_buffer_since_.nanoseconds() == 0) {
    local_horizon_low_buffer_since_ = now_time;
  }
  const double low_buffer_duration_s =
      (now_time - local_horizon_low_buffer_since_).seconds();
  const LocalHorizonExecutionDecision decision = evaluateLocalHorizonExecution(
      LocalHorizonExecutionInput{
          .semantics = active_trajectory_endpoint_semantics_,
          .remaining_s_m = remaining_s_m,
          .low_buffer_duration_s = low_buffer_duration_s,
          .endpoint_captured = finalPathGoalReached(),
      },
      local_horizon_execution_config_);
  local_horizon_exhaustion_active_ = decision.terminal_capture_enabled;
  if (!decision.latch_temporary_hold || temporary_replan_hold_active_) {
    return;
  }
  temporary_replan_hold_active_ = true;
  temporary_replan_hold_target_ = trajectory_goal_;
  latchTerminalPositionCaptureAltitude("local_horizon_exhausted");
  resetVelocityDiagnostics();
  RCLCPP_WARN(get_logger(),
              "LOCAL_HORIZON temporary_hold_latched=true remaining_s=%.2f "
              "low_buffer_duration_s=%.2f target=(%.2f,%.2f)",
              remaining_s_m, low_buffer_duration_s, trajectory_goal_.x,
              trajectory_goal_.y);
}

void Px4OffboardNode::updateTemporaryReplanHold() {
  if (!temporary_replan_truncation_active_ || temporary_replan_hold_active_ ||
      !finalPathGoalReached()) {
    return;
  }

  temporary_replan_hold_active_ = true;
  temporary_replan_hold_target_ = trajectory_goal_;
  latchTerminalPositionCaptureAltitude("temporary_replan_hold");
  no_path_hold_target_valid_ = false;
  resetVelocityDiagnostics();
  RCLCPP_WARN(
      get_logger(),
      "SAFE_TRAJECTORY_TRUNCATION temporary_hold_latched=true target=(%.2f, %.2f) "
      "current=(%.2f, %.2f) distance=%.2f speed=%.2f blocked_path_id=%" PRIu64,
      temporary_replan_hold_target_.x, temporary_replan_hold_target_.y,
      current_position_.x, current_position_.y,
      distance(current_position_, temporary_replan_hold_target_), current_speed_mps_,
      accepted_planner_path_id_);
}

void Px4OffboardNode::onReplanBlocker(const msg::ReplanBlockerEvent& msg) {
  pending_stationary_rollout_successor_.reset();
  pending_raw_obstacle_snapshot_.reset();
  pending_raw_obstacle_snapshot_deadline_.complete();
  if (crashed_ || !safe_trajectory_truncation_enabled_) {
    return;
  }
  if (msg.truncation_generation == 0U) {
    RCLCPP_ERROR(get_logger(),
                 "SAFE_TRAJECTORY_TRUNCATION ignored blocker event: "
                 "reason=invalid_generation blocked_path_id=%" PRIu64,
                 msg.blocked_path_id);
    return;
  }
  if (!accepted_planner_path_id_seen_ ||
      msg.blocked_path_id != accepted_planner_path_id_) {
    RCLCPP_WARN(
        get_logger(),
        "SAFE_TRAJECTORY_TRUNCATION ignored blocker event: reason=path_id_mismatch "
        "blocked_path_id=%" PRIu64 " accepted_path_id=%" PRIu64 " accepted_seen=%s",
        msg.blocked_path_id, accepted_planner_path_id_,
        accepted_planner_path_id_seen_ ? "true" : "false");
    return;
  }
  if (temporary_replan_truncation_active_) {
    RCLCPP_INFO(get_logger(),
                "SAFE_TRAJECTORY_TRUNCATION ignored duplicate blocker event: "
                "blocked_path_id=%" PRIu64 " temporary_hold=%s",
                msg.blocked_path_id, temporary_replan_hold_active_ ? "true" : "false");
    return;
  }
  if (!localPositionFresh() || !trajectorySamplesAreUsable(final_trajectory_samples_)) {
    RCLCPP_WARN(get_logger(),
                "SAFE_TRAJECTORY_TRUNCATION ignored blocker event: "
                "reason=trajectory_or_pose_unavailable blocked_path_id=%" PRIu64
                " pose_fresh=%s trajectory_valid=%s",
                msg.blocked_path_id, localPositionFresh() ? "true" : "false",
                trajectorySamplesAreUsable(final_trajectory_samples_) ? "true"
                                                                      : "false");
    return;
  }

  const std::optional<OccupancyGrid2D> raw_occupancy = currentRawObstacleGrid();
  const RawObstacleSnapshotMetadata* raw_metadata =
      raw_obstacle_snapshot_tracker_.current();
  const double critical_band_width_m =
      raw_metadata != nullptr ? raw_metadata->policy.critical_distance_m : 1.0;
  const SafeTrajectoryTruncationResult truncation = truncateTrajectoryBeforeBlocker(
      final_trajectory_samples_,
      SafeTrajectoryTruncationRequest{
          .current_position = current_position_,
          .blocker_path_distance_m = msg.blocker_path_distance_m,
          .truncation_margin_m = safe_trajectory_truncation_margin_m_,
          .raw_occupancy = raw_occupancy.has_value() ? &*raw_occupancy : nullptr,
          .critical_band_width_m = critical_band_width_m,
          .terminal_clearance_beyond_critical_m =
              safe_trajectory_terminal_prohibited_clearance_m_,
      });
  if (!truncation.applied) {
    RCLCPP_ERROR(get_logger(),
                 "SAFE_TRAJECTORY_TRUNCATION rejected blocker event: reason=%s "
                 "blocked_path_id=%" PRIu64
                 " blocker_distance=%.2fm margin=%.2fm raw_snapshot=%s"
                 " terminal_clearance_beyond_critical_m=%.2fm "
                 "required_raw_clearance_m=%.2fm",
                 truncation.reason, msg.blocked_path_id, msg.blocker_path_distance_m,
                 safe_trajectory_truncation_margin_m_,
                 raw_occupancy.has_value() ? "available" : "unavailable",
                 safe_trajectory_terminal_prohibited_clearance_m_,
                 critical_band_width_m +
                     safe_trajectory_terminal_prohibited_clearance_m_);
    return;
  }

  temporary_replan_truncation_active_ = true;
  temporary_replan_hold_active_ = false;
  temporary_replan_immediate_hold_ = truncation.immediate_hold;
  active_truncation_generation_ = msg.truncation_generation;
  pending_truncation_suffix_.reset();
  resetVerticalPreAlignment();
  no_path_hold_target_valid_ = false;
  if (truncation.immediate_hold) {
    temporary_replan_hold_active_ = true;
    temporary_replan_hold_target_ = current_position_;
    latchTerminalPositionCaptureAltitude("temporary_replan_hold_immediate");
    resetVelocityDiagnostics();
    TrajectoryPointSample hold_sample;
    hold_sample.point = current_position_;
    hold_sample.tangent =
        Point2{std::cos(current_heading_rad_), std::sin(current_heading_rad_)};
    hold_sample.z_m = std::isfinite(current_altitude_m_)
                          ? current_altitude_m_
                          : final_trajectory_samples_.front().z_m;
    active_truncation_terminal_sample_ = hold_sample;
    const std::array<TrajectoryPointSample, 1U> hold_samples{hold_sample};
    active_temporary_prefix_fingerprint_ = trajectoryPrefixFingerprint(hold_samples);
    if (active_temporary_prefix_fingerprint_ == 0U) {
      active_temporary_prefix_fingerprint_ = 1U;
    }
    publishReplanTruncation(msg, hold_sample, active_temporary_prefix_fingerprint_,
                            true);
    RCLCPP_WARN(
        get_logger(),
        "SAFE_TRAJECTORY_TRUNCATION immediate_hold=true blocked_path_id=%" PRIu64
        " current_s=%.2f blocker_s=%.2f nominal_stop_s=%.2f stop_s=%.2f "
        "prohibited_clearance=%.2fm clearance_adjusted=%s blocker=(%.2f, %.2f) "
        "source='%s'",
        msg.blocked_path_id, truncation.current_s_m, truncation.blocker_s_m,
        truncation.nominal_stop_s_m, truncation.stop_s_m,
        truncation.terminal_raw_clearance_m,
        truncation.clearance_adjusted ? "true" : "false", msg.blocker_position.x,
        msg.blocker_position.y, msg.source.c_str());
    return;
  }

  OffboardTrajectoryState state =
      buildOffboardTrajectoryState(truncation.samples, velocity_follower_config_);
  if (!state.valid) {
    temporary_replan_truncation_active_ = false;
    active_truncation_generation_ = 0U;
    active_temporary_prefix_fingerprint_ = 0U;
    RCLCPP_ERROR(get_logger(),
                 "SAFE_TRAJECTORY_TRUNCATION rejected generated prefix: "
                 "reason=invalid_prefix blocked_path_id=%" PRIu64,
                 msg.blocked_path_id);
    return;
  }

  path_points_.clear();
  path_points_.reserve(state.samples.size());
  for (const TrajectoryPointSample& sample : state.samples) {
    path_points_.push_back(sample.point);
  }
  path_valid_ = true;
  trajectory_goal_ = path_points_.back();
  trajectory_goal_valid_ = true;
  waypoint_index_ = 0U;
  const TrajectoryContinuityResult continuity{
      .decision = TrajectoryContinuityDecision::kResetSmoother,
      .reason = "safe_trajectory_truncation",
  };
  applyReceivedFinalTrajectoryPath("safe_trajectory_truncation", state, continuity);
  active_temporary_prefix_fingerprint_ =
      trajectoryPrefixFingerprint(final_trajectory_samples_);
  if (active_temporary_prefix_fingerprint_ == 0U) {
    active_temporary_prefix_fingerprint_ = 1U;
  }
  active_truncation_terminal_sample_ = final_trajectory_samples_.back();
  publishReplanTruncation(msg, final_trajectory_samples_.back(),
                          active_temporary_prefix_fingerprint_, false);
  RCLCPP_WARN(
      get_logger(),
      "SAFE_TRAJECTORY_TRUNCATION prefix_active=true blocked_path_id=%" PRIu64
      " blocker_distance=%.2fm margin=%.2fm current_s=%.2f blocker_s=%.2f "
      "nominal_stop_s=%.2f stop_s=%.2f prohibited_clearance=%.2fm "
      "required_prohibited_clearance=%.2fm clearance_adjusted=%s prefix_length=%.2fm "
      "target=(%.2f, %.2f) blocker=(%.2f, %.2f) "
      "memory_sequence=%" PRIu64 " source='%s'",
      msg.blocked_path_id, msg.blocker_path_distance_m,
      safe_trajectory_truncation_margin_m_, truncation.current_s_m,
      truncation.blocker_s_m, truncation.nominal_stop_s_m, truncation.stop_s_m,
      truncation.terminal_raw_clearance_m,
      safe_trajectory_terminal_prohibited_clearance_m_,
      truncation.clearance_adjusted ? "true" : "false", state.samples.back().s_m,
      trajectory_goal_.x, trajectory_goal_.y, msg.blocker_position.x,
      msg.blocker_position.y, msg.memory_snapshot_sequence, msg.source.c_str());
}

} // namespace drone_city_nav
