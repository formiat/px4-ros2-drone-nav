#include "px4_offboard_node.hpp"

namespace drone_city_nav {

void Px4OffboardNode::updateFinalGoalHold() {
  if (final_goal_hold_active_ || temporary_replan_truncation_active_ ||
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
  if (crashed_ || !safe_trajectory_truncation_enabled_) {
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

  const SafeTrajectoryTruncationResult truncation = truncateTrajectoryBeforeBlocker(
      final_trajectory_samples_,
      SafeTrajectoryTruncationRequest{
          .current_position = current_position_,
          .blocker_path_distance_m = msg.blocker_path_distance_m,
          .truncation_margin_m = safe_trajectory_truncation_margin_m_,
      });
  if (!truncation.applied) {
    RCLCPP_ERROR(get_logger(),
                 "SAFE_TRAJECTORY_TRUNCATION rejected blocker event: reason=%s "
                 "blocked_path_id=%" PRIu64 " blocker_distance=%.2fm margin=%.2fm",
                 truncation.reason, msg.blocked_path_id, msg.blocker_path_distance_m,
                 safe_trajectory_truncation_margin_m_);
    return;
  }

  temporary_replan_truncation_active_ = true;
  temporary_replan_hold_active_ = false;
  no_path_hold_target_valid_ = false;
  if (truncation.immediate_hold) {
    temporary_replan_hold_active_ = true;
    temporary_replan_hold_target_ = current_position_;
    latchTerminalPositionCaptureAltitude("temporary_replan_hold_immediate");
    resetVelocityDiagnostics();
    RCLCPP_WARN(
        get_logger(),
        "SAFE_TRAJECTORY_TRUNCATION immediate_hold=true blocked_path_id=%" PRIu64
        " current_s=%.2f blocker_s=%.2f stop_s=%.2f blocker=(%.2f, %.2f) "
        "source='%s'",
        msg.blocked_path_id, truncation.current_s_m, truncation.blocker_s_m,
        truncation.stop_s_m, msg.blocker_position.x, msg.blocker_position.y,
        msg.source.c_str());
    return;
  }

  OffboardTrajectoryState state =
      buildOffboardTrajectoryState(truncation.samples, velocity_follower_config_);
  if (!state.valid) {
    temporary_replan_truncation_active_ = false;
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
  RCLCPP_WARN(
      get_logger(),
      "SAFE_TRAJECTORY_TRUNCATION prefix_active=true blocked_path_id=%" PRIu64
      " blocker_distance=%.2fm margin=%.2fm current_s=%.2f blocker_s=%.2f "
      "stop_s=%.2f prefix_length=%.2fm target=(%.2f, %.2f) blocker=(%.2f, %.2f) "
      "memory_sequence=%" PRIu64 " source='%s'",
      msg.blocked_path_id, msg.blocker_path_distance_m,
      safe_trajectory_truncation_margin_m_, truncation.current_s_m,
      truncation.blocker_s_m, truncation.stop_s_m, state.samples.back().s_m,
      trajectory_goal_.x, trajectory_goal_.y, msg.blocker_position.x,
      msg.blocker_position.y, msg.memory_snapshot_sequence, msg.source.c_str());
}

} // namespace drone_city_nav
