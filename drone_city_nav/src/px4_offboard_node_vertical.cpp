#include <cmath>
#include <limits>

#include "px4_offboard_node.hpp"

namespace drone_city_nav {

void Px4OffboardNode::resetVerticalPreAlignment() {
  vertical_pre_alignment_active_ = false;
  vertical_pre_alignment_captured_ = false;
  vertical_pre_alignment_target_z_m_ = std::numeric_limits<double>::quiet_NaN();
  vertical_pre_alignment_profile_ = VerticalCaptureProfileState{};
  vertical_pre_alignment_last_update_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
  vertical_pre_alignment_stable_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
}

void Px4OffboardNode::updateVerticalPreAlignment() {
  if (!vertical_pre_alignment_active_ || vertical_pre_alignment_captured_ ||
      !temporary_replan_hold_active_ || !localPositionFresh() || !altitude_valid_) {
    return;
  }

  const rclcpp::Time current_time = get_clock()->now();
  double dt_s = static_cast<double>(kControllerPeriod.count()) / 1000.0;
  if (vertical_pre_alignment_last_update_time_.nanoseconds() > 0 &&
      current_time > vertical_pre_alignment_last_update_time_) {
    dt_s =
        std::clamp((current_time - vertical_pre_alignment_last_update_time_).seconds(),
                   0.001, 0.2);
  }
  vertical_pre_alignment_last_update_time_ = current_time;

  const VerticalCaptureProfileStep step = advanceVerticalCaptureProfile(
      vertical_pre_alignment_profile_, current_altitude_m_,
      current_vertical_velocity_up_mps_, current_vertical_velocity_valid_,
      vertical_pre_alignment_target_z_m_, dt_s,
      VerticalCaptureProfileConfig{
          .max_climb_speed_mps = vertical_follower_config_.max_climb_speed_mps,
          .max_descent_speed_mps = vertical_follower_config_.max_descent_speed_mps,
          .max_accel_mps2 = vertical_follower_config_.max_vertical_accel_mps2,
          .max_jerk_mps3 = vertical_follower_config_.max_vertical_jerk_mps3,
      });
  if (!step.valid) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "REPLAN_TRUNCATION vertical pre-alignment profile invalid");
    return;
  }
  vertical_pre_alignment_profile_ = step.state;
  terminal_position_capture_altitude_m_ = step.state.commanded_z_m;
  terminal_position_capture_altitude_valid_ = true;

  const bool captured =
      distance(current_position_, temporary_replan_hold_target_) <= 1.0 &&
      current_velocity_valid_ && std::isfinite(current_speed_mps_) &&
      current_speed_mps_ <= 0.5 &&
      std::abs(current_altitude_m_ - vertical_pre_alignment_target_z_m_) <= 0.3 &&
      current_vertical_velocity_valid_ &&
      std::abs(current_vertical_velocity_up_mps_) <= 0.3;
  if (!captured) {
    vertical_pre_alignment_stable_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    return;
  }
  if (vertical_pre_alignment_stable_since_.nanoseconds() == 0) {
    vertical_pre_alignment_stable_since_ = current_time;
    return;
  }
  if ((current_time - vertical_pre_alignment_stable_since_).seconds() < 0.5) {
    return;
  }

  vertical_pre_alignment_captured_ = true;
  terminal_position_capture_altitude_m_ = vertical_pre_alignment_target_z_m_;
  RCLCPP_INFO(get_logger(),
              "REPLAN_TRUNCATION vertical_pre_alignment_captured=true "
              "target_z=%.2f actual_z=%.2f vertical_speed=%.2f stable_s=%.2f",
              vertical_pre_alignment_target_z_m_, current_altitude_m_,
              current_vertical_velocity_up_mps_,
              (current_time - vertical_pre_alignment_stable_since_).seconds());
}

void Px4OffboardNode::resetVelocityDiagnostics() {
  resetVelocitySmootherState("diagnostics_reset", false);
  last_velocity_plan_valid_ = false;
  last_velocity_plan_ = VelocitySetpointPlan{};
  last_velocity_plan_.reason = VelocitySetpointReason::kHold;
  last_vertical_plan_valid_ = false;
  last_vertical_plan_ = VerticalSetpointPlan{};
  last_velocity_setpoint_ = Point2{};
  last_velocity_setpoint_speed_mps_ = 0.0;
  last_vertical_velocity_setpoint_mps_ = 0.0;
  last_target_altitude_m_ = std::numeric_limits<double>::quiet_NaN();
  last_altitude_error_m_ = std::numeric_limits<double>::quiet_NaN();
  last_trajectory_altitude_target_valid_ = false;
  last_offboard_setpoint_mode_ = OffboardSetpointMode::kPositionHold;
  last_velocity_plan_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
}

void Px4OffboardNode::clearTerminalPositionCaptureAltitude() {
  terminal_position_capture_altitude_m_ = std::numeric_limits<double>::quiet_NaN();
  terminal_position_capture_altitude_valid_ = false;
}

void Px4OffboardNode::latchTerminalPositionCaptureAltitude(const char* reason) {
  if (terminal_position_capture_altitude_valid_) {
    return;
  }

  if (altitude_valid_ && std::isfinite(current_altitude_m_)) {
    terminal_position_capture_altitude_m_ = current_altitude_m_;
    terminal_position_capture_altitude_valid_ = true;
    RCLCPP_INFO(get_logger(),
                "Terminal position capture altitude latched: reason=%s "
                "altitude=%.2f",
                reason != nullptr ? reason : "unknown",
                terminal_position_capture_altitude_m_);
    return;
  }

  terminal_position_capture_altitude_m_ = initial_altitude_m_;
  terminal_position_capture_altitude_valid_ = false;
  RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Terminal position capture altitude is not available; using initial altitude "
      "as a temporary position-setpoint fallback: reason=%s initial_altitude=%.2f",
      reason != nullptr ? reason : "unknown", initial_altitude_m_);
}

double Px4OffboardNode::positionSetpointAltitudeM(
    const bool terminal_position_capture_requested) const {
  const bool terminal_altitude_mode = terminal_position_capture_requested ||
                                      final_goal_hold_active_ ||
                                      temporary_replan_hold_active_;
  if (terminal_altitude_mode && std::isfinite(terminal_position_capture_altitude_m_)) {
    return terminal_position_capture_altitude_m_;
  }
  return initial_altitude_m_;
}

bool Px4OffboardNode::positionSetpointAltitudeValid(
    const bool terminal_position_capture_requested) const {
  const bool terminal_altitude_mode = terminal_position_capture_requested ||
                                      final_goal_hold_active_ ||
                                      temporary_replan_hold_active_;
  return terminal_altitude_mode ? terminal_position_capture_altitude_valid_ : true;
}

VerticalSetpointPlan Px4OffboardNode::planVerticalSetpointForCurrentTrajectory(
    const VelocitySetpointPlan& velocity_plan, const double dt_s) const {
  const double scalar_speed_mps = std::isfinite(velocity_plan.accel_limited_speed_mps)
                                      ? velocity_plan.accel_limited_speed_mps
                                      : velocity_plan.final_command_speed_mps;
  return planVerticalSetpoint(final_trajectory_samples_, velocity_plan.trajectory_s_m,
                              scalar_speed_mps, current_altitude_m_, altitude_valid_,
                              dt_s, vertical_follower_state_,
                              vertical_follower_config_);
}

} // namespace drone_city_nav
