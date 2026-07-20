#include <cmath>
#include <limits>

#include "px4_offboard_node.hpp"

namespace drone_city_nav {

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
