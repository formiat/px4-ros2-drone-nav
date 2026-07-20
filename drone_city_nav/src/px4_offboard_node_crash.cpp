#include "px4_offboard_node.hpp"

namespace drone_city_nav {

void Px4OffboardNode::onCrashState(const msg::CrashState& msg) {
  if (!msg.crashed || crashed_) {
    return;
  }

  crashed_ = true;
  crash_received_time_ = now();
  crash_drone_collision_ = msg.drone_collision;
  crash_obstacle_collision_ = msg.obstacle_collision;
  path_valid_ = false;
  trajectory_valid_ = false;
  trajectory_goal_valid_ = false;
  auto_arm_ = false;
  auto_offboard_ = false;
  last_force_disarm_command_time_ = now() - rclcpp::Duration::from_seconds(1.0);

  const rclcpp::Time contact_time{msg.stamp, RCL_ROS_TIME};
  const double delivery_latency_ms =
      contact_time.nanoseconds() > 0
          ? 1.0e-6 * static_cast<double>((now() - contact_time).nanoseconds())
          : std::numeric_limits<double>::quiet_NaN();
  RCLCPP_ERROR(
      get_logger(),
      "PHYSICAL_COLLISION offboard_crash_latched=true reason='%s' "
      "drone_collision='%s' obstacle_collision='%s' contact=(%.3f, %.3f, %.3f) "
      "altitude=%.2f speed=%.2f contact_to_offboard_ms=%.1f",
      msg.reason.c_str(), msg.drone_collision.c_str(), msg.obstacle_collision.c_str(),
      msg.contact_position.x, msg.contact_position.y, msg.contact_position.z,
      msg.altitude_m, msg.speed_mps, delivery_latency_ms);
}

void Px4OffboardNode::handleCrashedVehicle() {
  if (crash_disarm_confirmed_) {
    return;
  }

  const rclcpp::Time current_time = now();
  constexpr double kForceDisarmRetryPeriodS{0.2};
  if ((current_time - last_force_disarm_command_time_).seconds() <
      kForceDisarmRetryPeriodS) {
    return;
  }

  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        0.0F, 21196.0F);
  last_force_disarm_command_time_ = current_time;
  const double latency_ms =
      crash_received_time_.nanoseconds() > 0
          ? 1.0e-6 *
                static_cast<double>((current_time - crash_received_time_).nanoseconds())
          : std::numeric_limits<double>::quiet_NaN();
  RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "PHYSICAL_COLLISION force_disarm_sent=true armed=%s latency_ms=%.1f",
      isArmed() ? "true" : "false", latency_ms);
}

} // namespace drone_city_nav
