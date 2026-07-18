#include "px4_offboard_node.hpp"

namespace drone_city_nav {

void Px4OffboardNode::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
  if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
    if (local_position_valid_) {
      last_local_position_update_ns_ = 0;
    }
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid PX4 local position and holding the last known target: "
        "xy_valid=%s x=%.2f y=%.2f had_previous_position=%s",
        msg.xy_valid ? "true" : "false", static_cast<double>(msg.x),
        static_cast<double>(msg.y), local_position_valid_ ? "true" : "false");
    return;
  }

  last_local_position_update_ns_ = get_clock()->now().nanoseconds();
  current_position_ = Point2{static_cast<double>(msg.x) + px4_local_origin_.x,
                             static_cast<double>(msg.y) + px4_local_origin_.y};
  if (std::isfinite(msg.vx) && std::isfinite(msg.vy)) {
    current_velocity_ =
        Point2{static_cast<double>(msg.vx), static_cast<double>(msg.vy)};
    current_speed_mps_ = std::hypot(current_velocity_.x, current_velocity_.y);
    current_velocity_valid_ = true;
  } else {
    current_velocity_ = Point2{};
    current_speed_mps_ = std::numeric_limits<double>::quiet_NaN();
    current_velocity_valid_ = false;
  }
  if (msg.z_valid && std::isfinite(msg.z)) {
    current_altitude_m_ = -static_cast<double>(msg.z);
    altitude_valid_ = true;
    updateNavigationStartState();
  }
  if (std::isfinite(msg.heading)) {
    current_heading_rad_ = static_cast<double>(msg.heading);
  }
  local_position_valid_ = true;
  if (!takeoff_hold_target_valid_) {
    takeoff_hold_target_ = current_position_;
    takeoff_hold_target_valid_ = true;
  }

  if (!local_position_seen_) {
    local_position_seen_ = true;
    RCLCPP_INFO(get_logger(),
                "First valid PX4 local position: x=%.2f y=%.2f "
                "altitude=%.2f heading=%.2f",
                current_position_.x, current_position_.y, current_altitude_m_,
                current_heading_rad_);
  }
}

void Px4OffboardNode::onAttitude(const px4_msgs::msg::VehicleAttitude& msg) {
  last_attitude_update_ns_ = get_clock()->now().nanoseconds();
  const auto euler = quaternionToEuler(msg.q);
  if (!euler.has_value()) {
    attitude_valid_ = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Ignoring invalid PX4 attitude quaternion");
    return;
  }

  current_attitude_ = *euler;
  attitude_valid_ = true;
}

void Px4OffboardNode::onVehicleStatus(const px4_msgs::msg::VehicleStatus& msg) {
  vehicle_status_ = msg;
  vehicle_status_valid_ = true;

  const auto arming_state = static_cast<int>(msg.arming_state);
  const auto nav_state = static_cast<int>(msg.nav_state);
  if (arming_state != last_logged_arming_state_ ||
      nav_state != last_logged_nav_state_) {
    RCLCPP_INFO(get_logger(), "PX4 vehicle status: arming_state=%d nav_state=%d",
                arming_state, nav_state);
    last_logged_arming_state_ = arming_state;
    last_logged_nav_state_ = nav_state;
  }
}

void Px4OffboardNode::onProhibitedGrid(const nav_msgs::msg::OccupancyGrid& msg) {
  if (!(msg.info.resolution > 0.0F) || msg.info.width == 0U || msg.info.height == 0U ||
      msg.info.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      msg.info.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Ignoring invalid offboard prohibited grid metadata");
    return;
  }

  const std::size_t expected_size = static_cast<std::size_t>(msg.info.width) *
                                    static_cast<std::size_t>(msg.info.height);
  if (msg.data.size() != expected_size) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring offboard prohibited grid with mismatched data size: expected=%zu "
        "got=%zu",
        expected_size, msg.data.size());
    return;
  }

  prohibited_grid_ = msg;
  prohibited_grid_valid_ = true;
  last_prohibited_grid_update_ns_ = get_clock()->now().nanoseconds();
  if (!prohibited_grid_seen_logged_) {
    prohibited_grid_seen_logged_ = true;
    RCLCPP_INFO(get_logger(),
                "First offboard prohibited grid: size=%ux%u resolution=%.2f "
                "origin=(%.2f, %.2f)",
                msg.info.width, msg.info.height,
                static_cast<double>(msg.info.resolution), msg.info.origin.position.x,
                msg.info.origin.position.y);
  }
}

} // namespace drone_city_nav
