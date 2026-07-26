#include "px4_offboard_node.hpp"

namespace drone_city_nav {

void Px4OffboardNode::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
  const auto callback_wall_time = std::chrono::steady_clock::now();
  if (last_local_position_callback_wall_time_.has_value()) {
    const double callback_gap_s =
        std::chrono::duration<double>(callback_wall_time -
                                      *last_local_position_callback_wall_time_)
            .count();
    const double max_pose_staleness_s =
        static_cast<double>(max_pose_staleness_ns_) / 1.0e9;
    if (max_pose_staleness_s > 0.0 && callback_gap_s > max_pose_staleness_s) {
      const double timer_callback_age_s =
          last_control_timer_callback_wall_time_.has_value()
              ? std::chrono::duration<double>(callback_wall_time -
                                              *last_control_timer_callback_wall_time_)
                    .count()
              : std::numeric_limits<double>::infinity();
      RCLCPP_WARN(get_logger(),
                  "PX4 local-position callback gap: wall_gap_s=%.3f "
                  "timer_callback_age_s=%.3f message_pose_age_s=%.3f "
                  "xy_valid=%s z_valid=%s",
                  callback_gap_s, timer_callback_age_s, localPositionAgeSeconds(),
                  msg.xy_valid ? "true" : "false", msg.z_valid ? "true" : "false");
    }
  }
  last_local_position_callback_wall_time_ = callback_wall_time;

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
  if (msg.v_z_valid && std::isfinite(msg.vz)) {
    current_vertical_velocity_up_mps_ = -static_cast<double>(msg.vz);
    current_vertical_velocity_valid_ = true;
  } else {
    current_vertical_velocity_up_mps_ = std::numeric_limits<double>::quiet_NaN();
    current_vertical_velocity_valid_ = false;
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

  if (crashed_ && !crash_disarm_confirmed_ &&
      msg.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
    crash_disarm_confirmed_ = true;
    const double latency_ms =
        crash_received_time_.nanoseconds() > 0
            ? 1.0e-6 * static_cast<double>((now() - crash_received_time_).nanoseconds())
            : std::numeric_limits<double>::quiet_NaN();
    RCLCPP_ERROR(get_logger(),
                 "PHYSICAL_COLLISION disarm_confirmed=true latency_ms=%.1f "
                 "drone_collision='%s' obstacle_collision='%s'",
                 latency_ms, crash_drone_collision_.c_str(),
                 crash_obstacle_collision_.c_str());
  }

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

void Px4OffboardNode::onRawObstacleSnapshot(const msg::RawObstacleSnapshot& snapshot) {
  const nav_msgs::msg::OccupancyGrid& grid = snapshot.grid;
  const bool raw_values_valid =
      std::ranges::all_of(grid.data, [](const std::int8_t value) {
        return value == -1 || value == 0 || value == kRawOccupiedValue;
      });
  const bool grid_valid =
      grid.info.resolution > 0.0F && grid.info.width != 0U && grid.info.height != 0U &&
      grid.info.width <= static_cast<std::uint32_t>(std::numeric_limits<int>::max()) &&
      grid.info.height <= static_cast<std::uint32_t>(std::numeric_limits<int>::max()) &&
      grid.data.size() == static_cast<std::size_t>(grid.info.width) *
                              static_cast<std::size_t>(grid.info.height) &&
      raw_values_valid;
  const RawObstacleSnapshotMetadata metadata{
      .identity =
          RawObstacleSnapshotIdentity{
              .producer_instance_id = snapshot.producer_instance_id,
              .revision = snapshot.obstacle_snapshot_revision,
              .policy_fingerprint = snapshot.risk_policy_fingerprint,
          },
      .policy =
          ObstacleRiskPolicy{
              .critical_distance_m = snapshot.risk_critical_distance_m,
              .preferred_distance_m = snapshot.risk_preferred_distance_m,
          },
      .grid_valid = grid_valid,
  };
  if (!raw_obstacle_snapshot_tracker_.accept(metadata)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Ignoring malformed or out-of-order raw obstacle snapshot "
                         "producer=%" PRIu64 " revision=%" PRIu64,
                         snapshot.producer_instance_id,
                         snapshot.obstacle_snapshot_revision);
    return;
  }

  raw_obstacle_grid_ = grid;
  raw_obstacle_grid_valid_ = true;
  last_raw_obstacle_grid_update_ns_ = get_clock()->now().nanoseconds();
  if (!raw_obstacle_grid_seen_logged_) {
    raw_obstacle_grid_seen_logged_ = true;
    RCLCPP_INFO(get_logger(),
                "First raw obstacle snapshot: producer=%" PRIu64 " revision=%" PRIu64
                " policy=%" PRIu64 " size=%ux%u resolution=%.2f origin=(%.2f, %.2f)",
                snapshot.producer_instance_id, snapshot.obstacle_snapshot_revision,
                snapshot.risk_policy_fingerprint, grid.info.width, grid.info.height,
                static_cast<double>(grid.info.resolution), grid.info.origin.position.x,
                grid.info.origin.position.y);
  }
  if (pending_raw_obstacle_snapshot_.has_value()) {
    msg::ExecutableTrajectory command = std::move(*pending_raw_obstacle_snapshot_);
    pending_raw_obstacle_snapshot_.reset();
    pending_raw_obstacle_snapshot_received_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    processExecutableTrajectory(command, true);
  }
}

} // namespace drone_city_nav
