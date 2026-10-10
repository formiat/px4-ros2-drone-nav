#include "mppi_offboard_node.hpp"

namespace drone_city_nav {

void MppiOffboardNode::onAutopilotStatus(const AutopilotStatus& status) {
  const bool was_armed = vehicle_status_seen_ && vehicle_status_.armed;
  const bool armed = status.armed;
  vehicle_status_ = status;
  vehicle_status_seen_ = true;
  if (!armed && (was_armed || horizon_.has_value())) {
    execution_horizon_rearm_required_ = true;
    horizon_.reset();
    unavailable_path_hold_pin_.reset();
    if (horizon_admission_.current_producer_instance_id != 0U) {
      static_cast<void>(tombstoneExecutionHorizonIdentity(
          horizon_admission_,
          ExecutionHorizonAdmissionCandidate{
              .producer_instance_id = horizon_admission_.current_producer_instance_id,
              .sequence = horizon_admission_.current_sequence,
              .source_stamp_ns = horizon_admission_.latest_source_stamp_ns,
              .valid_from_ns = horizon_admission_.latest_valid_from_ns,
              .content_fingerprint = horizon_admission_.current_content_fingerprint,
          }));
    }
    RCLCPP_WARN(get_logger(), "EXECUTION_HORIZON cleared=true reason=vehicle_disarmed "
                              "action=require_new_identity_after_rearm");
  } else if (!was_armed && armed && execution_horizon_rearm_required_) {
    execution_horizon_rearm_required_ = false;
    RCLCPP_INFO(get_logger(), "EXECUTION_HORIZON rearm_observed=true "
                              "action=wait_for_new_identity");
  }
}

void MppiOffboardNode::onLocalState(const AutopilotLocalState& state) {
  if (!state.position_valid || !state.altitude_valid || !state.velocity_valid ||
      !state.vertical_velocity_valid) {
    position_valid_ = false;
    return;
  }
  // The setpoints go out in the autopilot's own frame, so the contract's
  // map position is read back through the transform this node commands in.
  const Point2 local_position =
      px4_map_transform_.mapPositionToLocal(Point2{state.position.x, state.position.y});
  local_x_ = local_position.x;
  local_y_ = local_position.y;
  altitude_m_ = state.position.z - px4_map_transform_.map_origin.z;
  velocity_x_ = state.velocity.x;
  velocity_y_ = state.velocity.y;
  velocity_up_mps_ = state.velocity.z;
  if (std::isfinite(state.yaw_rad)) {
    heading_rad_ = state.yaw_rad;
  }
  heading_valid_ = state.heading_valid && std::isfinite(state.yaw_rad);
  position_valid_ = true;
  publishNavigationState();
  publishRvizDrone();
}

void MppiOffboardNode::publishNavigationState() {
  if (!navigation_state_pub_) {
    return;
  }
  msg::VehicleNavigationState state;
  state.stamp = now();
  state.position =
      markerPoint(px4_map_transform_.localPositionToMap(Point2{local_x_, local_y_}),
                  mapAltitudeM());
  state.velocity.x = velocity_x_;
  state.velocity.y = velocity_y_;
  state.velocity.z = velocity_up_mps_;
  state.heading_rad = heading_rad_;
  state.position_valid = position_valid_;
  state.velocity_valid = position_valid_;
  state.heading_valid = heading_valid_;
  state.armed = vehicle_status_seen_ && vehicle_status_.armed;
  state.airborne = state.armed && altitude_m_ >= 1.0;
  state.navigation_ready =
      state.airborne && takeoff_complete_stamp_.has_value() &&
      (now() - *takeoff_complete_stamp_).seconds() >= takeoff_hover_s_;
  state.landing = blind_landing_;
  navigation_state_pub_->publish(state);
  publishNavigationReadiness(state.navigation_ready);
}

void MppiOffboardNode::publishNavigationReadiness(const bool ready) {
  if (last_navigation_readiness_.has_value() && *last_navigation_readiness_ == ready) {
    return;
  }
  std_msgs::msg::Bool readiness;
  readiness.data = ready;
  navigation_readiness_pub_->publish(readiness);
  last_navigation_readiness_ = ready;
  RCLCPP_INFO(get_logger(), "BOOTSTRAP_TAKEOFF_READINESS ready=%s",
              ready ? "true" : "false");
}

void MppiOffboardNode::publishRvizDrone() {
  if (!position_valid_) {
    return;
  }
  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = rviz_drone_follow_parent_frame_;
  const Point2 map_position =
      px4_map_transform_.localPositionToMap(Point2{local_x_, local_y_});
  const Point3 position = gazeboAlignedRvizFramePosition(
      Point3{map_position.x, map_position.y, mapAltitudeM()},
      gazebo_aligned_rviz_axes_swapped_);
  if (rviz_drone_follow_tf_broadcaster_) {
    rviz_drone_follow_tf_broadcaster_->sendTransform(
        droneFollowTransform(header, rviz_drone_follow_frame_, position));
  }
  if (rviz_drone_marker_pub_) {
    rviz_drone_marker_pub_->publish(
        droneMarker(header, rviz_drone_marker_id_, position,
                    rgba(static_cast<float>(rviz_drone_marker_color_r_),
                         static_cast<float>(rviz_drone_marker_color_g_),
                         static_cast<float>(rviz_drone_marker_color_b_), 1.0F)));
  }
}

double MppiOffboardNode::currentSpeedMps() const noexcept {
  return std::hypot(std::hypot(velocity_x_, velocity_y_), velocity_up_mps_);
}

void MppiOffboardNode::publishAppliedControlFeedback(
    const Point2 acceleration, const double vertical_acceleration,
    const double yaw_rate, const double yaw_acceleration,
    const bool control_authoritative, const std::uint8_t execution_mode) {
  if (!applied_control_feedback_pub_) {
    return;
  }
  msg::MppiControlFeedback feedback;
  feedback.header.stamp = now();
  feedback.header.frame_id = applied_control_feedback_frame_id_;
  feedback.producer_instance_id = offboard_producer_instance_id_;
  feedback.horizon_producer_instance_id =
      horizon_admission_.current_producer_instance_id;
  feedback.horizon_sequence = horizon_admission_.current_sequence;
  feedback.execution_mode = execution_mode;
  feedback.control_authoritative = control_authoritative;
  feedback.acceleration.x = acceleration.x;
  feedback.acceleration.y = acceleration.y;
  feedback.acceleration.z = vertical_acceleration;
  feedback.yaw_rate_radps = static_cast<float>(yaw_rate);
  feedback.yaw_acceleration_radps2 = static_cast<float>(yaw_acceleration);
  applied_control_feedback_pub_->publish(feedback);
}

void MppiOffboardNode::publishOffboardSessionHeartbeat() {
  if (!applied_control_feedback_pub_) {
    return;
  }
  msg::MppiControlFeedback heartbeat;
  heartbeat.header.stamp = now();
  heartbeat.header.frame_id = applied_control_feedback_frame_id_;
  heartbeat.producer_instance_id = offboard_producer_instance_id_;
  heartbeat.horizon_producer_instance_id = 0U;
  heartbeat.horizon_sequence = 0U;
  heartbeat.execution_mode = msg::MppiControlFeedback::EXECUTION_MODE_POSITION_HOLD;
  heartbeat.control_authoritative = false;
  applied_control_feedback_pub_->publish(heartbeat);
}

void MppiOffboardNode::publishUnavailableControlFeedback() {
  // Takeoff, destruction, expired/completed horizons, and local fallback
  // holds are not evidence that any planner horizon was applied.
  publishOffboardSessionHeartbeat();
}

double MppiOffboardNode::mapAltitudeM() const noexcept {
  return altitude_m_ + px4_map_transform_.map_origin.z;
}

double MppiOffboardNode::takeoffAltitudeM() const noexcept {
  return std::max(px4_map_transform_.map_origin.z + takeoff_climb_m_,
                  flight_envelope_config_.minimum_target_z_m +
                      kTakeoffCaptureToleranceM);
}

void MppiOffboardNode::publishCommand(const std::uint32_t command, const float param1,
                                      const float param2) {
  command_pub_->publish(
      buildVehicleCommand(nowMicros(), command, param1, param2, endpoint_));
}

std::uint64_t MppiOffboardNode::nowMicros() const {
  return static_cast<std::uint64_t>(std::max<std::int64_t>(0, now().nanoseconds()) /
                                    1000);
}

} // namespace drone_city_nav
