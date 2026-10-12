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
  // Specification K29: who flies the vehicle. The pilot's choice of mode and
  // the autopilot's failsafe win and are never undone here.
  const auto announce = [this](const std::string& event) {
    RCLCPP_WARN(get_logger(), "%s", event.c_str());
    publishMissionEvent(event);
  };
  if (in_control_seen_ && !flight_over_ &&
      (status.terminated || (was_armed && !armed))) {
    flight_over_ = true;
    released_ = false;
    announce(std::string{"AUTOPILOT_CONTROL state="} +
             (status.terminated ? "terminated" : "disarmed") +
             " mission_landing=" + (mission_landing_ ? "true" : "false"));
    return;
  }
  if (armed && status.external_control) {
    if (released_) {
      released_ = false;
      unavailable_path_hold_pin_.reset();
      announce("AUTOPILOT_CONTROL state=resumed");
    }
    in_control_seen_ = true;
  } else if (armed && in_control_seen_ && !released_ && !flight_over_) {
    released_ = true;
    unavailable_path_hold_pin_.reset();
    announce(std::string{"AUTOPILOT_CONTROL state=released reason="} +
             (status.failsafe            ? "failsafe"
              : status.autopilot_landing ? "autopilot_landing"
                                         : "pilot"));
  }
}

void MppiOffboardNode::onAutopilotLinks(const AutopilotLinks& links) {
  // A link is reported lost only once it was heard: a simulation without a
  // radio or a ground station never hears either.
  const auto follow = [this](const char* name, const bool lost, bool& heard,
                             bool& was_lost) {
    if (!lost) {
      if (heard && was_lost) {
        publishMissionEvent(std::string{name} + " lost=false");
        RCLCPP_WARN(get_logger(), "%s lost=false", name);
      }
      heard = true;
    } else if (heard && !was_lost) {
      publishMissionEvent(std::string{name} + " lost=true");
      RCLCPP_WARN(get_logger(), "%s lost=true", name);
    }
    was_lost = lost;
  };
  follow("RC_LINK", links.manual_control_lost, radio_heard_, radio_lost_);
  follow("GCS_LINK", links.ground_station_lost, ground_station_heard_,
         ground_station_lost_);
}

void MppiOffboardNode::onOperatorCommand(const std::string& text) {
  const std::optional<OperatorCommand> command = parseOperatorCommand(text);
  if (!command.has_value() || command->kind != OperatorCommandKind::kLand ||
      mission_landing_ || flight_over_ || !in_control_seen_ || !vehicle_status_.armed) {
    return;
  }
  mission_landing_ = true;
  mission_landing_local_ = Point2{local_x_, local_y_};
  mission_landing_yaw_rad_ = px4_map_transform_.mapYawToPx4Heading(heading_rad_);
  const Point2 map_xy = px4_map_transform_.localPositionToMap(mission_landing_local_);
  RCLCPP_WARN(get_logger(),
              "MISSION_LANDING started=true position=(%.2f,%.2f,%.2f) released=%s",
              map_xy.x, map_xy.y, mapAltitudeM(), released_ ? "true" : "false");
  publishMissionEvent("MISSION_LANDING started=true");
}

void MppiOffboardNode::publishMissionEvent(const std::string& event) {
  std_msgs::msg::String message;
  message.data = event;
  mission_event_pub_->publish(message);
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
  // A contact is a landing only while the vehicle is being landed (A9): the
  // blind descent (K19), the mission's landing (K30), or the autopilot's own.
  state.landing = blind_landing_ || mission_landing_ ||
                  (vehicle_status_seen_ && vehicle_status_.autopilot_landing);
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
