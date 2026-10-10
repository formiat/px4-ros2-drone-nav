#include "mppi_offboard_node.hpp"

namespace drone_city_nav {

void MppiOffboardNode::onHorizon(const msg::MppiTrajectoryHorizon& horizon) {
  if (horizon.target_offboard_instance_id != offboard_producer_instance_id_) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "EXECUTION_HORIZON rejected producer=%" PRIu64 " sequence=%" PRIu64
        " target_offboard=%" PRIu64 " current_offboard=%" PRIu64
        " reason=non_current_offboard_session",
        horizon.producer_instance_id, horizon.sequence,
        horizon.target_offboard_instance_id, offboard_producer_instance_id_);
    return;
  }

  const ExecutionHorizonAdmissionCandidate candidate{
      .producer_instance_id = horizon.producer_instance_id,
      .sequence = horizon.sequence,
      .source_stamp_ns = executionHorizonTimeNanoseconds(horizon.header.stamp),
      .valid_from_ns = executionHorizonTimeNanoseconds(horizon.valid_from),
      .content_fingerprint = executionHorizonContentFingerprint(horizon),
  };
  const std::uint64_t previous_sequence = horizon_admission_.current_sequence;
  const ExecutionHorizonPayloadStatus payload_status = assessExecutionHorizonPayload(
      horizon, ExecutionHorizonPayloadValidationConfig{
                   .expected_frame_id = "map",
                   .flight_envelope = &flight_envelope_config_,
               });
  const bool revoked =
      horizon.execution_mode == msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED;
  const bool expired =
      !revoked &&
      now().nanoseconds() >= executionHorizonTimeNanoseconds(horizon.valid_until);
  const bool rearm_blocked = execution_horizon_rearm_required_ && !revoked;
  const bool payload_admissible =
      payload_status == ExecutionHorizonPayloadStatus::kValid && !expired &&
      !rearm_blocked;
  const ExecutionHorizonAdmissionResult admission =
      admitExecutionHorizonIdentity(horizon_admission_, candidate, payload_admissible);
  if (admission.state_advanced) {
    horizon_admission_ = admission.next_state;
  }
  if (admission.revoke) {
    // A newer current-producer identity or an ambiguity is authoritative even
    // when its payload is unusable. A rejected prospective producer does not
    // receive authority and therefore cannot clear the resident horizon.
    horizon_.reset();
  }
  if (admission.replay) {
    // An exact replay proves no new lease or evidence. A rejected exact
    // identity remains tombstoned and cannot be repaired in place.
    return;
  }
  if (!admission.accept_identity &&
      (!candidate.valid() || payload_admissible || admission.stale ||
       admission.conflict || admission.retired_capacity_exhausted ||
       admission.prospective_capacity_exhausted)) {
    const char* const reason = admission.conflict ? "identity_content_conflict"
                               : admission.stale  ? "stale_identity"
                               : admission.retired_capacity_exhausted
                                   ? "retired_identity_capacity_exhausted"
                               : admission.prospective_capacity_exhausted
                                   ? "prospective_identity_capacity_exhausted"
                                   : "invalid_identity";
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "EXECUTION_HORIZON rejected producer=%" PRIu64
                         " sequence=%" PRIu64 " current_producer=%" PRIu64
                         " current_sequence=%" PRIu64 " reason=%s",
                         horizon.producer_instance_id, horizon.sequence,
                         horizon_admission_.current_producer_instance_id,
                         horizon_admission_.current_sequence, reason);
    return;
  }
  if (payload_status != ExecutionHorizonPayloadStatus::kValid || expired ||
      rearm_blocked) {
    const std::string_view rejection_reason =
        payload_status != ExecutionHorizonPayloadStatus::kValid
            ? executionHorizonPayloadStatusName(payload_status)
        : expired ? std::string_view{"expired_validity_window"}
                  : std::string_view{"vehicle_disarmed_require_new_identity"};
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "EXECUTION_HORIZON rejected sequence=%" PRIu64
                         " previous=%" PRIu64 " reason=%.*s",
                         horizon.sequence, previous_sequence,
                         static_cast<int>(rejection_reason.size()),
                         rejection_reason.data());
    return;
  }
  if (!admission.payload_installable) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "EXECUTION_HORIZON rejected producer=%" PRIu64
                         " sequence=%" PRIu64
                         " reason=non_advancing_timestamp_identity_tombstoned",
                         horizon.producer_instance_id, horizon.sequence);
    return;
  }
  if (revoked) {
    horizon_.reset();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "EXECUTION_HORIZON revoked=true producer=%" PRIu64
                         " sequence=%" PRIu64
                         " reason=%s action=local_non_authoritative_hold",
                         horizon.producer_instance_id, horizon.sequence,
                         executionReasonName(horizon.execution_reason));
    return;
  }
  const bool execution_changed = !horizon_.has_value() ||
                                 horizon_->execution_mode != horizon.execution_mode ||
                                 horizon_->execution_reason != horizon.execution_reason;
  horizon_ = horizon;
  unavailable_path_hold_pin_.release();
  RCLCPP_INFO(get_logger(),
              "EXECUTION_HORIZON accepted=true producer=%" PRIu64 " sequence=%" PRIu64
              " mode=%s",
              horizon.producer_instance_id, horizon.sequence,
              executionModeName(horizon.execution_mode));
  if (execution_changed) {
    RCLCPP_INFO(get_logger(),
                "EXECUTION_HORIZON mode=%s reason=%s sequence=%" PRIu64 " hold=%s"
                " target=(%.3f,%.3f,%.3f)",
                executionModeName(horizon.execution_mode),
                executionReasonName(horizon.execution_reason), horizon.sequence,
                horizon.stationary_position_hold ? "true" : "false",
                horizon.stationary_hold_position.x, horizon.stationary_hold_position.y,
                horizon.stationary_hold_position.z);
  }
}

bool MppiOffboardNode::horizonFresh() const {
  if (!horizon_.has_value()) {
    return false;
  }
  const std::int64_t now_ns = now().nanoseconds();
  return now_ns >= executionHorizonTimeNanoseconds(horizon_->valid_from) &&
         now_ns < executionHorizonTimeNanoseconds(horizon_->valid_until);
}

bool MppiOffboardNode::stationaryPositionHoldActive() const noexcept {
  return horizon_.has_value() && horizon_->stationary_position_hold && horizonFresh();
}

bool MppiOffboardNode::plannedFinitePathFresh() const {
  return horizon_.has_value() &&
         horizon_->execution_mode ==
             msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED &&
         horizonFresh();
}

bool MppiOffboardNode::plannedFinitePathCompleted() const {
  return horizon_.has_value() &&
         horizon_->execution_mode ==
             msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED &&
         now().nanoseconds() >= executionHorizonTimeNanoseconds(horizon_->valid_until);
}

// Level, holding no position: a position held on a dead-reckoned estimate
// tipped the landed vehicle over (r881). At 0.7 m/s one touched at 1.04.
void MppiOffboardNode::publishDeadReckoningLandingSetpoint() {
  constexpr double kLandingSpeedMps{0.5};
  constexpr double nan{std::numeric_limits<double>::quiet_NaN()};
  setpoint_pub_->publish(buildMppiTrajectorySetpoint(nowMicros(), Point2{nan, nan},
                                                     -kLandingSpeedMps,
                                                     Point2{0.0, 0.0}, nan, nan, 0.0));
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                       "DEAD_RECKONING_LANDING descending=true");
}

void MppiOffboardNode::publishTakeoffSetpoint() {
  setpoint_pub_->publish(buildPositionTrajectorySetpoint(
      nowMicros(), Point2{local_x_, local_y_},
      takeoffAltitudeM() - px4_map_transform_.map_origin.z,
      px4_map_transform_.mapYawToPx4Heading(heading_rad_)));
}

bool MppiOffboardNode::publishPrestartPlannedHorizonReceipt() {
  if (!plannedFinitePathFresh()) {
    return false;
  }
  // The takeoff/hover setpoint was emitted immediately before this receipt.
  // It proves delivery of the exact planned identity for cooperative startup,
  // but remains non-authoritative until mission-start permits trajectory use.
  publishAppliedControlFeedback(Point2{}, 0.0, 0.0, 0.0, false,
                                msg::MppiControlFeedback::EXECUTION_MODE_PLANNED);
  return true;
}

bool MppiOffboardNode::publishStationaryPositionHoldSetpoint() {
  if (!horizon_.has_value()) {
    return false;
  }
  const geometry_msgs::msg::Point& target = horizon_.value().stationary_hold_position;
  const Point2 local_target =
      px4_map_transform_.mapPositionToLocal(Point2{target.x, target.y});
  setpoint_pub_->publish(buildPositionTrajectorySetpoint(
      nowMicros(), local_target, target.z - px4_map_transform_.map_origin.z,
      px4_map_transform_.mapYawToPx4Heading(heading_rad_)));
  // This exact non-authoritative witness is valid only because the matching
  // certified stationary setpoint was emitted immediately above.
  publishAppliedControlFeedback(Point2{}, 0.0, 0.0, 0.0, false,
                                msg::MppiControlFeedback::EXECUTION_MODE_POSITION_HOLD);
  return true;
}

void MppiOffboardNode::publishCompletedFinitePathHoldSetpoint() {
  if (!horizon_.has_value() || horizon_->points.empty()) {
    return;
  }
  const msg::MppiHorizonPoint& terminal = horizon_->points.back();
  const Point2 local_target = px4_map_transform_.mapPositionToLocal(
      Point2{terminal.position.x, terminal.position.y});
  setpoint_pub_->publish(buildPositionTrajectorySetpoint(
      nowMicros(), local_target, terminal.position.z - px4_map_transform_.map_origin.z,
      px4_map_transform_.mapYawToPx4Heading(terminal.yaw_rad)));
  const Point2 map_position =
      px4_map_transform_.localPositionToMap(Point2{local_x_, local_y_});
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                       "FINITE_EXECUTION_PATH terminal_hold=true sequence=%" PRIu64
                       " target=(%.3f,%.3f,%.3f) current=(%.3f,%.3f,%.3f) speed=%.3f",
                       horizon_admission_.current_sequence, terminal.position.x,
                       terminal.position.y, terminal.position.z, map_position.x,
                       map_position.y, mapAltitudeM(), currentSpeedMps());
}

bool MppiOffboardNode::publishHorizonSetpoint() {
  if (!horizon_.has_value() || !horizonFresh()) {
    return false;
  }
  const msg::MppiTrajectoryHorizon& horizon = horizon_.value();
  const std::int64_t elapsed_ns = executionHorizonTimeAheadNs(
      now().nanoseconds() - executionHorizonTimeNanoseconds(horizon.valid_from),
      control_lookahead_ns_);
  const std::optional<ExecutionHorizonBracket> bracket = executionHorizonBracketAt(
      horizon.points.size(), horizon.control_interval_ns, elapsed_ns);
  if (!bracket || !bracket->valid()) {
    return false;
  }
  const auto& first = horizon.points[bracket->lower_index];
  const auto& second = horizon.points[bracket->upper_index];
  const double ratio = bracket->ratio();
  const Point2 map_position{interpolate(first.position.x, second.position.x, ratio),
                            interpolate(first.position.y, second.position.y, ratio)};
  const Point2 local_position = px4_map_transform_.mapPositionToLocal(map_position);
  const double altitude = interpolate(first.position.z, second.position.z, ratio) -
                          px4_map_transform_.map_origin.z;
  const Point2 map_velocity{interpolate(first.velocity.x, second.velocity.x, ratio),
                            interpolate(first.velocity.y, second.velocity.y, ratio)};
  const Point2 velocity = px4_map_transform_.mapVectorToLocal(map_velocity);
  const double vertical_velocity =
      interpolate(first.velocity.z, second.velocity.z, ratio);
  const Point2 map_acceleration{
      interpolate(first.acceleration.x, second.acceleration.x, ratio),
      interpolate(first.acceleration.y, second.acceleration.y, ratio)};
  const Point2 acceleration = px4_map_transform_.mapVectorToLocal(map_acceleration);
  const double vertical_acceleration =
      interpolate(first.acceleration.z, second.acceleration.z, ratio);
  // No yaw: the gaze law closes the heading each tick, and the autopilot takes
  // the rate planned its yaw rate loop's lag ahead (r781 to r825).
  const ExecutionHorizonBracket rate =
      executionHorizonBracketAt(
          horizon.points.size(), horizon.control_interval_ns,
          executionHorizonTimeAheadNs(elapsed_ns, yaw_rate_lead_ns_))
          .value_or(*bracket);
  const double map_yaw_rate =
      interpolate(horizon.points[rate.lower_index].yaw_rate_radps,
                  horizon.points[rate.upper_index].yaw_rate_radps, rate.ratio());
  const double map_yaw_acceleration =
      interpolate(first.yaw_acceleration_radps2, second.yaw_acceleration_radps2, ratio);
  setpoint_pub_->publish(buildMppiPathTrajectorySetpoint(
      nowMicros(), local_position, altitude, velocity, vertical_velocity, acceleration,
      vertical_acceleration, std::numeric_limits<double>::quiet_NaN(),
      px4_map_transform_.mapYawRateToPx4(map_yaw_rate)));
  publishAppliedControlFeedback(map_acceleration, vertical_acceleration, map_yaw_rate,
                                map_yaw_acceleration, true,
                                msg::MppiControlFeedback::EXECUTION_MODE_PLANNED);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "OFFBOARD_PLANNED_HORIZON_APPLIED producer=%" PRIu64
                       " sequence=%" PRIu64 " %s",
                       horizon_admission_.current_producer_instance_id,
                       horizon_admission_.current_sequence,
                       transportLatencyFields(horizon_delivery_ms_).c_str());
  return true;
}

void MppiOffboardNode::publishUnavailablePathHoldSetpoint() {
  // An expired stationary hold goes on at its own position. A pin where the
  // estimate stands when the lease lapses moves the hold with every
  // excursion: r244 re-anchored 0.25 m nearer a wall and stayed there.
  const bool expired_stationary_hold =
      horizon_.has_value() && horizon_->stationary_position_hold;
  const Point2 hold_local = expired_stationary_hold
                                ? px4_map_transform_.mapPositionToLocal(
                                      Point2{horizon_->stationary_hold_position.x,
                                             horizon_->stationary_hold_position.y})
                                : Point2{local_x_, local_y_};
  const double hold_altitude_m =
      expired_stationary_hold
          ? horizon_->stationary_hold_position.z - px4_map_transform_.map_origin.z
          : altitude_m_;
  const bool pinned_now = unavailable_path_hold_pin_.acquire(
      Point3{hold_local.x, hold_local.y, hold_altitude_m});
  const Point3& target = *unavailable_path_hold_pin_.pin();
  if (pinned_now) {
    const Point2 map_target =
        px4_map_transform_.localPositionToMap(Point2{target.x, target.y});
    RCLCPP_WARN(get_logger(),
                "FINITE_EXECUTION_PATH unavailable=true "
                "authority=local_non_authoritative action=%s speed=%.2f "
                "target=(%.3f,%.3f,%.3f)",
                currentSpeedMps() > kStationaryExecutionHoldSpeedToleranceMps
                    ? "brake_then_position_hold"
                    : "position_hold",
                currentSpeedMps(), map_target.x, map_target.y,
                target.z + px4_map_transform_.map_origin.z);
  }
  const Point2 local_target{target.x, target.y};
  const double yaw = px4_map_transform_.mapYawToPx4Heading(heading_rad_);
  // While the vehicle still moves, the hold brakes it at the stack's own
  // deceleration instead of leaving the braking to the position loop's
  // response to a zero error; once it rests, the pinned position holds it.
  if (currentSpeedMps() > kStationaryExecutionHoldSpeedToleranceMps) {
    const Point2 local_velocity =
        px4_map_transform_.mapVectorToLocal(Point2{velocity_x_, velocity_y_});
    setpoint_pub_->publish(buildBrakingHoldTrajectorySetpoint(
        nowMicros(), local_target, target.z, local_velocity, velocity_up_mps_,
        unavailable_path_braking_acceleration_mps2_,
        unavailable_path_braking_response_s_, yaw));
    return;
  }
  setpoint_pub_->publish(
      buildPositionTrajectorySetpoint(nowMicros(), local_target, target.z, yaw));
}

} // namespace drone_city_nav
