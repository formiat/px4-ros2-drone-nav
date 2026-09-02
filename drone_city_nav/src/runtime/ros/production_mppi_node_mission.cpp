#include <cinttypes>
#include <limits>
#include <memory>
#include <optional>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] builtin_interfaces::msg::Time
timeFromNanoseconds(const std::int64_t nanoseconds) noexcept {
  builtin_interfaces::msg::Time time;
  if (nanoseconds <= 0) {
    return time;
  }
  time.sec = static_cast<std::int32_t>(nanoseconds / 1'000'000'000LL);
  time.nanosec = static_cast<std::uint32_t>(nanoseconds % 1'000'000'000LL);
  return time;
}

} // namespace

MissionWaypointUpdate ProductionMppiNode::updateMissionWaypoint(
    const std::shared_ptr<const ProductionNavigationObjectiveState>& objective_state,
    const ProductionMppiNavigation& navigation,
    const ProductionMppiVehicleStatus& vehicle_status,
    const std::shared_ptr<const CommittedExecutionAuthority3D>& execution_authority,
    const std::uint64_t applied_control_discontinuity_generation,
    const bool applied_control_discontinuity_generation_valid,
    const bool vehicle_status_epoch_stable, const bool goal_capture_latched,
    const std::int64_t now_ns) {
  mission_goal_capture_attempt_invalidated_ = false;
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      objective_state != nullptr ? objective_state->objective : nullptr;
  if (!mission_waypoint_sequence_ || !mission_waypoint_capture_gate_ || !objective ||
      objective->tracking.has_value() || objective->immediate_hold ||
      !mission_waypoint_acknowledgement_pub_ || execution_authority == nullptr ||
      !execution_authority->valid()) {
    if (mission_waypoint_capture_gate_) {
      mission_waypoint_capture_gate_->reset();
    }
    return {};
  }
  const AppliedControlEvidence3D& applied_control = execution_authority->control();
  const ExecutionOwnerIdentity3D& execution_horizon_owner =
      execution_authority->owner();
  const std::shared_ptr<const ExecutionPlan3D> authority_plan =
      execution_authority->plan();
  const std::uint64_t hold_id =
      authority_plan != nullptr && authority_plan->stationaryHold() != nullptr
          ? authority_plan->stationaryHold()->hold_id
          : 0U;
  const std::uint64_t previous_horizon_sequence_same_hold =
      hold_id != 0U && last_capture_hold_id_ == hold_id &&
              last_capture_hold_sequence_ != execution_horizon_owner.sequence
          ? last_capture_hold_sequence_
          : 0U;
  last_capture_hold_id_ = hold_id;
  last_capture_hold_sequence_ = execution_horizon_owner.sequence;

  const MissionWaypointCaptureGateResult capture =
      mission_waypoint_capture_gate_->update(MissionWaypointCaptureObservation{
          .stamp_ns = now_ns,
          .goal = mission_waypoint_sequence_->activeGoal(),
          .position =
              Point3{navigation.state.x, navigation.state.y, navigation.state.z},
          .velocity =
              Point3{navigation.state.vx, navigation.state.vy, navigation.state.vz},
          .route_target = execution_horizon_owner.route_target,
          .stationary_hold_position = execution_horizon_owner.stationary_hold_position,
          .pose_receive_stamp_ns = navigation.receive_stamp_ns,
          .vehicle_status_receive_stamp_ns = vehicle_status.receive_stamp_ns,
          .horizon_valid_from_ns = execution_horizon_owner.valid_from_ns,
          .horizon_valid_until_ns = execution_horizon_owner.valid_until_ns,
          .feedback_source_stamp_ns = applied_control.source_stamp_ns,
          .feedback_receive_stamp_ns = applied_control.receive_stamp_ns,
          .horizon_producer_instance_id = execution_horizon_owner.producer_instance_id,
          .horizon_sequence = execution_horizon_owner.sequence,
          .hold_id = hold_id,
          .previous_horizon_sequence_same_hold = previous_horizon_sequence_same_hold,
          .target_offboard_instance_id =
              execution_horizon_owner.target_offboard_instance_id,
          .feedback_horizon_producer_instance_id =
              applied_control.horizon_producer_instance_id,
          .feedback_horizon_sequence = applied_control.horizon_sequence,
          .feedback_offboard_instance_id = applied_control.producer_instance_id,
          .feedback_continuity_generation = applied_control_discontinuity_generation,
          .goal_capture_latched = goal_capture_latched,
          .position_velocity_authoritative = navigation.position_velocity_authoritative,
          .vehicle_status_valid = vehicle_status.valid,
          .vehicle_status_epoch_stable = vehicle_status_epoch_stable,
          .armed = vehicle_status.armed,
          .horizon_valid = execution_horizon_owner.valid,
          .horizon_position_hold = execution_horizon_owner.execution_mode ==
                                   ExecutionAuthorityMode3D::kPositionHold,
          .horizon_goal_capture = execution_horizon_owner.execution_reason ==
                                  ExecutionAuthorityReason3D::kGoalCapture,
          .horizon_stationary_position_hold =
              execution_horizon_owner.stationary_position_hold,
          .feedback_valid = applied_control.valid,
          .feedback_position_hold =
              applied_control.execution_mode == ExecutionAuthorityMode3D::kPositionHold,
          .feedback_control_authoritative = applied_control.control_authoritative,
          .feedback_continuity_generation_valid =
              applied_control_discontinuity_generation_valid,
      });
  // A broken witness restarts the gate's own continuity; it is not evidence
  // against the resident hold. Invalidating the applied-control witness here
  // would change its continuity generation, which the next observation would
  // see as another break, and every break would revoke and re-lease the hold.
  if (capture.continuity_broken) {
    ++mission_capture_continuity_breaks_;
    mission_capture_last_break_reason_ = capture.continuity_break_reason;
  }
  if (goal_capture_latched && !capture.ready) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MISSION_CAPTURE_GATE ready=false evidence=%s reason=%s continuous_ms=%.0f "
        "breaks=%" PRIu64 " last_break=%s horizon=%" PRIu64 " feedback_horizon=%" PRIu64
        " feedback_age_ms=%.0f",
        capture.evidence_valid ? "valid" : "invalid", capture.ineligibility,
        static_cast<double>(capture.continuous_duration_ns) * 1.0e-6,
        mission_capture_continuity_breaks_, mission_capture_last_break_reason_,
        execution_horizon_owner.sequence, applied_control.horizon_sequence,
        applied_control.source_stamp_ns > 0
            ? static_cast<double>(now_ns - applied_control.source_stamp_ns) * 1.0e-6
            : -1.0);
  }
  if (!capture.ready ||
      mission_waypoint_acknowledgement_sequence_ ==
          std::numeric_limits<std::uint64_t>::max() ||
      mission_waypoint_sequence_->waypointCount() >
          std::numeric_limits<std::uint32_t>::max() ||
      (mission_waypoint_sequence_->activeIndex() + 1U <
           mission_waypoint_sequence_->waypointCount() &&
       objective->mission_epoch == std::numeric_limits<std::uint64_t>::max())) {
    return {};
  }

  MissionWaypointUpdate update;
  std::int64_t acknowledgement_stamp_ns{0};
  {
    // Goal capture is an irreversible mission transition. Linearize it with the
    // exact objective and controller witness sampled by this planning tick so a
    // concurrent objective, status, feedback, or horizon update cannot be
    // acknowledged and then overwritten by the successor leg below.
    const auto lock = evidence_boundary_.inputWithObjectiveReplan();
    const std::int64_t commit_now_ns = get_clock()->now().nanoseconds();
    const bool commit_time_valid =
        commit_now_ns >= now_ns && navigation_.receive_stamp_ns > 0 &&
        commit_now_ns >= navigation_.receive_stamp_ns &&
        static_cast<double>(commit_now_ns - navigation_.receive_stamp_ns) * 1.0e-6 <=
            config_.execution.maximum_pose_age_ms;
    const bool objective_current =
        navigation_objective_state_.load(std::memory_order_acquire) == objective_state;
    const bool navigation_current =
        navigation_.revision == navigation.revision &&
        navigation_.source_timestamp_us == navigation.source_timestamp_us &&
        navigation_.receive_stamp_ns == navigation.receive_stamp_ns &&
        navigation_.valid == navigation.valid &&
        navigation_.position_velocity_authoritative ==
            navigation.position_velocity_authoritative &&
        !navigation_revision_exhausted_ && !navigation_frame_reset_unresolved_;
    const bool status_current =
        vehicle_status_.revision == vehicle_status.revision &&
        vehicle_status_.source_timestamp_us == vehicle_status.source_timestamp_us &&
        vehicle_status_.receive_stamp_ns == vehicle_status.receive_stamp_ns &&
        vehicle_status_.valid == vehicle_status.valid &&
        vehicle_status_.armed == vehicle_status.armed &&
        !vehicle_status_epoch_probation_ && !vehicle_status_revision_exhausted_ &&
        vehicleStatusAuthoritativeForExecution(
            vehicle_status_, true, commit_now_ns,
            config_.execution.maximum_vehicle_status_age_ms);
    const bool authority_current =
        execution_supervisor_.authority() == execution_authority;
    const bool owner_current = authority_current && execution_horizon_owner.valid &&
                               execution_horizon_owner.valid_from_ns > 0 &&
                               execution_horizon_owner.valid_until_ns >
                                   execution_horizon_owner.valid_from_ns &&
                               commit_now_ns >= execution_horizon_owner.valid_from_ns &&
                               commit_now_ns < execution_horizon_owner.valid_until_ns;
    const bool feedback_current =
        authority_current && applied_control.valid &&
        !applied_control.control_authoritative &&
        applied_control.execution_mode == ExecutionAuthorityMode3D::kPositionHold &&
        applied_control.source_stamp_ns > 0 &&
        applied_control.receive_stamp_ns >= applied_control.source_stamp_ns &&
        commit_now_ns >= applied_control.source_stamp_ns &&
        commit_now_ns >= applied_control.receive_stamp_ns &&
        static_cast<double>(commit_now_ns - applied_control.source_stamp_ns) * 1.0e-6 <=
            config_.execution.maximum_control_feedback_age_ms &&
        static_cast<double>(commit_now_ns - applied_control.receive_stamp_ns) *
                1.0e-6 <=
            config_.execution.maximum_control_feedback_age_ms;
    const bool revocation_current =
        requested_execution_revocation_.load(std::memory_order_acquire) ==
        handled_execution_revocation_request_;
    const bool feedback_continuity_current =
        applied_control_discontinuity_generation_valid &&
        !applied_control_discontinuity_generation_exhausted_ &&
        applied_control_discontinuity_generation_ ==
            applied_control_discontinuity_generation;
    const bool active_goal_current =
        distance3D(mission_waypoint_sequence_->activeGoal(), objective->goal) <=
        config_.execution.mission_waypoint_capture_gate.target_match_tolerance_m;
    if (!commit_time_valid || !objective_current || !navigation_current ||
        !status_current || !authority_current || !owner_current || !feedback_current ||
        !feedback_continuity_current || !revocation_current || !active_goal_current) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "MISSION_CAPTURE_COMMIT rejected=true commit_time=%s objective=%s "
          "navigation=%s status=%s authority=%s owner=%s feedback=%s "
          "feedback_continuity=%s revocation=%s active_goal=%s",
          commit_time_valid ? "current" : "stale",
          objective_current ? "current" : "stale",
          navigation_current ? "current" : "stale",
          status_current ? "current" : "stale", authority_current ? "current" : "stale",
          owner_current ? "current" : "stale", feedback_current ? "current" : "stale",
          feedback_continuity_current ? "current" : "stale",
          revocation_current ? "current" : "stale",
          active_goal_current ? "current" : "stale");
      return {};
    }

    acknowledgement_stamp_ns = commit_now_ns;
    update = mission_waypoint_sequence_->acknowledgeGoalCapture();
    if (update.advanced) {
      mission_goal_ = mission_waypoint_sequence_->activeGoal();
      // The successor leg is a non-tracking objective, so it carries no minimum
      // tracking-route requirement. Publishing both halves together keeps the
      // requirement from outliving the epoch that produced it.
      navigation_objective_state_.store(
          std::make_shared<const ProductionNavigationObjectiveState>(
              ProductionNavigationObjectiveState{
                  .objective = std::make_shared<const ProductionNavigationObjective>(
                      ProductionNavigationObjective{
                          .goal = mission_goal_,
                          .tracking = std::nullopt,
                          .mission_epoch = objective->mission_epoch + 1U,
                          .sample_sequence = 0U,
                          .assignment_generation = 0U,
                          .target_detection_id = 0U,
                          .target_track_id = 0U,
                          .stamp_ns = commit_now_ns,
                          .continuous_tracking = false,
                          .immediate_hold = false,
                      }),
                  .minimum_tracking_route_mission_epoch = 0U,
                  .minimum_tracking_route_sample_sequence = 0U,
              }),
          std::memory_order_release);
      // Keep the completed leg's wire owner as the revocation witness, but make
      // its applied-control evidence unusable as soon as the objective epoch
      // advances. The next planning tick must linearize the queued revocation
      // before it can publish an owner for the successor leg.
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kNoExecutableHorizon);
      objective_replan_anchor_ = mission_goal_;
      objective_replan_stamp_ns_ = commit_now_ns;
    }
  }
  if (!update.waypoint_completed) {
    mission_waypoint_capture_gate_->reset();
    return update;
  }
  publishMissionWaypointAcknowledgement(*objective, update, applied_control,
                                        execution_horizon_owner,
                                        acknowledgement_stamp_ns);
  mission_waypoint_capture_gate_->reset();
  if (!update.advanced) {
    RCLCPP_INFO(get_logger(),
                "MISSION_WAYPOINT_ACKNOWLEDGED completed_index=%zu waypoint_count=%zu "
                "terminal=true horizon=%" PRIu64 " offboard=%" PRIu64,
                update.completed_index, mission_waypoint_sequence_->waypointCount(),
                execution_horizon_owner.sequence,
                execution_horizon_owner.target_offboard_instance_id);
    return update;
  }

  retireGoalHoldForSuccessorLeg();
  requestRouteRelease(RouteReleaseReason3D::kObjectiveChanged);
  RCLCPP_INFO(get_logger(),
              "MISSION_WAYPOINT_ACKNOWLEDGED completed_index=%zu waypoint_count=%zu "
              "horizon=%" PRIu64 " offboard=%" PRIu64 " next_goal=(%.2f,%.2f,%.2f)",
              update.completed_index, mission_waypoint_sequence_->waypointCount(),
              execution_horizon_owner.sequence,
              execution_horizon_owner.target_offboard_instance_id, mission_goal_.x,
              mission_goal_.y, mission_goal_.z);
  return update;
}

void ProductionMppiNode::publishMissionWaypointAcknowledgement(
    const ProductionNavigationObjective& completed_objective,
    const MissionWaypointUpdate& update,
    const AppliedControlEvidence3D& applied_control,
    const ExecutionOwnerIdentity3D& execution_horizon_owner,
    const std::int64_t now_ns) {
  msg::MissionWaypointAcknowledgement acknowledgement;
  acknowledgement.header.stamp = timeFromNanoseconds(now_ns);
  acknowledgement.header.frame_id = config_.world.frame_id;
  acknowledgement.producer_instance_id = execution_horizon_producer_instance_id_;
  acknowledgement.acknowledgement_sequence =
      ++mission_waypoint_acknowledgement_sequence_;
  acknowledgement.mission_epoch = completed_objective.mission_epoch;
  acknowledgement.completed_waypoint_index =
      static_cast<std::uint32_t>(update.completed_index);
  acknowledgement.completed_waypoint_count =
      static_cast<std::uint32_t>(mission_waypoint_sequence_->completedWaypointCount());
  acknowledgement.waypoint_count =
      static_cast<std::uint32_t>(mission_waypoint_sequence_->waypointCount());
  acknowledgement.active_waypoint_index =
      static_cast<std::uint32_t>(mission_waypoint_sequence_->activeIndex());
  acknowledgement.mission_completed = update.mission_completed;
  const Point3& completed_goal = completed_objective.goal;
  acknowledgement.completed_goal.x = completed_goal.x;
  acknowledgement.completed_goal.y = completed_goal.y;
  acknowledgement.completed_goal.z = completed_goal.z;
  acknowledgement.horizon_producer_instance_id =
      execution_horizon_owner.producer_instance_id;
  acknowledgement.horizon_sequence = execution_horizon_owner.sequence;
  acknowledgement.offboard_producer_instance_id =
      execution_horizon_owner.target_offboard_instance_id;
  acknowledgement.horizon_valid_from =
      timeFromNanoseconds(execution_horizon_owner.valid_from_ns);
  acknowledgement.horizon_valid_until =
      timeFromNanoseconds(execution_horizon_owner.valid_until_ns);
  acknowledgement.witness_stamp = timeFromNanoseconds(applied_control.source_stamp_ns);
  acknowledgement.route_target.x = execution_horizon_owner.route_target.x;
  acknowledgement.route_target.y = execution_horizon_owner.route_target.y;
  acknowledgement.route_target.z = execution_horizon_owner.route_target.z;
  acknowledgement.stationary_hold_position.x =
      execution_horizon_owner.stationary_hold_position.x;
  acknowledgement.stationary_hold_position.y =
      execution_horizon_owner.stationary_hold_position.y;
  acknowledgement.stationary_hold_position.z =
      execution_horizon_owner.stationary_hold_position.z;
  mission_waypoint_acknowledgement_pub_->publish(acknowledgement);
}

} // namespace drone_city_nav
