#include "production_mppi_node_planning_tick_rearm.hpp"

#include <memory>
#include <optional>

#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool exactZeroControl(const mppi::Control& control) noexcept {
  return control.ax == 0.0F && control.ay == 0.0F && control.az == 0.0F &&
         control.yaw_accel == 0.0F;
}

[[nodiscard]] bool
appliedControlEmpty(const ProductionMppiAppliedControl& control) noexcept {
  return !control.valid && !control.control_authoritative &&
         !control.yaw_acceleration_authoritative && exactZeroControl(control.control) &&
         control.yaw_rate_radps == 0.0F && control.source_stamp_ns == 0 &&
         control.receive_stamp_ns == 0 && control.producer_instance_id == 0U &&
         control.horizon_producer_instance_id == 0U && control.horizon_sequence == 0U &&
         control.content_fingerprint == 0U;
}

[[nodiscard]] bool
executionHorizonOwnerEmpty(const ProductionMppiExecutionHorizonOwner& owner) noexcept {
  return !owner.valid && !owner.stationary_position_hold &&
         owner.route_target.x == 0.0 && owner.route_target.y == 0.0 &&
         owner.route_target.z == 0.0 && owner.stationary_hold_position.x == 0.0 &&
         owner.stationary_hold_position.y == 0.0 &&
         owner.stationary_hold_position.z == 0.0 && owner.valid_from_ns == 0 &&
         owner.valid_until_ns == 0 && owner.producer_instance_id == 0U &&
         owner.target_offboard_instance_id == 0U && owner.sequence == 0U &&
         owner.execution_mode ==
             msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD &&
         owner.execution_reason == msg::MppiTrajectoryHorizon::EXECUTION_REASON_NONE;
}

[[nodiscard]] bool executionSnapshotRevokedEmpty(
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& snapshot) noexcept {
  return snapshot != nullptr && snapshot->valid() &&
         snapshot->phase == ExecutionRoutePhase3D::kRevoked &&
         !snapshot->route.has_value() && !snapshot->finite_execution.has_value() &&
         !snapshot->direct_tracking_execution.has_value() &&
         !snapshot->stationary_hold.has_value();
}

[[nodiscard]] bool sameRawMapVersion(const RawMapVersion& first,
                                     const RawMapVersion& second) noexcept {
  return first.producer_instance_id == second.producer_instance_id &&
         first.base_snapshot_revision == second.base_snapshot_revision &&
         first.revision == second.revision;
}

[[nodiscard]] bool observedWorldCurrentForStationaryRearm(
    const ProductionMppiPreparedEsdf& esdf,
    const ProductionMppiRawWorld3D* raw_world) noexcept {
  if (raw_world == nullptr || !raw_world->version.valid() ||
      raw_world->occupancy == nullptr || raw_world->execution_owner == nullptr ||
      !raw_world->execution_owner->valid() ||
      !sameRawMapVersion(raw_world->version, raw_world->execution_owner->version()) ||
      std::addressof(raw_world->execution_owner->occupancy()) !=
          raw_world->occupancy.get() ||
      esdf.observed_raw_world_owner == nullptr ||
      !esdf.observed_raw_world_owner->valid() ||
      !sameRawMapVersion(esdf.observed_raw_world_owner->version(),
                         raw_world->version) ||
      std::addressof(esdf.observed_raw_world_owner->occupancy()) !=
          raw_world->occupancy.get()) {
    return false;
  }
  return raw_world->execution_owner->sharesObservationOwner(
             *esdf.observed_raw_world_owner) &&
         raw_world->execution_owner->occupiedSnapshot() ==
             esdf.observed_raw_world_owner->occupiedSnapshot();
}

} // namespace

bool stationaryCaptureRearmEligibleForPlanningTick(
    const ProductionMppiStationaryCaptureRearmContext& context) {
  if (context.objective == nullptr || context.mission_waypoint_sequence == nullptr ||
      context.navigation == nullptr || context.vehicle_status == nullptr ||
      context.applied_control == nullptr ||
      context.execution_horizon_owner == nullptr ||
      context.offboard_session == nullptr || context.esdf == nullptr) {
    return false;
  }
  const bool stationary_rearm_candidate =
      !context.objective->tracking.has_value() && !context.objective->immediate_hold &&
      context.terminal_hold_enabled && context.goal_capture_latched;
  const bool validation_policy_current = stationary_rearm_candidate &&
                                         context.validation_policy != nullptr &&
                                         context.validation_policy->valid();
  const bool lidar_evidence_current =
      validation_policy_current && context.latest_lidar_evidence != nullptr &&
      assessLatestLidarEvidenceFreshness3D(
          *context.latest_lidar_evidence, context.now_ns,
          context.validation_policy->latestLidarMaximumAgeMs())
          .fresh;
  const bool static_world_current =
      stationary_rearm_candidate && context.use_static_map &&
      context.static_occupancy_3d != nullptr &&
      VersionedStaticWorld3D::captureOwned(navigationWorldCertificate3D(*context.esdf),
                                           context.static_occupancy_3d) != nullptr;
  const bool observed_world_current =
      stationary_rearm_candidate && context.observed_3d_world &&
      context.observation_age_ms <= context.maximum_esdf_age_ms &&
      observedWorldCurrentForStationaryRearm(*context.esdf,
                                             context.latest_raw_world_3d.get());

  return missionWaypointStationaryRearmEligible(
      MissionWaypointStationaryRearmGateConfig{
          .maximum_pose_age_s = context.maximum_pose_age_ms * 1.0e-3,
          .maximum_vehicle_status_age_s =
              context.capture_gate_config.maximum_vehicle_status_age_s,
          .maximum_offboard_session_age_s =
              context.maximum_control_feedback_age_ms * 1.0e-3,
          .position_tolerance_m = kStationaryExecutionHoldPositionToleranceM,
          .speed_tolerance_mps = kStationaryExecutionHoldSpeedToleranceMps,
          .yaw_rate_tolerance_radps = kStationaryExecutionHoldYawRateToleranceRadps,
      },
      MissionWaypointStationaryRearmObservation{
          .stamp_ns = context.now_ns,
          .mission_goal = context.mission_goal,
          .active_waypoint_goal = context.mission_waypoint_sequence->activeGoal(),
          .position = Point3{context.navigation->state.x, context.navigation->state.y,
                             context.navigation->state.z},
          .velocity = Point3{context.navigation->state.vx, context.navigation->state.vy,
                             context.navigation->state.vz},
          .pose_receive_stamp_ns = context.navigation->receive_stamp_ns,
          .vehicle_status_receive_stamp_ns = context.vehicle_status->receive_stamp_ns,
          .offboard_session_source_stamp_ns =
              context.offboard_session->latest_source_stamp_ns,
          .offboard_session_receive_stamp_ns =
              context.offboard_session_receive_stamp_ns,
          .offboard_instance_id =
              context.offboard_session->current_producer_instance_id,
          .yaw_rate_radps = context.navigation->state.yaw_rate,
          .objective_eligible = stationary_rearm_candidate,
          .goal_capture_latched = context.goal_capture_latched,
          .execution_input_state_authoritative =
              context.navigation->valid && context.navigation->full_state_authoritative,
          .position_velocity_authoritative =
              context.navigation->position_velocity_authoritative,
          .yaw_rate_authoritative = context.navigation->yaw_rate_authoritative,
          .vehicle_status_valid = context.vehicle_status->valid,
          .vehicle_status_epoch_stable = context.vehicle_status_epoch_stable,
          .armed = context.vehicle_status->armed,
          .offboard_session_valid = context.offboard_session->valid(),
          .applied_control_empty = appliedControlEmpty(*context.applied_control),
          .horizon_owner_empty =
              executionHorizonOwnerEmpty(*context.execution_horizon_owner),
          .execution_snapshot_revoked_empty =
              executionSnapshotRevokedEmpty(context.execution_snapshot),
          .validation_policy_current = validation_policy_current,
          .world_evidence_current = static_world_current || observed_world_current,
          .lidar_evidence_current = lidar_evidence_current,
      });
}

ProductionMppiExecutionInputPreparation prepareExecutionInputForPlanningTick(
    const ProductionMppiNavigation& navigation,
    const ProductionMppiAppliedControl& applied_control,
    const ProductionMppiExecutionHorizonOwner& execution_horizon_owner,
    const std::uint64_t execution_input_sequence, const std::int64_t now_ns,
    const double maximum_control_feedback_age_ms, const bool pose_predicted,
    const bool stationary_capture_rearm) {
  ProductionMppiExecutionInputPreparation result;
  result.control_feedback_fresh =
      appliedControlAuthoritativeForExecution(applied_control, execution_horizon_owner,
                                              now_ns, maximum_control_feedback_age_ms);
  result.measured_control_available =
      navigation.measured_acceleration_valid && !pose_predicted;

  std::optional<PreviousControlEvidence3D> previous_control_evidence;
  if (result.control_feedback_fresh && applied_control.horizon_sequence != 0U &&
      applied_control.source_stamp_ns > 0 &&
      applied_control.source_stamp_ns <= applied_control.receive_stamp_ns &&
      applied_control.receive_stamp_ns <= now_ns) {
    previous_control_evidence = PreviousControlEvidence3D{
        .control = applied_control.control,
        .source = ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback,
        .source_producer_instance_id = applied_control.horizon_producer_instance_id,
        .source_sequence = applied_control.horizon_sequence,
        .source_stamp_ns = applied_control.source_stamp_ns,
        .receive_stamp_ns = applied_control.receive_stamp_ns,
    };
    result.previous_control_source =
        ProductionMppiPreviousControlSource::kOffboardFeedback;
  } else if (stationary_capture_rearm) {
    previous_control_evidence = PreviousControlEvidence3D{
        .control = {},
        .source = ExecutionPreviousControlEvidenceSource3D::kAssumedZero,
        .source_sequence = execution_input_sequence,
        .source_stamp_ns = now_ns,
        .receive_stamp_ns = now_ns,
    };
    result.previous_control_source =
        ProductionMppiPreviousControlSource::kStationaryCaptureRearm;
  } else if (result.measured_control_available) {
    previous_control_evidence = PreviousControlEvidence3D{
        .control = navigation.measured_equivalent_control,
        .source = ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration,
        .source_sequence = navigation.source_timestamp_us,
        .source_stamp_ns = navigation.receive_stamp_ns,
        .receive_stamp_ns = navigation.receive_stamp_ns,
    };
    result.previous_control_source =
        ProductionMppiPreviousControlSource::kMeasuredAcceleration;
  }
  result.previous_control_available = previous_control_evidence.has_value();
  if (!previous_control_evidence.has_value()) {
    return result;
  }

  const ExecutionStateFieldProvenance3D source_sample{
      ExecutionStateFieldProvenance3D::kSourceSample};
  const ExecutionStateFieldProvenance3D effective_prediction{
      ExecutionStateFieldProvenance3D::kEffectiveTimePrediction};
  result.execution_input = VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
      .capture_sequence = execution_input_sequence,
      .pose_revision = navigation.revision,
      .pose_source_timestamp_us = navigation.source_timestamp_us,
      .pose_receive_stamp_ns = navigation.receive_stamp_ns,
      .effective_stamp_ns = now_ns,
      .state = navigation.state,
      .full_state_authoritative = navigation.full_state_authoritative,
      .state_provenance =
          ExecutionStateProvenance3D{
              .x = pose_predicted ? effective_prediction : source_sample,
              .y = pose_predicted ? effective_prediction : source_sample,
              .z = pose_predicted ? effective_prediction : source_sample,
              .vx = source_sample,
              .vy = source_sample,
              .vz = source_sample,
              .yaw = pose_predicted ? effective_prediction : source_sample,
              .yaw_rate = source_sample,
          },
      .purpose = stationary_capture_rearm
                     ? ExecutionInputPurpose3D::kStationaryCaptureRearm
                     : ExecutionInputPurpose3D::kGeneralExecution,
      .previous_control = previous_control_evidence->control,
      .previous_control_source = previous_control_evidence->source,
      .previous_control_source_producer_instance_id =
          previous_control_evidence->source_producer_instance_id,
      .previous_control_source_sequence = previous_control_evidence->source_sequence,
      .previous_control_source_stamp_ns = previous_control_evidence->source_stamp_ns,
      .previous_control_receive_stamp_ns = previous_control_evidence->receive_stamp_ns,
  });
  return result;
}

} // namespace drone_city_nav
