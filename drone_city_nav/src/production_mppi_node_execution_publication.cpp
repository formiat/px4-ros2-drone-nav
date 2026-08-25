#include "drone_city_nav/execution_horizon_contract_ros.hpp"
#include "drone_city_nav/execution_publication_currentness_3d.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

#include "production_mppi_node_execution_internal.hpp"

namespace drone_city_nav {

namespace production_mppi_execution_detail {

builtin_interfaces::msg::Time timeFromNanoseconds(const std::int64_t nanoseconds) {
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  time.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return time;
}

std::int64_t timeToNanoseconds(const builtin_interfaces::msg::Time& time) noexcept {
  return static_cast<std::int64_t>(time.sec) * 1000000000LL +
         static_cast<std::int64_t>(time.nanosec);
}

std::optional<std::int64_t>
canonicalHorizonEndTime(const std::int64_t valid_from_ns,
                        const std::int64_t duration_ns) noexcept {
  constexpr std::int64_t kMaximumCanonicalTimeNs =
      static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) *
          1'000'000'000LL +
      999'999'999LL;
  if (valid_from_ns <= 0 || duration_ns <= 0 || duration_ns > kMaximumCanonicalTimeNs ||
      valid_from_ns > kMaximumCanonicalTimeNs - duration_ns) {
    return std::nullopt;
  }
  return valid_from_ns + duration_ns;
}

bool sameState(const mppi::State& first, const mppi::State& second) noexcept {
  return first.x == second.x && first.y == second.y && first.z == second.z &&
         first.vx == second.vx && first.vy == second.vy && first.vz == second.vz &&
         first.yaw == second.yaw && first.yaw_rate == second.yaw_rate;
}

bool sameControl(const mppi::Control& first, const mppi::Control& second) noexcept {
  return first.ax == second.ax && first.ay == second.ay && first.az == second.az &&
         first.yaw_accel == second.yaw_accel;
}

LatestLidarEvidenceFreshness3D latestLidarEvidenceFreshness(
    const std::shared_ptr<const VersionedLatestLidarEvidence3D>& evidence,
    const std::int64_t now_ns, const double maximum_age_ms) noexcept {
  return evidence != nullptr
             ? assessLatestLidarEvidenceFreshness3D(*evidence, now_ns, maximum_age_ms)
             : LatestLidarEvidenceFreshness3D{};
}

bool bindHorizonRouteMetadata(msg::MppiTrajectoryHorizon& horizon,
                              const CertifiedRouteSuffix3D& route) {
  if (route.geometry == nullptr || route.geometry->constrained_spans == nullptr) {
    return false;
  }
  horizon.route_purpose = static_cast<std::uint8_t>(route.geometry->route_purpose);
  horizon.route_constrained = !route.geometry->constrained_spans->empty();
  return true;
}

void appendStationaryHoldPoint(msg::MppiTrajectoryHorizon& horizon,
                               const Point3& hold_position,
                               const std::int64_t time_from_start_ns,
                               const float yaw_rad) {
  msg::MppiHorizonPoint point;
  point.time_from_start_ns = time_from_start_ns;
  point.time_from_start_s =
      static_cast<float>(static_cast<double>(time_from_start_ns) / 1'000'000'000.0);
  point.position.x = hold_position.x;
  point.position.y = hold_position.y;
  point.position.z = hold_position.z;
  point.yaw_rad = yaw_rad;
  horizon.points.push_back(point);
}

bool appendFiniteExecutionPoints(msg::MppiTrajectoryHorizon& horizon,
                                 const std::span<const mppi::State> states,
                                 const std::span<const mppi::Control> controls,
                                 const mppi::Control& previous_applied_control,
                                 const std::int64_t control_interval_ns) {
  if (!horizon.points.empty() || controls.empty() ||
      states.size() != controls.size() + 1U || control_interval_ns <= 0 ||
      states.size() - 1U >
          static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max() /
                                   control_interval_ns)) {
    return false;
  }
  horizon.points.reserve(states.size());
  for (std::size_t index = 0U; index < states.size(); ++index) {
    const mppi::State& state = states[index];
    const mppi::Control& point_control =
        index == 0U ? previous_applied_control : controls[index - 1U];
    msg::MppiHorizonPoint point;
    point.time_from_start_ns = static_cast<std::int64_t>(index) * control_interval_ns;
    point.time_from_start_s = static_cast<float>(
        static_cast<double>(point.time_from_start_ns) / 1'000'000'000.0);
    point.position.x = state.x;
    point.position.y = state.y;
    point.position.z = state.z;
    point.velocity.x = state.vx;
    point.velocity.y = state.vy;
    point.velocity.z = state.vz;
    point.acceleration.x = point_control.ax;
    point.acceleration.y = point_control.ay;
    point.acceleration.z = point_control.az;
    point.yaw_rad = state.yaw;
    point.yaw_rate_radps = state.yaw_rate;
    point.yaw_acceleration_radps2 = point_control.yaw_accel;
    horizon.points.push_back(point);
  }
  return true;
}

} // namespace production_mppi_execution_detail

namespace {

constexpr std::uint64_t kExecutionRevocationReasonMask{0xffU};
constexpr std::uint64_t kExecutionRevocationRequestStep{kExecutionRevocationReasonMask +
                                                        1U};

[[nodiscard]] bool
failClosedExecutionReason(const ProductionMppiExecutionReason reason) noexcept {
  return reason == ProductionMppiExecutionReason::kNoExecutableHorizon ||
         reason == ProductionMppiExecutionReason::kNoExecutableRoute ||
         reason == ProductionMppiExecutionReason::kUnavailableWorld;
}

[[nodiscard]] std::shared_ptr<const VersionedObservedRawWorld3D>
snapshotRawOwner(const ExecutionRouteSnapshot3D& snapshot) {
  if (snapshot.stationary_hold.has_value()) {
    return snapshot.stationary_hold->observed_raw_world;
  }
  if (snapshot.finite_execution.has_value() &&
      snapshot.finite_execution->observed_raw_world != nullptr) {
    return snapshot.finite_execution->observed_raw_world;
  }
  if (snapshot.direct_tracking_execution.has_value() &&
      snapshot.direct_tracking_execution->observed_raw_world != nullptr) {
    return snapshot.direct_tracking_execution->observed_raw_world;
  }
  return snapshot.route.has_value() ? snapshot.route->observed_raw_world : nullptr;
}

[[nodiscard]] std::shared_ptr<const VersionedLatestLidarEvidence3D>
snapshotLidarOwner(const ExecutionRouteSnapshot3D& snapshot) {
  if (snapshot.stationary_hold.has_value()) {
    return snapshot.stationary_hold->latest_lidar_evidence;
  }
  if (snapshot.finite_execution.has_value()) {
    return snapshot.finite_execution->latest_lidar_evidence;
  }
  return snapshot.direct_tracking_execution.has_value()
             ? snapshot.direct_tracking_execution->latest_lidar_evidence
             : nullptr;
}

[[nodiscard]] std::shared_ptr<const VersionedExecutionValidationPolicy3D>
snapshotValidationPolicy(const ExecutionRouteSnapshot3D& snapshot) {
  if (snapshot.stationary_hold.has_value()) {
    return snapshot.stationary_hold->validation_policy;
  }
  if (snapshot.finite_execution.has_value()) {
    return snapshot.finite_execution->validation_policy;
  }
  if (snapshot.direct_tracking_execution.has_value()) {
    return snapshot.direct_tracking_execution->validation_policy;
  }
  return snapshot.route.has_value() ? snapshot.route->validation_policy : nullptr;
}

struct FiniteExecutionEvidenceView {
  const mppi::FiniteHorizon* horizon{nullptr};
  const VersionedExecutionInput3D* execution_input{nullptr};
  const VersionedExecutionValidationPolicy3D* policy{nullptr};
  const VersionedStaticWorld3D* static_world{nullptr};
  std::int64_t control_interval_ns{0};
};

[[nodiscard]] std::optional<FiniteExecutionEvidenceView>
finiteExecutionEvidenceView(const ExecutionRouteSnapshot3D& snapshot) noexcept {
  const auto make_view =
      [](const auto& execution) noexcept -> std::optional<FiniteExecutionEvidenceView> {
    if (execution.horizon == nullptr || execution.execution_input == nullptr ||
        execution.validation_policy == nullptr ||
        (execution.observed_raw_world == nullptr) ==
            (execution.static_world == nullptr)) {
      return std::nullopt;
    }
    return FiniteExecutionEvidenceView{
        .horizon = execution.horizon.get(),
        .execution_input = execution.execution_input.get(),
        .policy = execution.validation_policy.get(),
        .static_world = execution.static_world.get(),
        .control_interval_ns = execution.control_interval_ns,
    };
  };
  if (snapshot.finite_execution.has_value()) {
    return make_view(*snapshot.finite_execution);
  }
  if (snapshot.direct_tracking_execution.has_value()) {
    return make_view(*snapshot.direct_tracking_execution);
  }
  return std::nullopt;
}

[[nodiscard]] bool revalidateFiniteExecutionAgainstLatestEvidence(
    const ExecutionRouteSnapshot3D& snapshot,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& latest_raw,
    const std::shared_ptr<const VersionedLatestLidarEvidence3D>&
        latest_lidar) noexcept {
  const std::optional<FiniteExecutionEvidenceView> view =
      finiteExecutionEvidenceView(snapshot);
  if (!view.has_value() || view->horizon == nullptr ||
      view->execution_input == nullptr || view->policy == nullptr ||
      !view->policy->valid() || latest_lidar == nullptr || !latest_lidar->valid() ||
      view->control_interval_ns <= 0 ||
      view->horizon->states.size() != view->horizon->controls.size() + 1U ||
      view->horizon->controls.empty()) {
    return false;
  }
  const bool static_world = view->static_world != nullptr;
  if ((!static_world && (latest_raw == nullptr || !latest_raw->valid())) ||
      (static_world && !view->static_world->valid())) {
    return false;
  }
  const double step_s = static_cast<double>(view->control_interval_ns) * 1.0e-9;
  if (!std::isfinite(step_s) || step_s <= 0.0) {
    return false;
  }
  std::vector<mppi::TimedExecutionPathPoint> points;
  points.reserve(view->horizon->states.size());
  for (std::size_t index = 0U; index < view->horizon->states.size(); ++index) {
    points.push_back(mppi::TimedExecutionPathPoint{
        .time_from_start_s = static_cast<double>(index) * step_s,
        .state = view->horizon->states[index],
        .control = index == 0U ? view->execution_input->previousControl()
                               : view->horizon->controls[index - 1U],
    });
  }
  const std::optional<ProprioceptiveFreeSpaceSeed3D>& seed =
      !static_world ? latest_raw->proprioceptiveFreeSpaceSeed()
                    : std::optional<ProprioceptiveFreeSpaceSeed3D>{};
  const std::optional<LaunchSupportContact3D>& launch_support =
      !static_world ? latest_raw->launchSupportContact()
                    : std::optional<LaunchSupportContact3D>{};
  const mppi::FiniteExecutionPathWorld world{
      .flight_envelope = &view->policy->flightEnvelope(),
      .dynamics = &view->policy->dynamics(),
      .altitude_envelope = &view->policy->altitudeEnvelope(),
      .footprint = &view->policy->sweptFootprint(),
      .static_occupancy = static_world ? &view->static_world->occupancy() : nullptr,
      .observed_occupancy = !static_world ? &latest_raw->occupancy() : nullptr,
      .require_known_free_space = static_world,
      .proprioceptive_free_space_seed = seed ? std::addressof(*seed) : nullptr,
      .launch_support_contact =
          launch_support ? std::addressof(*launch_support) : nullptr,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = latest_lidar->hitPointsMapM(),
      .terminal_boundary = std::nullopt,
  };
  return mppi::validateCompleteFiniteExecutionPath(
             points, view->execution_input->previousControl(), world)
      .accepted();
}

} // namespace

msg::MppiTrajectoryHorizon
ProductionMppiNode::makeExecutionHorizon(const ProductionMppiExecutionCycle& cycle,
                                         const std::int64_t valid_until_ns,
                                         const ProductionMppiExecutionMode mode,
                                         const ProductionMppiExecutionReason reason) {
  msg::MppiTrajectoryHorizon horizon;
  horizon.header.stamp = now();
  horizon.header.frame_id = frame_id_;
  horizon.producer_instance_id = execution_horizon_producer_instance_id_;
  horizon.target_offboard_instance_id = cycle.target_offboard_instance_id;
  if (execution_horizon_sequence_ != std::numeric_limits<std::uint64_t>::max()) {
    horizon.sequence = ++execution_horizon_sequence_;
  }
  horizon.valid_from =
      production_mppi_execution_detail::timeFromNanoseconds(cycle.now_ns);
  horizon.valid_until =
      production_mppi_execution_detail::timeFromNanoseconds(valid_until_ns);
  horizon.control_interval_ns = cycle.finite_path_control_interval_ns;
  horizon.pose_revision = cycle.input.pose_revision;
  horizon.obstacle_revision = cycle.latest_obstacle_revision;
  horizon.risk_tier = static_cast<std::uint8_t>(cycle.result.selected_tier);
  horizon.execution_mode = static_cast<std::uint8_t>(mode);
  horizon.execution_reason = static_cast<std::uint8_t>(reason);
  horizon.route_purpose = static_cast<std::uint8_t>(cycle.publication_route_purpose);
  horizon.route_target.x = cycle.input.target.x;
  horizon.route_target.y = cycle.input.target.y;
  horizon.route_target.z = cycle.input.target.z;
  horizon.route_constrained = cycle.publication_route_constrained;
  return horizon;
}

bool ProductionMppiNode::commitAndPublishExecutionHorizon(
    const ProductionMppiExecutionCycle& cycle,
    const msg::MppiTrajectoryHorizon& horizon,
    const ProductionMppiHorizonCommit& commit) {
  using production_mppi_execution_detail::sameControl;
  using production_mppi_execution_detail::timeToNanoseconds;

  if (cycle.execution_input == nullptr || !cycle.execution_input->valid() ||
      assessExecutionHorizonPayload(horizon,
                                    ExecutionHorizonPayloadValidationConfig{
                                        .expected_frame_id = frame_id_,
                                        .flight_envelope = &flight_envelope_config_,
                                    }) != ExecutionHorizonPayloadStatus::kValid) {
    return false;
  }
  ProductionMppiExecutionHorizonOwner owner{
      .route_target = Point3{horizon.route_target.x, horizon.route_target.y,
                             horizon.route_target.z},
      .stationary_hold_position =
          Point3{horizon.stationary_hold_position.x, horizon.stationary_hold_position.y,
                 horizon.stationary_hold_position.z},
      .valid_from_ns = timeToNanoseconds(horizon.valid_from),
      .valid_until_ns = timeToNanoseconds(horizon.valid_until),
      .producer_instance_id = horizon.producer_instance_id,
      .target_offboard_instance_id = horizon.target_offboard_instance_id,
      .sequence = horizon.sequence,
      .execution_mode = horizon.execution_mode,
      .execution_reason = horizon.execution_reason,
      .stationary_position_hold = horizon.stationary_position_hold,
  };
  owner.valid =
      owner.producer_instance_id == execution_horizon_producer_instance_id_ &&
      owner.sequence != 0U && owner.valid_from_ns > 0 &&
      owner.valid_until_ns > owner.valid_from_ns &&
      owner.target_offboard_instance_id != 0U &&
      owner.execution_mode <= msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD;
  if (!owner.valid) {
    return false;
  }
  const std::scoped_lock input_lock{input_mutex_};
  const std::int64_t publication_now_ns = get_clock()->now().nanoseconds();
  const bool vehicle_status_epoch_stable =
      !vehicle_status_epoch_probation_ && !vehicle_status_revision_exhausted_;
  if (!vehicleStatusAuthoritativeForExecution(
          vehicle_status_, vehicle_status_epoch_stable, publication_now_ns,
          maximum_vehicle_status_age_ms_)) {
    if (execution_horizon_owner_.valid) {
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
    return false;
  }
  if (!use_static_map_) {
    const LatestObservation& current_observation = latest_observation_tracker_.latest();
    const double maximum_observation_age_ms =
        maximum_esdf_age_ms_ + stale_esdf_execution_window_ms_;
    if (required_raw_world_source_stamp_ns_ != 0 || raw_world_identity_conflicted_ ||
        !current_observation.available() ||
        current_observation.producer_instance_id != cycle.esdf.producer_instance_id ||
        current_observation.ageMs(publication_now_ns) > maximum_observation_age_ms) {
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
      return false;
    }
  }
  if (publication_now_ns < owner.valid_from_ns ||
      publication_now_ns >= owner.valid_until_ns) {
    return false;
  }
  if (requested_execution_revocation_.load(std::memory_order_acquire) !=
          handled_execution_revocation_request_ ||
      navigation_objective_.load(std::memory_order_acquire) != cycle.objective ||
      !navigation_.valid || !offboard_session_admission_.valid() ||
      offboard_session_admission_.latest_source_stamp_ns <= 0 ||
      offboard_session_receive_stamp_ns_ <= 0 ||
      publication_now_ns < offboard_session_admission_.latest_source_stamp_ns ||
      publication_now_ns < offboard_session_receive_stamp_ns_ ||
      static_cast<double>(publication_now_ns -
                          offboard_session_admission_.latest_source_stamp_ns) *
              1.0e-6 >
          maximum_control_feedback_age_ms_ ||
      static_cast<double>(publication_now_ns - offboard_session_receive_stamp_ns_) *
              1.0e-6 >
          maximum_control_feedback_age_ms_ ||
      offboard_session_admission_.current_producer_instance_id !=
          owner.target_offboard_instance_id ||
      navigation_.revision != cycle.execution_input->poseRevision() ||
      navigation_.source_timestamp_us !=
          cycle.execution_input->poseSourceTimestampUs() ||
      navigation_.receive_stamp_ns != cycle.execution_input->poseReceiveStampNs()) {
    return false;
  }
  const ExecutionRouteSnapshot3D* publication_snapshot{nullptr};
  if (cycle.snapshot_owner_required) {
    if ((commit.kind == ProductionMppiHorizonCommitKind::kPublishSnapshotTransition ||
         commit.kind ==
             ProductionMppiHorizonCommitKind::kCommitPendingSnapshotTransition) &&
        commit.transition != nullptr && commit.transition->applied() &&
        commit.transition->next != nullptr) {
      publication_snapshot = commit.transition->next.get();
    } else if (commit.kind ==
                   ProductionMppiHorizonCommitKind::kConfirmSnapshotUnchanged &&
               commit.expected_snapshot != nullptr) {
      publication_snapshot = commit.expected_snapshot.get();
    } else {
      return false;
    }
  }
  const std::shared_ptr<const VersionedExecutionValidationPolicy3D>
      snapshot_publication_policy =
          publication_snapshot != nullptr
              ? snapshotValidationPolicy(*publication_snapshot)
              : nullptr;
  const VersionedExecutionValidationPolicy3D* const publication_policy =
      snapshot_publication_policy != nullptr ? snapshot_publication_policy.get()
      : cycle.selected_policy != nullptr && cycle.selected_policy->valid()
          ? cycle.selected_policy
          : execution_validation_policy_.get();
  if (publication_policy == nullptr || !publication_policy->valid() ||
      !executionInputFreshAt(*cycle.execution_input, *publication_policy,
                             publication_now_ns)) {
    if (cycle.snapshot_owner_required || execution_horizon_owner_.valid) {
      requestExecutionRevocation(ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    return false;
  }
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> publication_lidar =
      publication_snapshot != nullptr ? snapshotLidarOwner(*publication_snapshot)
                                      : cycle.latest_lidar_evidence;
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      latest_lidar_evidence_.load(std::memory_order_acquire);
  if (latest_lidar_evidence_identity_conflicted_.load(std::memory_order_acquire) ||
      publication_lidar == nullptr ||
      (publication_lidar != current_lidar && !commit.latest_evidence_revalidated) ||
      !assessLatestLidarEvidenceFreshness3D(
           *publication_lidar, publication_now_ns,
           publication_policy->latestLidarMaximumAgeMs())
           .fresh) {
    requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    return false;
  }
  const bool legacy_raw_2d_required =
      !cycle.snapshot_owner_required && !use_static_map_ &&
      no_static_world_model_ == ProductionNoStaticWorldModel::kOccupancy2D;
  if (legacy_raw_2d_required &&
      (cycle.latest_raw_world == nullptr ||
       latest_raw_world_.load(std::memory_order_acquire) != cycle.latest_raw_world)) {
    requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    return false;
  }
  const StationaryExecutionHold3D* const capture_hold =
      commit.transition != nullptr && commit.transition->next != nullptr &&
              commit.transition->next->stationary_hold.has_value()
          ? std::addressof(*commit.transition->next->stationary_hold)
          : nullptr;
  const bool stationary_capture_rearm_commit =
      cycle.planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold &&
      cycle.execution_input->stationaryCaptureStateAuthoritative() &&
      commit.kind == ProductionMppiHorizonCommitKind::kPublishSnapshotTransition &&
      commit.expected_snapshot != nullptr &&
      commit.expected_snapshot->phase == ExecutionRoutePhase3D::kRevoked &&
      commit.transition != nullptr && commit.transition->applied() &&
      capture_hold != nullptr &&
      capture_hold->origin ==
          StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm &&
      capture_hold->terminal_execution_input == cycle.execution_input &&
      capture_hold->position.x == owner.stationary_hold_position.x &&
      capture_hold->position.y == owner.stationary_hold_position.y &&
      capture_hold->position.z == owner.stationary_hold_position.z &&
      owner.execution_mode ==
          msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD &&
      owner.execution_reason ==
          msg::MppiTrajectoryHorizon::EXECUTION_REASON_GOAL_CAPTURE &&
      owner.stationary_position_hold && !execution_horizon_owner_.valid &&
      !applied_control_.valid &&
      production_mppi_execution_detail::sameState(navigation_.state,
                                                  cycle.execution_input->state());
  bool control_evidence_current{false};
  if (cycle.execution_input->previousControlSource() ==
      ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback) {
    control_evidence_current =
        appliedControlAuthoritativeForExecution(
            applied_control_, execution_horizon_owner_, publication_now_ns,
            maximum_control_feedback_age_ms_) &&
        applied_control_.horizon_producer_instance_id ==
            cycle.execution_input->previousControlSourceProducerInstanceId() &&
        applied_control_.horizon_sequence ==
            cycle.execution_input->previousControlSourceSequence() &&
        applied_control_.source_stamp_ns ==
            cycle.execution_input->previousControlSourceStampNs() &&
        applied_control_.receive_stamp_ns ==
            cycle.execution_input->previousControlReceiveStampNs() &&
        sameControl(applied_control_.control, cycle.execution_input->previousControl());
  } else if (cycle.execution_input->previousControlSource() ==
             ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration) {
    control_evidence_current =
        navigation_.measured_acceleration_valid &&
        navigation_.source_timestamp_us ==
            cycle.execution_input->previousControlSourceSequence() &&
        navigation_.receive_stamp_ns ==
            cycle.execution_input->previousControlSourceStampNs() &&
        navigation_.receive_stamp_ns ==
            cycle.execution_input->previousControlReceiveStampNs() &&
        sameControl(navigation_.measured_equivalent_control,
                    cycle.execution_input->previousControl());
  } else if (cycle.execution_input->previousControlSource() ==
             ExecutionPreviousControlEvidenceSource3D::kAssumedZero) {
    control_evidence_current = stationary_capture_rearm_commit;
  }
  if (!control_evidence_current) {
    return false;
  }

  bool owner_committed{false};
  switch (commit.kind) {
    case ProductionMppiHorizonCommitKind::kNoOp:
      owner_committed = true;
      break;
    case ProductionMppiHorizonCommitKind::kRejectLegacyTrajectory:
      legacy_execution_arbiter_.rejectTrajectory();
      owner_committed = true;
      break;
    case ProductionMppiHorizonCommitKind::kPublishSnapshotTransition:
      owner_committed = commit.expected_snapshot != nullptr &&
                        commit.transition != nullptr &&
                        execution_route_store_.publish(commit.expected_snapshot,
                                                       *commit.transition) ==
                            ExecutionRoutePublicationStatus3D::kPublished;
      break;
    case ProductionMppiHorizonCommitKind::kConfirmSnapshotUnchanged:
      owner_committed = commit.expected_snapshot != nullptr &&
                        execution_route_store_.snapshot() == commit.expected_snapshot;
      break;
    case ProductionMppiHorizonCommitKind::kCommitPendingSnapshotTransition:
      owner_committed = commit.expected_snapshot != nullptr &&
                        commit.transition != nullptr &&
                        commit.expected_pending != nullptr &&
                        pending_certified_route_mailbox_.commitExecutionIfSame(
                            commit.expected_pending, execution_route_store_,
                            commit.expected_snapshot, *commit.transition);
      break;
  }
  if (!owner_committed) {
    return false;
  }
  if (commit.kind ==
      ProductionMppiHorizonCommitKind::kCommitPendingSnapshotTransition) {
    recordPendingRouteStrategyOutcome(commit.expected_pending, true);
  }
  applied_control_ = {};
  execution_horizon_owner_ = owner;
  execution_horizon_pub_->publish(horizon);
  RCLCPP_INFO(get_logger(),
              "EXECUTION_HORIZON published=true producer=%" PRIu64 " sequence=%" PRIu64
              " mode=%s",
              horizon.producer_instance_id, horizon.sequence,
              horizon.execution_mode ==
                      msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED
                  ? "planned"
                  : "non_planned");
  return true;
}

bool ProductionMppiNode::publishLegacyExecutionHorizon(
    const ProductionMppiExecutionCycle& cycle,
    const msg::MppiTrajectoryHorizon& horizon) {
  const std::scoped_lock evidence_lock{execution_evidence_commit_mutex_};
  return commitAndPublishExecutionHorizon(cycle, horizon,
                                          ProductionMppiHorizonCommit{});
}

bool ProductionMppiNode::commitExecutionSnapshotHorizon(
    const ProductionMppiExecutionCycle& cycle,
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& expected,
    const ExecutionRouteTransitionResult3D& transition,
    const msg::MppiTrajectoryHorizon& horizon,
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending) {
  if (expected == nullptr || !transition.applied() || transition.next == nullptr) {
    return false;
  }
  const std::shared_ptr<const VersionedObservedRawWorld3D> expected_raw =
      snapshotRawOwner(*transition.next);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> expected_lidar =
      snapshotLidarOwner(*transition.next);
  const std::shared_ptr<const VersionedExecutionValidationPolicy3D> policy =
      snapshotValidationPolicy(*transition.next);
  if (expected_lidar == nullptr || policy == nullptr || !policy->valid()) {
    return false;
  }

  const std::scoped_lock evidence_lock{execution_evidence_commit_mutex_};
  const std::shared_ptr<const ProductionMppiRawWorld3D> current_raw_container =
      latest_raw_world_3d_.load(std::memory_order_acquire);
  const bool raw_required = expected_raw != nullptr;
  const std::shared_ptr<const VersionedObservedRawWorld3D> current_raw =
      raw_required && current_raw_container != nullptr
          ? current_raw_container->execution_owner
          : nullptr;
  const ExecutionPublicationCurrentnessStatus3D currentness =
      assessExecutionPublicationCurrentness3D(ExecutionPublicationCurrentnessCheck3D{
          .expected_snapshot = expected,
          .current_snapshot = execution_route_store_.snapshot(),
          .raw_requirement = raw_required
                                 ? ExecutionPublicationRawRequirement3D::kRequired
                                 : ExecutionPublicationRawRequirement3D::kOptional,
          .expected_raw_world = expected_raw,
          .current_raw_world = current_raw,
          .expected_lidar_evidence = expected_lidar,
          .current_lidar_evidence =
              latest_lidar_evidence_.load(std::memory_order_acquire),
          .publication_now_ns = get_clock()->now().nanoseconds(),
          .maximum_lidar_age_ms = policy->latestLidarMaximumAgeMs(),
      });
  const bool latest_evidence_revalidated =
      currentness == ExecutionPublicationCurrentnessStatus3D::kRevalidationRequired &&
      revalidateFiniteExecutionAgainstLatestEvidence(
          *transition.next, current_raw,
          latest_lidar_evidence_.load(std::memory_order_acquire));
  if (currentness != ExecutionPublicationCurrentnessStatus3D::kCurrent &&
      !latest_evidence_revalidated) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_HORIZON published=false reason=evidence_not_current "
        "currentness=%.*s expected_snapshot=%" PRIu64,
        static_cast<int>(
            executionPublicationCurrentnessStatus3DName(currentness).size()),
        executionPublicationCurrentnessStatus3DName(currentness).data(),
        expected->version);
    return false;
  }
  const ProductionMppiHorizonCommit commit{
      .kind = expected_pending != nullptr
                  ? ProductionMppiHorizonCommitKind::kCommitPendingSnapshotTransition
                  : ProductionMppiHorizonCommitKind::kPublishSnapshotTransition,
      .expected_snapshot = expected,
      .transition = &transition,
      .expected_pending = expected_pending,
      .latest_evidence_revalidated = latest_evidence_revalidated,
  };
  return commitAndPublishExecutionHorizon(cycle, horizon, commit);
}

ProductionMppiExecutionPublication ProductionMppiNode::publishPositionHold(
    const ProductionMppiExecutionCycle& cycle, const Point3& hold_position,
    const ProductionMppiExecutionReason reason,
    const ProductionMppiHoldOwnershipTransition3D ownership_transition) {
  ProductionMppiExecutionPublication& publication = cycle.publication;
  if (!insideFlightEnvelope(hold_position, flight_envelope_config_)) {
    RCLCPP_ERROR(get_logger(),
                 "EXECUTION_HORIZON rejected reason=hold_outside_flight_envelope "
                 "target_z=%.3f",
                 hold_position.z);
    return publication;
  }
  Point3 owned_hold_position = hold_position;
  const std::scoped_lock evidence_lock{execution_evidence_commit_mutex_};
  std::shared_ptr<const ExecutionRouteSnapshot3D> hold_expected;
  std::optional<ExecutionRouteTransitionResult3D> hold_transition;
  if (cycle.snapshot_owner_required) {
    if (!cycle.latest_lidar_obstacle_fresh ||
        latest_lidar_evidence_.load(std::memory_order_acquire) !=
            cycle.latest_lidar_evidence) {
      return publication;
    }
    hold_expected = execution_route_store_.snapshot();
    if (hold_expected == nullptr ||
        hold_expected != cycle.route_execution.source_snapshot) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EXECUTION_HORIZON published=false reason=hold_source_not_current "
          "snapshot_present=%s",
          hold_expected != nullptr ? "true" : "false");
      return publication;
    }
    const bool stationary_capture_rearm =
        reason == ProductionMppiExecutionReason::kGoalCapture &&
        ownership_transition ==
            ProductionMppiHoldOwnershipTransition3D::kExplicitTransfer &&
        hold_expected->phase == ExecutionRoutePhase3D::kRevoked &&
        !hold_expected->route.has_value() &&
        !hold_expected->finite_execution.has_value() &&
        !hold_expected->direct_tracking_execution.has_value() &&
        !hold_expected->stationary_hold.has_value() &&
        cycle.planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold &&
        cycle.execution_input != nullptr &&
        cycle.execution_input->stationaryCaptureStateAuthoritative() &&
        cycle.selected_policy == execution_validation_policy_.get() &&
        (cycle.direct_observed_world == nullptr) !=
            (cycle.direct_static_world == nullptr);
    if (cycle.execution_input != nullptr &&
        cycle.execution_input->stationaryCaptureStateAuthoritative() &&
        !stationary_capture_rearm) {
      return publication;
    }
    if (stationary_capture_rearm && cycle.direct_observed_world != nullptr &&
        latest_raw_world_3d_.load(std::memory_order_acquire) !=
            cycle.latest_raw_world_3d) {
      return publication;
    }
    if (hold_expected->stationary_hold.has_value() &&
        ownership_transition ==
            ProductionMppiHoldOwnershipTransition3D::kEnterEmptyOwner) {
      owned_hold_position = hold_expected->stationary_hold->position;
    }
    const StationaryExecutionHold3D* const resident_hold =
        hold_expected->stationary_hold.has_value()
            ? std::addressof(*hold_expected->stationary_hold)
            : nullptr;
    const FiniteExecutionState3D* const route_execution =
        hold_expected->finite_execution.has_value()
            ? std::addressof(*hold_expected->finite_execution)
            : nullptr;
    const DirectTrackingFiniteExecution3D* const direct_execution =
        hold_expected->direct_tracking_execution.has_value()
            ? std::addressof(*hold_expected->direct_tracking_execution)
            : nullptr;
    const std::shared_ptr<const VersionedObservedRawWorld3D> source_observed =
        stationary_capture_rearm      ? cycle.direct_observed_world
        : resident_hold != nullptr    ? resident_hold->observed_raw_world
        : route_execution != nullptr  ? route_execution->observed_raw_world
        : direct_execution != nullptr ? direct_execution->observed_raw_world
                                      : nullptr;
    std::shared_ptr<const VersionedObservedRawWorld3D> current_observed;
    if (stationary_capture_rearm) {
      current_observed = source_observed;
    } else if (source_observed != nullptr) {
      const std::shared_ptr<const ProductionMppiRawWorld3D> current_raw =
          latest_raw_world_3d_.load(std::memory_order_acquire);
      if (current_raw == nullptr || current_raw->execution_owner == nullptr ||
          current_raw->execution_owner->version().producer_instance_id !=
              source_observed->version().producer_instance_id) {
        return publication;
      }
      current_observed = current_raw->execution_owner;
    }
    const std::shared_ptr<const VersionedStaticWorld3D> current_static =
        stationary_capture_rearm      ? cycle.direct_static_world
        : resident_hold != nullptr    ? resident_hold->static_world
        : route_execution != nullptr  ? route_execution->static_world
        : direct_execution != nullptr ? direct_execution->static_world
                                      : nullptr;
    const std::shared_ptr<const VersionedExecutionValidationPolicy3D> policy =
        stationary_capture_rearm      ? execution_validation_policy_
        : resident_hold != nullptr    ? resident_hold->validation_policy
        : route_execution != nullptr  ? route_execution->validation_policy
        : direct_execution != nullptr ? direct_execution->validation_policy
                                      : nullptr;
    const auto make_hold_certification = [&]() {
      return StationaryExecutionHoldCertification3D{
          .position = owned_hold_position,
          .execution_input = cycle.execution_input,
          .observed_raw_world = current_observed,
          .static_world = current_static,
          .validation_policy = policy,
          .latest_lidar_evidence = cycle.latest_lidar_evidence,
      };
    };
    const ExecutionRouteTransitionResult3D hold =
        stationary_capture_rearm
            ? armStationaryCaptureHold3D(*hold_expected, hold_expected->version,
                                         make_hold_certification())
            : transferToExecutionHold3D(*hold_expected, hold_expected->version,
                                        make_hold_certification());
    std::shared_ptr<const ExecutionRouteSnapshot3D> owned_snapshot;
    if (hold.applied()) {
      hold_transition.emplace(hold);
      owned_snapshot = hold.next;
    } else if (hold.status == ExecutionRouteTransitionStatus3D::kNoChange) {
      owned_snapshot = hold_expected;
    } else {
      return publication;
    }
    if (owned_snapshot == nullptr || !owned_snapshot->stationary_hold.has_value() ||
        owned_snapshot->route.has_value() ||
        owned_snapshot->finite_execution.has_value() ||
        owned_snapshot->direct_tracking_execution.has_value()) {
      return publication;
    }
    owned_hold_position = owned_snapshot->stationary_hold->position;
  }
  if (!insideFlightEnvelope(owned_hold_position, flight_envelope_config_)) {
    RCLCPP_ERROR(get_logger(),
                 "EXECUTION_HORIZON rejected reason=hold_outside_flight_envelope "
                 "target_z=%.3f",
                 owned_hold_position.z);
    return publication;
  }
  if (cycle.finite_path_control_interval_ns <= 0 ||
      cycle.finite_path_control_interval_ns >
          std::numeric_limits<std::int64_t>::max() / 2) {
    return publication;
  }
  const std::int64_t requested_hold_duration_ns =
      reason == ProductionMppiExecutionReason::kGoalCapture
          ? mission_goal_capture_hold_validity_ns_
          : stationary_hold_validity_ns_;
  const std::int64_t hold_duration_ns =
      std::max(requested_hold_duration_ns, 2 * cycle.finite_path_control_interval_ns);
  const std::optional<std::int64_t> hold_valid_until_ns =
      production_mppi_execution_detail::canonicalHorizonEndTime(cycle.now_ns,
                                                                hold_duration_ns);
  if (!hold_valid_until_ns.has_value()) {
    return publication;
  }
  msg::MppiTrajectoryHorizon horizon = makeExecutionHorizon(
      cycle, *hold_valid_until_ns, ProductionMppiExecutionMode::kPositionHold, reason);
  horizon.stationary_position_hold = true;
  horizon.stationary_hold_position.x = owned_hold_position.x;
  horizon.stationary_hold_position.y = owned_hold_position.y;
  horizon.stationary_hold_position.z = owned_hold_position.z;
  horizon.points.reserve(2U);
  production_mppi_execution_detail::appendStationaryHoldPoint(
      horizon, owned_hold_position, 0, cycle.exact_initial_state.yaw);
  production_mppi_execution_detail::appendStationaryHoldPoint(
      horizon, owned_hold_position, cycle.finite_path_control_interval_ns,
      cycle.exact_initial_state.yaw);

  ProductionMppiHorizonCommit commit;
  if (!cycle.snapshot_owner_required) {
    commit.kind = ProductionMppiHorizonCommitKind::kRejectLegacyTrajectory;
  } else if (hold_expected == nullptr) {
    return publication;
  } else if (hold_transition.has_value()) {
    commit.kind = ProductionMppiHorizonCommitKind::kPublishSnapshotTransition;
    commit.expected_snapshot = hold_expected;
    commit.transition = &*hold_transition;
  } else {
    commit.kind = ProductionMppiHorizonCommitKind::kConfirmSnapshotUnchanged;
    commit.expected_snapshot = hold_expected;
  }
  if (!commitAndPublishExecutionHorizon(cycle, horizon, commit)) {
    return publication;
  }
  publication.horizon = {
      mppi::State{.x = static_cast<float>(owned_hold_position.x),
                  .y = static_cast<float>(owned_hold_position.y),
                  .z = static_cast<float>(owned_hold_position.z),
                  .yaw = cycle.exact_initial_state.yaw},
      mppi::State{.x = static_cast<float>(owned_hold_position.x),
                  .y = static_cast<float>(owned_hold_position.y),
                  .z = static_cast<float>(owned_hold_position.z),
                  .yaw = cycle.exact_initial_state.yaw},
  };
  publication.mode = ProductionMppiExecutionMode::kPositionHold;
  publication.reason = reason;
  publication.latest_lidar_obstacle_sequence = cycle.latest_lidar_obstacle_sequence;
  publication.latest_lidar_obstacle_hit_count =
      cycle.latest_lidar_obstacle_points.size();
  publication.latest_lidar_obstacle_age_ms = cycle.latest_lidar_obstacle_age_ms;
  publication.latest_lidar_obstacle_fresh = cycle.latest_lidar_obstacle_fresh;
  publication.latest_lidar_obstacle_receive_time_fallback =
      cycle.latest_lidar_obstacle_receive_time_fallback;
  publication.published = true;
  return publication;
}

ProductionMppiExecutionPublication ProductionMppiNode::publishNoExecutablePathHold(
    const ProductionMppiExecutionCycle& cycle,
    const ProductionMppiExecutionReason reason) {
  if (cycle.snapshot_owner_required &&
      cycle.route_execution.source_snapshot != nullptr &&
      cycle.route_execution.source_snapshot->stationary_hold.has_value()) {
    ProductionMppiExecutionPublication hold = publishPositionHold(
        cycle, cycle.route_execution.source_snapshot->stationary_hold->position, reason,
        ProductionMppiHoldOwnershipTransition3D::kEnterEmptyOwner);
    if (hold.published) {
      return hold;
    }
  }
  if (std::optional<ProductionMppiExecutionPublication> retained =
          retainActiveFinitePath(cycle, reason);
      retained.has_value()) {
    return *retained;
  }
  if (cycle.snapshot_owner_required) {
    return publishExecutionRevocation(reason, cycle.now_ns);
  }
  if (!legacy_execution_arbiter_.noExecutableHoldPosition().has_value()) {
    legacy_execution_arbiter_.enterNoExecutableHold(Point3{
        cycle.exact_initial_state.x,
        cycle.exact_initial_state.y,
        clampToFlightEnvelope(cycle.exact_initial_state.z, flight_envelope_config_)
            .value_or(flight_envelope_config_.minimum_target_z_m),
    });
    const Point3& hold_position = *legacy_execution_arbiter_.noExecutableHoldPosition();
    RCLCPP_WARN(get_logger(),
                "MPPI_EXECUTION_CONTRACT transition=enter_no_executable_path_hold "
                "reason=%s origin=(%.3f,%.3f,%.3f) "
                "velocity=(%.3f,%.3f,%.3f) previous_acceleration_z=%.3f",
                productionMppiExecutionReasonName(reason), hold_position.x,
                hold_position.y, hold_position.z, cycle.exact_initial_state.vx,
                cycle.exact_initial_state.vy, cycle.exact_initial_state.vz,
                cycle.exact_previous_control.az);
  }
  return publishPositionHold(
      cycle, *legacy_execution_arbiter_.noExecutableHoldPosition(), reason,
      ProductionMppiHoldOwnershipTransition3D::kEnterEmptyOwner);
}

ProductionMppiExecutionPublication ProductionMppiNode::publishExecutionRevocation(
    const ProductionMppiExecutionReason reason, const std::int64_t now_ns) {
  ProductionMppiExecutionPublication publication;
  publication.mode = ProductionMppiExecutionMode::kRevoked;
  publication.reason = reason;
  if (!failClosedExecutionReason(reason) || execution_horizon_pub_ == nullptr ||
      now_ns <= 0 ||
      execution_horizon_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return publication;
  }

  const std::scoped_lock evidence_lock{execution_evidence_commit_mutex_};
  const std::shared_ptr<const ExecutionRouteSnapshot3D> expected =
      execution_route_store_.snapshot();
  if (expected == nullptr) {
    return publication;
  }
  const ExecutionRouteTransitionResult3D transition =
      revokeExecution3D(*expected, expected->version);
  const bool transition_required = transition.applied();
  if (!transition_required &&
      transition.status != ExecutionRouteTransitionStatus3D::kNoChange) {
    return publication;
  }

  msg::MppiTrajectoryHorizon revocation;
  {
    const std::scoped_lock input_lock{input_mutex_};
    const std::int64_t publication_now_ns = get_clock()->now().nanoseconds();
    // A revoked snapshot with no live horizon owner already represents the
    // requested tombstone. Do not emit a fresh transport sequence for every
    // duplicate callback; publish only while there is an owner to revoke.
    if (!transition_required && !execution_horizon_owner_.valid) {
      return publication;
    }
    const bool current_session =
        offboard_session_admission_.valid() &&
        offboard_session_admission_.latest_source_stamp_ns > 0 &&
        offboard_session_receive_stamp_ns_ > 0 &&
        publication_now_ns >= offboard_session_admission_.latest_source_stamp_ns &&
        publication_now_ns >= offboard_session_receive_stamp_ns_ &&
        static_cast<double>(publication_now_ns -
                            offboard_session_admission_.latest_source_stamp_ns) *
                1.0e-6 <=
            maximum_control_feedback_age_ms_ &&
        static_cast<double>(publication_now_ns - offboard_session_receive_stamp_ns_) *
                1.0e-6 <=
            maximum_control_feedback_age_ms_;
    if (!current_session) {
      return publication;
    }
    const std::uint64_t target_offboard_instance_id =
        offboard_session_admission_.current_producer_instance_id;
    if (target_offboard_instance_id == 0U) {
      return publication;
    }

    revocation.header.stamp = now();
    revocation.header.frame_id = frame_id_;
    revocation.producer_instance_id = execution_horizon_producer_instance_id_;
    revocation.target_offboard_instance_id = target_offboard_instance_id;
    revocation.sequence = execution_horizon_sequence_ + 1U;
    revocation.valid_from =
        production_mppi_execution_detail::timeFromNanoseconds(now_ns);
    revocation.valid_until = revocation.valid_from;
    revocation.execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED;
    revocation.execution_reason = static_cast<std::uint8_t>(reason);
    if (assessExecutionHorizonPayload(
            revocation,
            ExecutionHorizonPayloadValidationConfig{.expected_frame_id = frame_id_}) !=
        ExecutionHorizonPayloadStatus::kValid) {
      return publication;
    }

    const ExecutionRoutePublicationStatus3D snapshot_status =
        transition_required ? execution_route_store_.publish(expected, transition)
        : execution_route_store_.snapshot() == expected
            ? ExecutionRoutePublicationStatus3D::kPublished
            : ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
    if (snapshot_status != ExecutionRoutePublicationStatus3D::kPublished) {
      return publication;
    }
    execution_horizon_sequence_ = revocation.sequence;
    applied_control_ = {};
    execution_horizon_owner_ = {};
    execution_horizon_pub_->publish(revocation);
    publication.published = true;
  }
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                       "EXECUTION_HORIZON revoked=true snapshot_version=%" PRIu64
                       " owner_epoch=%" PRIu64 " sequence=%" PRIu64 " reason=%s",
                       transition_required ? transition.next->version
                                           : expected->version,
                       transition_required ? transition.next->execution_owner_epoch
                                           : expected->execution_owner_epoch,
                       revocation.sequence, productionMppiExecutionReasonName(reason));
  return publication;
}

bool ProductionMppiNode::handleRequestedExecutionRevocation(const std::int64_t now_ns) {
  const std::uint64_t requested_revocation =
      requested_execution_revocation_.load(std::memory_order_acquire);
  if (requested_revocation == handled_execution_revocation_request_) {
    return false;
  }

  const auto requested_reason = static_cast<ProductionMppiExecutionReason>(
      requested_revocation & kExecutionRevocationReasonMask);
  const ProductionMppiExecutionPublication revocation =
      publishExecutionRevocation(requested_reason, now_ns);
  if (revocation.published) {
    handled_execution_revocation_request_ = requested_revocation;
    return true;
  }

  bool revocation_already_satisfied{false};
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_};
    const std::shared_ptr<const ExecutionRouteSnapshot3D> snapshot =
        execution_route_store_.snapshot();
    const bool snapshot_has_executable_authority =
        snapshot != nullptr && (snapshot->finite_execution.has_value() ||
                                snapshot->direct_tracking_execution.has_value() ||
                                snapshot->stationary_hold.has_value());
    revocation_already_satisfied =
        !snapshot_has_executable_authority && !execution_horizon_owner_.valid;
  }
  if (revocation_already_satisfied) {
    handled_execution_revocation_request_ = requested_revocation;
  }
  return true;
}

void ProductionMppiNode::publishFailClosedExecutionRevocation(
    const ProductionMppiExecutionReason reason, const std::int64_t now_ns) {
  const bool snapshot_execution_enabled =
      use_static_map_ ||
      no_static_world_model_ == ProductionNoStaticWorldModel::kObservedOccupancy3D;
  if (!snapshot_execution_enabled) {
    return;
  }
  const std::shared_ptr<const ExecutionRouteSnapshot3D> snapshot =
      execution_route_store_.snapshot();
  const bool authority_present =
      snapshot != nullptr && (snapshot->phase == ExecutionRoutePhase3D::kRevoked ||
                              snapshot->finite_execution.has_value() ||
                              snapshot->direct_tracking_execution.has_value() ||
                              snapshot->stationary_hold.has_value());
  if (authority_present) {
    static_cast<void>(publishExecutionRevocation(reason, now_ns));
  }
}

void ProductionMppiNode::requestExecutionRevocation(
    const ProductionMppiExecutionReason reason) noexcept {
  if (!failClosedExecutionReason(reason)) {
    return;
  }
  const std::uint64_t encoded_reason = static_cast<std::uint8_t>(reason);
  std::uint64_t current =
      requested_execution_revocation_.load(std::memory_order_relaxed);
  while (current <=
         std::numeric_limits<std::uint64_t>::max() - kExecutionRevocationRequestStep) {
    const std::uint64_t desired = ((current & ~kExecutionRevocationReasonMask) +
                                   kExecutionRevocationRequestStep) |
                                  encoded_reason;
    if (requested_execution_revocation_.compare_exchange_weak(
            current, desired, std::memory_order_release, std::memory_order_relaxed)) {
      return;
    }
  }
}

ProductionMppiExecutionPublication
ProductionMppiNode::publishExplicitHold(const ProductionMppiExecutionCycle& cycle,
                                        const Point3& hold_position,
                                        const ProductionMppiExecutionReason reason) {
  if (!cycle.snapshot_owner_required) {
    legacy_execution_arbiter_.leaveNoExecutableHold();
  }
  return publishPositionHold(
      cycle, hold_position, reason,
      ProductionMppiHoldOwnershipTransition3D::kExplicitTransfer);
}

} // namespace drone_city_nav
