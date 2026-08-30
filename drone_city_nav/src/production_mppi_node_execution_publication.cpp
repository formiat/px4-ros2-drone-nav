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

#include "execution_publication_navigation_rebase_3d.hpp"
#include "production_mppi_node_execution_internal.hpp"
#include "production_mppi_node_planning_tick_rearm.hpp"

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

[[nodiscard]] bool progressPreparationPreservesCertifiedRouteEvidence(
    const ExecutionRouteSnapshot3D& expected,
    const ExecutionRouteSnapshot3D& prepared) noexcept {
  if (!expected.route.has_value() || !prepared.route.has_value()) {
    return false;
  }
  const CertifiedRouteSuffix3D& source = *expected.route;
  const CertifiedRouteSuffix3D& next = *prepared.route;
  return source.route_instance_id == next.route_instance_id &&
         source.owner.id == next.owner.id &&
         source.identity.generation == next.identity.generation &&
         source.geometry == next.geometry &&
         source.continuity_id == next.continuity_id &&
         source.observed_raw_world == next.observed_raw_world &&
         source.static_world == next.static_world &&
         source.validation_policy == next.validation_policy &&
         source.planned_endpoint_semantics == next.planned_endpoint_semantics;
}

struct FiniteExecutionEvidenceView {
  const mppi::FiniteHorizon* horizon{nullptr};
  const VersionedExecutionInput3D* execution_input{nullptr};
  const VersionedExecutionValidationPolicy3D* policy{nullptr};
  const VersionedStaticWorld3D* static_world{nullptr};
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  std::int64_t control_interval_ns{0};
};

template<typename Execution>
[[nodiscard]] std::optional<FiniteExecutionEvidenceView>
finiteExecutionArtifactEvidenceView(const Execution& execution) noexcept {
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
      .valid_from_ns = execution.valid_from_ns,
      .valid_until_ns = execution.valid_until_ns,
      .control_interval_ns = execution.control_interval_ns,
  };
}

[[nodiscard]] std::vector<mppi::TimedExecutionPathPoint>
timedExecutionPathPoints(const FiniteExecutionEvidenceView& view) {
  std::vector<mppi::TimedExecutionPathPoint> points;
  if (view.horizon == nullptr || view.execution_input == nullptr ||
      view.control_interval_ns <= 0 || view.horizon->controls.empty() ||
      view.horizon->states.size() != view.horizon->controls.size() + 1U) {
    return points;
  }
  const double step_s = static_cast<double>(view.control_interval_ns) * 1.0e-9;
  if (!std::isfinite(step_s) || step_s <= 0.0) {
    return points;
  }
  points.reserve(view.horizon->states.size());
  for (std::size_t index = 0U; index < view.horizon->states.size(); ++index) {
    points.push_back(mppi::TimedExecutionPathPoint{
        .time_from_start_s = static_cast<double>(index) * step_s,
        .state = view.horizon->states[index],
        .control = index == 0U ? view.execution_input->previousControl()
                               : view.horizon->controls[index - 1U],
    });
  }
  return points;
}

[[nodiscard]] std::optional<FiniteExecutionEvidenceView>
finiteExecutionEvidenceView(const ExecutionRouteSnapshot3D& snapshot) noexcept {
  if (snapshot.finite_execution.has_value()) {
    return finiteExecutionArtifactEvidenceView(*snapshot.finite_execution);
  }
  if (snapshot.direct_tracking_execution.has_value()) {
    return finiteExecutionArtifactEvidenceView(*snapshot.direct_tracking_execution);
  }
  return std::nullopt;
}

[[nodiscard]] bool revalidateFiniteExecutionAgainstLatestEvidence(
    const FiniteExecutionEvidenceView& view,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& latest_raw,
    const std::shared_ptr<const VersionedLatestLidarEvidence3D>&
        latest_lidar) noexcept {
  if (view.horizon == nullptr || view.execution_input == nullptr ||
      view.policy == nullptr || !view.policy->valid() || latest_lidar == nullptr ||
      !latest_lidar->valid() || view.control_interval_ns <= 0 ||
      view.horizon->states.size() != view.horizon->controls.size() + 1U ||
      view.horizon->controls.empty()) {
    return false;
  }
  const bool static_world = view.static_world != nullptr;
  if ((!static_world && (latest_raw == nullptr || !latest_raw->valid())) ||
      (static_world && !view.static_world->valid())) {
    return false;
  }
  const std::vector<mppi::TimedExecutionPathPoint> points =
      timedExecutionPathPoints(view);
  if (points.empty()) {
    return false;
  }
  const std::optional<LaunchSupportContact3D>& launch_support =
      !static_world ? latest_raw->launchSupportContact()
                    : std::optional<LaunchSupportContact3D>{};
  const mppi::FiniteExecutionPathWorld world{
      .flight_envelope = &view.policy->flightEnvelope(),
      .dynamics = &view.policy->dynamics(),
      .altitude_envelope = &view.policy->altitudeEnvelope(),
      .footprint = &view.policy->sweptFootprint(),
      .static_occupancy = static_world ? &view.static_world->occupancy() : nullptr,
      .observed_occupancy = !static_world ? &latest_raw->occupancy() : nullptr,
      .launch_support_contact =
          launch_support ? std::addressof(*launch_support) : nullptr,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = latest_lidar->hitPointsMapM(),
      .terminal_boundary = std::nullopt,
  };
  return mppi::validateCompleteFiniteExecutionPath(
             points, view.execution_input->previousControl(), world)
      .accepted();
}

[[nodiscard]] bool revalidateFiniteExecutionAgainstLatestEvidence(
    const ExecutionRouteSnapshot3D& snapshot,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& latest_raw,
    const std::shared_ptr<const VersionedLatestLidarEvidence3D>&
        latest_lidar) noexcept {
  if (snapshot.finite_execution.has_value()) {
    if (!snapshot.braking_fallback.has_value()) {
      return false;
    }
    const std::optional<FiniteExecutionEvidenceView> command_view =
        finiteExecutionArtifactEvidenceView(*snapshot.finite_execution);
    const std::optional<FiniteExecutionEvidenceView> braking_view =
        finiteExecutionArtifactEvidenceView(*snapshot.braking_fallback);
    return command_view.has_value() && braking_view.has_value() &&
           revalidateFiniteExecutionAgainstLatestEvidence(*command_view, latest_raw,
                                                          latest_lidar) &&
           revalidateFiniteExecutionAgainstLatestEvidence(*braking_view, latest_raw,
                                                          latest_lidar);
  }
  if (!snapshot.direct_tracking_execution.has_value()) {
    return false;
  }
  const std::optional<FiniteExecutionEvidenceView> direct_view =
      finiteExecutionArtifactEvidenceView(*snapshot.direct_tracking_execution);
  return direct_view.has_value() && revalidateFiniteExecutionAgainstLatestEvidence(
                                        *direct_view, latest_raw, latest_lidar);
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
  horizon.route_target.x = cycle.input.target.x;
  horizon.route_target.y = cycle.input.target.y;
  horizon.route_target.z = cycle.input.target.z;
  horizon.route_constrained = cycle.publication_route_constrained;
  return horizon;
}

ProductionMppiHorizonCommitStatus ProductionMppiNode::commitAndPublishExecutionHorizon(
    const ProductionMppiExecutionCycle& cycle,
    const msg::MppiTrajectoryHorizon& horizon,
    const ProductionMppiHorizonCommit& commit) {
  using production_mppi_execution_detail::sameControl;
  using production_mppi_execution_detail::timeToNanoseconds;
  const auto report_commit_failure = [this](const char* const stage) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "EXECUTION_HORIZON_COMMIT committed=false stage=%s", stage);
  };

  msg::MppiTrajectoryHorizon publication_horizon = horizon;
  std::shared_ptr<const VersionedExecutionInput3D> publication_execution_input =
      cycle.execution_input;
  std::optional<ExecutionRouteTransitionResult3D> rebased_transition;
  ProductionMppiHorizonCommit publication_commit = commit;
  if (publication_execution_input == nullptr || !publication_execution_input->valid() ||
      assessExecutionHorizonPayload(publication_horizon,
                                    ExecutionHorizonPayloadValidationConfig{
                                        .expected_frame_id = frame_id_,
                                        .flight_envelope = &flight_envelope_config_,
                                    }) != ExecutionHorizonPayloadStatus::kValid) {
    report_commit_failure("invalid_input_or_payload");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  ProductionMppiExecutionHorizonOwner owner{
      .route_target =
          Point3{publication_horizon.route_target.x, publication_horizon.route_target.y,
                 publication_horizon.route_target.z},
      .stationary_hold_position =
          Point3{publication_horizon.stationary_hold_position.x,
                 publication_horizon.stationary_hold_position.y,
                 publication_horizon.stationary_hold_position.z},
      .valid_from_ns = timeToNanoseconds(publication_horizon.valid_from),
      .valid_until_ns = timeToNanoseconds(publication_horizon.valid_until),
      .producer_instance_id = publication_horizon.producer_instance_id,
      .target_offboard_instance_id = publication_horizon.target_offboard_instance_id,
      .sequence = publication_horizon.sequence,
      .execution_mode = publication_horizon.execution_mode,
      .execution_reason = publication_horizon.execution_reason,
      .stationary_position_hold = publication_horizon.stationary_position_hold,
  };
  owner.valid =
      owner.producer_instance_id == execution_horizon_producer_instance_id_ &&
      owner.sequence != 0U && owner.valid_from_ns > 0 &&
      owner.valid_until_ns > owner.valid_from_ns &&
      owner.target_offboard_instance_id != 0U &&
      owner.execution_mode <= msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD;
  if (!owner.valid) {
    report_commit_failure("invalid_owner");
    return ProductionMppiHorizonCommitStatus::kRejected;
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
    report_commit_failure("vehicle_status_not_authoritative");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  std::shared_ptr<const ProductionMppiRawWorld3D> committed_3d;
  if (!use_static_map_) {
    const double maximum_observation_age_ms =
        maximum_esdf_age_ms_ + stale_esdf_execution_window_ms_;
    committed_3d = latest_raw_world_3d_.load(std::memory_order_acquire);
    const bool committed_world_current =
        committed_3d != nullptr &&
        committed_3d->version.producer_instance_id ==
            cycle.esdf.world->producer_instance_id &&
        committedRawWorldAgeMs(committed_3d.get(), publication_now_ns) <=
            maximum_observation_age_ms;
    if (raw_world_identity_conflicted_ || !committed_world_current) {
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
      report_commit_failure("raw_world_not_current");
      return ProductionMppiHorizonCommitStatus::kRejected;
    }
  }
  if (publication_now_ns < owner.valid_from_ns ||
      publication_now_ns >= owner.valid_until_ns) {
    report_commit_failure("horizon_time_window_not_current");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  const OffboardSessionPublicationCurrentnessStatus offboard_currentness =
      assessOffboardSessionPublicationCurrentness(
          offboard_session_admission_, offboard_session_receive_stamp_ns_,
          cycle.offboard_session, cycle.offboard_session_receive_stamp_ns,
          owner.target_offboard_instance_id, publication_now_ns,
          maximum_control_feedback_age_ms_);
  const char* const cycle_currentness_failure = [&]() -> const char* {
    if (requested_execution_revocation_.load(std::memory_order_acquire) !=
        handled_execution_revocation_request_) {
      return "revocation_epoch_changed";
    }
    if (navigation_objective_.load(std::memory_order_acquire) != cycle.objective) {
      return "objective_changed";
    }
    if (!navigation_.valid) {
      return "navigation_invalid";
    }
    if (offboard_currentness != OffboardSessionPublicationCurrentnessStatus::kCurrent) {
      return offboardSessionPublicationCurrentnessStatusName(offboard_currentness);
    }
    return nullptr;
  }();
  if (cycle_currentness_failure != nullptr) {
    report_commit_failure(cycle_currentness_failure);
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  const bool navigation_advanced =
      navigation_.revision != publication_execution_input->poseRevision() ||
      navigation_.source_timestamp_us !=
          publication_execution_input->poseSourceTimestampUs() ||
      navigation_.receive_stamp_ns != publication_execution_input->poseReceiveStampNs();
  const bool previous_control_evidence_current = [&]() noexcept {
    switch (publication_execution_input->previousControlSource()) {
      case ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback:
        return appliedControlAuthoritativeForExecution(
                   applied_control_, execution_horizon_owner_, publication_now_ns,
                   maximum_control_feedback_age_ms_) &&
               applied_control_.horizon_producer_instance_id ==
                   publication_execution_input
                       ->previousControlSourceProducerInstanceId() &&
               applied_control_.horizon_sequence ==
                   publication_execution_input->previousControlSourceSequence() &&
               applied_control_.source_stamp_ns ==
                   publication_execution_input->previousControlSourceStampNs() &&
               applied_control_.receive_stamp_ns ==
                   publication_execution_input->previousControlReceiveStampNs() &&
               sameControl(applied_control_.control,
                           publication_execution_input->previousControl());
      case ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration:
        return navigation_.measured_acceleration_valid &&
               navigation_.source_timestamp_us ==
                   publication_execution_input->previousControlSourceSequence() &&
               navigation_.receive_stamp_ns ==
                   publication_execution_input->previousControlSourceStampNs() &&
               navigation_.receive_stamp_ns ==
                   publication_execution_input->previousControlReceiveStampNs() &&
               sameControl(navigation_.measured_equivalent_control,
                           publication_execution_input->previousControl());
      case ExecutionPreviousControlEvidenceSource3D::kAssumedZero:
        // Stationary capture has its own exact empty-owner commit contract below.
        return true;
      case ExecutionPreviousControlEvidenceSource3D::kUnknown:
      case ExecutionPreviousControlEvidenceSource3D::kEngineFallback:
        return false;
    }
    return false;
  }();
  const bool execution_input_advanced =
      navigation_advanced || !previous_control_evidence_current;
  if (execution_input_advanced) {
    bool late_rebase_candidate_rejected{false};
    std::size_t late_rebase_source_control_index{0U};
    const auto rebase_for_current_navigation = [&]() -> const char* {
      const bool planned_snapshot_transition =
          publication_horizon.execution_mode ==
              msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED &&
          (commit.kind == ProductionMppiHorizonCommitKind::kPublishSnapshotTransition ||
           commit.kind ==
               ProductionMppiHorizonCommitKind::kCommitPendingSnapshotTransition) &&
          commit.expected_snapshot != nullptr && commit.transition != nullptr &&
          commit.transition->applied() && commit.transition->next != nullptr;
      if (!planned_snapshot_transition) {
        return "navigation_advanced_without_rebase_contract";
      }
      if (execution_input_capture_sequence_ ==
          std::numeric_limits<std::uint64_t>::max()) {
        return "late_rebase_input_sequence_exhausted";
      }
      const ProductionMppiExecutionInputPreparation current_input_preparation =
          prepareExecutionInputForPlanningTick(
              navigation_, applied_control_, execution_horizon_owner_,
              ++execution_input_capture_sequence_, publication_now_ns,
              maximum_control_feedback_age_ms_, false, false);
      const std::shared_ptr<const VersionedExecutionInput3D>& current_input =
          current_input_preparation.execution_input;
      if (!current_input_preparation.previous_control_available ||
          current_input == nullptr || !current_input->valid() ||
          !current_input->nominalStateAuthoritative()) {
        return "late_rebase_execution_input_unavailable";
      }

      const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
          latest_lidar_evidence_.load(std::memory_order_acquire);
      const std::shared_ptr<const VersionedObservedRawWorld3D> current_raw =
          committed_3d != nullptr ? committed_3d->execution_owner : nullptr;
      ExecutionPublicationNavigationRebaseResult3D rebase =
          rebaseExecutionPublicationForCurrentNavigation3D(
              ExecutionPublicationNavigationRebaseRequest3D{
                  .expected_snapshot = commit.expected_snapshot.get(),
                  .certification_snapshot = commit.certification_snapshot.get(),
                  .progress_preparation = commit.progress_preparation.get(),
                  .candidate_snapshot = commit.transition->next.get(),
                  .expected_pending = commit.expected_pending.get(),
                  .lifecycle_event =
                      cycle.route_execution.lifecycle_event.has_value()
                          ? std::addressof(*cycle.route_execution.lifecycle_event)
                          : nullptr,
                  .current_execution_input = current_input,
                  .current_lidar_evidence = current_lidar,
                  .current_observed_raw_world = current_raw,
                  .publication_now_ns = publication_now_ns,
                  .arrival_search_step_controls = cycle.arrival_search_step_controls,
                  .finite_horizon_config = &finite_horizon_config_,
                  .terminal_boundary = cycle.execution_path_world.terminal_boundary,
              });
      late_rebase_source_control_index = rebase.source_control_index;
      if (!rebase.rebased() || !rebase.transition.has_value() ||
          rebase.transition->next == nullptr) {
        late_rebase_candidate_rejected = true;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "EXECUTION_HORIZON_REBASE rebased=false status=%s path_validation=%s "
            "route_certification=%.*s route_adherence=%.*s "
            "source_control_index=%zu route_adherence_state_index=%zu "
            "route_adherence_failure_distance_m=%.3f transition=%.*s",
            executionPublicationNavigationRebaseStatus3DName(rebase.status),
            mppi::finiteExecutionPathStatusName(rebase.path_validation_status),
            static_cast<int>(finiteExecutionCertificationStatus3DName(
                                 rebase.route_certification_status)
                                 .size()),
            finiteExecutionCertificationStatus3DName(rebase.route_certification_status)
                .data(),
            static_cast<int>(
                finiteExecutionRouteAdherenceStatus3DName(rebase.route_adherence_status)
                    .size()),
            finiteExecutionRouteAdherenceStatus3DName(rebase.route_adherence_status)
                .data(),
            rebase.source_control_index, rebase.route_adherence_failure_state_index,
            rebase.route_adherence_failure_distance_m,
            static_cast<int>(
                executionRouteTransitionStatus3DName(rebase.transition_status).size()),
            executionRouteTransitionStatus3DName(rebase.transition_status).data());
        return executionPublicationNavigationRebaseStatus3DName(rebase.status);
      }
      ExecutionRouteTransitionResult3D current_transition =
          std::move(*rebase.transition);

      const std::optional<FiniteExecutionEvidenceView> rebased_view =
          finiteExecutionEvidenceView(*current_transition.next);
      if (!rebased_view.has_value() || rebased_view->horizon == nullptr ||
          rebased_view->execution_input == nullptr) {
        return "late_rebase_transition_invalid";
      }
      publication_horizon.points.clear();
      publication_horizon.valid_from =
          production_mppi_execution_detail::timeFromNanoseconds(publication_now_ns);
      const std::int64_t rebased_valid_until_ns = rebased_view->valid_until_ns;
      publication_horizon.valid_until =
          production_mppi_execution_detail::timeFromNanoseconds(rebased_valid_until_ns);
      publication_horizon.pose_revision = navigation_.revision;
      publication_horizon.control_interval_ns = rebased_view->control_interval_ns;
      if (!production_mppi_execution_detail::appendFiniteExecutionPoints(
              publication_horizon, rebased_view->horizon->states,
              rebased_view->horizon->controls, current_input->previousControl(),
              rebased_view->control_interval_ns) ||
          assessExecutionHorizonPayload(publication_horizon,
                                        ExecutionHorizonPayloadValidationConfig{
                                            .expected_frame_id = frame_id_,
                                            .flight_envelope = &flight_envelope_config_,
                                        }) != ExecutionHorizonPayloadStatus::kValid) {
        return "late_rebase_payload_invalid";
      }
      publication_execution_input = current_input;
      rebased_transition.emplace(std::move(current_transition));
      publication_commit.transition = std::addressof(*rebased_transition);
      owner.valid_from_ns = publication_now_ns;
      owner.valid_until_ns = rebased_valid_until_ns;
      return nullptr;
    };
    if (const char* const rebase_failure = rebase_for_current_navigation();
        rebase_failure != nullptr) {
      report_commit_failure(rebase_failure);
      const ExecutionRouteSnapshot3D* const retained_candidate =
          commit.transition != nullptr && commit.transition->next != nullptr
              ? commit.transition->next.get()
              : nullptr;
      const bool retained_execution =
          retained_candidate != nullptr &&
          ((retained_candidate->finite_execution.has_value() &&
            retained_candidate->finite_execution->kind ==
                FiniteExecutionKind3D::kRetained) ||
           (retained_candidate->direct_tracking_execution.has_value() &&
            retained_candidate->direct_tracking_execution->kind ==
                FiniteExecutionKind3D::kRetained));
      const bool resident_owner_witnessed = appliedControlAuthoritativeForExecution(
          applied_control_, execution_horizon_owner_, publication_now_ns,
          maximum_control_feedback_age_ms_);
      const bool no_revocation_pending =
          requested_execution_revocation_.load(std::memory_order_acquire) ==
          handled_execution_revocation_request_;
      const bool exact_resident_snapshot =
          commit.expected_snapshot != nullptr &&
          execution_route_store_.snapshot() == commit.expected_snapshot;
      const bool resident_execution_owner_matches =
          commit.expected_snapshot != nullptr &&
          execution_horizon_owner_.snapshot_execution_owner_epoch ==
              commit.expected_snapshot->execution_owner_epoch;
      const bool resident_owner_may_continue =
          canContinueResidentPlannedOwner(ProductionMppiResidentOwnerContinuationCheck{
              .owner = &execution_horizon_owner_,
              .now_ns = publication_now_ns,
              .retained_candidate = retained_execution,
              .exact_snapshot_current = exact_resident_snapshot,
              .execution_owner_matches = resident_execution_owner_matches,
              .revocation_pending = !no_revocation_pending,
              .owner_witnessed = resident_owner_witnessed,
          });
      if (late_rebase_candidate_rejected && resident_owner_may_continue) {
        // The retained replacement lost only the final navigation race. The
        // exact resident snapshot is still owned by a current, witnessed wire
        // lease, so leave that immutable lease in force and retry next tick.
        // No new snapshot or horizon is published on this disposition.
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "EXECUTION_HORIZON_COMMIT committed=false deferred=true "
                             "stage=%s resident_sequence=%" PRIu64,
                             rebase_failure, execution_horizon_owner_.sequence);
        return ProductionMppiHorizonCommitStatus::kDeferredResidentOwner;
      }
      return ProductionMppiHorizonCommitStatus::kRejected;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_HORIZON_COMMIT late_rebase=true source_pose_revision=%" PRIu64
        " publication_pose_revision=%" PRIu64
        " source_control_index=%zu control_evidence_advanced=%s",
        cycle.execution_input->poseRevision(),
        publication_execution_input->poseRevision(), late_rebase_source_control_index,
        previous_control_evidence_current ? "false" : "true");
  }
  const ExecutionRouteSnapshot3D* publication_snapshot{nullptr};
  if ((publication_commit.kind ==
           ProductionMppiHorizonCommitKind::kPublishSnapshotTransition ||
       publication_commit.kind ==
           ProductionMppiHorizonCommitKind::kCommitPendingSnapshotTransition) &&
      publication_commit.transition != nullptr &&
      publication_commit.transition->applied() &&
      publication_commit.transition->next != nullptr) {
    publication_snapshot = publication_commit.transition->next.get();
  } else if (publication_commit.kind ==
                 ProductionMppiHorizonCommitKind::kConfirmSnapshotUnchanged &&
             publication_commit.expected_snapshot != nullptr) {
    publication_snapshot = publication_commit.expected_snapshot.get();
  } else {
    report_commit_failure("invalid_snapshot_commit_contract");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  owner.snapshot_execution_owner_epoch = publication_snapshot->execution_owner_epoch;
  const std::shared_ptr<const VersionedExecutionValidationPolicy3D>
      snapshot_publication_policy = snapshotValidationPolicy(*publication_snapshot);
  const VersionedExecutionValidationPolicy3D* const publication_policy =
      snapshot_publication_policy.get();
  if (publication_policy == nullptr || !publication_policy->valid() ||
      !executionInputFreshAt(*publication_execution_input, *publication_policy,
                             publication_now_ns)) {
    requestExecutionRevocation(ProductionMppiExecutionReason::kNoExecutableHorizon);
    report_commit_failure("execution_input_not_fresh");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> publication_lidar =
      snapshotLidarOwner(*publication_snapshot);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      latest_lidar_evidence_.load(std::memory_order_acquire);
  if (latest_lidar_evidence_identity_conflicted_.load(std::memory_order_acquire) ||
      publication_lidar == nullptr || current_lidar == nullptr ||
      ((publication_lidar->evidenceId() != current_lidar->evidenceId() ||
        publication_lidar->contentFingerprint() !=
            current_lidar->contentFingerprint()) &&
       !publication_commit.latest_evidence_revalidated) ||
      (publication_policy->latestLidarFreshnessRequired() &&
       !assessLatestLidarEvidenceFreshness3D(
            *publication_lidar, publication_now_ns,
            publication_policy->latestLidarMaximumAgeMs())
            .fresh)) {
    requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    report_commit_failure("lidar_evidence_not_current");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  const StationaryExecutionHold3D* const capture_hold =
      publication_commit.transition != nullptr &&
              publication_commit.transition->next != nullptr &&
              publication_commit.transition->next->stationary_hold.has_value()
          ? std::addressof(*publication_commit.transition->next->stationary_hold)
          : nullptr;
  const bool stationary_capture_rearm_commit =
      cycle.planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold &&
      publication_execution_input->stationaryCaptureStateAuthoritative() &&
      publication_commit.kind ==
          ProductionMppiHorizonCommitKind::kPublishSnapshotTransition &&
      publication_commit.expected_snapshot != nullptr &&
      publication_commit.expected_snapshot->phase == ExecutionRoutePhase3D::kRevoked &&
      publication_commit.transition != nullptr &&
      publication_commit.transition->applied() && capture_hold != nullptr &&
      capture_hold->origin ==
          StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm &&
      capture_hold->terminal_execution_input == publication_execution_input &&
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
                                                  publication_execution_input->state());
  bool control_evidence_current{false};
  if (publication_execution_input->previousControlSource() ==
      ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback) {
    control_evidence_current =
        appliedControlAuthoritativeForExecution(
            applied_control_, execution_horizon_owner_, publication_now_ns,
            maximum_control_feedback_age_ms_) &&
        applied_control_.horizon_producer_instance_id ==
            publication_execution_input->previousControlSourceProducerInstanceId() &&
        applied_control_.horizon_sequence ==
            publication_execution_input->previousControlSourceSequence() &&
        applied_control_.source_stamp_ns ==
            publication_execution_input->previousControlSourceStampNs() &&
        applied_control_.receive_stamp_ns ==
            publication_execution_input->previousControlReceiveStampNs() &&
        sameControl(applied_control_.control,
                    publication_execution_input->previousControl());
  } else if (publication_execution_input->previousControlSource() ==
             ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration) {
    control_evidence_current =
        navigation_.measured_acceleration_valid &&
        navigation_.source_timestamp_us ==
            publication_execution_input->previousControlSourceSequence() &&
        navigation_.receive_stamp_ns ==
            publication_execution_input->previousControlSourceStampNs() &&
        navigation_.receive_stamp_ns ==
            publication_execution_input->previousControlReceiveStampNs() &&
        sameControl(navigation_.measured_equivalent_control,
                    publication_execution_input->previousControl());
  } else if (publication_execution_input->previousControlSource() ==
             ExecutionPreviousControlEvidenceSource3D::kAssumedZero) {
    control_evidence_current = stationary_capture_rearm_commit;
  }
  if (!control_evidence_current) {
    report_commit_failure("control_evidence_not_current");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }

  bool owner_committed{false};
  switch (publication_commit.kind) {
    case ProductionMppiHorizonCommitKind::kPublishSnapshotTransition:
      owner_committed =
          publication_commit.expected_snapshot != nullptr &&
          publication_commit.transition != nullptr &&
          execution_route_store_.publish(publication_commit.expected_snapshot,
                                         *publication_commit.transition) ==
              ExecutionRoutePublicationStatus3D::kPublished;
      break;
    case ProductionMppiHorizonCommitKind::kConfirmSnapshotUnchanged:
      owner_committed =
          publication_commit.expected_snapshot != nullptr &&
          execution_route_store_.snapshot() == publication_commit.expected_snapshot;
      break;
    case ProductionMppiHorizonCommitKind::kCommitPendingSnapshotTransition:
      owner_committed =
          publication_commit.expected_snapshot != nullptr &&
          publication_commit.transition != nullptr &&
          publication_commit.expected_pending != nullptr &&
          pending_certified_route_mailbox_.commitExecutionIfSame(
              publication_commit.expected_pending, execution_route_store_,
              publication_commit.expected_snapshot, *publication_commit.transition);
      break;
  }
  if (!owner_committed) {
    report_commit_failure("snapshot_owner_commit_rejected");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  applied_control_ = {};
  execution_horizon_owner_ = owner;
  execution_horizon_pub_->publish(publication_horizon);
  RCLCPP_INFO(get_logger(),
              "EXECUTION_HORIZON published=true producer=%" PRIu64 " sequence=%" PRIu64
              " mode=%s",
              publication_horizon.producer_instance_id, publication_horizon.sequence,
              publication_horizon.execution_mode ==
                      msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED
                  ? "planned"
                  : "non_planned");
  return ProductionMppiHorizonCommitStatus::kPublished;
}

ProductionMppiHorizonCommitStatus ProductionMppiNode::commitExecutionSnapshotHorizon(
    const ProductionMppiExecutionCycle& cycle,
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& expected,
    const ExecutionRouteTransitionResult3D& transition,
    const msg::MppiTrajectoryHorizon& horizon,
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& certification_snapshot,
    const std::shared_ptr<const ExecutionRouteTransitionResult3D>&
        progress_preparation) {
  const bool prepared_progress_valid =
      progress_preparation != nullptr && certification_snapshot != nullptr &&
      progress_preparation->applied() &&
      progress_preparation->predecessor == expected.get() &&
      progress_preparation->next == certification_snapshot;
  const bool certification_base_valid = progress_preparation != nullptr
                                            ? prepared_progress_valid
                                            : certification_snapshot == expected;
  if (expected == nullptr || !certification_base_valid || !transition.applied() ||
      transition.predecessor != expected.get() || transition.next == nullptr ||
      !transition.next->publishable()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_SNAPSHOT_COMMIT committed=false stage=invalid_transition");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  const std::shared_ptr<const VersionedObservedRawWorld3D> expected_raw =
      snapshotRawOwner(*transition.next);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> expected_lidar =
      snapshotLidarOwner(*transition.next);
  const std::shared_ptr<const VersionedExecutionValidationPolicy3D> policy =
      snapshotValidationPolicy(*transition.next);
  if (expected_lidar == nullptr || policy == nullptr || !policy->valid()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "EXECUTION_SNAPSHOT_COMMIT committed=false "
                         "stage=missing_lidar_or_validation_policy lidar_present=%s "
                         "policy_present=%s",
                         expected_lidar != nullptr ? "true" : "false",
                         policy != nullptr ? "true" : "false");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }

  const std::scoped_lock evidence_lock{execution_evidence_commit_mutex_,
                                       latest_lidar_evidence_commit_mutex_};
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
          .lidar_freshness_required = policy->latestLidarFreshnessRequired(),
      });
  // Progress may advance while the immutable route geometry and its certificate
  // world remain unchanged. In that case a newer raw/lidar sample only needs to
  // revalidate the two finite execution paths; it is not a reason to discard the
  // route or retry the entire route suffix against every memory revision.
  const bool progress_preserves_certified_route_evidence =
      progress_preparation == nullptr ||
      (certification_snapshot != nullptr &&
       progressPreparationPreservesCertifiedRouteEvidence(*expected,
                                                          *certification_snapshot));
  const bool latest_evidence_revalidated =
      progress_preserves_certified_route_evidence &&
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
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  const ProductionMppiHorizonCommit commit{
      .kind = expected_pending != nullptr
                  ? ProductionMppiHorizonCommitKind::kCommitPendingSnapshotTransition
                  : ProductionMppiHorizonCommitKind::kPublishSnapshotTransition,
      .expected_snapshot = expected,
      .certification_snapshot = certification_snapshot,
      .progress_preparation = progress_preparation,
      .transition = &transition,
      .expected_pending = expected_pending,
      .latest_evidence_revalidated = latest_evidence_revalidated,
  };
  return commitAndPublishExecutionHorizon(cycle, horizon, commit);
}

} // namespace drone_city_nav
