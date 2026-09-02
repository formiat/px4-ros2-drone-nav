#include "drone_city_nav/execution_horizon_contract_ros.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"

#include <atomic>
#include <chrono>
#include <cinttypes>
#include <limits>
#include <memory>
#include <mutex>
#include <span>
#include <string_view>
#include <utility>

#include "production_mppi_node_execution_internal.hpp"
#include "raw_world_ingress_ros_3d.hpp"

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

[[nodiscard]] const char* executionControlEvidenceSourceName(
    const ExecutionPreviousControlEvidenceSource3D source) noexcept {
  switch (source) {
    case ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback:
      return "offboard_feedback";
    case ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration:
      return "measured_acceleration";
    case ExecutionPreviousControlEvidenceSource3D::kAssumedZero:
      return "assumed_zero";
    case ExecutionPreviousControlEvidenceSource3D::kEngineFallback:
      return "engine_fallback";
    case ExecutionPreviousControlEvidenceSource3D::kUnknown:
      return "unknown";
  }
  return "unknown";
}

[[nodiscard]] double evidenceAgeMs(const std::int64_t now_ns,
                                   const std::int64_t receive_stamp_ns) noexcept {
  if (now_ns <= 0 || receive_stamp_ns <= 0 || receive_stamp_ns > now_ns) {
    return -1.0;
  }
  return static_cast<double>(now_ns - receive_stamp_ns) * 1.0e-6;
}

} // namespace

msg::MppiTrajectoryHorizon
ProductionMppiNode::makeExecutionHorizon(const ProductionMppiExecutionCycle& cycle,
                                         const std::int64_t valid_until_ns,
                                         const ProductionMppiExecutionMode mode,
                                         const ProductionMppiExecutionReason reason) {
  msg::MppiTrajectoryHorizon horizon;
  horizon.header.stamp = now();
  horizon.header.frame_id = config_.world.frame_id;
  horizon.producer_instance_id = execution_horizon_producer_instance_id_;
  horizon.target_offboard_instance_id = cycle.evidence.target_offboard_instance_id;
  // The wire sequence is allocated by the commit that publishes this horizon.
  // A candidate that never commits must not consume a number the controller
  // will then never see.
  horizon.sequence = 0U;
  horizon.valid_from =
      production_mppi_execution_detail::timeFromNanoseconds(cycle.controller.now_ns);
  horizon.valid_until =
      production_mppi_execution_detail::timeFromNanoseconds(valid_until_ns);
  horizon.control_interval_ns = cycle.controller.finite_path_control_interval_ns;
  horizon.pose_revision = cycle.controller.inputRef().pose_revision;
  horizon.obstacle_revision = cycle.controller.latest_obstacle_revision;
  horizon.risk_tier =
      static_cast<std::uint8_t>(cycle.controller.resultRef().selected_tier);
  horizon.execution_mode = static_cast<std::uint8_t>(mode);
  horizon.execution_reason = static_cast<std::uint8_t>(reason);
  horizon.route_target.x = cycle.controller.inputRef().target.x;
  horizon.route_target.y = cycle.controller.inputRef().target.y;
  horizon.route_target.z = cycle.controller.inputRef().target.z;
  horizon.route_constrained = cycle.route.publication_route_constrained;
  return horizon;
}

ProductionMppiHorizonCommitStatus ProductionMppiNode::commitAndPublishExecutionHorizon(
    const ProductionMppiExecutionCycle& cycle,
    const msg::MppiTrajectoryHorizon& horizon,
    ExecutionHorizonLeaseCandidate3D candidate) {
  using production_mppi_execution_detail::timeToNanoseconds;
  const auto report_commit_failure = [this](const char* const stage) {
    horizon_commit_rejections_.fetch_add(1U, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "EXECUTION_HORIZON_COMMIT committed=false stage=%s", stage);
  };

  const std::shared_ptr<const VersionedExecutionInput3D>& publication_execution_input =
      cycle.evidence.execution_input;
  if (publication_execution_input == nullptr || !publication_execution_input->valid()) {
    report_commit_failure("invalid_execution_input");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  const auto input_lock = evidence_boundary_.input();
  if (execution_horizon_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    report_commit_failure("horizon_sequence_exhausted");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  msg::MppiTrajectoryHorizon publication_horizon = horizon;
  publication_horizon.sequence = execution_horizon_sequence_ + 1U;
  if (assessExecutionHorizonPayload(
          publication_horizon, ExecutionHorizonPayloadValidationConfig{
                                   .expected_frame_id = config_.world.frame_id,
                                   .flight_envelope = &config_.world.flight_envelope,
                               }) != ExecutionHorizonPayloadStatus::kValid) {
    report_commit_failure("invalid_payload");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  const ExecutionOwnerIdentity3D owner{
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
      .execution_mode =
          static_cast<ExecutionAuthorityMode3D>(publication_horizon.execution_mode),
      .execution_reason =
          static_cast<ExecutionAuthorityReason3D>(publication_horizon.execution_reason),
      .stationary_position_hold = publication_horizon.stationary_position_hold,
  };
  const std::shared_ptr<const CommittedExecutionAuthority3D>
      resident_execution_authority = execution_supervisor_.authority();
  if (isControlEvidenceOnlyAuthoritySuccessor3D(candidate.expected_authority,
                                                resident_execution_authority)) {
    // Feedback installed or cleared since capture does not change the plan,
    // lease, or input this candidate was prepared against.
    candidate.expected_authority = resident_execution_authority;
  }
  const std::int64_t publication_now_ns = get_clock()->now().nanoseconds();
  const bool vehicle_status_epoch_stable =
      !vehicle_status_epoch_probation_ && !vehicle_status_revision_exhausted_;
  const bool vehicle_status_authoritative = vehicleStatusAuthoritativeForExecution(
      vehicle_status_, vehicle_status_epoch_stable, publication_now_ns,
      config_.execution.maximum_vehicle_status_age_ms);
  const RawWorldIngressSnapshot3D world_input = raw_world_ingress_->snapshot();
  const std::shared_ptr<const ProductionMppiRawWorld3D> committed_3d =
      world_input.latest_raw_world;
  const double maximum_observation_age_ms =
      config_.world.maximum_esdf_age_ms +
      config_.execution.stale_esdf_execution_window_ms;
  const double current_raw_age_ms =
      committed_3d != nullptr
          ? committedRawWorldAgeMs(committed_3d.get(), publication_now_ns)
          : -1.0;
  const OffboardSessionPublicationCurrentnessStatus offboard_currentness =
      assessOffboardSessionPublicationCurrentness(
          offboard_session_admission_, offboard_session_receive_stamp_ns_,
          cycle.evidence.offboard_session,
          cycle.evidence.offboard_session_receive_stamp_ns,
          owner.target_offboard_instance_id, publication_now_ns,
          config_.execution.maximum_control_feedback_age_ms);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      latest_lidar_evidence_.load(std::memory_order_acquire);
  candidate.owner = owner;
  candidate.expected_horizon_producer_instance_id =
      execution_horizon_producer_instance_id_;
  candidate.execution_input = publication_execution_input;
  const auto commit_started = std::chrono::steady_clock::now();
  latest_horizon_assembly_ms_ = std::chrono::duration<double, std::milli>(
                                    commit_started - latest_publication_started_)
                                    .count();
  const ExecutionHorizonCommitResult3D committed =
      execution_supervisor_.commitHorizon(ExecutionHorizonCommitRequest3D{
          .candidate = std::move(candidate),
          .runtime =
              ExecutionHorizonRuntimeCurrentness3D{
                  .vehicle_status_authoritative = vehicle_status_authoritative,
                  .raw_world_identity_conflicted =
                      world_input.raw_world_identity_conflicted,
                  .current_raw_age_ms = current_raw_age_ms,
                  .maximum_raw_age_ms = maximum_observation_age_ms,
                  .revocation_epoch_current =
                      requested_execution_revocation_.load(std::memory_order_acquire) ==
                      handled_execution_revocation_request_,
                  .objective_current =
                      navigationObjective() == cycle.evidence.objective,
                  .navigation_authoritative = navigation_.valid,
                  .offboard_session_currentness = offboard_currentness,
              },
          .navigation =
              ExecutionHorizonNavigationWitness3D{
                  .state = navigation_.state,
                  .measured_equivalent_control =
                      navigation_.measured_equivalent_control,
                  .pose_revision = navigation_.revision,
                  .source_timestamp_us = navigation_.source_timestamp_us,
                  .receive_stamp_ns = navigation_.receive_stamp_ns,
                  .measured_control_source_sequence = navigation_.source_timestamp_us,
                  .measured_control_source_stamp_ns = navigation_.receive_stamp_ns,
                  .measured_control_receive_stamp_ns = navigation_.receive_stamp_ns,
                  .measured_acceleration_authoritative =
                      navigation_.measured_acceleration_valid,
              },
          .current_observed_raw_world =
              !config_.world.use_static_map && committed_3d != nullptr
                  ? committed_3d->authoritativeOwner()
                  : nullptr,
          .current_lidar_evidence = current_lidar,
          .publication_now_ns = publication_now_ns,
          .maximum_control_feedback_age_ms =
              config_.execution.maximum_control_feedback_age_ms,
          .latest_lidar_identity_conflicted =
              latest_lidar_evidence_identity_conflicted_.load(
                  std::memory_order_acquire),
      });
  latest_horizon_commit_ms_ = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - commit_started)
                                  .count();
  switch (committed.revocation_request) {
    case ExecutionHorizonRevocationRequest3D::kNone:
      break;
    case ExecutionHorizonRevocationRequest3D::kUnavailableWorld:
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
      break;
    case ExecutionHorizonRevocationRequest3D::kNoExecutableHorizon:
      requestExecutionRevocation(ProductionMppiExecutionReason::kNoExecutableHorizon);
      break;
  }
  if (!committed.committed()) {
    if (committed.status ==
        ExecutionHorizonCommitStatus3D::kOffboardSessionNotCurrent) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EXECUTION_HORIZON_COMMIT offboard_session=%s session_source_age_ms=%.1f "
          "session_receive_age_ms=%.1f maximum_age_ms=%.1f",
          offboardSessionPublicationCurrentnessStatusName(offboard_currentness),
          static_cast<double>(publication_now_ns -
                              offboard_session_admission_.latest_source_stamp_ns) *
              1.0e-6,
          static_cast<double>(publication_now_ns - offboard_session_receive_stamp_ns_) *
              1.0e-6,
          config_.execution.maximum_control_feedback_age_ms);
    }
    if (committed.status == ExecutionHorizonCommitStatus3D::kEvidenceNotCurrent) {
      const std::string_view currentness = executionPublicationCurrentnessStatus3DName(
          committed.publication_currentness);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "EXECUTION_HORIZON_COMMIT evidence_currentness=%.*s "
                           "latest_evidence_revalidated=%s",
                           static_cast<int>(currentness.size()), currentness.data(),
                           committed.latest_evidence_revalidated ? "true" : "false");
    }
    report_commit_failure(executionHorizonCommitStatus3DName(committed.status));
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  execution_horizon_sequence_ = publication_horizon.sequence;
  latest_horizon_publication_ = ProductionMppiHorizonPublicationRecord{
      .sequence = publication_horizon.sequence,
      .publication_stamp_ns = publication_now_ns,
  };
  if (committed.replaced_applied_control) {
    recordAppliedControlDiscontinuityLocked();
  }
  const auto wire_started = std::chrono::steady_clock::now();
  execution_horizon_pub_->publish(publication_horizon);
  latest_horizon_wire_ms_ = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - wire_started)
                                .count();
  horizon_publications_.fetch_add(1U, std::memory_order_relaxed);
  RCLCPP_INFO(
      get_logger(),
      "EXECUTION_HORIZON published=true producer=%" PRIu64 " sequence=%" PRIu64
      " mode=%s previous_control=%s input_pose_age_ms=%.1f "
      "input_control_age_ms=%.1f capture_to_publication_ms=%.1f",
      publication_horizon.producer_instance_id, publication_horizon.sequence,
      publication_horizon.execution_mode ==
              msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED
          ? "planned"
          : "non_planned",
      executionControlEvidenceSourceName(
          publication_execution_input->previousControlSource()),
      evidenceAgeMs(publication_now_ns,
                    publication_execution_input->poseReceiveStampNs()),
      evidenceAgeMs(publication_now_ns,
                    publication_execution_input->previousControlReceiveStampNs()),
      evidenceAgeMs(publication_now_ns,
                    publication_execution_input->effectiveStampNs()));
  return ProductionMppiHorizonCommitStatus::kPublished;
}

ProductionMppiHorizonCommitStatus ProductionMppiNode::commitExecutionSnapshotHorizon(
    const ProductionMppiExecutionCycle& cycle,
    const std::shared_ptr<const ExecutionPlan3D>& expected,
    const ExecutionRouteTransitionResult3D& transition,
    const msg::MppiTrajectoryHorizon& horizon,
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    const std::shared_ptr<const ExecutionPlan3D>& certification_snapshot,
    const std::shared_ptr<const ExecutionRouteTransitionResult3D>& progress_preparation,
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority) {
  const auto evidence_lock = evidence_boundary_.evidenceWithLatestLidar();
  return commitAndPublishExecutionHorizon(
      cycle, horizon,
      ExecutionHorizonLeaseCandidate3D{
          .kind = expected_pending != nullptr
                      ? ExecutionHorizonCommitKind3D::kPendingTransition
                      : ExecutionHorizonCommitKind3D::kTransition,
          .expected_authority = expected_authority,
          .expected_plan = expected,
          .certification_plan = certification_snapshot,
          .progress_preparation = progress_preparation,
          .transition =
              std::make_shared<const ExecutionRouteTransitionResult3D>(transition),
          .expected_pending = expected_pending,
          .owner = {},
          .expected_horizon_producer_instance_id = 0U,
          .execution_input = cycle.evidence.execution_input,
          .stationary_capture_rearm_intent = false,
      });
}

} // namespace drone_city_nav
