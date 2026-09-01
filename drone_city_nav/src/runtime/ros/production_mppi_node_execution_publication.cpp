#include "drone_city_nav/execution_horizon_contract_ros.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"

#include <atomic>
#include <cinttypes>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <utility>

#include "execution_publication_navigation_rebase_3d.hpp"
#include "production_mppi_node_execution_internal.hpp"
#include "production_mppi_node_planning_tick_rearm.hpp"
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

struct FiniteExecutionEvidenceView {
  const mppi::FiniteHorizon* horizon{nullptr};
  const VersionedExecutionInput3D* execution_input{nullptr};
  std::int64_t valid_until_ns{0};
  std::int64_t control_interval_ns{0};
};

template<typename Execution>
[[nodiscard]] std::optional<FiniteExecutionEvidenceView>
finiteExecutionArtifactEvidenceView(const Execution& execution) noexcept {
  if (execution.horizon == nullptr || execution.execution_input == nullptr) {
    return std::nullopt;
  }
  return FiniteExecutionEvidenceView{
      .horizon = execution.horizon.get(),
      .execution_input = execution.execution_input.get(),
      .valid_until_ns = execution.valid_until_ns,
      .control_interval_ns = execution.control_interval_ns,
  };
}

[[nodiscard]] std::optional<FiniteExecutionEvidenceView>
finiteExecutionEvidenceView(const ExecutionPlan3D& snapshot) noexcept {
  if (const FiniteExecutionState3D* const execution = snapshot.finiteExecution()) {
    return finiteExecutionArtifactEvidenceView(*execution);
  }
  if (const DirectTrackingFiniteExecution3D* const execution =
          snapshot.directTrackingExecution()) {
    return finiteExecutionArtifactEvidenceView(*execution);
  }
  return std::nullopt;
}

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
  if (execution_horizon_sequence_ != std::numeric_limits<std::uint64_t>::max()) {
    horizon.sequence = ++execution_horizon_sequence_;
  }
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
  using production_mppi_execution_detail::sameControl;
  using production_mppi_execution_detail::timeToNanoseconds;
  const auto report_commit_failure = [this](const char* const stage) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "EXECUTION_HORIZON_COMMIT committed=false stage=%s", stage);
  };

  msg::MppiTrajectoryHorizon publication_horizon = horizon;
  std::shared_ptr<const VersionedExecutionInput3D> publication_execution_input =
      cycle.evidence.execution_input;
  std::optional<ExecutionRouteTransitionResult3D> rebased_transition;
  if (publication_execution_input == nullptr || !publication_execution_input->valid() ||
      assessExecutionHorizonPayload(
          publication_horizon, ExecutionHorizonPayloadValidationConfig{
                                   .expected_frame_id = config_.world.frame_id,
                                   .flight_envelope = &config_.world.flight_envelope,
                               }) != ExecutionHorizonPayloadStatus::kValid) {
    report_commit_failure("invalid_input_or_payload");
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  ExecutionOwnerIdentity3D owner{
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
  const std::scoped_lock input_lock{input_mutex_};
  const std::shared_ptr<const CommittedExecutionAuthority3D>
      resident_execution_authority = execution_supervisor_.authority();
  const bool control_evidence_only_authority_successor =
      isControlEvidenceOnlyAuthoritySuccessor3D(candidate.expected_authority,
                                                resident_execution_authority);
  if (control_evidence_only_authority_successor) {
    candidate.expected_authority = resident_execution_authority;
  }
  const bool resident_authority_current =
      resident_execution_authority != nullptr &&
      resident_execution_authority->valid() &&
      resident_execution_authority == candidate.expected_authority &&
      resident_execution_authority->plan() == candidate.expected_plan;
  const AppliedControlEvidence3D resident_control =
      resident_execution_authority != nullptr ? resident_execution_authority->control()
                                              : AppliedControlEvidence3D{};
  const ExecutionOwnerIdentity3D resident_owner =
      resident_execution_authority != nullptr ? resident_execution_authority->owner()
                                              : ExecutionOwnerIdentity3D{};
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
  const bool navigation_advanced =
      navigation_.revision != publication_execution_input->poseRevision() ||
      navigation_.source_timestamp_us !=
          publication_execution_input->poseSourceTimestampUs() ||
      navigation_.receive_stamp_ns != publication_execution_input->poseReceiveStampNs();
  const bool previous_control_evidence_current = [&]() noexcept {
    switch (publication_execution_input->previousControlSource()) {
      case ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback:
        return appliedControlAuthoritativeForExecution(
                   resident_control, resident_owner, publication_now_ns,
                   config_.execution.maximum_control_feedback_age_ms) &&
               resident_control.horizon_producer_instance_id ==
                   publication_execution_input
                       ->previousControlSourceProducerInstanceId() &&
               resident_control.horizon_sequence ==
                   publication_execution_input->previousControlSourceSequence() &&
               resident_control.source_stamp_ns ==
                   publication_execution_input->previousControlSourceStampNs() &&
               resident_control.receive_stamp_ns ==
                   publication_execution_input->previousControlReceiveStampNs() &&
               sameControl(resident_control.control,
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
      resident_authority_current &&
      (navigation_advanced || !previous_control_evidence_current);
  if (execution_input_advanced) {
    bool late_rebase_candidate_rejected{false};
    std::size_t late_rebase_source_control_index{0U};
    const auto rebase_for_current_navigation = [&]() -> const char* {
      const bool planned_snapshot_transition =
          publication_horizon.execution_mode ==
              msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED &&
          (candidate.kind == ExecutionHorizonCommitKind3D::kTransition ||
           candidate.kind == ExecutionHorizonCommitKind3D::kPendingTransition) &&
          candidate.expected_plan != nullptr && candidate.transition != nullptr &&
          candidate.transition->applied() && candidate.transition->next != nullptr;
      if (!planned_snapshot_transition) {
        return "navigation_advanced_without_rebase_contract";
      }
      if (execution_input_capture_sequence_ ==
          std::numeric_limits<std::uint64_t>::max()) {
        return "late_rebase_input_sequence_exhausted";
      }
      const ProductionMppiExecutionInputPreparation current_input_preparation =
          prepareExecutionInputForPlanningTick(
              navigation_, resident_execution_authority,
              ++execution_input_capture_sequence_, publication_now_ns,
              config_.execution.maximum_control_feedback_age_ms, false, false);
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
          committed_3d != nullptr ? committed_3d->authoritativeOwner() : nullptr;
      ExecutionPublicationNavigationRebaseResult3D rebase =
          rebaseExecutionPublicationForCurrentNavigation3D(
              ExecutionPublicationNavigationRebaseRequest3D{
                  .expected_snapshot = candidate.expected_plan.get(),
                  .certification_snapshot = candidate.certification_plan.get(),
                  .progress_preparation = candidate.progress_preparation.get(),
                  .candidate_snapshot = candidate.transition->next.get(),
                  .expected_pending = candidate.expected_pending.get(),
                  .lifecycle_event =
                      cycle.route.execution.lifecycle_event.has_value()
                          ? std::addressof(*cycle.route.execution.lifecycle_event)
                          : nullptr,
                  .current_execution_input = current_input,
                  .current_lidar_evidence = current_lidar,
                  .current_observed_raw_world = current_raw,
                  .publication_now_ns = publication_now_ns,
                  .arrival_search_step_controls =
                      cycle.controller.arrival_search_step_controls,
                  .finite_horizon_config = &config_.execution.finite_horizon,
                  .terminal_boundary =
                      cycle.evidence.execution_path_world.terminal_boundary,
              });
      late_rebase_source_control_index = rebase.source_control_index;
      if (!rebase.rebased() || !rebase.transition.has_value() ||
          rebase.transition->next == nullptr) {
        late_rebase_candidate_rejected = true;
        const double input_pose_age_ms =
            evidenceAgeMs(publication_now_ns, current_input->poseReceiveStampNs());
        const double input_control_age_ms = evidenceAgeMs(
            publication_now_ns, current_input->previousControlReceiveStampNs());
        const LatestLidarEvidenceFreshness3D lidar_freshness =
            assessLatestLidarEvidenceFreshness3D(
                *current_lidar, publication_now_ns,
                candidate.transition->next->route() != nullptr
                    ? candidate.transition->next->route()
                          ->validation_policy->latestLidarMaximumAgeMs()
                    : config_.execution.latest_lidar_obstacle_maximum_age_ms);
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "EXECUTION_HORIZON_REBASE rebased=false status=%s path_validation=%s "
            "route_certification=%.*s route_adherence=%.*s "
            "source_control_index=%zu route_adherence_state_index=%zu "
            "route_adherence_failure_distance_m=%.3f transition=%.*s "
            "input_control_source=%s input_pose_age_ms=%.3f "
            "input_control_age_ms=%.3f input_pose_maximum_age_ms=%.3f "
            "input_control_maximum_age_ms=%.3f lidar_age_ms=%.3f lidar_fresh=%s",
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
            executionRouteTransitionStatus3DName(rebase.transition_status).data(),
            executionControlEvidenceSourceName(current_input->previousControlSource()),
            input_pose_age_ms, input_control_age_ms,
            candidate.transition->next->route() != nullptr
                ? candidate.transition->next->route()
                      ->validation_policy->executionInputMaximumPoseAgeMs()
                : -1.0,
            candidate.transition->next->route() != nullptr
                ? candidate.transition->next->route()
                      ->validation_policy->executionInputMaximumControlAgeMs()
                : -1.0,
            lidar_freshness.age_ms, lidar_freshness.fresh ? "true" : "false");
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
          assessExecutionHorizonPayload(
              publication_horizon,
              ExecutionHorizonPayloadValidationConfig{
                  .expected_frame_id = config_.world.frame_id,
                  .flight_envelope = &config_.world.flight_envelope,
              }) != ExecutionHorizonPayloadStatus::kValid) {
        return "late_rebase_payload_invalid";
      }
      publication_execution_input = current_input;
      rebased_transition.emplace(std::move(current_transition));
      candidate.transition =
          std::make_shared<const ExecutionRouteTransitionResult3D>(*rebased_transition);
      owner.valid_from_ns = publication_now_ns;
      owner.valid_until_ns = rebased_valid_until_ns;
      return nullptr;
    };
    if (const char* const rebase_failure = rebase_for_current_navigation();
        rebase_failure != nullptr) {
      report_commit_failure(rebase_failure);
      const ExecutionPlan3D* const retained_candidate =
          candidate.transition != nullptr && candidate.transition->next != nullptr
              ? candidate.transition->next.get()
              : nullptr;
      const bool retained_execution =
          retained_candidate != nullptr &&
          ((retained_candidate->finiteExecution() != nullptr &&
            retained_candidate->finiteExecution()->kind ==
                FiniteExecutionKind3D::kRetained) ||
           (retained_candidate->directTrackingExecution() != nullptr &&
            retained_candidate->directTrackingExecution()->kind ==
                FiniteExecutionKind3D::kRetained));
      const bool resident_owner_witnessed = appliedControlAuthoritativeForExecution(
          resident_control, resident_owner, publication_now_ns,
          config_.execution.maximum_control_feedback_age_ms);
      const bool no_revocation_pending =
          requested_execution_revocation_.load(std::memory_order_acquire) ==
          handled_execution_revocation_request_;
      const bool exact_resident_snapshot =
          candidate.expected_plan != nullptr &&
          resident_execution_authority->plan() == candidate.expected_plan;
      const bool resident_execution_owner_matches =
          candidate.expected_plan != nullptr &&
          resident_owner.execution_owner_epoch ==
              candidate.expected_plan->execution_owner_epoch;
      const bool resident_owner_may_continue =
          canContinueResidentPlannedOwner(ProductionMppiResidentOwnerContinuationCheck{
              .owner = &resident_owner,
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
                             rebase_failure, resident_owner.sequence);
        return ProductionMppiHorizonCommitStatus::kDeferredResidentOwner;
      }
      return ProductionMppiHorizonCommitStatus::kRejected;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_HORIZON_COMMIT late_rebase=true source_pose_revision=%" PRIu64
        " publication_pose_revision=%" PRIu64
        " source_control_index=%zu control_evidence_advanced=%s "
        "authority_control_successor=%s",
        cycle.evidence.execution_input->poseRevision(),
        publication_execution_input->poseRevision(), late_rebase_source_control_index,
        previous_control_evidence_current ? "false" : "true",
        control_evidence_only_authority_successor ? "true" : "false");
  }
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      latest_lidar_evidence_.load(std::memory_order_acquire);
  candidate.owner = owner;
  candidate.expected_horizon_producer_instance_id =
      execution_horizon_producer_instance_id_;
  candidate.execution_input = publication_execution_input;
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
    report_commit_failure(executionHorizonCommitStatus3DName(committed.status));
    return ProductionMppiHorizonCommitStatus::kRejected;
  }
  if (committed.replaced_applied_control) {
    recordAppliedControlDiscontinuityLocked();
  }
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
    const std::shared_ptr<const ExecutionPlan3D>& expected,
    const ExecutionRouteTransitionResult3D& transition,
    const msg::MppiTrajectoryHorizon& horizon,
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    const std::shared_ptr<const ExecutionPlan3D>& certification_snapshot,
    const std::shared_ptr<const ExecutionRouteTransitionResult3D>& progress_preparation,
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority) {
  const std::scoped_lock evidence_lock{execution_evidence_commit_mutex_,
                                       latest_lidar_evidence_commit_mutex_};
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
