#include "drone_city_nav/execution_horizon_commit_3d.hpp"
#include "drone_city_nav/execution_supervisor_3d.hpp"
#include "drone_city_nav/finite_execution_path_3d.hpp"

#include <cmath>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav {
namespace {

struct FiniteExecutionEvidenceView3D {
  const FiniteMotionHorizon3D* horizon{nullptr};
  const VersionedExecutionInput3D* execution_input{nullptr};
  const VersionedExecutionValidationPolicy3D* policy{nullptr};
  const VersionedStaticWorld3D* static_world{nullptr};
  std::int64_t control_interval_ns{0};
};

template<typename Execution>
[[nodiscard]] std::optional<FiniteExecutionEvidenceView3D>
finiteExecutionEvidenceView(const Execution& execution) noexcept {
  if (execution.horizon == nullptr || execution.execution_input == nullptr ||
      execution.validation_policy == nullptr ||
      (execution.observed_raw_world == nullptr) ==
          (execution.static_world == nullptr)) {
    return std::nullopt;
  }
  return FiniteExecutionEvidenceView3D{
      .horizon = execution.horizon.get(),
      .execution_input = execution.execution_input.get(),
      .policy = execution.validation_policy.get(),
      .static_world = execution.static_world.get(),
      .control_interval_ns = execution.control_interval_ns,
  };
}

[[nodiscard]] std::vector<TimedExecutionPathPoint3D>
timedExecutionPathPoints(const FiniteExecutionEvidenceView3D& view) {
  std::vector<TimedExecutionPathPoint3D> points;
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
    points.push_back(TimedExecutionPathPoint3D{
        .time_from_start_s = static_cast<double>(index) * step_s,
        .state = view.horizon->states[index],
        .control = index == 0U ? view.execution_input->previousControl()
                               : view.horizon->controls[index - 1U],
    });
  }
  return points;
}

[[nodiscard]] bool revalidateFiniteExecution(
    const FiniteExecutionEvidenceView3D& view,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& latest_raw,
    const std::shared_ptr<const VersionedLatestLidarEvidence3D>& latest_lidar) {
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
  const std::vector<TimedExecutionPathPoint3D> points = timedExecutionPathPoints(view);
  if (points.empty()) {
    return false;
  }
  const std::optional<LaunchSupportContact3D>& launch_support =
      !static_world ? latest_raw->launchSupportContact()
                    : std::optional<LaunchSupportContact3D>{};
  const FiniteExecutionPathWorld3D world{
      .flight_envelope = &view.policy->flightEnvelope(),
      .dynamics = &view.policy->dynamics(),
      .altitude_envelope = &view.policy->altitudeEnvelope(),
      .footprint = &view.policy->sweptFootprint(),
      .static_occupancy = static_world ? &view.static_world->occupancy() : nullptr,
      .observed_occupancy = !static_world ? &latest_raw->occupancy() : nullptr,
      .launch_support_contact =
          launch_support ? std::addressof(*launch_support) : nullptr,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points =
          std::span<const Point3>{latest_lidar->hitPointsMapM()},
      .terminal_boundary = std::nullopt,
  };
  return validateCompleteFiniteExecutionPath3D(
             points, view.execution_input->previousControl(), world)
      .accepted();
}

[[nodiscard]] bool revalidateFiniteExecution(
    const ExecutionPlan3D& snapshot,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& latest_raw,
    const std::shared_ptr<const VersionedLatestLidarEvidence3D>& latest_lidar) {
  if (const FiniteExecutionState3D* const execution = snapshot.finiteExecution()) {
    const FiniteExecutionState3D* const braking = snapshot.brakingFallback();
    if (braking == nullptr) {
      return false;
    }
    const std::optional<FiniteExecutionEvidenceView3D> command =
        finiteExecutionEvidenceView(*execution);
    const std::optional<FiniteExecutionEvidenceView3D> fallback =
        finiteExecutionEvidenceView(*braking);
    return command.has_value() && fallback.has_value() &&
           revalidateFiniteExecution(*command, latest_raw, latest_lidar) &&
           revalidateFiniteExecution(*fallback, latest_raw, latest_lidar);
  }
  const DirectTrackingFiniteExecution3D* const direct =
      snapshot.directTrackingExecution();
  if (direct != nullptr) {
    const std::optional<FiniteExecutionEvidenceView3D> view =
        finiteExecutionEvidenceView(*direct);
    return view.has_value() &&
           revalidateFiniteExecution(*view, latest_raw, latest_lidar);
  }
  // A stationary hold executes no path; its evidence on the newest world is
  // the body at the hold position staying clear of raw occupancy. Without this
  // a re-lease fails on every world change and the hold churns through
  // expiry, revocation and re-arm while a pending route waits for a stable
  // base to hand off from.
  const StationaryExecutionHold3D* const hold = snapshot.stationaryHold();
  if (hold == nullptr || !hold->valid() || hold->terminal_execution_input == nullptr ||
      hold->validation_policy == nullptr || latest_lidar == nullptr ||
      !latest_lidar->valid()) {
    return false;
  }
  const bool static_world = hold->static_world != nullptr;
  if ((!static_world && (latest_raw == nullptr || !latest_raw->valid())) ||
      (static_world && !hold->static_world->valid())) {
    return false;
  }
  return execution_route_snapshot_3d_internal::stationaryHoldRawSafe(
      hold->position, *hold->terminal_execution_input,
      static_world ? nullptr : latest_raw.get(), hold->static_world.get(),
      *hold->validation_policy, *latest_lidar);
}

[[nodiscard]] std::shared_ptr<const VersionedObservedRawWorld3D>
snapshotRawOwner(const ExecutionPlan3D& snapshot) {
  if (const StationaryExecutionHold3D* const hold = snapshot.stationaryHold()) {
    return hold->observed_raw_world;
  }
  if (const FiniteExecutionState3D* const execution = snapshot.finiteExecution();
      execution != nullptr && execution->observed_raw_world != nullptr) {
    return execution->observed_raw_world;
  }
  if (const DirectTrackingFiniteExecution3D* const execution =
          snapshot.directTrackingExecution();
      execution != nullptr && execution->observed_raw_world != nullptr) {
    return execution->observed_raw_world;
  }
  const CertifiedRouteSuffix3D* const route = snapshot.route();
  return route != nullptr ? route->observed_raw_world : nullptr;
}

[[nodiscard]] std::shared_ptr<const VersionedLatestLidarEvidence3D>
snapshotLidarOwner(const ExecutionPlan3D& snapshot) {
  if (const StationaryExecutionHold3D* const hold = snapshot.stationaryHold()) {
    return hold->latest_lidar_evidence;
  }
  if (const FiniteExecutionState3D* const execution = snapshot.finiteExecution()) {
    return execution->latest_lidar_evidence;
  }
  const DirectTrackingFiniteExecution3D* const execution =
      snapshot.directTrackingExecution();
  return execution != nullptr ? execution->latest_lidar_evidence : nullptr;
}

[[nodiscard]] std::shared_ptr<const VersionedExecutionValidationPolicy3D>
snapshotValidationPolicy(const ExecutionPlan3D& snapshot) {
  if (const StationaryExecutionHold3D* const hold = snapshot.stationaryHold()) {
    return hold->validation_policy;
  }
  if (const FiniteExecutionState3D* const execution = snapshot.finiteExecution()) {
    return execution->validation_policy;
  }
  if (const DirectTrackingFiniteExecution3D* const execution =
          snapshot.directTrackingExecution()) {
    return execution->validation_policy;
  }
  const CertifiedRouteSuffix3D* const route = snapshot.route();
  return route != nullptr ? route->validation_policy : nullptr;
}

[[nodiscard]] bool
progressPreservesRouteEvidence(const ExecutionPlan3D& expected,
                               const ExecutionPlan3D& prepared) noexcept {
  const CertifiedRouteSuffix3D* const source = expected.route();
  const CertifiedRouteSuffix3D* const next = prepared.route();
  // The prepared progress may carry route evidence re-derived from a newer
  // raw revision of the same lineage; the commit revalidates the finite path
  // against the current evidence, so only the identity of the route and the
  // lineage of its raw evidence must be preserved.
  const auto same_raw_lineage = [](const auto& first, const auto& second) {
    if (first == second) {
      return true;
    }
    return first != nullptr && second != nullptr && first->valid() && second->valid() &&
           first->version().sameLineage(second->version()) &&
           first->version().revision <= second->version().revision;
  };
  return source != nullptr && next != nullptr &&
         source->route_instance_id == next->route_instance_id &&
         source->owner.id == next->owner.id &&
         source->identity.generation == next->identity.generation &&
         source->geometry == next->geometry &&
         source->continuity_id == next->continuity_id &&
         same_raw_lineage(source->observed_raw_world, next->observed_raw_world) &&
         source->static_world == next->static_world &&
         source->validation_policy == next->validation_policy &&
         source->planned_endpoint_semantics == next->planned_endpoint_semantics;
}

[[nodiscard]] bool sameState(const MotionState3D& first,
                             const MotionState3D& second) noexcept {
  return first.x == second.x && first.y == second.y && first.z == second.z &&
         first.vx == second.vx && first.vy == second.vy && first.vz == second.vz &&
         first.yaw == second.yaw && first.yaw_rate == second.yaw_rate;
}

[[nodiscard]] bool rawWorldCurrent(
    const ExecutionHorizonRuntimeCurrentness3D& currentness,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& expected_raw,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& current_raw) noexcept {
  if (expected_raw == nullptr) {
    return true;
  }
  return !currentness.raw_world_identity_conflicted && expected_raw->valid() &&
         current_raw != nullptr && current_raw->valid() &&
         expected_raw->version().producer_instance_id != 0U &&
         current_raw->version().producer_instance_id ==
             expected_raw->version().producer_instance_id &&
         std::isfinite(currentness.current_raw_age_ms) &&
         currentness.current_raw_age_ms >= 0.0 &&
         std::isfinite(currentness.maximum_raw_age_ms) &&
         currentness.maximum_raw_age_ms >= 0.0 &&
         currentness.current_raw_age_ms <= currentness.maximum_raw_age_ms;
}

[[nodiscard]] bool ownerEnumsKnown(const ExecutionOwnerIdentity3D& owner) noexcept {
  switch (owner.execution_mode) {
    case ExecutionAuthorityMode3D::kPlanned:
    case ExecutionAuthorityMode3D::kPositionHold:
      break;
    case ExecutionAuthorityMode3D::kRevoked:
      return false;
  }
  switch (owner.execution_reason) {
    case ExecutionAuthorityReason3D::kNone:
    case ExecutionAuthorityReason3D::kNoExecutableHorizon:
    case ExecutionAuthorityReason3D::kCooperativePassageYield:
    case ExecutionAuthorityReason3D::kGoalCapture:
    case ExecutionAuthorityReason3D::kNoExecutableRoute:
    case ExecutionAuthorityReason3D::kUnavailableWorld:
      return true;
  }
  return false;
}

// The execution input was captured from one navigation revision. Publication
// requires that navigation has not regressed or been reset since that capture;
// its age is bounded separately by the validation policy. Newer navigation
// samples do not invalidate the captured input: the horizon is timed from its
// own capture instant and the controller interpolates it by elapsed time.
[[nodiscard]] bool executionInputLineageCurrent(
    const VersionedExecutionInput3D& input,
    const ExecutionHorizonNavigationWitness3D& navigation) noexcept {
  return navigation.pose_revision >= input.poseRevision() &&
         navigation.receive_stamp_ns >= input.poseReceiveStampNs() &&
         navigation.source_timestamp_us >= input.poseSourceTimestampUs();
}

// Previous-control evidence stays valid while it still describes the lease the
// controller is executing. Offboard feedback must belong to the resident owner
// and that owner must still be witnessed; later samples of the same lease are
// not a contradiction. Measured acceleration only requires that navigation has
// not regressed past the sample that produced it.
[[nodiscard]] bool
previousControlCurrent(const VersionedExecutionInput3D& input,
                       const ExecutionHorizonNavigationWitness3D& navigation,
                       const AppliedControlEvidence3D& resident_control,
                       const ExecutionOwnerIdentity3D& resident_owner,
                       const std::int64_t now_ns,
                       const double maximum_control_feedback_age_ms,
                       const bool stationary_capture_rearm) noexcept {
  switch (input.previousControlSource()) {
    case ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback:
      return appliedControlCurrentForExecutionInput3D(
                 resident_control, resident_owner, now_ns,
                 maximum_control_feedback_age_ms) &&
             resident_control.horizon_producer_instance_id ==
                 input.previousControlSourceProducerInstanceId() &&
             resident_control.horizon_sequence >=
                 input.previousControlSourceSequence() &&
             resident_control.source_stamp_ns >= input.previousControlSourceStampNs() &&
             resident_control.receive_stamp_ns >= input.previousControlReceiveStampNs();
    case ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration:
      return navigation.measured_acceleration_authoritative &&
             navigation.measured_control_receive_stamp_ns >=
                 input.previousControlReceiveStampNs() &&
             navigation.measured_control_source_sequence >=
                 input.previousControlSourceSequence();
    case ExecutionPreviousControlEvidenceSource3D::kAssumedZero:
      return stationary_capture_rearm;
    case ExecutionPreviousControlEvidenceSource3D::kUnknown:
    case ExecutionPreviousControlEvidenceSource3D::kEngineFallback:
      return false;
  }
  return false;
}

[[nodiscard]] bool stationaryCaptureRearmCommit(
    const ExecutionHorizonLeaseCandidate3D& candidate,
    const ExecutionPlan3D& publication_plan, const ExecutionOwnerIdentity3D& owner,
    const ExecutionHorizonNavigationWitness3D& navigation,
    const ExecutionOwnerIdentity3D& resident_owner,
    const AppliedControlEvidence3D& resident_control) noexcept {
  const StationaryExecutionHold3D* const hold = publication_plan.stationaryHold();
  return candidate.stationary_capture_rearm_intent &&
         candidate.execution_input != nullptr &&
         candidate.execution_input->stationaryCaptureStateAuthoritative() &&
         candidate.kind == ExecutionHorizonCommitKind3D::kTransition &&
         candidate.expected_plan != nullptr &&
         candidate.expected_plan->phase() == ExecutionRoutePhase3D::kRevoked &&
         candidate.transition != nullptr && candidate.transition->applied() &&
         hold != nullptr &&
         hold->origin == StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm &&
         hold->terminal_execution_input == candidate.execution_input &&
         hold->position.x == owner.stationary_hold_position.x &&
         hold->position.y == owner.stationary_hold_position.y &&
         hold->position.z == owner.stationary_hold_position.z &&
         owner.execution_mode == ExecutionAuthorityMode3D::kPositionHold &&
         owner.execution_reason == ExecutionAuthorityReason3D::kGoalCapture &&
         owner.stationary_position_hold && !resident_owner.valid &&
         !resident_control.valid &&
         sameState(navigation.state, hold->terminal_execution_input->state());
}

[[nodiscard]] ExecutionHorizonCommitResult3D
reject(const ExecutionHorizonCommitStatus3D status,
       const ExecutionHorizonRevocationRequest3D revocation =
           ExecutionHorizonRevocationRequest3D::kNone) noexcept {
  ExecutionHorizonCommitResult3D result;
  result.status = status;
  result.revocation_request = revocation;
  return result;
}

} // namespace

bool ExecutionHorizonCommitResult3D::committed() const noexcept {
  return status == ExecutionHorizonCommitStatus3D::kCommitted &&
         lease_status == ExecutionRoutePublicationStatus3D::kPublished;
}

const char* appliedControlAuthorityFailure3D(const AppliedControlEvidence3D& control,
                                             const ExecutionOwnerIdentity3D& owner,
                                             const std::int64_t now_ns,
                                             const double maximum_age_ms) noexcept {
  if (!owner.valid) {
    return "owner_invalid";
  }
  if (!control.valid) {
    return "control_evidence_empty";
  }
  if (!control.control_authoritative) {
    return "control_not_authoritative";
  }
  if (control.producer_instance_id == 0U ||
      control.horizon_producer_instance_id == 0U) {
    return "control_identity_missing";
  }
  if (control.horizon_producer_instance_id != owner.producer_instance_id) {
    return "horizon_producer_mismatch";
  }
  if (control.producer_instance_id != owner.target_offboard_instance_id) {
    return "offboard_producer_mismatch";
  }
  if (control.execution_mode != ExecutionAuthorityMode3D::kPlanned ||
      owner.execution_mode != ExecutionAuthorityMode3D::kPlanned) {
    return "execution_mode_not_planned";
  }
  if (control.horizon_sequence == 0U || control.horizon_sequence != owner.sequence) {
    return "horizon_sequence_mismatch";
  }
  if (control.source_stamp_ns <= 0 || control.receive_stamp_ns <= 0 || now_ns < 0) {
    return "control_stamp_invalid";
  }
  if (owner.valid_from_ns <= 0 || owner.valid_until_ns <= owner.valid_from_ns) {
    return "owner_lease_invalid";
  }
  if (now_ns < owner.valid_from_ns || now_ns >= owner.valid_until_ns) {
    return "owner_lease_not_current";
  }
  if (control.source_stamp_ns < owner.valid_from_ns ||
      control.source_stamp_ns >= owner.valid_until_ns) {
    return "control_source_outside_lease";
  }
  if (!std::isfinite(maximum_age_ms) || !(maximum_age_ms > 0.0)) {
    return "maximum_age_invalid";
  }
  const double source_age_ms =
      static_cast<double>(now_ns - control.source_stamp_ns) * 1.0e-6;
  if (std::abs(source_age_ms) > maximum_age_ms) {
    return "control_source_stale";
  }
  const double receive_age_ms =
      static_cast<double>(now_ns - control.receive_stamp_ns) * 1.0e-6;
  if (std::abs(receive_age_ms) > maximum_age_ms) {
    return "control_receipt_stale";
  }
  return nullptr;
}

bool appliedControlAuthoritativeForExecution3D(const AppliedControlEvidence3D& control,
                                               const ExecutionOwnerIdentity3D& owner,
                                               const std::int64_t now_ns,
                                               const double maximum_age_ms) noexcept {
  return appliedControlAuthorityFailure3D(control, owner, now_ns, maximum_age_ms) ==
         nullptr;
}

bool appliedControlCurrentForExecutionInput3D(const AppliedControlEvidence3D& control,
                                              const ExecutionOwnerIdentity3D& owner,
                                              const std::int64_t now_ns,
                                              const double maximum_age_ms) noexcept {
  if (!owner.valid || !control.valid || control.producer_instance_id == 0U ||
      control.horizon_producer_instance_id == 0U ||
      control.horizon_producer_instance_id != owner.producer_instance_id ||
      control.producer_instance_id != owner.target_offboard_instance_id ||
      control.horizon_sequence == 0U || control.source_stamp_ns <= 0 ||
      control.receive_stamp_ns <= 0 || now_ns < 0 || !std::isfinite(maximum_age_ms) ||
      !(maximum_age_ms > 0.0)) {
    return false;
  }
  // A planned owner needs authoritative planned feedback of its own horizon or
  // of its immediate predecessor. A stationary hold owner needs the offboard's
  // hold feedback of that same hold: the offboard reports it as
  // non-authoritative because no planned control is being applied, and the
  // previous control of a certified stationary hold is exactly zero.
  const bool planned_pair =
      control.execution_mode == ExecutionAuthorityMode3D::kPlanned &&
      owner.execution_mode == ExecutionAuthorityMode3D::kPlanned &&
      control.control_authoritative &&
      (control.horizon_sequence == owner.sequence ||
       control.horizon_sequence + 1U == owner.sequence);
  const bool hold_pair =
      control.execution_mode == ExecutionAuthorityMode3D::kPositionHold &&
      owner.execution_mode == ExecutionAuthorityMode3D::kPositionHold &&
      control.horizon_sequence == owner.sequence;
  if (!planned_pair && !hold_pair) {
    return false;
  }
  const double source_age_ms =
      static_cast<double>(now_ns - control.source_stamp_ns) * 1.0e-6;
  const double receive_age_ms =
      static_cast<double>(now_ns - control.receive_stamp_ns) * 1.0e-6;
  return std::abs(source_age_ms) <= maximum_age_ms &&
         std::abs(receive_age_ms) <= maximum_age_ms;
}

const char* executionHorizonCommitStatus3DName(
    const ExecutionHorizonCommitStatus3D status) noexcept {
  switch (status) {
    case ExecutionHorizonCommitStatus3D::kCommitted:
      return "committed";
    case ExecutionHorizonCommitStatus3D::kInvalidRequest:
      return "invalid_request";
    case ExecutionHorizonCommitStatus3D::kAuthorityNotCurrent:
      return "authority_not_current";
    case ExecutionHorizonCommitStatus3D::kVehicleStatusNotAuthoritative:
      return "vehicle_status_not_authoritative";
    case ExecutionHorizonCommitStatus3D::kRawWorldNotCurrent:
      return "raw_world_not_current";
    case ExecutionHorizonCommitStatus3D::kHorizonTimeNotCurrent:
      return "horizon_time_not_current";
    case ExecutionHorizonCommitStatus3D::kRevocationEpochChanged:
      return "revocation_epoch_changed";
    case ExecutionHorizonCommitStatus3D::kObjectiveChanged:
      return "objective_changed";
    case ExecutionHorizonCommitStatus3D::kNavigationNotAuthoritative:
      return "navigation_not_authoritative";
    case ExecutionHorizonCommitStatus3D::kOffboardSessionNotCurrent:
      return "offboard_session_not_current";
    case ExecutionHorizonCommitStatus3D::kExecutionInputNotCurrent:
      return "execution_input_not_current";
    case ExecutionHorizonCommitStatus3D::kExecutionInputNotFresh:
      return "execution_input_not_fresh";
    case ExecutionHorizonCommitStatus3D::kEvidenceNotCurrent:
      return "evidence_not_current";
    case ExecutionHorizonCommitStatus3D::kLidarEvidenceNotCurrent:
      return "lidar_evidence_not_current";
    case ExecutionHorizonCommitStatus3D::kControlEvidenceNotCurrent:
      return "control_evidence_not_current";
    case ExecutionHorizonCommitStatus3D::kOwnerInvalid:
      return "owner_invalid";
    case ExecutionHorizonCommitStatus3D::kOwnerPlanIdentityInvalid:
      return "owner_plan_identity_invalid";
    case ExecutionHorizonCommitStatus3D::kLeaseRejected:
      return "lease_rejected";
  }
  return "unknown";
}

ExecutionHorizonCommitResult3D
ExecutionSupervisor3D::commitHorizon(ExecutionHorizonCommitRequest3D request) {
  ExecutionHorizonLeaseCandidate3D& candidate = request.candidate;
  const std::shared_ptr<const CommittedExecutionAuthority3D> resident_authority =
      manager_.authority();
  if (candidate.expected_authority == nullptr || candidate.expected_plan == nullptr ||
      candidate.execution_input == nullptr || request.publication_now_ns <= 0 ||
      candidate.expected_horizon_producer_instance_id == 0U ||
      !std::isfinite(request.maximum_control_feedback_age_ms) ||
      request.maximum_control_feedback_age_ms < 0.0) {
    return reject(ExecutionHorizonCommitStatus3D::kInvalidRequest);
  }
  if (resident_authority == nullptr || !resident_authority->valid() ||
      resident_authority != candidate.expected_authority ||
      resident_authority->plan() != candidate.expected_plan) {
    return reject(ExecutionHorizonCommitStatus3D::kAuthorityNotCurrent);
  }
  const bool progress_valid =
      candidate.progress_preparation != nullptr &&
      candidate.certification_plan != nullptr &&
      candidate.progress_preparation->applied() &&
      candidate.progress_preparation->predecessor == candidate.expected_plan.get() &&
      candidate.progress_preparation->next == candidate.certification_plan;
  const bool certification_base_valid =
      candidate.progress_preparation != nullptr
          ? progress_valid
          : candidate.certification_plan == candidate.expected_plan;
  const std::shared_ptr<const ExecutionPlan3D> publication_plan = [&] {
    switch (candidate.kind) {
      case ExecutionHorizonCommitKind3D::kTransition:
      case ExecutionHorizonCommitKind3D::kPendingTransition:
        return candidate.transition != nullptr && candidate.transition->applied() &&
                       candidate.transition->predecessor ==
                           candidate.expected_plan.get()
                   ? candidate.transition->next
                   : nullptr;
      case ExecutionHorizonCommitKind3D::kUnchangedPlan:
        return candidate.transition == nullptr ? candidate.expected_plan : nullptr;
    }
    return std::shared_ptr<const ExecutionPlan3D>{};
  }();
  const bool pending_shape_valid =
      candidate.kind == ExecutionHorizonCommitKind3D::kPendingTransition
          ? candidate.expected_pending != nullptr
          : candidate.expected_pending == nullptr;
  if (!certification_base_valid || publication_plan == nullptr ||
      !publication_plan->publishable() || !pending_shape_valid) {
    return reject(ExecutionHorizonCommitStatus3D::kInvalidRequest);
  }
  const std::shared_ptr<const VersionedObservedRawWorld3D> expected_raw =
      snapshotRawOwner(*publication_plan);
  const ExecutionOwnerIdentity3D& resident_owner = resident_authority->owner();
  const AppliedControlEvidence3D& resident_control = resident_authority->control();
  if (!request.runtime.vehicle_status_authoritative) {
    return reject(ExecutionHorizonCommitStatus3D::kVehicleStatusNotAuthoritative,
                  resident_owner.valid
                      ? ExecutionHorizonRevocationRequest3D::kUnavailableWorld
                      : ExecutionHorizonRevocationRequest3D::kNone);
  }
  if (!rawWorldCurrent(request.runtime, expected_raw,
                       request.current_observed_raw_world)) {
    return reject(ExecutionHorizonCommitStatus3D::kRawWorldNotCurrent,
                  ExecutionHorizonRevocationRequest3D::kUnavailableWorld);
  }
  if (request.publication_now_ns < candidate.owner.valid_from_ns ||
      request.publication_now_ns >= candidate.owner.valid_until_ns) {
    return reject(ExecutionHorizonCommitStatus3D::kHorizonTimeNotCurrent);
  }
  if (!request.runtime.revocation_epoch_current) {
    return reject(ExecutionHorizonCommitStatus3D::kRevocationEpochChanged);
  }
  if (!request.runtime.objective_current) {
    return reject(ExecutionHorizonCommitStatus3D::kObjectiveChanged);
  }
  if (!request.runtime.navigation_authoritative) {
    return reject(ExecutionHorizonCommitStatus3D::kNavigationNotAuthoritative);
  }
  if (request.runtime.offboard_session_currentness !=
      OffboardSessionPublicationCurrentnessStatus::kCurrent) {
    return reject(ExecutionHorizonCommitStatus3D::kOffboardSessionNotCurrent);
  }
  if (!candidate.execution_input->valid() ||
      !executionInputLineageCurrent(*candidate.execution_input, request.navigation)) {
    return reject(ExecutionHorizonCommitStatus3D::kExecutionInputNotCurrent);
  }

  const std::shared_ptr<const VersionedExecutionValidationPolicy3D> policy =
      snapshotValidationPolicy(*publication_plan);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> publication_lidar =
      snapshotLidarOwner(*publication_plan);
  if (policy == nullptr || !policy->valid() || publication_lidar == nullptr ||
      request.latest_lidar_identity_conflicted) {
    return reject(ExecutionHorizonCommitStatus3D::kLidarEvidenceNotCurrent,
                  ExecutionHorizonRevocationRequest3D::kUnavailableWorld);
  }
  ExecutionHorizonCommitResult3D result;
  result.publication_currentness =
      assessExecutionPublicationCurrentness3D(ExecutionPublicationCurrentnessCheck3D{
          .expected_snapshot = candidate.expected_plan,
          .current_snapshot = resident_authority->plan(),
          .raw_requirement = expected_raw != nullptr
                                 ? ExecutionPublicationRawRequirement3D::kRequired
                                 : ExecutionPublicationRawRequirement3D::kOptional,
          .expected_raw_world = expected_raw,
          .current_raw_world = request.current_observed_raw_world,
          .expected_lidar_evidence = publication_lidar,
          .current_lidar_evidence = request.current_lidar_evidence,
          .publication_now_ns = request.publication_now_ns,
          .maximum_lidar_age_ms = policy->latestLidarMaximumAgeMs(),
          .lidar_freshness_required = policy->latestLidarFreshnessRequired(),
      });
  const bool progress_preserves_route_evidence =
      candidate.progress_preparation == nullptr ||
      (candidate.certification_plan != nullptr &&
       progressPreservesRouteEvidence(*candidate.expected_plan,
                                      *candidate.certification_plan));
  result.latest_evidence_revalidated =
      progress_preserves_route_evidence &&
      result.publication_currentness ==
          ExecutionPublicationCurrentnessStatus3D::kRevalidationRequired &&
      revalidateFiniteExecution(*publication_plan, request.current_observed_raw_world,
                                request.current_lidar_evidence);
  if (result.publication_currentness !=
          ExecutionPublicationCurrentnessStatus3D::kCurrent &&
      !result.latest_evidence_revalidated) {
    result.status = ExecutionHorizonCommitStatus3D::kEvidenceNotCurrent;
    result.revocation_request = ExecutionHorizonRevocationRequest3D::kUnavailableWorld;
    return result;
  }
  if (!executionInputFreshAt(*candidate.execution_input, *policy,
                             request.publication_now_ns)) {
    result.status = ExecutionHorizonCommitStatus3D::kExecutionInputNotFresh;
    result.revocation_request =
        ExecutionHorizonRevocationRequest3D::kNoExecutableHorizon;
    return result;
  }
  if (request.current_lidar_evidence == nullptr ||
      ((publication_lidar->evidenceId() !=
            request.current_lidar_evidence->evidenceId() ||
        publication_lidar->contentFingerprint() !=
            request.current_lidar_evidence->contentFingerprint()) &&
       !result.latest_evidence_revalidated) ||
      (policy->latestLidarFreshnessRequired() &&
       !assessLatestLidarEvidenceFreshness3D(*request.current_lidar_evidence,
                                             request.publication_now_ns,
                                             policy->latestLidarMaximumAgeMs())
            .fresh)) {
    result.status = ExecutionHorizonCommitStatus3D::kLidarEvidenceNotCurrent;
    result.revocation_request = ExecutionHorizonRevocationRequest3D::kUnavailableWorld;
    return result;
  }

  ExecutionOwnerIdentity3D owner = candidate.owner;
  owner.execution_owner_epoch = publication_plan->execution_owner_epoch;
  owner.valid =
      ownerEnumsKnown(owner) &&
      owner.producer_instance_id == candidate.expected_horizon_producer_instance_id &&
      owner.sequence != 0U && owner.valid_from_ns > 0 &&
      owner.valid_until_ns > owner.valid_from_ns &&
      owner.target_offboard_instance_id != 0U;
  if (!owner.valid) {
    result.status = ExecutionHorizonCommitStatus3D::kOwnerInvalid;
    return result;
  }
  const bool capture_rearm = stationaryCaptureRearmCommit(
      candidate, *publication_plan, owner, request.navigation, resident_owner,
      resident_control);
  if (!previousControlCurrent(*candidate.execution_input, request.navigation,
                              resident_control, resident_owner,
                              request.publication_now_ns,
                              request.maximum_control_feedback_age_ms, capture_rearm)) {
    result.status = ExecutionHorizonCommitStatus3D::kControlEvidenceNotCurrent;
    return result;
  }
  if (!owner.validFor(*publication_plan)) {
    result.status = ExecutionHorizonCommitStatus3D::kOwnerPlanIdentityInvalid;
    return result;
  }

  switch (candidate.kind) {
    case ExecutionHorizonCommitKind3D::kTransition:
      result.lease_status =
          manager_.publishLeasedTransition(resident_authority, *candidate.transition,
                                           owner, std::move(candidate.execution_input));
      break;
    case ExecutionHorizonCommitKind3D::kUnchangedPlan:
      result.lease_status = manager_.publishLeaseForUnchangedPlanIfSame(
          resident_authority, candidate.expected_plan, owner,
          std::move(candidate.execution_input));
      break;
    case ExecutionHorizonCommitKind3D::kPendingTransition:
      result.lease_status = manager_.commitPendingLeasedTransition(
          candidate.expected_pending, resident_authority, *candidate.transition, owner,
          std::move(candidate.execution_input));
      break;
  }
  if (result.lease_status != ExecutionRoutePublicationStatus3D::kPublished) {
    result.status = ExecutionHorizonCommitStatus3D::kLeaseRejected;
    return result;
  }
  result.status = ExecutionHorizonCommitStatus3D::kCommitted;
  // A lease that inherits the witnessed control of its predecessor horizon
  // keeps the applied control continuous; only a lease that starts without
  // the resident's witness replaces it.
  const std::shared_ptr<const CommittedExecutionAuthority3D> published =
      manager_.authority();
  result.replaced_applied_control =
      resident_control.valid && (published == nullptr || !published->control().valid);
  return result;
}

} // namespace drone_city_nav
