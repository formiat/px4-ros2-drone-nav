#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <numeric>
#include <ranges>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav {

class ExecutionRouteTransitionFactory3D final {
public:
  [[nodiscard]] static ExecutionRouteTransitionResult3D
  failure(const ExecutionRouteTransitionStatus3D status) {
    return {status, nullptr, nullptr};
  }

  [[nodiscard]] static ExecutionRouteTransitionResult3D
  success(const ExecutionPlan3D& predecessor,
          std::shared_ptr<const ExecutionPlan3D> next) {
    return {ExecutionRouteTransitionStatus3D::kApplied, &predecessor, std::move(next)};
  }
};

namespace execution_route_snapshot_3d_internal {

[[nodiscard]] ExecutionRouteTransitionResult3D
transitionFailure(const ExecutionRouteTransitionStatus3D status) {
  return ExecutionRouteTransitionFactory3D::failure(status);
}

[[nodiscard]] ExecutionRouteTransitionStatus3D
checkCurrentAndVersion(const ExecutionPlan3D& current,
                       const std::uint64_t expected_snapshot_version) noexcept {
  if (!current.valid()) {
    return ExecutionRouteTransitionStatus3D::kInvalidCurrentSnapshot;
  }
  if (current.version != expected_snapshot_version) {
    return ExecutionRouteTransitionStatus3D::kStaleSnapshotVersion;
  }
  if (current.version == std::numeric_limits<std::uint64_t>::max()) {
    return ExecutionRouteTransitionStatus3D::kVersionExhausted;
  }
  return ExecutionRouteTransitionStatus3D::kApplied;
}

[[nodiscard]] ExecutionRouteTransitionStatus3D
checkGuard(const ExecutionPlan3D& current,
           const ExecutionRouteTransitionGuard3D& guard) noexcept {
  const ExecutionRouteTransitionStatus3D base =
      checkCurrentAndVersion(current, guard.expected_snapshot_version);
  if (base != ExecutionRouteTransitionStatus3D::kApplied) {
    return base;
  }
  const CertifiedRouteSuffix3D* const route = current.route();
  if (route == nullptr ||
      route->identity.generation != guard.expected_route_generation) {
    return ExecutionRouteTransitionStatus3D::kRouteGenerationMismatch;
  }
  if (route->geometry->compiled_trajectory_revision !=
      guard.expected_geometry_revision) {
    return ExecutionRouteTransitionStatus3D::kGeometryRevisionMismatch;
  }
  return ExecutionRouteTransitionStatus3D::kApplied;
}

[[nodiscard]] ExecutionRouteTransitionResult3D
finishTransition(const ExecutionPlan3D& current, ExecutionPlan3D next) {
  if (!next.valid()) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }
  return ExecutionRouteTransitionFactory3D::success(
      current, std::make_shared<const ExecutionPlan3D>(std::move(next)));
}

[[nodiscard]] const CertifiedRouteSuffix3D*
routePointer(const ExecutionPlan3D& snapshot) noexcept {
  return snapshot.route();
}

[[nodiscard]] CertifiedRouteSuffix3D* routePointer(ExecutionPlan3D& snapshot) noexcept {
  if (auto* following = std::get_if<FollowingPlan3D>(&snapshot.state)) {
    return std::addressof(following->route);
  }
  if (auto* braking = std::get_if<BrakingPlan3D>(&snapshot.state)) {
    return std::addressof(braking->route);
  }
  if (auto* stationary = std::get_if<StationaryHoldPlan3D>(&snapshot.state)) {
    auto* certified = std::get_if<CertifiedTerminalHoldPlan3D>(&stationary->owner);
    return certified != nullptr ? std::addressof(certified->route) : nullptr;
  }
  auto* awaiting = std::get_if<AwaitingSuccessorPlan3D>(&snapshot.state);
  if (awaiting == nullptr) {
    return nullptr;
  }
  if (auto* suspended = std::get_if<SuspendedRoutePlan3D>(&awaiting->owner)) {
    return std::addressof(suspended->route);
  }
  auto* continuation = std::get_if<ContinuationStopPlan3D>(&awaiting->owner);
  return continuation != nullptr ? std::addressof(continuation->route) : nullptr;
}

FiniteExecutionState3D* finiteExecutionPointer(ExecutionPlan3D& snapshot) noexcept {
  if (auto* following = std::get_if<FollowingPlan3D>(&snapshot.state)) {
    return std::addressof(following->execution.command_horizon);
  }
  if (auto* braking = std::get_if<BrakingPlan3D>(&snapshot.state)) {
    return std::addressof(braking->execution);
  }
  if (auto* stationary = std::get_if<StationaryHoldPlan3D>(&snapshot.state)) {
    auto* certified = std::get_if<CertifiedTerminalHoldPlan3D>(&stationary->owner);
    return certified != nullptr ? std::addressof(certified->execution.command_horizon)
                                : nullptr;
  }
  auto* awaiting = std::get_if<AwaitingSuccessorPlan3D>(&snapshot.state);
  auto* continuation = awaiting != nullptr
                           ? std::get_if<ContinuationStopPlan3D>(&awaiting->owner)
                           : nullptr;
  return continuation != nullptr
             ? std::addressof(continuation->execution.command_horizon)
             : nullptr;
}

FiniteExecutionState3D* brakingFallbackPointer(ExecutionPlan3D& snapshot) noexcept {
  if (auto* following = std::get_if<FollowingPlan3D>(&snapshot.state)) {
    return std::addressof(following->execution.braking_tail);
  }
  if (auto* braking = std::get_if<BrakingPlan3D>(&snapshot.state)) {
    return std::addressof(braking->execution);
  }
  if (auto* stationary = std::get_if<StationaryHoldPlan3D>(&snapshot.state)) {
    auto* certified = std::get_if<CertifiedTerminalHoldPlan3D>(&stationary->owner);
    return certified != nullptr ? std::addressof(certified->execution.braking_tail)
                                : nullptr;
  }
  auto* awaiting = std::get_if<AwaitingSuccessorPlan3D>(&snapshot.state);
  auto* continuation = awaiting != nullptr
                           ? std::get_if<ContinuationStopPlan3D>(&awaiting->owner)
                           : nullptr;
  return continuation != nullptr ? std::addressof(continuation->execution.braking_tail)
                                 : nullptr;
}

[[nodiscard]] bool sameControl(const mppi::Control& first,
                               const mppi::Control& second) noexcept {
  return first.ax == second.ax && first.ay == second.ay && first.az == second.az &&
         first.yaw_accel == second.yaw_accel;
}

[[nodiscard]] bool sameStateExact(const mppi::State& first,
                                  const mppi::State& second) noexcept {
  return first.x == second.x && first.y == second.y && first.z == second.z &&
         first.vx == second.vx && first.vy == second.vy && first.vz == second.vz &&
         first.yaw == second.yaw && first.yaw_rate == second.yaw_rate;
}

[[nodiscard]] bool
sameStateProvenance(const ExecutionStateProvenance3D& first,
                    const ExecutionStateProvenance3D& second) noexcept {
  return first.x == second.x && first.y == second.y && first.z == second.z &&
         first.vx == second.vx && first.vy == second.vy && first.vz == second.vz &&
         first.yaw == second.yaw && first.yaw_rate == second.yaw_rate;
}

[[nodiscard]] bool
sameSourceSampleStateUpdateAllowed(const VersionedExecutionInput3D& candidate,
                                   const VersionedExecutionInput3D& previous) noexcept {
  if (candidate.fullStateAuthoritative() != previous.fullStateAuthoritative() ||
      !sameStateProvenance(candidate.stateProvenance(), previous.stateProvenance())) {
    return false;
  }
  if (candidate.effectiveStampNs() == previous.effectiveStampNs()) {
    return sameStateExact(candidate.state(), previous.state());
  }
  const mppi::State& next = candidate.state();
  const mppi::State& old = previous.state();
  const ExecutionStateProvenance3D& provenance = candidate.stateProvenance();
  const auto field_update_allowed = [](const float next_value, const float old_value,
                                       const ExecutionStateFieldProvenance3D source) {
    return source == ExecutionStateFieldProvenance3D::kEffectiveTimePrediction ||
           next_value == old_value;
  };
  return field_update_allowed(next.x, old.x, provenance.x) &&
         field_update_allowed(next.y, old.y, provenance.y) &&
         field_update_allowed(next.z, old.z, provenance.z) &&
         field_update_allowed(next.vx, old.vx, provenance.vx) &&
         field_update_allowed(next.vy, old.vy, provenance.vy) &&
         field_update_allowed(next.vz, old.vz, provenance.vz) &&
         field_update_allowed(next.yaw, old.yaw, provenance.yaw) &&
         field_update_allowed(next.yaw_rate, old.yaw_rate, provenance.yaw_rate);
}

[[nodiscard]] ExecutionInputProgressRelation3D
executionInputProgressRelation(const VersionedExecutionInput3D& candidate,
                               const VersionedExecutionInput3D& previous) noexcept {
  if (!candidate.valid() || !previous.valid() ||
      candidate.captureSequence() < previous.captureSequence() ||
      candidate.poseRevision() < previous.poseRevision() ||
      candidate.poseSourceTimestampUs() < previous.poseSourceTimestampUs() ||
      candidate.poseReceiveStampNs() < previous.poseReceiveStampNs() ||
      candidate.effectiveStampNs() < previous.effectiveStampNs() ||
      candidate.previousControlSourceStampNs() <
          previous.previousControlSourceStampNs() ||
      candidate.previousControlReceiveStampNs() <
          previous.previousControlReceiveStampNs()) {
    return ExecutionInputProgressRelation3D::kInvalid;
  }
  if (candidate.captureSequence() == previous.captureSequence()) {
    return candidate.contentFingerprint() == previous.contentFingerprint()
               ? ExecutionInputProgressRelation3D::kReplay
               : ExecutionInputProgressRelation3D::kInvalid;
  }
  if (candidate.poseRevision() == previous.poseRevision()) {
    if (candidate.poseSourceTimestampUs() != previous.poseSourceTimestampUs() ||
        candidate.poseReceiveStampNs() != previous.poseReceiveStampNs() ||
        !sameSourceSampleStateUpdateAllowed(candidate, previous)) {
      return ExecutionInputProgressRelation3D::kInvalid;
    }
  } else if (candidate.poseSourceTimestampUs() <= previous.poseSourceTimestampUs() ||
             candidate.poseReceiveStampNs() <= previous.poseReceiveStampNs()) {
    return ExecutionInputProgressRelation3D::kInvalid;
  }
  if (candidate.previousControlSource() == previous.previousControlSource()) {
    if (candidate.previousControlSourceSequence() <
        previous.previousControlSourceSequence()) {
      return ExecutionInputProgressRelation3D::kInvalid;
    }
    if (candidate.previousControlSourceSequence() ==
        previous.previousControlSourceSequence()) {
      const bool same_source_stamp = candidate.previousControlSourceStampNs() ==
                                     previous.previousControlSourceStampNs();
      if (same_source_stamp) {
        if (candidate.previousControlReceiveStampNs() !=
                previous.previousControlReceiveStampNs() ||
            !sameControl(candidate.previousControl(), previous.previousControl())) {
          return ExecutionInputProgressRelation3D::kInvalid;
        }
        return ExecutionInputProgressRelation3D::kStrictlyNewer;
      }
      const bool refreshable_horizon_evidence =
          candidate.previousControlSource() ==
          ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback;
      if (!refreshable_horizon_evidence ||
          candidate.previousControlSourceStampNs() <=
              previous.previousControlSourceStampNs() ||
          candidate.previousControlReceiveStampNs() <=
              previous.previousControlReceiveStampNs()) {
        return ExecutionInputProgressRelation3D::kInvalid;
      }
    } else if (candidate.previousControlSourceStampNs() <=
                   previous.previousControlSourceStampNs() ||
               candidate.previousControlReceiveStampNs() <=
                   previous.previousControlReceiveStampNs()) {
      return ExecutionInputProgressRelation3D::kInvalid;
    }
  } else if (candidate.previousControlSourceStampNs() <=
                 previous.previousControlSourceStampNs() ||
             candidate.previousControlReceiveStampNs() <=
                 previous.previousControlReceiveStampNs()) {
    // Evidence domains have incomparable sequences. A source handoff is
    // therefore authorized only by strictly newer local source/receipt time.
    return ExecutionInputProgressRelation3D::kInvalid;
  }
  return ExecutionInputProgressRelation3D::kStrictlyNewer;
}

[[nodiscard]] bool
executionInputNotOlder(const VersionedExecutionInput3D& candidate,
                       const VersionedExecutionInput3D& previous) noexcept {
  return executionInputProgressRelation(candidate, previous) !=
         ExecutionInputProgressRelation3D::kInvalid;
}

[[nodiscard]] Point3
executionInputPosition(const VersionedExecutionInput3D& input) noexcept {
  return Point3{input.state().x, input.state().y, input.state().z};
}

void bindProgressToExecutionInput(
    CertifiedRouteProgress3D& progress,
    const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
    const double station_m) {
  progress.station_m = station_m;
  progress.last_observed_position = executionInputPosition(*execution_input);
  progress.execution_input = execution_input;
}

[[nodiscard]] bool
latestLidarEvidenceNotOlder(const VersionedLatestLidarEvidence3D& candidate,
                            const VersionedLatestLidarEvidence3D& previous) noexcept {
  if (candidate.producerInstanceId() != previous.producerInstanceId() ||
      candidate.sequence() < previous.sequence() ||
      candidate.poseGeneration() < previous.poseGeneration() ||
      candidate.receiveStampNs() < previous.receiveStampNs()) {
    return false;
  }
  const bool same_identity =
      candidate.sequence() == previous.sequence() &&
      candidate.acquisitionStampNs() == previous.acquisitionStampNs();
  if (same_identity) {
    return candidate.contentFingerprint() == previous.contentFingerprint();
  }
  return candidate.sequence() > previous.sequence() &&
         candidate.acquisitionStampNs() > previous.acquisitionStampNs();
}

[[nodiscard]] bool
directTrackingWorldNotOlder(const DirectTrackingFiniteExecution3D& candidate,
                            const DirectTrackingFiniteExecution3D& previous) noexcept {
  if (candidate.validation_policy == nullptr || previous.validation_policy == nullptr ||
      candidate.validation_policy->contentFingerprint() !=
          previous.validation_policy->contentFingerprint()) {
    return false;
  }
  if (previous.observed_raw_world != nullptr) {
    if (candidate.observed_raw_world == nullptr || candidate.static_world != nullptr ||
        candidate.observed_raw_world->version().producer_instance_id !=
            previous.observed_raw_world->version().producer_instance_id ||
        candidate.observed_raw_world->version().revision <
            previous.observed_raw_world->version().revision) {
      return false;
    }
    return candidate.observed_raw_world->version().revision !=
               previous.observed_raw_world->version().revision ||
           candidate.observed_raw_world->contentFingerprint() ==
               previous.observed_raw_world->contentFingerprint() ||
           candidate.observed_raw_world->sharesObservationOwner(
               *previous.observed_raw_world);
  }
  return candidate.observed_raw_world == nullptr && candidate.static_world != nullptr &&
         previous.static_world != nullptr &&
         staticWorldNotOlder(*candidate.static_world, *previous.static_world);
}

[[nodiscard]] bool directTrackingExecutionNotOlder(
    const DirectTrackingFiniteExecution3D& candidate,
    const DirectTrackingFiniteExecution3D& previous) noexcept {
  if (!candidate.valid() || !previous.valid() ||
      !sameDirectTrackingOwner(candidate.identity, previous.identity) ||
      candidate.identity.objective_sample_sequence <
          previous.identity.objective_sample_sequence ||
      candidate.identity.line_of_sight_generation <
          previous.identity.line_of_sight_generation ||
      candidate.trajectory_revision <= previous.trajectory_revision ||
      candidate.source_navigation_revision < previous.source_navigation_revision ||
      candidate.valid_from_ns < previous.valid_from_ns ||
      candidate.execution_input == nullptr || previous.execution_input == nullptr ||
      !executionInputNotOlder(*candidate.execution_input, *previous.execution_input) ||
      candidate.latest_lidar_evidence == nullptr ||
      previous.latest_lidar_evidence == nullptr ||
      !latestLidarEvidenceNotOlder(*candidate.latest_lidar_evidence,
                                   *previous.latest_lidar_evidence) ||
      !directTrackingWorldNotOlder(candidate, previous)) {
    return false;
  }
  return candidate.identity.objective_sample_sequence !=
             previous.identity.objective_sample_sequence ||
         candidate.identity.line_of_sight_generation !=
             previous.identity.line_of_sight_generation ||
         samePointExact(candidate.target, previous.target);
}

[[nodiscard]] bool directTrackingEvidenceNotOlderThanRoute(
    const DirectTrackingFiniteExecution3D& candidate,
    const FiniteExecutionState3D& previous) noexcept {
  if (candidate.validation_policy == nullptr || previous.validation_policy == nullptr ||
      candidate.validation_policy->contentFingerprint() !=
          previous.validation_policy->contentFingerprint() ||
      candidate.trajectory_revision <= previous.trajectory_revision ||
      candidate.execution_input == nullptr || previous.execution_input == nullptr ||
      !executionInputNotOlder(*candidate.execution_input, *previous.execution_input) ||
      candidate.latest_lidar_evidence == nullptr ||
      previous.latest_lidar_evidence == nullptr ||
      !latestLidarEvidenceNotOlder(*candidate.latest_lidar_evidence,
                                   *previous.latest_lidar_evidence)) {
    return false;
  }
  if (previous.observed_raw_world != nullptr) {
    return candidate.observed_raw_world != nullptr &&
           candidate.static_world == nullptr &&
           candidate.observed_raw_world->version().producer_instance_id ==
               previous.observed_raw_world->version().producer_instance_id &&
           candidate.observed_raw_world->version().revision >=
               previous.observed_raw_world->version().revision &&
           (candidate.observed_raw_world->version().revision !=
                previous.observed_raw_world->version().revision ||
            candidate.observed_raw_world->contentFingerprint() ==
                previous.observed_raw_world->contentFingerprint() ||
            candidate.observed_raw_world->sharesObservationOwner(
                *previous.observed_raw_world));
  }
  return candidate.observed_raw_world == nullptr && candidate.static_world != nullptr &&
         previous.static_world != nullptr &&
         staticWorldNotOlder(*candidate.static_world, *previous.static_world);
}

[[nodiscard]] bool routeExecutionEvidenceNotOlderThanDirect(
    const FiniteExecutionState3D& candidate,
    const DirectTrackingFiniteExecution3D& previous) noexcept {
  if (candidate.validation_policy == nullptr || previous.validation_policy == nullptr ||
      candidate.validation_policy->contentFingerprint() !=
          previous.validation_policy->contentFingerprint() ||
      candidate.trajectory_revision <= previous.trajectory_revision ||
      candidate.execution_input == nullptr || previous.execution_input == nullptr ||
      !executionInputNotOlder(*candidate.execution_input, *previous.execution_input) ||
      candidate.latest_lidar_evidence == nullptr ||
      previous.latest_lidar_evidence == nullptr ||
      !latestLidarEvidenceNotOlder(*candidate.latest_lidar_evidence,
                                   *previous.latest_lidar_evidence)) {
    return false;
  }
  if (previous.observed_raw_world != nullptr) {
    return candidate.observed_raw_world != nullptr &&
           candidate.static_world == nullptr &&
           candidate.observed_raw_world->version().producer_instance_id ==
               previous.observed_raw_world->version().producer_instance_id &&
           candidate.observed_raw_world->version().revision >=
               previous.observed_raw_world->version().revision &&
           (candidate.observed_raw_world->version().revision !=
                previous.observed_raw_world->version().revision ||
            candidate.observed_raw_world->contentFingerprint() ==
                previous.observed_raw_world->contentFingerprint() ||
            candidate.observed_raw_world->sharesObservationOwner(
                *previous.observed_raw_world));
  }
  return candidate.observed_raw_world == nullptr && candidate.static_world != nullptr &&
         previous.static_world != nullptr &&
         staticWorldNotOlder(*candidate.static_world, *previous.static_world);
}

namespace {

[[nodiscard]] bool executionWorldNotOlderThanHold(
    const std::shared_ptr<const VersionedObservedRawWorld3D>& candidate_observed,
    const std::shared_ptr<const VersionedStaticWorld3D>& candidate_static,
    const StationaryExecutionHold3D& previous) noexcept {
  if (previous.observed_raw_world != nullptr) {
    return candidate_observed != nullptr && candidate_static == nullptr &&
           candidate_observed->valid() &&
           candidate_observed->version().producer_instance_id ==
               previous.observed_raw_world->version().producer_instance_id &&
           candidate_observed->version().revision >=
               previous.observed_raw_world->version().revision &&
           (candidate_observed->version().revision !=
                previous.observed_raw_world->version().revision ||
            candidate_observed->contentFingerprint() ==
                previous.observed_raw_world->contentFingerprint() ||
            candidate_observed->sharesObservationOwner(*previous.observed_raw_world));
  }
  return previous.static_world != nullptr && candidate_observed == nullptr &&
         candidate_static != nullptr && candidate_static->valid() &&
         staticWorldNotOlder(*candidate_static, *previous.static_world);
}

template<typename Candidate>
[[nodiscard]] bool
executionEvidenceNotOlderThanHold(const Candidate& candidate,
                                  const StationaryExecutionHold3D& previous) noexcept {
  return previous.valid() && candidate.validation_policy != nullptr &&
         previous.validation_policy != nullptr &&
         candidate.validation_policy->contentFingerprint() ==
             previous.validation_policy->contentFingerprint() &&
         candidate.trajectory_revision > previous.source_trajectory_revision &&
         candidate.execution_input != nullptr &&
         previous.terminal_execution_input != nullptr &&
         executionInputNotOlder(*candidate.execution_input,
                                *previous.terminal_execution_input) &&
         distance3D(executionInputPosition(*candidate.execution_input),
                    previous.position) <= kStationaryExecutionHoldPositionToleranceM &&
         candidate.latest_lidar_evidence != nullptr &&
         previous.latest_lidar_evidence != nullptr &&
         latestLidarEvidenceNotOlder(*candidate.latest_lidar_evidence,
                                     *previous.latest_lidar_evidence) &&
         executionWorldNotOlderThanHold(candidate.observed_raw_world,
                                        candidate.static_world, previous);
}

} // namespace

bool routeExecutionEvidenceNotOlderThanHold(
    const FiniteExecutionState3D& candidate,
    const StationaryExecutionHold3D& previous) noexcept {
  return executionEvidenceNotOlderThanHold(candidate, previous);
}

bool directTrackingEvidenceNotOlderThanHold(
    const DirectTrackingFiniteExecution3D& candidate,
    const StationaryExecutionHold3D& previous) noexcept {
  return executionEvidenceNotOlderThanHold(candidate, previous);
}

[[nodiscard]] bool
candidateFiniteExecutionValid(const FiniteExecutionState3D& candidate,
                              const ExecutionPlan3D& current,
                              const CertifiedRouteSuffix3D* route,
                              const bool require_current_certificate) noexcept {
  if (candidate.source_snapshot_version != current.version ||
      candidate.revalidation_required || !candidate.validFor(route)) {
    return false;
  }
  if (route != nullptr &&
      std::abs(candidate.begin_route_station_m - route->progress.station_m) >
          kExecutionBindingToleranceM) {
    return false;
  }
  if (route != nullptr &&
      (candidate.horizon == nullptr || candidate.horizon->states.empty() ||
       distance3D(Point3{candidate.horizon->states.front().x,
                         candidate.horizon->states.front().y,
                         candidate.horizon->states.front().z},
                  route->progress.last_observed_position) >
           kExecutionBindingToleranceM)) {
    return false;
  }
  const FiniteExecutionState3D* const current_execution = current.finiteExecution();
  if (current_execution != nullptr &&
      (candidate.trajectory_revision <= current_execution->trajectory_revision ||
       candidate.source_navigation_revision <
           current_execution->source_navigation_revision ||
       candidate.valid_from_ns < current_execution->valid_from_ns ||
       (candidate.kind != FiniteExecutionKind3D::kNominal &&
        !finiteExecutionValidatedAgainstNewerRawWorld(candidate) &&
        candidate.valid_until_ns > current_execution->valid_until_ns))) {
    return false;
  }
  if (current_execution != nullptr) {
    const FiniteExecutionState3D& previous = *current_execution;
    if (candidate.execution_input == nullptr || previous.execution_input == nullptr ||
        !executionInputNotOlder(*candidate.execution_input,
                                *previous.execution_input)) {
      return false;
    }
    if (previous.latest_lidar_evidence != nullptr &&
        (candidate.latest_lidar_evidence == nullptr ||
         !latestLidarEvidenceNotOlder(*candidate.latest_lidar_evidence,
                                      *previous.latest_lidar_evidence))) {
      return false;
    }
  }
  return !require_current_certificate ||
         (route != nullptr &&
          sameCertificate(candidate.certificate, route->certificate));
}

[[nodiscard]] bool successorRouteEvidenceNotOlder(
    const CertifiedRouteSuffix3D& current_route,
    const CertifiedRouteSuffix3D& successor,
    const FiniteExecutionState3D& successor_execution) noexcept {
  if (current_route.validation_policy == nullptr ||
      successor.validation_policy == nullptr ||
      current_route.validation_policy->contentFingerprint() !=
          successor.validation_policy->contentFingerprint()) {
    return false;
  }

  const auto* const current_raw =
      std::get_if<ObservedRawRouteCertificate3D>(&current_route.certificate);
  const auto* const successor_raw =
      std::get_if<ObservedRawRouteCertificate3D>(&successor.certificate);
  if (current_raw != nullptr) {
    const auto* const successor_execution_raw =
        std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
            &successor_execution.validation_proof.lineage);
    if (successor_raw == nullptr || successor_execution_raw == nullptr ||
        successor_raw->producer_instance_id != current_raw->producer_instance_id ||
        successor_execution_raw->producer_instance_id !=
            successor_raw->producer_instance_id ||
        successor_raw->validated_through_revision <
            current_raw->validated_through_revision ||
        successor_execution_raw->validated_through_raw_revision <
            current_raw->validated_through_revision) {
      return false;
    }
    return successor_raw->validated_through_revision !=
               current_raw->validated_through_revision ||
           successor_raw->observed_world_content_fingerprint ==
               current_raw->observed_world_content_fingerprint;
  }

  const auto* const current_static =
      std::get_if<StaticRouteCertificate3D>(&current_route.certificate);
  const auto* const successor_static =
      std::get_if<StaticRouteCertificate3D>(&successor.certificate);
  if (current_static == nullptr || successor_static == nullptr ||
      !std::holds_alternative<StaticFiniteExecutionValidationLineage3D>(
          successor_execution.validation_proof.lineage)) {
    return false;
  }
  const NavigationWorldCertificate3D& current_world = current_static->world_certificate;
  const NavigationWorldCertificate3D& successor_world =
      successor_static->world_certificate;
  const bool nondecreasing =
      successor_world.producer_instance_id == current_world.producer_instance_id &&
      successor_world.esdf_source_raw_revision >=
          current_world.esdf_source_raw_revision &&
      successor_world.raw_validated_through_revision >=
          current_world.raw_validated_through_revision &&
      successor_world.local_world_generation >= current_world.local_world_generation &&
      successor_world.topology_revision >= current_world.topology_revision;
  if (!nondecreasing) {
    return false;
  }
  const bool strictly_advanced =
      successor_world.esdf_source_raw_revision >
          current_world.esdf_source_raw_revision ||
      successor_world.raw_validated_through_revision >
          current_world.raw_validated_through_revision ||
      successor_world.local_world_generation > current_world.local_world_generation ||
      successor_world.topology_revision > current_world.topology_revision;
  return strictly_advanced ||
         (sameWorldCertificate(successor_world, current_world) &&
          successor_static->static_occupancy_content_fingerprint ==
              current_static->static_occupancy_content_fingerprint);
}

[[nodiscard]] bool
successorEvidenceNotOlder(const CertifiedRouteSuffix3D& current_route,
                          const FiniteExecutionState3D& current_execution,
                          const CertifiedRouteSuffix3D& successor,
                          const FiniteExecutionState3D& successor_execution) noexcept {
  if (current_route.validation_policy == nullptr ||
      successor.validation_policy == nullptr ||
      current_route.validation_policy->contentFingerprint() !=
          successor.validation_policy->contentFingerprint()) {
    return false;
  }
  const auto* const current_raw =
      std::get_if<ObservedRawRouteCertificate3D>(&current_route.certificate);
  const auto* const successor_raw =
      std::get_if<ObservedRawRouteCertificate3D>(&successor.certificate);
  if (current_raw != nullptr) {
    const auto* const current_execution_raw =
        std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
            &current_execution.validation_proof.lineage);
    const auto* const successor_execution_raw =
        std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
            &successor_execution.validation_proof.lineage);
    if (successor_raw == nullptr || current_execution_raw == nullptr ||
        successor_execution_raw == nullptr ||
        current_execution_raw->producer_instance_id !=
            current_raw->producer_instance_id ||
        successor_execution_raw->producer_instance_id !=
            successor_raw->producer_instance_id) {
      return false;
    }
    if (successor_raw->producer_instance_id != current_raw->producer_instance_id) {
      // A producer switch requires an authoritative latest-world token. Until
      // publication supplies that token, fail closed instead of treating an
      // arbitrary internally consistent producer ID as fresh evidence.
      return false;
    }
    const std::uint64_t required_raw_revision =
        std::max(current_raw->validated_through_revision,
                 current_execution_raw->validated_through_raw_revision);
    if (successor_raw->validated_through_revision < required_raw_revision ||
        successor_execution_raw->validated_through_raw_revision <
            required_raw_revision) {
      return false;
    }
    if (successor_raw->validated_through_revision ==
            current_raw->validated_through_revision &&
        successor_raw->observed_world_content_fingerprint !=
            current_raw->observed_world_content_fingerprint) {
      return false;
    }
    return successor_raw->validated_through_revision !=
               current_execution_raw->validated_through_raw_revision ||
           successor_raw->observed_world_content_fingerprint ==
               current_execution_raw->observed_world_content_fingerprint;
  }

  const auto* const current_static =
      std::get_if<StaticRouteCertificate3D>(&current_route.certificate);
  const auto* const successor_static =
      std::get_if<StaticRouteCertificate3D>(&successor.certificate);
  if (current_static == nullptr || successor_static == nullptr ||
      !std::holds_alternative<StaticFiniteExecutionValidationLineage3D>(
          current_execution.validation_proof.lineage) ||
      !std::holds_alternative<StaticFiniteExecutionValidationLineage3D>(
          successor_execution.validation_proof.lineage)) {
    return false;
  }
  const NavigationWorldCertificate3D& current_world = current_static->world_certificate;
  const NavigationWorldCertificate3D& successor_world =
      successor_static->world_certificate;
  const bool nondecreasing =
      successor_world.producer_instance_id == current_world.producer_instance_id &&
      successor_world.esdf_source_raw_revision >=
          current_world.esdf_source_raw_revision &&
      successor_world.raw_validated_through_revision >=
          current_world.raw_validated_through_revision &&
      successor_world.local_world_generation >= current_world.local_world_generation &&
      successor_world.topology_revision >= current_world.topology_revision;
  if (!nondecreasing) {
    return false;
  }
  const bool strictly_advanced =
      successor_world.esdf_source_raw_revision >
          current_world.esdf_source_raw_revision ||
      successor_world.raw_validated_through_revision >
          current_world.raw_validated_through_revision ||
      successor_world.local_world_generation > current_world.local_world_generation ||
      successor_world.topology_revision > current_world.topology_revision;
  return strictly_advanced ||
         (sameWorldCertificate(successor_world, current_world) &&
          successor_static->static_occupancy_content_fingerprint ==
              current_static->static_occupancy_content_fingerprint);
}

} // namespace execution_route_snapshot_3d_internal
} // namespace drone_city_nav
