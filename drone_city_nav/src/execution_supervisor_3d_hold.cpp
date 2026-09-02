#include "drone_city_nav/execution_hold_3d.hpp"
#include "drone_city_nav/execution_supervisor_3d.hpp"

#include <memory>
#include <utility>

namespace drone_city_nav {
namespace {

struct HoldSourceEvidence3D {
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
};

[[nodiscard]] bool validHoldIntent(const ExecutionHoldIntent3D intent) noexcept {
  switch (intent) {
    case ExecutionHoldIntent3D::kRefreshResident:
    case ExecutionHoldIntent3D::kExplicitTransfer:
    case ExecutionHoldIntent3D::kExplicitTransferWithStationaryCaptureRearm:
      return true;
  }
  return false;
}

[[nodiscard]] HoldSourceEvidence3D
residentHoldSourceEvidence(const ExecutionPlan3D& plan) {
  if (const StationaryExecutionHold3D* const hold = plan.stationaryHold()) {
    return {
        .observed_raw_world = hold->observed_raw_world,
        .static_world = hold->static_world,
        .validation_policy = hold->validation_policy,
    };
  }
  if (const FiniteExecutionState3D* const route = plan.finiteExecution()) {
    return {
        .observed_raw_world = route->observed_raw_world,
        .static_world = route->static_world,
        .validation_policy = route->validation_policy,
    };
  }
  if (const DirectTrackingFiniteExecution3D* const direct =
          plan.directTrackingExecution()) {
    return {
        .observed_raw_world = direct->observed_raw_world,
        .static_world = direct->static_world,
        .validation_policy = direct->validation_policy,
    };
  }
  return {};
}

[[nodiscard]] bool
samePolicy(const std::shared_ptr<const VersionedExecutionValidationPolicy3D>& first,
           const std::shared_ptr<const VersionedExecutionValidationPolicy3D>&
               second) noexcept {
  return first != nullptr && second != nullptr && first->valid() && second->valid() &&
         first->policyId() == second->policyId() &&
         first->contentFingerprint() == second->contentFingerprint();
}

[[nodiscard]] bool
stationaryCaptureWorldCurrent(const ExecutionHoldRequest3D& request) noexcept {
  const bool observed = request.stationary_capture_observed_raw_world != nullptr;
  const bool static_world = request.stationary_capture_static_world != nullptr;
  if (observed == static_world) {
    return false;
  }
  if (static_world) {
    return request.stationary_capture_static_world->valid();
  }
  // The captured world must be on the current raw lineage: same producer and
  // not newer than the resident raw world. A revision that arrived while the
  // tick was running does not invalidate a hold at the vehicle's own position;
  // the commit revalidates the hold against compatible newer evidence.
  const VersionedObservedRawWorld3D& capture =
      *request.stationary_capture_observed_raw_world;
  const VersionedObservedRawWorld3D* const current =
      request.current_observed_raw_world.get();
  return !request.raw_world_identity_conflicted && capture.valid() &&
         current != nullptr && current->valid() &&
         capture.version().sameLineage(current->version()) &&
         capture.version().revision <= current->version().revision;
}

[[nodiscard]] bool
latestLidarCurrent(const ExecutionHoldRequest3D& request,
                   const VersionedExecutionValidationPolicy3D& policy) noexcept {
  // The captured scan must be on the current lidar lineage: same producer and
  // not newer than the scan installed now. A scan that arrived while the tick
  // was running is used by the next tick; it does not retract this one.
  if (request.latest_lidar_identity_conflicted ||
      request.latest_lidar_evidence == nullptr ||
      request.current_lidar_evidence == nullptr ||
      !request.latest_lidar_evidence->valid() ||
      !request.current_lidar_evidence->valid() ||
      request.latest_lidar_evidence->producerInstanceId() !=
          request.current_lidar_evidence->producerInstanceId() ||
      request.latest_lidar_evidence->sequence() >
          request.current_lidar_evidence->sequence()) {
    return false;
  }
  return !policy.latestLidarFreshnessRequired() ||
         assessLatestLidarEvidenceFreshness3D(*request.latest_lidar_evidence,
                                              request.validation_now_ns,
                                              policy.latestLidarMaximumAgeMs())
             .fresh;
}

[[nodiscard]] bool emptyRevokedPlan(const ExecutionPlan3D& plan) noexcept {
  return plan.phase() == ExecutionRoutePhase3D::kRevoked && plan.route() == nullptr &&
         plan.finiteExecution() == nullptr &&
         plan.directTrackingExecution() == nullptr && plan.stationaryHold() == nullptr;
}

[[nodiscard]] bool exclusiveStationaryHold(const ExecutionPlan3D& plan) noexcept {
  return plan.stationaryHold() != nullptr && plan.route() == nullptr &&
         plan.finiteExecution() == nullptr && plan.directTrackingExecution() == nullptr;
}

[[nodiscard]] ExecutionHoldPreparation3D prepareStationaryCaptureRearm(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const std::shared_ptr<const ExecutionPlan3D>& expected,
    const ExecutionHoldRequest3D& request) {
  ExecutionHoldPreparation3D result;
  result.expected_authority = expected_authority;
  result.stationary_capture_rearm = true;
  if (request.intent !=
          ExecutionHoldIntent3D::kExplicitTransferWithStationaryCaptureRearm ||
      !emptyRevokedPlan(*expected)) {
    result.status = ExecutionHoldPreparationStatus3D::kIntentNotApplicable;
    return result;
  }
  if (request.execution_input == nullptr ||
      !request.execution_input->stationaryCaptureStateAuthoritative()) {
    result.status = ExecutionHoldPreparationStatus3D::kExecutionInputInvalid;
    return result;
  }
  if (!samePolicy(request.selected_validation_policy,
                  request.stationary_capture_validation_policy) ||
      !stationaryCaptureWorldCurrent(request)) {
    result.status = ExecutionHoldPreparationStatus3D::kValidationWorldUnavailable;
    return result;
  }
  if (!latestLidarCurrent(request, *request.stationary_capture_validation_policy)) {
    result.status = ExecutionHoldPreparationStatus3D::kLidarEvidenceNotCurrent;
    return result;
  }
  const ExecutionRouteTransitionResult3D transition = armStationaryCaptureHold3D(
      *expected, expected->version,
      StationaryExecutionHoldCertification3D{
          .position = request.requested_position,
          .execution_input = request.execution_input,
          .observed_raw_world = request.stationary_capture_observed_raw_world,
          .static_world = request.stationary_capture_static_world,
          .validation_policy = request.stationary_capture_validation_policy,
          .latest_lidar_evidence = request.latest_lidar_evidence,
      });
  result.transition_status = transition.status;
  if (!transition.applied() || transition.next == nullptr ||
      !exclusiveStationaryHold(*transition.next) ||
      transition.next->stationaryHold()->origin !=
          StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm) {
    result.status = ExecutionHoldPreparationStatus3D::kTransitionRejected;
    return result;
  }
  result.kind = ExecutionHoldPreparationKind3D::kTransition;
  result.transition =
      std::make_shared<const ExecutionRouteTransitionResult3D>(transition);
  result.position = transition.next->stationaryHold()->position;
  result.status = ExecutionHoldPreparationStatus3D::kPrepared;
  return result;
}

[[nodiscard]] ExecutionHoldPreparation3D prepareResidentHold(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const std::shared_ptr<const ExecutionPlan3D>& expected,
    const ExecutionHoldRequest3D& request) {
  ExecutionHoldPreparation3D result;
  result.expected_authority = expected_authority;
  const StationaryExecutionHold3D* const resident_hold = expected->stationaryHold();
  if (request.intent == ExecutionHoldIntent3D::kRefreshResident &&
      resident_hold == nullptr) {
    result.status = ExecutionHoldPreparationStatus3D::kIntentNotApplicable;
    return result;
  }
  if (request.execution_input == nullptr ||
      !request.execution_input->nominalStateAuthoritative()) {
    result.status = ExecutionHoldPreparationStatus3D::kExecutionInputInvalid;
    return result;
  }
  const HoldSourceEvidence3D source = residentHoldSourceEvidence(*expected);
  if (source.validation_policy == nullptr || !source.validation_policy->valid() ||
      (source.observed_raw_world == nullptr) == (source.static_world == nullptr) ||
      (request.selected_validation_policy != nullptr &&
       !samePolicy(request.selected_validation_policy, source.validation_policy))) {
    result.status = ExecutionHoldPreparationStatus3D::kValidationWorldUnavailable;
    return result;
  }
  std::shared_ptr<const VersionedObservedRawWorld3D> current_observed;
  if (source.observed_raw_world != nullptr) {
    if (request.raw_world_identity_conflicted ||
        request.current_observed_raw_world == nullptr ||
        !request.current_observed_raw_world->valid() ||
        request.current_observed_raw_world->version().producer_instance_id !=
            source.observed_raw_world->version().producer_instance_id) {
      result.status = ExecutionHoldPreparationStatus3D::kValidationWorldUnavailable;
      return result;
    }
    current_observed = request.current_observed_raw_world;
  }
  if (!latestLidarCurrent(request, *source.validation_policy)) {
    result.status = ExecutionHoldPreparationStatus3D::kLidarEvidenceNotCurrent;
    return result;
  }

  const Point3 position =
      resident_hold != nullptr &&
              request.intent == ExecutionHoldIntent3D::kRefreshResident
          ? resident_hold->position
          : request.requested_position;
  const ExecutionRouteTransitionResult3D transition = transferToExecutionHold3D(
      *expected, expected->version,
      StationaryExecutionHoldCertification3D{
          .position = position,
          .execution_input = request.execution_input,
          .observed_raw_world = std::move(current_observed),
          .static_world = source.static_world,
          .validation_policy = source.validation_policy,
          .latest_lidar_evidence = request.latest_lidar_evidence,
      });
  result.transition_status = transition.status;
  const std::shared_ptr<const ExecutionPlan3D> prepared_plan =
      transition.applied() ? transition.next : expected;
  if ((transition.applied() && transition.next != nullptr) ||
      transition.status == ExecutionRouteTransitionStatus3D::kNoChange) {
    if (prepared_plan == nullptr || !exclusiveStationaryHold(*prepared_plan) ||
        prepared_plan->stationaryHold()->terminal_execution_input !=
            request.execution_input) {
      result.status = ExecutionHoldPreparationStatus3D::kTransitionRejected;
      return result;
    }
    result.kind = transition.applied() ? ExecutionHoldPreparationKind3D::kTransition
                                       : ExecutionHoldPreparationKind3D::kUnchangedPlan;
    if (transition.applied()) {
      result.transition =
          std::make_shared<const ExecutionRouteTransitionResult3D>(transition);
    }
    result.position = prepared_plan->stationaryHold()->position;
    result.status = ExecutionHoldPreparationStatus3D::kPrepared;
    return result;
  }
  result.status = ExecutionHoldPreparationStatus3D::kTransitionRejected;
  return result;
}

} // namespace

const char*
executionHoldPreparationKind3DName(const ExecutionHoldPreparationKind3D kind) noexcept {
  switch (kind) {
    case ExecutionHoldPreparationKind3D::kNone:
      return "none";
    case ExecutionHoldPreparationKind3D::kTransition:
      return "transition";
    case ExecutionHoldPreparationKind3D::kUnchangedPlan:
      return "unchanged_plan";
  }
  return "unknown";
}

const char* executionHoldPreparationStatus3DName(
    const ExecutionHoldPreparationStatus3D status) noexcept {
  switch (status) {
    case ExecutionHoldPreparationStatus3D::kPrepared:
      return "prepared";
    case ExecutionHoldPreparationStatus3D::kMissingAuthority:
      return "missing_authority";
    case ExecutionHoldPreparationStatus3D::kSourceNotCurrent:
      return "source_not_current";
    case ExecutionHoldPreparationStatus3D::kIntentNotApplicable:
      return "intent_not_applicable";
    case ExecutionHoldPreparationStatus3D::kExecutionInputInvalid:
      return "execution_input_invalid";
    case ExecutionHoldPreparationStatus3D::kLidarEvidenceNotCurrent:
      return "lidar_evidence_not_current";
    case ExecutionHoldPreparationStatus3D::kValidationWorldUnavailable:
      return "validation_world_unavailable";
    case ExecutionHoldPreparationStatus3D::kTransitionRejected:
      return "transition_rejected";
  }
  return "unknown";
}

bool ExecutionHoldPreparation3D::prepared() const noexcept {
  const std::shared_ptr<const ExecutionPlan3D> expected = expectedPlan();
  const std::shared_ptr<const ExecutionPlan3D> plan = preparedPlan();
  const bool transition_valid =
      (kind == ExecutionHoldPreparationKind3D::kTransition && transition != nullptr &&
       transition_status == ExecutionRouteTransitionStatus3D::kApplied &&
       transition->applied() && transition->predecessor == expected.get()) ||
      (kind == ExecutionHoldPreparationKind3D::kUnchangedPlan &&
       transition_status == ExecutionRouteTransitionStatus3D::kNoChange &&
       transition == nullptr && plan == expected);
  const StationaryExecutionHold3D* const hold =
      plan != nullptr ? plan->stationaryHold() : nullptr;
  return status == ExecutionHoldPreparationStatus3D::kPrepared && transition_valid &&
         expected_authority != nullptr && expected_authority->valid() &&
         expected != nullptr && plan != nullptr && plan->publishable() &&
         exclusiveStationaryHold(*plan) && hold != nullptr &&
         hold->terminal_execution_input != nullptr &&
         hold->terminal_execution_input->valid() &&
         (!stationary_capture_rearm ||
          hold->origin == StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm) &&
         hold->position.x == position.x && hold->position.y == position.y &&
         hold->position.z == position.z;
}

std::shared_ptr<const ExecutionPlan3D>
ExecutionHoldPreparation3D::expectedPlan() const noexcept {
  return expected_authority != nullptr ? expected_authority->plan() : nullptr;
}

std::shared_ptr<const ExecutionPlan3D>
ExecutionHoldPreparation3D::preparedPlan() const noexcept {
  return transition != nullptr ? transition->next : expectedPlan();
}

std::shared_ptr<const VersionedExecutionInput3D>
ExecutionHoldPreparation3D::executionInput() const noexcept {
  const std::shared_ptr<const ExecutionPlan3D> plan = preparedPlan();
  const StationaryExecutionHold3D* const hold =
      plan != nullptr ? plan->stationaryHold() : nullptr;
  return hold != nullptr ? hold->terminal_execution_input : nullptr;
}

ExecutionHoldPreparation3D
ExecutionSupervisor3D::prepareHold(ExecutionHoldRequest3D request) const {
  const ExecutionHoldRequest3D owned_request{std::move(request)};
  const std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority =
      manager_.authority();
  const std::shared_ptr<const ExecutionPlan3D> expected =
      expected_authority != nullptr ? expected_authority->plan() : nullptr;
  if (expected_authority == nullptr || !expected_authority->valid() ||
      expected == nullptr) {
    return {};
  }
  if (owned_request.cycle_source_plan != expected) {
    ExecutionHoldPreparation3D stale;
    stale.status = ExecutionHoldPreparationStatus3D::kSourceNotCurrent;
    stale.expected_authority = expected_authority;
    return stale;
  }
  if (!validHoldIntent(owned_request.intent)) {
    ExecutionHoldPreparation3D invalid_intent;
    invalid_intent.status = ExecutionHoldPreparationStatus3D::kIntentNotApplicable;
    invalid_intent.expected_authority = expected_authority;
    return invalid_intent;
  }
  if (owned_request.execution_input != nullptr &&
      owned_request.execution_input->stationaryCaptureStateAuthoritative()) {
    return prepareStationaryCaptureRearm(expected_authority, expected, owned_request);
  }
  return prepareResidentHold(expected_authority, expected, owned_request);
}

} // namespace drone_city_nav
