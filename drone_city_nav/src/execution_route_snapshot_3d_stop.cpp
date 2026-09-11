#include "drone_city_nav/execution_route_certification_3d.hpp"
#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"
#include "drone_city_nav/raw_occupancy_clearance_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav {

using namespace execution_route_snapshot_3d_internal;

namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  return value.has_value() ? std::addressof(value.value()) : nullptr;
}

void hashStopHorizon(std::uint64_t& hash, const FiniteMotionHorizon3D& horizon) {
  hashValue(hash, static_cast<std::uint64_t>(horizon.states.size()));
  for (const MotionState3D& state : horizon.states) {
    hashFloat(hash, state.x);
    hashFloat(hash, state.y);
    hashFloat(hash, state.z);
    hashFloat(hash, state.vx);
    hashFloat(hash, state.vy);
    hashFloat(hash, state.vz);
    hashFloat(hash, state.yaw);
    hashFloat(hash, state.yaw_rate);
  }
  hashValue(hash, static_cast<std::uint64_t>(horizon.controls.size()));
  for (const MotionControl3D& control : horizon.controls) {
    hashFloat(hash, control.ax);
    hashFloat(hash, control.ay);
    hashFloat(hash, control.az);
    hashFloat(hash, control.yaw_accel);
  }
  hashValue(hash, static_cast<std::uint64_t>(horizon.nominal_prefix_control_count));
  hashValue(hash, static_cast<std::uint64_t>(horizon.arrival_control_count));
}

void hashStopLineage(std::uint64_t& hash, const StopExecution3D& execution) {
  hashValue(hash,
            static_cast<std::uint64_t>(execution.validation_proof.lineage.index()));
  if (const auto* const static_lineage =
          std::get_if<StaticFiniteExecutionValidationLineage3D>(
              &execution.validation_proof.lineage)) {
    const NavigationWorldCertificate3D& world = static_lineage->world_certificate;
    hashValue(hash, world.producer_instance_id);
    hashValue(hash, world.esdf_fingerprint);
    hashValue(hash, world.esdf_source_raw_revision);
    hashValue(hash, world.esdf_source_occupied_fingerprint);
    hashValue(hash, world.raw_validated_through_revision);
    hashValue(hash, world.local_world_generation);
    hashValue(hash, world.topology_revision);
    hashValue(hash, static_lineage->static_occupancy_content_fingerprint);
    hashValue(hash, static_lineage->validation_policy_fingerprint);
    return;
  }
  if (const auto* const raw_lineage =
          std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
              &execution.validation_proof.lineage)) {
    hashValue(hash, raw_lineage->producer_instance_id);
    hashValue(hash, raw_lineage->validated_through_raw_revision);
    hashValue(hash, raw_lineage->validation_policy_fingerprint);
    hashValue(hash, raw_lineage->observed_world_content_fingerprint);
  }
}

[[nodiscard]] std::uint64_t
stopExecutionArtifactFingerprint(const StopExecution3D& execution) noexcept {
  if (execution.horizon == nullptr) {
    return 0U;
  }
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, execution.trajectory_revision);
  hashValue(hash, execution.source_snapshot_version);
  hashValue(hash, execution.source_navigation_revision);
  hashPoint(hash, execution.rest_position);
  hashValue(hash, execution.validation_policy != nullptr
                      ? execution.validation_policy->contentFingerprint()
                      : 0U);
  hashValue(hash, execution.execution_input != nullptr
                      ? execution.execution_input->contentFingerprint()
                      : 0U);
  hashValue(hash, execution.latest_lidar_evidence != nullptr
                      ? execution.latest_lidar_evidence->contentFingerprint()
                      : 0U);
  hashValue(hash, execution.observed_raw_world != nullptr
                      ? execution.observed_raw_world->contentFingerprint()
                      : 0U);
  hashValue(hash, execution.static_world != nullptr
                      ? execution.static_world->contentFingerprint()
                      : 0U);
  hashStopHorizon(hash, *execution.horizon);
  hashValue(hash, static_cast<std::uint64_t>(execution.valid_from_ns));
  hashValue(hash, static_cast<std::uint64_t>(execution.valid_until_ns));
  hashValue(hash, static_cast<std::uint64_t>(execution.control_interval_ns));
  hashValue(hash, execution.validation_proof.validation_contract_fingerprint);
  hashValue(hash, static_cast<std::uint64_t>(
                      std::llround(execution.clearance_reduction * 1.0e6)));
  hashStopLineage(hash, execution);
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] bool
stopWorldOwnerMatchesProof(const StopExecution3D& execution) noexcept {
  const bool raw_mode = execution.observed_raw_world != nullptr;
  const bool static_mode = execution.static_world != nullptr;
  if (raw_mode == static_mode) {
    return false;
  }
  if (raw_mode) {
    const auto* const lineage =
        std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
            &execution.validation_proof.lineage);
    return lineage != nullptr && execution.observed_raw_world->valid() &&
           lineage->producer_instance_id ==
               execution.observed_raw_world->version().producer_instance_id &&
           lineage->validated_through_raw_revision ==
               execution.observed_raw_world->version().revision &&
           lineage->observed_world_content_fingerprint ==
               execution.observed_raw_world->contentFingerprint();
  }
  const auto* const lineage = std::get_if<StaticFiniteExecutionValidationLineage3D>(
      &execution.validation_proof.lineage);
  return lineage != nullptr && execution.static_world->valid() &&
         lineage->static_occupancy_content_fingerprint ==
             execution.static_world->contentFingerprint();
}

// The margin the clearance envelope carries over the hull: the allowance every
// executed trajectory has for its own tracking error.
[[nodiscard]] double envelopeMarginM(const SweptFootprintConfig& footprint) noexcept {
  const SweptFootprintConfig body = physicalBodyFootprint(footprint);
  return std::max(0.0, footprint.radius_m - body.radius_m);
}

// How far the hull at `position` can be inflated before it meets occupied
// evidence, capped at `cap_m`.
template<typename Occupancy>
[[nodiscard]] double
hullInflationMarginM(const Occupancy& occupancy, const Point3& position,
                     const SweptFootprintConfig& footprint, const double cap_m,
                     const LaunchSupportContact3D* launch_support) {
  const SweptFootprintConfig body = physicalBodyFootprint(footprint);
  return rawBodyInflationMargin3D(occupancy, position,
                                  RawClearanceBody3D{
                                      .radius_m = body.radius_m,
                                      .lower_extent_m = body.lower_extent_m,
                                      .upper_extent_m = body.upper_extent_m,
                                  },
                                  cap_m, launch_support);
}

} // namespace

bool StopExecution3D::valid() const noexcept {
  const bool horizon_counts_valid =
      horizon != nullptr &&
      horizon->nominal_prefix_control_count <= horizon->controls.size() &&
      horizon->arrival_control_count <= horizon->controls.size() &&
      horizon->nominal_prefix_control_count + horizon->arrival_control_count ==
          horizon->controls.size();
  const bool duration_fits =
      horizon != nullptr && control_interval_ns > 0 && valid_from_ns > 0 &&
      horizon->controls.size() <=
          static_cast<std::uint64_t>(
              (std::numeric_limits<std::int64_t>::max() - valid_from_ns) /
              control_interval_ns);
  const std::int64_t expected_valid_until_ns =
      duration_fits
          ? valid_from_ns + static_cast<std::int64_t>(horizon->controls.size()) *
                                control_interval_ns
          : 0;
  if (trajectory_revision == 0U || source_snapshot_version == 0U ||
      source_navigation_revision == 0U || horizon == nullptr ||
      horizon->controls.empty() ||
      horizon->states.size() != horizon->controls.size() + 1U ||
      !horizon_counts_valid || !duration_fits ||
      valid_until_ns != expected_valid_until_ns ||
      !finiteMotionHorizonHasTerminalRestState3D(*horizon) ||
      !std::all_of(horizon->states.begin(), horizon->states.end(), finiteState) ||
      !std::all_of(horizon->controls.begin(), horizon->controls.end(), finiteControl)) {
    return false;
  }
  const MotionState3D& terminal_state = horizon->states.back();
  return execution_input != nullptr && execution_input->valid() &&
         execution_input->nominalStateAuthoritative() &&
         finiteStateNearlyEqual(horizon->states.front(), execution_input->state()) &&
         validation_policy != nullptr && validation_policy->valid() &&
         latest_lidar_evidence != nullptr && latest_lidar_evidence->valid() &&
         finitePoint(rest_position) &&
         samePointExact(rest_position,
                        Point3{terminal_state.x, terminal_state.y, terminal_state.z}) &&
         validation_proof.artifact_fingerprint != 0U &&
         validation_proof.validation_contract_fingerprint != 0U &&
         stopWorldOwnerMatchesProof(*this) &&
         validation_proof.artifact_fingerprint ==
             stopExecutionArtifactFingerprint(*this);
}

StopCertificationResult3D
certifyStopExecution3D(const ExecutionPlan3D& current,
                       StopExecutionCertification3D certification) {
  const auto rejected = [](const StopCertificationStatus3D status) {
    StopCertificationResult3D result;
    result.status = status;
    return result;
  };
  const bool raw_mode = certification.observed_raw_world != nullptr;
  const bool static_mode = certification.static_world != nullptr;
  if (!current.valid() || certification.trajectory_revision == 0U ||
      raw_mode == static_mode || certification.validation_policy == nullptr ||
      !certification.validation_policy->valid() ||
      certification.execution_input == nullptr ||
      !certification.execution_input->valid() ||
      !certification.execution_input->nominalStateAuthoritative() ||
      certification.latest_lidar_evidence == nullptr ||
      !certification.latest_lidar_evidence->valid() ||
      certification.valid_from_ns <= 0 ||
      certification.execution_input->effectiveStampNs() !=
          certification.valid_from_ns ||
      !executionInputFreshAt(*certification.execution_input,
                             *certification.validation_policy,
                             certification.valid_from_ns) ||
      !latestLidarEvidenceFreshAt(*certification.latest_lidar_evidence,
                                  *certification.validation_policy,
                                  certification.valid_from_ns) ||
      (raw_mode && !certification.observed_raw_world->valid()) ||
      (static_mode && !certification.static_world->valid())) {
    return rejected(certification.execution_input == nullptr ||
                            certification.latest_lidar_evidence == nullptr ||
                            certification.validation_policy == nullptr
                        ? StopCertificationStatus3D::kInvalidInput
                        : StopCertificationStatus3D::kEvidenceContractRejected);
  }

  const FiniteMotionHorizon3D& horizon = certification.horizon;
  const std::int64_t control_interval_ns = finitePathControlIntervalNanoseconds3D(
      certification.validation_policy->dynamics().dt_s);
  const MotionDynamicsConsistency3D horizon_dynamics =
      finiteMotionHorizonDynamicsConsistency3D(
          horizon, certification.execution_input->previousControl(),
          certification.validation_policy->dynamics());
  if (horizon.controls.empty() ||
      horizon.states.size() != horizon.controls.size() + 1U ||
      control_interval_ns <= 0 || !finiteMotionHorizonHasTerminalRestState3D(horizon) ||
      horizon_dynamics != MotionDynamicsConsistency3D::kConsistent ||
      horizon.controls.size() >
          static_cast<std::uint64_t>(
              (std::numeric_limits<std::int64_t>::max() - certification.valid_from_ns) /
              control_interval_ns)) {
    StopCertificationResult3D rejection =
        rejected(StopCertificationStatus3D::kHorizonContractRejected);
    rejection.dynamics_consistency = horizon_dynamics;
    return rejection;
  }
  if (!finiteStateNearlyEqual(horizon.states.front(),
                              certification.execution_input->state())) {
    return rejected(StopCertificationStatus3D::kInitialStateMismatch);
  }

  const LaunchSupportContact3D* const launch_support_contact =
      raw_mode
          ? optionalAddress(certification.observed_raw_world->launchSupportContact())
          : nullptr;
  // The body the sweep answers to: the clearance envelope with as much of its
  // margin given up as the caller has already seen it need.
  const SweptFootprintConfig validation_footprint =
      clearanceReducedFootprint(certification.validation_policy->sweptFootprint(),
                                certification.clearance_reduction);
  // Contact evidence at the pose the stop starts from. Evidence the body
  // already overlaps cannot forbid the vehicle from braking out of it.
  const std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_seed =
      proprioceptiveContactSeed3D(
          executionInputPosition(*certification.execution_input),
          certification.execution_input->previousControl(), validation_footprint,
          raw_mode ? std::addressof(certification.observed_raw_world->occupancy())
                   : nullptr);
  const IndexedPointCloudView3D latest_lidar_obstacle_points =
      certification.latest_lidar_evidence->indexedHitPoints();
  // No terminal boundary: a stop answers to the flight envelope, the vehicle's
  // dynamics and the swept body against raw occupancy, and to nothing else.
  const FiniteExecutionPathWorld3D validation_world{
      .flight_envelope = &certification.validation_policy->flightEnvelope(),
      .dynamics = &certification.validation_policy->dynamics(),
      .altitude_envelope = &certification.validation_policy->altitudeEnvelope(),
      .footprint = &validation_footprint,
      .static_occupancy =
          static_mode ? &certification.static_world->occupancy() : nullptr,
      .observed_occupancy =
          raw_mode ? &certification.observed_raw_world->occupancy() : nullptr,
      .launch_support_contact = launch_support_contact,
      .proprioceptive_free_space_seed = optionalAddress(proprioceptive_seed),
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = latest_lidar_obstacle_points,
      .terminal_boundary = std::nullopt,
  };
  const std::vector<TimedExecutionPathPoint3D> validation_points =
      timedExecutionPathPoints(horizon,
                               certification.execution_input->previousControl(),
                               control_interval_ns);
  const FiniteExecutionPathValidation3D path_validation =
      validateCompleteFiniteExecutionPath3D(
          validation_points, certification.execution_input->previousControl(),
          validation_world);
  if (!path_validation.accepted()) {
    StopCertificationResult3D rejection =
        rejected(StopCertificationStatus3D::kPathValidationRejected);
    rejection.path_validation_status = path_validation.status;
    rejection.clearance_reduction = certification.clearance_reduction;
    rejection.path_validation_failure_segment_index =
        path_validation.failure_segment_index;
    return rejection;
  }

  // Where the vehicle comes to rest it stays, and a vehicle at rest drifts
  // within the position error its controller holds it to. The rest pose
  // therefore keeps the whole margin the envelope carries over the hull, even
  // when the braking path itself had to give some of it up. A vehicle already
  // inside that band is exempt: it is where it is, and refusing it a stop
  // would leave it on the horizon the evidence has just invalidated. One
  // recorded flight came to rest a tenth of a metre from a wall on a
  // hull-certified stop, drifted a fifth of a metre while holding, and met it.
  const double rest_margin_required_m =
      envelopeMarginM(certification.validation_policy->sweptFootprint());
  if (rest_margin_required_m > 0.0) {
    const Point3 rest_position{horizon.states.back().x, horizon.states.back().y,
                               horizon.states.back().z};
    const Point3 initial_position =
        executionInputPosition(*certification.execution_input);
    const auto margin_m = [&](const Point3& position) {
      return raw_mode
                 ? hullInflationMarginM(certification.observed_raw_world->occupancy(),
                                        position, validation_footprint,
                                        rest_margin_required_m, launch_support_contact)
                 : hullInflationMarginM(certification.static_world->occupancy(),
                                        position, validation_footprint,
                                        rest_margin_required_m, nullptr);
    };
    if (margin_m(rest_position) + kGeometryTolerance < rest_margin_required_m &&
        margin_m(initial_position) + kGeometryTolerance >= rest_margin_required_m) {
      StopCertificationResult3D rejection =
          rejected(StopCertificationStatus3D::kRestClearanceRejected);
      rejection.clearance_reduction = certification.clearance_reduction;
      return rejection;
    }
  }

  const std::uint64_t collision_policy_fingerprint =
      validationPolicyFingerprint(validation_footprint, launch_support_contact);
  const std::uint64_t validation_contract_fingerprint = validationContractFingerprint(
      validation_world, certification.execution_input->previousControl(),
      ValidationContractOwners3D{
          .observed_raw_world = certification.observed_raw_world.get(),
          .static_world = certification.static_world.get(),
          .latest_lidar_evidence = certification.latest_lidar_evidence.get(),
      });
  if (collision_policy_fingerprint == 0U || validation_contract_fingerprint == 0U) {
    return rejected(StopCertificationStatus3D::kValidationContractInvalid);
  }
  FiniteExecutionValidationLineage3D lineage{
      StaticFiniteExecutionValidationLineage3D{}};
  if (raw_mode) {
    lineage = ObservedRawFiniteExecutionValidationLineage3D{
        .producer_instance_id =
            certification.observed_raw_world->version().producer_instance_id,
        .validated_through_raw_revision =
            certification.observed_raw_world->version().revision,
        .validation_policy_fingerprint = collision_policy_fingerprint,
        .observed_world_content_fingerprint =
            certification.observed_raw_world->contentFingerprint(),
        .observed_occupancy_content_fingerprint =
            certification.observed_raw_world->occupiedContentFingerprint(),
    };
  } else {
    lineage = StaticFiniteExecutionValidationLineage3D{
        .world_certificate = certification.static_world->certificate(),
        .static_occupancy_content_fingerprint =
            certification.static_world->contentFingerprint(),
        .validation_policy_fingerprint = collision_policy_fingerprint,
    };
  }

  const MotionState3D& terminal_state = horizon.states.back();
  const std::int64_t valid_until_ns =
      certification.valid_from_ns +
      static_cast<std::int64_t>(horizon.controls.size()) * control_interval_ns;
  StopExecution3D execution{
      .trajectory_revision = certification.trajectory_revision,
      .source_snapshot_version = current.version,
      .source_navigation_revision = certification.execution_input->poseRevision(),
      .rest_position = Point3{terminal_state.x, terminal_state.y, terminal_state.z},
      .horizon = std::make_shared<const FiniteMotionHorizon3D>(
          std::move(certification.horizon)),
      .observed_raw_world = std::move(certification.observed_raw_world),
      .static_world = std::move(certification.static_world),
      .validation_policy = std::move(certification.validation_policy),
      .execution_input = std::move(certification.execution_input),
      .latest_lidar_evidence = std::move(certification.latest_lidar_evidence),
      .valid_from_ns = certification.valid_from_ns,
      .valid_until_ns = valid_until_ns,
      .control_interval_ns = control_interval_ns,
      .validation_proof =
          FiniteExecutionValidationProof3D{
              .validation_contract_fingerprint = validation_contract_fingerprint,
              .lineage = lineage,
          },
  };
  execution.validation_footprint = validation_footprint;
  execution.clearance_reduction = certification.clearance_reduction;
  execution.validation_proof.artifact_fingerprint =
      stopExecutionArtifactFingerprint(execution);
  if (!execution.valid()) {
    return rejected(StopCertificationStatus3D::kInvalidArtifact);
  }
  return StopCertificationResult3D{
      .status = StopCertificationStatus3D::kCertified,
      .path_validation_status = path_validation.status,
      .clearance_reduction = certification.clearance_reduction,
      .execution = std::move(execution),
  };
}

std::string_view
stopCertificationStatus3DName(const StopCertificationStatus3D status) noexcept {
  switch (status) {
    case StopCertificationStatus3D::kCertified:
      return "certified";
    case StopCertificationStatus3D::kInvalidInput:
      return "invalid_input";
    case StopCertificationStatus3D::kEvidenceContractRejected:
      return "evidence_contract_rejected";
    case StopCertificationStatus3D::kHorizonContractRejected:
      return "horizon_contract_rejected";
    case StopCertificationStatus3D::kInitialStateMismatch:
      return "initial_state_mismatch";
    case StopCertificationStatus3D::kPathValidationRejected:
      return "path_validation_rejected";
    case StopCertificationStatus3D::kRestClearanceRejected:
      return "rest_clearance_rejected";
    case StopCertificationStatus3D::kValidationContractInvalid:
      return "validation_contract_invalid";
    case StopCertificationStatus3D::kInvalidArtifact:
      return "invalid_artifact";
  }
  return "unknown";
}

bool execution_route_snapshot_3d_internal::routeExecutionEvidenceNotOlderThanStop(
    const FiniteExecutionState3D& candidate, const StopExecution3D& previous) noexcept {
  // Only evidence order matters. The candidate route is certified from where
  // the vehicle stands now, which is somewhere along the stop, so no position
  // agreement with the stop's own start or rest point can be required.
  return previous.valid() && candidate.validation_policy != nullptr &&
         previous.validation_policy != nullptr &&
         candidate.validation_policy->contentFingerprint() ==
             previous.validation_policy->contentFingerprint() &&
         candidate.trajectory_revision > previous.trajectory_revision &&
         candidate.execution_input != nullptr && previous.execution_input != nullptr &&
         executionInputNotOlder(*candidate.execution_input,
                                *previous.execution_input) &&
         candidate.latest_lidar_evidence != nullptr &&
         previous.latest_lidar_evidence != nullptr &&
         latestLidarEvidenceNotOlder(*candidate.latest_lidar_evidence,
                                     *previous.latest_lidar_evidence);
}

ExecutionRouteTransitionResult3D
execution_route_snapshot_3d_internal::applyEnterStopExecutionCommand3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version,
    StopExecutionCertification3D certification,
    StopCertificationResult3D* const certification_report) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  if (current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max()) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kVersionExhausted);
  }
  // A stop supersedes whatever owned the vehicle, including an older stop. It
  // is refused only when the evidence it was certified from is older than the
  // evidence of the owner it would replace, because that would install a view
  // of the world the plan has already moved past.
  const StopExecution3D* const resident_stop = current.stopExecution();
  if (resident_stop != nullptr &&
      (certification.trajectory_revision <= resident_stop->trajectory_revision ||
       certification.execution_input == nullptr ||
       resident_stop->execution_input == nullptr ||
       !executionInputNotOlder(*certification.execution_input,
                               *resident_stop->execution_input))) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  const FiniteExecutionState3D* const route_execution = current.finiteExecution();
  const DirectTrackingFiniteExecution3D* const direct_execution =
      current.directTrackingExecution();
  if ((route_execution != nullptr &&
       certification.trajectory_revision <= route_execution->trajectory_revision) ||
      (direct_execution != nullptr &&
       certification.trajectory_revision <= direct_execution->trajectory_revision)) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  StopCertificationResult3D certified =
      certifyStopExecution3D(current, std::move(certification));
  if (certification_report != nullptr) {
    certification_report->status = certified.status;
    certification_report->dynamics_consistency = certified.dynamics_consistency;
    certification_report->path_validation_status = certified.path_validation_status;
    certification_report->clearance_reduction = certified.clearance_reduction;
    certification_report->path_validation_failure_segment_index =
        certified.path_validation_failure_segment_index;
  }
  if (!certified.certified() || !certified.execution.has_value()) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate,
                             ExecutionRouteTransitionDetail3D::kNextPlanInvalid);
  }
  ExecutionPlan3D next = current;
  ++next.version;
  next.route_generation_high_water = current.routeGenerationHighWater();
  // The stop is the resident route's end from where the vehicle stands, and
  // the search for its successor is already running against that route: the
  // plan keeps it, suspended, so the replacement is still weighed against
  // what the vehicle was flying. Dropped instead, a replacement costing twice
  // the route it replaced walked in unopposed at every block.
  const CertifiedRouteSuffix3D* const suspended = current.route();
  next.state = StopPlan3D{
      .execution = std::move(*certified.execution),
      .suspended_route = suspended != nullptr ? *suspended : CertifiedRouteSuffix3D{},
  };
  ++next.execution_owner_epoch;
  return finishTransition(current, std::move(next));
}

} // namespace drone_city_nav
