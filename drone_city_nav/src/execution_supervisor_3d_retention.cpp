#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_supervisor_3d.hpp"
#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"
#include "drone_city_nav/trajectory_control_reference_3d.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav {

namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  return value.has_value() ? std::addressof(*value) : nullptr;
}

template<typename FiniteExecution>
[[nodiscard]] std::vector<TimedExecutionPathPoint3D>
executionPathPoints(const FiniteExecution& execution) {
  std::vector<TimedExecutionPathPoint3D> points;
  if (execution.horizon == nullptr || execution.horizon->controls.empty() ||
      execution.horizon->states.size() != execution.horizon->controls.size() + 1U ||
      !executionHorizonTerminalOffsetNs(execution.horizon->states.size(),
                                        execution.control_interval_ns) ||
      execution.execution_input == nullptr) {
    return points;
  }
  points.reserve(execution.horizon->states.size());
  for (std::size_t index = 0U; index < execution.horizon->states.size(); ++index) {
    points.push_back(TimedExecutionPathPoint3D{
        .time_from_start_s = static_cast<double>(static_cast<std::int64_t>(index) *
                                                 execution.control_interval_ns) /
                             1'000'000'000.0,
        .state = execution.horizon->states[index],
        .control = index == 0U ? execution.execution_input->previousControl()
                               : execution.horizon->controls[index - 1U],
    });
  }
  return points;
}

[[nodiscard]] std::optional<FiniteExecutionPathTerminalBoundary3D>
validationTerminalBoundary(
    const FiniteExecutionState3D& execution, const CertifiedRouteSuffix3D& route,
    const std::shared_ptr<const std::vector<ControlRouteSample3D>>& mppi_reference) {
  if (!execution.terminal_boundary.has_value() || route.geometry == nullptr ||
      mppi_reference == nullptr) {
    return std::nullopt;
  }
  const FiniteRouteTerminalBoundary3D& boundary = *execution.terminal_boundary;
  return FiniteExecutionPathTerminalBoundary3D{
      .endpoint = boundary.endpoint,
      .forward = boundary.forward,
      .tolerance_m = boundary.tolerance_m,
      .activation_distance_m = boundary.activation_distance_m,
      .maximum_cross_track_m = boundary.maximum_cross_track_m,
      .activation_route = *mppi_reference,
      .initial_route_station_m = static_cast<float>(route.progress.station_m),
      .activation_route_station_m = static_cast<float>(
          std::max(route.progress.station_m, boundary.activation_route_station_m)),
  };
}

[[nodiscard]] bool
latestLidarEvidenceFresh(const ExecutionRetentionRequest3D& request,
                         const VersionedExecutionValidationPolicy3D& policy) noexcept {
  return request.latest_lidar_evidence != nullptr &&
         (!policy.latestLidarFreshnessRequired() ||
          assessLatestLidarEvidenceFreshness3D(*request.latest_lidar_evidence,
                                               request.lidar_validation_now_ns,
                                               policy.latestLidarMaximumAgeMs())
              .fresh);
}

// `live_seed` receives the contact evidence the returned world points at, so
// the caller owns storage that outlives the world.
[[nodiscard]] std::optional<FiniteExecutionPathWorld3D> snapshotValidationWorld(
    const ExecutionRetentionRequest3D& request, const CertifiedRouteSuffix3D& route,
    std::optional<FiniteExecutionPathTerminalBoundary3D> terminal_boundary,
    std::optional<ProprioceptiveFreeSpaceSeed3D>& live_seed) {
  if (!route.valid() || route.validation_policy == nullptr ||
      !route.validation_policy->valid() ||
      !latestLidarEvidenceFresh(request, *route.validation_policy)) {
    return std::nullopt;
  }
  const bool static_route = route.static_world != nullptr;
  const bool observed_route = route.observed_raw_world != nullptr;
  const VersionedObservedRawWorld3D* const observed_world =
      request.lifecycle_observed_raw_world != nullptr
          ? request.lifecycle_observed_raw_world.get()
          : route.observed_raw_world.get();
  if (static_route == observed_route ||
      (request.lifecycle_observed_raw_world != nullptr && !observed_route) ||
      (observed_route && (observed_world == nullptr || !observed_world->valid()))) {
    return std::nullopt;
  }
  const std::optional<LaunchSupportContact3D>* const launch_support_owner =
      observed_route ? std::addressof(observed_world->launchSupportContact()) : nullptr;
  const LaunchSupportContact3D* const launch_support_contact =
      launch_support_owner != nullptr && launch_support_owner->has_value()
          ? std::addressof(launch_support_owner->value())
          : nullptr;
  // Live contact evidence: the vehicle's pose now, not the pose the route was
  // certified from. Without it a wall that closed in on a moving vehicle
  // leaves it no validated motion at all, not even a stop.
  live_seed = execution_route_snapshot_3d_internal::seedWithRouteDeparture3D(
      proprioceptiveContactSeed3D(
          Point3{request.exact_initial_state.x, request.exact_initial_state.y,
                 request.exact_initial_state.z},
          request.exact_previous_control, route.validation_policy->sweptFootprint(),
          observed_route ? std::addressof(observed_world->occupancy()) : nullptr),
      &route);
  return FiniteExecutionPathWorld3D{
      .flight_envelope = &route.validation_policy->flightEnvelope(),
      .dynamics = &route.validation_policy->dynamics(),
      .altitude_envelope = &route.validation_policy->altitudeEnvelope(),
      .footprint = &route.validation_policy->sweptFootprint(),
      .static_occupancy = static_route ? &route.static_world->occupancy() : nullptr,
      .observed_occupancy = observed_route ? &observed_world->occupancy() : nullptr,
      .launch_support_contact = launch_support_contact,
      .proprioceptive_free_space_seed = optionalAddress(live_seed),
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = request.latest_lidar_evidence->indexedHitPoints(),
      .terminal_boundary = terminal_boundary,
  };
}

[[nodiscard]] std::optional<FiniteExecutionPathWorld3D>
directValidationWorld(const ExecutionRetentionRequest3D& request,
                      const DirectTrackingFiniteExecution3D& execution,
                      std::optional<ProprioceptiveFreeSpaceSeed3D>& live_seed) {
  if (!execution.valid() || execution.validation_policy == nullptr ||
      !latestLidarEvidenceFresh(request, *execution.validation_policy)) {
    return std::nullopt;
  }
  const bool static_world = execution.static_world != nullptr;
  const bool observed_world = execution.observed_raw_world != nullptr;
  if (static_world == observed_world) {
    return std::nullopt;
  }
  const std::optional<LaunchSupportContact3D>* const launch_support_owner =
      observed_world
          ? std::addressof(execution.observed_raw_world->launchSupportContact())
          : nullptr;
  const LaunchSupportContact3D* const launch_support_contact =
      launch_support_owner != nullptr && launch_support_owner->has_value()
          ? std::addressof(launch_support_owner->value())
          : nullptr;
  live_seed = proprioceptiveContactSeed3D(
      Point3{request.exact_initial_state.x, request.exact_initial_state.y,
             request.exact_initial_state.z},
      request.exact_previous_control, execution.validation_policy->sweptFootprint(),
      observed_world ? std::addressof(execution.observed_raw_world->occupancy())
                     : nullptr);
  return FiniteExecutionPathWorld3D{
      .flight_envelope = &execution.validation_policy->flightEnvelope(),
      .dynamics = &execution.validation_policy->dynamics(),
      .altitude_envelope = &execution.validation_policy->altitudeEnvelope(),
      .footprint = &execution.validation_policy->sweptFootprint(),
      .static_occupancy = static_world ? &execution.static_world->occupancy() : nullptr,
      .observed_occupancy =
          observed_world ? &execution.observed_raw_world->occupancy() : nullptr,
      .launch_support_contact = launch_support_contact,
      .proprioceptive_free_space_seed = optionalAddress(live_seed),
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points =
          std::span<const Point3>{request.latest_lidar_evidence->hitPointsMapM()},
      .terminal_boundary = std::nullopt,
  };
}

// A physical rejection of the resident path from where the vehicle stands is
// evidence the path was not certified on. Keeping the longest prefix that
// still clears it carried the vehicle at four metres a second to the last
// clear metre before a mapped wall, twice, and left nothing certifiable when
// the next scan showed more of it. The premise of the path is broken, and the
// only continuation it still supports is the stop that begins now; the next
// tick's candidate reads the same evidence and plans past it if it can.
[[nodiscard]] bool
physicallyRejected(const ExecutionRetentionResult3D& result) noexcept {
  const auto physical = [](const FiniteExecutionPathStatus3D status) {
    return status == FiniteExecutionPathStatus3D::kRawCollision ||
           status == FiniteExecutionPathStatus3D::kLatestLidarRawCollision;
  };
  return physical(result.actual_state_validation.status) ||
         physical(result.trajectory_validation.status);
}

template<typename Execution>
[[nodiscard]] std::size_t
retainedNominalPrefixControlCount(const ExecutionRetentionResult3D& result,
                                  const Execution& active) noexcept {
  return physicallyRejected(result) ? 0U : active.horizon->nominal_prefix_control_count;
}

template<typename Execution>
[[nodiscard]] std::size_t
retainedPreservedPrefixControlCount(const ExecutionRetentionResult3D& result,
                                    const Execution& active) noexcept {
  return physicallyRejected(result) ? 0U : active.horizon->controls.size();
}

[[nodiscard]] ExecutionRetentionResult3D prepareRouteRetention(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const std::shared_ptr<const ExecutionPlan3D>& expected,
    const ExecutionRetentionRequest3D& request) {
  ExecutionRetentionResult3D result;
  result.kind = ExecutionRetentionKind3D::kRoute;
  result.expected_authority = expected_authority;
  const CertifiedRouteSuffix3D* const route = expected->route();
  const FiniteExecutionState3D* const active = expected->finiteExecution();
  if (route == nullptr || active == nullptr || active->horizon == nullptr) {
    return result;
  }
  result.source_trajectory_revision = active->trajectory_revision;

  const RouteLifecycleEvent3D* const path_claim_ended =
      request.lifecycle_event.has_value() &&
              routeLifecycleEventEndsPathClaim3D(request.lifecycle_event->kind)
          ? std::addressof(*request.lifecycle_event)
          : nullptr;
  if (path_claim_ended != nullptr) {
    result.braking_event = path_claim_ended->kind;
    if (request.lifecycle_source_plan != expected ||
        path_claim_ended->generation != route->identity.generation) {
      result.status = ExecutionRetentionStatus3D::kInvalidLifecycleOwner;
      return result;
    }
    // Retention answers one question: is the resident path still executable
    // from where the vehicle is. This event says the path has lost its claim
    // on the vehicle, so continuing it is not the answer, and neither is a
    // braking tail rebuilt out of it: bringing the vehicle to rest is the
    // stop's job, and the stop validates the braking trajectory itself.
    result.status = ExecutionRetentionStatus3D::kBrakingDelegatedToStop;
    return result;
  }

  const std::vector<TimedExecutionPathPoint3D> points = executionPathPoints(*active);
  if (points.empty()) {
    result.status = ExecutionRetentionStatus3D::kInvalidActivePath;
    return result;
  }
  const std::shared_ptr<const std::vector<ControlRouteSample3D>> mppi_reference =
      route->geometry != nullptr ? adaptTrajectoryControlReference3D(*route->geometry)
                                 : nullptr;
  std::optional<ProprioceptiveFreeSpaceSeed3D> live_seed;
  const std::optional<FiniteExecutionPathWorld3D> continuation_world =
      snapshotValidationWorld(
          request, *route, validationTerminalBoundary(*active, *route, mppi_reference),
          live_seed);
  if (!continuation_world.has_value()) {
    result.status = ExecutionRetentionStatus3D::kValidationWorldUnavailable;
    return result;
  }
  result.actual_state_validation = validateFiniteExecutionPathContinuation3D(
      points, active->valid_from_ns, active->valid_until_ns, request.now_ns,
      request.exact_initial_state, request.exact_previous_control, *continuation_world);
  result.trajectory_validation = validateFiniteExecutionTrajectoryContinuation3D(
      points, active->valid_from_ns, active->valid_until_ns, request.now_ns,
      request.exact_initial_state, request.exact_previous_control, *continuation_world);
  if (active->trajectory_revision == std::numeric_limits<std::uint64_t>::max()) {
    result.status = ExecutionRetentionStatus3D::kTrajectoryRevisionExhausted;
    return result;
  }
  result.prepared_trajectory_revision = active->trajectory_revision + 1U;
  std::optional<FiniteExecutionPlan3D> recertified_plan;
  const std::size_t arrival_search_step_controls =
      finiteHorizonArrivalSearchStepControls3D(
          route->validation_policy->dynamics().dt_s);
  const FiniteExecutionPathCandidateValidator3D candidate_validator =
      [&](const FiniteMotionHorizon3D& candidate) {
        if (candidate.states.empty() || candidate.controls.empty()) {
          return false;
        }
        // The earliest stop along the retained path the world admits.
        const std::vector<FiniteMotionHorizon3D> braking_tails =
            buildFiniteBrakingHorizonsAlong3D(
                candidate, route->validation_policy->dynamics(),
                request.exact_previous_control, arrival_search_step_controls,
                request.finite_horizon_config);
        if (braking_tails.empty()) {
          return false;
        }
        FiniteExecutionCertification3D finite_execution{
            .trajectory_revision = result.prepared_trajectory_revision,
            .horizon = candidate,
            .execution_input = request.execution_input,
            .latest_lidar_evidence = request.latest_lidar_evidence,
            .valid_from_ns = request.now_ns,
            .kind = FiniteExecutionKind3D::kRetained,
        };
        FiniteExecutionPlanCertificationResult3D certification =
            certifyFiniteExecutionPlan3DDetailed(
                *expected, *route, std::move(finite_execution), braking_tails);
        const bool certified = certification.certified();
        result.certification = certification.command_horizon;
        recertified_plan = std::move(certification.plan);
        return certified;
      };
  const RebuiltFiniteExecutionPathContinuation3D rebuilt =
      rebuildFiniteExecutionPathContinuation3D(
          points, active->valid_from_ns, active->valid_until_ns, request.now_ns,
          request.exact_initial_state, request.exact_previous_control,
          retainedNominalPrefixControlCount(result, *active),
          retainedPreservedPrefixControlCount(result, *active),
          route->validation_policy->dynamics(), arrival_search_step_controls,
          request.finite_horizon_config, *continuation_world, candidate_validator);
  result.rebuild_validation = rebuilt.validation;
  result.arrival_shaping_attempts = rebuilt.arrival_shaping_attempts;
  if (!rebuilt.accepted()) {
    result.status = ExecutionRetentionStatus3D::kRebuildRejected;
    return result;
  }
  if (!recertified_plan.has_value()) {
    result.status = ExecutionRetentionStatus3D::kCertificationRejected;
    return result;
  }
  const ExecutionRouteTransitionGuard3D guard{
      .expected_snapshot_version = expected->version,
      .expected_route_generation = route->identity.generation,
      .expected_geometry_revision = route->geometry->compiled_trajectory_revision,
  };
  const ExecutionRouteTransitionResult3D transition = replaceFiniteExecutionPlan3D(
      *expected, guard,
      std::move(recertified_plan)
          .value()); // NOLINT(bugprone-unchecked-optional-access)
  const FiniteExecutionState3D* const transitioned_execution =
      transition.next != nullptr ? transition.next->finiteExecution() : nullptr;
  const FiniteExecutionState3D* const transitioned_braking =
      transition.next != nullptr ? transition.next->brakingFallback() : nullptr;
  if (!transition.applied() || transition.next == nullptr ||
      transitioned_execution == nullptr || transitioned_braking == nullptr ||
      transitioned_execution->horizon == nullptr ||
      transition.next->route() == nullptr) {
    result.status = ExecutionRetentionStatus3D::kTransitionRejected;
    return result;
  }
  result.transition =
      std::make_shared<const ExecutionRouteTransitionResult3D>(transition);
  result.status = ExecutionRetentionStatus3D::kPrepared;
  return result;
}

[[nodiscard]] ExecutionRetentionResult3D prepareDirectRetention(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const std::shared_ptr<const ExecutionPlan3D>& expected,
    const ExecutionRetentionRequest3D& request) {
  ExecutionRetentionResult3D result;
  result.kind = ExecutionRetentionKind3D::kDirectTracking;
  result.expected_authority = expected_authority;
  const DirectTrackingFiniteExecution3D* const active =
      expected->directTrackingExecution();
  if (expected->phase() != ExecutionRoutePhase3D::kDirectTracking ||
      active == nullptr || active->horizon == nullptr) {
    return result;
  }
  result.source_trajectory_revision = active->trajectory_revision;
  const std::vector<TimedExecutionPathPoint3D> points = executionPathPoints(*active);
  std::optional<ProprioceptiveFreeSpaceSeed3D> live_seed;
  const std::optional<FiniteExecutionPathWorld3D> continuation_world =
      directValidationWorld(request, *active, live_seed);
  if (points.empty()) {
    result.status = ExecutionRetentionStatus3D::kInvalidActivePath;
    return result;
  }
  if (!continuation_world.has_value()) {
    result.status = ExecutionRetentionStatus3D::kValidationWorldUnavailable;
    return result;
  }
  result.actual_state_validation = validateFiniteExecutionPathContinuation3D(
      points, active->valid_from_ns, active->valid_until_ns, request.now_ns,
      request.exact_initial_state, request.exact_previous_control, *continuation_world);
  result.trajectory_validation = validateFiniteExecutionTrajectoryContinuation3D(
      points, active->valid_from_ns, active->valid_until_ns, request.now_ns,
      request.exact_initial_state, request.exact_previous_control, *continuation_world);
  if (active->trajectory_revision == std::numeric_limits<std::uint64_t>::max()) {
    result.status = ExecutionRetentionStatus3D::kTrajectoryRevisionExhausted;
    return result;
  }
  result.prepared_trajectory_revision = active->trajectory_revision + 1U;
  std::optional<DirectTrackingFiniteExecution3D> recertified;
  const FiniteExecutionPathCandidateValidator3D candidate_validator =
      [&](const FiniteMotionHorizon3D& candidate) {
        recertified = certifyDirectTrackingExecution3D(
            *expected, DirectTrackingExecutionCertification3D{
                           .identity = active->identity,
                           .trajectory_revision = result.prepared_trajectory_revision,
                           .target = active->target,
                           .horizon = candidate,
                           .observed_raw_world = active->observed_raw_world,
                           .static_world = active->static_world,
                           .validation_policy = active->validation_policy,
                           .execution_input = request.execution_input,
                           .latest_lidar_evidence = request.latest_lidar_evidence,
                           .valid_from_ns = request.now_ns,
                           .kind = FiniteExecutionKind3D::kRetained,
                       });
        return recertified.has_value();
      };
  const RebuiltFiniteExecutionPathContinuation3D rebuilt =
      rebuildFiniteExecutionPathContinuation3D(
          points, active->valid_from_ns, active->valid_until_ns, request.now_ns,
          request.exact_initial_state, request.exact_previous_control,
          retainedNominalPrefixControlCount(result, *active),
          retainedPreservedPrefixControlCount(result, *active),
          active->validation_policy->dynamics(),
          finiteHorizonArrivalSearchStepControls3D(
              active->validation_policy->dynamics().dt_s),
          request.finite_horizon_config, *continuation_world, candidate_validator);
  result.rebuild_validation = rebuilt.validation;
  result.arrival_shaping_attempts = rebuilt.arrival_shaping_attempts;
  if (!rebuilt.accepted()) {
    result.status = ExecutionRetentionStatus3D::kRebuildRejected;
    return result;
  }
  if (!recertified.has_value()) {
    result.status = ExecutionRetentionStatus3D::kCertificationRejected;
    return result;
  }
  const ExecutionRouteTransitionResult3D transition =
      replaceDirectTrackingExecution3D(*expected, expected->version, *recertified);
  const DirectTrackingFiniteExecution3D* const transitioned_direct =
      transition.next != nullptr ? transition.next->directTrackingExecution() : nullptr;
  if (!transition.applied() || transition.next == nullptr ||
      transitioned_direct == nullptr || transitioned_direct->horizon == nullptr) {
    result.status = ExecutionRetentionStatus3D::kTransitionRejected;
    return result;
  }
  result.transition =
      std::make_shared<const ExecutionRouteTransitionResult3D>(transition);
  result.status = ExecutionRetentionStatus3D::kPrepared;
  return result;
}

} // namespace

const char* executionRetentionKind3DName(const ExecutionRetentionKind3D kind) noexcept {
  switch (kind) {
    case ExecutionRetentionKind3D::kNone:
      return "none";
    case ExecutionRetentionKind3D::kRoute:
      return "route";
    case ExecutionRetentionKind3D::kDirectTracking:
      return "direct_tracking";
  }
  return "unknown";
}

const char*
executionRetentionStatus3DName(const ExecutionRetentionStatus3D status) noexcept {
  switch (status) {
    case ExecutionRetentionStatus3D::kPrepared:
      return "prepared";
    case ExecutionRetentionStatus3D::kMissingResidentOwner:
      return "missing_resident_owner";
    case ExecutionRetentionStatus3D::kInvalidLifecycleOwner:
      return "invalid_lifecycle_owner";
    case ExecutionRetentionStatus3D::kInvalidActivePath:
      return "invalid_active_path";
    case ExecutionRetentionStatus3D::kValidationWorldUnavailable:
      return "validation_world_unavailable";
    case ExecutionRetentionStatus3D::kTrajectoryRevisionExhausted:
      return "trajectory_revision_exhausted";
    case ExecutionRetentionStatus3D::kRebuildRejected:
      return "rebuild_rejected";
    case ExecutionRetentionStatus3D::kBrakingDelegatedToStop:
      return "braking_delegated_to_stop";
    case ExecutionRetentionStatus3D::kCertificationRejected:
      return "certification_rejected";
    case ExecutionRetentionStatus3D::kTransitionRejected:
      return "transition_rejected";
  }
  return "unknown";
}

bool ExecutionRetentionResult3D::prepared() const noexcept {
  return status == ExecutionRetentionStatus3D::kPrepared &&
         expected_authority != nullptr && expected_authority->valid() &&
         transition != nullptr && transition->applied() && transition->next != nullptr;
}

std::shared_ptr<const ExecutionPlan3D>
ExecutionRetentionResult3D::expectedPlan() const noexcept {
  return expected_authority != nullptr ? expected_authority->plan() : nullptr;
}

ExecutionRetentionResult3D
ExecutionSupervisor3D::prepareRetention(ExecutionRetentionRequest3D request) const {
  const ExecutionRetentionRequest3D owned_request{std::move(request)};
  const RouteExecutionManagerSnapshot3D resident = manager_.snapshot();
  const std::shared_ptr<const ExecutionPlan3D> expected = resident.plan();
  if (!resident.valid() || expected == nullptr) {
    return {};
  }
  if (expected->directTrackingExecution() != nullptr) {
    return prepareDirectRetention(resident.authority, expected, owned_request);
  }
  return prepareRouteRetention(resident.authority, expected, owned_request);
}

} // namespace drone_city_nav
