#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "production_mppi_node_execution_internal.hpp"

namespace drone_city_nav {

namespace {

[[nodiscard]] std::vector<mppi::TimedExecutionPathPoint>
executionPathPoints(const FiniteExecutionState3D& execution) {
  std::vector<mppi::TimedExecutionPathPoint> points;
  if (execution.horizon == nullptr || execution.horizon->controls.empty() ||
      execution.horizon->states.size() != execution.horizon->controls.size() + 1U ||
      !executionHorizonTerminalOffsetNs(execution.horizon->states.size(),
                                        execution.control_interval_ns) ||
      execution.execution_input == nullptr) {
    return points;
  }
  points.reserve(execution.horizon->states.size());
  for (std::size_t index = 0U; index < execution.horizon->states.size(); ++index) {
    points.push_back(mppi::TimedExecutionPathPoint{
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

[[nodiscard]] std::vector<mppi::TimedExecutionPathPoint>
executionPathPoints(const DirectTrackingFiniteExecution3D& execution) {
  std::vector<mppi::TimedExecutionPathPoint> points;
  if (execution.horizon == nullptr || execution.horizon->controls.empty() ||
      execution.horizon->states.size() != execution.horizon->controls.size() + 1U ||
      !executionHorizonTerminalOffsetNs(execution.horizon->states.size(),
                                        execution.control_interval_ns) ||
      execution.execution_input == nullptr) {
    return points;
  }
  points.reserve(execution.horizon->states.size());
  for (std::size_t index = 0U; index < execution.horizon->states.size(); ++index) {
    points.push_back(mppi::TimedExecutionPathPoint{
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

[[nodiscard]] std::optional<mppi::FiniteExecutionPathTerminalBoundary>
validationTerminalBoundary(
    const FiniteExecutionState3D& execution, const CertifiedRouteSuffix3D& route,
    const std::shared_ptr<const std::vector<mppi::RouteSample3D>>& mppi_reference) {
  if (!execution.terminal_boundary.has_value() || route.geometry == nullptr ||
      mppi_reference == nullptr) {
    return std::nullopt;
  }
  const FiniteRouteTerminalBoundary3D& boundary = *execution.terminal_boundary;
  return mppi::FiniteExecutionPathTerminalBoundary{
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

} // namespace

std::optional<mppi::FiniteExecutionPathWorld>
ProductionMppiNode::exactSnapshotValidationWorld(
    const ProductionMppiExecutionCycle& cycle, const CertifiedRouteSuffix3D& route,
    std::optional<mppi::FiniteExecutionPathTerminalBoundary> terminal_boundary,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& observed_world_override) {
  const auto& latest_lidar_evidence = cycle.latest_lidar_evidence;
  const std::int64_t lidar_validation_now_ns = cycle.lidar_validation_now_ns;
  if (!route.valid() || route.validation_policy == nullptr ||
      !route.validation_policy->valid() || latest_lidar_evidence == nullptr ||
      (route.validation_policy->latestLidarFreshnessRequired() &&
       !production_mppi_execution_detail::latestLidarEvidenceFreshness(
            latest_lidar_evidence, lidar_validation_now_ns,
            route.validation_policy->latestLidarMaximumAgeMs())
            .fresh)) {
    return std::nullopt;
  }
  const bool static_route = route.static_world != nullptr;
  const bool observed_route = route.observed_raw_world != nullptr;
  const VersionedObservedRawWorld3D* const observed_world =
      observed_world_override != nullptr ? observed_world_override.get()
                                         : route.observed_raw_world.get();
  if (static_route == observed_route ||
      (observed_world_override != nullptr && !observed_route) ||
      (observed_route && (observed_world == nullptr || !observed_world->valid()))) {
    return std::nullopt;
  }
  const LaunchSupportContact3D* launch_support_contact =
      observed_route && observed_world->launchSupportContact().has_value()
          ? &*observed_world->launchSupportContact()
          : nullptr;
  return mppi::FiniteExecutionPathWorld{
      .flight_envelope = &route.validation_policy->flightEnvelope(),
      .dynamics = &route.validation_policy->dynamics(),
      .altitude_envelope = &route.validation_policy->altitudeEnvelope(),
      .footprint = &route.validation_policy->sweptFootprint(),
      .static_occupancy = static_route ? &route.static_world->occupancy() : nullptr,
      .observed_occupancy = observed_route ? &observed_world->occupancy() : nullptr,
      .launch_support_contact = launch_support_contact,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points =
          std::span<const Point3>{latest_lidar_evidence->hitPointsMapM()},
      .terminal_boundary = std::move(terminal_boundary),
  };
}

std::optional<mppi::FiniteExecutionPathWorld>
ProductionMppiNode::exactDirectValidationWorld(
    const ProductionMppiExecutionCycle& cycle,
    const DirectTrackingFiniteExecution3D& execution) {
  const auto& latest_lidar_evidence = cycle.latest_lidar_evidence;
  const std::int64_t lidar_validation_now_ns = cycle.lidar_validation_now_ns;
  if (!execution.valid() || execution.validation_policy == nullptr ||
      latest_lidar_evidence == nullptr ||
      (execution.validation_policy->latestLidarFreshnessRequired() &&
       !production_mppi_execution_detail::latestLidarEvidenceFreshness(
            latest_lidar_evidence, lidar_validation_now_ns,
            execution.validation_policy->latestLidarMaximumAgeMs())
            .fresh)) {
    return std::nullopt;
  }
  const bool static_world = execution.static_world != nullptr;
  const bool observed_world = execution.observed_raw_world != nullptr;
  if (static_world == observed_world) {
    return std::nullopt;
  }
  const LaunchSupportContact3D* launch_support_contact =
      observed_world && execution.observed_raw_world->launchSupportContact().has_value()
          ? &*execution.observed_raw_world->launchSupportContact()
          : nullptr;
  return mppi::FiniteExecutionPathWorld{
      .flight_envelope = &execution.validation_policy->flightEnvelope(),
      .dynamics = &execution.validation_policy->dynamics(),
      .altitude_envelope = &execution.validation_policy->altitudeEnvelope(),
      .footprint = &execution.validation_policy->sweptFootprint(),
      .static_occupancy = static_world ? &execution.static_world->occupancy() : nullptr,
      .observed_occupancy =
          observed_world ? &execution.observed_raw_world->occupancy() : nullptr,
      .launch_support_contact = launch_support_contact,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points =
          std::span<const Point3>{latest_lidar_evidence->hitPointsMapM()},
      .terminal_boundary = std::nullopt,
  };
}

std::optional<ProductionMppiExecutionPublication>
ProductionMppiNode::retainSnapshotFinitePath(
    const ProductionMppiExecutionCycle& cycle,
    const ProductionMppiExecutionReason replacement_failure_reason) {
  const auto& route_execution = cycle.route_execution;
  const auto& execution_input = cycle.execution_input;
  const auto& latest_lidar_evidence = cycle.latest_lidar_evidence;
  const std::int64_t now_ns = cycle.now_ns;
  const mppi::State& exact_initial_state = cycle.exact_initial_state;
  const mppi::Control& exact_previous_control = cycle.exact_previous_control;
  const auto latest_lidar_obstacle_points = cycle.latest_lidar_obstacle_points;
  const double latest_lidar_obstacle_age_ms = cycle.latest_lidar_obstacle_age_ms;
  const bool latest_lidar_obstacle_fresh = cycle.latest_lidar_obstacle_fresh;
  const bool latest_lidar_obstacle_receive_time_fallback =
      cycle.latest_lidar_obstacle_receive_time_fallback;
  const std::shared_ptr<const ExecutionPlan3D> expected =
      route_execution_manager_.plan();
  const CertifiedRouteSuffix3D* const expected_route =
      expected != nullptr ? expected->route() : nullptr;
  const FiniteExecutionState3D* const expected_execution =
      expected != nullptr ? expected->finiteExecution() : nullptr;
  if (expected == nullptr || expected_route == nullptr ||
      expected_execution == nullptr || expected_execution->horizon == nullptr) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_SNAPSHOT retained=false stage=missing_resident_owner "
        "snapshot_present=%s route_present=%s execution_present=%s",
        expected != nullptr ? "true" : "false",
        expected_route != nullptr ? "true" : "false",
        expected_execution != nullptr ? "true" : "false");
    return std::nullopt;
  }
  const CertifiedRouteSuffix3D& route = *expected_route;
  const FiniteExecutionState3D& active = *expected_execution;
  const RouteLifecycleEvent3D* const raw_invalidation =
      route_execution.lifecycle_event.has_value() &&
              route_execution.lifecycle_event->kind ==
                  RouteLifecycleEventKind3D::kRawInvalidated
          ? std::addressof(*route_execution.lifecycle_event)
          : nullptr;
  const RouteLifecycleEvent3D* const latest_lidar_invalidation =
      route_execution.lifecycle_event.has_value() &&
              route_execution.lifecycle_event->kind ==
                  RouteLifecycleEventKind3D::kLatestLidarInvalidated
          ? std::addressof(*route_execution.lifecycle_event)
          : nullptr;
  const RouteLifecycleEvent3D* const lifecycle_braking =
      route_execution.lifecycle_event.has_value() &&
              (latest_lidar_invalidation != nullptr ||
               route_execution.lifecycle_event->kind ==
                   RouteLifecycleEventKind3D::kObjectiveSuperseded ||
               route_execution.lifecycle_event->kind ==
                   RouteLifecycleEventKind3D::kCrossTrackExceeded ||
               route_execution.lifecycle_event->kind ==
                   RouteLifecycleEventKind3D::kTrackingTubeExceeded)
          ? std::addressof(*route_execution.lifecycle_event)
          : nullptr;
  const RouteLifecycleEvent3D* const braking_event =
      raw_invalidation != nullptr ? raw_invalidation : lifecycle_braking;
  const std::shared_ptr<const VersionedObservedRawWorld3D>&
      invalidating_observed_world = route_execution.lifecycle_observed_raw_world;
  if (raw_invalidation != nullptr) {
    if (route_execution.source_snapshot != expected ||
        route.observed_raw_world == nullptr || invalidating_observed_world == nullptr ||
        !invalidating_observed_world->valid() ||
        raw_invalidation->generation != route.identity.generation ||
        route.observed_raw_world->version().producer_instance_id !=
            raw_invalidation->raw_producer_instance_id ||
        invalidating_observed_world->version().producer_instance_id !=
            raw_invalidation->raw_producer_instance_id ||
        invalidating_observed_world->version().revision !=
            raw_invalidation->raw_revision) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "FINITE_EXECUTION_SNAPSHOT retained=false "
          "stage=raw_invalidation_owner_mismatch snapshot_version=%" PRIu64,
          expected->version);
      return std::nullopt;
    }
    // A raw invalidation is certified against a newer immutable occupancy
    // snapshot, so it must not be required to alias the route's older buffer.
    // The exact lifecycle version above and the full certification below prove
    // stream lineage and validate the replacement world contents.
  }
  if (lifecycle_braking != nullptr &&
      (route_execution.source_snapshot != expected ||
       lifecycle_braking->generation != route.identity.generation ||
       lifecycle_braking->raw_producer_instance_id != 0U ||
       lifecycle_braking->raw_revision != 0U)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_SNAPSHOT retained=false "
        "stage=lifecycle_braking_owner_mismatch snapshot_version=%" PRIu64,
        expected->version);
    return std::nullopt;
  }
  const std::vector<mppi::TimedExecutionPathPoint> points = executionPathPoints(active);
  if (points.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_SNAPSHOT retained=false stage=invalid_active_path "
        "snapshot_version=%" PRIu64,
        expected->version);
    return std::nullopt;
  }
  const std::shared_ptr<const std::vector<mppi::RouteSample3D>> mppi_reference =
      trajectory_reference_adapter_.adapt(route.geometry);
  const std::optional<mppi::FiniteExecutionPathWorld> continuation_world =
      exactSnapshotValidationWorld(
          cycle, route,
          lifecycle_braking != nullptr
              ? std::nullopt
              : validationTerminalBoundary(active, route, mppi_reference),
          invalidating_observed_world);
  if (!continuation_world.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "FINITE_EXECUTION_SNAPSHOT retained=false "
                         "stage=validation_world_unavailable snapshot_version=%" PRIu64
                         " lidar_present=%s route_valid=%s policy_present=%s",
                         expected->version,
                         latest_lidar_evidence != nullptr ? "true" : "false",
                         route.valid() ? "true" : "false",
                         route.validation_policy != nullptr ? "true" : "false");
    return std::nullopt;
  }
  const mppi::FiniteExecutionPathValidation actual_state_validation =
      mppi::validateFiniteExecutionPathContinuation(
          points, active.valid_from_ns, active.valid_until_ns, now_ns,
          exact_initial_state, exact_previous_control, *continuation_world);

  const mppi::FiniteExecutionPathValidation trajectory_validation =
      mppi::validateFiniteExecutionTrajectoryContinuation(
          points, active.valid_from_ns, active.valid_until_ns, now_ns,
          exact_initial_state, exact_previous_control, *continuation_world);
  const std::size_t route_arrival_search_step_controls =
      mppi::finiteHorizonArrivalSearchStepControls(
          route.validation_policy->dynamics().dt_s);
  if (active.trajectory_revision == std::numeric_limits<std::uint64_t>::max()) {
    RCLCPP_ERROR(get_logger(),
                 "FINITE_EXECUTION_SNAPSHOT retained=false "
                 "stage=trajectory_revision_exhausted snapshot_version=%" PRIu64,
                 expected->version);
    return std::nullopt;
  }
  const std::uint64_t next_trajectory_revision = active.trajectory_revision + 1U;
  std::optional<FiniteExecutionState3D> recertified_braking_tail;
  std::optional<FiniteExecutionPlan3D> recertified_plan;
  FiniteExecutionCertificationResult3D certification_diagnostic;
  const mppi::FiniteExecutionPathCandidateValidator candidate_validator =
      [&](const mppi::FiniteHorizon& candidate) {
        if (candidate.states.empty() || candidate.controls.empty()) {
          return false;
        }
        const std::optional<mppi::FiniteHorizon> braking_tail =
            mppi::buildFiniteBrakingHorizon(
                candidate.states.front(), candidate.controls.size(),
                route.validation_policy->dynamics(), exact_previous_control,
                finite_horizon_config_);
        if (!braking_tail.has_value()) {
          return false;
        }
        FiniteExecutionCertification3D finite_execution{
            .trajectory_revision = next_trajectory_revision,
            .horizon = candidate,
            .execution_input = execution_input,
            .latest_lidar_evidence = latest_lidar_evidence,
            .valid_from_ns = now_ns,
            .kind = FiniteExecutionKind3D::kRetained,
        };
        if (raw_invalidation != nullptr) {
          finite_execution.horizon = *braking_tail;
          finite_execution.kind = FiniteExecutionKind3D::kEmergencyBrakeTail;
          certification_diagnostic = certifyRawInvalidatedFiniteExecution3DDetailed(
              *expected,
              RawInvalidatedFiniteExecutionCertification3D{
                  .invalidation = *raw_invalidation,
                  .invalidating_observed_raw_world = invalidating_observed_world,
                  .finite_execution = finite_execution,
              });
          recertified_braking_tail = certification_diagnostic.execution;
          return certification_diagnostic.certified();
        }
        if (lifecycle_braking != nullptr) {
          finite_execution.horizon = *braking_tail;
          finite_execution.kind = FiniteExecutionKind3D::kEmergencyBrakeTail;
          certification_diagnostic = certifyLifecycleBrakingFiniteExecution3DDetailed(
              *expected, LifecycleBrakingFiniteExecutionCertification3D{
                             .lifecycle_event = *lifecycle_braking,
                             .finite_execution = std::move(finite_execution),
                         });
          recertified_braking_tail = certification_diagnostic.execution;
          return recertified_braking_tail.has_value();
        }
        FiniteExecutionPlanCertificationResult3D certification =
            certifyFiniteExecutionPlan3DDetailed(
                *expected, route,
                FiniteExecutionPlanCertification3D{
                    .command_horizon = std::move(finite_execution),
                    .braking_tail = *braking_tail,
                });
        const bool certified = certification.certified();
        certification_diagnostic = certification.command_horizon;
        recertified_plan = std::move(certification.plan);
        return certified;
      };
  const mppi::RebuiltFiniteExecutionPathContinuation rebuilt =
      mppi::rebuildFiniteExecutionPathContinuation(
          points, active.valid_from_ns, active.valid_until_ns, now_ns,
          exact_initial_state, exact_previous_control,
          active.horizon->nominal_prefix_control_count,
          // The complete remaining sequence was already certified, including
          // its arrival tail. Preserve it first and let current-world validation
          // back off only the suffix that actually requires rebuilding.
          active.horizon->controls.size(), route.validation_policy->dynamics(),
          route_arrival_search_step_controls, finite_horizon_config_,
          *continuation_world, candidate_validator);
  if (!rebuilt.accepted() ||
      (braking_event != nullptr ? !recertified_braking_tail.has_value()
                                : !recertified_plan.has_value())) {
    const std::string_view certification_status =
        finiteExecutionCertificationStatus3DName(certification_diagnostic.status);
    const std::string_view adherence_status = finiteExecutionRouteAdherenceStatus3DName(
        certification_diagnostic.route_adherence_status);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_SNAPSHOT retained=false snapshot_version=%" PRIu64
        " replacement_failure_reason=%s trajectory_validation=%s "
        "actual_state_validation=%s rebuild_validation=%s "
        "certification=%.*s route_adherence=%.*s "
        "route_adherence_failure_distance_m=%.3f",
        expected->version,
        productionMppiExecutionReasonName(replacement_failure_reason),
        mppi::finiteExecutionPathStatusName(trajectory_validation.status),
        mppi::finiteExecutionPathStatusName(actual_state_validation.status),
        mppi::finiteExecutionPathStatusName(rebuilt.validation.status),
        static_cast<int>(certification_status.size()), certification_status.data(),
        static_cast<int>(adherence_status.size()), adherence_status.data(),
        certification_diagnostic.route_adherence_failure_distance_m);
    return std::nullopt;
  }
  const ExecutionRouteTransitionGuard3D guard{
      .expected_snapshot_version = expected->version,
      .expected_route_generation = route.identity.generation,
      .expected_geometry_revision = route.geometry->compiled_trajectory_revision,
  };
  const ExecutionRouteTransitionResult3D transition =
      braking_event != nullptr
          ? retireCertifiedRoute3D(*expected, guard, *braking_event,
                                   *recertified_braking_tail)
          : replaceFiniteExecutionPlan3D(*expected, guard, *recertified_plan);
  const FiniteExecutionState3D* const transitioned_execution =
      transition.next != nullptr ? transition.next->finiteExecution() : nullptr;
  const FiniteExecutionState3D* const transitioned_braking =
      transition.next != nullptr ? transition.next->brakingFallback() : nullptr;
  const CertifiedRouteSuffix3D* const transitioned_route =
      transition.next != nullptr ? transition.next->route() : nullptr;
  if (!transition.applied() || transition.next == nullptr ||
      transitioned_execution == nullptr || transitioned_braking == nullptr ||
      transitioned_execution->horizon == nullptr || transitioned_route == nullptr ||
      (braking_event != nullptr &&
       transition.next->phase() != ExecutionRoutePhase3D::kBraking)) {
    const std::string_view transition_status =
        executionRouteTransitionStatus3DName(transition.status);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_SNAPSHOT retained=false stage=transition_rejected "
        "snapshot_version=%" PRIu64 " transition=%.*s",
        expected->version, static_cast<int>(transition_status.size()),
        transition_status.data());
    return std::nullopt;
  }

  const mppi::FiniteHorizon& finite_horizon = *transitioned_execution->horizon;
  msg::MppiTrajectoryHorizon horizon = makeExecutionHorizon(
      cycle, transitioned_execution->valid_until_ns,
      ProductionMppiExecutionMode::kPlanned, ProductionMppiExecutionReason::kNone);
  if (!production_mppi_execution_detail::bindHorizonRouteMetadata(
          horizon, *transitioned_route)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_SNAPSHOT retained=false stage=route_metadata_rejected "
        "snapshot_version=%" PRIu64,
        transition.next->version);
    return std::nullopt;
  }
  if (transitioned_execution->observed_raw_world != nullptr) {
    horizon.obstacle_revision =
        transitioned_execution->observed_raw_world->version().revision;
  }
  if (!production_mppi_execution_detail::appendFiniteExecutionPoints(
          horizon, finite_horizon.states, finite_horizon.controls,
          exact_previous_control, transitioned_execution->control_interval_ns)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_SNAPSHOT retained=false stage=horizon_encoding_rejected "
        "snapshot_version=%" PRIu64,
        transition.next->version);
    return std::nullopt;
  }
  const ProductionMppiHorizonCommitStatus commit_status =
      commitExecutionSnapshotHorizon(cycle, expected, transition, horizon, nullptr,
                                     expected, nullptr);
  if (commit_status == ProductionMppiHorizonCommitStatus::kRejected) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_SNAPSHOT retained=false stage=publication_commit_rejected "
        "snapshot_version=%" PRIu64,
        expected->version);
    return std::nullopt;
  }
  const bool published = commit_status == ProductionMppiHorizonCommitStatus::kPublished;
  const mppi::FiniteHorizon& reported_horizon =
      published ? finite_horizon : *active.horizon;
  ProductionMppiExecutionPublication retained;
  retained.horizon = reported_horizon.states;
  retained.mode = ProductionMppiExecutionMode::kPlanned;
  retained.reason = replacement_failure_reason;
  retained.planned_control_count = reported_horizon.controls.size();
  retained.nominal_prefix_control_count = reported_horizon.nominal_prefix_control_count;
  retained.arrival_control_count = reported_horizon.arrival_control_count;
  retained.arrival_shaping_attempts = rebuilt.arrival_shaping_attempts;
  retained.first_control = reported_horizon.controls.front();
  retained.first_control_available = true;
  retained.latest_lidar_obstacle_sequence = latest_lidar_evidence->sequence();
  retained.latest_lidar_obstacle_hit_count = latest_lidar_obstacle_points.size();
  retained.latest_lidar_obstacle_age_ms = latest_lidar_obstacle_age_ms;
  retained.latest_lidar_obstacle_fresh = latest_lidar_obstacle_fresh;
  retained.latest_lidar_obstacle_receive_time_fallback =
      latest_lidar_obstacle_receive_time_fallback;
  retained.retained_previous_finite_path = true;
  retained.resident_owner_continues = !published;
  retained.terminal_rest_state = true;
  retained.published = published;
  const std::string_view braking_event_name =
      braking_event != nullptr ? routeLifecycleEventKind3DName(braking_event->kind)
                               : std::string_view{"none"};
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "FINITE_EXECUTION_SNAPSHOT retained=true recertified=true published=%s "
      "snapshot_version=%" PRIu64 " trajectory_revision=%" PRIu64
      " braking_event=%.*s actual_state_validation=%s "
      "trajectory_validation=%s",
      published ? "true" : "false",
      published ? transition.next->version : expected->version,
      published ? next_trajectory_revision : active.trajectory_revision,
      static_cast<int>(braking_event_name.size()), braking_event_name.data(),
      mppi::finiteExecutionPathStatusName(actual_state_validation.status),
      mppi::finiteExecutionPathStatusName(trajectory_validation.status));
  return retained;
}

std::optional<ProductionMppiExecutionPublication>
ProductionMppiNode::retainDirectFinitePath(
    const ProductionMppiExecutionCycle& cycle,
    const ProductionMppiExecutionReason replacement_failure_reason) {
  const auto& execution_input = cycle.execution_input;
  const auto& latest_lidar_evidence = cycle.latest_lidar_evidence;
  const std::int64_t now_ns = cycle.now_ns;
  const mppi::State& exact_initial_state = cycle.exact_initial_state;
  const mppi::Control& exact_previous_control = cycle.exact_previous_control;
  const auto latest_lidar_obstacle_points = cycle.latest_lidar_obstacle_points;
  const double latest_lidar_obstacle_age_ms = cycle.latest_lidar_obstacle_age_ms;
  const bool latest_lidar_obstacle_fresh = cycle.latest_lidar_obstacle_fresh;
  const bool latest_lidar_obstacle_receive_time_fallback =
      cycle.latest_lidar_obstacle_receive_time_fallback;
  const std::shared_ptr<const ExecutionPlan3D> expected =
      route_execution_manager_.plan();
  const DirectTrackingFiniteExecution3D* const expected_direct =
      expected != nullptr ? expected->directTrackingExecution() : nullptr;
  if (expected == nullptr ||
      expected->phase() != ExecutionRoutePhase3D::kDirectTracking ||
      expected_direct == nullptr || expected_direct->horizon == nullptr) {
    return std::nullopt;
  }
  const DirectTrackingFiniteExecution3D& active = *expected_direct;
  const std::vector<mppi::TimedExecutionPathPoint> points = executionPathPoints(active);
  const std::optional<mppi::FiniteExecutionPathWorld> continuation_world =
      exactDirectValidationWorld(cycle, active);
  if (points.empty() || !continuation_world.has_value()) {
    return std::nullopt;
  }
  const mppi::FiniteExecutionPathValidation actual_state_validation =
      mppi::validateFiniteExecutionPathContinuation(
          points, active.valid_from_ns, active.valid_until_ns, now_ns,
          exact_initial_state, exact_previous_control, *continuation_world);
  const mppi::FiniteExecutionPathValidation trajectory_validation =
      mppi::validateFiniteExecutionTrajectoryContinuation(
          points, active.valid_from_ns, active.valid_until_ns, now_ns,
          exact_initial_state, exact_previous_control, *continuation_world);
  const std::size_t direct_arrival_search_step_controls =
      mppi::finiteHorizonArrivalSearchStepControls(
          active.validation_policy->dynamics().dt_s);
  if (active.trajectory_revision == std::numeric_limits<std::uint64_t>::max()) {
    return std::nullopt;
  }
  std::optional<DirectTrackingFiniteExecution3D> recertified;
  const mppi::FiniteExecutionPathCandidateValidator candidate_validator =
      [&](const mppi::FiniteHorizon& candidate) {
        recertified = certifyDirectTrackingExecution3D(
            *expected, DirectTrackingExecutionCertification3D{
                           .identity = active.identity,
                           .trajectory_revision = active.trajectory_revision + 1U,
                           .target = active.target,
                           .horizon = candidate,
                           .observed_raw_world = active.observed_raw_world,
                           .static_world = active.static_world,
                           .validation_policy = active.validation_policy,
                           .execution_input = execution_input,
                           .latest_lidar_evidence = latest_lidar_evidence,
                           .valid_from_ns = now_ns,
                           .kind = FiniteExecutionKind3D::kRetained,
                       });
        return recertified.has_value();
      };
  const mppi::RebuiltFiniteExecutionPathContinuation rebuilt =
      mppi::rebuildFiniteExecutionPathContinuation(
          points, active.valid_from_ns, active.valid_until_ns, now_ns,
          exact_initial_state, exact_previous_control,
          active.horizon->nominal_prefix_control_count,
          // Retention starts from a certified finite sequence. Keep its arrival
          // controls unless current evidence proves that a suffix must change.
          active.horizon->controls.size(), active.validation_policy->dynamics(),
          direct_arrival_search_step_controls, finite_horizon_config_,
          *continuation_world, candidate_validator);
  if (!rebuilt.accepted() || !recertified.has_value()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "DIRECT_EXECUTION_SNAPSHOT retained=false snapshot_version=%" PRIu64
        " replacement_failure_reason=%s trajectory_validation=%s "
        "actual_state_validation=%s rebuild_validation=%s",
        expected->version,
        productionMppiExecutionReasonName(replacement_failure_reason),
        mppi::finiteExecutionPathStatusName(trajectory_validation.status),
        mppi::finiteExecutionPathStatusName(actual_state_validation.status),
        mppi::finiteExecutionPathStatusName(rebuilt.validation.status));
    return std::nullopt;
  }
  const ExecutionRouteTransitionResult3D transition =
      replaceDirectTrackingExecution3D(*expected, expected->version, *recertified);
  const DirectTrackingFiniteExecution3D* const transitioned_direct =
      transition.next != nullptr ? transition.next->directTrackingExecution() : nullptr;
  if (!transition.applied() || transition.next == nullptr ||
      transitioned_direct == nullptr || transitioned_direct->horizon == nullptr) {
    return std::nullopt;
  }

  const DirectTrackingFiniteExecution3D& committed = *transitioned_direct;
  const mppi::FiniteHorizon& finite_horizon = *committed.horizon;
  msg::MppiTrajectoryHorizon horizon = makeExecutionHorizon(
      cycle, committed.valid_until_ns, ProductionMppiExecutionMode::kPlanned,
      ProductionMppiExecutionReason::kNone);
  horizon.route_constrained = false;
  horizon.route_target.x = committed.target.x;
  horizon.route_target.y = committed.target.y;
  horizon.route_target.z = committed.target.z;
  if (!production_mppi_execution_detail::appendFiniteExecutionPoints(
          horizon, finite_horizon.states, finite_horizon.controls,
          exact_previous_control, committed.control_interval_ns)) {
    return std::nullopt;
  }
  const ProductionMppiHorizonCommitStatus commit_status =
      commitExecutionSnapshotHorizon(cycle, expected, transition, horizon, nullptr,
                                     expected, nullptr);
  if (commit_status == ProductionMppiHorizonCommitStatus::kRejected) {
    return std::nullopt;
  }
  const bool published = commit_status == ProductionMppiHorizonCommitStatus::kPublished;
  const mppi::FiniteHorizon& reported_horizon =
      published ? finite_horizon : *active.horizon;
  ProductionMppiExecutionPublication retained;
  retained.horizon = reported_horizon.states;
  retained.mode = ProductionMppiExecutionMode::kPlanned;
  retained.reason = replacement_failure_reason;
  retained.planned_control_count = reported_horizon.controls.size();
  retained.nominal_prefix_control_count = reported_horizon.nominal_prefix_control_count;
  retained.arrival_control_count = reported_horizon.arrival_control_count;
  retained.arrival_shaping_attempts = rebuilt.arrival_shaping_attempts;
  retained.first_control = reported_horizon.controls.front();
  retained.first_control_available = true;
  retained.latest_lidar_obstacle_sequence = latest_lidar_evidence->sequence();
  retained.latest_lidar_obstacle_hit_count = latest_lidar_obstacle_points.size();
  retained.latest_lidar_obstacle_age_ms = latest_lidar_obstacle_age_ms;
  retained.latest_lidar_obstacle_fresh = latest_lidar_obstacle_fresh;
  retained.latest_lidar_obstacle_receive_time_fallback =
      latest_lidar_obstacle_receive_time_fallback;
  retained.retained_previous_finite_path = true;
  retained.resident_owner_continues = !published;
  retained.terminal_rest_state = true;
  retained.published = published;
  return retained;
}

std::optional<ProductionMppiExecutionPublication>
ProductionMppiNode::retainActiveFinitePath(
    const ProductionMppiExecutionCycle& cycle,
    const ProductionMppiExecutionReason replacement_failure_reason) {
  const std::shared_ptr<const ExecutionPlan3D> resident =
      route_execution_manager_.plan();
  if (resident != nullptr && resident->directTrackingExecution() != nullptr) {
    return retainDirectFinitePath(cycle, replacement_failure_reason);
  }
  return retainSnapshotFinitePath(cycle, replacement_failure_reason);
}

} // namespace drone_city_nav
