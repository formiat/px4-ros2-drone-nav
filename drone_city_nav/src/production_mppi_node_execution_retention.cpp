#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

#include "production_mppi_node_execution_internal.hpp"

namespace drone_city_nav {

namespace {

constexpr float kArrivalSearchIntervalS{0.5F};

[[nodiscard]] mppi::TimedExecutionPathPoint
executionPathPoint(const msg::MppiHorizonPoint& point) noexcept {
  return mppi::TimedExecutionPathPoint{
      .time_from_start_s =
          static_cast<double>(point.time_from_start_ns) / 1'000'000'000.0,
      .state =
          mppi::State{
              .x = static_cast<float>(point.position.x),
              .y = static_cast<float>(point.position.y),
              .z = static_cast<float>(point.position.z),
              .vx = static_cast<float>(point.velocity.x),
              .vy = static_cast<float>(point.velocity.y),
              .vz = static_cast<float>(point.velocity.z),
              .yaw = point.yaw_rad,
              .yaw_rate = point.yaw_rate_radps,
          },
      .control =
          mppi::Control{
              .ax = static_cast<float>(point.acceleration.x),
              .ay = static_cast<float>(point.acceleration.y),
              .az = static_cast<float>(point.acceleration.z),
              .yaw_accel = point.yaw_acceleration_radps2,
          },
  };
}

[[nodiscard]] std::vector<mppi::TimedExecutionPathPoint>
executionPathPoints(const msg::MppiTrajectoryHorizon& horizon) {
  std::vector<mppi::TimedExecutionPathPoint> points;
  points.reserve(horizon.points.size());
  std::ranges::transform(horizon.points, std::back_inserter(points),
                         executionPathPoint);
  return points;
}

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
validationTerminalBoundary(const FiniteExecutionState3D& execution,
                           const CertifiedRouteSuffix3D& route) {
  if (!execution.terminal_boundary.has_value() || route.geometry == nullptr ||
      route.geometry->mppi_route == nullptr) {
    return std::nullopt;
  }
  const FiniteRouteTerminalBoundary3D& boundary = *execution.terminal_boundary;
  return mppi::FiniteExecutionPathTerminalBoundary{
      .endpoint = boundary.endpoint,
      .forward = boundary.forward,
      .tolerance_m = boundary.tolerance_m,
      .activation_distance_m = boundary.activation_distance_m,
      .maximum_cross_track_m = boundary.maximum_cross_track_m,
      .activation_route = *route.geometry->mppi_route,
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
      !production_mppi_execution_detail::latestLidarEvidenceFreshness(
           latest_lidar_evidence, lidar_validation_now_ns,
           route.validation_policy->latestLidarMaximumAgeMs())
           .fresh) {
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
  const ProprioceptiveFreeSpaceSeed3D* free_space_seed =
      observed_route && observed_world->proprioceptiveFreeSpaceSeed().has_value()
          ? &*observed_world->proprioceptiveFreeSpaceSeed()
          : nullptr;
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
      .require_known_free_space = static_route,
      .proprioceptive_free_space_seed = free_space_seed,
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
      !production_mppi_execution_detail::latestLidarEvidenceFreshness(
           latest_lidar_evidence, lidar_validation_now_ns,
           execution.validation_policy->latestLidarMaximumAgeMs())
           .fresh) {
    return std::nullopt;
  }
  const bool static_world = execution.static_world != nullptr;
  const bool observed_world = execution.observed_raw_world != nullptr;
  if (static_world == observed_world) {
    return std::nullopt;
  }
  const ProprioceptiveFreeSpaceSeed3D* free_space_seed =
      observed_world &&
              execution.observed_raw_world->proprioceptiveFreeSpaceSeed().has_value()
          ? &*execution.observed_raw_world->proprioceptiveFreeSpaceSeed()
          : nullptr;
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
      .require_known_free_space = static_world,
      .proprioceptive_free_space_seed = free_space_seed,
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
  const std::shared_ptr<const ExecutionRouteSnapshot3D> expected =
      execution_route_store_.snapshot();
  if (expected == nullptr || !expected->route.has_value() ||
      !expected->finite_execution.has_value() ||
      expected->finite_execution->horizon == nullptr) {
    return std::nullopt;
  }
  const CertifiedRouteSuffix3D& route = *expected->route;
  const FiniteExecutionState3D& active = *expected->finite_execution;
  const RouteLifecycleEvent3D* const raw_invalidation =
      route_execution.lifecycle_event.has_value() &&
              route_execution.lifecycle_event->kind ==
                  RouteLifecycleEventKind3D::kRawInvalidated
          ? std::addressof(*route_execution.lifecycle_event)
          : nullptr;
  const std::shared_ptr<const VersionedObservedRawWorld3D>&
      invalidating_observed_world = route_execution.lifecycle_observed_raw_world;
  if (raw_invalidation != nullptr) {
    if (route_execution.source_snapshot != expected ||
        route.observed_raw_world == nullptr || invalidating_observed_world == nullptr ||
        !invalidating_observed_world->valid() ||
        invalidating_observed_world->version().producer_instance_id !=
            raw_invalidation->raw_producer_instance_id ||
        invalidating_observed_world->version().revision !=
            raw_invalidation->raw_revision ||
        !route.observed_raw_world->sharesObservationOwner(
            *invalidating_observed_world)) {
      return std::nullopt;
    }
  }
  const std::vector<mppi::TimedExecutionPathPoint> points = executionPathPoints(active);
  if (points.empty()) {
    return std::nullopt;
  }
  const std::optional<mppi::FiniteExecutionPathWorld> continuation_world =
      exactSnapshotValidationWorld(cycle, route,
                                   validationTerminalBoundary(active, route),
                                   invalidating_observed_world);
  if (!continuation_world.has_value()) {
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
  const std::size_t route_arrival_search_step_controls = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(
              kArrivalSearchIntervalS / route.validation_policy->dynamics().dt_s)));
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
          *continuation_world);
  if (!rebuilt.accepted() ||
      active.trajectory_revision == std::numeric_limits<std::uint64_t>::max()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_SNAPSHOT retained=false snapshot_version=%" PRIu64
        " replacement_failure_reason=%s trajectory_validation=%s "
        "actual_state_validation=%s rebuild_validation=%s",
        expected->version,
        productionMppiExecutionReasonName(replacement_failure_reason),
        mppi::finiteExecutionPathStatusName(trajectory_validation.status),
        mppi::finiteExecutionPathStatusName(actual_state_validation.status),
        mppi::finiteExecutionPathStatusName(rebuilt.validation.status));
    return std::nullopt;
  }
  const auto make_finite_certification = [&]() {
    return FiniteExecutionCertification3D{
        .trajectory_revision = active.trajectory_revision + 1U,
        .horizon = *rebuilt.horizon,
        .execution_input = execution_input,
        .latest_lidar_evidence = latest_lidar_evidence,
        .valid_from_ns = now_ns,
        .kind = raw_invalidation != nullptr ? FiniteExecutionKind3D::kEmergencyBrakeTail
                                            : FiniteExecutionKind3D::kRetained,
    };
  };
  const std::optional<FiniteExecutionState3D> recertified =
      raw_invalidation != nullptr
          ? certifyRawInvalidatedFiniteExecution3D(
                *expected,
                RawInvalidatedFiniteExecutionCertification3D{
                    .invalidation = *raw_invalidation,
                    .invalidating_observed_raw_world = invalidating_observed_world,
                    .finite_execution = make_finite_certification(),
                })
          : certifyFiniteExecution3D(*expected, route, make_finite_certification());
  if (!recertified.has_value()) {
    return std::nullopt;
  }
  const ExecutionRouteTransitionGuard3D guard{
      .expected_snapshot_version = expected->version,
      .expected_route_generation = route.identity.generation,
      .expected_geometry_revision = route.geometry->executable_geometry_revision,
  };
  const ExecutionRouteTransitionResult3D transition =
      raw_invalidation != nullptr
          ? retireCertifiedRoute3D(*expected, guard, *raw_invalidation, *recertified)
          : replaceFiniteExecution3D(*expected, guard, *recertified);
  if (!transition.applied() || transition.next == nullptr ||
      !transition.next->finite_execution.has_value() ||
      transition.next->finite_execution->horizon == nullptr ||
      !transition.next->route.has_value() ||
      (raw_invalidation != nullptr &&
       transition.next->phase != ExecutionRoutePhase3D::kBraking)) {
    return std::nullopt;
  }

  const mppi::FiniteHorizon& finite_horizon =
      *transition.next->finite_execution->horizon;
  msg::MppiTrajectoryHorizon horizon = makeExecutionHorizon(
      cycle, transition.next->finite_execution->valid_until_ns,
      ProductionMppiExecutionMode::kPlanned, ProductionMppiExecutionReason::kNone);
  if (!production_mppi_execution_detail::bindHorizonRouteMetadata(
          horizon, *transition.next->route)) {
    return std::nullopt;
  }
  if (transition.next->finite_execution->observed_raw_world != nullptr) {
    horizon.obstacle_revision =
        transition.next->finite_execution->observed_raw_world->version().revision;
  }
  if (!production_mppi_execution_detail::appendFiniteExecutionPoints(
          horizon, finite_horizon.states, finite_horizon.controls,
          exact_previous_control,
          transition.next->finite_execution->control_interval_ns)) {
    return std::nullopt;
  }
  if (!commitExecutionSnapshotHorizon(cycle, expected, transition, horizon, nullptr)) {
    return std::nullopt;
  }
  ProductionMppiExecutionPublication retained;
  retained.horizon = finite_horizon.states;
  retained.mode = ProductionMppiExecutionMode::kPlanned;
  retained.reason = replacement_failure_reason;
  retained.planned_control_count = finite_horizon.controls.size();
  retained.nominal_prefix_control_count = finite_horizon.nominal_prefix_control_count;
  retained.arrival_control_count = finite_horizon.arrival_control_count;
  retained.arrival_shaping_attempts = rebuilt.arrival_shaping_attempts;
  retained.first_control = finite_horizon.controls.front();
  retained.first_control_available = true;
  retained.latest_lidar_obstacle_sequence = latest_lidar_evidence->sequence();
  retained.latest_lidar_obstacle_hit_count = latest_lidar_obstacle_points.size();
  retained.latest_lidar_obstacle_age_ms = latest_lidar_obstacle_age_ms;
  retained.latest_lidar_obstacle_fresh = latest_lidar_obstacle_fresh;
  retained.latest_lidar_obstacle_receive_time_fallback =
      latest_lidar_obstacle_receive_time_fallback;
  retained.retained_previous_finite_path = true;
  retained.terminal_rest_state = true;
  retained.published = true;
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "FINITE_EXECUTION_SNAPSHOT retained=true recertified=true "
      "snapshot_version=%" PRIu64 " trajectory_revision=%" PRIu64
      " raw_invalidation=%s actual_state_validation=%s "
      "trajectory_validation=%s",
      transition.next->version, transition.next->finite_execution->trajectory_revision,
      raw_invalidation != nullptr ? "true" : "false",
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
  const std::shared_ptr<const ExecutionRouteSnapshot3D> expected =
      execution_route_store_.snapshot();
  if (expected == nullptr ||
      expected->phase != ExecutionRoutePhase3D::kDirectTracking ||
      !expected->direct_tracking_execution.has_value() ||
      expected->direct_tracking_execution->horizon == nullptr) {
    return std::nullopt;
  }
  const DirectTrackingFiniteExecution3D& active = *expected->direct_tracking_execution;
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
  const std::size_t direct_arrival_search_step_controls = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(
              kArrivalSearchIntervalS / active.validation_policy->dynamics().dt_s)));
  const mppi::RebuiltFiniteExecutionPathContinuation rebuilt =
      mppi::rebuildFiniteExecutionPathContinuation(
          points, active.valid_from_ns, active.valid_until_ns, now_ns,
          exact_initial_state, exact_previous_control,
          active.horizon->nominal_prefix_control_count,
          // Retention starts from a certified finite sequence. Keep its arrival
          // controls unless current evidence proves that a suffix must change.
          active.horizon->controls.size(), active.validation_policy->dynamics(),
          direct_arrival_search_step_controls, finite_horizon_config_,
          *continuation_world);
  if (!rebuilt.accepted() ||
      active.trajectory_revision == std::numeric_limits<std::uint64_t>::max()) {
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
  const std::optional<DirectTrackingFiniteExecution3D> recertified =
      certifyDirectTrackingExecution3D(
          *expected, DirectTrackingExecutionCertification3D{
                         .identity = active.identity,
                         .trajectory_revision = active.trajectory_revision + 1U,
                         .target = active.target,
                         .horizon = *rebuilt.horizon,
                         .observed_raw_world = active.observed_raw_world,
                         .static_world = active.static_world,
                         .validation_policy = active.validation_policy,
                         .execution_input = execution_input,
                         .latest_lidar_evidence = latest_lidar_evidence,
                         .valid_from_ns = now_ns,
                         .kind = FiniteExecutionKind3D::kRetained,
                     });
  if (!recertified.has_value()) {
    return std::nullopt;
  }
  const ExecutionRouteTransitionResult3D transition =
      replaceDirectTrackingExecution3D(*expected, expected->version, *recertified);
  if (!transition.applied() || transition.next == nullptr ||
      !transition.next->direct_tracking_execution.has_value() ||
      transition.next->direct_tracking_execution->horizon == nullptr) {
    return std::nullopt;
  }

  const DirectTrackingFiniteExecution3D& committed =
      *transition.next->direct_tracking_execution;
  const mppi::FiniteHorizon& finite_horizon = *committed.horizon;
  msg::MppiTrajectoryHorizon horizon = makeExecutionHorizon(
      cycle, committed.valid_until_ns, ProductionMppiExecutionMode::kPlanned,
      ProductionMppiExecutionReason::kNone);
  horizon.route_purpose =
      static_cast<std::uint8_t>(Lattice3DRoutePurpose::kMissionTransit);
  horizon.route_constrained = false;
  horizon.route_target.x = committed.target.x;
  horizon.route_target.y = committed.target.y;
  horizon.route_target.z = committed.target.z;
  if (!production_mppi_execution_detail::appendFiniteExecutionPoints(
          horizon, finite_horizon.states, finite_horizon.controls,
          exact_previous_control, committed.control_interval_ns)) {
    return std::nullopt;
  }
  if (!commitExecutionSnapshotHorizon(cycle, expected, transition, horizon, nullptr)) {
    return std::nullopt;
  }
  ProductionMppiExecutionPublication retained;
  retained.horizon = finite_horizon.states;
  retained.mode = ProductionMppiExecutionMode::kPlanned;
  retained.reason = replacement_failure_reason;
  retained.planned_control_count = finite_horizon.controls.size();
  retained.nominal_prefix_control_count = finite_horizon.nominal_prefix_control_count;
  retained.arrival_control_count = finite_horizon.arrival_control_count;
  retained.arrival_shaping_attempts = rebuilt.arrival_shaping_attempts;
  retained.first_control = finite_horizon.controls.front();
  retained.first_control_available = true;
  retained.latest_lidar_obstacle_sequence = latest_lidar_evidence->sequence();
  retained.latest_lidar_obstacle_hit_count = latest_lidar_obstacle_points.size();
  retained.latest_lidar_obstacle_age_ms = latest_lidar_obstacle_age_ms;
  retained.latest_lidar_obstacle_fresh = latest_lidar_obstacle_fresh;
  retained.latest_lidar_obstacle_receive_time_fallback =
      latest_lidar_obstacle_receive_time_fallback;
  retained.retained_previous_finite_path = true;
  retained.terminal_rest_state = true;
  retained.published = true;
  return retained;
}

std::optional<ProductionMppiExecutionPublication>
ProductionMppiNode::retainActiveFinitePath(
    const ProductionMppiExecutionCycle& cycle,
    const ProductionMppiExecutionReason replacement_failure_reason) {
  const bool snapshot_owner_required = cycle.snapshot_owner_required;
  const bool latest_lidar_obstacle_fresh = cycle.latest_lidar_obstacle_fresh;
  const std::int64_t now_ns = cycle.now_ns;
  const mppi::State& exact_initial_state = cycle.exact_initial_state;
  const mppi::Control& exact_previous_control = cycle.exact_previous_control;
  const auto& execution_path_world = cycle.execution_path_world;
  const mppi::DynamicsConfig* const execution_dynamics = cycle.execution_dynamics;
  const std::size_t arrival_search_step_controls = cycle.arrival_search_step_controls;
  const auto& latest_lidar_evidence = cycle.latest_lidar_evidence;
  const auto latest_lidar_obstacle_points = cycle.latest_lidar_obstacle_points;
  const double latest_lidar_obstacle_age_ms = cycle.latest_lidar_obstacle_age_ms;
  const bool latest_lidar_obstacle_receive_time_fallback =
      cycle.latest_lidar_obstacle_receive_time_fallback;
  const std::int64_t finite_path_control_interval_ns =
      cycle.finite_path_control_interval_ns;
  if (snapshot_owner_required) {
    if (const std::shared_ptr<const ExecutionRouteSnapshot3D> resident =
            execution_route_store_.snapshot();
        resident != nullptr && resident->direct_tracking_execution.has_value()) {
      return retainDirectFinitePath(cycle, replacement_failure_reason);
    }
    return retainSnapshotFinitePath(cycle, replacement_failure_reason);
  }
  if (!latest_lidar_obstacle_fresh) {
    legacy_execution_arbiter_.rejectTrajectory();
    return std::nullopt;
  }
  ProductionMppiActiveFiniteExecutionPath* const active_trajectory =
      legacy_execution_arbiter_.activeTrajectory();
  if (active_trajectory == nullptr) {
    return std::nullopt;
  }
  ProductionMppiActiveFiniteExecutionPath& active = *active_trajectory;
  if (active.message.execution_mode !=
          msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED ||
      active.message.stationary_position_hold) {
    legacy_execution_arbiter_.rejectTrajectory();
    return std::nullopt;
  }
  const std::vector<mppi::TimedExecutionPathPoint> points =
      executionPathPoints(active.message);
  if (points.empty()) {
    legacy_execution_arbiter_.rejectTrajectory();
    return std::nullopt;
  }
  const std::int64_t original_valid_until_ns =
      production_mppi_execution_detail::timeToNanoseconds(active.message.valid_until);
  mppi::FiniteExecutionPathWorld continuation_world = execution_path_world;
  continuation_world.terminal_boundary = active.terminal_boundary;
  const mppi::FiniteExecutionPathValidation actual_state_validation =
      mppi::validateFiniteExecutionPathContinuation(
          points,
          production_mppi_execution_detail::timeToNanoseconds(
              active.message.valid_from),
          original_valid_until_ns, now_ns, exact_initial_state, exact_previous_control,
          continuation_world);
  const mppi::FiniteExecutionPathValidation trajectory_validation =
      mppi::validateFiniteExecutionTrajectoryContinuation(
          points,
          production_mppi_execution_detail::timeToNanoseconds(
              active.message.valid_from),
          original_valid_until_ns, now_ns, exact_initial_state, exact_previous_control,
          continuation_world);

  const mppi::RebuiltFiniteExecutionPathContinuation rebuilt =
      mppi::rebuildFiniteExecutionPathContinuation(
          points,
          production_mppi_execution_detail::timeToNanoseconds(
              active.message.valid_from),
          original_valid_until_ns, now_ns, exact_initial_state, exact_previous_control,
          active.publication.nominal_prefix_control_count,
          // Preserve every still-active certified control before falling back to
          // arrival reshaping under newly observed constraints.
          points.size() - 1U, *execution_dynamics, arrival_search_step_controls,
          finite_horizon_config_, continuation_world);
  const std::size_t expected_index =
      std::min(rebuilt.source_control_index, points.size() - 1U);
  const mppi::State& expected_state = points[expected_index].state;
  const double tracking_error_m = distance3D(
      Point3{exact_initial_state.x, exact_initial_state.y, exact_initial_state.z},
      Point3{expected_state.x, expected_state.y, expected_state.z});
  if (!rebuilt.accepted()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_PATH retained=false replacement_failure_reason=%s "
        "rebuild_validation=%s failure_segment=%zu "
        "failure=(%.3f,%.3f,%.3f) current_z=%.3f expected_z=%.3f "
        "tracking_error_m=%.3f trajectory_validation=%s "
        "trajectory_failure_segment=%zu actual_state_validation=%s "
        "actual_state_failure_segment=%zu arrival_shaping_attempts=%zu",
        productionMppiExecutionReasonName(replacement_failure_reason),
        mppi::finiteExecutionPathStatusName(rebuilt.validation.status),
        rebuilt.validation.failure_segment_index, rebuilt.validation.failure_point.x,
        rebuilt.validation.failure_point.y, rebuilt.validation.failure_point.z,
        exact_initial_state.z, expected_state.z, tracking_error_m,
        mppi::finiteExecutionPathStatusName(trajectory_validation.status),
        trajectory_validation.failure_segment_index,
        mppi::finiteExecutionPathStatusName(actual_state_validation.status),
        actual_state_validation.failure_segment_index,
        rebuilt.arrival_shaping_attempts);
    legacy_execution_arbiter_.rejectTrajectory();
    return std::nullopt;
  }

  const mppi::FiniteHorizon& finite_horizon = *rebuilt.horizon;
  msg::MppiTrajectoryHorizon horizon = makeExecutionHorizon(
      cycle, rebuilt.valid_until_ns, ProductionMppiExecutionMode::kPlanned,
      ProductionMppiExecutionReason::kNone);
  horizon.risk_tier = active.message.risk_tier;
  if (!production_mppi_execution_detail::appendFiniteExecutionPoints(
          horizon, finite_horizon.states, finite_horizon.controls,
          exact_previous_control, finite_path_control_interval_ns)) {
    return std::nullopt;
  }
  if (!publishLegacyExecutionHorizon(cycle, horizon)) {
    return std::nullopt;
  }

  ProductionMppiExecutionPublication retained = active.publication;
  retained.horizon = finite_horizon.states;
  retained.planned_control_count = finite_horizon.controls.size();
  retained.nominal_prefix_control_count = finite_horizon.nominal_prefix_control_count;
  retained.arrival_control_count = finite_horizon.arrival_control_count;
  retained.arrival_shaping_attempts = rebuilt.arrival_shaping_attempts;
  retained.first_control = finite_horizon.controls.front();
  retained.first_control_available = true;
  retained.latest_lidar_obstacle_sequence = latest_lidar_evidence->sequence();
  retained.latest_lidar_obstacle_hit_count = latest_lidar_obstacle_points.size();
  retained.latest_lidar_obstacle_age_ms = latest_lidar_obstacle_age_ms;
  retained.latest_lidar_obstacle_fresh = latest_lidar_obstacle_fresh;
  retained.latest_lidar_obstacle_receive_time_fallback =
      latest_lidar_obstacle_receive_time_fallback;
  retained.finite_path_validation_backoff = rebuilt.path_validation_backoff;
  retained.latest_lidar_path_validation_backoff =
      rebuilt.latest_lidar_path_validation_backoff;
  retained.retained_previous_finite_path = true;
  retained.terminal_rest_state = true;
  retained.published = true;
  active.message = std::move(horizon);
  active.publication = retained;
  legacy_execution_arbiter_.confirmRetainedTrajectory();
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "FINITE_EXECUTION_PATH retained=true replacement_failure_reason=%s "
      "rebased_from_actual=true source_trajectory_validation=%s "
      "source_actual_state_validation=%s source_control=%zu tracking_error_m=%.3f "
      "vertical_error_m=%.3f arrival_shaping_attempts=%zu "
      "original_valid_until_ns=%" PRId64 " rebuilt_valid_until_ns=%" PRId64,
      productionMppiExecutionReasonName(replacement_failure_reason),
      mppi::finiteExecutionPathStatusName(trajectory_validation.status),
      mppi::finiteExecutionPathStatusName(actual_state_validation.status),
      rebuilt.source_control_index, tracking_error_m,
      static_cast<double>(exact_initial_state.z - expected_state.z),
      rebuilt.arrival_shaping_attempts, original_valid_until_ns,
      rebuilt.valid_until_ns);
  return retained;
}

} // namespace drone_city_nav
