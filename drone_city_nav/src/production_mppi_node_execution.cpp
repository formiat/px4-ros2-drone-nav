#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

#include "production_mppi_node_execution_internal.hpp"
#include "production_mppi_route_world.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {

namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  if (!value.has_value()) {
    return nullptr;
  }
  return std::addressof(value.value());
}

[[nodiscard]] std::optional<mppi::FiniteExecutionPathTerminalBoundary>
finiteRouteTerminalBoundary(
    const mppi::MppiTickInput& input,
    const RouteEndpointSemantics3D endpoint_semantics) noexcept {
  if (!routeEndpointUsesLocalBoundary3D(endpoint_semantics) ||
      !input.route.has_value() || !input.route->points ||
      input.route->points->size() < 2U) {
    return std::nullopt;
  }
  const std::vector<mppi::RouteSample3D>& route = *input.route->points;
  const mppi::RouteSample3D& previous = route[route.size() - 2U];
  const mppi::RouteSample3D& endpoint = route.back();
  const Vec3 forward{
      static_cast<double>(endpoint.x_m - previous.x_m),
      static_cast<double>(endpoint.y_m - previous.y_m),
      static_cast<double>(endpoint.z_m - previous.z_m),
  };
  if (std::hypot(std::hypot(forward.x, forward.y), forward.z) <= 1.0e-6) {
    return std::nullopt;
  }
  return mppi::FiniteExecutionPathTerminalBoundary{
      .endpoint = Point3{endpoint.x_m, endpoint.y_m, endpoint.z_m},
      .forward = forward,
      .activation_distance_m = 10.0,
      .maximum_cross_track_m = 2.0,
      .activation_route = route,
      .initial_route_station_m = input.route->initial_station_m,
      .activation_route_station_m = previous.station_m,
  };
}

} // namespace

ProductionMppiExecutionPublication ProductionMppiNode::publishExecutionHorizon(
    const mppi::MppiTickInput& input, const mppi::MppiTickResult& result,
    const WorldSnapshot3D& world,
    const ProductionRouteExecutionSelection3D& route_execution,
    const std::shared_ptr<const ProductionNavigationObjective>& objective,
    const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
    const std::shared_ptr<const VersionedLatestLidarEvidence3D>& latest_lidar_evidence,
    const OffboardSessionAdmissionState& offboard_session,
    const std::int64_t offboard_session_receive_stamp_ns,
    const ProductionMppiPlanningState planning_state, const std::int64_t now_ns) {
  ProductionMppiExecutionPublication publication;
  if (!execution_horizon_pub_) {
    return publication;
  }
  const bool stationary_capture_rearm =
      planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold &&
      execution_input != nullptr &&
      execution_input->stationaryCaptureStateAuthoritative();
  if (objective == nullptr || execution_input == nullptr || !execution_input->valid() ||
      (!execution_input->nominalStateAuthoritative() && !stationary_capture_rearm) ||
      execution_input->effectiveStampNs() != now_ns ||
      execution_input->poseRevision() != input.pose_revision ||
      !production_mppi_execution_detail::sameState(execution_input->state(),
                                                   input.initial_state) ||
      !input.previous_applied_control.has_value() ||
      !production_mppi_execution_detail::sameControl(execution_input->previousControl(),
                                                     *input.previous_applied_control)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "FINITE_EXECUTION_INPUT rejected=true reason=owner_mismatch "
                          "pose_revision=%" PRIu64,
                          input.pose_revision);
    return publication;
  }
  const mppi::State& exact_initial_state = execution_input->state();
  const mppi::Control& exact_previous_control = execution_input->previousControl();
  std::uint64_t target_offboard_instance_id{0U};
  const bool offboard_session_fresh =
      offboard_session.valid() && offboard_session.latest_source_stamp_ns > 0 &&
      offboard_session_receive_stamp_ns > 0 &&
      now_ns >= offboard_session.latest_source_stamp_ns &&
      now_ns >= offboard_session_receive_stamp_ns &&
      static_cast<double>(now_ns - offboard_session.latest_source_stamp_ns) * 1.0e-6 <=
          maximum_control_feedback_age_ms_ &&
      static_cast<double>(now_ns - offboard_session_receive_stamp_ns) * 1.0e-6 <=
          maximum_control_feedback_age_ms_;
  if (offboard_session_fresh) {
    target_offboard_instance_id = offboard_session.current_producer_instance_id;
  }
  if (target_offboard_instance_id == 0U) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_HORIZON published=false reason=offboard_session_unavailable");
    return publication;
  }
  if (route_execution.source_snapshot == nullptr) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_HORIZON published=false reason=missing_snapshot_owner");
    requestExecutionRevocation(ProductionMppiExecutionReason::kNoExecutableHorizon);
    return publication;
  }
  const std::int64_t lidar_validation_now_ns = get_clock()->now().nanoseconds();
  const Point3 mission_goal = objective->goal;
  const std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world_3d =
      world_pipeline_->latestRawWorld();
  const bool direct_tracking_requested =
      route_execution.direct_tracking_identity.has_value();
  const CertifiedRouteSuffix3D* const selected_snapshot_route =
      !direct_tracking_requested ? route_execution.route.get() : nullptr;
  const ProprioceptiveFreeSpaceSeed3D proprioceptive_free_space_seed{
      .position =
          Point3{exact_initial_state.x, exact_initial_state.y, exact_initial_state.z},
      .body_axis = bodyAxisFromWorldAcceleration(Vec3{exact_previous_control.ax,
                                                      exact_previous_control.ay,
                                                      exact_previous_control.az}),
      .footprint = physical_footprint_config_,
  };
  std::shared_ptr<const VersionedObservedRawWorld3D> direct_observed_world;
  std::shared_ptr<const VersionedStaticWorld3D> direct_static_world;
  if (direct_tracking_requested || stationary_capture_rearm) {
    if (use_static_map_ && static_occupancy_3d_ != nullptr) {
      const DirectTrackingFiniteExecution3D* const direct_execution =
          route_execution.source_snapshot->directTrackingExecution();
      const CertifiedRouteSuffix3D* const source_route =
          route_execution.source_snapshot->route();
      if (direct_execution != nullptr && direct_execution->static_world != nullptr) {
        direct_static_world = direct_execution->static_world;
      } else if (source_route != nullptr && source_route->static_world != nullptr) {
        direct_static_world = source_route->static_world;
      } else {
        direct_static_world = VersionedStaticWorld3D::captureOwned(
            navigationWorldCertificate3D(world), world.static_occupancy);
      }
    } else if (latest_raw_world_3d != nullptr &&
               latest_raw_world_3d->execution_owner != nullptr &&
               latest_raw_world_3d->occupancy != nullptr &&
               latest_raw_world_3d->execution_owner->valid() &&
               std::addressof(latest_raw_world_3d->execution_owner->occupancy()) ==
                   latest_raw_world_3d->occupancy.get() &&
               latest_raw_world_3d->execution_owner->version().producer_instance_id ==
                   latest_raw_world_3d->version.producer_instance_id &&
               latest_raw_world_3d->execution_owner->version().base_snapshot_revision ==
                   latest_raw_world_3d->version.base_snapshot_revision &&
               latest_raw_world_3d->execution_owner->version().revision ==
                   latest_raw_world_3d->version.revision) {
      direct_observed_world = latest_raw_world_3d->execution_owner->deriveRouteEvidence(
          proprioceptive_free_space_seed, world.launch_support_contact);
    }
  }
  const VersionedExecutionValidationPolicy3D* selected_policy{nullptr};
  if (selected_snapshot_route != nullptr) {
    selected_policy = selected_snapshot_route->validation_policy.get();
  } else if (direct_tracking_requested || stationary_capture_rearm) {
    selected_policy = execution_validation_policy_.get();
  }
  const double latest_lidar_maximum_age_ms =
      selected_policy != nullptr ? selected_policy->latestLidarMaximumAgeMs()
                                 : latest_lidar_obstacle_maximum_age_ms_;
  const LatestLidarEvidenceFreshness3D latest_lidar_freshness =
      production_mppi_execution_detail::latestLidarEvidenceFreshness(
          latest_lidar_evidence, lidar_validation_now_ns, latest_lidar_maximum_age_ms);
  double latest_lidar_obstacle_age_ms{-1.0};
  bool latest_lidar_obstacle_fresh{false};
  bool latest_lidar_obstacle_receive_time_fallback{false};
  std::span<const Point3> latest_lidar_obstacle_points;
  if (latest_lidar_evidence != nullptr) {
    latest_lidar_obstacle_age_ms = latest_lidar_freshness.age_ms;
    latest_lidar_obstacle_fresh = latest_lidar_freshness.fresh;
    latest_lidar_obstacle_receive_time_fallback =
        latest_lidar_freshness.receive_time_fallback;
    if (latest_lidar_obstacle_fresh ||
        (selected_policy != nullptr &&
         !selected_policy->latestLidarFreshnessRequired())) {
      latest_lidar_obstacle_points =
          std::span<const Point3>{latest_lidar_evidence->hitPointsMapM()};
    }
  }
  const std::uint64_t latest_lidar_obstacle_sequence =
      latest_lidar_evidence != nullptr ? latest_lidar_evidence->sequence() : 0U;
  const bool exact_snapshot_world =
      selected_policy != nullptr && selected_policy->valid() &&
      ((selected_snapshot_route != nullptr &&
        (selected_snapshot_route->static_world != nullptr) !=
            (selected_snapshot_route->observed_raw_world != nullptr)) ||
       ((direct_tracking_requested || stationary_capture_rearm) &&
        (direct_static_world != nullptr) != (direct_observed_world != nullptr)));
  bool publication_route_constrained{false};
  if (!direct_tracking_requested && selected_snapshot_route != nullptr &&
      selected_snapshot_route->geometry != nullptr &&
      selected_snapshot_route->geometry->constrained_spans != nullptr) {
    publication_route_constrained =
        !selected_snapshot_route->geometry->constrained_spans->empty();
  }
  const OccupancyGrid3D* static_occupancy{nullptr};
  const ObservedOccupancyGrid3D* observed_occupancy{nullptr};
  if (exact_snapshot_world) {
    if (direct_static_world != nullptr) {
      static_occupancy = std::addressof(direct_static_world->occupancy());
    } else if (selected_snapshot_route != nullptr &&
               selected_snapshot_route->static_world != nullptr) {
      static_occupancy =
          std::addressof(selected_snapshot_route->static_world->occupancy());
    }
    if (direct_observed_world != nullptr) {
      observed_occupancy = std::addressof(direct_observed_world->occupancy());
    } else if (selected_snapshot_route != nullptr &&
               selected_snapshot_route->observed_raw_world != nullptr) {
      observed_occupancy =
          std::addressof(selected_snapshot_route->observed_raw_world->occupancy());
    }
  }
  RouteEndpointSemantics3D finite_boundary_endpoint_semantics =
      RouteEndpointSemantics3D::kContinuation;
  if (selected_snapshot_route != nullptr) {
    finite_boundary_endpoint_semantics =
        selected_snapshot_route->planned_endpoint_semantics;
  }
  const std::optional<mppi::FiniteExecutionPathTerminalBoundary>
      route_terminal_boundary =
          direct_tracking_requested
              ? std::nullopt
              : finiteRouteTerminalBoundary(input, finite_boundary_endpoint_semantics);
  const LaunchSupportContact3D* launch_support_contact_owner{nullptr};
  if (exact_snapshot_world && direct_observed_world != nullptr) {
    launch_support_contact_owner =
        optionalAddress(direct_observed_world->launchSupportContact());
  } else if (exact_snapshot_world && selected_snapshot_route != nullptr &&
             selected_snapshot_route->observed_raw_world != nullptr) {
    launch_support_contact_owner = optionalAddress(
        selected_snapshot_route->observed_raw_world->launchSupportContact());
  }
  const FlightEnvelopeConfig* const execution_flight_envelope =
      exact_snapshot_world ? &selected_policy->flightEnvelope() : nullptr;
  const mppi::DynamicsConfig* const execution_dynamics =
      exact_snapshot_world ? &selected_policy->dynamics() : nullptr;
  const mppi::AltitudeEnvelopeConfig* const execution_altitude_envelope =
      exact_snapshot_world ? &selected_policy->altitudeEnvelope() : nullptr;
  const SweptFootprintConfig* const execution_footprint =
      exact_snapshot_world ? &selected_policy->sweptFootprint() : nullptr;
  const mppi::FiniteExecutionPathWorld execution_path_world{
      .flight_envelope = execution_flight_envelope,
      .dynamics = execution_dynamics,
      .altitude_envelope = execution_altitude_envelope,
      .footprint = execution_footprint,
      .static_occupancy = static_occupancy,
      .observed_occupancy = observed_occupancy,
      .launch_support_contact = launch_support_contact_owner,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = latest_lidar_obstacle_points,
      .terminal_boundary = route_terminal_boundary,
  };
  const float execution_dt_s = execution_dynamics != nullptr
                                   ? execution_dynamics->dt_s
                                   : mppi_config_.dynamics.dt_s;
  const std::size_t arrival_search_step_controls =
      mppi::finiteHorizonArrivalSearchStepControls(execution_dt_s);
  const std::int64_t finite_path_control_interval_ns =
      mppi::finitePathControlIntervalNanoseconds(execution_dt_s);
  std::uint64_t latest_obstacle_revision = input.obstacle_revision;
  if (exact_snapshot_world && direct_observed_world != nullptr) {
    latest_obstacle_revision = direct_observed_world->version().revision;
  } else if (exact_snapshot_world && selected_snapshot_route != nullptr &&
             selected_snapshot_route->observed_raw_world != nullptr) {
    latest_obstacle_revision =
        selected_snapshot_route->observed_raw_world->version().revision;
  }

  const ProductionMppiExecutionCycle cycle{
      .input = input,
      .result = result,
      .world = world,
      .route_execution = route_execution,
      .objective = objective,
      .execution_input = execution_input,
      .latest_lidar_evidence = latest_lidar_evidence,
      .offboard_session = offboard_session,
      .offboard_session_receive_stamp_ns = offboard_session_receive_stamp_ns,
      .planning_state = planning_state,
      .now_ns = now_ns,
      .publication = publication,
      .exact_initial_state = exact_initial_state,
      .exact_previous_control = exact_previous_control,
      .target_offboard_instance_id = target_offboard_instance_id,
      .lidar_validation_now_ns = lidar_validation_now_ns,
      .mission_goal = mission_goal,
      .latest_raw_world_3d = latest_raw_world_3d,
      .direct_tracking_requested = direct_tracking_requested,
      .selected_snapshot_route = selected_snapshot_route,
      .direct_observed_world = direct_observed_world,
      .direct_static_world = direct_static_world,
      .selected_policy = selected_policy,
      .latest_lidar_obstacle_age_ms = latest_lidar_obstacle_age_ms,
      .latest_lidar_obstacle_fresh = latest_lidar_obstacle_fresh,
      .latest_lidar_obstacle_receive_time_fallback =
          latest_lidar_obstacle_receive_time_fallback,
      .latest_lidar_obstacle_points = latest_lidar_obstacle_points,
      .latest_lidar_obstacle_sequence = latest_lidar_obstacle_sequence,
      .exact_snapshot_world = exact_snapshot_world,
      .publication_route_constrained = publication_route_constrained,
      .route_terminal_boundary = route_terminal_boundary,
      .execution_flight_envelope = execution_flight_envelope,
      .execution_dynamics = execution_dynamics,
      .execution_altitude_envelope = execution_altitude_envelope,
      .execution_footprint = execution_footprint,
      .execution_path_world = execution_path_world,
      .arrival_search_step_controls = arrival_search_step_controls,
      .finite_path_control_interval_ns = finite_path_control_interval_ns,
      .latest_obstacle_revision = latest_obstacle_revision,
  };

  return publishPreparedExecutionCycle(cycle);
}

ProductionMppiExecutionPublication ProductionMppiNode::publishPreparedExecutionCycle(
    const ProductionMppiExecutionCycle& cycle) {
  const mppi::MppiTickInput& input = cycle.input;
  const mppi::MppiTickResult& result = cycle.result;
  const ProductionRouteExecutionSelection3D& route_execution = cycle.route_execution;
  const std::shared_ptr<const VersionedExecutionInput3D>& execution_input =
      cycle.execution_input;
  const std::shared_ptr<const VersionedLatestLidarEvidence3D>& latest_lidar_evidence =
      cycle.latest_lidar_evidence;
  ProductionMppiExecutionPublication& publication = cycle.publication;
  const mppi::Control& exact_previous_control = cycle.exact_previous_control;
  const Point3& mission_goal = cycle.mission_goal;
  const bool direct_tracking_requested = cycle.direct_tracking_requested;
  const CertifiedRouteSuffix3D* const selected_snapshot_route =
      cycle.selected_snapshot_route;
  const std::shared_ptr<const VersionedObservedRawWorld3D>& direct_observed_world =
      cycle.direct_observed_world;
  const std::shared_ptr<const VersionedStaticWorld3D>& direct_static_world =
      cycle.direct_static_world;
  const VersionedExecutionValidationPolicy3D* const selected_policy =
      cycle.selected_policy;
  const double latest_lidar_obstacle_age_ms = cycle.latest_lidar_obstacle_age_ms;
  const bool latest_lidar_obstacle_fresh = cycle.latest_lidar_obstacle_fresh;
  const bool latest_lidar_obstacle_receive_time_fallback =
      cycle.latest_lidar_obstacle_receive_time_fallback;
  const std::span<const Point3> latest_lidar_obstacle_points =
      cycle.latest_lidar_obstacle_points;
  const std::uint64_t latest_lidar_obstacle_sequence =
      cycle.latest_lidar_obstacle_sequence;
  const bool exact_snapshot_world = cycle.exact_snapshot_world;
  const FlightEnvelopeConfig* const execution_flight_envelope =
      cycle.execution_flight_envelope;
  const mppi::DynamicsConfig* const execution_dynamics = cycle.execution_dynamics;
  const mppi::AltitudeEnvelopeConfig* const execution_altitude_envelope =
      cycle.execution_altitude_envelope;
  const SweptFootprintConfig* const execution_footprint = cycle.execution_footprint;
  const mppi::FiniteExecutionPathWorld& execution_path_world =
      cycle.execution_path_world;
  const std::size_t arrival_search_step_controls = cycle.arrival_search_step_controls;
  const std::int64_t finite_path_control_interval_ns =
      cycle.finite_path_control_interval_ns;
  const ProductionMppiPlanningState planning_state = cycle.planning_state;
  const std::int64_t now_ns = cycle.now_ns;

  if (planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold) {
    ProductionMppiExecutionPublication hold = publishExplicitHold(
        cycle, mission_goal, ProductionMppiExecutionReason::kGoalCapture);
    if (hold.published) {
      return hold;
    }
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  if (planning_state == ProductionMppiPlanningState::kMissionCommandPositionHold) {
    ProductionMppiExecutionPublication hold = publishExplicitHold(
        cycle, Point3{input.target.x, input.target.y, input.target.z},
        ProductionMppiExecutionReason::kGoalCapture);
    if (hold.published) {
      return hold;
    }
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  if (planning_state == ProductionMppiPlanningState::kNoExecutableRouteHold) {
    // Route lifecycle invalidation requires the arbiter to revalidate the actual
    // remaining finite trajectory. A physically safe terminal-rest path may still
    // be retained while route replacement runs.
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableRoute);
  }
  if (planning_state == ProductionMppiPlanningState::kCooperativePassageYieldHold) {
    ProductionMppiExecutionPublication hold = publishExplicitHold(
        cycle, Point3{input.target.x, input.target.y, input.target.z},
        ProductionMppiExecutionReason::kCooperativePassageYield);
    if (hold.published) {
      return hold;
    }
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  const bool latest_lidar_evidence_usable =
      latest_lidar_evidence != nullptr &&
      (latest_lidar_obstacle_fresh ||
       (selected_policy != nullptr &&
        !selected_policy->latestLidarFreshnessRequired()));
  if (!latest_lidar_evidence_usable || execution_dynamics == nullptr ||
      execution_flight_envelope == nullptr || execution_altitude_envelope == nullptr ||
      execution_footprint == nullptr) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_HORIZON executable=false reason=missing_exact_owner "
        "lidar_present=%s lidar_fresh=%s lidar_freshness_required=%s "
        "snapshot_world=%s action=hold",
        latest_lidar_evidence != nullptr ? "true" : "false",
        latest_lidar_obstacle_fresh ? "true" : "false",
        selected_policy != nullptr && selected_policy->latestLidarFreshnessRequired()
            ? "true"
            : "false",
        exact_snapshot_world ? "true" : "false");
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }

  const std::span<const mppi::State> states{result.horizon};
  const std::span<const mppi::Control> controls{result.controls};
  if (states.size() < 2U || controls.empty()) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MPPI_EXECUTION_CONTRACT degraded=true classification=%s repair=%s "
        "states=%zu controls=%zu action=hold_no_executable_path",
        mppi::mppiPostUpdateClassificationName(
            result.post_update_classification.classification),
        mppi::mppiPostUpdateRepairName(result.post_update_repair), states.size(),
        controls.size());
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  const auto altitude_violation =
      std::ranges::find_if(states, [this](const mppi::State& state) {
        return !insideFlightEnvelope(state.z, flight_envelope_config_);
      });
  const bool nominal_altitude_violation =
      result.altitude_envelope_violation || altitude_violation != states.end();
  if (nominal_altitude_violation) {
    const auto [minimum_z, maximum_z] = std::ranges::minmax(
        states, {}, [](const mppi::State& state) { return state.z; });
    const std::size_t violation_index = altitude_violation == states.end()
                                            ? states.size()
                                            : static_cast<std::size_t>(std::distance(
                                                  states.begin(), altitude_violation));
    const double violation_z = altitude_violation == states.end()
                                   ? std::numeric_limits<double>::quiet_NaN()
                                   : altitude_violation->z;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_HORIZON nominal_degraded=true "
        "reason=flight_envelope_violation "
        "initial_z=%.3f minimum_z=%.3f maximum_z=%.3f violation_index=%zu "
        "violation_z=%.3f envelope=[%.3f,%.3f) classification=%s repair=%s "
        "action=shape_and_validate_finite_path",
        states.front().z, minimum_z.z, maximum_z.z, violation_index, violation_z,
        flight_envelope_config_.minimum_target_z_m,
        flight_envelope_config_.maximum_target_z_m,
        mppi::mppiPostUpdateClassificationName(
            result.post_update_classification.classification),
        mppi::mppiPostUpdateRepairName(result.post_update_repair));
  }

  const bool nominal_candidate_degraded =
      nominal_altitude_violation || !result.post_update_classification.executable;
  if (nominal_candidate_degraded) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MPPI_EXECUTION_CONTRACT nominal_degraded=true classification=%s repair=%s "
        "altitude_violation=%s action=shape_and_validate_finite_path",
        mppi::mppiPostUpdateClassificationName(
            result.post_update_classification.classification),
        mppi::mppiPostUpdateRepairName(result.post_update_repair),
        nominal_altitude_violation ? "true" : "false");
  }

  const std::shared_ptr<const ExecutionPlan3D> expected_snapshot =
      route_execution.source_snapshot;
  const std::shared_ptr<const ExecutionPlan3D> execution_certification_snapshot =
      route_execution.certification_snapshot != nullptr
          ? route_execution.certification_snapshot
          : expected_snapshot;
  const CertifiedRouteSuffix3D* route_certification_target{nullptr};
  std::uint64_t route_trajectory_revision{0U};
  std::optional<FiniteExecutionPlanCertificationResult3D> route_candidate_certification;
  mppi::FiniteExecutionPathCandidateValidator route_candidate_validator;
  const std::shared_ptr<const ExecutionPlan3D> resident =
      route_execution_manager_.plan();
  if (execution_certification_snapshot == nullptr || resident != expected_snapshot) {
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  if (!direct_tracking_requested) {
    if (route_execution.pending_activation) {
      route_certification_target = route_execution.route.get();
    } else {
      route_certification_target = execution_certification_snapshot->route();
    }
    std::uint64_t previous_trajectory_revision{0U};
    const FiniteExecutionState3D* const finite_execution =
        execution_certification_snapshot->finiteExecution();
    const DirectTrackingFiniteExecution3D* const direct_tracking_execution =
        execution_certification_snapshot->directTrackingExecution();
    if (finite_execution != nullptr) {
      previous_trajectory_revision = finite_execution->trajectory_revision;
    } else if (direct_tracking_execution != nullptr) {
      previous_trajectory_revision = direct_tracking_execution->trajectory_revision;
    }
    if (route_certification_target == nullptr ||
        previous_trajectory_revision == std::numeric_limits<std::uint64_t>::max()) {
      return publishNoExecutablePathHold(
          cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    route_trajectory_revision = previous_trajectory_revision + 1U;
    route_candidate_validator =
        [&, route_trajectory_revision](const mppi::FiniteHorizon& candidate) {
          if (candidate.states.empty() || candidate.controls.empty()) {
            route_candidate_certification.reset();
            return false;
          }
          const std::optional<mppi::FiniteHorizon> braking_tail =
              mppi::buildFiniteBrakingHorizon(
                  candidate.states.front(), candidate.controls.size(),
                  *execution_dynamics, exact_previous_control, finite_horizon_config_);
          if (!braking_tail.has_value()) {
            route_candidate_certification.reset();
            return false;
          }
          route_candidate_certification.emplace(certifyFiniteExecutionPlan3DDetailed(
              *execution_certification_snapshot, *route_certification_target,
              FiniteExecutionPlanCertification3D{
                  .command_horizon =
                      FiniteExecutionCertification3D{
                          .trajectory_revision = route_trajectory_revision,
                          .horizon = candidate,
                          .execution_input = execution_input,
                          .latest_lidar_evidence = latest_lidar_evidence,
                          .valid_from_ns = now_ns,
                          .kind = FiniteExecutionKind3D::kNominal,
                      },
                  .braking_tail = *braking_tail,
              }));
          return route_candidate_certification->certified();
        };
  }

  mppi::ValidatedFiniteExecutionPath validated_path =
      mppi::buildValidatedFiniteExecutionPath(
          states, controls, exact_previous_control, *execution_dynamics,
          arrival_search_step_controls, finite_horizon_config_, execution_path_world,
          std::move(route_candidate_validator));
  if (!direct_tracking_requested && !route_execution.pending_activation &&
      selected_snapshot_route != nullptr &&
      validated_path.physicalObstacleValidationBackoff()) {
    const bool persistent_raw_collision =
        validated_path.persistent_raw_path_validation_backoff;
    handlePhysicalTrajectoryCollision(
        selected_snapshot_route->identity.generation,
        persistent_raw_collision ? selected_snapshot_route->observed_raw_world
                                 : nullptr,
        persistent_raw_collision ? "selected_finite_candidate_persistent_raw"
                                 : "selected_finite_candidate_latest_lidar",
        ProductionMppiPhysicalTrajectoryAuthority::kUnownedCandidate);
  }
  if (!validated_path.accepted()) {
    if (route_candidate_certification.has_value() &&
        !route_candidate_certification->certified() &&
        route_certification_target != nullptr) {
      const bool command_rejected =
          !route_candidate_certification->command_horizon.certified();
      const FiniteExecutionCertificationResult3D& certification =
          command_rejected ? route_candidate_certification->command_horizon
                           : route_candidate_certification->braking_tail;
      const std::string_view status_name =
          finiteExecutionCertificationStatus3DName(certification.status);
      const std::string_view adherence_status_name =
          finiteExecutionRouteAdherenceStatus3DName(
              certification.route_adherence_status);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "FINITE_EXECUTION_CERTIFICATION certified=false artifact=%s status=%.*s "
          "route_adherence_status=%.*s route_adherence_state_index=%zu "
          "route_adherence_failure_distance_m=%.3f "
          "snapshot_version=%" PRIu64 " route_generation=%" PRIu64
          " geometry_revision=%" PRIu64 " trajectory_revision=%" PRIu64,
          command_rejected ? "command" : "braking_tail",
          static_cast<int>(status_name.size()), status_name.data(),
          static_cast<int>(adherence_status_name.size()), adherence_status_name.data(),
          certification.route_adherence_failure_state_index,
          certification.route_adherence_failure_distance_m, expected_snapshot->version,
          route_certification_target->identity.generation,
          route_certification_target->geometry->compiled_trajectory_revision,
          route_trajectory_revision);
    }
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_HORIZON executable=false reason=%s attempts=%zu "
        "planned_controls=%zu failure_segment=%zu failure=(%.3f,%.3f,%.3f) "
        "latest_lidar_sequence=%" PRIu64 " latest_lidar_age_ms=%.1f "
        "latest_lidar_hits=%zu action=hold_no_executable_path",
        mppi::finiteExecutionPathStatusName(validated_path.validation.status),
        validated_path.arrival_shaping_attempts, controls.size(),
        validated_path.validation.failure_segment_index,
        validated_path.validation.failure_point.x,
        validated_path.validation.failure_point.y,
        validated_path.validation.failure_point.z, latest_lidar_obstacle_sequence,
        latest_lidar_obstacle_age_ms, latest_lidar_obstacle_points.size());
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  mppi::FiniteHorizon executable_path =
      std::move(validated_path.horizon).value_or(mppi::FiniteHorizon{});

  std::shared_ptr<const ExecutionPlan3D> committed_snapshot;
  std::optional<ExecutionRouteTransitionResult3D> snapshot_transition;
  const std::shared_ptr<const ExecutionPlan3D>& expected = expected_snapshot;
  if (direct_tracking_requested) {
    if (!route_execution.direct_tracking_identity->valid() ||
        execution_validation_policy_ == nullptr ||
        (direct_observed_world == nullptr) == (direct_static_world == nullptr)) {
      return publishNoExecutablePathHold(
          cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    const DirectTrackingFiniteExecution3D* const expected_direct =
        expected->directTrackingExecution();
    const FiniteExecutionState3D* const expected_finite = expected->finiteExecution();
    std::uint64_t previous_trajectory_revision{0U};
    if (expected_direct != nullptr) {
      previous_trajectory_revision = expected_direct->trajectory_revision;
    } else if (expected_finite != nullptr) {
      previous_trajectory_revision = expected_finite->trajectory_revision;
    }
    if (previous_trajectory_revision == std::numeric_limits<std::uint64_t>::max()) {
      return publishNoExecutablePathHold(
          cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    const std::optional<DirectTrackingFiniteExecution3D> certified_execution =
        certifyDirectTrackingExecution3D(
            *expected,
            DirectTrackingExecutionCertification3D{
                .identity = *route_execution.direct_tracking_identity,
                .trajectory_revision = previous_trajectory_revision + 1U,
                .target = Point3{input.target.x, input.target.y, input.target.z},
                .horizon = std::move(executable_path),
                .observed_raw_world = direct_observed_world,
                .static_world = direct_static_world,
                .validation_policy = execution_validation_policy_,
                .execution_input = execution_input,
                .latest_lidar_evidence = latest_lidar_evidence,
                .valid_from_ns = now_ns,
                .kind = FiniteExecutionKind3D::kNominal,
            });
    if (!certified_execution.has_value()) {
      return publishNoExecutablePathHold(
          cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    const ExecutionRouteTransitionResult3D transition =
        expected->phase() == ExecutionRoutePhase3D::kDirectTracking
            ? replaceDirectTrackingExecution3D(*expected, expected->version,
                                               *certified_execution)
            : transferToDirectTracking3D(*expected, expected->version,
                                         *certified_execution);
    if (!transition.applied() || transition.next == nullptr ||
        transition.next->directTrackingExecution() == nullptr) {
      return publishNoExecutablePathHold(
          cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    committed_snapshot = transition.next;
    snapshot_transition.emplace(transition);
  } else {
    const CertifiedRouteSuffix3D* const target_route = route_certification_target;
    if (target_route == nullptr || !route_candidate_certification.has_value() ||
        !route_candidate_certification->certified()) {
      return publishNoExecutablePathHold(
          cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    const FiniteExecutionPlanCertificationResult3D& certified_execution =
        *route_candidate_certification;
    const FiniteExecutionPlan3D& execution = *certified_execution.plan;
    const std::shared_ptr<const ExecutionPlan3D>& transition_base =
        execution_certification_snapshot;
    if (transition_base == nullptr) {
      return publishNoExecutablePathHold(
          cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    if (route_execution.pending_activation && transition_base->route() != nullptr &&
        (route_execution.pending_route == nullptr ||
         (route_execution.pending_route->base_kind ==
              PendingExecutionBaseKind3D::kRoute &&
          !route_execution.pending_route->route_splice.has_value()))) {
      return publishNoExecutablePathHold(
          cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    const ExecutionRouteTransitionResult3D prepared_transition = [&] {
      if (transition_base->phase() == ExecutionRoutePhase3D::kDirectTracking) {
        return transferDirectTrackingToCertifiedRoute3D(
            *transition_base, transition_base->version, *target_route, execution);
      }
      if (transition_base->route() == nullptr) {
        return activateCertifiedRoute3D(*transition_base, transition_base->version,
                                        *target_route, execution);
      }
      const ExecutionRouteTransitionGuard3D guard{
          .expected_snapshot_version = transition_base->version,
          .expected_route_generation = transition_base->route()->identity.generation,
          .expected_geometry_revision =
              transition_base->route()->geometry->compiled_trajectory_revision,
      };
      if (!route_execution.pending_activation) {
        return replaceFiniteExecutionPlan3D(*transition_base, guard, execution);
      }
      if (route_execution.pending_route->base_kind ==
          PendingExecutionBaseKind3D::kRouteHandoff) {
        return replaceCertifiedRouteAtHandoff3D(*transition_base, guard, *target_route,
                                                execution);
      }
      return replaceCertifiedRoute3D(*transition_base, guard, *target_route, execution,
                                     *route_execution.pending_route->route_splice);
    }();
    const ExecutionRouteTransitionResult3D transition =
        route_execution.progress_preparation != nullptr
            ? composeExecutionPlanTransition3D(
                  *expected, *route_execution.progress_preparation, prepared_transition)
            : prepared_transition;
    if (!transition.applied() || transition.next == nullptr ||
        transition.next->route() == nullptr ||
        transition.next->finiteExecution() == nullptr ||
        transition.next->brakingFallback() == nullptr) {
      const std::string_view status_name =
          executionRouteTransitionStatus3DName(transition.status);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "FINITE_EXECUTION_TRANSITION applied=false status=%.*s "
                           "snapshot_version=%" PRIu64 " route_generation=%" PRIu64,
                           static_cast<int>(status_name.size()), status_name.data(),
                           expected->version, target_route->identity.generation);
      return publishNoExecutablePathHold(
          cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    committed_snapshot = transition.next;
    snapshot_transition.emplace(transition);
  }

  if (committed_snapshot == nullptr) {
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  const FiniteExecutionState3D* const committed_finite =
      committed_snapshot->finiteExecution();
  const DirectTrackingFiniteExecution3D* const committed_direct =
      committed_snapshot->directTrackingExecution();
  const CertifiedRouteSuffix3D* const committed_route = committed_snapshot->route();
  const mppi::FiniteHorizon* committed_path{nullptr};
  std::int64_t committed_valid_until_ns{0};
  if (committed_finite != nullptr) {
    committed_path = committed_finite->horizon.get();
    committed_valid_until_ns = committed_finite->valid_until_ns;
  } else if (committed_direct != nullptr) {
    committed_path = committed_direct->horizon.get();
    committed_valid_until_ns = committed_direct->valid_until_ns;
  }
  if (committed_path == nullptr || committed_valid_until_ns <= 0) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_PUBLICATION published=false stage=missing_committed_path");
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }

  const std::span<const mppi::State> execution_states{committed_path->states};
  const std::span<const mppi::Control> execution_controls{committed_path->controls};
  if (validated_path.path_validation_backoff || nominal_candidate_degraded) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_PATH executable=true terminal_rest_embedded=true "
        "nominal_degraded=%s validation_backoff=%s arrival_shaping_attempts=%zu "
        "nominal_prefix_controls=%zu planned_controls=%zu arrival_controls=%zu "
        "latest_lidar_backoff=%s latest_lidar_sequence=%" PRIu64
        " latest_lidar_age_ms=%.1f latest_lidar_hits=%zu",
        nominal_candidate_degraded ? "true" : "false",
        validated_path.path_validation_backoff ? "true" : "false",
        validated_path.arrival_shaping_attempts,
        committed_path->nominal_prefix_control_count, controls.size(),
        committed_path->arrival_control_count,
        validated_path.latest_lidar_path_validation_backoff ? "true" : "false",
        latest_lidar_obstacle_sequence, latest_lidar_obstacle_age_ms,
        latest_lidar_obstacle_points.size());
  }

  msg::MppiTrajectoryHorizon horizon = makeExecutionHorizon(
      cycle, committed_valid_until_ns, ProductionMppiExecutionMode::kPlanned,
      ProductionMppiExecutionReason::kNone);
  if (committed_direct != nullptr) {
    horizon.route_constrained = false;
    horizon.route_target.x = committed_direct->target.x;
    horizon.route_target.y = committed_direct->target.y;
    horizon.route_target.z = committed_direct->target.z;
    if (committed_direct->observed_raw_world != nullptr) {
      horizon.obstacle_revision =
          committed_direct->observed_raw_world->version().revision;
    }
  } else if (committed_route != nullptr && committed_finite != nullptr) {
    if (!production_mppi_execution_detail::bindHorizonRouteMetadata(horizon,
                                                                    *committed_route)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "FINITE_EXECUTION_PUBLICATION published=false "
                           "stage=invalid_route_metadata");
      return publishNoExecutablePathHold(
          cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    if (committed_finite->observed_raw_world != nullptr) {
      horizon.obstacle_revision =
          committed_finite->observed_raw_world->version().revision;
    }
  }
  if (!production_mppi_execution_detail::appendFiniteExecutionPoints(
          horizon, execution_states, execution_controls, exact_previous_control,
          finite_path_control_interval_ns)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_PUBLICATION published=false "
        "stage=invalid_finite_execution_points states=%zu controls=%zu",
        execution_states.size(), execution_controls.size());
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  if (!snapshot_transition.has_value() ||
      (route_execution.pending_activation &&
       route_execution.pending_route == nullptr) ||
      commitExecutionSnapshotHorizon(
          cycle, route_execution.source_snapshot, *snapshot_transition, horizon,
          route_execution.pending_activation ? route_execution.pending_route : nullptr,
          route_execution.certification_snapshot,
          route_execution.progress_preparation) !=
          ProductionMppiHorizonCommitStatus::kPublished) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "FINITE_EXECUTION_PUBLICATION published=false "
                         "stage=snapshot_horizon_commit pending_activation=%s",
                         route_execution.pending_activation ? "true" : "false");
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  if (route_execution.pending_activation && route_execution.pending_route != nullptr &&
      committed_route != nullptr && committed_route->observed_raw_world != nullptr) {
    const std::uint64_t blocked_raw_revision =
        observed_route_blocked_raw_revision_.load(std::memory_order_acquire);
    if (blocked_raw_revision != 0U &&
        blocked_raw_revision <=
            committed_route->observed_raw_world->version().revision) {
      observed_route_blocked_raw_revision_.store(0U, std::memory_order_release);
    }
  }
  publication.horizon.assign(execution_states.begin(), execution_states.end());
  publication.mode = ProductionMppiExecutionMode::kPlanned;
  publication.reason = ProductionMppiExecutionReason::kNone;
  publication.planned_control_count = execution_controls.size();
  publication.nominal_prefix_control_count =
      committed_path->nominal_prefix_control_count;
  publication.arrival_control_count = committed_path->arrival_control_count;
  publication.arrival_shaping_attempts = validated_path.arrival_shaping_attempts;
  publication.first_control = execution_controls.front();
  publication.first_control_available = true;
  publication.latest_lidar_obstacle_sequence = latest_lidar_obstacle_sequence;
  publication.latest_lidar_obstacle_hit_count = latest_lidar_obstacle_points.size();
  publication.latest_lidar_obstacle_age_ms = latest_lidar_obstacle_age_ms;
  publication.finite_path_validation_backoff = validated_path.path_validation_backoff;
  publication.finite_path_validation_status = validated_path.validation.status;
  publication.finite_path_first_failed_validation_status =
      validated_path.first_failed_validation_status;
  publication.latest_lidar_obstacle_fresh = latest_lidar_obstacle_fresh;
  publication.latest_lidar_obstacle_receive_time_fallback =
      latest_lidar_obstacle_receive_time_fallback;
  publication.latest_lidar_path_validation_backoff =
      validated_path.latest_lidar_path_validation_backoff;
  publication.terminal_rest_state = true;
  publication.published = true;
  return publication;
}

} // namespace drone_city_nav
