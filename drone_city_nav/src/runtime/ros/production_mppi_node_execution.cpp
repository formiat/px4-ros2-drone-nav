#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"
#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <span>
#include <string>
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
          config_.execution.maximum_control_feedback_age_ms &&
      static_cast<double>(now_ns - offboard_session_receive_stamp_ns) * 1.0e-6 <=
          config_.execution.maximum_control_feedback_age_ms;
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
  const ProprioceptiveFreeSpaceSeed3D proprioceptive_free_space_seed =
      proprioceptiveContactSeed3D(
          Point3{exact_initial_state.x, exact_initial_state.y, exact_initial_state.z},
          exact_previous_control, config_.world.physical_footprint,
          latest_raw_world_3d != nullptr
              ? std::addressof(latest_raw_world_3d->occupancy())
              : nullptr)
          .value_or(proprioceptiveContactSeed3D(
              Point3{exact_initial_state.x, exact_initial_state.y,
                     exact_initial_state.z},
              exact_previous_control, config_.world.physical_footprint, 0.0));
  std::shared_ptr<const VersionedObservedRawWorld3D> direct_observed_world;
  std::shared_ptr<const VersionedStaticWorld3D> direct_static_world;
  if (direct_tracking_requested || stationary_capture_rearm) {
    if (config_.world.use_static_map && world.static_occupancy != nullptr) {
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
    } else if (latest_raw_world_3d != nullptr) {
      direct_observed_world = latest_raw_world_3d->deriveRouteEvidence(
          proprioceptive_free_space_seed, world.launch_support_contact);
    }
  }
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> selected_policy;
  if (selected_snapshot_route != nullptr) {
    selected_policy = selected_snapshot_route->validation_policy;
  } else if (direct_tracking_requested || stationary_capture_rearm) {
    selected_policy = config_.execution.validation_policy;
  }
  const double latest_lidar_maximum_age_ms =
      selected_policy != nullptr
          ? selected_policy->latestLidarMaximumAgeMs()
          : config_.execution.latest_lidar_obstacle_maximum_age_ms;
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
  // Contact evidence for this cycle's validations is the vehicle's pose now,
  // the same seed the observed world was derived with.
  const ProprioceptiveFreeSpaceSeed3D* const proprioceptive_seed_owner =
      observed_occupancy != nullptr ? std::addressof(proprioceptive_free_space_seed)
                                    : nullptr;
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
      .proprioceptive_free_space_seed = proprioceptive_seed_owner,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = latest_lidar_obstacle_points,
      .terminal_boundary = route_terminal_boundary,
  };
  const float execution_dt_s = execution_dynamics != nullptr
                                   ? execution_dynamics->dt_s
                                   : config_.control.mppi.dynamics.dt_s;
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
      .evidence =
          EvidenceSnapshot3D{
              .objective = objective,
              .execution_input = execution_input,
              .latest_lidar_evidence = latest_lidar_evidence,
              .offboard_session = offboard_session,
              .offboard_session_receive_stamp_ns = offboard_session_receive_stamp_ns,
              .exact_initial_state = exact_initial_state,
              .exact_previous_control = exact_previous_control,
              .target_offboard_instance_id = target_offboard_instance_id,
              .lidar_validation_now_ns = lidar_validation_now_ns,
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
              .execution_flight_envelope = execution_flight_envelope,
              .execution_dynamics = execution_dynamics,
              .execution_altitude_envelope = execution_altitude_envelope,
              .execution_footprint = execution_footprint,
              .execution_path_world = execution_path_world,
          },
      .route =
          RouteDecision3D{
              .execution = route_execution,
              .mission_goal = mission_goal,
              .selected_snapshot_route = selected_snapshot_route,
              .planning_state = planning_state,
              .direct_tracking_requested = direct_tracking_requested,
              .publication_route_constrained = publication_route_constrained,
          },
      .controller =
          ControllerCycle3D{
              .input = std::addressof(input),
              .result = std::addressof(result),
              .world = std::addressof(world),
              .now_ns = now_ns,
              .arrival_search_step_controls = arrival_search_step_controls,
              .finite_path_control_interval_ns = finite_path_control_interval_ns,
              .latest_obstacle_revision = latest_obstacle_revision,
          },
      .publication = std::addressof(publication),
  };

  return publishPreparedExecutionCycle(cycle);
}

void ProductionMppiNode::logPhysicalRejectionCells(
    const ProductionMppiExecutionCycle& cycle, const HorizonCandidate3D& candidate) {
  // Names the occupied raw voxels around the point where the longest
  // candidate failed, so a rejection can be traced to the evidence behind it.
  const ObservedOccupancyGrid3D* const occupancy =
      cycle.evidence.execution_path_world.observed_occupancy;
  if (occupancy == nullptr || candidate.first_failed_validation_status !=
                                  mppi::FiniteExecutionPathStatus::kRawCollision) {
    return;
  }
  constexpr double kHorizontalRadiusM{1.2};
  constexpr double kVerticalHalfExtentM{0.8};
  constexpr std::size_t kMaximumCells{6U};
  const Point3& failure = candidate.first_failed_validation_point;
  const double resolution = occupancy->bounds().resolution_m;
  std::string cells;
  std::size_t cell_count{0U};
  for (double z = failure.z - kVerticalHalfExtentM;
       z <= failure.z + kVerticalHalfExtentM; z += resolution) {
    for (double y = failure.y - kHorizontalRadiusM; y <= failure.y + kHorizontalRadiusM;
         y += resolution) {
      for (double x = failure.x - kHorizontalRadiusM;
           x <= failure.x + kHorizontalRadiusM; x += resolution) {
        const std::optional<GridIndex3D> index =
            occupancy->worldToCell(Point3{x, y, z});
        if (!index.has_value() || !occupancy->isOccupied(*index)) {
          continue;
        }
        ++cell_count;
        if (cell_count <= kMaximumCells) {
          const Point3 center = occupancy->cellCenter(*index);
          char buffer[64];
          std::snprintf(buffer, sizeof(buffer), "(%.2f,%.2f,%.2f)", center.x, center.y,
                        center.z);
          cells += buffer;
        }
      }
    }
  }
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                       "TRAJECTORY_COLLISION_CELLS failure=(%.2f,%.2f,%.2f) "
                       "occupied_nearby=%zu cells=%s",
                       failure.x, failure.y, failure.z, cell_count, cells.c_str());
}

void ProductionMppiNode::releaseRouteRejectedByCertificationWhileStationary(
    const ProductionMppiExecutionCycle& cycle, const HorizonCandidate3D& candidate) {
  // Every candidate the controller offers is certified against the resident
  // route: its adherence, its tracking tube, its handoff. While the vehicle
  // still moves, a rejection only keeps the resident owner in force and the
  // next tick offers another candidate. Once the vehicle stands still and the
  // candidates are still rejected by the route contract rather than by
  // physical evidence, the resident route cannot be re-entered from where the
  // vehicle is: it has diverged from it, and waiting for a certifiable
  // candidate would be a blocking latch. The route is released as diverged so
  // the search starts again from the vehicle.
  // Route-contract rejections: the certification of the candidate, the
  // certification target itself, or the plan transition the certified
  // candidate would make. Transient transition outcomes (a moved snapshot, an
  // exhausted version, a conflicting finite execution) are retried, not
  // released.
  const bool certification_rejection =
      candidate.status == HorizonCandidateStatus3D::kFinitePathRejected &&
      candidate.validation_status ==
          mppi::FiniteExecutionPathStatus::kCandidateRejected;
  const bool transition_rejection =
      candidate.status == HorizonCandidateStatus3D::kTransitionRejected &&
      (candidate.transition_status ==
           ExecutionRouteTransitionStatus3D::kCertificateRegression ||
       candidate.transition_status ==
           ExecutionRouteTransitionStatus3D::kNonMonotonicProgress ||
       candidate.transition_status ==
           ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected ||
       candidate.transition_status ==
           ExecutionRouteTransitionStatus3D::kRouteGenerationMismatch ||
       candidate.transition_status ==
           ExecutionRouteTransitionStatus3D::kGeometryRevisionMismatch);
  if ((!certification_rejection && !transition_rejection &&
       candidate.status != HorizonCandidateStatus3D::kCertificationRejected) ||
      candidate.physical_rejection.has_value()) {
    return;
  }
  const std::shared_ptr<const ExecutionPlan3D> resident = execution_supervisor_.plan();
  const std::uint64_t route_generation =
      resident != nullptr ? resident->routeGenerationHighWater() : 0U;
  if (route_generation == 0U ||
      certification_release_route_generation_ >= route_generation) {
    return;
  }
  const mppi::State& vehicle_state = cycle.controller.inputRef().initial_state;
  const double vehicle_speed_mps = std::hypot(static_cast<double>(vehicle_state.vx),
                                              static_cast<double>(vehicle_state.vy),
                                              static_cast<double>(vehicle_state.vz));
  if (vehicle_speed_mps > kStationaryExecutionHoldSpeedToleranceMps) {
    return;
  }
  certification_release_route_generation_ = route_generation;
  const std::string_view status_name = horizonCandidateStatus3DName(candidate.status);
  const std::string_view certification_name =
      finiteExecutionCertificationStatus3DName(candidate.certification_status);
  const std::string_view transition_name =
      executionRouteTransitionStatus3DName(candidate.transition_status);
  const std::string_view detail_name =
      executionRouteTransitionDetail3DName(candidate.transition_detail);
  RCLCPP_WARN(get_logger(),
              "ROUTE_CERTIFICATION_RELEASE route_generation=%" PRIu64
              " status=%.*s certification=%.*s transition=%.*s detail=%.*s "
              "adherence_failure_m=%.2f speed_mps=%.2f action=release_diverged_route",
              route_generation, static_cast<int>(status_name.size()),
              status_name.data(), static_cast<int>(certification_name.size()),
              certification_name.data(), static_cast<int>(transition_name.size()),
              transition_name.data(), static_cast<int>(detail_name.size()),
              detail_name.data(), candidate.route_adherence_failure_distance_m,
              vehicle_speed_mps);
  requestRouteRelease(RouteReleaseReason3D::kDiverged, route_generation);
}

bool ProductionMppiNode::retirePendingRouteRejectedByTransition(
    const ProductionMppiExecutionCycle& cycle, const HorizonCandidate3D& candidate) {
  // A pending successor is an optimistic proposal that every tick tries to
  // activate over the resident plan. When the plan reducer rejects that
  // activation on the candidate's own contract, the same proposal can never
  // activate later, yet while it stays pending the lifecycle measures every
  // newer successor against it and the selection keeps offering it instead
  // of the resident route's own candidates. Acknowledging it hands the slot
  // back to the lifecycle so a fresh successor can be planned from the vehicle.
  const ProductionRouteExecutionSelection3D& route_execution = cycle.route.execution;
  if (!route_execution.pending_activation || route_execution.pending_route == nullptr ||
      candidate.status != HorizonCandidateStatus3D::kTransitionRejected ||
      !pendingRouteActivationStructurallyRejected3D(candidate.transition_status,
                                                    candidate.transition_detail)) {
    return false;
  }
  const std::shared_ptr<const PendingCertifiedRoute3D> pending =
      route_execution.pending_route;
  const bool acknowledged = execution_supervisor_.acknowledgePendingIfSame(pending);
  const std::string_view transition_name =
      executionRouteTransitionStatus3DName(candidate.transition_status);
  const std::string_view detail_name =
      executionRouteTransitionDetail3DName(candidate.transition_detail);
  RCLCPP_WARN(get_logger(),
              "PENDING_ROUTE_RETIRED route_generation=%" PRIu64
              " base_route_generation=%" PRIu64 " transition=%.*s detail=%.*s "
              "acknowledged=%s action=%s",
              pending->route.identity.generation, pending->base_route_generation,
              static_cast<int>(transition_name.size()), transition_name.data(),
              static_cast<int>(detail_name.size()), detail_name.data(),
              acknowledged ? "true" : "false",
              acknowledged ? "release_pending_slot" : "pending_already_replaced");
  return acknowledged;
}

ProductionMppiExecutionPublication ProductionMppiNode::publishPreparedExecutionCycle(
    const ProductionMppiExecutionCycle& cycle) {
  if (cycle.publication == nullptr) {
    return {};
  }
  if (!cycle.valid() || execution_horizon_assembler_ == nullptr) {
    return cycle.publicationRef();
  }
  ProductionMppiExecutionPublication& publication = cycle.publicationRef();
  const ProductionRouteExecutionSelection3D& route_execution = cycle.route.execution;
  HorizonCandidate3D candidate =
      execution_horizon_assembler_->assemble(cycle, execution_supervisor_.plan());

  if (candidate.physical_rejection.has_value()) {
    const HorizonCandidatePhysicalRejection3D& rejection =
        candidate.physical_rejection.value();
    logPhysicalRejectionCells(cycle, candidate);
    // A rejected candidate never owned vehicle motion, so while a resident
    // finite execution is still moving the vehicle it only yields to that
    // owner and the next tick tries again. Once the vehicle no longer moves,
    // whether under a hold or under a candidate shortened until it makes no
    // progress, it can leave that state only along a route-directed
    // candidate: a physically rejected one means the resident route is
    // blocked from where the vehicle actually stands, whatever its suffix
    // validation says, and waiting for the same candidate to pass would be a
    // blocking latch. The route is released for a fresh search anchored at
    // the vehicle instead.
    const std::shared_ptr<const ExecutionPlan3D> resident =
        execution_supervisor_.plan();
    const mppi::State& vehicle_state = cycle.controller.inputRef().initial_state;
    const double vehicle_speed_mps = std::hypot(static_cast<double>(vehicle_state.vx),
                                                static_cast<double>(vehicle_state.vy),
                                                static_cast<double>(vehicle_state.vz));
    const bool resident_without_motion_owner =
        resident != nullptr && resident->finiteExecution() == nullptr &&
        resident->brakingFallback() == nullptr &&
        resident->directTrackingExecution() == nullptr;
    const bool resident_stationary =
        resident != nullptr &&
        resident->routeGenerationHighWater() == rejection.route_generation &&
        (resident_without_motion_owner ||
         vehicle_speed_mps <= kStationaryExecutionHoldSpeedToleranceMps);
    handlePhysicalTrajectoryCollision(
        rejection.route_generation, rejection.observed_raw_world,
        rejection.source == HorizonCandidateObstacleSource3D::kPersistentRaw
            ? "selected_finite_candidate_persistent_raw"
            : "selected_finite_candidate_latest_lidar",
        resident_stationary
            ? ProductionMppiPhysicalTrajectoryAuthority::kResidentOwner
            : ProductionMppiPhysicalTrajectoryAuthority::kUnownedCandidate);
  }

  if (candidate.status == HorizonCandidateStatus3D::kExplicitHold) {
    ProductionMppiExecutionPublication hold = publishExplicitHold(
        cycle, candidate.explicit_hold_position, candidate.explicit_hold_reason);
    if (hold.published) {
      return hold;
    }
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  if (!candidate.planned()) {
    const std::string_view status_name = horizonCandidateStatus3DName(candidate.status);
    const std::string_view certification_name =
        finiteExecutionCertificationStatus3DName(candidate.certification_status);
    const std::string_view braking_certification_name =
        finiteExecutionCertificationStatus3DName(
            candidate.braking_tail_certification_status);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_HORIZON_ASSEMBLY planned=false status=%.*s "
        "validation=%s precondition=%s attempts=%zu "
        "failure_segment=%zu first_remaining=%zu "
        "first_failure=%s@%zu(%.2f,%.2f,%.2f) "
        "certification=%.*s braking_tail=%.*s adherence_failure_m=%.2f "
        "transition=%.*s detail=%.*s action=hold_no_executable_path",
        static_cast<int>(status_name.size()), status_name.data(),
        mppi::finiteExecutionPathStatusName(candidate.validation_status),
        candidate.finite_path_rejected_precondition, candidate.arrival_shaping_attempts,
        candidate.validation_failure_segment,
        candidate.validation_first_remaining_point,
        mppi::finiteExecutionPathStatusName(candidate.first_failed_validation_status),
        candidate.first_failed_validation_segment,
        candidate.first_failed_validation_point.x,
        candidate.first_failed_validation_point.y,
        candidate.first_failed_validation_point.z,
        static_cast<int>(certification_name.size()), certification_name.data(),
        static_cast<int>(braking_certification_name.size()),
        braking_certification_name.data(), candidate.route_adherence_failure_distance_m,
        static_cast<int>(
            executionRouteTransitionStatus3DName(candidate.transition_status).size()),
        executionRouteTransitionStatus3DName(candidate.transition_status).data(),
        static_cast<int>(
            executionRouteTransitionDetail3DName(candidate.transition_detail).size()),
        executionRouteTransitionDetail3DName(candidate.transition_detail).data());
    if (!retirePendingRouteRejectedByTransition(cycle, candidate)) {
      releaseRouteRejectedByCertificationWhileStationary(cycle, candidate);
    }
    const auto physical_status = [](const mppi::FiniteExecutionPathStatus status) {
      return status == mppi::FiniteExecutionPathStatus::kRawCollision ||
             status == mppi::FiniteExecutionPathStatus::kLatestLidarRawCollision;
    };
    // Physical means collision evidence against the path. A validation
    // backoff on its own is not: a candidate rejected at its route endpoint
    // or for adherence backs off exactly the same way, and a vehicle arriving
    // at its goal must not be stopped and revoked for it.
    const bool physical_candidate_rejection =
        candidate.physical_rejection.has_value() ||
        candidate.persistent_raw_path_validation_backoff ||
        candidate.latest_lidar_path_validation_backoff ||
        physical_status(candidate.validation_status) ||
        physical_status(candidate.first_failed_validation_status);
    ProductionMppiExecutionPublication hold = publishNoExecutablePathHold(
        cycle, candidate.failure_reason, physical_candidate_rejection);
    // The hold replaces the rejected candidate; its diagnostics describe why.
    hold.arrival_shaping_attempts = candidate.arrival_shaping_attempts;
    hold.finite_path_validation_backoff = candidate.path_validation_backoff;
    hold.finite_path_validation_status = candidate.validation_status;
    hold.finite_path_rejected_precondition =
        candidate.finite_path_rejected_precondition;
    hold.finite_path_first_failed_validation_status =
        candidate.first_failed_validation_status;
    hold.latest_lidar_path_validation_backoff =
        candidate.latest_lidar_path_validation_backoff;
    return hold;
  }

  const std::shared_ptr<const ExecutionPlan3D>& committed_snapshot =
      candidate.committed_snapshot;
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
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }

  const std::span<const mppi::State> execution_states{committed_path->states};
  const std::span<const mppi::Control> execution_controls{committed_path->controls};
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
      return publishNoExecutablePathHold(
          cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
    if (committed_finite->observed_raw_world != nullptr) {
      horizon.obstacle_revision =
          committed_finite->observed_raw_world->version().revision;
    }
  }
  if (!production_mppi_execution_detail::appendFiniteExecutionPoints(
          horizon, execution_states, execution_controls,
          cycle.evidence.exact_previous_control,
          cycle.controller.finite_path_control_interval_ns)) {
    return publishNoExecutablePathHold(
        cycle, ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  if ((route_execution.pending_activation &&
       route_execution.pending_route == nullptr) ||
      commitExecutionSnapshotHorizon(
          cycle, route_execution.source_snapshot, candidate.transition.value(), horizon,
          route_execution.pending_activation ? route_execution.pending_route : nullptr,
          route_execution.certification_snapshot, route_execution.progress_preparation,
          route_execution.source_authority) !=
          ProductionMppiHorizonCommitStatus::kPublished) {
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
  publication.arrival_shaping_attempts = candidate.arrival_shaping_attempts;
  publication.first_control = execution_controls.front();
  publication.first_control_available = true;
  publication.latest_lidar_obstacle_sequence =
      cycle.evidence.latest_lidar_obstacle_sequence;
  publication.latest_lidar_obstacle_hit_count =
      cycle.evidence.latest_lidar_obstacle_points.size();
  publication.latest_lidar_obstacle_age_ms =
      cycle.evidence.latest_lidar_obstacle_age_ms;
  publication.finite_path_validation_backoff = candidate.path_validation_backoff;
  publication.finite_path_validation_status = candidate.validation_status;
  publication.finite_path_rejected_precondition =
      candidate.finite_path_rejected_precondition;
  publication.finite_path_first_failed_validation_status =
      candidate.first_failed_validation_status;
  publication.latest_lidar_obstacle_fresh = cycle.evidence.latest_lidar_obstacle_fresh;
  publication.latest_lidar_obstacle_receive_time_fallback =
      cycle.evidence.latest_lidar_obstacle_receive_time_fallback;
  publication.latest_lidar_path_validation_backoff =
      candidate.latest_lidar_path_validation_backoff;
  publication.terminal_rest_state = true;
  publication.published = true;
  return publication;
}

} // namespace drone_city_nav
