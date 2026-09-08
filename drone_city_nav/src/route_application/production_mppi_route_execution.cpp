#include "drone_city_nav/rolling_route_telemetry_3d.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <variant>

#include "../execution_route_snapshot_3d_internal.hpp"
#include "production_mppi_route_world.hpp"
#include "route_execution_selector_3d.hpp"

namespace drone_city_nav {
namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  if (!value.has_value()) {
    return nullptr;
  }
  return std::addressof(value.value());
}

[[nodiscard]] std::shared_ptr<const VersionedObservedRawWorld3D>
deriveLatestObservedRouteEvidence(
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
    const CertifiedRouteSuffix3D& route) {
  if (latest_raw_world == nullptr || route.observed_raw_world == nullptr) {
    return nullptr;
  }
  return latest_raw_world->deriveRouteEvidence(
      route.observed_raw_world->proprioceptiveFreeSpaceSeed(),
      route.observed_raw_world->launchSupportContact());
}

void bindObservedRouteEvidence(
    RouteExecutionObservation3D& observation,
    const VersionedObservedRawWorld3D& observed_world) noexcept {
  observation.latest_raw_occupancy = &observed_world.occupancy();
  observation.latest_raw_producer_instance_id =
      observed_world.version().producer_instance_id;
  observation.latest_raw_revision = observed_world.version().revision;
  observation.launch_support_contact = nullptr;
  const auto& launch_support_contact = observed_world.launchSupportContact();
  if (launch_support_contact.has_value()) {
    observation.launch_support_contact = std::addressof(launch_support_contact.value());
  }
  observation.proprioceptive_free_space_seed = nullptr;
  const auto& proprioceptive_seed = observed_world.proprioceptiveFreeSpaceSeed();
  if (proprioceptive_seed.has_value()) {
    observation.proprioceptive_free_space_seed =
        std::addressof(proprioceptive_seed.value());
  }
}

[[nodiscard]] bool trackingTubeProfileMatchesCurrentWorld(
    const CertifiedRouteSuffix3D& route,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& observed_world) {
  if (route.geometry == nullptr || route.geometry->route == nullptr ||
      route.geometry->tracking_error_tube == nullptr ||
      !route.geometry->tracking_error_tube->obstacle_evidence_available) {
    return false;
  }

  TrackingErrorTubeWorld3D world;
  std::uint64_t certified_occupied_fingerprint{0U};
  if (const auto* const raw_certificate =
          std::get_if<ObservedRawRouteCertificate3D>(&route.certificate)) {
    if (observed_world == nullptr) {
      return false;
    }
    certified_occupied_fingerprint =
        raw_certificate->geometry_derivation_occupancy_content_fingerprint;
    const auto& launch_support_contact = observed_world->launchSupportContact();
    const auto& proprioceptive_seed = observed_world->proprioceptiveFreeSpaceSeed();
    world = TrackingErrorTubeWorld3D{
        .observed_occupancy = &observed_world->occupancy(),
        .occupied_content_fingerprint = observed_world->occupiedContentFingerprint(),
        .launch_support_contact = launch_support_contact.has_value()
                                      ? std::addressof(launch_support_contact.value())
                                      : nullptr,
        .proprioceptive_free_space_seed =
            proprioceptive_seed.has_value()
                ? std::addressof(proprioceptive_seed.value())
                : nullptr,
    };
  } else {
    const auto* const static_certificate =
        std::get_if<StaticRouteCertificate3D>(&route.certificate);
    if (static_certificate == nullptr || route.static_world == nullptr) {
      return false;
    }
    certified_occupied_fingerprint =
        static_certificate->geometry_derivation_occupancy_content_fingerprint;
    world = TrackingErrorTubeWorld3D{
        .occupancy = &route.static_world->occupancy(),
        .occupied_content_fingerprint = route.static_world->contentFingerprint(),
    };
  }

  if (world.occupied_content_fingerprint == 0U ||
      certified_occupied_fingerprint == 0U) {
    return false;
  }
  return world.occupied_content_fingerprint == certified_occupied_fingerprint ||
         trackingErrorTubeProfile3DMatchesWorld(
             *route.geometry->route, *route.geometry->tracking_error_tube, world);
}

[[nodiscard]] RouteExecutionObservation3D makeExecutionObservation(
    const WorldSnapshot3D& world, const ProductionNavigationObjective* const objective,
    const ProductionMppiNavigation& navigation,
    const std::uint64_t minimum_tracking_sample_sequence,
    const double maximum_cross_track_m, const SweptFootprintConfig& footprint,
    const FlightEnvelopeConfig& flight_envelope) {
  return RouteExecutionObservation3D{
      .current_objective = objective != nullptr ? makeStaticRouteObjective(*objective)
                                                : StaticRouteObjective{},
      .minimum_tracking_sample_sequence = minimum_tracking_sample_sequence,
      .position = {navigation.state.x, navigation.state.y, navigation.state.z},
      .maximum_cross_track_m = maximum_cross_track_m,
      .footprint = footprint,
      .launch_support_contact = world.launch_support_contact
                                    ? std::addressof(*world.launch_support_contact)
                                    : nullptr,
      .proprioceptive_free_space_seed =
          world.proprioceptive_free_space_seed
              ? std::addressof(*world.proprioceptive_free_space_seed)
              : nullptr,
      .flight_envelope = flight_envelope,
  };
}

[[nodiscard]] RouteActivationObservation3D makeActivationObservation(
    const WorldSnapshot3D& world, const ProductionNavigationObjective* const objective,
    const ProductionMppiNavigation& navigation,
    const std::uint64_t minimum_tracking_sample_sequence,
    const double maximum_cross_track_m, const SweptFootprintConfig& footprint,
    const FlightEnvelopeConfig& flight_envelope, const bool raw_validation_required) {
  return RouteActivationObservation3D{
      .resident_world = navigationWorldCertificate3D(world),
      .current_objective = objective != nullptr ? makeStaticRouteObjective(*objective)
                                                : StaticRouteObjective{},
      .minimum_tracking_sample_sequence = minimum_tracking_sample_sequence,
      .position = {navigation.state.x, navigation.state.y, navigation.state.z},
      .maximum_cross_track_m = maximum_cross_track_m,
      .footprint = footprint,
      .flight_envelope = flight_envelope,
      .raw_validation_required = raw_validation_required,
  };
}

[[nodiscard]] std::shared_ptr<const CertifiedRouteSuffix3D> refreshPendingRoute(
    const PendingCertifiedRoute3D& pending, const WorldSnapshot3D& world,
    const ProductionNavigationObjective* const objective,
    const ProductionMppiNavigation& navigation,
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
    const std::uint64_t minimum_tracking_sample_sequence,
    const double maximum_cross_track_m, const SweptFootprintConfig& footprint,
    const FlightEnvelopeConfig& flight_envelope) {
  const bool observed = pending.route.observed_raw_world != nullptr;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_owner;
  if (observed) {
    observed_owner = deriveLatestObservedRouteEvidence(latest_raw_world, pending.route);
    if (observed_owner == nullptr) {
      return nullptr;
    }
  }
  const RouteActivationObservation3D observation = makeActivationObservation(
      world, objective, navigation, minimum_tracking_sample_sequence,
      maximum_cross_track_m, footprint, flight_envelope, observed);
  const std::optional<CertifiedRouteSuffix3D> refreshed =
      recertifyExecutionRoute3D(pending.route, observation, std::move(observed_owner));
  return refreshed.has_value()
             ? std::make_shared<const CertifiedRouteSuffix3D>(*refreshed)
             : nullptr;
}

[[nodiscard]] bool pendingRouteSnapshotSemanticallyCurrent(
    const PendingCertifiedRoute3D& pending, const WorldSnapshot3D& world,
    const ProductionNavigationObjective* const objective,
    const std::uint64_t minimum_tracking_sample_sequence) noexcept {
  return objective != nullptr &&
         assessRoutePublication3D(pending.route.identity.proposal,
                                  navigationWorldCertificate3D(world))
             .compatible() &&
         staticRouteObjectiveMatches(pending.route.identity.proposal.objective,
                                     makeStaticRouteObjective(*objective),
                                     minimum_tracking_sample_sequence,
                                     std::numeric_limits<double>::infinity());
}

[[nodiscard]] RouteProgressProjection3D
routeProjection(const CertifiedRouteSuffix3D& route,
                const Point3& current_position) noexcept {
  RouteProgressProjection3D projection;
  if (route.geometry == nullptr || route.geometry->route == nullptr ||
      route.geometry->route->empty()) {
    return projection;
  }
  const RouteProjection3D measured = projectOntoRoute3DWithinStationWindow(
      *route.geometry->route, current_position, route.progress.station_m,
      route.endStationM());
  if (!measured.valid) {
    return projection;
  }
  projection.valid = true;
  projection.station_m = route.progress.station_m;
  projection.total_length_m = route.endStationM();
  projection.remaining_m = route.remainingM();
  projection.cross_track_m = measured.distance_m;
  projection.point = {measured.point.x, measured.point.y};
  return projection;
}

[[nodiscard]] bool
pendingRoutePermanentlyObsolete(const PendingCertifiedRoute3D& pending,
                                const ExecutionPlan3D& snapshot) noexcept {
  if (!pending.valid() || !snapshot.valid()) {
    return false;
  }
  if (snapshot.execution_owner_epoch < pending.base_execution_owner_epoch) {
    return false;
  }
  return snapshot.execution_owner_epoch > pending.base_execution_owner_epoch ||
         !pendingCertifiedRouteEligible3D(pending, snapshot);
}

} // namespace

RouteExecutionSelector3D::RouteExecutionSelector3D(
    ExecutionSupervisor3D& execution_supervisor,
    const RouteExecutionSelectorConfig3D& config)
    : execution_supervisor_{execution_supervisor},
      config_{config} {
}

std::optional<double> RouteExecutionSelector3D::latestLidarBlockedStation(
    const CertifiedRouteSuffix3D& route, const RouteProjection3D& projection,
    const VersionedLatestLidarEvidence3D& latest_lidar,
    const VersionedObservedRawWorld3D* const contact_world) {
  const std::uint64_t geometry_revision =
      route.geometry != nullptr ? route.geometry->compiled_trajectory_revision : 0U;
  LatestLidarWindowCache3D& cache = latest_lidar_window_cache_;
  if (cache.valid &&
      cache.lidar_producer_instance_id == latest_lidar.producerInstanceId() &&
      cache.lidar_sequence == latest_lidar.sequence() &&
      cache.route_generation == route.identity.generation &&
      cache.geometry_revision == geometry_revision) {
    return cache.blocked_station_m;
  }
  std::optional<double> blocked_station_m;
  if (route.geometry != nullptr && route.geometry->route != nullptr) {
    const std::vector<RouteSample3D>& samples = *route.geometry->route;
    const LaunchSupportContact3D* launch_support_contact{nullptr};
    const ProprioceptiveFreeSpaceSeed3D* proprioceptive_seed{nullptr};
    if (contact_world != nullptr) {
      launch_support_contact = optionalAddress(contact_world->launchSupportContact());
      proprioceptive_seed =
          optionalAddress(contact_world->proprioceptiveFreeSpaceSeed());
    }
    const RawRouteSuffixValidation3D window = validateRawRouteWindow3D(
        samples, projection, config_.latest_lidar_route_lookahead_m,
        OccupiedCollisionWorld3D{
            .observed_occupancy = nullptr,
            .static_occupancy = nullptr,
            .planar_occupancy = nullptr,
            .raw_point_cloud = latest_lidar.indexedHitPoints(),
            .launch_support_contact = launch_support_contact,
            .proprioceptive_free_space_seed = proprioceptive_seed,
            .footprint = config_.physical_footprint,
            .flight_envelope = config_.flight_envelope,
        });
    if (window.status == RawRouteSuffixStatus3D::kRawCollision && !samples.empty()) {
      const std::size_t blocked_segment =
          std::min(window.failure_route_segment, samples.size() - 1U);
      blocked_station_m = samples[blocked_segment].station_m;
    }
  }
  cache = LatestLidarWindowCache3D{
      .lidar_producer_instance_id = latest_lidar.producerInstanceId(),
      .lidar_sequence = latest_lidar.sequence(),
      .route_generation = route.identity.generation,
      .geometry_revision = geometry_revision,
      .blocked_station_m = blocked_station_m,
      .valid = true,
  };
  return blocked_station_m;
}

RouteExecutionSelectorResult3D
RouteExecutionSelector3D::select(const RouteExecutionSelectorRequest3D& request) {
  RouteExecutionSelectorResult3D output;
  if (!request.valid()) {
    return output;
  }
  const WorldSnapshot3D& world = *request.world;
  const ProductionNavigationObjective* const objective = request.objective;
  const ProductionMppiNavigation& navigation = request.navigation;
  const auto& execution_input = request.execution_input;
  const auto& latest_raw_world = request.latest_raw_world;
  const auto& latest_lidar_evidence = request.latest_lidar_evidence;
  const std::int64_t validation_stamp_ns = request.validation_stamp_ns;
  const std::uint64_t minimum_tracking_sample_sequence =
      request.minimum_tracking_sample_sequence;
  const bool observed_3d_world = request.observed_3d_world;
  ProductionRouteExecutionSelection3D& result = output.selection;
  result = ProductionRouteExecutionSelection3D{
      .route = nullptr,
      .source_authority = nullptr,
      .source_snapshot = nullptr,
      .certification_snapshot = nullptr,
      .progress_preparation = nullptr,
      .pending_route = nullptr,
      .lifecycle_observed_raw_world = nullptr,
      .projection = {},
      .tracking_error_tube = {},
      .tracking_error_tube_handoff = {},
      .status = RouteExecutionStatus3D::kNoActiveRoute,
      .lifecycle_event = std::nullopt,
      .hold_position =
          Point3{navigation.state.x, navigation.state.y,
                 clampToFlightEnvelope(navigation.state.z, config_.flight_envelope)
                     .value_or(config_.flight_envelope.minimum_target_z_m)},
      .station_m = 0.0,
      .route_usable = false,
      .tracking_error_tube_handoff_active = false,
      .execution_owner_available = false,
      .pending_activation = false,
      .physical_trajectory_invalidated = false,
      .raw_blocked_station_m = std::nullopt,
      .latest_lidar_blocked_station_m = std::nullopt,
      .direct_tracking_identity = request.direct_tracking_identity,
  };
  const RouteExecutionManagerSnapshot3D manager_snapshot =
      execution_supervisor_.snapshot();
  result.source_authority = manager_snapshot.authority;
  result.source_snapshot =
      result.source_authority != nullptr ? result.source_authority->plan() : nullptr;
  result.certification_snapshot = result.source_snapshot;
  result.execution_owner_available =
      result.source_snapshot != nullptr &&
      (result.source_snapshot->finiteExecution() != nullptr ||
       result.source_snapshot->directTrackingExecution() != nullptr ||
       result.source_snapshot->stopExecution() != nullptr ||
       result.source_snapshot->stationaryHold() != nullptr);
  result.stationary_hold_owner =
      result.source_snapshot != nullptr &&
      result.source_snapshot->finiteExecution() == nullptr &&
      result.source_snapshot->directTrackingExecution() == nullptr &&
      result.source_snapshot->stationaryHold() != nullptr;
  if (result.source_snapshot == nullptr) {
    return output;
  }
  const std::shared_ptr<const PendingCertifiedRoute3D>& stale_pending =
      manager_snapshot.pending;
  if (stale_pending != nullptr &&
      !pendingCertifiedRouteEligible3D(*stale_pending, *result.source_snapshot) &&
      pendingRoutePermanentlyObsolete(*stale_pending, *result.source_snapshot)) {
    static_cast<void>(execution_supervisor_.acknowledgePendingIfSame(stale_pending));
  }
  if (result.direct_tracking_identity.has_value()) {
    return output;
  }
  if (execution_input == nullptr || !execution_input->valid() ||
      !execution_input->nominalStateAuthoritative()) {
    return output;
  }
  ProductionMppiNavigation execution_navigation = navigation;
  execution_navigation.state = execution_input->state();

  const SweptFootprintConfig& footprint = config_.physical_footprint;
  const double execution_maximum_cross_track_m =
      config_.route_cross_track_constraints_enabled
          ? config_.route_tracking.maximum_cross_track_m
          : std::numeric_limits<double>::max();
  bool active_usable{false};
  if (result.source_snapshot->route() != nullptr) {
    const std::shared_ptr<const ExecutionPlan3D> active_source_snapshot =
        result.source_snapshot;
    const CertifiedRouteSuffix3D& active_route = *active_source_snapshot->route();
    const FiniteExecutionState3D* const active_finite_execution =
        active_source_snapshot->finiteExecution();
    const ExecutionRouteTransitionGuard3D guard{
        .expected_snapshot_version = result.source_snapshot->version,
        .expected_route_generation = active_route.identity.generation,
        .expected_geometry_revision =
            active_route.geometry->compiled_trajectory_revision,
    };
    const bool observed_route = active_route.observed_raw_world != nullptr;
    const std::uint64_t physically_invalidated_through_generation =
        request.physically_invalidated_through_generation;
    const bool physical_invalidation_latched =
        physically_invalidated_through_generation >= active_route.identity.generation;
    if (physical_invalidation_latched) {
      result.status = RouteExecutionStatus3D::kRawCollision;
      result.physical_trajectory_invalidated = true;
    } else {
      if (observed_route && !observed_3d_world) {
        return output;
      }
      RouteExecutionObservation3D observation = makeExecutionObservation(
          world, objective, execution_navigation, minimum_tracking_sample_sequence,
          execution_maximum_cross_track_m, footprint, config_.flight_envelope);
      std::shared_ptr<const VersionedObservedRawWorld3D> latest_observed_owner;
      std::shared_ptr<const VersionedObservedRawWorld3D> collision_observed_owner;
      bool active_trajectory_raw_collision{false};
      bool active_trajectory_latest_lidar_collision{false};
      FiniteExecutionPathValidation3D active_trajectory_raw_validation;
      FiniteExecutionPathValidation3D active_trajectory_lidar_validation;
      if (observed_route) {
        latest_observed_owner =
            deriveLatestObservedRouteEvidence(latest_raw_world, active_route);
        if (latest_observed_owner != nullptr &&
            active_source_snapshot->phase() == ExecutionRoutePhase3D::kFollowing &&
            active_finite_execution != nullptr) {
          active_trajectory_raw_validation =
              validateRemainingFiniteExecutionAgainstObservedWorld3D(
                  *active_finite_execution, &active_route, *execution_input,
                  *latest_observed_owner, execution_input->effectiveStampNs());
          active_trajectory_raw_collision = active_trajectory_raw_validation.status ==
                                            FiniteExecutionPathStatus3D::kRawCollision;
        }
        // A memory revision is not itself an invalidation. Use its owned raw
        // occupancy only as a collision witness for the remaining resident
        // route. Progress stays bound to the immutable certifying world below.
        const std::shared_ptr<const VersionedObservedRawWorld3D>&
            route_collision_world =
                latest_observed_owner != nullptr ? latest_observed_owner
                                                 : active_route.observed_raw_world;
        if (route_collision_world != nullptr) {
          bindObservedRouteEvidence(observation, *route_collision_world);
        }
      }
      if (latest_lidar_evidence != nullptr &&
          active_source_snapshot->phase() == ExecutionRoutePhase3D::kFollowing &&
          active_finite_execution != nullptr) {
        active_trajectory_lidar_validation =
            validateRemainingFiniteExecutionAgainstLatestLidar3D(
                *active_finite_execution, &active_route, *execution_input,
                *latest_lidar_evidence, validation_stamp_ns);
        active_trajectory_latest_lidar_collision =
            active_trajectory_lidar_validation.status ==
            FiniteExecutionPathStatus3D::kLatestLidarRawCollision;
      }
      const auto* const raw_certificate =
          std::get_if<ObservedRawRouteCertificate3D>(&active_route.certificate);
      observation.previously_validated_through_raw_revision =
          raw_certificate != nullptr ? raw_certificate->validated_through_revision : 0U;
      observation.minimum_station_m = active_route.progress.station_m;
      observation.maximum_station_m =
          std::min(active_route.endStationM(),
                   active_route.progress.station_m +
                       distance3D(active_route.progress.last_observed_position,
                                  observation.position) +
                       1.0e-6);
      RouteExecutionAssessment3D diagnostic_assessment = assessRouteExecution3D(
          &active_route.identity, *active_route.geometry->route, observation);
      const bool resident_route_raw_collision =
          diagnostic_assessment.status == RouteExecutionStatus3D::kRawCollision;
      const ProductionMppiResidentObstacleDisposition obstacle_disposition =
          residentObstacleDisposition(ProductionMppiResidentObstacleEvidence{
              .route_suffix_persistent_raw = resident_route_raw_collision,
              .finite_execution_persistent_raw = active_trajectory_raw_collision,
              .finite_execution_latest_lidar = active_trajectory_latest_lidar_collision,
          });
      const bool route_suffix_replacement_required =
          obstacle_disposition ==
          ProductionMppiResidentObstacleDisposition::kRouteSuffixReplacementRequired;
      const bool finite_execution_physically_invalidated =
          obstacle_disposition == ProductionMppiResidentObstacleDisposition::
                                      kPersistentRawFiniteExecutionInvalidated ||
          obstacle_disposition == ProductionMppiResidentObstacleDisposition::
                                      kLatestLidarFiniteExecutionInvalidated;
      result.physical_trajectory_invalidated = finite_execution_physically_invalidated;
      // The latest scan bounds the route ahead as it bounds the horizon. The
      // persistent memory integrates a hit only after its own confidence
      // stages, and until then the route is clear to everything but the
      // horizon's own validation, which meets the hit at the end of the
      // horizon and stops the vehicle hard. The scan is checked along the
      // route for the distance the vehicle needs to react, and a hit there
      // is a station the speed policy brakes toward, as toward a persistent
      // block; the persistent memory stays the authority on the route itself.
      if (observed_route && latest_lidar_evidence != nullptr &&
          active_source_snapshot->phase() == ExecutionRoutePhase3D::kFollowing &&
          active_finite_execution != nullptr &&
          active_finite_execution->validation_policy != nullptr &&
          config_.latest_lidar_route_lookahead_m > 0.0 &&
          diagnostic_assessment.projection.valid &&
          execution_route_snapshot_3d_internal::latestLidarEvidenceFreshAt(
              *latest_lidar_evidence, *active_finite_execution->validation_policy,
              validation_stamp_ns)) {
        const std::shared_ptr<const VersionedObservedRawWorld3D>& contact_world =
            latest_observed_owner != nullptr ? latest_observed_owner
                                             : active_route.observed_raw_world;
        result.latest_lidar_blocked_station_m =
            latestLidarBlockedStation(active_route, diagnostic_assessment.projection,
                                      *latest_lidar_evidence, contact_world.get());
      }
      const bool persistent_raw_collision =
          obstacle_disposition == ProductionMppiResidentObstacleDisposition::
                                      kPersistentRawFiniteExecutionInvalidated;
      if (persistent_raw_collision) {
        collision_observed_owner = latest_observed_owner != nullptr
                                       ? latest_observed_owner
                                       : active_route.observed_raw_world;
      }
      if (route_suffix_replacement_required) {
        const std::vector<RouteSample3D>& blocked_route = *active_route.geometry->route;
        const std::size_t blocked_segment =
            std::min(diagnostic_assessment.raw_validation.failure_route_segment,
                     blocked_route.size() - 1U);
        result.raw_blocked_station_m = blocked_route[blocked_segment].station_m;
        output.effects.push_back(RouteExecutionSelectorEffect3D{
            .kind = RouteExecutionSelectorEffectKind3D::kRequestRouteRelease,
            .release_reason = RouteReleaseReason3D::kBlocked,
            .obstacle_disposition = obstacle_disposition,
            .observed_raw_world = nullptr,
            .route_generation = active_route.identity.generation,
        });
      } else if (finite_execution_physically_invalidated) {
        const FiniteExecutionPathValidation3D& physical_validation =
            obstacle_disposition == ProductionMppiResidentObstacleDisposition::
                                        kPersistentRawFiniteExecutionInvalidated
                ? active_trajectory_raw_validation
                : active_trajectory_lidar_validation;
        diagnostic_assessment.status = RouteExecutionStatus3D::kRawCollision;
        diagnostic_assessment.raw_validation.status =
            RawRouteSuffixStatus3D::kRawCollision;
        diagnostic_assessment.raw_validation.failure_point =
            physical_validation.failure_point;
      }
      if ((diagnostic_assessment.usable() || route_suffix_replacement_required) &&
          config_.route_tracking_tube_constraints_enabled) {
        if (!trackingTubeProfileMatchesCurrentWorld(active_route,
                                                    active_route.observed_raw_world)) {
          result.tracking_error_tube.status =
              TrackingErrorTubeExecutionStatus3D::kInvalidProfile;
          diagnostic_assessment.status = RouteExecutionStatus3D::kInvalidRoute;
        } else {
          result.tracking_error_tube = assessTrackingErrorTubeExecution3D(
              *active_route.geometry->route,
              *active_route.geometry->tracking_error_tube,
              TrackingErrorTubeExecutionObservation3D{
                  .station_m = diagnostic_assessment.projection.station_m,
                  .cross_track_error_m = diagnostic_assessment.projection.distance_m,
                  .speed_mps = routeSpeed3D(Vec3{execution_navigation.state.vx,
                                                 execution_navigation.state.vy,
                                                 execution_navigation.state.vz}),
              });
        }
        if (!result.tracking_error_tube.accepted()) {
          if (result.tracking_error_tube.status ==
                  TrackingErrorTubeExecutionStatus3D::kSpeedLimitExceeded ||
              result.tracking_error_tube.status ==
                  TrackingErrorTubeExecutionStatus3D::kCrossTrackExceeded) {
            result.tracking_error_tube_handoff = assessCertifiedTrackingTubeHandoff3D(
                *active_source_snapshot, active_route, *execution_input);
          }
          result.tracking_error_tube_handoff_active =
              result.tracking_error_tube_handoff.active();
          if (!result.tracking_error_tube_handoff_active) {
            diagnostic_assessment.status =
                result.tracking_error_tube.status ==
                        TrackingErrorTubeExecutionStatus3D::kInvalidObservation
                    ? RouteExecutionStatus3D::kInvalidRoute
                    : RouteExecutionStatus3D::kTrackingTubeViolation;
          }
        }
      }
      const bool route_execution_assessment_usable =
          diagnostic_assessment.usable() ||
          (route_suffix_replacement_required &&
           diagnostic_assessment.status == RouteExecutionStatus3D::kRawCollision);
      const ExecutionRouteTransitionResult3D advanced =
          route_execution_assessment_usable &&
                  !result.tracking_error_tube_handoff_active
              ? advanceCertifiedRoute3D(*result.source_snapshot, guard, observation,
                                        execution_input,
                                        active_route.observed_raw_world)
              : ExecutionRouteTransitionResult3D{};
      const bool optional_policy_invalidation =
          (config_.route_cross_track_constraints_enabled &&
           diagnostic_assessment.status ==
               RouteExecutionStatus3D::kExcessiveCrossTrack) ||
          (config_.route_tracking_tube_constraints_enabled &&
           diagnostic_assessment.status ==
               RouteExecutionStatus3D::kTrackingTubeViolation);
      const bool route_invalidation_required =
          finite_execution_physically_invalidated ||
          diagnostic_assessment.status == RouteExecutionStatus3D::kObjectiveMismatch ||
          optional_policy_invalidation;
      if (result.tracking_error_tube_handoff_active ||
          (!route_execution_assessment_usable && !route_invalidation_required)) {
        // Contract/projection diagnostics and connector history do not describe
        // a physical intersection of the published finite execution. The same
        // retention rule applies while an immutable tracking-tube handoff owns
        // the controller. Keep the certified owner and retry.
        active_usable = true;
        result.status = RouteExecutionStatus3D::kUsable;
      } else if (!advanced.applied()) {
        if (advanced.status == ExecutionRouteTransitionStatus3D::kNoChange) {
          active_usable = route_execution_assessment_usable;
        } else if (route_execution_assessment_usable) {
          // Progress and publication are optimistic transactions. A stale guard,
          // non-monotonic projection, or another snapshot conflict only means
          // that this observation must be retried against the current owner. It
          // is not physical evidence and must never synthesize a raw collision.
          active_usable = true;
          result.status = RouteExecutionStatus3D::kUsable;
        } else {
          result.status = diagnostic_assessment.status;
          const std::uint64_t generation = active_route.identity.generation;
          const bool raw_invalidated = persistent_raw_collision;
          const bool latest_lidar_invalidated =
              obstacle_disposition == ProductionMppiResidentObstacleDisposition::
                                          kLatestLidarFiniteExecutionInvalidated;
          if (raw_invalidated) {
            result.lifecycle_observed_raw_world = collision_observed_owner;
          }
          RouteLifecycleEventKind3D event_kind =
              RouteLifecycleEventKind3D::kControlCandidateRejected;
          RouteReleaseReason3D release_reason = RouteReleaseReason3D::kBlocked;
          if (raw_invalidated) {
            event_kind = RouteLifecycleEventKind3D::kRawInvalidated;
          } else if (latest_lidar_invalidated) {
            event_kind = RouteLifecycleEventKind3D::kLatestLidarInvalidated;
          } else if (result.status == RouteExecutionStatus3D::kObjectiveMismatch) {
            event_kind = RouteLifecycleEventKind3D::kObjectiveSuperseded;
            release_reason = RouteReleaseReason3D::kObjectiveChanged;
          } else if (result.status == RouteExecutionStatus3D::kExcessiveCrossTrack) {
            event_kind = RouteLifecycleEventKind3D::kCrossTrackExceeded;
            release_reason = RouteReleaseReason3D::kDiverged;
          } else if (result.status == RouteExecutionStatus3D::kTrackingTubeViolation) {
            event_kind = RouteLifecycleEventKind3D::kTrackingTubeExceeded;
            release_reason = RouteReleaseReason3D::kDiverged;
          }
          result.lifecycle_event = RouteLifecycleEvent3D{
              .kind = event_kind,
              .generation = generation,
              .raw_producer_instance_id =
                  raw_invalidated && collision_observed_owner != nullptr
                      ? collision_observed_owner->version().producer_instance_id
                      : 0U,
              .raw_revision = raw_invalidated && collision_observed_owner != nullptr
                                  ? collision_observed_owner->version().revision
                                  : 0U,
              .latest_lidar_evidence =
                  latest_lidar_invalidated && latest_lidar_evidence != nullptr
                      ? latest_lidar_evidence->evidenceId()
                      : LatestLidarEvidenceId3D{},
          };
          if (raw_invalidated || latest_lidar_invalidated) {
            output.effects.push_back(RouteExecutionSelectorEffect3D{
                .kind = RouteExecutionSelectorEffectKind3D::
                    kHandlePhysicalTrajectoryCollision,
                .release_reason = RouteReleaseReason3D::kBlocked,
                .obstacle_disposition = obstacle_disposition,
                .observed_raw_world =
                    raw_invalidated ? collision_observed_owner : nullptr,
                .route_generation = generation,
            });
          } else {
            output.effects.push_back(RouteExecutionSelectorEffect3D{
                .kind = RouteExecutionSelectorEffectKind3D::kRequestRouteRelease,
                .release_reason = release_reason,
                .obstacle_disposition = obstacle_disposition,
                .observed_raw_world = nullptr,
                .route_generation = generation,
            });
          }
        }
      } else {
        // This transition intentionally remains caller-local. Publishing it would
        // expose progress and route-certificate generations without the finite
        // command horizon and its immediate braking fallback. The planning stage
        // certifies both against this exact snapshot and composes the two
        // transitions into one manager publication rooted at source_snapshot.
        result.progress_preparation =
            std::make_shared<const ExecutionRouteTransitionResult3D>(advanced);
        result.certification_snapshot = advanced.next;
        active_usable = true;
      }
    }
  }

  const std::shared_ptr<const ExecutionPlan3D>& route_state =
      result.certification_snapshot != nullptr ? result.certification_snapshot
                                               : result.source_snapshot;
  result.pending_route = execution_supervisor_.pending();
  if (result.pending_route != nullptr &&
      !pendingCertifiedRouteEligible3D(*result.pending_route, *route_state) &&
      pendingRoutePermanentlyObsolete(*result.pending_route, *route_state)) {
    if (execution_supervisor_.acknowledgePendingIfSame(result.pending_route)) {
      result.pending_route.reset();
    } else {
      // A newer publication defeated the exact acknowledgement. Preserve that
      // resident identity so this tick may assess it and recovery cannot
      // mistake the caller-local stale pointer for an empty pending slot.
      result.pending_route = execution_supervisor_.pending();
    }
  }
  if (result.pending_route != nullptr &&
      pendingCertifiedRouteEligible3D(*result.pending_route, *route_state)) {
    const bool snapshot_retention_authorized =
        pendingCertifiedRouteRetainsSnapshotCertificate3D(*result.pending_route);
    const bool retain_snapshot_certificate =
        snapshot_retention_authorized &&
        pendingRouteSnapshotSemanticallyCurrent(*result.pending_route, world, objective,
                                                minimum_tracking_sample_sequence);
    std::shared_ptr<const CertifiedRouteSuffix3D> refreshed_pending =
        retain_snapshot_certificate
            ? std::make_shared<const CertifiedRouteSuffix3D>(
                  result.pending_route->route)
            : refreshPendingRoute(*result.pending_route, world, objective,
                                  execution_navigation, latest_raw_world,
                                  minimum_tracking_sample_sequence,
                                  config_.route_tracking.maximum_cross_track_m,
                                  footprint, config_.flight_envelope);
    RouteSpliceReadiness3D splice_readiness{.status =
                                                RouteSpliceReadinessStatus3D::kReady};
    const bool route_splice_required =
        result.pending_route->base_kind == PendingExecutionBaseKind3D::kRoute;
    const std::optional<CertifiedRouteSplice3D>& route_splice_candidate =
        result.pending_route->route_splice;
    const CertifiedRouteSplice3D* route_splice = nullptr;
    if (route_splice_candidate.has_value()) {
      route_splice = std::addressof(route_splice_candidate.value());
    }
    const CertifiedRouteSuffix3D* const resident_route = route_state->route();
    if (route_splice_required) {
      if (route_splice == nullptr || resident_route == nullptr) {
        splice_readiness.status = RouteSpliceReadinessStatus3D::kInvalidProof;
      } else if (refreshed_pending == nullptr) {
        splice_readiness.status =
            RouteSpliceReadinessStatus3D::kSuccessorProjectionUnavailable;
      } else {
        splice_readiness = assessRouteSpliceReadiness3D(
            *route_splice, *resident_route, *refreshed_pending,
            Point3{execution_navigation.state.x, execution_navigation.state.y,
                   execution_navigation.state.z});
      }
    }
    if (refreshed_pending != nullptr && splice_readiness.ready()) {
      result.route = std::move(refreshed_pending);
      result.pending_activation = true;
      result.route_usable = true;
      result.status = RouteExecutionStatus3D::kUsable;
    } else if (route_splice_required) {
      const bool splice_expired =
          route_splice != nullptr && resident_route != nullptr &&
          routeSpliceWindowExpired3D(*route_splice, *resident_route);
      const bool permanently_unavailable =
          splice_expired || !splice_readiness.canStillBecomeReady();
      if (permanently_unavailable &&
          execution_supervisor_.acknowledgePendingIfSame(result.pending_route)) {
        result.pending_route.reset();
      }
    } else if (snapshot_retention_authorized) {
      const bool acknowledged =
          execution_supervisor_.acknowledgePendingIfSame(result.pending_route);
      if (acknowledged) {
        result.pending_route.reset();
      } else {
        result.pending_route = execution_supervisor_.pending();
      }
    }
  }

  if (result.route == nullptr && active_usable &&
      (route_state->phase() == ExecutionRoutePhase3D::kFollowing ||
       route_state->phase() == ExecutionRoutePhase3D::kAwaitingSuccessor) &&
      route_state->route() != nullptr) {
    result.route =
        std::make_shared<const CertifiedRouteSuffix3D>(*route_state->route());
    result.route_usable = true;
    result.status = RouteExecutionStatus3D::kUsable;
  }
  if (result.route != nullptr) {
    result.projection = routeProjection(
        *result.route, Point3{execution_input->state().x, execution_input->state().y,
                              execution_input->state().z});
    result.station_m = result.route->progress.station_m;
    result.route_usable = result.route_usable && result.projection.valid &&
                          (!config_.route_cross_track_constraints_enabled ||
                           result.tracking_error_tube_handoff_active ||
                           result.projection.cross_track_m <=
                               config_.route_tracking.maximum_cross_track_m);
    if (!result.route_usable) {
      result.status = result.projection.valid
                          ? RouteExecutionStatus3D::kExcessiveCrossTrack
                          : RouteExecutionStatus3D::kInvalidProjection;
      result.route.reset();
    }
  }
  return output;
}

} // namespace drone_city_nav
