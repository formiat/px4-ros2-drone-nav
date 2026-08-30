#include <algorithm>
#include <cinttypes>
#include <limits>
#include <memory>
#include <variant>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool sameRawMapVersion(const RawMapVersion& first,
                                     const RawMapVersion& second) noexcept {
  return first.producer_instance_id == second.producer_instance_id &&
         first.base_snapshot_revision == second.base_snapshot_revision &&
         first.revision == second.revision;
}

[[nodiscard]] bool
rawWorldExecutionOwnerExact(const ProductionMppiRawWorld3D& raw_world) noexcept {
  return raw_world.version.valid() && raw_world.occupancy != nullptr &&
         raw_world.execution_owner != nullptr && raw_world.execution_owner->valid() &&
         sameRawMapVersion(raw_world.version, raw_world.execution_owner->version()) &&
         std::addressof(raw_world.execution_owner->occupancy()) ==
             raw_world.occupancy.get();
}

[[nodiscard]] std::shared_ptr<const VersionedObservedRawWorld3D>
deriveLatestObservedRouteEvidence(
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
    const CertifiedRouteSuffix3D& route) {
  if (latest_raw_world == nullptr || !rawWorldExecutionOwnerExact(*latest_raw_world) ||
      route.observed_raw_world == nullptr) {
    return nullptr;
  }
  const std::shared_ptr<const VersionedObservedRawWorld3D> derived =
      latest_raw_world->execution_owner->deriveRouteEvidence(
          route.observed_raw_world->proprioceptiveFreeSpaceSeed(),
          route.observed_raw_world->launchSupportContact());
  if (derived == nullptr || !derived->valid() ||
      !sameRawMapVersion(derived->version(), latest_raw_world->version) ||
      std::addressof(derived->occupancy()) != latest_raw_world->occupancy.get() ||
      !latest_raw_world->execution_owner->sharesObservationOwner(*derived) ||
      derived->occupiedSnapshot() !=
          latest_raw_world->execution_owner->occupiedSnapshot()) {
    return nullptr;
  }
  return derived;
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
}

[[nodiscard]] const char* residentObstacleSource(
    const ProductionMppiResidentObstacleDisposition disposition) noexcept {
  switch (disposition) {
    case ProductionMppiResidentObstacleDisposition::kRouteSuffixReplacementRequired:
      return "resident_route_suffix_persistent_raw";
    case ProductionMppiResidentObstacleDisposition::
        kPersistentRawFiniteExecutionInvalidated:
      return "active_finite_trajectory_persistent_raw";
    case ProductionMppiResidentObstacleDisposition::
        kLatestLidarFiniteExecutionInvalidated:
      return "active_finite_trajectory_latest_lidar";
    case ProductionMppiResidentObstacleDisposition::kClear:
      return "none";
  }
  return "none";
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
    world = TrackingErrorTubeWorld3D{
        .observed_occupancy = &observed_world->occupancy(),
        .occupied_content_fingerprint = observed_world->occupiedContentFingerprint(),
        .launch_support_contact = observed_world->launchSupportContact().has_value()
                                      ? &*observed_world->launchSupportContact()
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
                                const ExecutionRouteSnapshot3D& snapshot) noexcept {
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

ProductionRouteExecutionSelection3D ProductionMppiNode::resolveRouteExecution3D(
    const WorldSnapshot3D& world, const ProductionNavigationObjective* const objective,
    const ProductionMppiNavigation& navigation,
    const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
    const std::shared_ptr<const VersionedLatestLidarEvidence3D>& latest_lidar_evidence,
    const std::int64_t validation_stamp_ns,
    const std::uint64_t minimum_tracking_sample_sequence,
    std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity,
    const bool observed_3d_world) {
  ProductionRouteExecutionSelection3D result{
      .route = nullptr,
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
                 clampToFlightEnvelope(navigation.state.z, flight_envelope_config_)
                     .value_or(flight_envelope_config_.minimum_target_z_m)},
      .station_m = 0.0,
      .route_usable = false,
      .tracking_error_tube_handoff_active = false,
      .execution_owner_available = false,
      .pending_activation = false,
      .physical_trajectory_invalidated = false,
      .direct_tracking_identity = std::move(direct_tracking_identity),
  };
  result.source_snapshot = execution_route_store_.snapshot();
  result.certification_snapshot = result.source_snapshot;
  result.execution_owner_available =
      result.source_snapshot != nullptr &&
      (result.source_snapshot->finite_execution.has_value() ||
       result.source_snapshot->direct_tracking_execution.has_value() ||
       result.source_snapshot->stationary_hold.has_value());
  if (result.source_snapshot == nullptr) {
    return result;
  }
  const std::shared_ptr<const PendingCertifiedRoute3D> stale_pending =
      pending_certified_route_mailbox_.snapshot();
  if (stale_pending != nullptr &&
      !pendingCertifiedRouteEligible3D(*stale_pending, *result.source_snapshot) &&
      pendingRoutePermanentlyObsolete(*stale_pending, *result.source_snapshot)) {
    static_cast<void>(
        pending_certified_route_mailbox_.acknowledgeIfSame(stale_pending));
  }
  if (result.direct_tracking_identity.has_value()) {
    return result;
  }
  if (execution_input == nullptr || !execution_input->valid() ||
      !execution_input->nominalStateAuthoritative()) {
    return result;
  }
  ProductionMppiNavigation execution_navigation = navigation;
  execution_navigation.state = execution_input->state();

  const SweptFootprintConfig& footprint = physical_footprint_config_;
  const double execution_maximum_cross_track_m =
      optional_constraints_.route_cross_track_constraints_enabled
          ? route_tracking_policy_.maximum_cross_track_m
          : std::numeric_limits<double>::max();
  bool active_usable{false};
  if (result.source_snapshot->route.has_value()) {
    const std::shared_ptr<const ExecutionRouteSnapshot3D> active_source_snapshot =
        result.source_snapshot;
    const CertifiedRouteSuffix3D& active_route = *active_source_snapshot->route;
    const ExecutionRouteTransitionGuard3D guard{
        .expected_snapshot_version = result.source_snapshot->version,
        .expected_route_generation = result.source_snapshot->route->identity.generation,
        .expected_geometry_revision =
            result.source_snapshot->route->geometry->compiled_trajectory_revision,
    };
    const bool observed_route = active_route.observed_raw_world != nullptr;
    const std::uint64_t physically_invalidated_through_generation =
        physical_trajectory_replan_route_generation_.load(std::memory_order_acquire);
    const bool physical_invalidation_latched =
        physically_invalidated_through_generation >= active_route.identity.generation;
    if (physical_invalidation_latched) {
      result.status = RouteExecutionStatus3D::kRawCollision;
      result.physical_trajectory_invalidated = true;
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ROUTE_EXECUTION3D snapshot_version=%" PRIu64 " route_generation=%" PRIu64
          " status=physical_trajectory_invalidated "
          "action=hold_resident_owner_until_certified_successor",
          result.source_snapshot->version, active_route.identity.generation);
    } else {
      if (observed_route && !observed_3d_world) {
        return result;
      }
      RouteExecutionObservation3D observation = makeExecutionObservation(
          world, objective, execution_navigation, minimum_tracking_sample_sequence,
          execution_maximum_cross_track_m, footprint, flight_envelope_config_);
      std::shared_ptr<const VersionedObservedRawWorld3D> latest_observed_owner;
      std::shared_ptr<const VersionedObservedRawWorld3D> collision_observed_owner;
      bool active_trajectory_raw_collision{false};
      bool active_trajectory_latest_lidar_collision{false};
      mppi::FiniteExecutionPathValidation active_trajectory_raw_validation;
      mppi::FiniteExecutionPathValidation active_trajectory_lidar_validation;
      if (observed_route) {
        latest_observed_owner =
            deriveLatestObservedRouteEvidence(latest_raw_world, active_route);
        if (latest_observed_owner != nullptr &&
            active_source_snapshot->phase == ExecutionRoutePhase3D::kFollowing &&
            active_source_snapshot->finite_execution.has_value()) {
          active_trajectory_raw_validation =
              validateRemainingFiniteExecutionAgainstObservedWorld3D(
                  *active_source_snapshot->finite_execution, *execution_input,
                  *latest_observed_owner, execution_input->effectiveStampNs());
          active_trajectory_raw_collision =
              active_trajectory_raw_validation.status ==
              mppi::FiniteExecutionPathStatus::kRawCollision;
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
          active_source_snapshot->phase == ExecutionRoutePhase3D::kFollowing &&
          active_source_snapshot->finite_execution.has_value()) {
        active_trajectory_lidar_validation =
            validateRemainingFiniteExecutionAgainstLatestLidar3D(
                *active_source_snapshot->finite_execution, *execution_input,
                *latest_lidar_evidence, validation_stamp_ns);
        active_trajectory_latest_lidar_collision =
            active_trajectory_lidar_validation.status ==
            mppi::FiniteExecutionPathStatus::kLatestLidarRawCollision;
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
      const bool persistent_raw_collision =
          obstacle_disposition == ProductionMppiResidentObstacleDisposition::
                                      kPersistentRawFiniteExecutionInvalidated;
      if (persistent_raw_collision) {
        collision_observed_owner = latest_observed_owner != nullptr
                                       ? latest_observed_owner
                                       : active_route.observed_raw_world;
      }
      if (route_suffix_replacement_required) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ROUTE_EXECUTION3D snapshot_version=%" PRIu64 " route_generation=%" PRIu64
            " status=route_suffix_obstructed scope=persistent_raw_route_suffix "
            "failure_segment=%zu failure_point=(%.3f,%.3f,%.3f) "
            "action=retain_finite_execution_and_request_background_successor",
            result.source_snapshot->version, active_route.identity.generation,
            diagnostic_assessment.raw_validation.failure_route_segment,
            diagnostic_assessment.raw_validation.failure_point.x,
            diagnostic_assessment.raw_validation.failure_point.y,
            diagnostic_assessment.raw_validation.failure_point.z);
        requestRouteRelease(RouteReleaseReason3D::kBlocked,
                            active_route.identity.generation);
      } else if (finite_execution_physically_invalidated) {
        const mppi::FiniteExecutionPathValidation& physical_validation =
            obstacle_disposition == ProductionMppiResidentObstacleDisposition::
                                        kPersistentRawFiniteExecutionInvalidated
                ? active_trajectory_raw_validation
                : active_trajectory_lidar_validation;
        diagnostic_assessment.status = RouteExecutionStatus3D::kRawCollision;
        diagnostic_assessment.raw_validation.status =
            RawRouteSuffixStatus3D::kRawCollision;
        diagnostic_assessment.raw_validation.failure_point =
            physical_validation.failure_point;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ROUTE_EXECUTION3D snapshot_version=%" PRIu64 " route_generation=%" PRIu64
            " status=raw_collision scope=%s "
            "failure_segment=%zu failure_point=(%.3f,%.3f,%.3f)",
            result.source_snapshot->version, active_route.identity.generation,
            obstacle_disposition == ProductionMppiResidentObstacleDisposition::
                                        kPersistentRawFiniteExecutionInvalidated
                ? "persistent_raw_finite_trajectory"
                : "latest_lidar_finite_trajectory",
            physical_validation.failure_segment_index,
            physical_validation.failure_point.x, physical_validation.failure_point.y,
            physical_validation.failure_point.z);
      }
      if ((diagnostic_assessment.usable() || route_suffix_replacement_required) &&
          optional_constraints_.route_tracking_tube_constraints_enabled) {
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
          (optional_constraints_.route_cross_track_constraints_enabled &&
           diagnostic_assessment.status ==
               RouteExecutionStatus3D::kExcessiveCrossTrack) ||
          (optional_constraints_.route_tracking_tube_constraints_enabled &&
           diagnostic_assessment.status ==
               RouteExecutionStatus3D::kTrackingTubeViolation);
      const bool route_invalidation_required =
          finite_execution_physically_invalidated ||
          diagnostic_assessment.status == RouteExecutionStatus3D::kObjectiveMismatch ||
          optional_policy_invalidation;
      if (result.tracking_error_tube_handoff_active) {
        active_usable = true;
        result.status = RouteExecutionStatus3D::kUsable;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ROUTE_EXECUTION3D snapshot_version=%" PRIu64 " route_generation=%" PRIu64
            " status=usable tracking_tube_handoff=%.*s "
            "handoff_reference_state_index=%zu "
            "handoff_speed_limit_mps=%.3f handoff_radius_m=%.3f "
            "handoff_tracking_error_m=%.3f action=retain_immutable_handoff",
            result.source_snapshot->version, active_route.identity.generation,
            static_cast<int>(trackingErrorTubeHandoffStatus3DName(
                                 result.tracking_error_tube_handoff.status)
                                 .size()),
            trackingErrorTubeHandoffStatus3DName(
                result.tracking_error_tube_handoff.status)
                .data(),
            result.tracking_error_tube_handoff.reference_state_index,
            result.tracking_error_tube_handoff.reference_speed_limit_mps,
            result.tracking_error_tube_handoff.tracking_error_radius_m,
            result.tracking_error_tube_handoff.actual_tracking_error_m);
      } else if (!route_execution_assessment_usable && !route_invalidation_required) {
        // Contract/projection diagnostics and connector history do not describe
        // a physical intersection of the published finite execution. Keep the
        // certified owner and retry.
        active_usable = true;
        result.status = RouteExecutionStatus3D::kUsable;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ROUTE_EXECUTION3D snapshot_version=%" PRIu64 " route_generation=%" PRIu64
            " diagnostic_status=%.*s "
            "action=retain_certified_route_nonphysical_diagnostic",
            result.source_snapshot->version, active_route.identity.generation,
            static_cast<int>(
                routeExecutionStatus3DName(diagnostic_assessment.status).size()),
            routeExecutionStatus3DName(diagnostic_assessment.status).data());
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
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "ROUTE_EXECUTION3D snapshot_version=%" PRIu64 " route_generation=%" PRIu64
              " status=usable transition=%.*s "
              "action=retain_certified_owner_and_retry_progress",
              result.source_snapshot->version, active_route.identity.generation,
              static_cast<int>(
                  executionRouteTransitionStatus3DName(advanced.status).size()),
              executionRouteTransitionStatus3DName(advanced.status).data());
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
            handlePhysicalTrajectoryCollision(
                generation, raw_invalidated ? collision_observed_owner : nullptr,
                residentObstacleSource(obstacle_disposition),
                ProductionMppiPhysicalTrajectoryAuthority::kResidentOwner);
          } else {
            requestRouteRelease(release_reason, generation);
          }
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "ROUTE_EXECUTION3D snapshot_version=%" PRIu64 " route_generation=%" PRIu64
              " status=%.*s transition=%.*s tracking_tube_status=%.*s "
              "tracking_tube_handoff_status=%.*s "
              "tracking_tube_speed_limit_mps=%.3f "
              "tracking_tube_radius_m=%.3f actual_speed_mps=%.3f "
              "actual_cross_track_m=%.3f "
              "action=retain_certified_owner_and_request_successor",
              result.source_snapshot->version, generation,
              static_cast<int>(routeExecutionStatus3DName(result.status).size()),
              routeExecutionStatus3DName(result.status).data(),
              static_cast<int>(
                  executionRouteTransitionStatus3DName(advanced.status).size()),
              executionRouteTransitionStatus3DName(advanced.status).data(),
              static_cast<int>(trackingErrorTubeExecutionStatus3DName(
                                   result.tracking_error_tube.status)
                                   .size()),
              trackingErrorTubeExecutionStatus3DName(result.tracking_error_tube.status)
                  .data(),
              static_cast<int>(trackingErrorTubeHandoffStatus3DName(
                                   result.tracking_error_tube_handoff.status)
                                   .size()),
              trackingErrorTubeHandoffStatus3DName(
                  result.tracking_error_tube_handoff.status)
                  .data(),
              result.tracking_error_tube.speed_limit_mps,
              result.tracking_error_tube.tube_radius_m,
              routeSpeed3D(Vec3{execution_navigation.state.vx,
                                execution_navigation.state.vy,
                                execution_navigation.state.vz}),
              diagnostic_assessment.projection.distance_m);
        }
      } else {
        // This transition intentionally remains caller-local. Publishing it would
        // expose progress and route-certificate generations without the finite
        // command horizon and its immediate braking fallback. The planning stage
        // certifies both against this exact snapshot and composes the two
        // transitions into one store CAS rooted at source_snapshot.
        result.progress_preparation =
            std::make_shared<const ExecutionRouteTransitionResult3D>(advanced);
        result.certification_snapshot = advanced.next;
        active_usable = true;
      }
    }
  }

  const std::shared_ptr<const ExecutionRouteSnapshot3D>& route_state =
      result.certification_snapshot != nullptr ? result.certification_snapshot
                                               : result.source_snapshot;
  result.pending_route = pending_certified_route_mailbox_.snapshot();
  if (result.pending_route != nullptr &&
      !pendingCertifiedRouteEligible3D(*result.pending_route, *route_state) &&
      pendingRoutePermanentlyObsolete(*result.pending_route, *route_state)) {
    if (pending_certified_route_mailbox_.acknowledgeIfSame(result.pending_route)) {
      result.pending_route.reset();
    } else {
      // A newer publication defeated the exact acknowledgement. Preserve that
      // resident identity so this tick may assess it and recovery cannot
      // mistake the caller-local stale pointer for an empty mailbox.
      result.pending_route = pending_certified_route_mailbox_.snapshot();
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
                                  route_tracking_policy_.maximum_cross_track_m,
                                  footprint, flight_envelope_config_);
    if (retain_snapshot_certificate) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ROUTE_HANDOFF3D pending_generation=%" PRIu64 " base_generation=%" PRIu64
          " status=snapshot_certificate_retained "
          "action=validate_finite_horizons_against_latest_physical_evidence",
          result.pending_route->route.identity.generation,
          result.pending_route->base_route_generation);
    }
    RouteSpliceReadiness3D splice_readiness{.status =
                                                RouteSpliceReadinessStatus3D::kReady};
    const bool route_splice_required =
        result.pending_route->base_kind == PendingExecutionBaseKind3D::kRoute;
    if (route_splice_required) {
      if (!result.pending_route->route_splice.has_value() ||
          !route_state->route.has_value()) {
        splice_readiness.status = RouteSpliceReadinessStatus3D::kInvalidProof;
      } else if (refreshed_pending == nullptr) {
        splice_readiness.status =
            RouteSpliceReadinessStatus3D::kSuccessorProjectionUnavailable;
      } else {
        splice_readiness = assessRouteSpliceReadiness3D(
            *result.pending_route->route_splice, *route_state->route,
            *refreshed_pending,
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
          result.pending_route->route_splice.has_value() &&
          route_state->route.has_value() &&
          routeSpliceWindowExpired3D(*result.pending_route->route_splice,
                                     *route_state->route);
      const bool permanently_unavailable =
          splice_expired || !splice_readiness.canStillBecomeReady();
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ROUTE_SPLICE3D pending_generation=%" PRIu64 " base_generation=%" PRIu64
          " status=%.*s expired=%s "
          "base_station_m=%.2f successor_station_m=%.2f "
          "position_separation_m=%.2f tangent_alignment=%.3f action=%s",
          result.pending_route->route.identity.generation,
          result.pending_route->base_route_generation,
          static_cast<int>(
              routeSpliceReadinessStatus3DName(splice_readiness.status).size()),
          routeSpliceReadinessStatus3DName(splice_readiness.status).data(),
          splice_expired ? "true" : "false", splice_readiness.base_station_m,
          splice_readiness.successor_station_m, splice_readiness.position_separation_m,
          splice_readiness.tangent_alignment,
          permanently_unavailable ? "discard_and_replan" : "retain_active_route");
      if (permanently_unavailable &&
          pending_certified_route_mailbox_.acknowledgeIfSame(result.pending_route)) {
        result.pending_route.reset();
      }
    } else if (snapshot_retention_authorized) {
      const std::uint64_t pending_generation =
          result.pending_route->route.identity.generation;
      const std::uint64_t base_generation = result.pending_route->base_route_generation;
      const bool acknowledged =
          pending_certified_route_mailbox_.acknowledgeIfSame(result.pending_route);
      if (acknowledged) {
        result.pending_route.reset();
      } else {
        result.pending_route = pending_certified_route_mailbox_.snapshot();
      }
      RCLCPP_INFO(get_logger(),
                  "ROUTE_HANDOFF3D pending_generation=%" PRIu64
                  " base_generation=%" PRIu64
                  " status=semantic_refresh_rejected acknowledged=%s "
                  "action=release_for_current_objective_successor",
                  pending_generation, base_generation, acknowledged ? "true" : "false");
    }
  }

  if (result.route == nullptr && active_usable &&
      (route_state->phase == ExecutionRoutePhase3D::kFollowing ||
       route_state->phase == ExecutionRoutePhase3D::kAwaitingSuccessor) &&
      route_state->route.has_value()) {
    result.route = std::make_shared<const CertifiedRouteSuffix3D>(*route_state->route);
    result.route_usable = true;
    result.status = RouteExecutionStatus3D::kUsable;
  }
  if (result.route != nullptr) {
    result.projection = routeProjection(
        *result.route, Point3{execution_input->state().x, execution_input->state().y,
                              execution_input->state().z});
    result.station_m = result.route->progress.station_m;
    result.route_usable =
        result.route_usable && result.projection.valid &&
        (!optional_constraints_.route_cross_track_constraints_enabled ||
         result.tracking_error_tube_handoff_active ||
         result.projection.cross_track_m <=
             route_tracking_policy_.maximum_cross_track_m);
    if (!result.route_usable) {
      result.status = result.projection.valid
                          ? RouteExecutionStatus3D::kExcessiveCrossTrack
                          : RouteExecutionStatus3D::kInvalidProjection;
      result.route.reset();
    }
  }
  return result;
}

} // namespace drone_city_nav
