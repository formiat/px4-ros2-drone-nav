#include <algorithm>
#include <cinttypes>
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
        .free_space_seed = observed_world->proprioceptiveFreeSpaceSeed().has_value()
                               ? &*observed_world->proprioceptiveFreeSpaceSeed()
                               : nullptr,
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
        .occupancy_policy = TrackingErrorTubeOccupancyPolicy3D::kKnownStaticBounds,
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

[[nodiscard]] RouteExecutionObservation3D
makeExecutionObservation(const ProductionMppiPreparedEsdf& world,
                         const ProductionNavigationObjective* const objective,
                         const ProductionMppiNavigation& navigation,
                         const std::uint64_t minimum_tracking_sample_sequence,
                         const double maximum_cross_track_m,
                         const SweptFootprintConfig& footprint) {
  return RouteExecutionObservation3D{
      .current_objective = objective != nullptr ? makeStaticRouteObjective(*objective)
                                                : StaticRouteObjective{},
      .minimum_tracking_sample_sequence = minimum_tracking_sample_sequence,
      .position = {navigation.state.x, navigation.state.y, navigation.state.z},
      .maximum_cross_track_m = maximum_cross_track_m,
      .footprint = footprint,
      .proprioceptive_free_space_seed =
          world.proprioceptive_free_space_seed
              ? std::addressof(*world.proprioceptive_free_space_seed)
              : nullptr,
      .launch_support_contact = world.launch_support_contact
                                    ? std::addressof(*world.launch_support_contact)
                                    : nullptr,
  };
}

[[nodiscard]] RouteActivationObservation3D
makeActivationObservation(const ProductionMppiPreparedEsdf& world,
                          const ProductionNavigationObjective* const objective,
                          const ProductionMppiNavigation& navigation,
                          const std::uint64_t minimum_tracking_sample_sequence,
                          const double maximum_cross_track_m,
                          const SweptFootprintConfig& footprint,
                          const bool raw_validation_required) {
  return RouteActivationObservation3D{
      .resident_world = navigationWorldCertificate3D(world),
      .current_objective = objective != nullptr ? makeStaticRouteObjective(*objective)
                                                : StaticRouteObjective{},
      .minimum_tracking_sample_sequence = minimum_tracking_sample_sequence,
      .position = {navigation.state.x, navigation.state.y, navigation.state.z},
      .maximum_cross_track_m = maximum_cross_track_m,
      .footprint = footprint,
      .raw_validation_required = raw_validation_required,
  };
}

[[nodiscard]] std::shared_ptr<const CertifiedRouteSuffix3D> refreshPendingRoute(
    const PendingCertifiedRoute3D& pending, const ProductionMppiPreparedEsdf& world,
    const ProductionNavigationObjective* const objective,
    const ProductionMppiNavigation& navigation,
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
    const std::uint64_t minimum_tracking_sample_sequence,
    const double maximum_cross_track_m, const SweptFootprintConfig& footprint) {
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
      maximum_cross_track_m, footprint, observed);
  const std::optional<CertifiedRouteSuffix3D> refreshed =
      recertifyExecutionRoute3D(pending.route, observation, std::move(observed_owner));
  return refreshed.has_value()
             ? std::make_shared<const CertifiedRouteSuffix3D>(*refreshed)
             : nullptr;
}

[[nodiscard]] GlobalGuideProjection
routeProjection(const CertifiedRouteSuffix3D& route,
                const Point3& current_position) noexcept {
  GlobalGuideProjection projection;
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
    const ProductionMppiPreparedEsdf& world,
    const ProductionNavigationObjective* const objective,
    const ProductionMppiNavigation& navigation,
    const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
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
  bool active_usable{false};
  if (result.source_snapshot->route.has_value()) {
    const std::shared_ptr<const ExecutionRouteSnapshot3D> active_source_snapshot =
        result.source_snapshot;
    const CertifiedRouteSuffix3D& active_route = *active_source_snapshot->route;
    const ExecutionRouteTransitionGuard3D guard{
        .expected_snapshot_version = result.source_snapshot->version,
        .expected_route_generation = result.source_snapshot->route->identity.generation,
        .expected_geometry_revision =
            result.source_snapshot->route->geometry->executable_geometry_revision,
    };
    const bool observed_route = active_route.observed_raw_world != nullptr;
    if (observed_route && !observed_3d_world) {
      return result;
    }
    {
      RouteExecutionObservation3D observation = makeExecutionObservation(
          world, objective, execution_navigation, minimum_tracking_sample_sequence,
          active_guide_config_.maximum_cross_track_m, footprint);
      std::shared_ptr<const VersionedObservedRawWorld3D> observed_owner;
      if (observed_route) {
        observed_owner =
            deriveLatestObservedRouteEvidence(latest_raw_world, active_route);
        if (observed_owner != nullptr) {
          observation.latest_raw_occupancy = &observed_owner->occupancy();
          observation.latest_raw_producer_instance_id =
              observed_owner->version().producer_instance_id;
          observation.latest_raw_revision = observed_owner->version().revision;
          observation.proprioceptive_free_space_seed =
              observed_owner->proprioceptiveFreeSpaceSeed().has_value()
                  ? std::addressof(*observed_owner->proprioceptiveFreeSpaceSeed())
                  : nullptr;
          observation.launch_support_contact =
              observed_owner->launchSupportContact().has_value()
                  ? std::addressof(*observed_owner->launchSupportContact())
                  : nullptr;
        }
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
      if (diagnostic_assessment.usable()) {
        if (!trackingTubeProfileMatchesCurrentWorld(active_route, observed_owner)) {
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
      const ExecutionRouteTransitionResult3D advanced =
          diagnostic_assessment.usable() && !result.tracking_error_tube_handoff_active
              ? advanceCertifiedRoute3D(*result.source_snapshot, guard, observation,
                                        execution_input, observed_owner)
              : ExecutionRouteTransitionResult3D{};
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
      } else if (!advanced.applied()) {
        if (advanced.status == ExecutionRouteTransitionStatus3D::kNoChange) {
          active_usable = diagnostic_assessment.usable();
        } else if (diagnostic_assessment.usable()) {
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
          const bool raw_invalidated =
              result.status == RouteExecutionStatus3D::kRawCollision;
          if (raw_invalidated) {
            result.lifecycle_observed_raw_world = observed_owner;
          }
          RouteLifecycleEventKind3D event_kind =
              RouteLifecycleEventKind3D::kControlCandidateRejected;
          GlobalGuideReleaseReason release_reason = GlobalGuideReleaseReason::kBlocked;
          if (raw_invalidated) {
            event_kind = RouteLifecycleEventKind3D::kRawInvalidated;
          } else if (result.status == RouteExecutionStatus3D::kObjectiveMismatch) {
            event_kind = RouteLifecycleEventKind3D::kObjectiveSuperseded;
            release_reason = GlobalGuideReleaseReason::kObjectiveChanged;
          } else if (result.status == RouteExecutionStatus3D::kExcessiveCrossTrack) {
            event_kind = RouteLifecycleEventKind3D::kCrossTrackExceeded;
            release_reason = GlobalGuideReleaseReason::kDiverged;
          } else if (result.status == RouteExecutionStatus3D::kTrackingTubeViolation) {
            event_kind = RouteLifecycleEventKind3D::kTrackingTubeExceeded;
            release_reason = GlobalGuideReleaseReason::kDiverged;
          }
          result.lifecycle_event = RouteLifecycleEvent3D{
              .kind = event_kind,
              .generation = generation,
              .raw_producer_instance_id =
                  raw_invalidated && observed_owner != nullptr
                      ? observed_owner->version().producer_instance_id
                      : 0U,
              .raw_revision = raw_invalidated && observed_owner != nullptr
                                  ? observed_owner->version().revision
                                  : 0U,
          };
          requestGuideRelease(release_reason, generation);
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
    std::shared_ptr<const CertifiedRouteSuffix3D> refreshed_pending =
        refreshPendingRoute(*result.pending_route, world, objective,
                            execution_navigation, latest_raw_world,
                            minimum_tracking_sample_sequence,
                            active_guide_config_.maximum_cross_track_m, footprint);
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
    }
  }

  if (result.route == nullptr && active_usable &&
      route_state->phase == ExecutionRoutePhase3D::kFollowing &&
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
        (result.tracking_error_tube_handoff_active ||
         result.projection.cross_track_m <= active_guide_config_.maximum_cross_track_m);
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
