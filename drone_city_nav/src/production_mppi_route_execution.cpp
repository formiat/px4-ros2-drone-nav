#include <algorithm>
#include <cinttypes>
#include <memory>
#include <variant>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

// Raw occupancy may advance while a suffix is being checked. Bound optimistic
// retries so route progress cannot monopolize a planning tick; exhaustion falls
// through to the exact-evidence emergency-brake path.
constexpr std::size_t kMaximumRawProgressPublicationAttempts{3U};

[[nodiscard]] SweptFootprintConfig
executionFootprint(const RiskAwareLattice3DConfig& lattice_config,
                   const SweptFootprintConfig& physical_config) noexcept {
  return SweptFootprintConfig{
      .radius_m = lattice_config.physical_footprint_radius_m,
      .lower_extent_m = lattice_config.physical_footprint_lower_extent_m,
      .upper_extent_m = lattice_config.physical_footprint_upper_extent_m,
      .perimeter_samples = physical_config.perimeter_samples,
      .radial_rings = physical_config.radial_rings,
      .axial_samples = physical_config.axial_samples,
      .sweep_step_m = physical_config.sweep_step_m,
  };
}

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

[[nodiscard]] bool observedRouteEvidenceIsCurrent(
    const std::shared_ptr<const VersionedObservedRawWorld3D>& route_evidence,
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world) noexcept {
  return route_evidence != nullptr && route_evidence->valid() &&
         latest_raw_world != nullptr &&
         rawWorldExecutionOwnerExact(*latest_raw_world) &&
         sameRawMapVersion(route_evidence->version(), latest_raw_world->version) &&
         std::addressof(route_evidence->occupancy()) ==
             latest_raw_world->occupancy.get() &&
         latest_raw_world->execution_owner->sharesObservationOwner(*route_evidence) &&
         route_evidence->occupiedSnapshot() ==
             latest_raw_world->execution_owner->occupiedSnapshot();
}

[[nodiscard]] bool
sameExecutionStateProvenance(const ExecutionStateProvenance3D& first,
                             const ExecutionStateProvenance3D& second) noexcept {
  return first.x == second.x && first.y == second.y && first.z == second.z &&
         first.vx == second.vx && first.vy == second.vy && first.vz == second.vz &&
         first.yaw == second.yaw && first.yaw_rate == second.yaw_rate;
}

[[nodiscard]] bool sameExecutionState(const mppi::State& first,
                                      const mppi::State& second) noexcept {
  return first.x == second.x && first.y == second.y && first.z == second.z &&
         first.vx == second.vx && first.vy == second.vy && first.vz == second.vz &&
         first.yaw == second.yaw && first.yaw_rate == second.yaw_rate;
}

[[nodiscard]] bool sameExecutionControl(const mppi::Control& first,
                                        const mppi::Control& second) noexcept {
  return first.ax == second.ax && first.ay == second.ay && first.az == second.az &&
         first.yaw_accel == second.yaw_accel;
}

[[nodiscard]] bool
executionStateUpdateAllowed(const VersionedExecutionInput3D& candidate,
                            const VersionedExecutionInput3D& previous) noexcept {
  if (candidate.fullStateAuthoritative() != previous.fullStateAuthoritative() ||
      !sameExecutionStateProvenance(candidate.stateProvenance(),
                                    previous.stateProvenance())) {
    return false;
  }
  if (candidate.effectiveStampNs() == previous.effectiveStampNs()) {
    return sameExecutionState(candidate.state(), previous.state());
  }
  const mppi::State& next = candidate.state();
  const mppi::State& old = previous.state();
  const ExecutionStateProvenance3D& provenance = candidate.stateProvenance();
  const auto field_update_allowed = [](const float next_value, const float old_value,
                                       const ExecutionStateFieldProvenance3D source) {
    return source == ExecutionStateFieldProvenance3D::kEffectiveTimePrediction ||
           next_value == old_value;
  };
  return field_update_allowed(next.x, old.x, provenance.x) &&
         field_update_allowed(next.y, old.y, provenance.y) &&
         field_update_allowed(next.z, old.z, provenance.z) &&
         field_update_allowed(next.vx, old.vx, provenance.vx) &&
         field_update_allowed(next.vy, old.vy, provenance.vy) &&
         field_update_allowed(next.vz, old.vz, provenance.vz) &&
         field_update_allowed(next.yaw, old.yaw, provenance.yaw) &&
         field_update_allowed(next.yaw_rate, old.yaw_rate, provenance.yaw_rate);
}

[[nodiscard]] bool
executionInputNotOlder(const VersionedExecutionInput3D& candidate,
                       const VersionedExecutionInput3D& previous) noexcept {
  if (!candidate.valid() || !previous.valid() ||
      candidate.captureSequence() < previous.captureSequence() ||
      candidate.poseRevision() < previous.poseRevision() ||
      candidate.poseSourceTimestampUs() < previous.poseSourceTimestampUs() ||
      candidate.poseReceiveStampNs() < previous.poseReceiveStampNs() ||
      candidate.effectiveStampNs() < previous.effectiveStampNs() ||
      candidate.previousControlSourceStampNs() <
          previous.previousControlSourceStampNs() ||
      candidate.previousControlReceiveStampNs() <
          previous.previousControlReceiveStampNs()) {
    return false;
  }
  if (candidate.captureSequence() == previous.captureSequence()) {
    return candidate.contentFingerprint() == previous.contentFingerprint();
  }
  if (candidate.poseRevision() == previous.poseRevision()) {
    if (candidate.poseSourceTimestampUs() != previous.poseSourceTimestampUs() ||
        candidate.poseReceiveStampNs() != previous.poseReceiveStampNs() ||
        !executionStateUpdateAllowed(candidate, previous)) {
      return false;
    }
  } else if (candidate.poseSourceTimestampUs() <= previous.poseSourceTimestampUs() ||
             candidate.poseReceiveStampNs() <= previous.poseReceiveStampNs()) {
    return false;
  }
  if (candidate.previousControlSource() == previous.previousControlSource()) {
    if (candidate.previousControlSourceSequence() <
        previous.previousControlSourceSequence()) {
      return false;
    }
    if (candidate.previousControlSourceSequence() ==
        previous.previousControlSourceSequence()) {
      if (candidate.previousControlSourceStampNs() ==
          previous.previousControlSourceStampNs()) {
        return candidate.previousControlReceiveStampNs() ==
                   previous.previousControlReceiveStampNs() &&
               sameExecutionControl(candidate.previousControl(),
                                    previous.previousControl());
      }
      if (candidate.previousControlSource() !=
              ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback ||
          candidate.previousControlSourceStampNs() <=
              previous.previousControlSourceStampNs() ||
          candidate.previousControlReceiveStampNs() <=
              previous.previousControlReceiveStampNs()) {
        return false;
      }
    } else if (candidate.previousControlSourceStampNs() <=
                   previous.previousControlSourceStampNs() ||
               candidate.previousControlReceiveStampNs() <=
                   previous.previousControlReceiveStampNs()) {
      return false;
    }
  } else if (candidate.previousControlSourceStampNs() <=
                 previous.previousControlSourceStampNs() ||
             candidate.previousControlReceiveStampNs() <=
                 previous.previousControlReceiveStampNs()) {
    return false;
  }
  return true;
}

[[nodiscard]] bool
sameRouteContinuityLineage(const RouteContinuityLineage3D& first,
                           const RouteContinuityLineage3D& second) noexcept {
  return first.mission_epoch == second.mission_epoch &&
         first.assignment_generation == second.assignment_generation &&
         first.target_detection_id == second.target_detection_id &&
         first.target_track_id == second.target_track_id;
}

[[nodiscard]] bool
sameObservedCertificateLineage(const ObservedRawRouteCertificate3D& candidate,
                               const ObservedRawRouteCertificate3D& expected) noexcept {
  return candidate.route_generation == expected.route_generation &&
         candidate.geometry_revision == expected.geometry_revision &&
         candidate.physical_route_fingerprint == expected.physical_route_fingerprint &&
         candidate.producer_instance_id == expected.producer_instance_id &&
         candidate.validation_policy_fingerprint ==
             expected.validation_policy_fingerprint &&
         candidate.execution_validation_policy_fingerprint ==
             expected.execution_validation_policy_fingerprint &&
         candidate.passage_geometry_revision == expected.passage_geometry_revision &&
         candidate.passage_volume_config_fingerprint ==
             expected.passage_volume_config_fingerprint &&
         candidate.passage_derivation_occupancy_content_fingerprint ==
             expected.passage_derivation_occupancy_content_fingerprint;
}

[[nodiscard]] bool
sameRouteEvidenceLineage(const CertifiedRouteSuffix3D& candidate,
                         const CertifiedRouteSuffix3D& expected) noexcept {
  if (!candidate.valid() || !expected.valid() ||
      candidate.identity.generation != expected.identity.generation ||
      candidate.geometry->executable_geometry_revision !=
          expected.geometry->executable_geometry_revision ||
      candidate.geometry->physical_route_fingerprint !=
          expected.geometry->physical_route_fingerprint ||
      candidate.continuity_id != expected.continuity_id ||
      !sameRouteContinuityLineage(candidate.continuity_lineage,
                                  expected.continuity_lineage) ||
      candidate.validation_policy->policyId() !=
          expected.validation_policy->policyId() ||
      candidate.planned_endpoint_semantics != expected.planned_endpoint_semantics ||
      candidate.certificate.index() != expected.certificate.index()) {
    return false;
  }
  if (expected.observed_raw_world != nullptr) {
    const auto* const candidate_certificate =
        std::get_if<ObservedRawRouteCertificate3D>(&candidate.certificate);
    const auto* const expected_certificate =
        std::get_if<ObservedRawRouteCertificate3D>(&expected.certificate);
    return candidate_certificate != nullptr && expected_certificate != nullptr &&
           candidate.observed_raw_world != nullptr &&
           candidate.static_world == nullptr && expected.static_world == nullptr &&
           sameObservedCertificateLineage(*candidate_certificate,
                                          *expected_certificate) &&
           candidate_certificate->validated_through_revision >=
               expected_certificate->validated_through_revision &&
           sameRawMapVersion(candidate.observed_raw_world->version(),
                             expected.observed_raw_world->version()) &&
           candidate.observed_raw_world->contentFingerprint() ==
               expected.observed_raw_world->contentFingerprint() &&
           candidate.observed_raw_world->sharesObservationOwner(
               *expected.observed_raw_world);
  }
  return candidate.observed_raw_world == nullptr &&
         expected.observed_raw_world == nullptr && candidate.static_world != nullptr &&
         candidate.static_world == expected.static_world;
}

[[nodiscard]] bool newerResidentCanSupersedeLostPublication(
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& expected_snapshot,
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& attempted_snapshot,
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& resident_snapshot,
    const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& publication_raw_evidence,
    const std::shared_ptr<const ProductionMppiRawWorld3D>& current_raw_world) noexcept {
  if (expected_snapshot == nullptr || attempted_snapshot == nullptr ||
      resident_snapshot == nullptr || execution_input == nullptr ||
      !expected_snapshot->valid() || !attempted_snapshot->valid() ||
      !resident_snapshot->valid() ||
      resident_snapshot->version <= expected_snapshot->version ||
      resident_snapshot->phase != ExecutionRoutePhase3D::kFollowing ||
      !attempted_snapshot->route.has_value() || !resident_snapshot->route.has_value() ||
      !sameRouteEvidenceLineage(*resident_snapshot->route,
                                *attempted_snapshot->route) ||
      resident_snapshot->route->progress.execution_input == nullptr ||
      !executionInputNotOlder(*resident_snapshot->route->progress.execution_input,
                              *execution_input)) {
    return false;
  }
  if (attempted_snapshot->route->observed_raw_world == nullptr) {
    return publication_raw_evidence == nullptr;
  }
  return observedRouteEvidenceIsCurrent(publication_raw_evidence, current_raw_world) &&
         sameRawMapVersion(resident_snapshot->route->observed_raw_world->version(),
                           publication_raw_evidence->version()) &&
         resident_snapshot->route->observed_raw_world->sharesObservationOwner(
             *publication_raw_evidence);
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
      .pending_route = nullptr,
      .lifecycle_observed_raw_world = nullptr,
      .projection = {},
      .status = RouteExecutionStatus3D::kNoActiveRoute,
      .lifecycle_event = std::nullopt,
      .hold_position =
          Point3{navigation.state.x, navigation.state.y,
                 clampToFlightEnvelope(navigation.state.z, flight_envelope_config_)
                     .value_or(flight_envelope_config_.minimum_target_z_m)},
      .station_m = 0.0,
      .route_usable = false,
      .execution_owner_available = false,
      .pending_activation = false,
      .direct_tracking_identity = std::move(direct_tracking_identity),
  };
  result.source_snapshot = execution_route_store_.snapshot();
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
    if (pending_certified_route_mailbox_.acknowledgeIfSame(stale_pending)) {
      recordPendingRouteStrategyOutcome(stale_pending, false);
    }
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

  const SweptFootprintConfig footprint =
      executionFootprint(lattice_3d_config_, physical_footprint_config_);
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
    std::shared_ptr<const ProductionMppiRawWorld3D> assessment_raw_world =
        latest_raw_world;
    const std::size_t maximum_attempts =
        observed_route ? kMaximumRawProgressPublicationAttempts : 1U;
    for (std::size_t attempt_index = 0U; attempt_index < maximum_attempts;
         ++attempt_index) {
      RouteExecutionObservation3D observation = makeExecutionObservation(
          world, objective, execution_navigation, minimum_tracking_sample_sequence,
          active_guide_config_.maximum_cross_track_m, footprint);
      std::shared_ptr<const VersionedObservedRawWorld3D> observed_owner;
      if (observed_route) {
        observed_owner =
            deriveLatestObservedRouteEvidence(assessment_raw_world, active_route);
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
      const RouteExecutionAssessment3D diagnostic_assessment = assessRouteExecution3D(
          &active_route.identity, *active_route.geometry->route, observation);
      const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
          *result.source_snapshot, guard, observation, execution_input, observed_owner);
      if (!advanced.applied()) {
        if (advanced.status == ExecutionRouteTransitionStatus3D::kNoChange) {
          active_usable = diagnostic_assessment.usable();
        } else {
          result.status = diagnostic_assessment.status;
          if (result.status == RouteExecutionStatus3D::kUsable) {
            result.status = RouteExecutionStatus3D::kRawCollision;
          }
          const std::uint64_t generation = active_route.identity.generation;
          const bool raw_invalidated =
              result.status != RouteExecutionStatus3D::kObjectiveMismatch &&
              result.status != RouteExecutionStatus3D::kExcessiveCrossTrack;
          if (raw_invalidated) {
            result.lifecycle_observed_raw_world = observed_owner;
          }
          RouteLifecycleEventKind3D event_kind =
              RouteLifecycleEventKind3D::kRawInvalidated;
          GlobalGuideReleaseReason release_reason = GlobalGuideReleaseReason::kBlocked;
          if (result.status == RouteExecutionStatus3D::kObjectiveMismatch) {
            event_kind = RouteLifecycleEventKind3D::kObjectiveSuperseded;
            release_reason = GlobalGuideReleaseReason::kObjectiveChanged;
          } else if (result.status == RouteExecutionStatus3D::kExcessiveCrossTrack) {
            event_kind = RouteLifecycleEventKind3D::kCrossTrackExceeded;
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
              " status=%.*s transition=%.*s "
              "action=retain_certified_owner_and_request_successor",
              result.source_snapshot->version, generation,
              static_cast<int>(routeExecutionStatus3DName(result.status).size()),
              routeExecutionStatus3DName(result.status).data(),
              static_cast<int>(
                  executionRouteTransitionStatus3DName(advanced.status).size()),
              executionRouteTransitionStatus3DName(advanced.status).data());
        }
        break;
      }
      ExecutionRoutePublicationStatus3D publication_status{
          ExecutionRoutePublicationStatus3D::kInvalidCandidate};
      std::shared_ptr<const ProductionMppiRawWorld3D> publication_raw_world;
      std::shared_ptr<const ExecutionRouteSnapshot3D> resident_after_cas_loss;
      bool publication_raw_current{active_route.observed_raw_world == nullptr};
      bool resident_accepted{false};
      {
        const std::scoped_lock lock{execution_evidence_commit_mutex_};
        if (active_route.observed_raw_world != nullptr) {
          publication_raw_world = latest_raw_world_3d_.load(std::memory_order_acquire);
          publication_raw_current =
              observedRouteEvidenceIsCurrent(observed_owner, publication_raw_world);
        }
        if (publication_raw_current) {
          publication_status =
              execution_route_store_.publish(result.source_snapshot, advanced);
          if (publication_status ==
              ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion) {
            resident_after_cas_loss = execution_route_store_.snapshot();
            resident_accepted = newerResidentCanSupersedeLostPublication(
                result.source_snapshot, advanced.next, resident_after_cas_loss,
                execution_input, observed_owner, publication_raw_world);
          }
        }
      }
      if (!publication_raw_current && observed_route &&
          attempt_index + 1U < maximum_attempts && publication_raw_world != nullptr &&
          publication_raw_world != assessment_raw_world) {
        assessment_raw_world = std::move(publication_raw_world);
        continue;
      }
      if (publication_status == ExecutionRoutePublicationStatus3D::kPublished) {
        result.source_snapshot = advanced.next;
        active_usable = true;
      } else if (resident_accepted) {
        result.source_snapshot = std::move(resident_after_cas_loss);
        result.execution_owner_available =
            result.source_snapshot->finite_execution.has_value() ||
            result.source_snapshot->direct_tracking_execution.has_value() ||
            result.source_snapshot->stationary_hold.has_value();
        active_usable = true;
      } else {
        if (resident_after_cas_loss != nullptr) {
          result.source_snapshot = std::move(resident_after_cas_loss);
          result.execution_owner_available =
              result.source_snapshot->finite_execution.has_value() ||
              result.source_snapshot->direct_tracking_execution.has_value() ||
              result.source_snapshot->stationary_hold.has_value();
        }
        result.status = publication_raw_current
                            ? RouteExecutionStatus3D::kInvalidRoute
                            : RouteExecutionStatus3D::kWorldLineageMismatch;
        const std::uint64_t generation = active_route.identity.generation;
        if (!publication_raw_current) {
          result.lifecycle_observed_raw_world =
              deriveLatestObservedRouteEvidence(publication_raw_world, active_route);
        }
        const RouteLifecycleEventKind3D event_kind =
            publication_raw_current
                ? RouteLifecycleEventKind3D::kControlCandidateRejected
                : RouteLifecycleEventKind3D::kRawInvalidated;
        result.lifecycle_event = RouteLifecycleEvent3D{
            .kind = event_kind,
            .generation = generation,
            .raw_producer_instance_id =
                !publication_raw_current && publication_raw_world != nullptr
                    ? publication_raw_world->version.producer_instance_id
                    : 0U,
            .raw_revision = !publication_raw_current && publication_raw_world != nullptr
                                ? publication_raw_world->version.revision
                                : 0U,
        };
        requestGuideRelease(GlobalGuideReleaseReason::kBlocked, generation);
        const char* const publication_reason =
            !publication_raw_current ? "raw_evidence_not_current"
            : publication_status ==
                    ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion
                ? "resident_not_compatible"
                : "invalid_publication_candidate";
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ROUTE_EXECUTION3D snapshot_version=%" PRIu64 " route_generation=%" PRIu64
            " status=%.*s publication=%s "
            "action=fail_closed_and_request_successor",
            result.source_snapshot->version, generation,
            static_cast<int>(routeExecutionStatus3DName(result.status).size()),
            routeExecutionStatus3DName(result.status).data(), publication_reason);
      }
      break;
    }
  }

  result.pending_route = pending_certified_route_mailbox_.snapshot();
  if (result.pending_route != nullptr &&
      !pendingCertifiedRouteEligible3D(*result.pending_route,
                                       *result.source_snapshot) &&
      pendingRoutePermanentlyObsolete(*result.pending_route, *result.source_snapshot)) {
    if (pending_certified_route_mailbox_.acknowledgeIfSame(result.pending_route)) {
      recordPendingRouteStrategyOutcome(result.pending_route, false);
      result.pending_route.reset();
    } else {
      // A newer publication defeated the exact acknowledgement. Preserve that
      // resident identity so this tick may assess it and recovery cannot
      // mistake the caller-local stale pointer for an empty mailbox.
      result.pending_route = pending_certified_route_mailbox_.snapshot();
    }
  }
  if (result.pending_route != nullptr &&
      pendingCertifiedRouteEligible3D(*result.pending_route, *result.source_snapshot)) {
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
          !result.source_snapshot->route.has_value()) {
        splice_readiness.status = RouteSpliceReadinessStatus3D::kInvalidProof;
      } else if (refreshed_pending == nullptr) {
        splice_readiness.status =
            RouteSpliceReadinessStatus3D::kSuccessorProjectionUnavailable;
      } else {
        splice_readiness = assessRouteSpliceReadiness3D(
            *result.pending_route->route_splice, *result.source_snapshot->route,
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
          result.source_snapshot->route.has_value() &&
          routeSpliceWindowExpired3D(*result.pending_route->route_splice,
                                     *result.source_snapshot->route);
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
        recordPendingRouteStrategyOutcome(result.pending_route, false);
        result.pending_route.reset();
      }
    }
  }

  if (result.route == nullptr && active_usable &&
      result.source_snapshot->phase == ExecutionRoutePhase3D::kFollowing &&
      result.source_snapshot->route.has_value()) {
    result.route =
        std::make_shared<const CertifiedRouteSuffix3D>(*result.source_snapshot->route);
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
        result.projection.cross_track_m <= active_guide_config_.maximum_cross_track_m;
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
