#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_certification_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/producer_instance_id.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <numeric>
#include <ranges>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav {

using namespace execution_route_snapshot_3d_internal;

namespace execution_route_snapshot_3d_internal {

constexpr std::uint64_t kCertifiedRouteInstanceDomain{0x525445494e535433ULL};
constexpr std::uint64_t kRouteOwnerDomain{0x5254454f574e5233ULL};

[[nodiscard]] std::optional<CertifiedRouteSuffix3D>
certifyExecutionRoute3DImpl(const ExecutionRouteActivation3D& activation,
                            const CertifiedRouteSuffix3D* const sealed_source,
                            RouteCertificationStatus3D* const status) {
  const auto rejected = [status](const RouteCertificationStatus3D verdict) {
    if (status != nullptr) {
      *status = verdict;
    }
    return std::optional<CertifiedRouteSuffix3D>{};
  };
  const bool requires_observed_raw_certificate =
      observedRawLineage(activation.proposal.validated_world);
  if (activation.observation.raw_validation_required !=
          requires_observed_raw_certificate ||
      activation.validation_policy == nullptr ||
      !activation.validation_policy->valid()) {
    return rejected(RouteCertificationStatus3D::kInvalidInput);
  }
  if (!footprintConservativelyContains(
          activation.observation.footprint,
          activation.validation_policy->sweptFootprint())) {
    return rejected(RouteCertificationStatus3D::kFootprintNotContained);
  }
  RouteActivationObservation3D owned_observation = activation.observation;
  owned_observation.flight_envelope = activation.validation_policy->flightEnvelope();
  const LaunchSupportContact3D* observed_launch_support{nullptr};
  const ProprioceptiveFreeSpaceSeed3D* observed_proprioceptive_seed{nullptr};
  std::optional<ProprioceptiveFreeSpaceSeed3D> departure_seed;
  if (requires_observed_raw_certificate) {
    if (activation.static_world != nullptr ||
        activation.observed_raw_world == nullptr ||
        !activation.observed_raw_world->valid() ||
        activation.observed_raw_world->version().producer_instance_id !=
            activation.proposal.validated_world.producer_instance_id) {
      return rejected(RouteCertificationStatus3D::kRawEvidenceLineageMismatch);
    }
    if (activation.observed_raw_world->version().revision <
        activation.proposal.validated_world.raw_validated_through_revision) {
      return rejected(RouteCertificationStatus3D::kRawEvidenceNotCurrent);
    }
    owned_observation.latest_raw_occupancy =
        &activation.observed_raw_world->occupancy();
    owned_observation.latest_raw_producer_instance_id =
        activation.observed_raw_world->version().producer_instance_id;
    owned_observation.latest_raw_revision =
        activation.observed_raw_world->version().revision;
    const std::optional<LaunchSupportContact3D>& launch_support =
        activation.observed_raw_world->launchSupportContact();
    observed_launch_support =
        launch_support.has_value() ? std::addressof(*launch_support) : nullptr;
    owned_observation.launch_support_contact = observed_launch_support;
    // The seed answers for the route's own departure: contact along it, an
    // obstacle nowhere else.
    const std::optional<ProprioceptiveFreeSpaceSeed3D>& proprioceptive_seed =
        activation.observed_raw_world->proprioceptiveFreeSpaceSeed();
    if (proprioceptive_seed.has_value() && activation.geometry != nullptr &&
        activation.geometry->route != nullptr) {
      departure_seed = *proprioceptive_seed;
      departure_seed->departure_chain = departureChain3D(
          *activation.geometry->route, activation.geometry->departure_end_station_m);
    }
    observed_proprioceptive_seed =
        departure_seed.has_value() ? std::addressof(*departure_seed) : nullptr;
    owned_observation.proprioceptive_free_space_seed = observed_proprioceptive_seed;
  } else {
    if (activation.observed_raw_world != nullptr ||
        activation.static_world == nullptr || !activation.static_world->valid() ||
        !sameWorldCertificate(activation.static_world->certificate(),
                              activation.proposal.validated_world)) {
      return rejected(RouteCertificationStatus3D::kStaticWorldMismatch);
    }
    owned_observation.latest_raw_occupancy = nullptr;
    owned_observation.latest_raw_producer_instance_id = 0U;
    owned_observation.latest_raw_revision = 0U;
    owned_observation.launch_support_contact = nullptr;
    owned_observation.proprioceptive_free_space_seed = nullptr;
  }
  const RouteEndpointSemantics3D planned_endpoint_semantics =
      routeEndpointSemantics3D(activation.proposal.reaches_mission_goal,
                               !activation.proposal.objective.continuous_tracking);
  const std::optional<ActivatedRouteIdentity3D> identity =
      activateRouteProposal3D(activation.proposal, activation.route_generation);
  const std::optional<ActiveIntent3D> active_intent =
      activeIntent3D(activation.proposal);
  if (!identity.has_value() || !active_intent.has_value() ||
      activation.geometry == nullptr || activation.decorations == nullptr) {
    return rejected(RouteCertificationStatus3D::kIdentityRejected);
  }
  RouteOwnerIdentity3D route_owner;
  if (activation.retained_route_owner.has_value()) {
    if (!activation.retained_route_owner->valid() ||
        !sameActiveIntent3D(activation.retained_route_owner->active_intent,
                            *active_intent)) {
      return rejected(RouteCertificationStatus3D::kRetainedOwnerMismatch);
    }
    route_owner = *activation.retained_route_owner;
  } else {
    route_owner = RouteOwnerIdentity3D{
        .id = createProducerInstanceId(kRouteOwnerDomain),
        .active_intent = *active_intent,
    };
  }
  if (!route_owner.valid()) {
    return rejected(RouteCertificationStatus3D::kRetainedOwnerMismatch);
  }
  const std::shared_ptr<const CompiledTrajectory3D> geometry = activation.geometry;
  const std::shared_ptr<const RouteDecorations3D> decorations = activation.decorations;
  if (geometry == nullptr || decorations == nullptr ||
      !compiledTrajectoryValid3D(*geometry, *identity) ||
      !routeDecorationsValid3D(*decorations, *geometry, identity->generation)) {
    return rejected(RouteCertificationStatus3D::kGeometryInvalid);
  }
  if (!footprintConservativelyContains(
          geometry->tracking_error_tube->physical_footprint,
          activation.validation_policy->sweptFootprint())) {
    return rejected(RouteCertificationStatus3D::kTubeFootprintNotContained);
  }
  if (!geometry->constrained_spans->empty() &&
      !sameFootprintConfig(decorations->passage_volume_config.footprint,
                           owned_observation.footprint)) {
    return rejected(RouteCertificationStatus3D::kPassageFootprintMismatch);
  }
  TrackingErrorTubeWorld3D tracking_tube_world;
  if (requires_observed_raw_certificate) {
    tracking_tube_world = TrackingErrorTubeWorld3D{
        .observed_occupancy = &activation.observed_raw_world->occupancy(),
        .occupied_content_fingerprint =
            activation.observed_raw_world->occupiedContentFingerprint(),
        .launch_support_contact = observed_launch_support,
        .proprioceptive_free_space_seed = observed_proprioceptive_seed,
    };
  } else {
    tracking_tube_world = TrackingErrorTubeWorld3D{
        .occupancy = &activation.static_world->occupancy(),
        .occupied_content_fingerprint = activation.static_world->contentFingerprint(),
    };
  }
  if (!trackingErrorTubeProfile3DMatchesWorld(
          *geometry->route, *geometry->tracking_error_tube, tracking_tube_world)) {
    return rejected(RouteCertificationStatus3D::kTrackingTubeWorldMismatch);
  }
  const bool passage_geometry_matches_world =
      requires_observed_raw_certificate
          ? canonicalPassageGeometryMatchesObservedWorld(
                *geometry, *decorations, *activation.observed_raw_world,
                decorations->passage_volume_config)
          : canonicalPassageGeometryMatchesWorld(*geometry, *decorations,
                                                 activation.static_world->occupancy(),
                                                 decorations->passage_volume_config);
  if (!passage_geometry_matches_world) {
    return rejected(RouteCertificationStatus3D::kPassageGeometryMismatch);
  }
  const std::uint64_t route_decorations_revision =
      routeDecorationsRevision3D(*decorations);
  const std::uint64_t passage_config_fingerprint =
      passageVolumeConfigFingerprint(decorations->passage_volume_config);
  const std::uint64_t geometry_derivation_occupancy_content_fingerprint =
      requires_observed_raw_certificate
          ? activation.observed_raw_world->occupiedContentFingerprint()
          : activation.static_world->contentFingerprint();
  if (route_decorations_revision == 0U || passage_config_fingerprint == 0U ||
      geometry_derivation_occupancy_content_fingerprint == 0U) {
    return rejected(RouteCertificationStatus3D::kDerivationFingerprintInvalid);
  }

  const RouteActivationAssessment3D assessment =
      assessRouteActivation3D(activation.proposal, *geometry->route, owned_observation);
  if (!assessment.accepted()) {
    // Name the rule the assessment refused on. A route the admission's own
    // assessment accepted can still be refused here, because the
    // certification measures the same route against the evidence and the body
    // it will be executed with; a bare refusal left one recorded flight
    // holding for over two seconds with nothing to act on.
    return rejected(!assessment.publication.compatible()
                        ? RouteCertificationStatus3D::kAssessmentPublicationIncompatible
                    : !assessment.objective_matches
                        ? RouteCertificationStatus3D::kAssessmentObjectiveMismatch
                    : !assessment.projection.valid
                        ? RouteCertificationStatus3D::kAssessmentProjectionInvalid
                    : !assessment.cross_track_accepted
                        ? RouteCertificationStatus3D::kAssessmentCrossTrackExceeded
                    : !assessment.raw_world_compatible
                        ? RouteCertificationStatus3D::kAssessmentRawWorldIncompatible
                        : RouteCertificationStatus3D::kAssessmentRawValidationRejected);
  }
  const double end_station_m = geometry->route->back().station_m;
  const RouteInstanceId3D route_instance_id{
      .value = createProducerInstanceId(kCertifiedRouteInstanceDomain)};
  if (!route_instance_id.valid()) {
    return rejected(RouteCertificationStatus3D::kInvalidArtifact);
  }
  RouteSuffixCertificate3D certificate;
  if (requires_observed_raw_certificate) {
    if (!assessment.raw_validation.connector_validated) {
      return rejected(RouteCertificationStatus3D::kRawConnectorNotValidated);
    }
    if (!assessment.raw_validation.suffix_validated) {
      return rejected(RouteCertificationStatus3D::kRawSuffixNotValidated);
    }
    const std::uint64_t policy_fingerprint = validationPolicyFingerprint(
        owned_observation.footprint, owned_observation.launch_support_contact);
    if (policy_fingerprint == 0U) {
      return rejected(RouteCertificationStatus3D::kPolicyFingerprintInvalid);
    }
    certificate = ObservedRawRouteCertificate3D{
        .route_instance_id = route_instance_id,
        .route_generation = identity->generation,
        .geometry_revision = geometry->compiled_trajectory_revision,
        .physical_route_fingerprint = geometry->physical_route_fingerprint,
        .producer_instance_id = identity->proposal.validated_world.producer_instance_id,
        .validated_through_revision = assessment.raw_validated_through_revision,
        .validation_policy_fingerprint = policy_fingerprint,
        .execution_validation_policy_fingerprint =
            activation.validation_policy->contentFingerprint(),
        .observed_world_content_fingerprint =
            activation.observed_raw_world->contentFingerprint(),
        .route_decorations_revision = route_decorations_revision,
        .passage_volume_config_fingerprint = passage_config_fingerprint,
        .geometry_derivation_occupancy_content_fingerprint =
            geometry_derivation_occupancy_content_fingerprint,
        .suffix_start_station_m = assessment.raw_validation.validated_from_station_m,
        .certified_end_station_m = end_station_m,
    };
  } else {
    const std::uint64_t policy_fingerprint =
        validationPolicyFingerprint(owned_observation.footprint, nullptr);
    if (policy_fingerprint == 0U ||
        !validateStaticRouteSuffixAgainstOwner(
            *activation.static_world, *geometry->route, assessment.projection,
            owned_observation, activation.validation_policy->flightEnvelope())) {
      return rejected(RouteCertificationStatus3D::kStaticSuffixRejected);
    }
    certificate = StaticRouteCertificate3D{
        .route_instance_id = route_instance_id,
        .route_generation = identity->generation,
        .geometry_revision = geometry->compiled_trajectory_revision,
        .physical_route_fingerprint = geometry->physical_route_fingerprint,
        .static_occupancy_content_fingerprint =
            activation.static_world->contentFingerprint(),
        .validation_policy_fingerprint = policy_fingerprint,
        .execution_validation_policy_fingerprint =
            activation.validation_policy->contentFingerprint(),
        .route_decorations_revision = route_decorations_revision,
        .passage_volume_config_fingerprint = passage_config_fingerprint,
        .geometry_derivation_occupancy_content_fingerprint =
            geometry_derivation_occupancy_content_fingerprint,
        .world_certificate = identity->proposal.validated_world,
        .suffix_start_station_m = assessment.projection.station_m,
        .certified_end_station_m = end_station_m,
    };
  }

  CertifiedRouteSuffix3D result{
      .route_instance_id = route_instance_id,
      .owner = route_owner,
      .parent_route_instance_id =
          sealed_source != nullptr
              ? std::optional<RouteInstanceId3D>{sealed_source->route_instance_id}
              : std::nullopt,
      .identity = *identity,
      .geometry = geometry,
      .decorations = decorations,
      .certificate = certificate,
      .progress = {.route_generation = identity->generation,
                   .geometry_revision = geometry->compiled_trajectory_revision,
                   .station_m = assessment.projection.station_m,
                   .last_observed_position = owned_observation.position,
                   .execution_input = nullptr},
      .continuity_lineage = activation.continuity_lineage,
      .continuity_id =
          routeContinuityId3D(identity->proposal.intent, activation.continuity_lineage),
      .observed_raw_world = activation.observed_raw_world,
      .static_world = activation.static_world,
      .validation_policy = activation.validation_policy,
      .planned_endpoint_semantics = planned_endpoint_semantics,
  };
  if (!result.valid()) {
    return rejected(RouteCertificationStatus3D::kInvalidArtifact);
  }
  if (status != nullptr) {
    *status = RouteCertificationStatus3D::kCertified;
  }
  return result;
}

} // namespace execution_route_snapshot_3d_internal

std::string_view
routeCertificationStatus3DName(const RouteCertificationStatus3D status) noexcept {
  switch (status) {
    case RouteCertificationStatus3D::kNotAttempted:
      return "not_attempted";
    case RouteCertificationStatus3D::kCertified:
      return "certified";
    case RouteCertificationStatus3D::kInvalidInput:
      return "invalid_input";
    case RouteCertificationStatus3D::kFootprintNotContained:
      return "footprint_not_contained";
    case RouteCertificationStatus3D::kRawEvidenceLineageMismatch:
      return "raw_evidence_lineage_mismatch";
    case RouteCertificationStatus3D::kRawEvidenceNotCurrent:
      return "raw_evidence_not_current";
    case RouteCertificationStatus3D::kStaticWorldMismatch:
      return "static_world_mismatch";
    case RouteCertificationStatus3D::kIdentityRejected:
      return "identity_rejected";
    case RouteCertificationStatus3D::kRetainedOwnerMismatch:
      return "retained_owner_mismatch";
    case RouteCertificationStatus3D::kGeometryInvalid:
      return "geometry_invalid";
    case RouteCertificationStatus3D::kTubeFootprintNotContained:
      return "tube_footprint_not_contained";
    case RouteCertificationStatus3D::kPassageFootprintMismatch:
      return "passage_footprint_mismatch";
    case RouteCertificationStatus3D::kTrackingTubeWorldMismatch:
      return "tracking_tube_world_mismatch";
    case RouteCertificationStatus3D::kPassageGeometryMismatch:
      return "passage_geometry_mismatch";
    case RouteCertificationStatus3D::kDerivationFingerprintInvalid:
      return "derivation_fingerprint_invalid";
    case RouteCertificationStatus3D::kAssessmentRejected:
      return "assessment_rejected";
    case RouteCertificationStatus3D::kAssessmentPublicationIncompatible:
      return "assessment_publication_incompatible";
    case RouteCertificationStatus3D::kAssessmentObjectiveMismatch:
      return "assessment_objective_mismatch";
    case RouteCertificationStatus3D::kAssessmentProjectionInvalid:
      return "assessment_projection_invalid";
    case RouteCertificationStatus3D::kAssessmentCrossTrackExceeded:
      return "assessment_cross_track_exceeded";
    case RouteCertificationStatus3D::kAssessmentRawWorldIncompatible:
      return "assessment_raw_world_incompatible";
    case RouteCertificationStatus3D::kAssessmentRawValidationRejected:
      return "assessment_raw_validation_rejected";
    case RouteCertificationStatus3D::kRawConnectorNotValidated:
      return "raw_connector_not_validated";
    case RouteCertificationStatus3D::kRawSuffixNotValidated:
      return "raw_suffix_not_validated";
    case RouteCertificationStatus3D::kPolicyFingerprintInvalid:
      return "policy_fingerprint_invalid";
    case RouteCertificationStatus3D::kStaticSuffixRejected:
      return "static_suffix_rejected";
    case RouteCertificationStatus3D::kInvalidArtifact:
      return "invalid_artifact";
  }
  return "unknown";
}

std::optional<CertifiedRouteSuffix3D>
certifyExecutionRoute3D(const ExecutionRouteActivation3D& activation,
                        RouteCertificationStatus3D* const status) {
  return certifyExecutionRoute3DImpl(activation, nullptr, status);
}

std::optional<CertifiedRouteSuffix3D> recertifyExecutionRoute3D(
    const CertifiedRouteSuffix3D& sealed_source,
    const RouteActivationObservation3D& observation,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world,
    RouteCertificationStatus3D* const status) {
  if (!sealed_source.valid() || sealed_source.geometry == nullptr ||
      (sealed_source.observed_raw_world != nullptr) !=
          (observed_raw_world != nullptr)) {
    if (status != nullptr) {
      *status = RouteCertificationStatus3D::kInvalidInput;
    }
    return std::nullopt;
  }
  return certifyExecutionRoute3DImpl(
      ExecutionRouteActivation3D{
          .route_generation = sealed_source.identity.generation,
          .proposal = sealed_source.identity.proposal,
          .geometry = sealed_source.geometry,
          .decorations = sealed_source.decorations,
          .observation = observation,
          .continuity_lineage = sealed_source.continuity_lineage,
          .observed_raw_world = std::move(observed_raw_world),
          .static_world = sealed_source.static_world,
          .validation_policy = sealed_source.validation_policy,
          .retained_route_owner = sealed_source.owner,
      },
      std::addressof(sealed_source), status);
}

} // namespace drone_city_nav
