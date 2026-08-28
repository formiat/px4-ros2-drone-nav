#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
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
                            const CertifiedRouteSuffix3D* const sealed_source) {
  const bool requires_observed_raw_certificate =
      observedRawLineage(activation.proposal.validated_world);
  if (activation.observation.raw_validation_required !=
          requires_observed_raw_certificate ||
      activation.validation_policy == nullptr ||
      !activation.validation_policy->valid() ||
      !footprintConservativelyContains(
          activation.observation.footprint,
          activation.validation_policy->sweptFootprint())) {
    return std::nullopt;
  }
  RouteActivationObservation3D owned_observation = activation.observation;
  if (requires_observed_raw_certificate) {
    if (activation.static_world != nullptr ||
        activation.observed_raw_world == nullptr ||
        !activation.observed_raw_world->valid() ||
        activation.observed_raw_world->version().producer_instance_id !=
            activation.proposal.validated_world.producer_instance_id ||
        activation.observed_raw_world->version().revision <
            activation.proposal.validated_world.raw_validated_through_revision) {
      return std::nullopt;
    }
    owned_observation.latest_raw_occupancy =
        &activation.observed_raw_world->occupancy();
    owned_observation.latest_raw_producer_instance_id =
        activation.observed_raw_world->version().producer_instance_id;
    owned_observation.latest_raw_revision =
        activation.observed_raw_world->version().revision;
    owned_observation.proprioceptive_free_space_seed =
        activation.observed_raw_world->proprioceptiveFreeSpaceSeed().has_value()
            ? &*activation.observed_raw_world->proprioceptiveFreeSpaceSeed()
            : nullptr;
    owned_observation.launch_support_contact =
        activation.observed_raw_world->launchSupportContact().has_value()
            ? &*activation.observed_raw_world->launchSupportContact()
            : nullptr;
  } else {
    if (activation.observed_raw_world != nullptr ||
        activation.static_world == nullptr || !activation.static_world->valid() ||
        !sameWorldCertificate(activation.static_world->certificate(),
                              activation.proposal.validated_world)) {
      return std::nullopt;
    }
    owned_observation.latest_raw_occupancy = nullptr;
    owned_observation.latest_raw_producer_instance_id = 0U;
    owned_observation.latest_raw_revision = 0U;
    owned_observation.proprioceptive_free_space_seed = nullptr;
    owned_observation.launch_support_contact = nullptr;
  }
  const RouteEndpointSemantics3D planned_endpoint_semantics = routeEndpointSemantics3D(
      activation.proposal.intent, activation.proposal.evidence.reaches_intent_target,
      activation.proposal.reaches_mission_goal,
      !activation.proposal.objective.continuous_tracking);
  const std::optional<ActivatedRouteIdentity3D> identity =
      activateRouteProposal3D(activation.proposal, activation.route_generation);
  const std::optional<ActiveIntent3D> active_intent =
      activeIntent3D(activation.proposal);
  if (!identity.has_value() || !active_intent.has_value() ||
      activation.geometry == nullptr) {
    return std::nullopt;
  }
  RouteOwnerIdentity3D route_owner;
  if (activation.retained_route_owner.has_value()) {
    if (!activation.retained_route_owner->valid() ||
        !sameActiveIntent3D(activation.retained_route_owner->active_intent,
                            *active_intent)) {
      return std::nullopt;
    }
    route_owner = *activation.retained_route_owner;
  } else {
    route_owner = RouteOwnerIdentity3D{
        .id = createProducerInstanceId(kRouteOwnerDomain),
        .active_intent = *active_intent,
    };
  }
  if (!route_owner.valid()) {
    return std::nullopt;
  }
  const std::shared_ptr<const ExecutionRouteGeometry3D> geometry =
      sealed_source != nullptr ? activation.geometry
                               : captureExecutionRouteGeometry3D(*activation.geometry);
  if (geometry == nullptr || !executionRouteGeometryValid3D(*geometry, *identity)) {
    return std::nullopt;
  }
  if (!samePassageVolumeConfig(geometry->passage_volume_config,
                               activation.passage_volume_config) ||
      (!geometry->constrained_spans->empty() &&
       !sameFootprintConfig(activation.passage_volume_config.footprint,
                            owned_observation.footprint))) {
    return std::nullopt;
  }
  const bool passage_geometry_matches_world =
      requires_observed_raw_certificate
          ? canonicalPassageGeometryMatchesObservedWorld(
                *geometry, *activation.observed_raw_world,
                activation.passage_volume_config)
          : canonicalPassageGeometryMatchesWorld(*geometry,
                                                 activation.static_world->occupancy(),
                                                 activation.passage_volume_config);
  if (!passage_geometry_matches_world) {
    return std::nullopt;
  }
  const std::uint64_t passage_geometry_revision =
      executionPassageGeometryRevision3D(*geometry);
  const std::uint64_t passage_config_fingerprint =
      passageVolumeConfigFingerprint(activation.passage_volume_config);
  const std::uint64_t passage_derivation_occupancy_content_fingerprint =
      requires_observed_raw_certificate
          ? activation.observed_raw_world->occupiedContentFingerprint()
          : activation.static_world->contentFingerprint();
  if (passage_geometry_revision == 0U || passage_config_fingerprint == 0U ||
      passage_derivation_occupancy_content_fingerprint == 0U) {
    return std::nullopt;
  }

  const RouteActivationAssessment3D assessment =
      assessRouteActivation3D(activation.proposal, *geometry->route, owned_observation);
  if (!assessment.accepted()) {
    return std::nullopt;
  }
  const double end_station_m = geometry->route->back().station_m;
  const RouteInstanceId3D route_instance_id{
      .value = createProducerInstanceId(kCertifiedRouteInstanceDomain)};
  if (!route_instance_id.valid()) {
    return std::nullopt;
  }
  RouteSuffixCertificate3D certificate;
  if (requires_observed_raw_certificate) {
    if (!assessment.raw_validation.connector_validated ||
        !assessment.raw_validation.suffix_validated) {
      return std::nullopt;
    }
    const std::uint64_t policy_fingerprint = validationPolicyFingerprint(
        owned_observation.footprint, ObservedSpaceValidationPolicy::kAllowUnknown,
        owned_observation.proprioceptive_free_space_seed,
        owned_observation.launch_support_contact);
    if (policy_fingerprint == 0U) {
      return std::nullopt;
    }
    certificate = ObservedRawRouteCertificate3D{
        .route_instance_id = route_instance_id,
        .route_generation = identity->generation,
        .geometry_revision = geometry->executable_geometry_revision,
        .physical_route_fingerprint = geometry->physical_route_fingerprint,
        .producer_instance_id = identity->proposal.validated_world.producer_instance_id,
        .validated_through_revision = assessment.raw_validated_through_revision,
        .validation_policy_fingerprint = policy_fingerprint,
        .execution_validation_policy_fingerprint =
            activation.validation_policy->contentFingerprint(),
        .observed_world_content_fingerprint =
            activation.observed_raw_world->contentFingerprint(),
        .passage_geometry_revision = passage_geometry_revision,
        .passage_volume_config_fingerprint = passage_config_fingerprint,
        .passage_derivation_occupancy_content_fingerprint =
            passage_derivation_occupancy_content_fingerprint,
        .suffix_start_station_m = assessment.raw_validation.validated_from_station_m,
        .certified_end_station_m = end_station_m,
    };
  } else {
    const std::uint64_t policy_fingerprint = validationPolicyFingerprint(
        owned_observation.footprint, ObservedSpaceValidationPolicy::kRequireKnownFree,
        nullptr, nullptr);
    if (policy_fingerprint == 0U || !validateStaticRouteSuffixAgainstOwner(
                                        *activation.static_world, *geometry->route,
                                        assessment.projection, owned_observation)) {
      return std::nullopt;
    }
    certificate = StaticRouteCertificate3D{
        .route_instance_id = route_instance_id,
        .route_generation = identity->generation,
        .geometry_revision = geometry->executable_geometry_revision,
        .physical_route_fingerprint = geometry->physical_route_fingerprint,
        .static_occupancy_content_fingerprint =
            activation.static_world->contentFingerprint(),
        .validation_policy_fingerprint = policy_fingerprint,
        .execution_validation_policy_fingerprint =
            activation.validation_policy->contentFingerprint(),
        .passage_geometry_revision = passage_geometry_revision,
        .passage_volume_config_fingerprint = passage_config_fingerprint,
        .passage_derivation_occupancy_content_fingerprint =
            passage_derivation_occupancy_content_fingerprint,
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
      .certificate = certificate,
      .progress = {.route_generation = identity->generation,
                   .geometry_revision = geometry->executable_geometry_revision,
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
  return result.valid() ? std::optional<CertifiedRouteSuffix3D>{std::move(result)}
                        : std::nullopt;
}

} // namespace execution_route_snapshot_3d_internal

std::optional<CertifiedRouteSuffix3D>
certifyExecutionRoute3D(const ExecutionRouteActivation3D& activation) {
  return certifyExecutionRoute3DImpl(activation, nullptr);
}

std::optional<CertifiedRouteSuffix3D> recertifyExecutionRoute3D(
    const CertifiedRouteSuffix3D& sealed_source,
    const RouteActivationObservation3D& observation,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world) {
  if (!sealed_source.valid() || sealed_source.geometry == nullptr) {
    return std::nullopt;
  }
  const bool observed = sealed_source.observed_raw_world != nullptr;
  if (observed != (observed_raw_world != nullptr)) {
    return std::nullopt;
  }
  return certifyExecutionRoute3DImpl(
      ExecutionRouteActivation3D{
          .route_generation = sealed_source.identity.generation,
          .proposal = sealed_source.identity.proposal,
          .geometry = sealed_source.geometry,
          .observation = observation,
          .passage_volume_config = sealed_source.geometry->passage_volume_config,
          .continuity_lineage = sealed_source.continuity_lineage,
          .observed_raw_world = std::move(observed_raw_world),
          .static_world = sealed_source.static_world,
          .validation_policy = sealed_source.validation_policy,
          .retained_route_owner = sealed_source.owner,
      },
      std::addressof(sealed_source));
}

} // namespace drone_city_nav
