#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_certificates_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/occupied_collision_oracle_3d.hpp"

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

namespace drone_city_nav::execution_route_snapshot_3d_internal {

[[nodiscard]] bool validStationInterval(const double begin_station_m,
                                        const double end_station_m,
                                        const double route_end_station_m) noexcept {
  return std::isfinite(begin_station_m) && std::isfinite(end_station_m) &&
         std::isfinite(route_end_station_m) && route_end_station_m > 0.0 &&
         begin_station_m >= 0.0 &&
         end_station_m + kStationToleranceM >= begin_station_m &&
         end_station_m <= route_end_station_m + kStationToleranceM &&
         std::abs(end_station_m - route_end_station_m) <= kStationToleranceM;
}

[[nodiscard]] std::uint64_t expectedStaticOccupancyFingerprint(
    const StaticRouteCertificate3D& certificate) noexcept {
  return certificate.world_certificate.esdf_source_occupied_fingerprint != 0U
             ? certificate.world_certificate.esdf_source_occupied_fingerprint
             : certificate.world_certificate.esdf_fingerprint;
}

[[nodiscard]] CertificateView3D
certificateView(const RouteSuffixCertificate3D& certificate) noexcept {
  if (const auto* const static_certificate =
          std::get_if<StaticRouteCertificate3D>(&certificate)) {
    return {
        .route_instance_id = static_certificate->route_instance_id,
        .route_generation = static_certificate->route_generation,
        .geometry_revision = static_certificate->geometry_revision,
        .physical_route_fingerprint = static_certificate->physical_route_fingerprint,
        .producer_instance_id =
            static_certificate->world_certificate.producer_instance_id,
        .validated_through_revision =
            static_certificate->world_certificate.raw_validated_through_revision,
        .validation_policy_fingerprint =
            static_certificate->validation_policy_fingerprint,
        .execution_validation_policy_fingerprint =
            static_certificate->execution_validation_policy_fingerprint,
        .world_content_fingerprint =
            static_certificate->static_occupancy_content_fingerprint,
        .route_decorations_revision = static_certificate->route_decorations_revision,
        .passage_volume_config_fingerprint =
            static_certificate->passage_volume_config_fingerprint,
        .geometry_derivation_occupancy_content_fingerprint =
            static_certificate->geometry_derivation_occupancy_content_fingerprint,
        .suffix_start_station_m = static_certificate->suffix_start_station_m,
        .certified_end_station_m = static_certificate->certified_end_station_m,
        .observed_raw = false,
    };
  }
  const auto* const raw_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&certificate);
  if (raw_certificate == nullptr) {
    return {};
  }
  return {
      .route_instance_id = raw_certificate->route_instance_id,
      .route_generation = raw_certificate->route_generation,
      .geometry_revision = raw_certificate->geometry_revision,
      .physical_route_fingerprint = raw_certificate->physical_route_fingerprint,
      .producer_instance_id = raw_certificate->producer_instance_id,
      .validated_through_revision = raw_certificate->validated_through_revision,
      .validation_policy_fingerprint = raw_certificate->validation_policy_fingerprint,
      .execution_validation_policy_fingerprint =
          raw_certificate->execution_validation_policy_fingerprint,
      .world_content_fingerprint = raw_certificate->observed_world_content_fingerprint,
      .route_decorations_revision = raw_certificate->route_decorations_revision,
      .passage_volume_config_fingerprint =
          raw_certificate->passage_volume_config_fingerprint,
      .geometry_derivation_occupancy_content_fingerprint =
          raw_certificate->geometry_derivation_occupancy_content_fingerprint,
      .suffix_start_station_m = raw_certificate->suffix_start_station_m,
      .certified_end_station_m = raw_certificate->certified_end_station_m,
      .observed_raw = true,
  };
}

void hashDouble(std::uint64_t& hash, const double value) noexcept {
  hashValue(hash, canonicalDoubleBits(value));
}

void hashFloat(std::uint64_t& hash, const float value) noexcept {
  hashValue(hash, value == 0.0F ? 0U
                                : static_cast<std::uint64_t>(
                                      std::bit_cast<std::uint32_t>(value)));
}

void hashVector(std::uint64_t& hash, const Vec3& vector) noexcept {
  hashDouble(hash, vector.x);
  hashDouble(hash, vector.y);
  hashDouble(hash, vector.z);
}

void hashCertificate(std::uint64_t& hash,
                     const RouteSuffixCertificate3D& certificate) noexcept {
  const CertificateView3D view = certificateView(certificate);
  hashValue(hash, view.route_instance_id.value);
  hashValue(hash, view.route_generation);
  hashValue(hash, view.geometry_revision);
  hashValue(hash, view.physical_route_fingerprint);
  hashValue(hash, view.producer_instance_id);
  hashValue(hash, view.validated_through_revision);
  hashValue(hash, view.validation_policy_fingerprint);
  hashValue(hash, view.execution_validation_policy_fingerprint);
  hashValue(hash, view.world_content_fingerprint);
  hashValue(hash, view.route_decorations_revision);
  hashValue(hash, view.passage_volume_config_fingerprint);
  hashValue(hash, view.geometry_derivation_occupancy_content_fingerprint);
  hashDouble(hash, view.suffix_start_station_m);
  hashDouble(hash, view.certified_end_station_m);
  hashValue(hash, view.observed_raw ? 1U : 0U);
  const auto* const static_certificate =
      std::get_if<StaticRouteCertificate3D>(&certificate);
  if (static_certificate == nullptr) {
    return;
  }
  const NavigationWorldCertificate3D& world = static_certificate->world_certificate;
  hashValue(hash, world.esdf_fingerprint);
  hashValue(hash, world.esdf_source_raw_revision);
  hashValue(hash, world.esdf_source_occupied_fingerprint);
  hashValue(hash, world.local_world_generation);
  hashValue(hash, world.topology_revision);
}

[[nodiscard]] bool validationLineageValidForCertificate(
    const FiniteExecutionValidationLineage3D& lineage,
    const RouteSuffixCertificate3D& certificate) noexcept {
  const CertificateView3D certificate_view = certificateView(certificate);
  if (certificate_view.observed_raw) {
    const auto* const raw_lineage =
        std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(&lineage);
    return raw_lineage != nullptr && raw_lineage->producer_instance_id != 0U &&
           raw_lineage->producer_instance_id == certificate_view.producer_instance_id &&
           raw_lineage->validated_through_raw_revision >=
               certificate_view.validated_through_revision &&
           raw_lineage->validation_policy_fingerprint != 0U &&
           raw_lineage->observed_world_content_fingerprint != 0U &&
           (raw_lineage->validated_through_raw_revision >
                certificate_view.validated_through_revision ||
            raw_lineage->observed_world_content_fingerprint ==
                certificate_view.world_content_fingerprint);
  }
  const auto* const static_lineage =
      std::get_if<StaticFiniteExecutionValidationLineage3D>(&lineage);
  const auto* const static_certificate =
      std::get_if<StaticRouteCertificate3D>(&certificate);
  return static_lineage != nullptr && static_certificate != nullptr &&
         sameWorldCertificate(static_lineage->world_certificate,
                              static_certificate->world_certificate) &&
         static_lineage->static_occupancy_content_fingerprint ==
             static_certificate->static_occupancy_content_fingerprint &&
         static_lineage->validation_policy_fingerprint != 0U;
}

[[nodiscard]] std::uint64_t
finiteExecutionArtifactFingerprint(const FiniteExecutionState3D& execution) noexcept {
  if (execution.horizon == nullptr) {
    return 0U;
  }
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, execution.trajectory_revision);
  hashValue(hash, execution.source_snapshot_version);
  hashValue(hash, execution.source_navigation_revision);
  hashValue(hash, execution.source_route_instance_id.value);
  hashValue(hash, execution.source_route_generation);
  hashValue(hash, execution.source_geometry_revision);
  hashValue(hash, execution.source_physical_route_fingerprint);
  hashValue(hash, execution.validation_policy != nullptr
                      ? execution.validation_policy->contentFingerprint()
                      : 0U);
  hashValue(hash, execution.execution_input != nullptr
                      ? execution.execution_input->contentFingerprint()
                      : 0U);
  hashValue(hash, execution.latest_lidar_evidence != nullptr
                      ? execution.latest_lidar_evidence->contentFingerprint()
                      : 0U);
  hashCertificate(hash, execution.certificate);
  hashValue(hash, static_cast<std::uint64_t>(execution.horizon->states.size()));
  for (const MotionState3D& state : execution.horizon->states) {
    hashFloat(hash, state.x);
    hashFloat(hash, state.y);
    hashFloat(hash, state.z);
    hashFloat(hash, state.vx);
    hashFloat(hash, state.vy);
    hashFloat(hash, state.vz);
    hashFloat(hash, state.yaw);
    hashFloat(hash, state.yaw_rate);
  }
  hashValue(hash, static_cast<std::uint64_t>(execution.horizon->controls.size()));
  for (const MotionControl3D& control : execution.horizon->controls) {
    hashFloat(hash, control.ax);
    hashFloat(hash, control.ay);
    hashFloat(hash, control.az);
    hashFloat(hash, control.yaw_accel);
  }
  hashValue(hash, static_cast<std::uint64_t>(
                      execution.horizon->nominal_prefix_control_count));
  hashValue(hash, static_cast<std::uint64_t>(execution.horizon->arrival_control_count));
  hashValue(hash, execution.terminal_boundary.has_value() ? 1U : 0U);
  if (execution.terminal_boundary.has_value()) {
    const FiniteRouteTerminalBoundary3D& boundary = *execution.terminal_boundary;
    hashPoint(hash, boundary.endpoint);
    hashVector(hash, boundary.forward);
    hashDouble(hash, boundary.tolerance_m);
    hashDouble(hash, boundary.activation_distance_m);
    hashDouble(hash, boundary.maximum_cross_track_m);
    hashDouble(hash, boundary.initial_route_station_m);
    hashDouble(hash, boundary.activation_route_station_m);
  }
  hashValue(hash, execution.stop_boundary.route_generation);
  hashValue(hash, execution.stop_boundary.route_instance_id.value);
  hashValue(hash, execution.stop_boundary.geometry_revision);
  hashValue(hash, execution.stop_boundary.physical_route_fingerprint);
  hashValue(hash, execution.stop_boundary.trajectory_revision);
  hashValue(hash, execution.stop_boundary.raw_validated_through_revision);
  hashDouble(hash, execution.stop_boundary.station_m);
  hashDouble(hash, execution.stop_boundary.position_tolerance_m);
  hashPoint(hash, execution.stop_boundary.position);
  hashDouble(hash, execution.begin_route_station_m);
  hashValue(hash, static_cast<std::uint64_t>(execution.valid_from_ns));
  hashValue(hash, static_cast<std::uint64_t>(execution.valid_until_ns));
  hashValue(hash, static_cast<std::uint64_t>(execution.control_interval_ns));
  hashValue(hash, static_cast<std::uint64_t>(execution.kind));
  hashValue(hash, execution.validation_proof.validation_contract_fingerprint);
  hashValue(hash,
            static_cast<std::uint64_t>(execution.validation_proof.lineage.index()));
  if (const auto* const static_lineage =
          std::get_if<StaticFiniteExecutionValidationLineage3D>(
              &execution.validation_proof.lineage)) {
    const NavigationWorldCertificate3D& world = static_lineage->world_certificate;
    hashValue(hash, world.producer_instance_id);
    hashValue(hash, world.esdf_fingerprint);
    hashValue(hash, world.esdf_source_raw_revision);
    hashValue(hash, world.esdf_source_occupied_fingerprint);
    hashValue(hash, world.raw_validated_through_revision);
    hashValue(hash, world.local_world_generation);
    hashValue(hash, world.topology_revision);
    hashValue(hash, static_lineage->static_occupancy_content_fingerprint);
    hashValue(hash, static_lineage->validation_policy_fingerprint);
  } else if (const auto* const raw_lineage =
                 std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
                     &execution.validation_proof.lineage)) {
    hashValue(hash, raw_lineage->producer_instance_id);
    hashValue(hash, raw_lineage->validated_through_raw_revision);
    hashValue(hash, raw_lineage->validation_policy_fingerprint);
    hashValue(hash, raw_lineage->observed_world_content_fingerprint);
  }
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] bool
sameDirectTrackingOwner(const DirectTrackingOwnerIdentity3D& first,
                        const DirectTrackingOwnerIdentity3D& second) noexcept {
  return first.mission_epoch == second.mission_epoch &&
         first.assignment_generation == second.assignment_generation &&
         first.target_detection_id == second.target_detection_id &&
         first.target_track_id == second.target_track_id;
}

[[nodiscard]] std::uint64_t directTrackingExecutionArtifactFingerprint(
    const DirectTrackingFiniteExecution3D& execution) noexcept {
  if (execution.horizon == nullptr) {
    return 0U;
  }
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, execution.identity.mission_epoch);
  hashValue(hash, execution.identity.assignment_generation);
  hashValue(hash, execution.identity.target_detection_id);
  hashValue(hash, execution.identity.target_track_id);
  hashValue(hash, execution.identity.objective_sample_sequence);
  hashValue(hash, execution.identity.line_of_sight_generation);
  hashValue(hash, execution.trajectory_revision);
  hashValue(hash, execution.source_snapshot_version);
  hashValue(hash, execution.source_navigation_revision);
  hashPoint(hash, execution.target);
  hashValue(hash, execution.validation_policy != nullptr
                      ? execution.validation_policy->contentFingerprint()
                      : 0U);
  hashValue(hash, execution.execution_input != nullptr
                      ? execution.execution_input->contentFingerprint()
                      : 0U);
  hashValue(hash, execution.latest_lidar_evidence != nullptr
                      ? execution.latest_lidar_evidence->contentFingerprint()
                      : 0U);
  hashValue(hash, execution.observed_raw_world != nullptr
                      ? execution.observed_raw_world->contentFingerprint()
                      : 0U);
  hashValue(hash, execution.static_world != nullptr
                      ? execution.static_world->contentFingerprint()
                      : 0U);
  hashValue(hash, static_cast<std::uint64_t>(execution.horizon->states.size()));
  for (const MotionState3D& state : execution.horizon->states) {
    hashFloat(hash, state.x);
    hashFloat(hash, state.y);
    hashFloat(hash, state.z);
    hashFloat(hash, state.vx);
    hashFloat(hash, state.vy);
    hashFloat(hash, state.vz);
    hashFloat(hash, state.yaw);
    hashFloat(hash, state.yaw_rate);
  }
  hashValue(hash, static_cast<std::uint64_t>(execution.horizon->controls.size()));
  for (const MotionControl3D& control : execution.horizon->controls) {
    hashFloat(hash, control.ax);
    hashFloat(hash, control.ay);
    hashFloat(hash, control.az);
    hashFloat(hash, control.yaw_accel);
  }
  hashValue(hash, static_cast<std::uint64_t>(
                      execution.horizon->nominal_prefix_control_count));
  hashValue(hash, static_cast<std::uint64_t>(execution.horizon->arrival_control_count));
  hashValue(hash, static_cast<std::uint64_t>(execution.valid_from_ns));
  hashValue(hash, static_cast<std::uint64_t>(execution.valid_until_ns));
  hashValue(hash, static_cast<std::uint64_t>(execution.control_interval_ns));
  hashValue(hash, static_cast<std::uint64_t>(execution.kind));
  hashValue(hash, execution.validation_proof.validation_contract_fingerprint);
  hashValue(hash,
            static_cast<std::uint64_t>(execution.validation_proof.lineage.index()));
  if (const auto* const static_lineage =
          std::get_if<StaticFiniteExecutionValidationLineage3D>(
              &execution.validation_proof.lineage)) {
    const NavigationWorldCertificate3D& world = static_lineage->world_certificate;
    hashValue(hash, world.producer_instance_id);
    hashValue(hash, world.esdf_fingerprint);
    hashValue(hash, world.esdf_source_raw_revision);
    hashValue(hash, world.esdf_source_occupied_fingerprint);
    hashValue(hash, world.raw_validated_through_revision);
    hashValue(hash, world.local_world_generation);
    hashValue(hash, world.topology_revision);
    hashValue(hash, static_lineage->static_occupancy_content_fingerprint);
    hashValue(hash, static_lineage->validation_policy_fingerprint);
  } else if (const auto* const raw_lineage =
                 std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
                     &execution.validation_proof.lineage)) {
    hashValue(hash, raw_lineage->producer_instance_id);
    hashValue(hash, raw_lineage->validated_through_raw_revision);
    hashValue(hash, raw_lineage->validation_policy_fingerprint);
    hashValue(hash, raw_lineage->observed_world_content_fingerprint);
  }
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] bool
certificateValidForSource(const RouteSuffixCertificate3D& certificate,
                          const RouteInstanceId3D route_instance_id,
                          const std::uint64_t route_generation,
                          const std::uint64_t geometry_revision,
                          const std::uint64_t physical_route_fingerprint) noexcept {
  const CertificateView3D view = certificateView(certificate);
  if (!route_instance_id.valid() || view.route_instance_id != route_instance_id ||
      view.route_generation == 0U || view.route_generation != route_generation ||
      view.geometry_revision == 0U || view.geometry_revision != geometry_revision ||
      view.physical_route_fingerprint == 0U ||
      view.physical_route_fingerprint != physical_route_fingerprint ||
      view.route_decorations_revision == 0U ||
      view.execution_validation_policy_fingerprint == 0U ||
      view.passage_volume_config_fingerprint == 0U ||
      view.geometry_derivation_occupancy_content_fingerprint == 0U ||
      !std::isfinite(view.suffix_start_station_m) ||
      !std::isfinite(view.certified_end_station_m) ||
      view.suffix_start_station_m < 0.0 ||
      view.certified_end_station_m + kStationToleranceM < view.suffix_start_station_m) {
    return false;
  }
  if (view.observed_raw) {
    return view.producer_instance_id != 0U && view.validated_through_revision != 0U &&
           view.validation_policy_fingerprint != 0U &&
           view.world_content_fingerprint != 0U;
  }
  return view.world_content_fingerprint != 0U &&
         view.validation_policy_fingerprint != 0U &&
         std::get<StaticRouteCertificate3D>(certificate).world_certificate.valid();
}

[[nodiscard]] bool
sameCertificateBinding(const RouteSuffixCertificate3D& first,
                       const RouteSuffixCertificate3D& second) noexcept {
  const CertificateView3D first_view = certificateView(first);
  const CertificateView3D second_view = certificateView(second);
  if (first.index() != second.index() ||
      first_view.route_instance_id != second_view.route_instance_id ||
      first_view.route_generation != second_view.route_generation ||
      first_view.geometry_revision != second_view.geometry_revision ||
      first_view.physical_route_fingerprint != second_view.physical_route_fingerprint ||
      first_view.producer_instance_id != second_view.producer_instance_id ||
      first_view.validation_policy_fingerprint !=
          second_view.validation_policy_fingerprint ||
      first_view.execution_validation_policy_fingerprint !=
          second_view.execution_validation_policy_fingerprint ||
      first_view.world_content_fingerprint != second_view.world_content_fingerprint ||
      first_view.route_decorations_revision != second_view.route_decorations_revision ||
      first_view.passage_volume_config_fingerprint !=
          second_view.passage_volume_config_fingerprint ||
      first_view.geometry_derivation_occupancy_content_fingerprint !=
          second_view.geometry_derivation_occupancy_content_fingerprint ||
      first_view.observed_raw != second_view.observed_raw) {
    return false;
  }
  if (!first_view.observed_raw) {
    return sameWorldCertificate(
        std::get<StaticRouteCertificate3D>(first).world_certificate,
        std::get<StaticRouteCertificate3D>(second).world_certificate);
  }
  return first_view.validation_policy_fingerprint ==
         second_view.validation_policy_fingerprint;
}

[[nodiscard]] bool
certificateNotNewerThan(const RouteSuffixCertificate3D& artifact,
                        const RouteSuffixCertificate3D& route) noexcept {
  if (!sameCertificateBinding(artifact, route)) {
    return false;
  }
  const CertificateView3D artifact_view = certificateView(artifact);
  const CertificateView3D route_view = certificateView(route);
  return artifact_view.validated_through_revision <=
             route_view.validated_through_revision &&
         artifact_view.certified_end_station_m <=
             route_view.certified_end_station_m + kStationToleranceM;
}

[[nodiscard]] bool
certificateEligibleForRevalidation(const RouteSuffixCertificate3D& artifact,
                                   const RouteSuffixCertificate3D& route) noexcept {
  const CertificateView3D artifact_view = certificateView(artifact);
  const CertificateView3D route_view = certificateView(route);
  if (artifact.index() != route.index() ||
      artifact_view.route_instance_id != route_view.route_instance_id ||
      artifact_view.route_generation != route_view.route_generation ||
      artifact_view.geometry_revision != route_view.geometry_revision ||
      artifact_view.physical_route_fingerprint !=
          route_view.physical_route_fingerprint ||
      artifact_view.execution_validation_policy_fingerprint !=
          route_view.execution_validation_policy_fingerprint ||
      artifact_view.route_decorations_revision !=
          route_view.route_decorations_revision ||
      artifact_view.passage_volume_config_fingerprint !=
          route_view.passage_volume_config_fingerprint ||
      artifact_view.observed_raw != route_view.observed_raw ||
      artifact_view.certified_end_station_m >
          route_view.certified_end_station_m + kStationToleranceM) {
    return false;
  }
  if (!artifact_view.observed_raw) {
    return sameWorldCertificate(
               std::get<StaticRouteCertificate3D>(artifact).world_certificate,
               std::get<StaticRouteCertificate3D>(route).world_certificate) &&
           artifact_view.world_content_fingerprint ==
               route_view.world_content_fingerprint;
  }
  return artifact_view.producer_instance_id == route_view.producer_instance_id &&
         artifact_view.validation_policy_fingerprint ==
             route_view.validation_policy_fingerprint &&
         artifact_view.validated_through_revision <=
             route_view.validated_through_revision;
}

[[nodiscard]] bool sameCertificate(const RouteSuffixCertificate3D& first,
                                   const RouteSuffixCertificate3D& second) noexcept {
  if (!sameCertificateBinding(first, second)) {
    return false;
  }
  const CertificateView3D first_view = certificateView(first);
  const CertificateView3D second_view = certificateView(second);
  return first_view.validated_through_revision ==
             second_view.validated_through_revision &&
         nearlyEqual(first_view.suffix_start_station_m,
                     second_view.suffix_start_station_m, kStationToleranceM) &&
         nearlyEqual(first_view.certified_end_station_m,
                     second_view.certified_end_station_m, kStationToleranceM);
}

[[nodiscard]] bool rawWorldMatchesCertificate(
    const std::shared_ptr<const VersionedObservedRawWorld3D>& world,
    const ObservedRawRouteCertificate3D& certificate,
    const bool require_exact_revision) noexcept {
  return world != nullptr && world->valid() &&
         world->version().producer_instance_id == certificate.producer_instance_id &&
         (require_exact_revision
              ? world->version().revision == certificate.validated_through_revision
              : world->version().revision >= certificate.validated_through_revision) &&
         world->contentFingerprint() == certificate.observed_world_content_fingerprint;
}

[[nodiscard]] bool staticWorldMatchesCertificate(
    const std::shared_ptr<const VersionedStaticWorld3D>& world,
    const StaticRouteCertificate3D& certificate) noexcept {
  return world != nullptr && world->valid() &&
         sameWorldCertificate(world->certificate(), certificate.world_certificate) &&
         world->contentFingerprint() ==
             certificate.static_occupancy_content_fingerprint;
}

[[nodiscard]] bool validateStaticRouteSuffixAgainstOwner(
    const VersionedStaticWorld3D& world, const std::span<const RouteSample3D> route,
    const RouteProjection3D& projection,
    const RouteActivationObservation3D& observation,
    const FlightEnvelopeConfig& flight_envelope) noexcept {
  if (!projection.valid || route.size() < 2U) {
    return false;
  }
  RouteSample3D previous = sampleRoute3DAtStation(route, projection.station_m);
  const FootprintBodyAxis footprint_axis{};
  const OccupiedCollisionOracle3D oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = nullptr,
      .static_occupancy = std::addressof(world.occupancy()),
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = observation.footprint,
      .flight_envelope = flight_envelope,
  }};
  if (!oracle
           .validateSegment(observation.position, footprint_axis, projection.point,
                            footprint_axis)
           .clear()) {
    return false;
  }
  const auto first = std::ranges::upper_bound(route, projection.station_m, {},
                                              &RouteSample3D::station_m);
  for (auto sample = first; sample != route.end(); ++sample) {
    if (!oracle
             .validateSegment(previous.position, footprint_axis, sample->position,
                              footprint_axis)
             .clear()) {
      return false;
    }
    previous = *sample;
  }
  return true;
}

[[nodiscard]] bool
finiteWorldOwnerMatchesProof(const FiniteExecutionState3D& execution) noexcept {
  const CertificateView3D certificate_view = certificateView(execution.certificate);
  if (execution.validation_policy == nullptr || !execution.validation_policy->valid() ||
      execution.validation_policy->contentFingerprint() !=
          certificate_view.execution_validation_policy_fingerprint ||
      execution.execution_input == nullptr || !execution.execution_input->valid() ||
      !execution.execution_input->nominalStateAuthoritative() ||
      !executionInputFreshAt(*execution.execution_input, *execution.validation_policy,
                             execution.valid_from_ns) ||
      execution.execution_input->poseRevision() !=
          execution.source_navigation_revision ||
      execution.execution_input->effectiveStampNs() != execution.valid_from_ns ||
      execution.latest_lidar_evidence == nullptr ||
      !latestLidarEvidenceFreshAt(*execution.latest_lidar_evidence,
                                  *execution.validation_policy,
                                  execution.valid_from_ns)) {
    return false;
  }
  if (const auto* const raw_certificate =
          std::get_if<ObservedRawRouteCertificate3D>(&execution.certificate)) {
    const auto* const raw_lineage =
        std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
            &execution.validation_proof.lineage);
    return execution.static_world == nullptr && raw_lineage != nullptr &&
           execution.observed_raw_world != nullptr &&
           execution.observed_raw_world->valid() &&
           raw_lineage->producer_instance_id == raw_certificate->producer_instance_id &&
           raw_lineage->validated_through_raw_revision >=
               raw_certificate->validated_through_revision &&
           (raw_lineage->validated_through_raw_revision >
                raw_certificate->validated_through_revision ||
            raw_lineage->observed_world_content_fingerprint ==
                raw_certificate->observed_world_content_fingerprint) &&
           execution.observed_raw_world->version().producer_instance_id ==
               raw_lineage->producer_instance_id &&
           execution.observed_raw_world->version().revision ==
               raw_lineage->validated_through_raw_revision &&
           execution.observed_raw_world->contentFingerprint() ==
               raw_lineage->observed_world_content_fingerprint;
  }
  const auto* const static_certificate =
      std::get_if<StaticRouteCertificate3D>(&execution.certificate);
  const auto* const static_lineage =
      std::get_if<StaticFiniteExecutionValidationLineage3D>(
          &execution.validation_proof.lineage);
  return static_certificate != nullptr && static_lineage != nullptr &&
         execution.observed_raw_world == nullptr &&
         staticWorldMatchesCertificate(execution.static_world, *static_certificate) &&
         sameWorldCertificate(execution.static_world->certificate(),
                              static_lineage->world_certificate) &&
         execution.static_world->contentFingerprint() ==
             static_lineage->static_occupancy_content_fingerprint;
}

[[nodiscard]] bool directTrackingWorldOwnerMatchesProof(
    const DirectTrackingFiniteExecution3D& execution) noexcept {
  if (execution.validation_policy == nullptr || !execution.validation_policy->valid() ||
      execution.execution_input == nullptr || !execution.execution_input->valid() ||
      !execution.execution_input->nominalStateAuthoritative() ||
      !executionInputFreshAt(*execution.execution_input, *execution.validation_policy,
                             execution.valid_from_ns) ||
      execution.execution_input->poseRevision() !=
          execution.source_navigation_revision ||
      execution.execution_input->effectiveStampNs() != execution.valid_from_ns ||
      execution.latest_lidar_evidence == nullptr ||
      !latestLidarEvidenceFreshAt(*execution.latest_lidar_evidence,
                                  *execution.validation_policy,
                                  execution.valid_from_ns) ||
      (execution.observed_raw_world == nullptr) ==
          (execution.static_world == nullptr)) {
    return false;
  }
  if (execution.observed_raw_world != nullptr) {
    const auto* const raw_lineage =
        std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
            &execution.validation_proof.lineage);
    return raw_lineage != nullptr && execution.observed_raw_world->valid() &&
           execution.static_world == nullptr &&
           raw_lineage->producer_instance_id ==
               execution.observed_raw_world->version().producer_instance_id &&
           raw_lineage->validated_through_raw_revision ==
               execution.observed_raw_world->version().revision &&
           raw_lineage->observed_world_content_fingerprint ==
               execution.observed_raw_world->contentFingerprint() &&
           raw_lineage->validation_policy_fingerprint ==
               validationPolicyFingerprint(
                   execution.validation_policy->sweptFootprint(),
                   execution.observed_raw_world->launchSupportContact().has_value()
                       ? std::addressof(
                             *execution.observed_raw_world->launchSupportContact())
                       : nullptr);
  }
  const auto* const static_lineage =
      std::get_if<StaticFiniteExecutionValidationLineage3D>(
          &execution.validation_proof.lineage);
  return static_lineage != nullptr && execution.static_world->valid() &&
         execution.observed_raw_world == nullptr &&
         sameWorldCertificate(static_lineage->world_certificate,
                              execution.static_world->certificate()) &&
         static_lineage->static_occupancy_content_fingerprint ==
             execution.static_world->contentFingerprint() &&
         static_lineage->validation_policy_fingerprint ==
             validationPolicyFingerprint(execution.validation_policy->sweptFootprint(),
                                         nullptr);
}

[[nodiscard]] std::uint64_t
rawValidatedRevision(const FiniteExecutionValidationLineage3D& lineage) noexcept {
  const auto* const raw_lineage =
      std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(&lineage);
  return raw_lineage == nullptr ? 0U : raw_lineage->validated_through_raw_revision;
}

[[nodiscard]] bool finiteExecutionValidatedAgainstNewerRawWorld(
    const FiniteExecutionState3D& execution) noexcept {
  const auto* const raw_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&execution.certificate);
  const auto* const raw_lineage =
      std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
          &execution.validation_proof.lineage);
  return raw_certificate != nullptr && raw_lineage != nullptr &&
         raw_lineage->validated_through_raw_revision >
             raw_certificate->validated_through_revision;
}

[[nodiscard]] bool
rawInvalidationProofMatchesEvent(const FiniteExecutionState3D& execution,
                                 const RouteLifecycleEvent3D& event) noexcept {
  const auto* const raw_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&execution.certificate);
  const auto* const raw_lineage =
      std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
          &execution.validation_proof.lineage);
  return event.kind == RouteLifecycleEventKind3D::kRawInvalidated &&
         !event.latest_lidar_evidence.valid() && raw_certificate != nullptr &&
         raw_lineage != nullptr && execution.observed_raw_world != nullptr &&
         raw_certificate->producer_instance_id == event.raw_producer_instance_id &&
         raw_lineage->producer_instance_id == event.raw_producer_instance_id &&
         raw_lineage->validated_through_raw_revision == event.raw_revision &&
         execution.observed_raw_world->version().producer_instance_id ==
             event.raw_producer_instance_id &&
         execution.observed_raw_world->version().revision == event.raw_revision &&
         execution.observed_raw_world->contentFingerprint() ==
             raw_lineage->observed_world_content_fingerprint;
}

[[nodiscard]] bool
latestLidarInvalidationProofMatchesEvent(const FiniteExecutionState3D& execution,
                                         const RouteLifecycleEvent3D& event) noexcept {
  return event.kind == RouteLifecycleEventKind3D::kLatestLidarInvalidated &&
         event.raw_producer_instance_id == 0U && event.raw_revision == 0U &&
         event.latest_lidar_evidence.valid() &&
         execution.latest_lidar_evidence != nullptr &&
         execution.latest_lidar_evidence->evidenceId() == event.latest_lidar_evidence;
}

[[nodiscard]] bool
terminalStopBoundaryValid(const CertifiedStopBoundary3D& boundary,
                          const FiniteExecutionState3D& execution) noexcept {
  if (boundary.route_instance_id != execution.source_route_instance_id ||
      boundary.route_generation != execution.source_route_generation ||
      boundary.geometry_revision != execution.source_geometry_revision ||
      boundary.physical_route_fingerprint !=
          execution.source_physical_route_fingerprint ||
      boundary.trajectory_revision != execution.trajectory_revision ||
      boundary.raw_validated_through_revision !=
          rawValidatedRevision(execution.validation_proof.lineage) ||
      !std::isfinite(boundary.station_m) ||
      boundary.station_m + kStationToleranceM < execution.begin_route_station_m ||
      !std::isfinite(boundary.position_tolerance_m) ||
      boundary.position_tolerance_m <= 0.0 || !finitePoint(boundary.position) ||
      execution.horizon == nullptr || execution.horizon->states.empty()) {
    return false;
  }
  const CertificateView3D certificate = certificateView(execution.certificate);
  if (boundary.station_m + kStationToleranceM < certificate.suffix_start_station_m ||
      boundary.station_m > certificate.certified_end_station_m + kStationToleranceM) {
    return false;
  }
  const MotionState3D& terminal = execution.horizon->states.back();
  return distance3D(boundary.position, Point3{terminal.x, terminal.y, terminal.z}) <=
         boundary.position_tolerance_m;
}

} // namespace drone_city_nav::execution_route_snapshot_3d_internal
