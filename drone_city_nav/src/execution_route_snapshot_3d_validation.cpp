#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"

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

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finiteVector(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] bool finiteState(const mppi::State& state) noexcept {
  return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.z) &&
         std::isfinite(state.vx) && std::isfinite(state.vy) &&
         std::isfinite(state.vz) && std::isfinite(state.yaw) &&
         std::isfinite(state.yaw_rate);
}

[[nodiscard]] bool finiteControl(const mppi::Control& control) noexcept {
  return std::isfinite(control.ax) && std::isfinite(control.ay) &&
         std::isfinite(control.az) && std::isfinite(control.yaw_accel);
}

[[nodiscard]] bool knownFiniteExecutionKind(const FiniteExecutionKind3D kind) noexcept {
  switch (kind) {
    case FiniteExecutionKind3D::kNominal:
    case FiniteExecutionKind3D::kRetained:
    case FiniteExecutionKind3D::kEmergencyBrakeTail:
      return true;
  }
  return false;
}

[[nodiscard]] bool
knownRouteLifecycleEventKind(const RouteLifecycleEventKind3D kind) noexcept {
  switch (kind) {
    case RouteLifecycleEventKind3D::kCompleted:
    case RouteLifecycleEventKind3D::kRawInvalidated:
    case RouteLifecycleEventKind3D::kLatestLidarInvalidated:
    case RouteLifecycleEventKind3D::kObjectiveSuperseded:
    case RouteLifecycleEventKind3D::kControlCandidateRejected:
    case RouteLifecycleEventKind3D::kCrossTrackExceeded:
    case RouteLifecycleEventKind3D::kTrackingTubeExceeded:
      return true;
  }
  return false;
}

[[nodiscard]] bool nearlyEqual(const double first, const double second,
                               const double tolerance) noexcept {
  return std::isfinite(first) && std::isfinite(second) &&
         std::abs(first - second) <=
             tolerance * std::max({1.0, std::abs(first), std::abs(second)});
}

[[nodiscard]] bool
sameWorldCertificate(const NavigationWorldCertificate3D& first,
                     const NavigationWorldCertificate3D& second) noexcept {
  return first.producer_instance_id == second.producer_instance_id &&
         first.esdf_fingerprint == second.esdf_fingerprint &&
         first.esdf_source_raw_revision == second.esdf_source_raw_revision &&
         first.esdf_source_occupied_fingerprint ==
             second.esdf_source_occupied_fingerprint &&
         first.raw_validated_through_revision ==
             second.raw_validated_through_revision &&
         first.local_world_generation == second.local_world_generation &&
         first.topology_revision == second.topology_revision;
}

[[nodiscard]] bool
staticWorldNotOlder(const VersionedStaticWorld3D& candidate,
                    const VersionedStaticWorld3D& previous) noexcept {
  const NavigationWorldCertificate3D& next = candidate.certificate();
  const NavigationWorldCertificate3D& old = previous.certificate();
  if (!candidate.valid() || !previous.valid() ||
      candidate.contentFingerprint() != previous.contentFingerprint() ||
      next.producer_instance_id != old.producer_instance_id ||
      next.esdf_source_raw_revision < old.esdf_source_raw_revision ||
      next.raw_validated_through_revision < old.raw_validated_through_revision ||
      next.local_world_generation < old.local_world_generation ||
      next.topology_revision < old.topology_revision) {
    return false;
  }
  const bool advanced =
      next.esdf_source_raw_revision > old.esdf_source_raw_revision ||
      next.raw_validated_through_revision > old.raw_validated_through_revision ||
      next.local_world_generation > old.local_world_generation ||
      next.topology_revision > old.topology_revision;
  return advanced || sameWorldCertificate(next, old);
}

[[nodiscard]] bool
observedRawLineage(const NavigationWorldCertificate3D& certificate) noexcept {
  return certificate.producer_instance_id != 0U ||
         certificate.esdf_source_raw_revision != 0U ||
         certificate.raw_validated_through_revision != 0U;
}

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t byte = 0U; byte < sizeof(value); ++byte) {
    hash ^= (value >> (byte * 8U)) & 0xffU;
    hash *= kFnvPrime;
  }
}

[[nodiscard]] std::uint64_t canonicalDoubleBits(const double value) noexcept {
  return value == 0.0 ? 0U : std::bit_cast<std::uint64_t>(value);
}

void hashPoint(std::uint64_t& hash, const Point3& point) noexcept {
  hashValue(hash, canonicalDoubleBits(point.x));
  hashValue(hash, canonicalDoubleBits(point.y));
  hashValue(hash, canonicalDoubleBits(point.z));
}

void hashAxis(std::uint64_t& hash, const FootprintBodyAxis& axis) noexcept {
  hashValue(hash, canonicalDoubleBits(axis.x));
  hashValue(hash, canonicalDoubleBits(axis.y));
  hashValue(hash, canonicalDoubleBits(axis.z));
}

[[nodiscard]] bool footprintValid(const SweptFootprintConfig& footprint) noexcept {
  return std::isfinite(footprint.radius_m) && footprint.radius_m >= 0.0 &&
         std::isfinite(footprint.lower_extent_m) && footprint.lower_extent_m >= 0.0 &&
         std::isfinite(footprint.upper_extent_m) && footprint.upper_extent_m >= 0.0 &&
         std::isfinite(footprint.sweep_step_m) && footprint.sweep_step_m > 0.0 &&
         std::isfinite(footprint.safe_clearance_threshold_m) &&
         footprint.safe_clearance_threshold_m >= 0.0 && footprint.axial_samples != 0U;
}

[[nodiscard]] bool sameFootprintConfig(const SweptFootprintConfig& first,
                                       const SweptFootprintConfig& second) noexcept {
  return first.radius_m == second.radius_m &&
         first.lower_extent_m == second.lower_extent_m &&
         first.upper_extent_m == second.upper_extent_m &&
         first.perimeter_samples == second.perimeter_samples &&
         first.radial_rings == second.radial_rings &&
         first.axial_samples == second.axial_samples &&
         first.sweep_step_m == second.sweep_step_m &&
         first.safe_clearance_threshold_m == second.safe_clearance_threshold_m;
}

[[nodiscard]] bool
footprintConservativelyContains(const SweptFootprintConfig& outer,
                                const SweptFootprintConfig& inner) noexcept {
  return footprintValid(outer) && footprintValid(inner) &&
         outer.radius_m + kGeometryTolerance >= inner.radius_m &&
         outer.lower_extent_m + kGeometryTolerance >= inner.lower_extent_m &&
         outer.upper_extent_m + kGeometryTolerance >= inner.upper_extent_m &&
         outer.perimeter_samples >= inner.perimeter_samples &&
         outer.radial_rings >= inner.radial_rings &&
         outer.axial_samples >= inner.axial_samples &&
         outer.sweep_step_m <= inner.sweep_step_m + kGeometryTolerance &&
         outer.safe_clearance_threshold_m + kGeometryTolerance >=
             inner.safe_clearance_threshold_m;
}

[[nodiscard]] bool
sameFreeSpaceSeed(const ProprioceptiveFreeSpaceSeed3D& first,
                  const ProprioceptiveFreeSpaceSeed3D& second) noexcept {
  return first.position.x == second.position.x &&
         first.position.y == second.position.y &&
         first.position.z == second.position.z &&
         first.body_axis.x == second.body_axis.x &&
         first.body_axis.y == second.body_axis.y &&
         first.body_axis.z == second.body_axis.z &&
         sameFootprintConfig(first.footprint, second.footprint);
}

[[nodiscard]] bool sameAxisAlignedBox(const AxisAlignedBox3D& first,
                                      const AxisAlignedBox3D& second) noexcept {
  return first.minimum.x == second.minimum.x && first.minimum.y == second.minimum.y &&
         first.minimum.z == second.minimum.z && first.maximum.x == second.maximum.x &&
         first.maximum.y == second.maximum.y && first.maximum.z == second.maximum.z;
}

[[nodiscard]] bool
sameCanonicalLaunchSupport(const LaunchSupportContact3D& candidate,
                           const LaunchSupportContact3D& canonical) noexcept {
  return launchSupportContactValid3D(candidate) &&
         launchSupportContactValid3D(canonical) &&
         sameFreeSpaceSeed(candidate.seed, canonical.seed) &&
         candidate.contact_cells.size() == canonical.contact_cells.size() &&
         std::ranges::equal(candidate.contact_cells, canonical.contact_cells,
                            sameAxisAlignedBox) &&
         candidate.occupied_evidence_cells == canonical.occupied_evidence_cells &&
         candidate.evidence_source == canonical.evidence_source &&
         candidate.maximum_lateral_departure_m ==
             canonical.maximum_lateral_departure_m &&
         candidate.maximum_axial_settling_m == canonical.maximum_axial_settling_m;
}

[[nodiscard]] bool
launchSupportMatchesOwnedOccupancy(const LaunchSupportContact3D& support,
                                   const ObservedOccupancyGrid3D& occupancy) {
  if (!launchSupportContactValid3D(support)) {
    return false;
  }
  if (support.evidence_source == LaunchSupportEvidenceSource::kObservedOccupancy) {
    const std::optional<LaunchSupportContact3D> canonical =
        detectLaunchSupportContact3D(occupancy, support.seed);
    return canonical.has_value() && sameCanonicalLaunchSupport(support, *canonical);
  }
  const LaunchSupportContact3D canonical =
      makeVehicleLandedSupportContact3D(occupancy.bounds(), support.seed);
  return sameCanonicalLaunchSupport(support, canonical);
}

void hashFootprint(std::uint64_t& hash,
                   const SweptFootprintConfig& footprint) noexcept {
  hashValue(hash, canonicalDoubleBits(footprint.radius_m));
  hashValue(hash, canonicalDoubleBits(footprint.lower_extent_m));
  hashValue(hash, canonicalDoubleBits(footprint.upper_extent_m));
  hashValue(hash, static_cast<std::uint64_t>(footprint.perimeter_samples));
  hashValue(hash, static_cast<std::uint64_t>(footprint.radial_rings));
  hashValue(hash, static_cast<std::uint64_t>(footprint.axial_samples));
  hashValue(hash, canonicalDoubleBits(footprint.sweep_step_m));
  hashValue(hash, canonicalDoubleBits(footprint.safe_clearance_threshold_m));
}

[[nodiscard]] bool hashProprioceptiveFreeSpaceSeed(
    std::uint64_t& hash,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed) noexcept {
  hashValue(hash, free_space_seed == nullptr ? 0U : 1U);
  if (free_space_seed == nullptr) {
    return true;
  }
  const double axis_norm =
      std::hypot(std::hypot(free_space_seed->body_axis.x, free_space_seed->body_axis.y),
                 free_space_seed->body_axis.z);
  if (!finitePoint(free_space_seed->position) || !std::isfinite(axis_norm) ||
      std::abs(axis_norm - 1.0) > 1.0e-6 ||
      !std::isfinite(free_space_seed->body_axis.x) ||
      !std::isfinite(free_space_seed->body_axis.y) ||
      !std::isfinite(free_space_seed->body_axis.z) ||
      !footprintValid(free_space_seed->footprint)) {
    return false;
  }
  hashPoint(hash, free_space_seed->position);
  hashAxis(hash, free_space_seed->body_axis);
  hashFootprint(hash, free_space_seed->footprint);
  return true;
}

[[nodiscard]] bool hashLaunchSupportContact(
    std::uint64_t& hash,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  hashValue(hash, launch_support_contact == nullptr ? 0U : 1U);
  if (launch_support_contact == nullptr) {
    return true;
  }
  const LaunchSupportContact3D& support = *launch_support_contact;
  if (!launchSupportContactValid3D(support)) {
    return false;
  }
  hashPoint(hash, support.seed.position);
  hashAxis(hash, support.seed.body_axis);
  hashFootprint(hash, support.seed.footprint);
  hashValue(hash, static_cast<std::uint64_t>(support.contact_cells.size()));
  for (const AxisAlignedBox3D& cell : support.contact_cells) {
    if (!finitePoint(cell.minimum) || !finitePoint(cell.maximum)) {
      return false;
    }
    hashPoint(hash, cell.minimum);
    hashPoint(hash, cell.maximum);
  }
  hashValue(hash, static_cast<std::uint64_t>(support.occupied_evidence_cells));
  hashValue(hash, static_cast<std::uint64_t>(support.evidence_source));
  hashValue(hash, canonicalDoubleBits(support.maximum_lateral_departure_m));
  hashValue(hash, canonicalDoubleBits(support.minimum_axial_departure_m));
  hashValue(hash, canonicalDoubleBits(support.maximum_axial_settling_m));
  return true;
}

[[nodiscard]] std::uint64_t validationPolicyFingerprint(
    const SweptFootprintConfig& footprint,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  if (!footprintValid(footprint)) {
    return 0U;
  }

  std::uint64_t hash{kFnvOffset};
  hashFootprint(hash, footprint);
  if (!hashLaunchSupportContact(hash, launch_support_contact)) {
    return 0U;
  }
  return hash == 0U ? 1U : hash;
}

void hashGridBounds(std::uint64_t& hash, const GridBounds3D& bounds) noexcept {
  hashValue(hash, canonicalDoubleBits(bounds.origin_x));
  hashValue(hash, canonicalDoubleBits(bounds.origin_y));
  hashValue(hash, canonicalDoubleBits(bounds.origin_z));
  hashValue(hash, canonicalDoubleBits(bounds.resolution_m));
  hashValue(hash,
            static_cast<std::uint64_t>(static_cast<std::int64_t>(bounds.width_cells)));
  hashValue(hash,
            static_cast<std::uint64_t>(static_cast<std::int64_t>(bounds.height_cells)));
  hashValue(hash,
            static_cast<std::uint64_t>(static_cast<std::int64_t>(bounds.depth_cells)));
}

void hashObservedOccupancy(std::uint64_t& hash,
                           const ObservedOccupancyGrid3D& occupancy) {
  hashGridBounds(hash, occupancy.bounds());
  std::vector<std::pair<OccupancyChunkIndex3D, const ObservedOccupancyChunk3D*>> chunks;
  chunks.reserve(occupancy.chunks().size());
  for (const auto& [index, storage] : occupancy.chunks()) {
    chunks.emplace_back(index, &storage.get());
  }
  std::ranges::sort(chunks, {}, [](const auto& entry) {
    return std::tuple{entry.first.x, entry.first.y, entry.first.z};
  });
  hashValue(hash, static_cast<std::uint64_t>(chunks.size()));
  for (const auto& [index, chunk] : chunks) {
    hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(index.x)));
    hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(index.y)));
    hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(index.z)));
    for (const std::uint64_t word : chunk->observed) {
      hashValue(hash, word);
    }
    for (const std::uint64_t word : chunk->occupied) {
      hashValue(hash, word);
    }
  }
}

[[nodiscard]] std::uint64_t
observedOccupancyContentFingerprint(const ObservedOccupancyGrid3D& occupancy) {
  std::uint64_t hash{kFnvOffset};
  hashObservedOccupancy(hash, occupancy);
  return hash;
}

[[nodiscard]] std::uint64_t observedWorldContentFingerprintFromObservation(
    const std::uint64_t observation_content_fingerprint,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
    const LaunchSupportContact3D* const launch_support_contact) {
  std::uint64_t hash{observation_content_fingerprint};
  if (!hashProprioceptiveFreeSpaceSeed(hash, free_space_seed) ||
      !hashLaunchSupportContact(hash, launch_support_contact)) {
    return 0U;
  }
  return hash == 0U ? 1U : hash;
}

void hashValidationTerminalBoundary(
    std::uint64_t& hash,
    const std::optional<mppi::FiniteExecutionPathTerminalBoundary>& boundary) {
  hashValue(hash, boundary.has_value() ? 1U : 0U);
  if (!boundary.has_value()) {
    return;
  }
  hashPoint(hash, boundary->endpoint);
  hashValue(hash, canonicalDoubleBits(boundary->forward.x));
  hashValue(hash, canonicalDoubleBits(boundary->forward.y));
  hashValue(hash, canonicalDoubleBits(boundary->forward.z));
  hashValue(hash, canonicalDoubleBits(boundary->tolerance_m));
  hashValue(hash, canonicalDoubleBits(boundary->activation_distance_m));
  hashValue(hash, canonicalDoubleBits(boundary->maximum_cross_track_m));
  hashValue(hash, canonicalDoubleBits(boundary->initial_route_station_m));
  hashValue(hash, canonicalDoubleBits(boundary->activation_route_station_m));
  hashValue(hash, static_cast<std::uint64_t>(boundary->activation_route.size()));
  for (const mppi::RouteSample3D& sample : boundary->activation_route) {
    hashValue(hash, canonicalDoubleBits(sample.x_m));
    hashValue(hash, canonicalDoubleBits(sample.y_m));
    hashValue(hash, canonicalDoubleBits(sample.z_m));
    hashValue(hash, canonicalDoubleBits(sample.tangent_x));
    hashValue(hash, canonicalDoubleBits(sample.tangent_y));
    hashValue(hash, canonicalDoubleBits(sample.tangent_z));
    hashValue(hash, canonicalDoubleBits(sample.station_m));
  }
}

// Owners authenticate immutable content at capture. Exact view identity lets
// certification bind that content without walking occupancy chunks again.
[[nodiscard]] std::optional<ValidationWorldOwnerContent3D>
validationWorldOwnerContent(const mppi::FiniteExecutionPathWorld& world,
                            const ValidationContractOwners3D& owners) noexcept {
  if (world.raw_occupancy != nullptr ||
      (owners.observed_raw_world == nullptr) == (owners.static_world == nullptr)) {
    return std::nullopt;
  }
  if (owners.observed_raw_world != nullptr) {
    const VersionedObservedRawWorld3D& owner = *owners.observed_raw_world;
    const LaunchSupportContact3D* const owned_support =
        owner.launchSupportContact().has_value()
            ? std::addressof(*owner.launchSupportContact())
            : nullptr;
    const std::uint64_t content_fingerprint = owner.contentFingerprint();
    if (!owner.valid() || content_fingerprint == 0U ||
        world.static_occupancy != nullptr ||
        world.observed_occupancy != std::addressof(owner.occupancy()) ||
        world.launch_support_contact != owned_support) {
      return std::nullopt;
    }
    return ValidationWorldOwnerContent3D{
        .kind = 2U,
        .content_fingerprint = content_fingerprint,
    };
  }

  const VersionedStaticWorld3D& owner = *owners.static_world;
  const std::uint64_t content_fingerprint = owner.contentFingerprint();
  if (!owner.valid() || content_fingerprint == 0U ||
      world.static_occupancy != std::addressof(owner.occupancy()) ||
      world.observed_occupancy != nullptr || world.launch_support_contact != nullptr) {
    return std::nullopt;
  }
  return ValidationWorldOwnerContent3D{
      .kind = 1U,
      .content_fingerprint = content_fingerprint,
  };
}

// Lidar content is likewise authenticated once. The span must be the complete
// owner-backed point array before its cached fingerprint can enter the proof.
[[nodiscard]] std::uint64_t validationLidarOwnerContentFingerprint(
    const std::span<const Point3> points,
    const VersionedLatestLidarEvidence3D* const owner) noexcept {
  if (owner == nullptr) {
    return 0U;
  }
  const std::vector<Point3>& owned_points = owner->hitPointsMapM();
  const std::uint64_t content_fingerprint = owner->contentFingerprint();
  if (content_fingerprint == 0U || points.size() != owned_points.size() ||
      points.data() != owned_points.data()) {
    return 0U;
  }
  return content_fingerprint;
}

[[nodiscard]] std::uint64_t
validationContractFingerprint(const mppi::FiniteExecutionPathWorld& world,
                              const mppi::Control& previous_applied_control,
                              const ValidationContractOwners3D& owners) {
  if (world.flight_envelope == nullptr || world.dynamics == nullptr ||
      world.altitude_envelope == nullptr || world.footprint == nullptr ||
      !finiteControl(previous_applied_control)) {
    return 0U;
  }

  std::uint64_t hash{kFnvOffset};
  hashValue(hash, canonicalDoubleBits(world.flight_envelope->minimum_target_z_m));
  hashValue(hash, canonicalDoubleBits(world.flight_envelope->maximum_target_z_m));
  const mppi::DynamicsConfig& dynamics = *world.dynamics;
  hashValue(hash, canonicalDoubleBits(dynamics.dt_s));
  hashValue(hash, canonicalDoubleBits(dynamics.linear_drag_1ps));
  hashValue(hash, canonicalDoubleBits(dynamics.maximum_horizontal_acceleration_mps2));
  hashValue(hash, canonicalDoubleBits(dynamics.maximum_vertical_acceleration_mps2));
  hashValue(hash, canonicalDoubleBits(dynamics.maximum_horizontal_speed_mps));
  hashValue(hash, canonicalDoubleBits(dynamics.maximum_vertical_speed_mps));
  hashValue(hash, canonicalDoubleBits(dynamics.maximum_translational_speed_mps));
  hashValue(hash, canonicalDoubleBits(dynamics.maximum_yaw_acceleration_radps2));
  hashValue(hash, canonicalDoubleBits(dynamics.maximum_yaw_rate_radps));
  hashValue(hash, canonicalDoubleBits(dynamics.maximum_control_jerk_mps3));
  const mppi::AltitudeEnvelopeConfig& altitude = *world.altitude_envelope;
  hashValue(hash, canonicalDoubleBits(altitude.minimum_z_m));
  hashValue(hash, canonicalDoubleBits(altitude.maximum_z_m));
  hashValue(hash, canonicalDoubleBits(altitude.guaranteed_vertical_deceleration_mps2));
  hashValue(hash, canonicalDoubleBits(altitude.reaction_latency_s));
  const std::uint64_t policy_fingerprint =
      validationPolicyFingerprint(*world.footprint, world.launch_support_contact);
  if (policy_fingerprint == 0U) {
    return 0U;
  }
  hashValue(hash, policy_fingerprint);
  hashValue(hash, canonicalDoubleBits(previous_applied_control.ax));
  hashValue(hash, canonicalDoubleBits(previous_applied_control.ay));
  hashValue(hash, canonicalDoubleBits(previous_applied_control.az));
  hashValue(hash, canonicalDoubleBits(previous_applied_control.yaw_accel));

  const std::optional<ValidationWorldOwnerContent3D> world_content =
      validationWorldOwnerContent(world, owners);
  if (!world_content.has_value()) {
    return 0U;
  }
  hashValue(hash, world_content->kind);
  hashValue(hash, world_content->content_fingerprint);

  const std::uint64_t lidar_content_fingerprint =
      validationLidarOwnerContentFingerprint(world.latest_lidar_obstacle_points,
                                             owners.latest_lidar_evidence);
  if (lidar_content_fingerprint == 0U) {
    return 0U;
  }
  hashValue(hash, lidar_content_fingerprint);
  hashValidationTerminalBoundary(hash, world.terminal_boundary);
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] std::optional<FiniteRouteTerminalBoundary3D>
canonicalFiniteRouteTerminalBoundary(const CertifiedRouteSuffix3D& route,
                                     const double initial_station_m) noexcept {
  if (route.geometry == nullptr || route.geometry->route == nullptr ||
      route.geometry->route->size() < 2U || !std::isfinite(initial_station_m) ||
      initial_station_m < 0.0 ||
      initial_station_m > route.endStationM() + kStationToleranceM) {
    return std::nullopt;
  }
  const std::span<const RouteSample3D> samples{*route.geometry->route};
  const RouteSample3D& previous = samples[samples.size() - 2U];
  const RouteSample3D& endpoint = samples.back();
  const Vec3 forward{endpoint.position.x - previous.position.x,
                     endpoint.position.y - previous.position.y,
                     endpoint.position.z - previous.position.z};
  if (std::hypot(std::hypot(forward.x, forward.y), forward.z) <= kStationToleranceM) {
    return std::nullopt;
  }
  return FiniteRouteTerminalBoundary3D{
      .endpoint = endpoint.position,
      .forward = forward,
      .tolerance_m = kTerminalBoundaryToleranceM,
      .activation_distance_m = kTerminalBoundaryActivationDistanceM,
      .maximum_cross_track_m = kMaximumRouteCrossTrackM,
      .initial_route_station_m = initial_station_m,
      .activation_route_station_m = std::max(initial_station_m, previous.station_m),
  };
}

std::optional<mppi::FiniteExecutionPathTerminalBoundary> makeValidationTerminalBoundary(
    const std::optional<FiniteRouteTerminalBoundary3D>& boundary,
    const CertifiedRouteSuffix3D& route,
    const std::span<const mppi::RouteSample3D> mppi_reference) {
  if (!boundary.has_value() || route.geometry == nullptr ||
      mppi_reference.size() != route.geometry->route->size()) {
    return std::nullopt;
  }
  return mppi::FiniteExecutionPathTerminalBoundary{
      .endpoint = boundary->endpoint,
      .forward = boundary->forward,
      .tolerance_m = boundary->tolerance_m,
      .activation_distance_m = boundary->activation_distance_m,
      .maximum_cross_track_m = boundary->maximum_cross_track_m,
      .activation_route = mppi_reference,
      .initial_route_station_m = static_cast<float>(boundary->initial_route_station_m),
      .activation_route_station_m =
          static_cast<float>(boundary->activation_route_station_m),
  };
}

[[nodiscard]] bool sameTerminalBoundary(
    const std::optional<FiniteRouteTerminalBoundary3D>& first,
    const std::optional<FiniteRouteTerminalBoundary3D>& second) noexcept {
  if (first.has_value() != second.has_value()) {
    return false;
  }
  return !first.has_value() ||
         (first->endpoint.x == second->endpoint.x &&
          first->endpoint.y == second->endpoint.y &&
          first->endpoint.z == second->endpoint.z &&
          first->forward.x == second->forward.x &&
          first->forward.y == second->forward.y &&
          first->forward.z == second->forward.z &&
          first->tolerance_m == second->tolerance_m &&
          first->activation_distance_m == second->activation_distance_m &&
          first->maximum_cross_track_m == second->maximum_cross_track_m &&
          first->initial_route_station_m == second->initial_route_station_m &&
          first->activation_route_station_m == second->activation_route_station_m);
}

[[nodiscard]] bool
latestLidarEvidenceFreshAt(const VersionedLatestLidarEvidence3D& evidence,
                           const VersionedExecutionValidationPolicy3D& policy,
                           const std::int64_t validation_stamp_ns) noexcept {
  constexpr double kNanosecondsPerMillisecond{1.0e6};
  if (!evidence.valid() || validation_stamp_ns <= 0) {
    return false;
  }
  if (!policy.latestLidarFreshnessRequired()) {
    return true;
  }
  const double maximum_age_ns =
      policy.latestLidarMaximumAgeMs() * kNanosecondsPerMillisecond;
  const std::int64_t acquisition_age_ns =
      validation_stamp_ns - evidence.acquisitionStampNs();
  const std::int64_t receive_age_ns = validation_stamp_ns - evidence.receiveStampNs();
  return receive_age_ns >= 0 && static_cast<double>(receive_age_ns) <= maximum_age_ns &&
         (acquisition_age_ns < 0 ||
          static_cast<double>(acquisition_age_ns) <= maximum_age_ns);
}

[[nodiscard]] std::vector<mppi::TimedExecutionPathPoint>
timedExecutionPathPoints(const mppi::FiniteHorizon& horizon,
                         const mppi::Control& previous_applied_control,
                         const std::int64_t control_interval_ns) {
  std::vector<mppi::TimedExecutionPathPoint> points;
  if (control_interval_ns <= 0 || horizon.controls.empty() ||
      horizon.states.size() != horizon.controls.size() + 1U ||
      !executionHorizonTerminalOffsetNs(horizon.states.size(), control_interval_ns)) {
    return points;
  }
  points.reserve(horizon.states.size());
  for (std::size_t index = 0U; index < horizon.states.size(); ++index) {
    points.push_back(mppi::TimedExecutionPathPoint{
        .time_from_start_s = static_cast<double>(static_cast<std::int64_t>(index) *
                                                 control_interval_ns) /
                             1'000'000'000.0,
        .state = horizon.states[index],
        .control =
            index == 0U ? previous_applied_control : horizon.controls[index - 1U],
    });
  }
  return points;
}

[[nodiscard]] bool finiteStateNearlyEqual(const mppi::State& first,
                                          const mppi::State& second) noexcept {
  constexpr float kDynamicsTolerance{2.0e-3F};
  const auto close = [](const float left, const float right) noexcept {
    return std::isfinite(left) && std::isfinite(right) &&
           std::abs(left - right) <= kDynamicsTolerance;
  };
  return close(first.x, second.x) && close(first.y, second.y) &&
         close(first.z, second.z) && close(first.vx, second.vx) &&
         close(first.vy, second.vy) && close(first.vz, second.vz) &&
         std::abs(std::remainder(first.yaw - second.yaw,
                                 2.0F * std::numbers::pi_v<float>)) <=
             kDynamicsTolerance &&
         close(first.yaw_rate, second.yaw_rate);
}

[[nodiscard]] bool
finiteHorizonDynamicallyConsistent(const mppi::FiniteHorizon& horizon,
                                   const mppi::Control& previous_applied_control,
                                   const mppi::DynamicsConfig& dynamics) noexcept {
  if (horizon.controls.empty() ||
      horizon.states.size() != horizon.controls.size() + 1U ||
      !finiteControl(previous_applied_control) || !std::isfinite(dynamics.dt_s) ||
      dynamics.dt_s <= 0.0F ||
      !std::isfinite(dynamics.maximum_horizontal_acceleration_mps2) ||
      dynamics.maximum_horizontal_acceleration_mps2 <= 0.0F ||
      !std::isfinite(dynamics.maximum_vertical_acceleration_mps2) ||
      dynamics.maximum_vertical_acceleration_mps2 <= 0.0F ||
      !std::isfinite(dynamics.maximum_yaw_acceleration_radps2) ||
      dynamics.maximum_yaw_acceleration_radps2 <= 0.0F ||
      !std::isfinite(dynamics.maximum_control_jerk_mps3) ||
      dynamics.maximum_control_jerk_mps3 <= 0.0F) {
    return false;
  }

  constexpr float kControlTolerance{1.0e-4F};
  const float maximum_control_delta =
      dynamics.maximum_control_jerk_mps3 * dynamics.dt_s;
  mppi::Control previous_control = previous_applied_control;
  for (std::size_t index = 0U; index < horizon.controls.size(); ++index) {
    const mppi::Control& control = horizon.controls[index];
    if (!finiteControl(control) ||
        std::hypot(control.ax, control.ay) >
            dynamics.maximum_horizontal_acceleration_mps2 + kControlTolerance ||
        std::abs(control.az) >
            dynamics.maximum_vertical_acceleration_mps2 + kControlTolerance ||
        std::abs(control.yaw_accel) >
            dynamics.maximum_yaw_acceleration_radps2 + kControlTolerance ||
        std::abs(control.ax - previous_control.ax) >
            maximum_control_delta + kControlTolerance ||
        std::abs(control.ay - previous_control.ay) >
            maximum_control_delta + kControlTolerance ||
        std::abs(control.az - previous_control.az) >
            maximum_control_delta + kControlTolerance) {
      return false;
    }
    const mppi::State expected =
        mppi::integrateReference(horizon.states[index], control, dynamics);
    if (!finiteStateNearlyEqual(expected, horizon.states[index + 1U])) {
      return false;
    }
    previous_control = control;
  }
  return true;
}

} // namespace drone_city_nav::execution_route_snapshot_3d_internal
