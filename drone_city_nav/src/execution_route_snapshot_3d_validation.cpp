#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_certification_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"
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

[[nodiscard]] bool finiteState(const MotionState3D& state) noexcept {
  return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.z) &&
         std::isfinite(state.vx) && std::isfinite(state.vy) &&
         std::isfinite(state.vz) && std::isfinite(state.yaw) &&
         std::isfinite(state.yaw_rate);
}

[[nodiscard]] bool finiteControl(const MotionControl3D& control) noexcept {
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

void hashValidationTerminalBoundary(
    std::uint64_t& hash,
    const std::optional<FiniteExecutionPathTerminalBoundary3D>& boundary) {
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
  for (const ControlRouteSample3D& sample : boundary->activation_route) {
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
validationWorldOwnerContent(const FiniteExecutionPathWorld3D& world,
                            const ValidationContractOwners3D& owners) noexcept {
  if (world.raw_occupancy != nullptr ||
      (owners.observed_raw_world == nullptr) == (owners.static_world == nullptr)) {
    return std::nullopt;
  }
  if (owners.observed_raw_world != nullptr) {
    const VersionedObservedRawWorld3D& owner = *owners.observed_raw_world;
    const std::optional<LaunchSupportContact3D>& launch_support =
        owner.launchSupportContact();
    const LaunchSupportContact3D* const owned_support =
        launch_support.has_value() ? std::addressof(*launch_support) : nullptr;
    const std::uint64_t content_fingerprint = owner.contentFingerprint();
    // The proprioceptive seed is transient evidence like the newest lidar
    // returns: it is the vehicle's pose at validation time, never part of the
    // owner's immutable content.
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
validationContractFingerprint(const FiniteExecutionPathWorld3D& world,
                              const MotionControl3D& previous_applied_control,
                              const ValidationContractOwners3D& owners) {
  if (world.flight_envelope == nullptr || world.dynamics == nullptr ||
      world.altitude_envelope == nullptr || world.footprint == nullptr ||
      !finiteControl(previous_applied_control)) {
    return 0U;
  }

  std::uint64_t hash{kFnvOffset};
  hashValue(hash, canonicalDoubleBits(world.flight_envelope->minimum_target_z_m));
  hashValue(hash, canonicalDoubleBits(world.flight_envelope->maximum_target_z_m));
  const MotionDynamicsConfig3D& dynamics = *world.dynamics;
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
  const MotionAltitudeEnvelopeConfig3D& altitude = *world.altitude_envelope;
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

std::optional<FiniteExecutionPathTerminalBoundary3D> makeValidationTerminalBoundary(
    const std::optional<FiniteRouteTerminalBoundary3D>& boundary,
    const CertifiedRouteSuffix3D& route,
    const std::span<const ControlRouteSample3D> mppi_reference) {
  if (!boundary.has_value() || route.geometry == nullptr ||
      mppi_reference.size() != route.geometry->route->size()) {
    return std::nullopt;
  }
  return FiniteExecutionPathTerminalBoundary3D{
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

[[nodiscard]] std::vector<TimedExecutionPathPoint3D>
timedExecutionPathPoints(const FiniteMotionHorizon3D& horizon,
                         const MotionControl3D& previous_applied_control,
                         const std::int64_t control_interval_ns) {
  std::vector<TimedExecutionPathPoint3D> points;
  if (control_interval_ns <= 0 || horizon.controls.empty() ||
      horizon.states.size() != horizon.controls.size() + 1U ||
      !executionHorizonTerminalOffsetNs(horizon.states.size(), control_interval_ns)) {
    return points;
  }
  points.reserve(horizon.states.size());
  for (std::size_t index = 0U; index < horizon.states.size(); ++index) {
    points.push_back(TimedExecutionPathPoint3D{
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

[[nodiscard]] bool finiteStateNearlyEqual(const MotionState3D& first,
                                          const MotionState3D& second) noexcept {
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
finiteHorizonDynamicallyConsistent(const FiniteMotionHorizon3D& horizon,
                                   const MotionControl3D& previous_applied_control,
                                   const MotionDynamicsConfig3D& dynamics) noexcept {
  return finiteMotionHorizonDynamicsConsistency3D(horizon, previous_applied_control,
                                                  dynamics) ==
         MotionDynamicsConsistency3D::kConsistent;
}

} // namespace drone_city_nav::execution_route_snapshot_3d_internal
