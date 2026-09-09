#include "drone_city_nav/tracking_error_tube_3d.hpp"

#include "drone_city_nav/raw_occupancy_clearance_3d.hpp"
#include "drone_city_nav/tracking_error_tube_handoff_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr double kSpeedToleranceMps{1.0e-9};
constexpr double kExecutionTolerance{1.0e-6};
constexpr double kHandoffStationToleranceM{1.0e-6};

[[nodiscard]] bool nearlyEqual(const double first, const double second) noexcept {
  const double scale = std::max({1.0, std::abs(first), std::abs(second)});
  return std::abs(first - second) <= kExecutionTolerance * scale;
}

[[nodiscard]] bool
physicalFootprintIsValid(const SweptFootprintConfig& footprint) noexcept {
  return std::isfinite(footprint.radius_m) && footprint.radius_m >= 0.0 &&
         std::isfinite(footprint.body_radius_m) && footprint.body_radius_m >= 0.0 &&
         std::isfinite(footprint.body_lower_extent_m) &&
         footprint.body_lower_extent_m >= 0.0 &&
         std::isfinite(footprint.body_upper_extent_m) &&
         footprint.body_upper_extent_m >= 0.0 &&
         std::isfinite(footprint.lower_extent_m) && footprint.lower_extent_m >= 0.0 &&
         std::isfinite(footprint.upper_extent_m) && footprint.upper_extent_m >= 0.0 &&
         footprint.perimeter_samples >= 3U && footprint.radial_rings > 0U &&
         footprint.axial_samples > 0U && std::isfinite(footprint.sweep_step_m) &&
         footprint.sweep_step_m > 0.0 &&
         std::isfinite(footprint.safe_clearance_threshold_m) &&
         footprint.safe_clearance_threshold_m >= 0.0;
}

[[nodiscard]] bool
routeIsValidForTube(const std::span<const RouteSample3D> route) noexcept {
  if (route.size() < 2U) {
    return false;
  }
  double previous_station_m{-std::numeric_limits<double>::infinity()};
  Point3 previous_position{};
  for (std::size_t index = 0U; index < route.size(); ++index) {
    const RouteSample3D& sample = route[index];
    const double segment_length_m =
        index > 0U ? distance3D(previous_position, sample.position) : 0.0;
    if (!std::isfinite(sample.position.x) || !std::isfinite(sample.position.y) ||
        !std::isfinite(sample.position.z) || !std::isfinite(sample.station_m) ||
        sample.station_m < 0.0 || sample.station_m <= previous_station_m ||
        (index > 0U &&
         (!std::isfinite(segment_length_m) || !(segment_length_m > 0.0)))) {
      return false;
    }
    previous_station_m = sample.station_m;
    previous_position = sample.position;
  }
  return true;
}

[[nodiscard]] Point3 statePosition(const MotionState3D& state) noexcept {
  return Point3{state.x, state.y, state.z};
}

[[nodiscard]] double stateSpeedMps(const MotionState3D& state) noexcept {
  return std::hypot(
      std::hypot(static_cast<double>(state.vx), static_cast<double>(state.vy)),
      static_cast<double>(state.vz));
}

[[nodiscard]] MotionState3D interpolateState(const MotionState3D& first,
                                             const MotionState3D& second,
                                             const double ratio) noexcept {
  const auto interpolate = [ratio](const float start, const float stop) {
    return static_cast<float>(
        std::lerp(static_cast<double>(start), static_cast<double>(stop), ratio));
  };
  return MotionState3D{
      .x = interpolate(first.x, second.x),
      .y = interpolate(first.y, second.y),
      .z = interpolate(first.z, second.z),
      .vx = interpolate(first.vx, second.vx),
      .vy = interpolate(first.vy, second.vy),
      .vz = interpolate(first.vz, second.vz),
      .yaw = interpolate(first.yaw, second.yaw),
      .yaw_rate = interpolate(first.yaw_rate, second.yaw_rate),
  };
}

[[nodiscard]] bool
worldConfigurationIsValid(const TrackingErrorTubeWorld3D& world) noexcept {
  if (world.observed_occupancy != nullptr && world.occupancy != nullptr) {
    return false;
  }
  if (world.observed_occupancy == nullptr &&
      (world.launch_support_contact != nullptr ||
       world.proprioceptive_free_space_seed != nullptr)) {
    return false;
  }
  const bool evidence_available =
      world.observed_occupancy != nullptr || world.occupancy != nullptr;
  if (!evidence_available) {
    return world.occupied_content_fingerprint == 0U;
  }
  const std::uint64_t canonical_fingerprint =
      world.observed_occupancy != nullptr
          ? world.observed_occupancy->occupiedContentFingerprint()
          : world.occupancy->contentFingerprint();
  return canonical_fingerprint != 0U &&
         world.occupied_content_fingerprint == canonical_fingerprint;
}

[[nodiscard]] bool segmentAccepted(const TrackingErrorTubeWorld3D& world,
                                   const Point3& first, const Point3& second,
                                   const SweptFootprintConfig& footprint) noexcept {
  constexpr FootprintBodyAxis kBodyAxis{};
  const OccupiedCollisionOracle3D oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = world.observed_occupancy,
      .static_occupancy = world.occupancy,
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = world.launch_support_contact,
      .proprioceptive_free_space_seed = world.proprioceptive_free_space_seed,
      .footprint = footprint,
      .flight_envelope = std::nullopt,
  }};
  return oracle.validateSegment(first, kBodyAxis, second, kBodyAxis).clear();
}

// The largest uniform body inflation that clears raw occupied evidence along
// the whole segment: the minimum over body poses sampled at the sweep step,
// each an exact distance query against nearby occupied voxel boxes.
[[nodiscard]] double segmentInflationMarginM(
    const TrackingErrorTubeWorld3D& world, const Point3& first, const Point3& second,
    const SweptFootprintConfig& physical_footprint, const double maximum_body_tilt_rad,
    const double maximum_margin_m) {
  // The lean law: the margin is measured from the body enveloped over every
  // tilt the dynamics reach, so a segment the leaning body cannot clear gets
  // no margin and the progress floor for a speed.
  const SweptFootprintConfig leaning =
      tiltEnvelopedFootprint(physical_footprint, maximum_body_tilt_rad);
  const RawClearanceBody3D body{
      .radius_m = leaning.radius_m,
      .lower_extent_m = leaning.lower_extent_m,
      .upper_extent_m = leaning.upper_extent_m,
  };
  const double length_m = distance3D(first, second);
  const auto pose_count = static_cast<std::size_t>(
      std::ceil(length_m / std::max(physical_footprint.sweep_step_m, 1.0e-6)));
  double margin_m = maximum_margin_m;
  for (std::size_t pose = 0U; pose <= pose_count && margin_m > 0.0; ++pose) {
    const double ratio =
        pose_count == 0U ? 1.0
                         : static_cast<double>(pose) / static_cast<double>(pose_count);
    const Point3 position{std::lerp(first.x, second.x, ratio),
                          std::lerp(first.y, second.y, ratio),
                          std::lerp(first.z, second.z, ratio)};
    if (world.observed_occupancy != nullptr) {
      margin_m = std::min(
          margin_m, rawBodyInflationMargin3D(*world.observed_occupancy, position, body,
                                             margin_m, world.launch_support_contact));
    } else if (world.occupancy != nullptr) {
      margin_m = std::min(margin_m, rawBodyInflationMargin3D(*world.occupancy, position,
                                                             body, margin_m));
    }
  }
  return margin_m;
}

[[nodiscard]] double segmentSpeedLimitMps(
    const TrackingErrorTubeWorld3D& world, const Point3& first, const Point3& second,
    const SweptFootprintConfig& physical_footprint,
    const TrackingErrorTubeConfig3D& config, const double maximum_speed_mps) noexcept {
  // The physical body stays the only hard authority; the tube only shapes the
  // speed ceiling above it.
  if (!segmentAccepted(world, first, second, physical_footprint)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double maximum_margin_m = trackingErrorTubeRadiusM(config, maximum_speed_mps);
  const double margin_m =
      segmentInflationMarginM(world, first, second, physical_footprint,
                              config.maximum_body_tilt_rad, maximum_margin_m);
  if (margin_m >= maximum_margin_m) {
    return maximum_speed_mps;
  }
  return trackingErrorTubeSpeedLimitMps(config, margin_m, maximum_speed_mps);
}

} // namespace

bool trackingErrorTubeProfile3DIsValid(const TrackingErrorTubeProfile3D& profile,
                                       const std::size_t route_sample_count) noexcept {
  if (!profile.valid || route_sample_count < 2U ||
      !trackingErrorTubeConfig3DIsValid(profile.config) ||
      !physicalFootprintIsValid(profile.physical_footprint) ||
      profile.speed_limits_mps.size() != route_sample_count ||
      !std::isfinite(profile.unconstrained_speed_limit_mps) ||
      !(profile.unconstrained_speed_limit_mps > kSpeedToleranceMps) ||
      !std::isfinite(profile.minimum_speed_limit_mps) ||
      !(profile.minimum_speed_limit_mps > kSpeedToleranceMps) ||
      !std::isfinite(profile.maximum_tracking_error_m) ||
      profile.maximum_tracking_error_m < 0.0 ||
      profile.constrained_segment_count >= route_sample_count) {
    return false;
  }
  if (!std::ranges::all_of(profile.speed_limits_mps, [&](const double limit_mps) {
        return std::isfinite(limit_mps) && limit_mps >= 0.0 &&
               limit_mps <= profile.unconstrained_speed_limit_mps + kExecutionTolerance;
      })) {
    return false;
  }
  const auto [minimum_limit, maximum_limit] =
      std::ranges::minmax_element(profile.speed_limits_mps);
  if (!nearlyEqual(profile.minimum_speed_limit_mps, *minimum_limit) ||
      !nearlyEqual(profile.maximum_tracking_error_m,
                   trackingErrorTubeRadiusM(profile.config, *maximum_limit))) {
    return false;
  }
  return profile.obstacle_evidence_available ||
         (profile.constrained_segment_count == 0U &&
          std::ranges::all_of(profile.speed_limits_mps, [&](const double limit_mps) {
            return nearlyEqual(limit_mps, profile.unconstrained_speed_limit_mps);
          }));
}

std::string_view trackingErrorTubeProfileStatus3DName(
    const TrackingErrorTubeProfileStatus3D status) noexcept {
  switch (status) {
    case TrackingErrorTubeProfileStatus3D::kBuilt:
      return "built";
    case TrackingErrorTubeProfileStatus3D::kInvalidRoute:
      return "invalid_route";
    case TrackingErrorTubeProfileStatus3D::kInvalidWorld:
      return "invalid_world";
    case TrackingErrorTubeProfileStatus3D::kInvalidFootprint:
      return "invalid_footprint";
    case TrackingErrorTubeProfileStatus3D::kInvalidConfig:
      return "invalid_config";
    case TrackingErrorTubeProfileStatus3D::kInvalidSpeedCeiling:
      return "invalid_speed_ceiling";
    case TrackingErrorTubeProfileStatus3D::kNonFiniteSegmentLimit:
      return "non_finite_segment_limit";
    case TrackingErrorTubeProfileStatus3D::kInvalidProfile:
      return "invalid_profile";
  }
  return "unknown";
}

TrackingErrorTubeProfile3D makeTrackingErrorTubeProfile3D(
    const std::span<const RouteSample3D> route, const TrackingErrorTubeWorld3D& world,
    const SweptFootprintConfig& physical_footprint,
    const TrackingErrorTubeConfig3D& config, const double maximum_speed_mps,
    TrackingErrorTubeProfileReport3D* const outcome) {
  const auto report = [outcome](const TrackingErrorTubeProfileStatus3D verdict,
                                const std::size_t failure_segment_index = 0U) {
    if (outcome != nullptr) {
      outcome->status = verdict;
      outcome->failure_segment_index = failure_segment_index;
    }
    return TrackingErrorTubeProfile3D{};
  };
  TrackingErrorTubeProfile3D result;
  if (!routeIsValidForTube(route)) {
    return report(TrackingErrorTubeProfileStatus3D::kInvalidRoute);
  }
  if (!worldConfigurationIsValid(world)) {
    return report(TrackingErrorTubeProfileStatus3D::kInvalidWorld);
  }
  if (!physicalFootprintIsValid(physical_footprint)) {
    return report(TrackingErrorTubeProfileStatus3D::kInvalidFootprint);
  }
  if (!trackingErrorTubeConfig3DIsValid(config)) {
    return report(TrackingErrorTubeProfileStatus3D::kInvalidConfig);
  }
  if (!std::isfinite(maximum_speed_mps) || !(maximum_speed_mps > kSpeedToleranceMps)) {
    return report(TrackingErrorTubeProfileStatus3D::kInvalidSpeedCeiling);
  }

  result.obstacle_evidence_available =
      world.observed_occupancy != nullptr || world.occupancy != nullptr;
  result.config = config;
  result.physical_footprint = physical_footprint;
  result.speed_limits_mps.assign(route.size(), maximum_speed_mps);
  result.unconstrained_speed_limit_mps = maximum_speed_mps;
  result.minimum_speed_limit_mps = maximum_speed_mps;
  for (std::size_t index = 1U; index < route.size(); ++index) {
    const double segment_limit_mps =
        segmentSpeedLimitMps(world, route[index - 1U].position, route[index].position,
                             physical_footprint, config, maximum_speed_mps);
    if (!std::isfinite(segment_limit_mps)) {
      return report(TrackingErrorTubeProfileStatus3D::kNonFiniteSegmentLimit, index);
    }
    result.speed_limits_mps[index - 1U] =
        std::min(result.speed_limits_mps[index - 1U], segment_limit_mps);
    result.speed_limits_mps[index] =
        std::min(result.speed_limits_mps[index], segment_limit_mps);
    result.minimum_speed_limit_mps =
        std::min(result.minimum_speed_limit_mps, segment_limit_mps);
    result.constrained_segment_count +=
        segment_limit_mps + kSpeedToleranceMps < maximum_speed_mps ? 1U : 0U;
  }
  result.maximum_tracking_error_m = trackingErrorTubeRadiusM(
      config, *std::ranges::max_element(result.speed_limits_mps));
  result.valid = true;
  result.valid = trackingErrorTubeProfile3DIsValid(result, route.size());
  if (!result.valid) {
    return report(TrackingErrorTubeProfileStatus3D::kInvalidProfile);
  }
  if (outcome != nullptr) {
    outcome->status = TrackingErrorTubeProfileStatus3D::kBuilt;
    outcome->failure_segment_index = 0U;
  }
  return result;
}

bool trackingErrorTubeProfile3DMatchesWorld(const std::span<const RouteSample3D> route,
                                            const TrackingErrorTubeProfile3D& profile,
                                            const TrackingErrorTubeWorld3D& world) {
  if (!trackingErrorTubeProfile3DIsValid(profile, route.size()) ||
      !profile.obstacle_evidence_available || !worldConfigurationIsValid(world) ||
      (world.observed_occupancy == nullptr && world.occupancy == nullptr)) {
    return false;
  }
  const TrackingErrorTubeProfile3D derived = makeTrackingErrorTubeProfile3D(
      route, world, profile.physical_footprint, profile.config,
      profile.unconstrained_speed_limit_mps);
  // A sealed profile matches a world it is admissible on: nowhere may the
  // ceiling it seals exceed the ceiling the world now permits. A world that has
  // only loosened around the route leaves the sealed ceilings lower than they
  // need to be, and flying under a lower ceiling is safe; a world that has
  // tightened anywhere would let the profile overstate, and is refused. Read
  // as bit-for-bit equality of every ceiling, the rule made every certified
  // route perishable on every scan: a voxel that opened a little clearance
  // beside the route changed one ceiling by a hundredth of a metre a second,
  // and the successor sealed on the previous scan -- and the route the vehicle
  // was following -- were both refused for it. One recorded flight stood with
  // a successor sealed on every tick of a hold it could not leave.
  const bool sealed_ceilings_admissible =
      derived.speed_limits_mps.size() == profile.speed_limits_mps.size() &&
      std::ranges::equal(profile.speed_limits_mps, derived.speed_limits_mps,
                         [](const double sealed_mps, const double permitted_mps) {
                           return sealed_mps <= permitted_mps + kSpeedToleranceMps;
                         });
  return derived.valid && derived.obstacle_evidence_available &&
         sealed_ceilings_admissible &&
         derived.config.response_time_s == profile.config.response_time_s &&
         derived.config.maximum_body_tilt_rad == profile.config.maximum_body_tilt_rad &&
         derived.config.minimum_progress_speed_mps ==
             profile.config.minimum_progress_speed_mps &&
         derived.physical_footprint.radius_m == profile.physical_footprint.radius_m &&
         derived.physical_footprint.lower_extent_m ==
             profile.physical_footprint.lower_extent_m &&
         derived.physical_footprint.upper_extent_m ==
             profile.physical_footprint.upper_extent_m &&
         derived.physical_footprint.perimeter_samples ==
             profile.physical_footprint.perimeter_samples &&
         derived.physical_footprint.radial_rings ==
             profile.physical_footprint.radial_rings &&
         derived.physical_footprint.axial_samples ==
             profile.physical_footprint.axial_samples &&
         derived.physical_footprint.sweep_step_m ==
             profile.physical_footprint.sweep_step_m &&
         derived.physical_footprint.safe_clearance_threshold_m ==
             profile.physical_footprint.safe_clearance_threshold_m &&
         derived.unconstrained_speed_limit_mps == profile.unconstrained_speed_limit_mps;
}

std::string_view trackingErrorTubeExecutionStatus3DName(
    const TrackingErrorTubeExecutionStatus3D status) noexcept {
  switch (status) {
    case TrackingErrorTubeExecutionStatus3D::kAccepted:
      return "accepted";
    case TrackingErrorTubeExecutionStatus3D::kInvalidProfile:
      return "invalid_profile";
    case TrackingErrorTubeExecutionStatus3D::kInvalidObservation:
      return "invalid_observation";
    case TrackingErrorTubeExecutionStatus3D::kSpeedLimitExceeded:
      return "speed_limit_exceeded";
    case TrackingErrorTubeExecutionStatus3D::kCrossTrackExceeded:
      return "cross_track_exceeded";
  }
  return "invalid_status";
}

TrackingErrorTubeExecutionAssessment3D assessTrackingErrorTubeExecution3D(
    const std::span<const RouteSample3D> route,
    const TrackingErrorTubeProfile3D& profile,
    const TrackingErrorTubeExecutionObservation3D& observation) noexcept {
  TrackingErrorTubeExecutionAssessment3D result;
  if (!trackingErrorTubeProfile3DIsValid(profile, route.size())) {
    return result;
  }
  if (!std::isfinite(observation.station_m) ||
      !std::isfinite(observation.cross_track_error_m) ||
      !std::isfinite(observation.speed_mps) || observation.cross_track_error_m < 0.0 ||
      observation.speed_mps < 0.0 ||
      observation.station_m < route.front().station_m - kExecutionTolerance ||
      observation.station_m > route.back().station_m + kExecutionTolerance) {
    result.status = TrackingErrorTubeExecutionStatus3D::kInvalidObservation;
    return result;
  }

  const double station_m = std::clamp(observation.station_m, route.front().station_m,
                                      route.back().station_m);
  const auto upper =
      std::ranges::upper_bound(route, station_m, {}, [](const RouteSample3D& sample) {
        return sample.station_m;
      });
  if (upper == route.begin()) {
    result.speed_limit_mps = profile.speed_limits_mps.front();
  } else if (upper == route.end()) {
    result.speed_limit_mps = profile.speed_limits_mps.back();
  } else {
    const std::size_t upper_index =
        static_cast<std::size_t>(std::distance(route.begin(), upper));
    result.speed_limit_mps = std::min(profile.speed_limits_mps[upper_index - 1U],
                                      profile.speed_limits_mps[upper_index]);
  }
  result.tube_radius_m =
      trackingErrorTubeRadiusM(profile.config, result.speed_limit_mps);
  if (!std::isfinite(result.tube_radius_m)) {
    result.status = TrackingErrorTubeExecutionStatus3D::kInvalidProfile;
    return result;
  }
  if (observation.speed_mps > result.speed_limit_mps + kExecutionTolerance) {
    result.status = TrackingErrorTubeExecutionStatus3D::kSpeedLimitExceeded;
    return result;
  }
  if (observation.cross_track_error_m > result.tube_radius_m + kExecutionTolerance) {
    result.status = TrackingErrorTubeExecutionStatus3D::kCrossTrackExceeded;
    return result;
  }
  result.status = TrackingErrorTubeExecutionStatus3D::kAccepted;
  return result;
}

std::string_view trackingErrorTubeHandoffStatus3DName(
    const TrackingErrorTubeHandoffStatus3D status) noexcept {
  switch (status) {
    case TrackingErrorTubeHandoffStatus3D::kActive:
      return "active";
    case TrackingErrorTubeHandoffStatus3D::kNotRequired:
      return "not_required";
    case TrackingErrorTubeHandoffStatus3D::kInvalidContract:
      return "invalid_contract";
    case TrackingErrorTubeHandoffStatus3D::kOutsideValidityWindow:
      return "outside_validity_window";
    case TrackingErrorTubeHandoffStatus3D::kReferenceAcquiredRouteTube:
      return "reference_acquired_route_tube";
    case TrackingErrorTubeHandoffStatus3D::kSpeedLimitExceeded:
      return "speed_limit_exceeded";
    case TrackingErrorTubeHandoffStatus3D::kTrackingErrorExceeded:
      return "tracking_error_exceeded";
  }
  return "invalid_status";
}

TrackingErrorTubeHandoffAssessment3D assessTrackingErrorTubeHandoff3D(
    const std::span<const RouteSample3D> route,
    const TrackingErrorTubeProfile3D& profile,
    const std::span<const MotionState3D> handoff_states,
    const double begin_route_station_m, const std::int64_t valid_from_ns,
    const std::int64_t valid_until_ns, const std::int64_t control_interval_ns,
    const TrackingErrorTubeHandoffObservation3D& observation) noexcept {
  TrackingErrorTubeHandoffAssessment3D result;
  const std::size_t control_count =
      handoff_states.empty() ? 0U : handoff_states.size() - 1U;
  const bool duration_fits =
      control_interval_ns > 0 && valid_from_ns > 0 && control_count > 0U &&
      control_count <= static_cast<std::size_t>(
                           (std::numeric_limits<std::int64_t>::max() - valid_from_ns) /
                           control_interval_ns);
  if (!trackingErrorTubeProfile3DIsValid(profile, route.size()) ||
      handoff_states.size() < 2U || !duration_fits ||
      valid_until_ns != valid_from_ns + static_cast<std::int64_t>(control_count) *
                                            control_interval_ns ||
      !std::isfinite(begin_route_station_m) ||
      begin_route_station_m < route.front().station_m - kHandoffStationToleranceM ||
      begin_route_station_m > route.back().station_m + kHandoffStationToleranceM ||
      !std::ranges::all_of(handoff_states,
                           [](const MotionState3D& state) {
                             return std::isfinite(state.x) && std::isfinite(state.y) &&
                                    std::isfinite(state.z) && std::isfinite(state.vx) &&
                                    std::isfinite(state.vy) &&
                                    std::isfinite(state.vz) &&
                                    std::isfinite(state.yaw) &&
                                    std::isfinite(state.yaw_rate);
                           }) ||
      !std::isfinite(observation.state.x) || !std::isfinite(observation.state.y) ||
      !std::isfinite(observation.state.z) || !std::isfinite(observation.state.vx) ||
      !std::isfinite(observation.state.vy) || !std::isfinite(observation.state.vz)) {
    return result;
  }
  if (observation.stamp_ns < valid_from_ns || observation.stamp_ns >= valid_until_ns) {
    result.status = TrackingErrorTubeHandoffStatus3D::kOutsideValidityWindow;
    return result;
  }

  const std::int64_t elapsed_ns = observation.stamp_ns - valid_from_ns;
  const std::size_t lower_index =
      static_cast<std::size_t>(elapsed_ns / control_interval_ns);
  if (lower_index >= control_count) {
    result.status = TrackingErrorTubeHandoffStatus3D::kOutsideValidityWindow;
    return result;
  }
  const double interpolation_ratio =
      static_cast<double>(elapsed_ns % control_interval_ns) /
      static_cast<double>(control_interval_ns);
  result.reference_state_index = lower_index;

  std::vector<RouteProjection3D> projections;
  projections.reserve(handoff_states.size());
  RouteProjection3D previous_projection = projectOntoRoute3DWithinStationWindow(
      route, statePosition(handoff_states.front()),
      std::max(route.front().station_m, begin_route_station_m),
      std::min(route.back().station_m,
               begin_route_station_m + kHandoffStationToleranceM));
  if (!previous_projection.valid) {
    return result;
  }
  projections.push_back(previous_projection);
  const TrackingErrorTubeExecutionAssessment3D initial =
      assessTrackingErrorTubeExecution3D(
          route, profile,
          TrackingErrorTubeExecutionObservation3D{
              .station_m = previous_projection.station_m,
              .cross_track_error_m = previous_projection.distance_m,
              .speed_mps = stateSpeedMps(handoff_states.front()),
          });
  if (initial.accepted()) {
    result.status = TrackingErrorTubeHandoffStatus3D::kNotRequired;
    return result;
  }

  bool route_tube_acquired{false};
  Point3 previous_position = statePosition(handoff_states.front());
  for (std::size_t index = 1U; index < handoff_states.size(); ++index) {
    const Point3 position = statePosition(handoff_states[index]);
    const double travel_m = distance3D(previous_position, position);
    if (!std::isfinite(travel_m)) {
      return result;
    }
    const RouteProjection3D projection = projectOntoRoute3DWithinStationWindow(
        route, position,
        std::max(route.front().station_m, previous_projection.station_m),
        std::min(route.back().station_m,
                 previous_projection.station_m + travel_m + kHandoffStationToleranceM));
    if (!projection.valid || projection.station_m + kHandoffStationToleranceM <
                                 previous_projection.station_m) {
      return result;
    }
    const TrackingErrorTubeExecutionAssessment3D assessment =
        assessTrackingErrorTubeExecution3D(
            route, profile,
            TrackingErrorTubeExecutionObservation3D{
                .station_m = projection.station_m,
                .cross_track_error_m = projection.distance_m,
                .speed_mps = stateSpeedMps(handoff_states[index]),
            });
    if (assessment.accepted()) {
      route_tube_acquired = true;
    } else if (route_tube_acquired) {
      return result;
    }
    projections.push_back(projection);
    previous_projection = projection;
    previous_position = position;
  }
  if (!route_tube_acquired) {
    return result;
  }

  const MotionState3D reference =
      interpolateState(handoff_states[lower_index], handoff_states[lower_index + 1U],
                       interpolation_ratio);
  const RouteProjection3D reference_projection = projectOntoRoute3DWithinStationWindow(
      route, statePosition(reference), projections[lower_index].station_m,
      projections[lower_index + 1U].station_m + kHandoffStationToleranceM);
  if (!reference_projection.valid) {
    return result;
  }
  const TrackingErrorTubeExecutionAssessment3D reference_route_tube =
      assessTrackingErrorTubeExecution3D(
          route, profile,
          TrackingErrorTubeExecutionObservation3D{
              .station_m = reference_projection.station_m,
              .cross_track_error_m = reference_projection.distance_m,
              .speed_mps = stateSpeedMps(reference),
          });
  if (reference_route_tube.accepted()) {
    result.status = TrackingErrorTubeHandoffStatus3D::kReferenceAcquiredRouteTube;
    return result;
  }

  result.reference_speed_limit_mps =
      std::max(stateSpeedMps(handoff_states[lower_index]),
               stateSpeedMps(handoff_states[lower_index + 1U]));
  result.tracking_error_radius_m =
      trackingErrorTubeRadiusM(profile.config, result.reference_speed_limit_mps);
  const double actual_speed_mps = stateSpeedMps(observation.state);
  result.actual_tracking_error_m =
      distance3D(statePosition(observation.state), statePosition(reference));
  if (!std::isfinite(result.tracking_error_radius_m) ||
      !std::isfinite(result.actual_tracking_error_m)) {
    return result;
  }
  if (actual_speed_mps > result.reference_speed_limit_mps + kExecutionTolerance) {
    result.status = TrackingErrorTubeHandoffStatus3D::kSpeedLimitExceeded;
    return result;
  }
  if (result.actual_tracking_error_m >
      result.tracking_error_radius_m + kExecutionTolerance) {
    result.status = TrackingErrorTubeHandoffStatus3D::kTrackingErrorExceeded;
    return result;
  }
  result.status = TrackingErrorTubeHandoffStatus3D::kActive;
  return result;
}

} // namespace drone_city_nav
