#include "drone_city_nav/compiled_trajectory_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <ranges>
#include <span>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kStationToleranceM{1.0e-6};
constexpr double kGeometryTolerance{1.0e-4};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finiteVector(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] bool nearlyEqual(const double first, const double second,
                               const double tolerance) noexcept {
  return std::abs(first - second) <= tolerance;
}

[[nodiscard]] double vectorNorm(const Vec3& vector) noexcept {
  return std::hypot(std::hypot(vector.x, vector.y), vector.z);
}

[[nodiscard]] double vectorDot(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

[[nodiscard]] bool
endpointSemanticsValid(const RouteEndpointSemantics3D semantics) noexcept {
  switch (semantics) {
    case RouteEndpointSemantics3D::kContinuation:
    case RouteEndpointSemantics3D::kLocalStop:
    case RouteEndpointSemantics3D::kMissionStop:
    case RouteEndpointSemantics3D::kEmergencyBrakeTail:
      return true;
  }
  return false;
}

[[nodiscard]] bool validEnvelopeSamples(const ConstrainedRouteSpan& span) noexcept {
  if (span.envelope.empty()) {
    return false;
  }
  double previous_station_m{-std::numeric_limits<double>::infinity()};
  for (const RouteEnvelopeSample& sample : span.envelope) {
    if (!std::isfinite(sample.station_m) ||
        !std::isfinite(sample.lateral_free_left_m) ||
        !std::isfinite(sample.lateral_free_right_m) || !std::isfinite(sample.min_z_m) ||
        !std::isfinite(sample.max_z_m) || !std::isfinite(sample.minimum_clearance_m) ||
        !std::isfinite(sample.reference_z_m) ||
        !std::isfinite(sample.reference_speed_mps) ||
        sample.station_m <= previous_station_m ||
        sample.station_m + kStationToleranceM < span.begin_station_m ||
        sample.station_m > span.end_station_m + kStationToleranceM ||
        sample.lateral_free_left_m < 0.0 || sample.lateral_free_right_m < 0.0 ||
        sample.min_z_m > sample.max_z_m ||
        sample.reference_z_m < sample.min_z_m - kGeometryTolerance ||
        sample.reference_z_m > sample.max_z_m + kGeometryTolerance ||
        sample.minimum_clearance_m < 0.0 || sample.reference_speed_mps < 0.0) {
      return false;
    }
    previous_station_m = sample.station_m;
  }
  return span.envelope.front().station_m <= span.begin_station_m + kStationToleranceM &&
         span.envelope.back().station_m + kStationToleranceM >= span.end_station_m;
}

[[nodiscard]] bool
validTraversalSegmentSpans(const ConstrainedRouteSpan& span) noexcept {
  double previous_end_station_m{span.begin_station_m};
  for (const PassageTraversalSegmentSpan& segment : span.segment_spans) {
    if (segment.passage_segment_id.empty() || !std::isfinite(segment.begin_station_m) ||
        !std::isfinite(segment.end_station_m) ||
        segment.begin_station_m + kStationToleranceM < span.begin_station_m ||
        segment.end_station_m > span.end_station_m + kStationToleranceM ||
        segment.end_station_m <= segment.begin_station_m ||
        segment.begin_station_m + kStationToleranceM < previous_end_station_m) {
      return false;
    }
    previous_end_station_m = segment.end_station_m;
  }
  return true;
}

} // namespace

const char* compiledTrajectoryFailureReason3DName(
    const CompiledTrajectoryFailureReason3D reason) noexcept {
  switch (reason) {
    case CompiledTrajectoryFailureReason3D::kNotAttempted:
      return "not_attempted";
    case CompiledTrajectoryFailureReason3D::kValid:
      return "valid";
    case CompiledTrajectoryFailureReason3D::kMissingRoute:
      return "missing_route";
    case CompiledTrajectoryFailureReason3D::kInvalidInitialState:
      return "invalid_initial_state";
    case CompiledTrajectoryFailureReason3D::kTooFewSamples:
      return "too_few_samples";
    case CompiledTrajectoryFailureReason3D::kNonFiniteSample:
      return "non_finite_sample";
    case CompiledTrajectoryFailureReason3D::kInvalidTangent:
      return "invalid_tangent";
    case CompiledTrajectoryFailureReason3D::kNonMonotonicStation:
      return "non_monotonic_station";
    case CompiledTrajectoryFailureReason3D::kSegmentStationMismatch:
      return "segment_station_mismatch";
    case CompiledTrajectoryFailureReason3D::kIncomingTangentMismatch:
      return "incoming_tangent_mismatch";
    case CompiledTrajectoryFailureReason3D::kTerminalTangentMismatch:
      return "terminal_tangent_mismatch";
    case CompiledTrajectoryFailureReason3D::kInvalidTrackingErrorTube:
      return "invalid_tracking_error_tube";
    case CompiledTrajectoryFailureReason3D::kInvalidTimeProfile:
      return "invalid_time_profile";
    case CompiledTrajectoryFailureReason3D::kInvalidConstrainedSpans:
      return "invalid_constrained_spans";
    case CompiledTrajectoryFailureReason3D::kInvalidFingerprint:
      return "invalid_fingerprint";
    case CompiledTrajectoryFailureReason3D::kDerivedResourceMismatch:
      return "derived_resource_mismatch";
  }
  return "unknown";
}

CompiledTrajectoryValidation3D validateCompiledTrajectorySamples3D(
    const std::span<const RouteSample3D> route) noexcept {
  using Failure = CompiledTrajectoryFailureReason3D;
  if (route.size() < 2U) {
    return {Failure::kTooFewSamples, route.size()};
  }
  double previous_station_m{-std::numeric_limits<double>::infinity()};
  Point3 previous_position{};
  Vec3 previous_tangent{};
  for (std::size_t index = 0U; index < route.size(); ++index) {
    const RouteSample3D& sample = route[index];
    const double tangent_norm = vectorNorm(sample.tangent);
    if (!finitePoint(sample.position) || !finiteVector(sample.tangent) ||
        !std::isfinite(sample.station_m) ||
        !std::isfinite(sample.reference_speed_mps) || sample.station_m < 0.0 ||
        sample.reference_speed_mps < 0.0 ||
        (sample.transition == RouteKinematicTransition3D::kStopAndTurn &&
         sample.reference_speed_mps > 1.0e-6)) {
      return {Failure::kNonFiniteSample, index};
    }
    if (!nearlyEqual(tangent_norm, 1.0, 1.0e-3)) {
      return {Failure::kInvalidTangent, index};
    }
    if (sample.station_m <= previous_station_m) {
      return {Failure::kNonMonotonicStation, index};
    }
    if (index == 0U) {
      if (!nearlyEqual(sample.station_m, 0.0, kStationToleranceM)) {
        return {Failure::kSegmentStationMismatch, index};
      }
    } else {
      const double segment_length_m = distance3D(previous_position, sample.position);
      const double station_delta_m = sample.station_m - previous_station_m;
      if (segment_length_m <= kStationToleranceM ||
          !nearlyEqual(station_delta_m, segment_length_m, 1.0e-4)) {
        return {Failure::kSegmentStationMismatch, index};
      }
      const Vec3 segment_direction{
          (sample.position.x - previous_position.x) / segment_length_m,
          (sample.position.y - previous_position.y) / segment_length_m,
          (sample.position.z - previous_position.z) / segment_length_m};
      if (vectorDot(previous_tangent, segment_direction) <= 0.0) {
        return {Failure::kIncomingTangentMismatch, index};
      }
      if (index + 1U == route.size() &&
          vectorDot(sample.tangent, segment_direction) <= 0.0) {
        return {Failure::kTerminalTangentMismatch, index};
      }
    }
    previous_station_m = sample.station_m;
    previous_position = sample.position;
    previous_tangent = sample.tangent;
  }
  return {Failure::kValid, route.size() - 1U};
}

bool compiledTrajectoryResourcesValid3D(
    const CompiledTrajectory3D& trajectory,
    const std::uint64_t expected_route_generation) noexcept {
  if (!endpointSemanticsValid(trajectory.endpoint_semantics) ||
      trajectory.route == nullptr || trajectory.tracking_error_tube == nullptr ||
      trajectory.constrained_spans == nullptr ||
      !trajectory.exact_initial_state.valid() || !trajectory.time_profile.valid() ||
      trajectory.time_profile.arrival_times_s.size() != trajectory.route->size() ||
      trajectory.time_profile.departure_times_s.size() != trajectory.route->size() ||
      trajectory.materialized_route_fingerprint == 0U ||
      trajectory.physical_route_fingerprint == 0U ||
      !validateCompiledTrajectorySamples3D(*trajectory.route).valid() ||
      routeFingerprint(*trajectory.route) != trajectory.physical_route_fingerprint ||
      !trackingErrorTubeProfile3DIsValid(*trajectory.tracking_error_tube,
                                         trajectory.route->size())) {
    return false;
  }
  constexpr double kTerminalSpeedToleranceMps{1.0e-4};
  const double terminal_speed_mps = trajectory.route->back().reference_speed_mps;
  if (routeEndpointHasTerminalStop3D(trajectory.endpoint_semantics)
          ? std::abs(terminal_speed_mps) > kTerminalSpeedToleranceMps
          : terminal_speed_mps <= kTerminalSpeedToleranceMps) {
    return false;
  }
  for (std::size_t index = 0U; index < trajectory.route->size(); ++index) {
    if ((*trajectory.route)[index].reference_speed_mps >
        trajectory.tracking_error_tube->speed_limits_mps[index] + kGeometryTolerance) {
      return false;
    }
  }

  const std::vector<ConstrainedRouteSpan>& spans = *trajectory.constrained_spans;
  double previous_span_end_station_m{-std::numeric_limits<double>::infinity()};
  for (const ConstrainedRouteSpan& span : spans) {
    if (span.route_generation == 0U ||
        (expected_route_generation != 0U &&
         span.route_generation != expected_route_generation) ||
        span.passage_traversal_id.empty() ||
        (span.direction_sign != -1 && span.direction_sign != 1) ||
        !std::isfinite(span.begin_station_m) || !std::isfinite(span.end_station_m) ||
        span.begin_station_m < 0.0 || span.end_station_m <= span.begin_station_m ||
        span.begin_station_m + kStationToleranceM < previous_span_end_station_m ||
        span.end_station_m > trajectory.route->back().station_m + kStationToleranceM ||
        !validEnvelopeSamples(span) || !validTraversalSegmentSpans(span)) {
      return false;
    }
    previous_span_end_station_m = span.end_station_m;
  }
  return true;
}

} // namespace drone_city_nav
