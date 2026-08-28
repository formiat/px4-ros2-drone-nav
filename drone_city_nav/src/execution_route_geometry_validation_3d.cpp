#include "drone_city_nav/execution_route_snapshot_3d.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <span>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav {

const char* executionRouteGeometryFailureReasonName3D(
    const ExecutionRouteGeometryFailureReason3D reason) noexcept {
  switch (reason) {
    case ExecutionRouteGeometryFailureReason3D::kNotAttempted:
      return "not_attempted";
    case ExecutionRouteGeometryFailureReason3D::kValid:
      return "valid";
    case ExecutionRouteGeometryFailureReason3D::kMissingRoute:
      return "missing_route";
    case ExecutionRouteGeometryFailureReason3D::kTooFewSamples:
      return "too_few_samples";
    case ExecutionRouteGeometryFailureReason3D::kNonFiniteSample:
      return "non_finite_sample";
    case ExecutionRouteGeometryFailureReason3D::kInvalidTangent:
      return "invalid_tangent";
    case ExecutionRouteGeometryFailureReason3D::kNonMonotonicStation:
      return "non_monotonic_station";
    case ExecutionRouteGeometryFailureReason3D::kSegmentStationMismatch:
      return "segment_station_mismatch";
    case ExecutionRouteGeometryFailureReason3D::kIncomingTangentMismatch:
      return "incoming_tangent_mismatch";
    case ExecutionRouteGeometryFailureReason3D::kTerminalTangentMismatch:
      return "terminal_tangent_mismatch";
    case ExecutionRouteGeometryFailureReason3D::kInvalidTimeProfile:
      return "invalid_time_profile";
    case ExecutionRouteGeometryFailureReason3D::kInvalidProjection:
      return "invalid_projection";
    case ExecutionRouteGeometryFailureReason3D::kInvalidConstrainedSpans:
      return "invalid_constrained_spans";
    case ExecutionRouteGeometryFailureReason3D::kInvalidPassageResources:
      return "invalid_passage_resources";
    case ExecutionRouteGeometryFailureReason3D::kInvalidFingerprint:
      return "invalid_fingerprint";
    case ExecutionRouteGeometryFailureReason3D::kDerivedResourceMismatch:
      return "derived_resource_mismatch";
  }
  return "unknown";
}

ExecutionRouteGeometryValidation3D validateExecutionRouteGeometrySamples3D(
    const std::span<const RouteSample3D> route) noexcept {
  using Failure = ExecutionRouteGeometryFailureReason3D;
  if (route.size() < 2U) {
    return {Failure::kTooFewSamples, route.size()};
  }
  double previous_station_m{-std::numeric_limits<double>::infinity()};
  Point3 previous_position{};
  Vec3 previous_tangent{};
  for (std::size_t index = 0U; index < route.size(); ++index) {
    const RouteSample3D& sample = route[index];
    const double tangent_norm =
        std::hypot(std::hypot(sample.tangent.x, sample.tangent.y), sample.tangent.z);
    if (!execution_route_snapshot_3d_internal::finitePoint(sample.position) ||
        !execution_route_snapshot_3d_internal::finiteVector(sample.tangent) ||
        !std::isfinite(sample.station_m) ||
        !std::isfinite(sample.reference_speed_mps) || sample.station_m < 0.0 ||
        sample.reference_speed_mps < 0.0 ||
        (sample.transition == RouteKinematicTransition3D::kStopAndTurn &&
         sample.reference_speed_mps > 1.0e-6)) {
      return {Failure::kNonFiniteSample, index};
    }
    if (!execution_route_snapshot_3d_internal::nearlyEqual(tangent_norm, 1.0, 1.0e-3)) {
      return {Failure::kInvalidTangent, index};
    }
    if (sample.station_m <= previous_station_m) {
      return {Failure::kNonMonotonicStation, index};
    }
    if (index == 0U) {
      if (!execution_route_snapshot_3d_internal::nearlyEqual(
              sample.station_m, 0.0,
              execution_route_snapshot_3d_internal::kStationToleranceM)) {
        return {Failure::kSegmentStationMismatch, index};
      }
    } else {
      const double segment_length_m = distance3D(previous_position, sample.position);
      const double station_delta_m = sample.station_m - previous_station_m;
      if (segment_length_m <=
              execution_route_snapshot_3d_internal::kStationToleranceM ||
          !execution_route_snapshot_3d_internal::nearlyEqual(
              station_delta_m, segment_length_m, 1.0e-4)) {
        return {Failure::kSegmentStationMismatch, index};
      }
      const Vec3 segment_direction{
          (sample.position.x - previous_position.x) / segment_length_m,
          (sample.position.y - previous_position.y) / segment_length_m,
          (sample.position.z - previous_position.z) / segment_length_m};
      const auto dot = [](const Vec3& first, const Vec3& second) noexcept {
        return first.x * second.x + first.y * second.y + first.z * second.z;
      };
      if (dot(previous_tangent, segment_direction) <= 0.0) {
        return {Failure::kIncomingTangentMismatch, index};
      }
      if (index + 1U == route.size() && dot(sample.tangent, segment_direction) <= 0.0) {
        return {Failure::kTerminalTangentMismatch, index};
      }
    }
    previous_station_m = sample.station_m;
    previous_position = sample.position;
    previous_tangent = sample.tangent;
  }
  return {Failure::kValid, route.size() - 1U};
}

} // namespace drone_city_nav
