#include "drone_city_nav/route_compiler_3d.hpp"

#include "drone_city_nav/route_time_parameterization.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <ranges>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool
spansStructurallyValid(const std::span<const RouteSample3D> route,
                       const std::span<const ConstrainedRouteSpan> spans) noexcept {
  if (route.empty()) {
    return false;
  }
  double previous_end_station_m{-1.0};
  for (const ConstrainedRouteSpan& span : spans) {
    if (span.passage_traversal_id.empty() ||
        (span.direction_sign != -1 && span.direction_sign != 1) ||
        !std::isfinite(span.begin_station_m) || !std::isfinite(span.end_station_m) ||
        span.begin_station_m < 0.0 || span.end_station_m <= span.begin_station_m ||
        span.begin_station_m + 1.0e-6 < previous_end_station_m ||
        span.end_station_m > route.back().station_m + 1.0e-6 || span.envelope.empty()) {
      return false;
    }
    double previous_envelope_station_m{-1.0};
    for (const RouteEnvelopeSample& sample : span.envelope) {
      if (!std::isfinite(sample.station_m) ||
          sample.station_m <= previous_envelope_station_m ||
          sample.station_m + 1.0e-6 < span.begin_station_m ||
          sample.station_m > span.end_station_m + 1.0e-6) {
        return false;
      }
      previous_envelope_station_m = sample.station_m;
    }
    if (span.envelope.front().station_m > span.begin_station_m + 1.0e-6 ||
        span.envelope.back().station_m + 1.0e-6 < span.end_station_m) {
      return false;
    }
    previous_end_station_m = span.end_station_m;
  }
  return true;
}

[[nodiscard]] bool
passageResourcesStructurallyValid(const RouteCompilerInput3D& input) noexcept {
  if (input.passage_volumes.size() != input.constrained_spans.size() ||
      (!input.cooperative_passage_assignments.empty() &&
       input.cooperative_passage_assignments.size() !=
           input.constrained_spans.size())) {
    return false;
  }
  std::vector<PassageTraversalId> expected_ids;
  for (const ConstrainedRouteSpan& span : input.constrained_spans) {
    if (std::ranges::find(expected_ids, span.passage_traversal_id) ==
        expected_ids.end()) {
      expected_ids.push_back(span.passage_traversal_id);
    }
  }
  return expected_ids == input.selected_passage_traversal_ids;
}

[[nodiscard]] std::shared_ptr<const std::vector<mppi::RouteSample3D>>
compileTimeProfile(const RouteCompilerInput3D& input,
                   const std::span<const double> tracking_speed_limits_mps) {
  const RouteTimeParameterization3D timing = parameterizeRouteTime3D(
      input.route, input.constrained_spans, input.config.unconstrained_speed_mps,
      input.config.constrained_speed_mps, input.endpoint_semantics,
      input.config.speed_policy, input.config.dynamics, std::nullopt,
      tracking_speed_limits_mps);
  if (!timing.valid || timing.reference_speeds_mps.size() != input.route.size()) {
    return nullptr;
  }
  auto compiled = std::make_shared<std::vector<mppi::RouteSample3D>>();
  compiled->reserve(input.route.size());
  for (std::size_t index = 0U; index < input.route.size(); ++index) {
    const RouteSample3D& sample = input.route[index];
    compiled->push_back(mppi::RouteSample3D{
        .x_m = static_cast<float>(sample.position.x),
        .y_m = static_cast<float>(sample.position.y),
        .z_m = static_cast<float>(sample.position.z),
        .tangent_x = static_cast<float>(sample.tangent.x),
        .tangent_y = static_cast<float>(sample.tangent.y),
        .tangent_z = static_cast<float>(sample.tangent.z),
        .station_m = static_cast<float>(sample.station_m),
        .reference_speed_mps = static_cast<float>(timing.reference_speeds_mps[index]),
        .required_risk_tier = sample.required_risk_tier,
    });
  }
  return compiled;
}

} // namespace

RouteCompilationResult3D compileExecutionRoute3D(RouteCompilerInput3D input) {
  using Failure = ExecutionRouteGeometryFailureReason3D;
  RouteCompilationResult3D result;
  if (input.route.size() < 2U) {
    result.validation = {Failure::kTooFewSamples, input.route.size()};
    return result;
  }
  if (!canonicalizeRouteKinematics3D(input.route,
                                     input.config.minimum_continuous_turn_alignment,
                                     &result.stop_turn_count)) {
    result.validation = {Failure::kNonFiniteSample, 0U};
    return result;
  }
  result.validation = validateExecutionRouteGeometrySamples3D(input.route);
  if (!result.validation.valid()) {
    return result;
  }
  if (!spansStructurallyValid(input.route, input.constrained_spans)) {
    result.validation = {Failure::kInvalidConstrainedSpans, 0U};
    return result;
  }
  if (!passageResourcesStructurallyValid(input)) {
    result.validation = {Failure::kInvalidPassageResources, 0U};
    return result;
  }
  if (input.materialized_route_fingerprint == 0U) {
    result.validation = {Failure::kInvalidFingerprint, 0U};
    return result;
  }
  const double maximum_profile_speed_mps = std::min(
      {input.config.unconstrained_speed_mps, input.config.speed_policy.cruise_speed_mps,
       input.config.speed_policy.absolute_speed_limit_mps,
       static_cast<double>(input.config.dynamics.maximum_horizontal_speed_mps),
       static_cast<double>(input.config.dynamics.maximum_translational_speed_mps)});
  result.tracking_error_tube =
      std::make_shared<const TrackingErrorTubeProfile3D>(makeTrackingErrorTubeProfile3D(
          input.route, input.tracking_world, input.config.physical_footprint,
          input.config.tracking_error_tube, maximum_profile_speed_mps));
  if (!result.tracking_error_tube->valid) {
    result.validation = {Failure::kInvalidTimeProfile, 0U};
    return result;
  }
  const std::shared_ptr<const std::vector<mppi::RouteSample3D>> mppi_route =
      compileTimeProfile(input, result.tracking_error_tube->speed_limits_mps);
  if (mppi_route == nullptr || mppi_route->size() != input.route.size()) {
    result.validation = {Failure::kInvalidTimeProfile, 0U};
    return result;
  }
  auto projection = std::make_shared<std::vector<Point2>>();
  projection->reserve(input.route.size());
  for (const RouteSample3D& sample : input.route) {
    projection->push_back(Point2{sample.position.x, sample.position.y});
  }
  if (projection->size() != input.route.size()) {
    result.validation = {Failure::kInvalidProjection, 0U};
    return result;
  }
  auto geometry = std::make_shared<ExecutionRouteGeometry3D>(ExecutionRouteGeometry3D{
      .mppi_route = mppi_route,
      .route =
          std::make_shared<const std::vector<RouteSample3D>>(std::move(input.route)),
      .tracking_error_tube = result.tracking_error_tube,
      .route_2d_projection = std::move(projection),
      .constrained_spans = std::make_shared<const std::vector<ConstrainedRouteSpan>>(
          std::move(input.constrained_spans)),
      .passage_volumes = std::make_shared<const std::vector<PassageVolume>>(
          std::move(input.passage_volumes)),
      .cooperative_passage_assignments =
          std::make_shared<const std::vector<CooperativePassageAssignment>>(
              std::move(input.cooperative_passage_assignments)),
      .selected_passage_traversal_ids =
          std::make_shared<const std::vector<PassageTraversalId>>(
              std::move(input.selected_passage_traversal_ids)),
      .passage_volume_config = input.passage_volume_config,
      .materialized_route_fingerprint = input.materialized_route_fingerprint,
  });
  geometry->physical_route_fingerprint = routeFingerprint(*geometry->route);
  geometry->executable_geometry_revision = executionRouteGeometryRevision3D(*geometry);
  if (geometry->physical_route_fingerprint == 0U ||
      geometry->executable_geometry_revision == 0U ||
      executionPassageGeometryRevision3D(*geometry) == 0U) {
    result.validation = {Failure::kDerivedResourceMismatch, 0U};
    return result;
  }
  result.geometry = std::move(geometry);
  result.validation = {Failure::kValid, result.geometry->route->size() - 1U};
  return result;
}

} // namespace drone_city_nav
