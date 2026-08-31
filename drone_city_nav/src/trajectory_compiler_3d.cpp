#include "drone_city_nav/trajectory_compiler_3d.hpp"

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
passageResourcesStructurallyValid(const TrajectoryCompilerInput3D& input) noexcept {
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

} // namespace

TrajectoryCompilationResult3D
TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D input) {
  using Failure = CompiledTrajectoryFailureReason3D;
  TrajectoryCompilationResult3D result;
  if (!input.exact_initial_state.valid()) {
    result.validation = {Failure::kInvalidInitialState, 0U};
    return result;
  }
  if (input.route_generation == 0U) {
    result.validation = {Failure::kInvalidPassageResources, 0U};
    return result;
  }
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
  result.validation = validateCompiledTrajectorySamples3D(input.route);
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
  const double maximum_profile_speed_mps =
      std::min(input.config.unconstrained_speed_mps,
               input.config.time_model.maximum_horizontal_speed_mps);
  auto tracking_error_tube =
      std::make_shared<const TrackingErrorTubeProfile3D>(makeTrackingErrorTubeProfile3D(
          input.route, input.tracking_world, input.config.physical_footprint,
          input.config.tracking_error_tube, maximum_profile_speed_mps));
  if (!tracking_error_tube->valid) {
    result.validation = {Failure::kInvalidTrackingErrorTube, 0U};
    return result;
  }
  RouteTimeParameterization3D parameterization = parameterizeRouteTime3D(
      input.route, input.constrained_spans, input.config.unconstrained_speed_mps,
      input.config.constrained_speed_mps, input.endpoint_semantics,
      input.config.maximum_lateral_acceleration_mps2, input.config.time_model,
      input.exact_initial_state.velocity, tracking_error_tube->speed_limits_mps);
  if (!parameterization.valid ||
      parameterization.reference_speeds_mps.size() != input.route.size()) {
    result.validation = {Failure::kInvalidTimeProfile, 0U};
    return result;
  }
  for (std::size_t index = 0U; index < input.route.size(); ++index) {
    input.route[index].reference_speed_mps =
        parameterization.reference_speeds_mps[index];
  }
  result.validation = validateCompiledTrajectorySamples3D(input.route);
  if (!result.validation.valid()) {
    return result;
  }

  const std::uint64_t physical_route_fingerprint = routeFingerprint(input.route);
  if (physical_route_fingerprint == 0U) {
    result.validation = {Failure::kInvalidFingerprint, 0U};
    return result;
  }
  const auto route =
      std::make_shared<const std::vector<RouteSample3D>>(std::move(input.route));
  const auto constrained_spans =
      std::make_shared<const std::vector<ConstrainedRouteSpan>>(
          std::move(input.constrained_spans));
  const auto passage_volumes = std::make_shared<const std::vector<PassageVolume>>(
      std::move(input.passage_volumes));
  const auto cooperative_assignments =
      std::make_shared<const std::vector<CooperativePassageAssignment>>(
          std::move(input.cooperative_passage_assignments));
  const auto traversal_ids = std::make_shared<const std::vector<PassageTraversalId>>(
      std::move(input.selected_passage_traversal_ids));
  const CompiledTrajectoryTimeProfile3D time_profile{
      .travel_time_s = parameterization.travel_time_s,
      .translation_time_s = parameterization.translation_time_s,
      .stationary_turn_time_s = parameterization.stationary_turn_time_s,
      .arrival_times_s = std::move(parameterization.arrival_times_s),
      .departure_times_s = std::move(parameterization.departure_times_s),
  };

  auto trajectory =
      std::shared_ptr<const CompiledTrajectory3D>{new CompiledTrajectory3D(
          input.exact_initial_state, input.endpoint_semantics, route,
          std::move(tracking_error_tube), constrained_spans, passage_volumes,
          cooperative_assignments, traversal_ids, input.passage_volume_config,
          time_profile, input.materialized_route_fingerprint,
          physical_route_fingerprint)};
  if (!compiledTrajectoryResourcesValid3D(*trajectory, input.route_generation)) {
    result.validation = {Failure::kInvalidPassageResources, 0U};
    return result;
  }
  if (trajectory->compiled_trajectory_revision == 0U ||
      compiledTrajectoryRevision3D(*trajectory) !=
          trajectory->compiled_trajectory_revision ||
      compiledTrajectoryPassageRevision3D(*trajectory) == 0U) {
    result.validation = {Failure::kDerivedResourceMismatch, 0U};
    return result;
  }
  result.trajectory = std::move(trajectory);
  result.validation = {Failure::kValid, route->size() - 1U};
  return result;
}

} // namespace drone_city_nav
