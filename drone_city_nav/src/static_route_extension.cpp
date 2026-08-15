#include "drone_city_nav/static_route_extension.hpp"

#include "drone_city_nav/esdf_query.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>

namespace drone_city_nav {
namespace {

[[nodiscard]] double boundedLatencySeconds(const double latency_ms,
                                           const StaticRouteExtensionConfig& config) {
  const double measured_s = std::max(0.0, latency_ms) / 1000.0;
  return std::clamp(measured_s + std::max(0.0, config.latency_margin_s), 0.0,
                    std::max(0.0, config.maximum_latency_s));
}

[[nodiscard]] unsigned
deferredReplanPriority(const GlobalGuideReleaseReason reason) noexcept {
  switch (reason) {
    case GlobalGuideReleaseReason::kObjectiveChanged:
      return 7U;
    case GlobalGuideReleaseReason::kNoEligibleRollouts:
      return 5U;
    case GlobalGuideReleaseReason::kDiverged:
      return 4U;
    case GlobalGuideReleaseReason::kExhausted:
      return 3U;
    case GlobalGuideReleaseReason::kStalled:
      return 2U;
    case GlobalGuideReleaseReason::kBlocked:
    case GlobalGuideReleaseReason::kNoActiveGuide:
      return 1U;
    case GlobalGuideReleaseReason::kNone:
      return 0U;
  }
  return 0U;
}

} // namespace

bool StaticRouteReplanGate::tryBegin(const std::uint64_t route_generation) noexcept {
  if (generation_.has_value()) {
    return false;
  }
  generation_ = route_generation;
  return true;
}

void StaticRouteReplanGate::finish(const std::uint64_t route_generation) noexcept {
  if (generation_ == std::optional<std::uint64_t>{route_generation}) {
    generation_.reset();
  }
}

bool StaticRouteReplanGate::inFlight() const noexcept {
  return generation_.has_value();
}

std::uint64_t StaticRouteReplanGate::generation() const noexcept {
  return generation_.value_or(0U);
}

void StaticRouteDeferredReplanLatch::defer(
    const StaticRouteDeferredReplan request) noexcept {
  if (request.reason == GlobalGuideReleaseReason::kNone ||
      request.route_generation == 0U) {
    return;
  }
  if (!request_.has_value() || request_->route_generation != request.route_generation ||
      deferredReplanPriority(request.reason) >
          deferredReplanPriority(request_->reason)) {
    request_ = request;
  }
}

std::optional<StaticRouteDeferredReplan>
StaticRouteDeferredReplanLatch::finishExtension(
    const std::uint64_t route_generation, const bool extension_activated) noexcept {
  if (!request_.has_value() || request_->route_generation != route_generation) {
    return std::nullopt;
  }
  std::optional<StaticRouteDeferredReplan> completed = request_;
  request_.reset();
  return extension_activated ? std::nullopt : completed;
}

bool StaticRouteDeferredReplanLatch::pending() const noexcept {
  return request_.has_value();
}

StaticRouteSearchRetryDecision StaticRouteFailedSearchLatch::evaluate(
    const StaticRouteSearchRetryConfig& config,
    const StaticRouteSearchContext& context) const noexcept {
  if (!failure_.has_value()) {
    return {.allow = true, .trigger = StaticRouteSearchRetryTrigger::kNoFailure};
  }

  const StaticRouteSearchContext& failure = *failure_;
  StaticRouteSearchRetryDecision decision;
  decision.pose_change_m = distance3D(failure.search_start, context.search_start);
  if (failure.objective.available && context.objective.available) {
    decision.objective_change_m =
        distance3D(failure.objective.goal, context.objective.goal);
  }
  if (context.stamp_ns > failure.stamp_ns) {
    decision.elapsed_s =
        static_cast<double>(context.stamp_ns - failure.stamp_ns) * 1.0e-9;
  }

  if (failure.base_route_generation != context.base_route_generation) {
    decision.allow = true;
    decision.trigger = StaticRouteSearchRetryTrigger::kRouteGenerationChanged;
    return decision;
  }
  const bool objective_identity_changed =
      failure.objective.available != context.objective.available ||
      failure.objective.mission_epoch != context.objective.mission_epoch ||
      failure.objective.continuous_tracking != context.objective.continuous_tracking ||
      failure.minimum_tracking_sample_sequence !=
          context.minimum_tracking_sample_sequence;
  if (objective_identity_changed ||
      decision.objective_change_m + 1.0e-9 >=
          std::max(0.0, config.minimum_objective_change_m)) {
    decision.allow = true;
    decision.trigger = StaticRouteSearchRetryTrigger::kObjectiveChanged;
    return decision;
  }
  if (decision.pose_change_m + 1.0e-9 >= std::max(0.0, config.minimum_pose_change_m)) {
    decision.allow = true;
    decision.trigger = StaticRouteSearchRetryTrigger::kPoseChanged;
    return decision;
  }
  if (decision.elapsed_s + 1.0e-9 >= std::max(0.0, config.minimum_retry_interval_s)) {
    decision.allow = true;
    decision.trigger = StaticRouteSearchRetryTrigger::kRetryIntervalElapsed;
    return decision;
  }
  return decision;
}

void StaticRouteFailedSearchLatch::recordFailure(
    const StaticRouteSearchContext& context) noexcept {
  failure_ = context;
}

void StaticRouteFailedSearchLatch::clear() noexcept {
  failure_.reset();
}

bool StaticRouteFailedSearchLatch::latched() const noexcept {
  return failure_.has_value();
}

StaticRouteRoiRefreshRequest StaticRouteRoiRefreshLifecycle::queue(
    const std::uint64_t base_route_generation,
    const StaticRouteRoiRefreshRequest::Purpose purpose) noexcept {
  if (base_route_generation == 0U) {
    return {};
  }
  const std::uint64_t sequence =
      next_sequence_.fetch_add(1U, std::memory_order_relaxed) + 1U;
  requested_base_route_generation_.store(base_route_generation,
                                         std::memory_order_relaxed);
  requested_purpose_.store(purpose, std::memory_order_relaxed);
  requested_sequence_.store(sequence, std::memory_order_release);
  return {.sequence = sequence,
          .base_route_generation = base_route_generation,
          .purpose = purpose};
}

StaticRouteRoiRefreshRequest StaticRouteRoiRefreshLifecycle::latest() const noexcept {
  const std::uint64_t sequence = requested_sequence_.load(std::memory_order_acquire);
  return {.sequence = sequence,
          .base_route_generation =
              requested_base_route_generation_.load(std::memory_order_relaxed),
          .purpose = requested_purpose_.load(std::memory_order_relaxed)};
}

bool StaticRouteRoiRefreshLifecycle::pending(
    const StaticRouteRoiRefreshRequest& request) const noexcept {
  return request.sequence != 0U &&
         request.sequence > completed_sequence_.load(std::memory_order_acquire);
}

void StaticRouteRoiRefreshLifecycle::complete(const std::uint64_t sequence) noexcept {
  std::uint64_t completed = completed_sequence_.load(std::memory_order_relaxed);
  while (completed < sequence && !completed_sequence_.compare_exchange_weak(
                                     completed, sequence, std::memory_order_release,
                                     std::memory_order_relaxed)) {
  }
}

StaticRouteExtensionDecision evaluateStaticRouteExtension(
    const StaticRouteExtensionConfig& config,
    const StaticRouteExtensionObservation& observation) noexcept {
  StaticRouteExtensionDecision decision;
  const double speed_mps = std::max(0.0, observation.horizontal_speed_mps);
  const double search_latency_s =
      boundedLatencySeconds(observation.guide_search_latency_ms, config);
  const double build_and_search_latency_s = boundedLatencySeconds(
      observation.esdf_build_latency_ms + observation.guide_search_latency_ms, config);
  decision.extension_trigger_remaining_m =
      std::max(0.0, config.minimum_remaining_m) + speed_mps * search_latency_s;
  decision.roi_refresh_trigger_remaining_m = std::max(0.0, config.minimum_remaining_m) +
                                             speed_mps * build_and_search_latency_s;

  if (observation.route_generation == 0U || observation.route_reaches_mission_goal ||
      observation.request_in_flight || !std::isfinite(observation.route_station_m) ||
      !std::isfinite(observation.route_remaining_m) ||
      observation.route_remaining_m < 0.0) {
    return decision;
  }
  if (observation.last_request_generation == observation.route_generation) {
    const bool enough_progress = observation.route_station_m >=
                                 observation.last_request_station_m +
                                     std::max(0.0, config.minimum_retry_progress_m);
    const double elapsed_s =
        observation.request_stamp_ns > observation.last_request_stamp_ns
            ? static_cast<double>(observation.request_stamp_ns -
                                  observation.last_request_stamp_ns) /
                  1.0e9
            : 0.0;
    if (!enough_progress &&
        elapsed_s < std::max(0.0, config.minimum_retry_interval_s)) {
      return decision;
    }
  }
  if (!observation.next_planning_goal_inside_esdf &&
      observation.route_remaining_m <= decision.roi_refresh_trigger_remaining_m) {
    decision.request_roi_refresh = true;
    return decision;
  }
  decision.request_extension =
      observation.route_remaining_m <= decision.extension_trigger_remaining_m;
  return decision;
}

bool deferStaticRouteReleaseDuringExtension(
    const bool request_in_flight, const GlobalGuideReleaseReason reason) noexcept {
  return request_in_flight && reason != GlobalGuideReleaseReason::kNone &&
         reason != GlobalGuideReleaseReason::kNoActiveGuide &&
         reason != GlobalGuideReleaseReason::kBlocked;
}

Point3 staticRoutePlanningGoal(const Point3& start, const Point3& mission_goal,
                               const double planning_distance_m) noexcept {
  const double full_distance = distance3D(start, mission_goal);
  if (!(planning_distance_m > 0.0) || !(full_distance > planning_distance_m)) {
    return mission_goal;
  }
  const double ratio = planning_distance_m / full_distance;
  return Point3{std::lerp(start.x, mission_goal.x, ratio),
                std::lerp(start.y, mission_goal.y, ratio),
                std::lerp(start.z, mission_goal.z, ratio)};
}

bool staticRoutePointInsideEsdf(const mppi::EsdfGrid& grid, const Point3& point,
                                const double margin_m) noexcept {
  if (grid.width <= 0 || grid.height <= 0 || grid.depth <= 1 ||
      !(grid.resolution_m > 0.0F)) {
    return false;
  }
  const double maximum_x = static_cast<double>(grid.origin_x_m) +
                           static_cast<double>(grid.width) * grid.resolution_m;
  const double maximum_y = static_cast<double>(grid.origin_y_m) +
                           static_cast<double>(grid.height) * grid.resolution_m;
  const double maximum_z = static_cast<double>(grid.origin_z_m) +
                           static_cast<double>(grid.depth) * grid.resolution_m;
  const double margin = std::max(0.0, margin_m);
  return point.x >= static_cast<double>(grid.origin_x_m) + margin &&
         point.x < maximum_x - margin &&
         point.y >= static_cast<double>(grid.origin_y_m) + margin &&
         point.y < maximum_y - margin &&
         point.z >= static_cast<double>(grid.origin_z_m) && point.z < maximum_z;
}

bool staticRouteObjectiveMatches(const StaticRouteObjective& route_objective,
                                 const StaticRouteObjective& current_objective,
                                 const std::uint64_t minimum_tracking_sample_sequence,
                                 const double maximum_tracking_goal_error_m) noexcept {
  if (!route_objective.available || !current_objective.available ||
      route_objective.mission_epoch != current_objective.mission_epoch ||
      route_objective.continuous_tracking != current_objective.continuous_tracking) {
    return false;
  }
  if (!current_objective.continuous_tracking) {
    return true;
  }
  return route_objective.sample_sequence >= minimum_tracking_sample_sequence &&
         distance3D(route_objective.goal, current_objective.goal) <=
             std::max(0.0, maximum_tracking_goal_error_m);
}

bool staticRouteHasProtectedConstrainedSuffix(
    const std::span<const RouteSample3D> route,
    const std::span<const ConstrainedRouteSpan> constrained_spans,
    const Point3& current_position, const double protected_departure_m) noexcept {
  const RouteProjection3D projection = projectOntoRoute3D(route, current_position);
  if (!projection.valid) {
    return false;
  }
  const double departure_m = std::max(0.0, protected_departure_m);
  return std::ranges::any_of(constrained_spans, [&](const ConstrainedRouteSpan& span) {
    return projection.station_m + 1.0e-9 >= span.begin_station_m - departure_m &&
           projection.station_m <= span.end_station_m + departure_m;
  });
}

StaticRouteCandidateValidation validateStaticRouteCandidate(
    const std::span<const RouteSample3D> active_route,
    const std::span<const RouteSample3D> candidate_route, const mppi::EsdfGrid& grid,
    const std::span<const float> esdf_m, const Point3& mission_goal,
    const double minimum_endpoint_improvement_m, const bool reaches_mission_goal,
    const FlightEnvelopeConfig& flight_envelope,
    const StaticRouteReplacementPolicy replacement_policy,
    const SweptFootprintConfig& footprint_config) noexcept {
  if (candidate_route.size() < 2U) {
    return {.status = StaticRouteCandidateStatus::kEmpty};
  }
  if (!std::ranges::all_of(candidate_route, [&](const RouteSample3D& sample) {
        return insideFlightEnvelope(sample.position, flight_envelope);
      })) {
    return {.status = StaticRouteCandidateStatus::kOutsideFlightEnvelope};
  }
  for (std::size_t index = 1U; index < candidate_route.size(); ++index) {
    const SweptFootprintResult footprint =
        validateSweptFootprint(grid, esdf_m, candidate_route[index - 1U].position,
                               candidate_route[index].position, footprint_config);
    if (footprint.status == SweptFootprintStatus::kOutsideGrid) {
      return {.status = StaticRouteCandidateStatus::kOutsideEsdf};
    }
    if (footprint.status == SweptFootprintStatus::kInvalidEsdf) {
      return {.status = StaticRouteCandidateStatus::kInvalidEsdf};
    }
    if (footprint.status == SweptFootprintStatus::kRawCollision) {
      return {.status = StaticRouteCandidateStatus::kRawCollision};
    }
  }
  double improvement_m = 0.0;
  if (!active_route.empty()) {
    improvement_m = distance3D(active_route.back().position, mission_goal) -
                    distance3D(candidate_route.back().position, mission_goal);
    if (replacement_policy ==
            StaticRouteReplacementPolicy::kRequireEndpointImprovement &&
        !reaches_mission_goal &&
        improvement_m + 1.0e-9 < std::max(0.0, minimum_endpoint_improvement_m)) {
      return {.status = StaticRouteCandidateStatus::kNoEndpointImprovement,
              .endpoint_improvement_m = improvement_m};
    }
  }
  return {.status = StaticRouteCandidateStatus::kAccepted,
          .endpoint_improvement_m = improvement_m,
          .accepted = true};
}

std::string_view
staticRouteReplacementPolicyName(const StaticRouteReplacementPolicy policy) noexcept {
  switch (policy) {
    case StaticRouteReplacementPolicy::kRequireEndpointImprovement:
      return "require_endpoint_improvement";
    case StaticRouteReplacementPolicy::kAllowSafetyReplan:
      return "allow_safety_replan";
  }
  return "unknown";
}

std::string_view
staticRouteCandidateStatusName(const StaticRouteCandidateStatus status) noexcept {
  switch (status) {
    case StaticRouteCandidateStatus::kAccepted:
      return "accepted";
    case StaticRouteCandidateStatus::kEmpty:
      return "empty";
    case StaticRouteCandidateStatus::kOutsideEsdf:
      return "outside_esdf";
    case StaticRouteCandidateStatus::kInvalidEsdf:
      return "invalid_esdf";
    case StaticRouteCandidateStatus::kRawCollision:
      return "raw_collision";
    case StaticRouteCandidateStatus::kOutsideFlightEnvelope:
      return "outside_flight_envelope";
    case StaticRouteCandidateStatus::kInvalidChannelSpan:
      return "invalid_channel_span";
    case StaticRouteCandidateStatus::kProtectedConstrainedSuffix:
      return "protected_constrained_suffix";
    case StaticRouteCandidateStatus::kNoEndpointImprovement:
      return "no_endpoint_improvement";
  }
  return "unknown";
}

std::string_view
staticRouteActivationStatusName(const StaticRouteActivationStatus status) noexcept {
  switch (status) {
    case StaticRouteActivationStatus::kNotAttempted:
      return "not_attempted";
    case StaticRouteActivationStatus::kActivated:
      return "activated";
    case StaticRouteActivationStatus::kCandidateNotExecutable:
      return "candidate_not_executable";
    case StaticRouteActivationStatus::kCandidateValidationRejected:
      return "candidate_validation_rejected";
    case StaticRouteActivationStatus::kStaleWorldRevision:
      return "stale_world_revision";
    case StaticRouteActivationStatus::kStaleRouteGeneration:
      return "stale_route_generation";
    case StaticRouteActivationStatus::kStaleObjective:
      return "stale_objective";
    case StaticRouteActivationStatus::kDynamicHandoffRejected:
      return "dynamic_handoff_rejected";
  }
  return "unknown";
}

std::string_view staticRouteSearchRetryTriggerName(
    const StaticRouteSearchRetryTrigger trigger) noexcept {
  switch (trigger) {
    case StaticRouteSearchRetryTrigger::kNoFailure:
      return "no_failure";
    case StaticRouteSearchRetryTrigger::kRouteGenerationChanged:
      return "route_generation_changed";
    case StaticRouteSearchRetryTrigger::kObjectiveChanged:
      return "objective_changed";
    case StaticRouteSearchRetryTrigger::kPoseChanged:
      return "pose_changed";
    case StaticRouteSearchRetryTrigger::kRetryIntervalElapsed:
      return "retry_interval_elapsed";
    case StaticRouteSearchRetryTrigger::kSuppressed:
      return "suppressed";
  }
  return "unknown";
}

} // namespace drone_city_nav
