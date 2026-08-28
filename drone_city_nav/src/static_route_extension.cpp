#include "drone_city_nav/static_route_extension.hpp"

#include "drone_city_nav/esdf_query.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool
extensionConfigValidImpl(const StaticRouteExtensionConfig& config) noexcept {
  return std::isfinite(config.minimum_remaining_m) &&
         config.minimum_remaining_m >= 0.0 &&
         std::isfinite(config.required_certified_overlap_m) &&
         config.required_certified_overlap_m > 0.0 &&
         std::isfinite(config.latency_margin_s) && config.latency_margin_s >= 0.0 &&
         std::isfinite(config.maximum_latency_s) && config.maximum_latency_s > 0.0 &&
         std::isfinite(config.maximum_horizontal_acceleration_mps2) &&
         config.maximum_horizontal_acceleration_mps2 > 0.0 &&
         std::isfinite(config.maximum_vertical_acceleration_mps2) &&
         config.maximum_vertical_acceleration_mps2 > 0.0 &&
         std::isfinite(config.maximum_control_jerk_mps3) &&
         config.maximum_control_jerk_mps3 > 0.0 &&
         stoppingCapabilityIsValid(config.stopping_capability);
}

template<std::size_t Size>
[[nodiscard]] double percentile(const std::array<double, Size>& samples,
                                const std::size_t sample_count,
                                const double quantile) noexcept {
  if (sample_count == 0U || sample_count > samples.size() || !std::isfinite(quantile) ||
      quantile <= 0.0 || quantile > 1.0) {
    return 0.0;
  }
  std::array<double, Size> ordered = samples;
  std::ranges::sort(ordered.begin(), ordered.begin() + sample_count);
  const std::size_t rank =
      static_cast<std::size_t>(std::ceil(quantile * static_cast<double>(sample_count)));
  return ordered[std::clamp<std::size_t>(rank, 1U, sample_count) - 1U];
}

[[nodiscard]] double boundedLatencySeconds(const double latency_ms,
                                           const StaticRouteExtensionConfig& config) {
  const double measured_s = std::max(0.0, latency_ms) / 1000.0;
  return std::clamp(measured_s + std::max(0.0, config.latency_margin_s), 0.0,
                    std::max(0.0, config.maximum_latency_s));
}

[[nodiscard]] double jerkLimitedAxisStoppingDistanceM(
    const double speed_mps, const double forward_acceleration_mps2,
    const double guaranteed_deceleration_mps2, const double maximum_acceleration_mps2,
    const double maximum_jerk_mps3, const double reaction_latency_s) noexcept {
  if (!std::isfinite(speed_mps) || speed_mps < 0.0 ||
      !std::isfinite(forward_acceleration_mps2) ||
      !std::isfinite(guaranteed_deceleration_mps2) ||
      !(guaranteed_deceleration_mps2 > 0.0) ||
      !std::isfinite(maximum_acceleration_mps2) || !(maximum_acceleration_mps2 > 0.0) ||
      !std::isfinite(maximum_jerk_mps3) || !(maximum_jerk_mps3 > 0.0) ||
      !std::isfinite(reaction_latency_s) || reaction_latency_s < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  if (!(speed_mps > 0.0)) {
    return 0.0;
  }

  const double forward_acceleration = std::clamp(
      std::max(0.0, forward_acceleration_mps2), 0.0, maximum_acceleration_mps2);
  const double reaction_distance_m =
      speed_mps * reaction_latency_s +
      0.5 * forward_acceleration * reaction_latency_s * reaction_latency_s;
  const double speed_after_reaction_mps =
      speed_mps + forward_acceleration * reaction_latency_s;
  const double ramp_time_s =
      (forward_acceleration + guaranteed_deceleration_mps2) / maximum_jerk_mps3;
  const double stop_during_ramp_s =
      (forward_acceleration +
       std::sqrt(forward_acceleration * forward_acceleration +
                 2.0 * maximum_jerk_mps3 * speed_after_reaction_mps)) /
      maximum_jerk_mps3;
  const double applied_ramp_time_s = std::min(ramp_time_s, stop_during_ramp_s);
  const double ramp_time_squared_s2 = applied_ramp_time_s * applied_ramp_time_s;
  const double ramp_distance_m =
      speed_after_reaction_mps * applied_ramp_time_s +
      0.5 * forward_acceleration * ramp_time_squared_s2 -
      maximum_jerk_mps3 * ramp_time_squared_s2 * applied_ramp_time_s / 6.0;
  if (stop_during_ramp_s <= ramp_time_s) {
    return std::max(0.0, reaction_distance_m + ramp_distance_m);
  }
  const double speed_after_ramp_mps =
      speed_after_reaction_mps + forward_acceleration * ramp_time_s -
      0.5 * maximum_jerk_mps3 * ramp_time_s * ramp_time_s;
  const double constant_deceleration_distance_m = speed_after_ramp_mps *
                                                  speed_after_ramp_mps /
                                                  (2.0 * guaranteed_deceleration_mps2);
  return std::max(0.0, reaction_distance_m + ramp_distance_m +
                           constant_deceleration_distance_m);
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

bool staticRouteExtensionConfigValid(
    const StaticRouteExtensionConfig& config) noexcept {
  return extensionConfigValidImpl(config);
}

void StaticRoutePlanningLatencyTracker::record(
    const double planning_latency_ms, const double world_build_latency_ms) noexcept {
  if (!std::isfinite(planning_latency_ms) || planning_latency_ms < 0.0 ||
      !std::isfinite(world_build_latency_ms) || world_build_latency_ms < 0.0) {
    return;
  }
  planning_latency_ms_[next_index_] = planning_latency_ms;
  build_and_planning_latency_ms_[next_index_] =
      planning_latency_ms + world_build_latency_ms;
  next_index_ = (next_index_ + 1U) % kMaximumSamples;
  sample_count_ = std::min(sample_count_ + 1U, kMaximumSamples);
}

StaticRoutePlanningLatencyStats
StaticRoutePlanningLatencyTracker::stats() const noexcept {
  return StaticRoutePlanningLatencyStats{
      .sample_count = sample_count_,
      .planning_p95_ms = percentile(planning_latency_ms_, sample_count_, 0.95),
      .planning_p99_ms = percentile(planning_latency_ms_, sample_count_, 0.99),
      .build_and_planning_p99_ms =
          percentile(build_and_planning_latency_ms_, sample_count_, 0.99),
  };
}

void StaticRoutePlanningLatencyTracker::clear() noexcept {
  planning_latency_ms_.fill(0.0);
  build_and_planning_latency_ms_.fill(0.0);
  next_index_ = 0U;
  sample_count_ = 0U;
}

double jerkLimitedHorizontalStoppingDistanceM(
    const double horizontal_speed_mps, const double forward_acceleration_mps2,
    const StoppingCapability& capability,
    const double maximum_horizontal_acceleration_mps2,
    const double maximum_control_jerk_mps3) noexcept {
  if (!stoppingCapabilityIsValid(capability)) {
    return std::numeric_limits<double>::infinity();
  }
  return jerkLimitedAxisStoppingDistanceM(
      horizontal_speed_mps, forward_acceleration_mps2,
      capability.guaranteed_horizontal_deceleration_mps2,
      maximum_horizontal_acceleration_mps2, maximum_control_jerk_mps3,
      capability.reaction_latency_s);
}

bool JerkLimitedStoppingDistance3D::valid() const noexcept {
  return std::isfinite(horizontal_m) && horizontal_m >= 0.0 &&
         std::isfinite(vertical_m) && vertical_m >= 0.0 &&
         std::isfinite(route_station_m) && route_station_m >= horizontal_m &&
         route_station_m >= vertical_m;
}

JerkLimitedStoppingDistance3D
jerkLimitedStoppingDistance3D(const double horizontal_speed_mps,
                              const double forward_horizontal_acceleration_mps2,
                              const double vertical_speed_mps,
                              const double forward_vertical_acceleration_mps2,
                              const StaticRouteExtensionConfig& config) noexcept {
  if (!staticRouteExtensionConfigValid(config)) {
    return {.horizontal_m = std::numeric_limits<double>::infinity(),
            .vertical_m = std::numeric_limits<double>::infinity(),
            .route_station_m = std::numeric_limits<double>::infinity()};
  }
  const double horizontal_m = jerkLimitedAxisStoppingDistanceM(
      horizontal_speed_mps, forward_horizontal_acceleration_mps2,
      config.stopping_capability.guaranteed_horizontal_deceleration_mps2,
      config.maximum_horizontal_acceleration_mps2, config.maximum_control_jerk_mps3,
      config.stopping_capability.reaction_latency_s);
  const double vertical_m = jerkLimitedAxisStoppingDistanceM(
      vertical_speed_mps, forward_vertical_acceleration_mps2,
      config.stopping_capability.guaranteed_vertical_deceleration_mps2,
      config.maximum_vertical_acceleration_mps2, config.maximum_control_jerk_mps3,
      config.stopping_capability.reaction_latency_s);
  return {.horizontal_m = horizontal_m,
          .vertical_m = vertical_m,
          .route_station_m = horizontal_m + vertical_m};
}

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

std::uint64_t
staticRouteSearchGeneration(const bool snapshot_owned_execution,
                            const std::uint64_t prepared_guide_generation,
                            const std::uint64_t committed_route_generation) noexcept {
  return snapshot_owned_execution ? committed_route_generation
                                  : prepared_guide_generation;
}

bool StaticRouteSearchRequestIdentity::valid() const noexcept {
  return kind != StaticRouteSearchRequestKind::kInvalid;
}

bool StaticRouteSearchCurrencyAssessment::current() const noexcept {
  return status == StaticRouteSearchCurrencyStatus::kCurrent;
}

StaticRouteSearchRequestIdentity identifyStaticRouteSearchRequest(
    const std::uint64_t world_route_generation, const bool extension_request,
    const std::uint64_t extension_base_generation, const bool replan_request,
    const std::uint64_t replan_base_generation) noexcept {
  if (extension_request && replan_request) {
    return {};
  }
  if (extension_request) {
    if (extension_base_generation == 0U ||
        extension_base_generation != world_route_generation) {
      return {};
    }
    return {.kind = StaticRouteSearchRequestKind::kExtension,
            .base_route_generation = extension_base_generation};
  }
  if (replan_request) {
    if (replan_base_generation != world_route_generation) {
      return {};
    }
    return {.kind = replan_base_generation == 0U
                        ? StaticRouteSearchRequestKind::kInitialRetry
                        : StaticRouteSearchRequestKind::kReplan,
            .base_route_generation = replan_base_generation};
  }
  return {.kind = world_route_generation == 0U
                      ? StaticRouteSearchRequestKind::kInitial
                      : StaticRouteSearchRequestKind::kResidentRefresh,
          .base_route_generation = world_route_generation};
}

StaticRouteSearchCurrencyAssessment assessStaticRouteSearchCurrency(
    const StaticRouteSearchRequestIdentity& request,
    const std::uint64_t resident_route_generation) noexcept {
  StaticRouteSearchCurrencyAssessment result{
      .request = request,
      .resident_route_generation = resident_route_generation,
  };
  if (!request.valid()) {
    return result;
  }
  if (resident_route_generation == request.base_route_generation) {
    result.status = StaticRouteSearchCurrencyStatus::kCurrent;
  } else if (resident_route_generation > request.base_route_generation) {
    result.status = StaticRouteSearchCurrencyStatus::kSupersededByResidentRoute;
  } else {
    result.status = StaticRouteSearchCurrencyStatus::kResidentRoutePredatesRequest;
  }
  return result;
}

bool staticRouteSearchFailureLatchEligible(
    const StaticRouteSearchRequestIdentity& request,
    const std::uint64_t resident_route_generation, const bool world_compatible,
    const bool objective_matches) noexcept {
  return assessStaticRouteSearchCurrency(request, resident_route_generation)
             .current() &&
         world_compatible && objective_matches;
}

std::string_view
staticRouteSearchRequestKindName(const StaticRouteSearchRequestKind kind) noexcept {
  switch (kind) {
    case StaticRouteSearchRequestKind::kInvalid:
      return "invalid";
    case StaticRouteSearchRequestKind::kInitial:
      return "initial";
    case StaticRouteSearchRequestKind::kInitialRetry:
      return "initial_retry";
    case StaticRouteSearchRequestKind::kResidentRefresh:
      return "resident_refresh";
    case StaticRouteSearchRequestKind::kExtension:
      return "extension";
    case StaticRouteSearchRequestKind::kReplan:
      return "replan";
  }
  return "unknown";
}

std::string_view staticRouteSearchCurrencyStatusName(
    const StaticRouteSearchCurrencyStatus status) noexcept {
  switch (status) {
    case StaticRouteSearchCurrencyStatus::kCurrent:
      return "current";
    case StaticRouteSearchCurrencyStatus::kInvalidRequest:
      return "invalid_request";
    case StaticRouteSearchCurrencyStatus::kSupersededByResidentRoute:
      return "superseded_by_resident_route";
    case StaticRouteSearchCurrencyStatus::kResidentRoutePredatesRequest:
      return "resident_route_predates_request";
  }
  return "unknown";
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

std::optional<StaticRouteDeferredReplan>
StaticRouteDeferredReplanLatch::finishReplan(const std::uint64_t route_generation,
                                             const bool route_activated) noexcept {
  if (!request_.has_value() || request_->route_generation != route_generation) {
    return std::nullopt;
  }
  std::optional<StaticRouteDeferredReplan> completed = request_;
  request_.reset();
  if (route_activated &&
      completed->reason == GlobalGuideReleaseReason::kNoActiveGuide) {
    return std::nullopt;
  }
  return completed;
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
      !staticRouteAssignmentMatches(failure.objective, context.objective) ||
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

bool CertifiedRouteReserveAssessment3D::accepted() const noexcept {
  return status == CertifiedRouteReserveStatus3D::kSufficient ||
         status == CertifiedRouteReserveStatus3D::kTerminalExempt;
}

CertifiedRouteReserveAssessment3D assessCertifiedRouteReserve3D(
    const StaticRouteExtensionDecision& decision, const double available_route_m,
    const RouteEndpointSemantics3D endpoint_semantics) noexcept {
  CertifiedRouteReserveAssessment3D result{
      .available_m = available_route_m,
      .required_m = decision.extension_trigger_remaining_m,
  };
  if (!std::isfinite(result.available_m) || result.available_m < 0.0 ||
      !decision.valid || !std::isfinite(result.required_m) || result.required_m < 0.0 ||
      !std::isfinite(decision.braking_path_m) ||
      !std::isfinite(decision.required_certified_overlap_m)) {
    return result;
  }
  switch (endpoint_semantics) {
    case RouteEndpointSemantics3D::kMissionStop:
    case RouteEndpointSemantics3D::kEmergencyBrakeTail:
      result.status = CertifiedRouteReserveStatus3D::kTerminalExempt;
      return result;
    case RouteEndpointSemantics3D::kContinuation:
    case RouteEndpointSemantics3D::kObservationStop:
      break;
    default:
      return result;
  }
  result.shortfall_m = std::max(0.0, result.required_m - result.available_m);
  result.status = result.shortfall_m <= 1.0e-6
                      ? CertifiedRouteReserveStatus3D::kSufficient
                      : CertifiedRouteReserveStatus3D::kInsufficient;
  return result;
}

std::string_view
certifiedRouteReserveStatus3DName(const CertifiedRouteReserveStatus3D status) noexcept {
  switch (status) {
    case CertifiedRouteReserveStatus3D::kSufficient:
      return "sufficient";
    case CertifiedRouteReserveStatus3D::kTerminalExempt:
      return "terminal_exempt";
    case CertifiedRouteReserveStatus3D::kInsufficient:
      return "insufficient";
    case CertifiedRouteReserveStatus3D::kInvalid:
      return "invalid";
  }
  return "invalid";
}

StaticRouteExtensionDecision evaluateStaticRouteExtension(
    const StaticRouteExtensionConfig& config,
    const StaticRouteExtensionObservation& observation) noexcept {
  StaticRouteExtensionDecision decision;
  if (!staticRouteExtensionConfigValid(config) ||
      !std::isfinite(observation.horizontal_speed_mps) ||
      !std::isfinite(observation.forward_acceleration_mps2) ||
      !std::isfinite(observation.vertical_speed_mps) ||
      !std::isfinite(observation.forward_vertical_acceleration_mps2) ||
      !std::isfinite(observation.planning_latency_p95_ms) ||
      !std::isfinite(observation.planning_latency_p99_ms) ||
      !std::isfinite(observation.build_and_planning_latency_p99_ms)) {
    return decision;
  }
  const double horizontal_speed_mps = std::max(0.0, observation.horizontal_speed_mps);
  const double vertical_speed_mps = std::max(0.0, observation.vertical_speed_mps);
  const double speed_mps = std::hypot(horizontal_speed_mps, vertical_speed_mps);
  const double planning_p95_s =
      boundedLatencySeconds(observation.planning_latency_p95_ms, config);
  const double planning_p99_s =
      boundedLatencySeconds(std::max(observation.planning_latency_p95_ms,
                                     observation.planning_latency_p99_ms),
                            config);
  const double build_and_planning_p99_s =
      boundedLatencySeconds(std::max(observation.planning_latency_p99_ms,
                                     observation.build_and_planning_latency_p99_ms),
                            config);
  const JerkLimitedStoppingDistance3D stopping = jerkLimitedStoppingDistance3D(
      horizontal_speed_mps, observation.forward_acceleration_mps2, vertical_speed_mps,
      observation.forward_vertical_acceleration_mps2, config);
  decision.horizontal_braking_path_m = stopping.horizontal_m;
  decision.vertical_braking_path_m = stopping.vertical_m;
  decision.braking_path_m = stopping.route_station_m;
  decision.required_certified_overlap_m = config.required_certified_overlap_m;
  if (!stopping.valid()) {
    return {};
  }
  const double protected_distance_m =
      decision.required_certified_overlap_m + decision.braking_path_m;
  decision.planning_p95_trigger_remaining_m = std::max(
      config.minimum_remaining_m, protected_distance_m + speed_mps * planning_p95_s);
  decision.extension_trigger_remaining_m = std::max(
      config.minimum_remaining_m, protected_distance_m + speed_mps * planning_p99_s);
  decision.roi_refresh_trigger_remaining_m =
      std::max(config.minimum_remaining_m,
               protected_distance_m + speed_mps * build_and_planning_p99_s);
  decision.valid = true;

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

ObservationRouteReplacementDecision evaluateObservationRouteReplacement(
    const ObservationRouteReplacementObservation& observation) noexcept {
  ObservationRouteReplacementDecision decision;
  decision.score_improvement = observation.active_score - observation.candidate_score;
  if (!observation.candidate_frontier.has_value() ||
      observation.candidate_frontier->id.value == 0U) {
    return decision;
  }
  if (!observation.active_frontier.has_value() ||
      observation.active_frontier->id.value == 0U) {
    decision.status = ObservationRouteReplacementStatus::kNoActiveFrontier;
    decision.accepted = true;
    return decision;
  }
  if (observation.candidate_frontier->supporting_map_revision <
      observation.active_frontier->supporting_map_revision) {
    decision.status = ObservationRouteReplacementStatus::kStaleCandidate;
    return decision;
  }
  if (observation.active_route_exhausted) {
    decision.status = ObservationRouteReplacementStatus::kActiveRouteExhausted;
    decision.accepted = true;
    return decision;
  }
  if (!observation.active_frontier_still_valid) {
    decision.status = ObservationRouteReplacementStatus::kActiveFrontierRetired;
    decision.accepted = true;
    return decision;
  }
  if (observation.candidate_frontier->id == observation.active_frontier->id) {
    decision.status = ObservationRouteReplacementStatus::kSameFrontierRetained;
    decision.accepted = observation.route_extension_requested;
    return decision;
  }
  if (observation.active_frontier_reached) {
    decision.status = ObservationRouteReplacementStatus::kActiveFrontierReached;
    decision.accepted = true;
    return decision;
  }
  // Mission-goal proximity is only a soft term in the frontier score. A
  // separate endpoint gate reintroduces goal-monotonic replacement and can
  // discard a still-useful observation route before it has exposed its
  // boundary. Completed, retired, and exhausted routes are handled above.
  if (decision.score_improvement + 1.0e-9 >=
      std::max(0.0, observation.minimum_score_improvement)) {
    decision.status = ObservationRouteReplacementStatus::kScoreImproved;
    decision.accepted = true;
    return decision;
  }
  decision.status = ObservationRouteReplacementStatus::kInsufficientProgress;
  return decision;
}

std::string_view observationRouteReplacementStatusName(
    const ObservationRouteReplacementStatus status) noexcept {
  switch (status) {
    case ObservationRouteReplacementStatus::kInvalidCandidate:
      return "invalid_candidate";
    case ObservationRouteReplacementStatus::kNoActiveFrontier:
      return "no_active_frontier";
    case ObservationRouteReplacementStatus::kActiveFrontierRetired:
      return "active_frontier_retired";
    case ObservationRouteReplacementStatus::kActiveFrontierReached:
      return "active_frontier_reached";
    case ObservationRouteReplacementStatus::kActiveRouteExhausted:
      return "active_route_exhausted";
    case ObservationRouteReplacementStatus::kScoreImproved:
      return "score_improved";
    case ObservationRouteReplacementStatus::kSameFrontierRetained:
      return "same_frontier_retained";
    case ObservationRouteReplacementStatus::kStaleCandidate:
      return "stale_candidate";
    case ObservationRouteReplacementStatus::kInsufficientProgress:
      return "insufficient_progress";
  }
  return "unknown";
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

bool staticRouteAssignmentMatches(const StaticRouteObjective& first,
                                  const StaticRouteObjective& second) noexcept {
  if (!first.continuous_tracking && !second.continuous_tracking) {
    return true;
  }
  return first.assignment_generation != 0U &&
         first.assignment_generation == second.assignment_generation &&
         first.target_detection_id != 0U &&
         first.target_detection_id == second.target_detection_id &&
         first.target_track_id != 0U && first.target_track_id == second.target_track_id;
}

bool staticRouteObjectiveMatches(const StaticRouteObjective& route_objective,
                                 const StaticRouteObjective& current_objective,
                                 const std::uint64_t minimum_tracking_sample_sequence,
                                 const double maximum_tracking_goal_error_m) noexcept {
  if (!route_objective.available || !current_objective.available ||
      route_objective.mission_epoch != current_objective.mission_epoch ||
      route_objective.continuous_tracking != current_objective.continuous_tracking ||
      !staticRouteAssignmentMatches(route_objective, current_objective)) {
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

bool staticRouteReplacementProtected(
    const std::span<const RouteSample3D> route,
    const std::span<const ConstrainedRouteSpan> constrained_spans,
    const Point3& current_position, const StaticRouteObjective& route_objective,
    const StaticRouteObjective& search_objective,
    const double protected_departure_m) noexcept {
  const bool same_assignment =
      route_objective.available && search_objective.available &&
      route_objective.mission_epoch == search_objective.mission_epoch &&
      staticRouteAssignmentMatches(route_objective, search_objective);
  return same_assignment &&
         staticRouteHasProtectedConstrainedSuffix(
             route, constrained_spans, current_position, protected_departure_m);
}

StaticRouteCandidateValidation validateStaticRouteCandidate(
    const std::span<const RouteSample3D> active_route,
    const std::span<const RouteSample3D> candidate_route, const mppi::EsdfGrid& grid,
    const std::span<const float> esdf_m, const Point3& mission_goal,
    const double minimum_endpoint_improvement_m, const bool reaches_mission_goal,
    const FlightEnvelopeConfig& flight_envelope,
    const StaticRouteReplacementPolicy replacement_policy,
    const SweptFootprintConfig& footprint_config, const bool require_known_free_space,
    const bool raw_occupancy_authoritative) noexcept {
  if (candidate_route.size() < 2U) {
    return {.status = StaticRouteCandidateStatus::kEmpty};
  }
  if (!std::ranges::all_of(candidate_route, [&](const RouteSample3D& sample) {
        return insideFlightEnvelope(sample.position, flight_envelope);
      })) {
    return {.status = StaticRouteCandidateStatus::kOutsideFlightEnvelope};
  }
  for (std::size_t index = 1U;
       !raw_occupancy_authoritative && index < candidate_route.size(); ++index) {
    const SweptFootprintResult footprint =
        validateSweptFootprint(grid, esdf_m, candidate_route[index - 1U].position,
                               candidate_route[index].position, footprint_config);
    if (footprint.status == SweptFootprintStatus::kOutsideGrid ||
        (footprint.status == SweptFootprintStatus::kUnknownSpace &&
         require_known_free_space)) {
      return {.status = StaticRouteCandidateStatus::kOutsideEsdf,
              .failure_segment_index = index - 1U,
              .failure_point = footprint.failure_point};
    }
    if (footprint.status == SweptFootprintStatus::kInvalidEsdf) {
      return {.status = StaticRouteCandidateStatus::kInvalidEsdf,
              .failure_segment_index = index - 1U,
              .failure_point = footprint.failure_point};
    }
    if (footprint.status == SweptFootprintStatus::kRawCollision) {
      return {.status = StaticRouteCandidateStatus::kRawCollision,
              .failure_segment_index = index - 1U,
              .failure_point = footprint.failure_point};
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
    case StaticRouteReplacementPolicy::kAllowAnyValidatedReplacement:
      return "allow_any_validated_replacement";
    case StaticRouteReplacementPolicy::kAllowSafetyReplan:
      return "allow_safety_replan";
    case StaticRouteReplacementPolicy::kAllowTopologicalProgress:
      return "allow_topological_progress";
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
    case StaticRouteCandidateStatus::kInvalidPassageSpan:
      return "invalid_passage_span";
    case StaticRouteCandidateStatus::kProtectedConstrainedSuffix:
      return "protected_constrained_suffix";
    case StaticRouteCandidateStatus::kInvalidCertifiedReserve:
      return "invalid_certified_reserve";
    case StaticRouteCandidateStatus::kInsufficientCertifiedReserve:
      return "insufficient_certified_reserve";
    case StaticRouteCandidateStatus::kNoEndpointImprovement:
      return "no_endpoint_improvement";
    case StaticRouteCandidateStatus::kNoExplorationProgress:
      return "no_exploration_progress";
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
    case StaticRouteActivationStatus::kCertifiedPending:
      return "certified_pending";
    case StaticRouteActivationStatus::kCandidateNotExecutable:
      return "candidate_not_executable";
    case StaticRouteActivationStatus::kCandidateValidationRejected:
      return "candidate_validation_rejected";
    case StaticRouteActivationStatus::kWorldPublicationRejected:
      return "world_publication_rejected";
    case StaticRouteActivationStatus::kActivationSnapshotSuperseded:
      return "activation_snapshot_superseded";
    case StaticRouteActivationStatus::kStaleRouteGeneration:
      return "stale_route_generation";
    case StaticRouteActivationStatus::kStaleObjective:
      return "stale_objective";
    case StaticRouteActivationStatus::kInvalidExecutionGeometry:
      return "invalid_execution_geometry";
    case StaticRouteActivationStatus::kDynamicHandoffRejected:
      return "dynamic_handoff_rejected";
    case StaticRouteActivationStatus::kCertifiedSpliceRejected:
      return "certified_splice_rejected";
    case StaticRouteActivationStatus::kEquivalentActiveSegmentRetained:
      return "equivalent_active_segment_retained";
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
