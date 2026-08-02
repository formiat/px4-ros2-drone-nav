#include "drone_city_nav/static_route_extension.hpp"

#include "drone_city_nav/esdf_query.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] double boundedLatencySeconds(const double latency_ms,
                                           const StaticRouteExtensionConfig& config) {
  const double measured_s = std::max(0.0, latency_ms) / 1000.0;
  return std::clamp(measured_s + std::max(0.0, config.latency_margin_s), 0.0,
                    std::max(0.0, config.maximum_latency_s));
}

} // namespace

bool StaticRouteReplanGate::tryBegin(const std::uint64_t route_generation) noexcept {
  if (route_generation == 0U || generation_ != 0U) {
    return false;
  }
  generation_ = route_generation;
  return true;
}

void StaticRouteReplanGate::finish(const std::uint64_t route_generation) noexcept {
  if (generation_ == route_generation) {
    generation_ = 0U;
  }
}

bool StaticRouteReplanGate::inFlight() const noexcept {
  return generation_ != 0U;
}

std::uint64_t StaticRouteReplanGate::generation() const noexcept {
  return generation_;
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

bool staticRoutePointInsideEsdf(const mppi::EsdfGrid& grid,
                                const Point3& point) noexcept {
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
  return point.x >= grid.origin_x_m && point.x < maximum_x &&
         point.y >= grid.origin_y_m && point.y < maximum_y &&
         point.z >= grid.origin_z_m && point.z < maximum_z;
}

StaticRouteCandidateValidation validateStaticRouteCandidate(
    const std::span<const RouteSample3D> active_route,
    const std::span<const RouteSample3D> candidate_route, const mppi::EsdfGrid& grid,
    const std::span<const float> esdf_m, const Point3& mission_goal,
    const double minimum_endpoint_improvement_m, const bool reaches_mission_goal,
    const bool required_continuation) noexcept {
  if (candidate_route.size() < 2U) {
    return {.status = StaticRouteCandidateStatus::kEmpty};
  }
  for (const RouteSample3D& sample : candidate_route) {
    const EsdfQueryResult query = queryConservativeEsdf3D(
        grid, esdf_m, static_cast<float>(sample.position.x),
        static_cast<float>(sample.position.y), static_cast<float>(sample.position.z));
    if (query.status == EsdfQueryStatus::kOutsideGrid) {
      return {.status = StaticRouteCandidateStatus::kOutsideEsdf};
    }
    if (query.status != EsdfQueryStatus::kValid) {
      return {.status = StaticRouteCandidateStatus::kInvalidEsdf};
    }
    if (query.raw_occupied) {
      return {.status = StaticRouteCandidateStatus::kRawCollision};
    }
  }
  double improvement_m = 0.0;
  if (!active_route.empty()) {
    improvement_m = distance3D(active_route.back().position, mission_goal) -
                    distance3D(candidate_route.back().position, mission_goal);
    if (!required_continuation && !reaches_mission_goal &&
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
    case StaticRouteCandidateStatus::kInvalidChannelSpan:
      return "invalid_channel_span";
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
  }
  return "unknown";
}

} // namespace drone_city_nav
