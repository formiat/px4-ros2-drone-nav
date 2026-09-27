#include "drone_city_nav/mission_goal_capture.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool sameMission(const Point3& first, const Point3& second) noexcept {
  constexpr double kMissionEqualityToleranceM{1.0e-6};
  return std::abs(first.x - second.x) <= kMissionEqualityToleranceM &&
         std::abs(first.y - second.y) <= kMissionEqualityToleranceM &&
         std::abs(first.z - second.z) <= kMissionEqualityToleranceM;
}

// The goal limiter brings the vehicle to rest 0.125 m before the goal; over
// fifteen lit flights it latched 0.07 to 0.54 m from the goal by estimate
// (r737 to r765).
constexpr double kApproachEndRadiusM{0.75};
// Resting without closing this much on the goal for this long, the vehicle
// has come as close as it will.
constexpr double kRestingProgressM{0.1};
constexpr std::int64_t kRestingStallNs{3'000'000'000};

[[nodiscard]] bool finiteMission(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

} // namespace

MissionGoalCaptureLatch::MissionGoalCaptureLatch(const MissionGoalCaptureConfig& config)
    : config_{config} {
  if (!(config_.capture_radius_m > 0.0)) {
    throw std::invalid_argument{"mission goal capture radius must be positive"};
  }
  if (!(config_.stationary_speed_tolerance_mps > 0.0)) {
    throw std::invalid_argument{
        "mission goal capture stationary speed tolerance must be positive"};
  }
}

MissionGoalCaptureResult
MissionGoalCaptureLatch::update(const MissionGoalCaptureObservation& observation) {
  MissionGoalCaptureResult result;
  if (!finiteMission(observation.mission_goal) || !std::isfinite(observation.state.x) ||
      !std::isfinite(observation.state.y) || !std::isfinite(observation.state.z) ||
      !std::isfinite(observation.state.vx) || !std::isfinite(observation.state.vy) ||
      !std::isfinite(observation.state.vz)) {
    return result;
  }
  if (!mission_initialized_ || !sameMission(mission_goal_, observation.mission_goal)) {
    mission_goal_ = observation.mission_goal;
    mission_initialized_ = true;
    latched_ = false;
    resting_closest_m_.reset();
  }

  result.distance_m =
      distance3D(Point3{observation.state.x, observation.state.y, observation.state.z},
                 mission_goal_);
  result.speed_mps = std::hypot(std::hypot(observation.state.vx, observation.state.vy),
                                observation.state.vz);
  const bool inside_capture_radius = result.distance_m <= config_.capture_radius_m;
  const bool resting_inside_capture_radius =
      inside_capture_radius &&
      result.speed_mps <= config_.stationary_speed_tolerance_mps;
  if (latched_ && !inside_capture_radius) {
    // Outside the radius the capture cannot be acknowledged, and only a route
    // brings the vehicle back inside: the goal is open again until the
    // vehicle rests inside the radius once more.
    latched_ = false;
  }
  if (!resting_inside_capture_radius) {
    resting_closest_m_.reset();
  } else if (!resting_closest_m_.has_value() ||
             result.distance_m < *resting_closest_m_ - kRestingProgressM) {
    resting_closest_m_ = result.distance_m;
    resting_closest_stamp_ns_ = observation.stamp_ns;
  }
  const bool approach_ended =
      resting_inside_capture_radius &&
      (result.distance_m <= kApproachEndRadiusM ||
       observation.stamp_ns - resting_closest_stamp_ns_ >= kRestingStallNs);
  if (!latched_ && observation.terminal_route_available && approach_ended) {
    latched_ = true;
    result.newly_latched = true;
  }
  result.latched = latched_;
  return result;
}

bool MissionGoalCaptureLatch::latchedFor(const Point3& mission_goal) const noexcept {
  return mission_initialized_ && latched_ && finiteMission(mission_goal) &&
         sameMission(mission_goal_, mission_goal);
}

} // namespace drone_city_nav
