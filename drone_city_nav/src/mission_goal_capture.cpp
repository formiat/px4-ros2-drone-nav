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
  }

  result.distance_m =
      distance3D(Point3{observation.state.x, observation.state.y, observation.state.z},
                 mission_goal_);
  result.speed_mps = std::hypot(std::hypot(observation.state.vx, observation.state.vy),
                                observation.state.vz);
  const bool resting_inside_capture_radius =
      result.distance_m <= config_.capture_radius_m &&
      result.speed_mps <= config_.stationary_speed_tolerance_mps;
  if (!latched_ && observation.terminal_route_available &&
      resting_inside_capture_radius) {
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
