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
}

MissionGoalCaptureResult
MissionGoalCaptureLatch::update(const MissionGoalCaptureObservation& observation) {
  MissionGoalCaptureResult result;
  if (!finiteMission(observation.mission_goal) || !std::isfinite(observation.state.x) ||
      !std::isfinite(observation.state.y)) {
    return result;
  }
  if (!mission_initialized_ || !sameMission(mission_goal_, observation.mission_goal)) {
    mission_goal_ = observation.mission_goal;
    mission_initialized_ = true;
    latched_ = false;
  }

  result.horizontal_distance_m =
      std::hypot(static_cast<double>(observation.state.x) - mission_goal_.x,
                 static_cast<double>(observation.state.y) - mission_goal_.y);
  if (!latched_ && observation.terminal_route_available &&
      result.horizontal_distance_m <= config_.capture_radius_m) {
    latched_ = true;
    result.newly_latched = true;
  }
  result.latched = latched_;
  return result;
}

} // namespace drone_city_nav
