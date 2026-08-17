#include "drone_city_nav/mission_waypoint_sequence.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

} // namespace

std::vector<Point3>
missionWaypointsFromFlatParameters(const std::span<const double> parameters) {
  if (parameters.empty()) {
    throw std::invalid_argument{
        "mission_goal_sequence_xyz_m must contain at least one x,y,z waypoint"};
  }
  if (parameters.size() % 3U != 0U) {
    throw std::invalid_argument{
        "mission_goal_sequence_xyz_m must contain complete x,y,z waypoint triples"};
  }

  std::vector<Point3> waypoints;
  waypoints.reserve(parameters.size() / 3U);
  for (std::size_t index = 0U; index < parameters.size(); index += 3U) {
    const Point3 waypoint{parameters[index], parameters[index + 1U],
                          parameters[index + 2U]};
    if (!finitePoint(waypoint)) {
      throw std::invalid_argument{"mission waypoint must be finite"};
    }
    waypoints.push_back(waypoint);
  }
  return waypoints;
}

MissionWaypointSequence::MissionWaypointSequence(
    std::vector<Point3> waypoints, const MissionWaypointSequenceConfig config)
    : waypoints_{std::move(waypoints)},
      config_{config} {
  if (waypoints_.empty()) {
    throw std::invalid_argument{"mission waypoint sequence must not be empty"};
  }
  if (!(config_.goal_radius_m > 0.0) || !(config_.stop_speed_mps >= 0.0) ||
      !(config_.stop_hold_s >= 0.0) || !std::isfinite(config_.goal_radius_m) ||
      !std::isfinite(config_.stop_speed_mps) || !std::isfinite(config_.stop_hold_s)) {
    throw std::invalid_argument{"invalid mission waypoint sequence configuration"};
  }
  for (const Point3& waypoint : waypoints_) {
    if (!finitePoint(waypoint)) {
      throw std::invalid_argument{"mission waypoint must be finite"};
    }
  }
}

const Point3& MissionWaypointSequence::activeGoal() const noexcept {
  return waypoints_[active_index_];
}

std::size_t MissionWaypointSequence::activeIndex() const noexcept {
  return active_index_;
}

std::size_t MissionWaypointSequence::waypointCount() const noexcept {
  return waypoints_.size();
}

std::size_t MissionWaypointSequence::completedWaypointCount() const noexcept {
  return completed_waypoint_count_;
}

bool MissionWaypointSequence::missionCompleted() const noexcept {
  return mission_completed_;
}

MissionWaypointUpdate
MissionWaypointSequence::update(const MissionWaypointObservation& observation) {
  MissionWaypointUpdate update;
  if (mission_completed_) {
    update.mission_completed = true;
    return update;
  }
  if (observation.stamp_ns <= 0 || !std::isfinite(observation.horizontal_speed_mps) ||
      !observation.goal_captured ||
      observation.horizontal_speed_mps > config_.stop_speed_mps) {
    stopped_since_ns_ = 0;
    return update;
  }
  if (stopped_since_ns_ == 0) {
    stopped_since_ns_ = observation.stamp_ns;
    return update;
  }

  const std::int64_t required_hold_ns =
      static_cast<std::int64_t>(std::llround(config_.stop_hold_s * 1'000'000'000.0));
  if (observation.stamp_ns - stopped_since_ns_ < required_hold_ns) {
    return update;
  }

  update.waypoint_completed = true;
  update.completed_index = active_index_;
  ++completed_waypoint_count_;
  stopped_since_ns_ = 0;
  if (active_index_ + 1U < waypoints_.size()) {
    ++active_index_;
    update.advanced = true;
    return update;
  }
  mission_completed_ = true;
  update.mission_completed = true;
  return update;
}

} // namespace drone_city_nav
