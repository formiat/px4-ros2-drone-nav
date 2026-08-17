#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

struct MissionWaypointSequenceConfig {
  double goal_radius_m{2.0};
  double stop_speed_mps{0.8};
  double stop_hold_s{2.0};
};

struct MissionWaypointObservation {
  std::int64_t stamp_ns{0};
  bool goal_captured{false};
  double horizontal_speed_mps{0.0};
};

struct MissionWaypointUpdate {
  bool waypoint_completed{false};
  bool advanced{false};
  bool mission_completed{false};
  std::size_t completed_index{0U};
};

[[nodiscard]] std::vector<Point3>
missionWaypointsFromFlatParameters(std::span<const double> parameters);

class MissionWaypointSequence final {
public:
  MissionWaypointSequence(std::vector<Point3> waypoints,
                          MissionWaypointSequenceConfig config = {});

  [[nodiscard]] const Point3& activeGoal() const noexcept;
  [[nodiscard]] std::size_t activeIndex() const noexcept;
  [[nodiscard]] std::size_t waypointCount() const noexcept;
  [[nodiscard]] std::size_t completedWaypointCount() const noexcept;
  [[nodiscard]] bool missionCompleted() const noexcept;

  [[nodiscard]] MissionWaypointUpdate
  update(const MissionWaypointObservation& observation);

private:
  std::vector<Point3> waypoints_;
  MissionWaypointSequenceConfig config_;
  std::size_t active_index_{0U};
  std::size_t completed_waypoint_count_{0U};
  bool mission_completed_{false};
  std::int64_t stopped_since_ns_{0};
};

} // namespace drone_city_nav
