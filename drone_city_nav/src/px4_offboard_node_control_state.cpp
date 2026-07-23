#include <limits>
#include <numbers>

#include "px4_offboard_node.hpp"

namespace drone_city_nav {

[[nodiscard]] bool Px4OffboardNode::missionStartReady() const {
  return localPositionFresh() && pathFollowingReady();
}

[[nodiscard]] UpcomingTurn
Px4OffboardNode::upcomingTurnAtWaypoint(const std::size_t index) const {
  if (!path_valid_ || !localPositionFresh()) {
    return UpcomingTurn{};
  }
  return drone_city_nav::upcomingTurnAtWaypoint(path_points_, index, current_position_,
                                                true, pathFollowerConfig());
}

[[nodiscard]] const char*
Px4OffboardNode::pathSegmentTypeName(const double turn_angle_rad) const {
  if (!path_valid_) {
    return "no_path";
  }
  if (turn_angle_rad < 0.15) {
    return "straight";
  }
  if (turn_angle_rad < std::numbers::pi / 2.0) {
    return "gentle_turn";
  }
  return "sharp_turn";
}

[[nodiscard]] const char*
Px4OffboardNode::motionPhaseName(const bool hold_position) const noexcept {
  if (final_goal_hold_active_) {
    return "final_goal_hold";
  }
  if (temporary_replan_hold_active_) {
    return "temporary_replan_hold";
  }
  if (no_path_hold_target_valid_) {
    return "hold_no_path";
  }
  if (path_valid_ && !finalTrajectoryReady()) {
    return "hold_invalid_trajectory";
  }
  if (last_terminal_position_capture_active_) {
    return "terminal_position_capture";
  }
  if (hold_position) {
    return "hold";
  }
  return "path_following";
}

[[nodiscard]] bool Px4OffboardNode::prohibitedGridFresh() const {
  if (!prohibited_grid_valid_ || last_prohibited_grid_update_ns_ <= 0) {
    return false;
  }
  if (max_clearance_grid_staleness_ns_ <= 0) {
    return true;
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  return now_ns >= last_prohibited_grid_update_ns_ &&
         now_ns - last_prohibited_grid_update_ns_ <= max_clearance_grid_staleness_ns_;
}

[[nodiscard]] bool Px4OffboardNode::localPositionFresh() const {
  if (!local_position_valid_ || last_local_position_update_ns_ <= 0) {
    return false;
  }
  if (max_pose_staleness_ns_ <= 0) {
    return true;
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  return now_ns >= last_local_position_update_ns_ &&
         now_ns - last_local_position_update_ns_ <= max_pose_staleness_ns_;
}

[[nodiscard]] std::optional<OccupancyGrid2D>
Px4OffboardNode::currentProhibitedGrid() const {
  if (!prohibited_grid_valid_ || !(prohibited_grid_.info.resolution > 0.0F) ||
      prohibited_grid_.info.width == 0U || prohibited_grid_.info.height == 0U ||
      prohibited_grid_.info.width >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      prohibited_grid_.info.height >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }

  const auto width = static_cast<int>(prohibited_grid_.info.width);
  const auto height = static_cast<int>(prohibited_grid_.info.height);
  const std::size_t expected_data_size =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (prohibited_grid_.data.size() != expected_data_size) {
    return std::nullopt;
  }

  OccupancyGrid2D grid{GridBounds{
      prohibited_grid_.info.origin.position.x, prohibited_grid_.info.origin.position.y,
      static_cast<double>(prohibited_grid_.info.resolution), width, height}};
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const GridIndex cell{x, y};
      const std::size_t index =
          static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(x);
      if (prohibited_grid_.data[index] >= kInflatedOccupancyValue) {
        grid.setOccupied(cell);
      } else if (prohibited_grid_.data[index] == 0) {
        grid.setFree(cell);
      }
    }
  }
  return grid;
}

[[nodiscard]] std::optional<OccupancyGrid2D>
Px4OffboardNode::currentRawObstacleGrid() const {
  if (!prohibited_grid_valid_ || !(prohibited_grid_.info.resolution > 0.0F) ||
      prohibited_grid_.info.width == 0U || prohibited_grid_.info.height == 0U ||
      prohibited_grid_.info.width >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      prohibited_grid_.info.height >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }

  const auto width = static_cast<int>(prohibited_grid_.info.width);
  const auto height = static_cast<int>(prohibited_grid_.info.height);
  const std::size_t expected_data_size =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (prohibited_grid_.data.size() != expected_data_size) {
    return std::nullopt;
  }

  OccupancyGrid2D grid{GridBounds{
      prohibited_grid_.info.origin.position.x, prohibited_grid_.info.origin.position.y,
      static_cast<double>(prohibited_grid_.info.resolution), width, height}};
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const GridIndex cell{x, y};
      const std::size_t index =
          static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(x);
      if (prohibited_grid_.data[index] == 100) {
        grid.setOccupied(cell);
      } else if (prohibited_grid_.data[index] >= 0) {
        grid.setFree(cell);
      }
    }
  }
  return grid;
}

[[nodiscard]] double Px4OffboardNode::localPositionAgeSeconds() const {
  if (last_local_position_update_ns_ <= 0) {
    return std::numeric_limits<double>::infinity();
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (now_ns <= last_local_position_update_ns_) {
    return 0.0;
  }
  return static_cast<double>(now_ns - last_local_position_update_ns_) / 1.0e9;
}

[[nodiscard]] double Px4OffboardNode::attitudeAgeSeconds() const {
  if (last_attitude_update_ns_ <= 0) {
    return std::numeric_limits<double>::infinity();
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (now_ns <= last_attitude_update_ns_) {
    return 0.0;
  }
  return static_cast<double>(now_ns - last_attitude_update_ns_) / 1.0e9;
}

} // namespace drone_city_nav
