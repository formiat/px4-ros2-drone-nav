#include <algorithm>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include "planner_node.hpp"

namespace drone_city_nav {
[[nodiscard]] std_msgs::msg::Header PlannerNode::makePlannerHeader() const {
  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = frame_id_;
  return header;
}

void PlannerNode::publishStaticMapDebug(const OccupancyGrid2D& grid,
                                        const bool log_publication) {
  const StaticMapDebugConfig config{makePlannerHeader(),
                                    static_cast<float>(kGroundDebugZ)};
  static_map_pub_->publish(staticMapGridMessage(grid, config));
  static_map_points_pub_->publish(staticMapPointCloud(grid, config));
  if (static_map_debug_.has_value()) {
    static_building_markers_pub_->publish(
        staticMapBuildingMarkers(*static_map_debug_, config));
  } else {
    static_building_markers_pub_->publish(
        staticMapBuildingDeleteMarkers(config.header));
  }
  if (log_publication) {
    RCLCPP_INFO(get_logger(),
                "Published static map grid: cells=%zu occupied=%zu "
                "points_topic='%s' building_markers_topic='%s' "
                "republish_period=%.2fs",
                grid.cellCount(), static_map_occupied_cells_,
                static_map_points_pub_->get_topic_name(),
                static_building_markers_pub_->get_topic_name(),
                static_map_debug_publish_period_s_);
  }
}

void PlannerNode::republishStaticMapDebug() {
  if (!static_grid_.has_value()) {
    return;
  }

  publishStaticMapDebug(*static_grid_, false);
}

void PlannerNode::publishKnownPassageDebug(const bool log_publication) {
  if (!known_passage_markers_pub_) {
    return;
  }

  const std_msgs::msg::Header header = makePlannerHeader();
  if (!known_passages_.has_value()) {
    known_passage_markers_pub_->publish(buildKnownPassageDeleteMarkers(header));
    if (log_publication) {
      RCLCPP_INFO(get_logger(),
                  "Published known passage delete markers: topic='%s' "
                  "source_path='%s'",
                  known_passage_markers_pub_->get_topic_name(),
                  known_passages_resolved_path_.string().c_str());
    }
    return;
  }

  known_passage_markers_pub_->publish(
      buildKnownPassageDebugMarkers(header, *known_passages_));
  if (log_publication) {
    RCLCPP_INFO(get_logger(),
                "Published known passage markers: structures=%zu openings=%zu "
                "topic='%s' republish_period=%.2fs",
                known_passage_structures_, known_passage_openings_,
                known_passage_markers_pub_->get_topic_name(),
                known_passage_debug_publish_period_s_);
  }
}

void PlannerNode::republishKnownPassageDebug() {
  publishKnownPassageDebug(false);
}

void PlannerNode::publishProhibitedGrid(const OccupancyGrid2D& grid) {
  prohibited_grid_pub_->publish(
      prohibitedGridToRos(grid, ProhibitedGridToRosConfig{makePlannerHeader()}));
}

std::uint64_t PlannerNode::publishPath(const std::vector<Point2>& points,
                                       const PathPublicationReason reason,
                                       const TrajectoryPlannerStats* trajectory_stats) {
  recordPathPublication(reason, points.empty());
  const std::uint64_t path_id = next_path_id_++;
  last_published_path_id_ = path_id;

  if (points.empty()) {
    last_valid_path_points_.clear();
    last_valid_trajectory_samples_.clear();
  }

  const PathMetrics metrics = pointPathMetrics(points);
  const std_msgs::msg::Header header = makePlannerHeader();
  const std::uint64_t path_stamp_ns = stampNanoseconds(header.stamp);
  const nav_msgs::msg::Path path =
      pathToRos(std::span<const Point2>{points.data(), points.size()}, header,
                initial_altitude_m_);

  std_msgs::msg::UInt64 path_id_msg;
  path_id_msg.data = path_id;
  path_id_pub_->publish(path_id_msg);
  if (trajectory_stats != nullptr && !points.empty()) {
    publishTrajectoryDiagnostics(path_id, path_stamp_ns, *trajectory_stats);
  }
  path_pub_->publish(path);
  if (!path.poses.empty()) {
    waypoint_pub_->publish(path.poses.front());
  }

  logPathUpdate(path, metrics, reason, path_id);
  logPlannerCountersThrottled();
  return path_id;
}

std::uint64_t
PlannerNode::publishTrajectoryPath(const std::span<const TrajectoryPointSample> samples,
                                   const PathPublicationReason reason,
                                   const TrajectoryPlannerStats* trajectory_stats) {
  std::vector<Point2> points = trajectorySamplePoints(samples);
  recordPathPublication(reason, points.empty());
  const std::uint64_t path_id = next_path_id_++;
  last_published_path_id_ = path_id;

  if (points.empty()) {
    last_valid_path_points_.clear();
    last_valid_trajectory_samples_.clear();
  }

  const PathMetrics metrics = pointPathMetrics(points);
  const std_msgs::msg::Header header = makePlannerHeader();
  const std::uint64_t path_stamp_ns = stampNanoseconds(header.stamp);
  const nav_msgs::msg::Path path = pathToRos(samples, header);

  std_msgs::msg::UInt64 path_id_msg;
  path_id_msg.data = path_id;
  path_id_pub_->publish(path_id_msg);
  if (trajectory_stats != nullptr && !points.empty()) {
    publishTrajectoryDiagnostics(path_id, path_stamp_ns, *trajectory_stats);
  }
  path_pub_->publish(path);
  if (!path.poses.empty()) {
    waypoint_pub_->publish(path.poses.front());
  }

  logPathUpdate(path, metrics, reason, path_id);
  logPlannerCountersThrottled();
  return path_id;
}

void PlannerNode::publishTrajectoryDiagnostics(
    const std::uint64_t path_id, const std::uint64_t path_stamp_ns,
    const TrajectoryPlannerStats& stats) const {
  if (!trajectory_diagnostics_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = trajectoryPlannerDiagnosticsJson(path_id, path_stamp_ns, stats);
  trajectory_diagnostics_pub_->publish(msg);
}

void PlannerNode::publishPlanningFailureHold() {
  if (!last_valid_path_points_.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Clearing path after replanning failure; holding position instead of reusing "
        "stale waypoints");
  }
  publishPath({}, PathPublicationReason::kHoldAfterPlanningFailure);
}

} // namespace drone_city_nav
