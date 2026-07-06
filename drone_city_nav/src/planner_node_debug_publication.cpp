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
  if (log_publication) {
    RCLCPP_INFO(get_logger(),
                "Published static map grid: cells=%zu occupied=%zu "
                "points_topic='%s' republish_period=%.2fs",
                grid.cellCount(), static_map_occupied_cells_,
                static_map_points_pub_->get_topic_name(),
                static_map_debug_publish_period_s_);
  }
}

void PlannerNode::republishStaticMapDebug() {
  if (!static_grid_.has_value()) {
    return;
  }

  publishStaticMapDebug(*static_grid_, false);
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
  }

  const PathMetrics metrics = pointPathMetrics(points);
  const std_msgs::msg::Header header = makePlannerHeader();
  const std::uint64_t path_stamp_ns = stampNanoseconds(header.stamp);
  const nav_msgs::msg::Path path =
      pathToRos(std::span<const Point2>{points.data(), points.size()}, header,
                cruise_altitude_m_);

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
