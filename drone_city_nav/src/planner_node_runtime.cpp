#include "planner_node.hpp"

namespace drone_city_nav {

void PlannerNode::invalidateCurrentPose() {
  current_pose_ = Pose2{};
  current_altitude_m_ = std::numeric_limits<double>::quiet_NaN();
  pose_valid_ = false;
  altitude_valid_ = false;
  last_scan_projection_pose_valid_ = false;
  last_pose_update_ns_ = 0;
}

[[nodiscard]] double PlannerNode::poseAgeSeconds(const std::int64_t now_ns) const {
  if (last_pose_update_ns_ <= 0 || now_ns <= last_pose_update_ns_) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(now_ns - last_pose_update_ns_) / 1.0e9;
}

[[nodiscard]] double PlannerNode::scanAgeSeconds(const std::int64_t now_ns) const {
  if (last_scan_update_ns_ <= 0 || now_ns <= last_scan_update_ns_) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(now_ns - last_scan_update_ns_) / 1.0e9;
}

bool PlannerNode::keepCurrentPathIfStillClear(const OccupancyGrid2D& grid) {
  if (!stable_path_reuse_enabled_ || last_valid_path_points_.size() < 2U) {
    return false;
  }

  const StablePathDecision decision = planner_core_.evaluateStablePath(
      grid, last_valid_path_points_, current_pose_.position, goal_);
  if (decision.reason == StablePathDecisionReason::kGoalMismatch ||
      decision.reason == StablePathDecisionReason::kProjectionUnavailable ||
      decision.reason == StablePathDecisionReason::kNoPreviousPath ||
      decision.reason == StablePathDecisionReason::kDisabled) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Stable path reuse rejected; running A*: reason=%s "
        "previous_waypoints=%zu endpoint_goal_distance=%.2fm "
        "goal_tolerance=%.2fm deviation=%.2fm "
        "current=(%.2f, %.2f) goal=(%.2f, %.2f)",
        stablePathDecisionReasonName(decision.reason), last_valid_path_points_.size(),
        decision.endpoint_goal_distance_m, stable_path_goal_tolerance_m_,
        decision.deviation_m, current_pose_.position.x, current_pose_.position.y,
        goal_.x, goal_.y);
    return false;
  }

  if (decision.reason == StablePathDecisionReason::kProhibitedConfirmed) {
    ++prohibited_replans_;
    const Point2 prohibited_start =
        decision.prohibited_segment_index < decision.remaining_path.size()
            ? decision.remaining_path[decision.prohibited_segment_index]
            : Point2{};
    const Point2 prohibited_end =
        decision.prohibited_segment_index + 1U < decision.remaining_path.size()
            ? decision.remaining_path[decision.prohibited_segment_index + 1U]
            : Point2{};
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Current path intersects newly available prohibited obstacle data; "
        "running A* from current pose: reason=%s "
        "remaining_waypoints=%zu deviation=%.2fm prohibited_segment=%zu "
        "segment_start=(%.2f, %.2f) segment_end=(%.2f, %.2f)",
        stablePathDecisionReasonName(decision.reason), decision.remaining_path.size(),
        decision.deviation_m, decision.prohibited_segment_index, prohibited_start.x,
        prohibited_start.y, prohibited_end.x, prohibited_end.y);
    return false;
  }

  last_valid_path_points_ = decision.remaining_path;
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Keeping current path because the remaining path is still clear: "
      "reason=%s remaining_waypoints=%zu deviation=%.2fm distance_to_goal=%.2f",
      stablePathDecisionReasonName(decision.reason), last_valid_path_points_.size(),
      decision.deviation_m, distance(current_pose_.position, goal_));
  return true;
}

void PlannerNode::logPathUpdate(const nav_msgs::msg::Path& path,
                                const PathMetrics& metrics,
                                const PathPublicationReason reason,
                                const std::uint64_t path_id) {
  const std::size_t path_size = path.poses.size();
  const bool path_changed = path_size != last_logged_path_size_;
  const std::string counters = plannerCountersSummary();
  const std::uint64_t path_stamp_ns = stampNanoseconds(path.header.stamp);
  if (path_size == 0U) {
    if (path_changed) {
      RCLCPP_WARN(get_logger(),
                  "Published empty path: path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                  " reason=%s counters[%s]",
                  path_id, path_stamp_ns, pathPublicationReasonName(reason),
                  counters.c_str());
      last_logged_path_size_ = path_size;
    }
    return;
  }

  const Point2 first{path.poses.front().pose.position.x,
                     path.poses.front().pose.position.y};
  const Point2 last{path.poses.back().pose.position.x,
                    path.poses.back().pose.position.y};
  const bool endpoint_changed =
      squaredDistance(first, last_logged_path_first_) > 0.01 ||
      squaredDistance(last, last_logged_path_last_) > 0.01;
  if (path_changed || endpoint_changed) {
    std::ostringstream preview;
    const std::size_t preview_count = std::min<std::size_t>(path_size, 6U);
    for (std::size_t i = 0U; i < preview_count; ++i) {
      if (i != 0U) {
        preview << " -> ";
      }
      preview << "(" << path.poses[i].pose.position.x << ", "
              << path.poses[i].pose.position.y << ")";
    }
    RCLCPP_INFO(get_logger(),
                "Published path: path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                " reason=%s waypoints=%zu segments=%zu "
                "straight_segments=%zu turns=%zu length=%.2f first=(%.2f, %.2f) "
                "segment_lengths[min=%.2f mean=%.2f max=%.2f lt2=%zu lt5=%zu "
                "lt10=%zu] "
                "last=(%.2f, %.2f) counters[%s] preview=%s",
                path_id, path_stamp_ns, pathPublicationReasonName(reason), path_size,
                metrics.segments, metrics.straight_segments, metrics.turns,
                metrics.length_m, first.x, first.y, metrics.min_segment_length_m,
                metrics.mean_segment_length_m, metrics.max_segment_length_m,
                metrics.segments_shorter_than_2m, metrics.segments_shorter_than_5m,
                metrics.segments_shorter_than_10m, last.x, last.y, counters.c_str(),
                preview.str().c_str());
    last_logged_path_size_ = path_size;
    last_logged_path_first_ = first;
    last_logged_path_last_ = last;
  }
}

void PlannerNode::recordPathPublication(const PathPublicationReason reason,
                                        const bool empty_path) {
  ++path_publications_;
  if (empty_path) {
    ++hold_path_publications_;
  } else {
    ++non_empty_path_publications_;
  }

  switch (reason) {
    case PathPublicationReason::kComputedPath:
      ++computed_path_publications_;
      break;
    case PathPublicationReason::kHoldNoPose:
    case PathPublicationReason::kHoldNoPlanningGrid:
    case PathPublicationReason::kHoldInvalidPath:
    case PathPublicationReason::kHoldAfterPlanningFailure:
      break;
  }
}

[[nodiscard]] std::string PlannerNode::plannerCountersSummary() const {
  std::ostringstream summary;
  summary << "astar_runs=" << astar_runs_ << " astar_successes=" << astar_successes_
          << " astar_failures=" << astar_failures_
          << " prohibited_replans=" << prohibited_replans_
          << " path_publications=" << path_publications_
          << " non_empty_path_publications=" << non_empty_path_publications_
          << " hold_path_publications=" << hold_path_publications_
          << " computed_path_publications=" << computed_path_publications_;
  return summary.str();
}

void PlannerNode::logPlannerCountersThrottled() {
  const std::string counters = plannerCountersSummary();
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Planner counters: %s",
                       counters.c_str());
}

} // namespace drone_city_nav
