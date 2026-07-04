#include <algorithm>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include "planner_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool segmentTraversableForGrid(const Point2 start, const Point2 end,
                                             const void* context) {
  const auto& grid = *static_cast<const OccupancyGrid2D*>(context);
  return pathSegmentIsTraversable(grid, start, end);
}

[[nodiscard]] bool segmentAllowedForGrid(const Point2 start, const Point2 end,
                                         const void* context) {
  const auto& grid = *static_cast<const OccupancyGrid2D*>(context);
  return pathSegmentIsAllowed(grid, start, end);
}

} // namespace

[[nodiscard]] PublishedPathSafetySummary PlannerNode::summarizePublishedPathSafety(
    const OccupancyGrid2D& grid, const std::span<const Point2> path_points) const {
  return summarizePathSafety(path_points, segmentTraversableForGrid,
                             segmentAllowedForGrid, &grid);
}

void PlannerNode::logPublishedPathSafety(const OccupancyGrid2D& grid,
                                         const std::span<const Point2> path_points,
                                         const char* source_label) const {
  const PublishedPathSafetySummary summary =
      summarizePublishedPathSafety(grid, path_points);
  const bool unsafe_path = summary.non_traversable_segments > 0U;
  if (unsafe_path) {
    RCLCPP_WARN(
        get_logger(),
        "%s published path traversability: segments=%zu "
        "non_traversable_segments=%zu escape_segments=%zu traversable=false "
        "first_non_traversable_segment=%zu "
        "segment_start=(%.2f, %.2f) segment_end=(%.2f, %.2f)",
        source_label, summary.segments, summary.non_traversable_segments,
        summary.escape_segments, summary.first_non_traversable_segment,
        summary.first_non_traversable_start.x, summary.first_non_traversable_start.y,
        summary.first_non_traversable_end.x, summary.first_non_traversable_end.y);
    return;
  }

  if (summary.escape_segments > 0U) {
    RCLCPP_WARN(get_logger(),
                "%s published path traversability: segments=%zu "
                "non_traversable_segments=0 escape_segments=%zu traversable=true",
                source_label, summary.segments, summary.escape_segments);
    return;
  }

  RCLCPP_INFO(get_logger(),
              "%s published path traversability: segments=%zu "
              "non_traversable_segments=0 escape_segments=0 traversable=true",
              source_label, summary.segments);
}

[[nodiscard]] bool
PlannerNode::connectRouteToCurrentPose(const OccupancyGrid2D& grid,
                                       std::vector<Point2>& path_points,
                                       const char* source_label) const {
  if (path_points.empty()) {
    return false;
  }

  const Point2 current_position = current_pose_.position;
  const Point2 first_path_point = path_points.front();
  const double distance_to_first_m = distance(current_position, first_path_point);
  if (distance_to_first_m < 1.0e-6) {
    path_points.front() = current_position;
    return true;
  }

  if (distance_to_first_m < grid.resolution() && path_points.size() >= 2U &&
      pathSegmentIsTraversable(grid, current_position, path_points[1])) {
    path_points.front() = current_position;
    return true;
  }

  if (pathSegmentIsTraversable(grid, current_position, first_path_point)) {
    path_points.insert(path_points.begin(), current_position);
    return true;
  }

  RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "%s route candidate rejected because current pose cannot connect to the "
      "planned "
      "start with a traversable segment: current=(%.2f, %.2f) first=(%.2f, %.2f) "
      "distance=%.2fm",
      source_label, current_position.x, current_position.y, first_path_point.x,
      first_path_point.y, distance_to_first_m);
  return false;
}

void PlannerNode::logRejectedUnsafeRoute(const OccupancyGrid2D& grid,
                                         const std::span<const Point2> path_points,
                                         const char* source_label,
                                         const char* reason) const {
  const PublishedPathSafetySummary summary =
      summarizePublishedPathSafety(grid, path_points);
  RCLCPP_WARN(
      get_logger(),
      "%s route rejected before optimized trajectory build: reason='%s' segments=%zu "
      "non_traversable_segments=%zu escape_segments=%zu "
      "first_non_traversable_segment=%zu "
      "segment_start=(%.2f, %.2f) "
      "segment_end=(%.2f, %.2f)",
      source_label, reason, summary.segments, summary.non_traversable_segments,
      summary.escape_segments, summary.first_non_traversable_segment,
      summary.first_non_traversable_start.x, summary.first_non_traversable_start.y,
      summary.first_non_traversable_end.x, summary.first_non_traversable_end.y);
}

} // namespace drone_city_nav
