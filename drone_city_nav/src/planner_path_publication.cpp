#include "drone_city_nav/planner_path_publication.hpp"

namespace drone_city_nav {

[[nodiscard]] const char*
pathPublicationReasonName(const PathPublicationReason reason) noexcept {
  switch (reason) {
    case PathPublicationReason::kComputedPath:
      return "computed_path";
    case PathPublicationReason::kHoldNoPose:
      return "hold_no_pose";
    case PathPublicationReason::kHoldNoPlanningGrid:
      return "hold_no_planning_grid";
    case PathPublicationReason::kHoldInvalidPath:
      return "hold_invalid_path";
    case PathPublicationReason::kHoldAfterPlanningFailure:
      return "hold_after_planning_failure";
  }

  return "unknown";
}

void recordPathPublication(PathPublicationCounters& counters,
                           const PathPublicationReason reason, const bool empty_path) {
  ++counters.path_publications;
  if (empty_path) {
    ++counters.hold_path_publications;
  } else {
    ++counters.non_empty_path_publications;
  }

  if (reason == PathPublicationReason::kComputedPath) {
    ++counters.computed_path_publications;
  }
}

[[nodiscard]] PublishedPathSafetySummary
summarizePathSafety(const std::span<const Point2> path_points,
                    bool (*segment_traversable)(Point2, Point2, const void*),
                    bool (*segment_allowed)(Point2, Point2, const void*),
                    const void* context) {
  PublishedPathSafetySummary summary{};
  if (path_points.size() < 2U) {
    return summary;
  }

  summary.segments = path_points.size() - 1U;
  for (std::size_t index = 1U; index < path_points.size(); ++index) {
    const Point2 segment_start = path_points[index - 1U];
    const Point2 segment_end = path_points[index];
    if (segment_traversable(segment_start, segment_end, context)) {
      if (!segment_allowed(segment_start, segment_end, context)) {
        ++summary.escape_segments;
      }
    } else {
      ++summary.non_traversable_segments;
      if (!summary.has_non_traversable_segment) {
        summary.first_non_traversable_segment = index - 1U;
        summary.first_non_traversable_start = segment_start;
        summary.first_non_traversable_end = segment_end;
        summary.has_non_traversable_segment = true;
      }
    }
  }
  return summary;
}

} // namespace drone_city_nav
