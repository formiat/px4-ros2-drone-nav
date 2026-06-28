#include "drone_city_nav/planner_diagnostics_format.hpp"

#include <algorithm>
#include <sstream>

namespace drone_city_nav {

[[nodiscard]] std::string
plannerCountersSummary(const PlannerCountersSnapshot& counters) {
  std::ostringstream summary;
  summary << "astar_runs=" << counters.astar_runs
          << " astar_successes=" << counters.astar_successes
          << " astar_failures=" << counters.astar_failures
          << " prohibited_replans=" << counters.prohibited_replans
          << " path_publications=" << counters.publication.path_publications
          << " non_empty_path_publications="
          << counters.publication.non_empty_path_publications
          << " hold_path_publications=" << counters.publication.hold_path_publications
          << " computed_path_publications="
          << counters.publication.computed_path_publications;
  return summary.str();
}

[[nodiscard]] std::string pathPreview(const std::span<const Point2> path_points,
                                      const std::size_t max_points) {
  std::ostringstream preview;
  const std::size_t preview_count = std::min(path_points.size(), max_points);
  for (std::size_t i = 0U; i < preview_count; ++i) {
    if (i != 0U) {
      preview << " -> ";
    }
    preview << "(" << path_points[i].x << ", " << path_points[i].y << ")";
  }
  return preview.str();
}

} // namespace drone_city_nav
