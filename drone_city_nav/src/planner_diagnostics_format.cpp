#include "drone_city_nav/planner_diagnostics_format.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] std::string
plannerCountersSummary(const PlannerCountersSnapshot& counters) {
  std::ostringstream summary;
  summary << "astar_runs=" << counters.astar_runs
          << " astar_successes=" << counters.astar_successes
          << " astar_failures=" << counters.astar_failures
          << " repair_astar_runs=" << counters.repair_astar_runs
          << " prohibited_replans=" << counters.prohibited_replans
          << " path_publications=" << counters.publication.path_publications
          << " non_empty_path_publications="
          << counters.publication.non_empty_path_publications
          << " hold_path_publications=" << counters.publication.hold_path_publications
          << " computed_path_publications="
          << counters.publication.computed_path_publications
          << " rollout_cycles=" << counters.rollout_cycles
          << " rollout_candidates=" << counters.rollout_candidates
          << " rollout_publications=" << counters.rollout_publications
          << " rollout_recovery_requests=" << counters.rollout_recovery_requests
          << " rollout_failures=" << counters.rollout_failures
          << " rollout_deadline_missed=" << counters.rollout_deadline_missed
          << " rollout_duration_p50_ms=" << counters.rollout_duration_p50_ms
          << " rollout_duration_p95_ms=" << counters.rollout_duration_p95_ms;
  return summary.str();
}

double percentile(const std::span<const double> values, const double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  std::vector<double> sorted{values.begin(), values.end()};
  std::ranges::sort(sorted);
  const double bounded_fraction = std::clamp(fraction, 0.0, 1.0);
  const std::size_t index = static_cast<std::size_t>(
      std::ceil(bounded_fraction * static_cast<double>(sorted.size())) - 1.0);
  return sorted[std::min(index, sorted.size() - 1U)];
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
