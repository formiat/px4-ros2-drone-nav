#pragma once

#include "drone_city_nav/planner_path_publication.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace drone_city_nav {

struct PlannerCountersSnapshot {
  std::uint64_t astar_runs{0U};
  std::uint64_t astar_successes{0U};
  std::uint64_t astar_failures{0U};
  std::uint64_t prohibited_replans{0U};
  PathPublicationCounters publication{};
  std::uint64_t rollout_cycles{0U};
  std::uint64_t rollout_candidates{0U};
  std::uint64_t rollout_publications{0U};
  std::uint64_t rollout_recovery_requests{0U};
  std::uint64_t rollout_failures{0U};
  double rollout_duration_p50_ms{0.0};
  double rollout_duration_p95_ms{0.0};
};

[[nodiscard]] std::string
plannerCountersSummary(const PlannerCountersSnapshot& counters);

[[nodiscard]] double percentile(std::span<const double> values, double fraction);

[[nodiscard]] std::string pathPreview(std::span<const Point2> path_points,
                                      std::size_t max_points);

} // namespace drone_city_nav
