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
};

[[nodiscard]] std::string
plannerCountersSummary(const PlannerCountersSnapshot& counters);

[[nodiscard]] std::string pathPreview(std::span<const Point2> path_points,
                                      std::size_t max_points);

} // namespace drone_city_nav
