#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace drone_city_nav {

enum class PathPublicationReason : std::uint8_t {
  kComputedPath,
  kHoldNoPose,
  kHoldNoPlanningGrid,
  kHoldInvalidPath,
  kHoldAfterPlanningFailure,
};

struct PathPublicationCounters {
  std::uint64_t path_publications{0U};
  std::uint64_t non_empty_path_publications{0U};
  std::uint64_t hold_path_publications{0U};
  std::uint64_t computed_path_publications{0U};
};

struct PublishedPathSafetySummary {
  std::size_t segments{0U};
  std::size_t non_traversable_segments{0U};
  std::size_t escape_segments{0U};
  std::size_t first_non_traversable_segment{0U};
  Point2 first_non_traversable_start{};
  Point2 first_non_traversable_end{};
  bool has_non_traversable_segment{false};
};

[[nodiscard]] const char*
pathPublicationReasonName(PathPublicationReason reason) noexcept;

void recordPathPublication(PathPublicationCounters& counters,
                           PathPublicationReason reason, bool empty_path);

[[nodiscard]] PublishedPathSafetySummary
summarizePathSafety(std::span<const Point2> path_points,
                    bool (*segment_traversable)(Point2, Point2, const void*),
                    bool (*segment_allowed)(Point2, Point2, const void*),
                    const void* context);

} // namespace drone_city_nav
