#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

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

enum class RouteCandidateStatus : std::uint8_t {
  kAccepted,
  kEmptyInput,
  kRejectedNonTraversable,
};

struct RouteCandidateDecision {
  RouteCandidateStatus status{RouteCandidateStatus::kEmptyInput};
  std::vector<Point2> points;
  std::size_t pre_collapse_points{0U};
  std::size_t collapsed_points{0U};
  bool collapse_reverted{false};
};

[[nodiscard]] RouteCandidateDecision
selectRouteCandidate(std::span<const Point2> pre_collapse_points,
                     double collinearity_tolerance_m,
                     bool (*path_traversable)(std::span<const Point2>, const void*),
                     const void* context);

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
