#include "drone_city_nav/planner_path_publication.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool pathAlwaysTraversable(const std::span<const Point2> points,
                                         const void* context) {
  (void)points;
  (void)context;
  return true;
}

[[nodiscard]] bool pathTraversable(const std::span<const Point2> points,
                                   const void* context) {
  (void)context;
  return points.size() != 2U || points.front().x != 0.0 || points.back().x != 10.0;
}

[[nodiscard]] bool pathNeverTraversable(const std::span<const Point2> points,
                                        const void* context) {
  (void)points;
  (void)context;
  return false;
}

[[nodiscard]] bool segmentTraversable(Point2 start, Point2 end,
                                      const void* unused_context) {
  (void)unused_context;
  return start.x != 1.0 || end.x != 2.0;
}

[[nodiscard]] bool segmentAllowed(Point2 start, Point2 end,
                                  const void* unused_context) {
  (void)unused_context;
  return start.x != 0.0 || end.x != 1.0;
}

} // namespace

TEST(PlannerPathPublication, NamesPublicationReasons) {
  EXPECT_STREQ(pathPublicationReasonName(PathPublicationReason::kComputedPath),
               "computed_path");
  EXPECT_STREQ(pathPublicationReasonName(PathPublicationReason::kHoldNoPose),
               "hold_no_pose");
  EXPECT_STREQ(pathPublicationReasonName(PathPublicationReason::kHoldInvalidPath),
               "hold_invalid_path");
}

TEST(PlannerPathPublication, RecordsPublicationCounters) {
  PathPublicationCounters counters;

  recordPathPublication(counters, PathPublicationReason::kComputedPath, false);
  recordPathPublication(counters, PathPublicationReason::kHoldNoPlanningGrid, true);

  EXPECT_EQ(counters.path_publications, 2U);
  EXPECT_EQ(counters.non_empty_path_publications, 1U);
  EXPECT_EQ(counters.hold_path_publications, 1U);
  EXPECT_EQ(counters.computed_path_publications, 1U);
}

TEST(PlannerPathPublication, SummarizesTraversabilityAndEscapeSegments) {
  std::vector<Point2> points{Point2{0.0, 0.0}, Point2{1.0, 0.0}, Point2{2.0, 0.0},
                             Point2{3.0, 0.0}};

  const PublishedPathSafetySummary summary =
      summarizePathSafety(points, segmentTraversable, segmentAllowed, nullptr);

  EXPECT_EQ(summary.segments, 3U);
  EXPECT_EQ(summary.escape_segments, 1U);
  EXPECT_EQ(summary.non_traversable_segments, 1U);
  EXPECT_TRUE(summary.has_non_traversable_segment);
  EXPECT_EQ(summary.first_non_traversable_segment, 1U);
  EXPECT_DOUBLE_EQ(summary.first_non_traversable_start.x, 1.0);
  EXPECT_DOUBLE_EQ(summary.first_non_traversable_end.x, 2.0);
}

TEST(PlannerPathPublication, SelectsCollapsedRouteWhenTraversable) {
  const std::vector<Point2> points{Point2{0.0, 0.0}, Point2{5.0, 0.0},
                                   Point2{10.0, 0.0}};

  const RouteCandidateDecision decision =
      selectRouteCandidate(points, 0.01, pathAlwaysTraversable, nullptr);

  EXPECT_EQ(decision.status, RouteCandidateStatus::kAccepted);
  EXPECT_FALSE(decision.collapse_reverted);
  EXPECT_EQ(decision.pre_collapse_points, 3U);
  EXPECT_EQ(decision.collapsed_points, 2U);
  EXPECT_EQ(decision.points.size(), 2U);
}

TEST(PlannerPathPublication, RestoresPreCollapseRouteWhenCollapseIsBlocked) {
  const std::vector<Point2> points{Point2{0.0, 0.0}, Point2{5.0, 0.0},
                                   Point2{10.0, 0.0}};

  const RouteCandidateDecision decision =
      selectRouteCandidate(points, 0.01, pathTraversable, nullptr);

  EXPECT_EQ(decision.status, RouteCandidateStatus::kAccepted);
  EXPECT_TRUE(decision.collapse_reverted);
  EXPECT_EQ(decision.pre_collapse_points, 3U);
  EXPECT_EQ(decision.collapsed_points, 2U);
  EXPECT_EQ(decision.points.size(), 3U);
}

TEST(PlannerPathPublication, RejectsNonTraversablePreCollapseRoute) {
  const std::vector<Point2> points{Point2{0.0, 0.0}, Point2{5.0, 0.0},
                                   Point2{10.0, 0.0}};

  const RouteCandidateDecision decision =
      selectRouteCandidate(points, 0.01, pathNeverTraversable, nullptr);

  EXPECT_EQ(decision.status, RouteCandidateStatus::kRejectedNonTraversable);
}

} // namespace drone_city_nav
