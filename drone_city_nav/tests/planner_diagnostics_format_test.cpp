#include "drone_city_nav/planner_diagnostics_format.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace drone_city_nav {

TEST(PlannerDiagnosticsFormat, FormatsCountersSummaryFields) {
  const PlannerCountersSnapshot counters{3U, 2U, 1U, 4U,
                                         PathPublicationCounters{5U, 6U, 7U, 8U}};

  const std::string summary = plannerCountersSummary(counters);

  EXPECT_NE(summary.find("astar_runs=3"), std::string::npos);
  EXPECT_NE(summary.find("astar_successes=2"), std::string::npos);
  EXPECT_NE(summary.find("astar_failures=1"), std::string::npos);
  EXPECT_NE(summary.find("prohibited_replans=4"), std::string::npos);
  EXPECT_NE(summary.find("path_publications=5"), std::string::npos);
  EXPECT_NE(summary.find("non_empty_path_publications=6"), std::string::npos);
  EXPECT_NE(summary.find("hold_path_publications=7"), std::string::npos);
  EXPECT_NE(summary.find("computed_path_publications=8"), std::string::npos);
}

TEST(PlannerDiagnosticsFormat, BuildsBoundedPathPreview) {
  const std::vector<Point2> points{Point2{0.0, 1.0}, Point2{2.0, 3.0},
                                   Point2{4.0, 5.0}};

  EXPECT_EQ(pathPreview(points, 2U), "(0, 1) -> (2, 3)");
  EXPECT_EQ(pathPreview(points, 10U), "(0, 1) -> (2, 3) -> (4, 5)");
  EXPECT_TRUE(pathPreview({}, 2U).empty());
}

} // namespace drone_city_nav
