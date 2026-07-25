#include "drone_city_nav/planner_diagnostics_format.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace drone_city_nav {

TEST(PlannerDiagnosticsFormat, FormatsCountersSummaryFields) {
  PlannerCountersSnapshot counters;
  counters.astar_runs = 3U;
  counters.astar_successes = 2U;
  counters.astar_failures = 1U;
  counters.repair_astar_runs = 9U;
  counters.prohibited_replans = 4U;
  counters.publication = PathPublicationCounters{5U, 6U, 7U, 8U};

  const std::string summary = plannerCountersSummary(counters);

  EXPECT_NE(summary.find("astar_runs=3"), std::string::npos);
  EXPECT_NE(summary.find("astar_successes=2"), std::string::npos);
  EXPECT_NE(summary.find("astar_failures=1"), std::string::npos);
  EXPECT_NE(summary.find("repair_astar_runs=9"), std::string::npos);
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

TEST(PlannerDiagnosticsFormat, FormatsRolloutSummaryAndPercentiles) {
  PlannerCountersSnapshot counters;
  counters.rollout_cycles = 9U;
  counters.rollout_candidates = 27U;
  counters.rollout_publications = 5U;
  counters.rollout_recovery_requests = 2U;
  counters.rollout_failures = 1U;
  counters.rollout_deadline_missed = 3U;
  counters.rollout_duration_p50_ms = 3.0;
  counters.rollout_duration_p95_ms = 8.0;

  const std::string summary = plannerCountersSummary(counters);

  EXPECT_NE(summary.find("rollout_cycles=9"), std::string::npos);
  EXPECT_NE(summary.find("rollout_candidates=27"), std::string::npos);
  EXPECT_NE(summary.find("rollout_deadline_missed=3"), std::string::npos);
  EXPECT_NE(summary.find("rollout_duration_p50_ms=3"), std::string::npos);
  EXPECT_NE(summary.find("rollout_duration_p95_ms=8"), std::string::npos);
  EXPECT_DOUBLE_EQ(percentile(std::vector<double>{1.0, 2.0, 3.0, 8.0}, 0.50), 2.0);
  EXPECT_DOUBLE_EQ(percentile(std::vector<double>{1.0, 2.0, 3.0, 8.0}, 0.95), 8.0);
}

} // namespace drone_city_nav
