#include "drone_city_nav/topological_exploration_memory_3d.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr DirectedTopologyEdge3D kForward{
    .edge_id = IncrementalTopologyEdgeId{100U},
    .from = IncrementalTopologyNodeId{1U},
    .to = IncrementalTopologyNodeId{2U},
};
constexpr DirectedTopologyEdge3D kReverse{
    .edge_id = IncrementalTopologyEdgeId{100U},
    .from = IncrementalTopologyNodeId{2U},
    .to = IncrementalTopologyNodeId{1U},
};

TEST(TopologicalExplorationMemory3DTest, KeepsDirectedTraversalEvidenceIndependent) {
  TopologicalExplorationMemory3D memory;

  memory.recordTraversal(kForward, 4U, 12.5);
  memory.recordTraversal(kForward, 5U, 8.0);
  memory.recordTraversal(kReverse, 5U, 3.0);

  const DirectedTopologyEdgeEvidence3D forward = memory.evidence(kForward, 5U);
  const DirectedTopologyEdgeEvidence3D reverse = memory.evidence(kReverse, 5U);
  EXPECT_EQ(forward.traversal_count, 2U);
  EXPECT_DOUBLE_EQ(forward.traversed_distance_m, 20.5);
  EXPECT_EQ(forward.result, TopologicalExplorationResult3D::kTraversed);
  EXPECT_EQ(reverse.traversal_count, 1U);
  EXPECT_DOUBLE_EQ(reverse.traversed_distance_m, 3.0);
}

TEST(TopologicalExplorationMemory3DTest, ReopensDeadEndAfterSupportingRevisionChanges) {
  TopologicalExplorationMemory3D memory;
  memory.recordDeadEnd(kForward, 8U);

  EXPECT_EQ(memory.evidence(kForward, 8U).result,
            TopologicalExplorationResult3D::kDeadEnd);
  const DirectedTopologyEdgeEvidence3D reopened = memory.evidence(kForward, 9U);
  EXPECT_EQ(reopened.result, TopologicalExplorationResult3D::kUnknown);
  EXPECT_EQ(reopened.conclusion_revision, 0U);
}

TEST(TopologicalExplorationMemory3DTest, CoverageIsFiniteSoftAndRevisionDecayed) {
  TopologicalExplorationMemory3DConfig config;
  config.revision_decay = 0.02;
  TopologicalExplorationMemory3D memory{config};
  const Point3 point{2.2, -1.2, 7.1};
  memory.recordVisited(point, 10U);
  memory.recordVisited(point, 10U);
  memory.recordObserved(point, 10U);

  const double fresh = memory.softCoveragePenalty(point, 10U);
  const double old = memory.softCoveragePenalty(point, 100U);
  EXPECT_TRUE(std::isfinite(fresh));
  EXPECT_GT(fresh, 0.0);
  EXPECT_GT(fresh, old);
  EXPECT_GT(old, 0.0);
  EXPECT_DOUBLE_EQ(memory.softCoveragePenalty({100.0, 100.0, 100.0}, 10U), 0.0);
}

TEST(TopologicalExplorationMemory3DTest,
     DefaultCoveragePersistsUntilMissionLegChanges) {
  TopologicalExplorationMemory3D memory;
  const Point3 point{2.2, -1.2, 7.1};
  memory.recordVisited(point, 10U);
  memory.recordObserved(point, 10U);

  const double initial = memory.softCoveragePenalty(point, 10U);
  EXPECT_GT(initial, 0.0);
  EXPECT_DOUBLE_EQ(memory.softCoveragePenalty(point, 10000U), initial);

  memory.beginMissionLeg();

  EXPECT_DOUBLE_EQ(memory.softCoveragePenalty(point, 10000U), 0.0);
}

TEST(TopologicalExplorationMemory3DTest,
     CoverageSoftlyDiscouragesNearbyUnvisitedCells) {
  TopologicalExplorationMemory3D memory;
  memory.recordVisited({0.1, 0.1, 0.1}, 10U);

  const double visited = memory.softCoveragePenalty({0.1, 0.1, 0.1}, 10U);
  const double nearby = memory.softCoveragePenalty({2.1, 0.1, 0.1}, 10U);

  EXPECT_GT(visited, nearby);
  EXPECT_GT(nearby, 0.0);
  EXPECT_DOUBLE_EQ(memory.softCoveragePenalty({6.1, 0.1, 0.1}, 10U), 0.0);
}

TEST(TopologicalExplorationMemory3DTest,
     ZeroInfluenceRadiusPreservesExactCellCoverage) {
  TopologicalExplorationMemory3DConfig config;
  config.coverage_influence_radius_m = 0.0;
  TopologicalExplorationMemory3D memory{config};
  memory.recordVisited({0.1, 0.1, 0.1}, 10U);

  EXPECT_GT(memory.softCoveragePenalty({0.1, 0.1, 0.1}, 10U), 0.0);
  EXPECT_DOUBLE_EQ(memory.softCoveragePenalty({2.1, 0.1, 0.1}, 10U), 0.0);
}

TEST(TopologicalExplorationMemory3DTest, SamplesVisitedPathWithoutHardExclusion) {
  TopologicalExplorationMemory3D memory;
  const std::array points{Point3{0.0, 0.0, 0.0}, Point3{8.0, 0.0, 0.0}};

  memory.recordVisitedPath(points, 3U, 1.0);

  EXPECT_GE(memory.coverageCellCount(), 4U);
  EXPECT_GT(memory.softCoveragePenalty({4.0, 0.0, 0.0}, 3U), 0.0);
}

TEST(TopologicalExplorationMemory3DTest, TrailCollapsesOrdinaryBacktracking) {
  TopologicalExplorationMemory3D memory;
  memory.resetTrail({1U});
  memory.recordTrailTransition({1U}, {2U});
  memory.recordTrailTransition({2U}, {3U});
  memory.recordTrailTransition({3U}, {2U});

  ASSERT_EQ(memory.trail().size(), 2U);
  EXPECT_EQ(memory.trail()[0], IncrementalTopologyNodeId{1U});
  EXPECT_EQ(memory.trail()[1], IncrementalTopologyNodeId{2U});
}

TEST(TopologicalExplorationMemory3DTest, CountsFrontierSelectionsByStableIdentity) {
  TopologicalExplorationMemory3D memory;
  memory.recordFrontierSelection({17U});
  memory.recordFrontierSelection({17U});
  memory.recordFrontierSelection({18U});

  EXPECT_EQ(memory.frontierSelectionCount({17U}), 2U);
  EXPECT_EQ(memory.frontierSelectionCount({18U}), 1U);
  EXPECT_EQ(memory.frontierSelectionCount({19U}), 0U);
}

TEST(TopologicalExplorationMemory3DTest, CountsCompletedFrontiersSeparately) {
  TopologicalExplorationMemory3D memory;
  memory.recordFrontierCompletion({17U});
  memory.recordFrontierCompletion({17U});
  memory.recordFrontierCompletion({18U});

  EXPECT_EQ(memory.frontierCompletionCount({17U}), 2U);
  EXPECT_EQ(memory.frontierCompletionCount({18U}), 1U);
  EXPECT_EQ(memory.frontierCompletionCount({19U}), 0U);
}

TEST(TopologicalExplorationMemory3DTest,
     NewMissionLegClearsAllGoalRelativeExplorationHistory) {
  TopologicalExplorationMemory3D memory;
  const Point3 visited{4.0, 5.0, 6.0};
  memory.recordTraversal(kForward, 8U, 12.0);
  memory.recordTraversal(kReverse, 8U, 3.0);
  memory.recordDeadEnd(kReverse, 8U);
  memory.recordVisited(visited, 8U);
  memory.recordObserved(visited, 8U);
  memory.recordFrontierSelection({17U});
  memory.recordFrontierCompletion({17U});
  memory.resetTrail({1U});
  memory.recordTrailTransition({1U}, {2U});

  memory.beginMissionLeg();

  const DirectedTopologyEdgeEvidence3D forward = memory.evidence(kForward, 8U);
  const DirectedTopologyEdgeEvidence3D reverse = memory.evidence(kReverse, 8U);
  EXPECT_EQ(forward.result, TopologicalExplorationResult3D::kUnknown);
  EXPECT_EQ(forward.traversal_count, 0U);
  EXPECT_EQ(reverse.result, TopologicalExplorationResult3D::kUnknown);
  EXPECT_EQ(reverse.traversal_count, 0U);
  EXPECT_DOUBLE_EQ(reverse.traversed_distance_m, 0.0);
  EXPECT_DOUBLE_EQ(memory.softCoveragePenalty(visited, 8U), 0.0);
  EXPECT_EQ(memory.frontierSelectionCount({17U}), 0U);
  EXPECT_EQ(memory.frontierCompletionCount({17U}), 0U);
  EXPECT_TRUE(memory.trail().empty());
}

} // namespace
} // namespace drone_city_nav
