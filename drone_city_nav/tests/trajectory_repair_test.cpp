#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/raw_obstacle_snapshot_tracker.hpp"
#include "drone_city_nav/trajectory_repair.hpp"
#include "drone_city_nav/truncation_suffix_protocol.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D freeGrid() {
  OccupancyGrid2D grid{GridBounds{-0.5, -10.5, 1.0, 241, 22}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  return grid;
}

[[nodiscard]] std::vector<TrajectoryPointSample> lineSamples(const double length_m) {
  return trajectoryPointSamplesFromPoints(
      std::vector<Point2>{{0.0, 0.0}, {length_m, 0.0}});
}

} // namespace

TEST(TrajectoryRepair, ProgressProjectionIsMonotonic) {
  ExecutableTrajectoryArtifact artifact{
      .path_id = 5U,
      .samples = trajectoryPointSamplesFromPoints(
          std::vector<Point2>{{0.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}, {10.0, 0.0}}),
      .current_s_m = 20.0,
  };

  const ExecutableTrajectoryProgress progress =
      updateExecutableTrajectoryProgress(artifact, Point2{5.0, 5.0});
  ASSERT_TRUE(progress.valid);
  EXPECT_GE(artifact.current_s_m, 20.0);
}

TEST(TrajectoryRepair, ProgressDoesNotAdvanceWhenProjectionDiverged) {
  ExecutableTrajectoryArtifact artifact{
      .path_id = 5U,
      .samples = lineSamples(20.0),
      .current_s_m = 5.0,
  };

  const ExecutableTrajectoryProgress progress =
      updateExecutableTrajectoryProgress(artifact, Point2{20.0, 10.0}, 3.0);

  EXPECT_FALSE(progress.valid);
  EXPECT_TRUE(progress.diverged);
  EXPECT_GT(progress.cross_track_m, 3.0);
  EXPECT_DOUBLE_EQ(artifact.current_s_m, 5.0);
}

TEST(TrajectoryRepair, ExecutableSuffixIgnoresBlockerBehindProgress) {
  OccupancyGrid2D grid = freeGrid();
  grid.setOccupied(GridIndex{5, 10});
  ExecutableTrajectoryArtifact artifact{
      .path_id = 7U,
      .samples = lineSamples(30.0),
      .current_s_m = 10.0,
  };
  const ExecutableTrajectoryProgress progress =
      updateExecutableTrajectoryProgress(artifact, Point2{10.0, 0.0}, 3.0);
  const ExecutableSuffixDecision decision =
      evaluateExecutableSuffix(grid, artifact, progress, 0.5);

  EXPECT_TRUE(decision.progress.valid);
  EXPECT_FALSE(decision.blocked);
  EXPECT_FALSE(decision.exhausted);
}

TEST(TrajectoryRepair, ExecutableSuffixFindsBlockerAheadOfProgress) {
  OccupancyGrid2D grid = freeGrid();
  grid.setOccupied(GridIndex{15, 10});
  ExecutableTrajectoryArtifact artifact{
      .path_id = 7U,
      .samples = lineSamples(30.0),
      .current_s_m = 10.0,
  };
  const ExecutableTrajectoryProgress progress =
      updateExecutableTrajectoryProgress(artifact, Point2{10.0, 0.0}, 3.0);
  const ExecutableSuffixDecision decision =
      evaluateExecutableSuffix(grid, artifact, progress, 0.5);

  ASSERT_TRUE(decision.blocked);
  ASSERT_TRUE(decision.blocked_span.has_value());
  const BlockedSpan blocked = decision.blocked_span.value_or(BlockedSpan{});
  EXPECT_EQ(blocked.trigger, BlockedSpanTrigger::kRawOccupied);
  EXPECT_GT(blocked.first_blocked_s_m, progress.projected_s_m);
}

TEST(TrajectoryRepair,
     PublishedSuccessorMismatchKeepsAcceptedExecutableSuffixProtectedOnce) {
  constexpr std::uint64_t kAcceptedPathId{60U};
  constexpr std::uint64_t kPublishedPendingPathId{74U};
  OccupancyGrid2D grid = freeGrid();
  grid.setOccupied(GridIndex{15, 10});
  ExecutableTrajectoryArtifact accepted_artifact{
      .path_id = kAcceptedPathId,
      .samples = lineSamples(30.0),
      .current_s_m = 10.0,
  };

  ASSERT_NE(accepted_artifact.path_id, kPublishedPendingPathId);
  const ExecutableTrajectoryProgress progress =
      updateExecutableTrajectoryProgress(accepted_artifact, Point2{10.0, 0.0}, 3.0);
  const ExecutableSuffixDecision suffix =
      evaluateExecutableSuffix(grid, accepted_artifact, progress, 0.5);

  ASSERT_TRUE(suffix.blocked);
  const BlockedSpan blocked_span = suffix.blocked_span.value_or(BlockedSpan{});
  EXPECT_EQ(blocked_span.trigger, BlockedSpanTrigger::kRawOccupied);
  EXPECT_EQ(classifyRuntimeBlockerHandoff(accepted_artifact.path_id, std::nullopt),
            RuntimeBlockerHandoffAction::kBegin);
  EXPECT_EQ(classifyRuntimeBlockerHandoff(accepted_artifact.path_id,
                                          accepted_artifact.path_id),
            RuntimeBlockerHandoffAction::kAlreadyPending);
}

TEST(TrajectoryRepair,
     MissionGoalAckAdoptsExecutablePathBeforeSingleRawBlockerHandoff) {
  constexpr std::uint64_t kMissionGoalPathId{101U};
  ASSERT_TRUE(trajectoryActivationAckRequired(TrajectoryActivationAckContract{
      .explicitly_required = true,
      .endpoint_semantics = TrajectoryEndpointSemantics::kMissionGoal,
  }));
  const TruncationSuffixAckEvaluation ack = evaluateOrdinaryTrajectoryAck(
      kMissionGoalPathId, kMissionGoalPathId, TruncationSuffixAckDecision::kAccepted);
  ASSERT_EQ(ack.action, TruncationSuffixAckAction::kAdopt);
  ASSERT_TRUE(trajectoryAckClearsPending(ack.action));

  OccupancyGrid2D grid = freeGrid();
  grid.setOccupied(GridIndex{15, 10});
  ExecutableTrajectoryArtifact adopted_artifact{
      .path_id = kMissionGoalPathId,
      .samples = lineSamples(30.0),
      .current_s_m = 10.0,
  };
  const ExecutableTrajectoryProgress progress =
      updateExecutableTrajectoryProgress(adopted_artifact, Point2{10.0, 0.0}, 3.0);
  const ExecutableSuffixDecision suffix =
      evaluateExecutableSuffix(grid, adopted_artifact, progress, 0.5);

  ASSERT_TRUE(suffix.blocked);
  EXPECT_EQ(classifyRuntimeBlockerHandoff(kMissionGoalPathId, std::nullopt),
            RuntimeBlockerHandoffAction::kBegin);
  EXPECT_EQ(classifyRuntimeBlockerHandoff(kMissionGoalPathId, kMissionGoalPathId),
            RuntimeBlockerHandoffAction::kAlreadyPending);
}

TEST(TrajectoryRepair, DefinitiveSnapshotRejectClearsMissionGoalPendingForRetry) {
  constexpr std::uint64_t kPendingPathId{102U};
  ASSERT_TRUE(trajectoryActivationAckRequired(TrajectoryActivationAckContract{
      .explicitly_required = true,
      .endpoint_semantics = TrajectoryEndpointSemantics::kMissionGoal,
  }));

  for (const RawSnapshotRelation relation :
       {RawSnapshotRelation::kPolicyMismatch, RawSnapshotRelation::kRetiredProducer,
        RawSnapshotRelation::kMalformed}) {
    ASSERT_EQ(classifyRawSnapshotTrajectoryDisposition(relation),
              RawSnapshotTrajectoryDisposition::kReject);
    const TruncationSuffixAckEvaluation ack = evaluateOrdinaryTrajectoryAck(
        kPendingPathId, kPendingPathId, TruncationSuffixAckDecision::kRejected);
    EXPECT_EQ(ack.action, TruncationSuffixAckAction::kRetry);
    EXPECT_TRUE(trajectoryAckClearsPending(ack.action));
  }
}

TEST(TrajectoryRepair, ExecutableSuffixRiskIgnoresObstacleBehindProgress) {
  OccupancyGrid2D grid = freeGrid();
  grid.setOccupied(GridIndex{5, 10});
  const ObstacleRiskField risk_field =
      ObstacleRiskField::build(grid, ObstacleRiskPolicy{1.0, 4.0});
  ExecutableTrajectoryArtifact artifact{
      .path_id = 60U,
      .samples = lineSamples(30.0),
      .current_s_m = 10.0,
  };
  const ExecutableTrajectoryProgress progress =
      updateExecutableTrajectoryProgress(artifact, Point2{10.0, 0.0}, 3.0);

  const PathRiskScore full_risk = risk_field.evaluate(grid, artifact.samples);
  const std::optional<PathRiskScore> suffix_risk =
      evaluateExecutableSuffixRisk(grid, risk_field, artifact, progress);

  ASSERT_TRUE(suffix_risk.has_value());
  const PathRiskScore observed_suffix_risk =
      suffix_risk.value_or(PathRiskScore{.outside_bounds = true});
  EXPECT_TRUE(full_risk.intersects_raw_occupied);
  EXPECT_TRUE(observed_suffix_risk.hardValid());
  EXPECT_EQ(observed_suffix_risk.worst_tier, ObstacleRiskTier::kPreferred);
}

TEST(TrajectoryRepair, TruncationHoldCaptureRequiresPositionAndSpeed) {
  EXPECT_TRUE(
      truncationHoldCaptured(Point2{10.2, 20.1}, 0.3, Point2{10.0, 20.0}, 1.0, 0.5));
  EXPECT_FALSE(
      truncationHoldCaptured(Point2{11.1, 20.0}, 0.3, Point2{10.0, 20.0}, 1.0, 0.5));
  EXPECT_FALSE(
      truncationHoldCaptured(Point2{10.2, 20.1}, 0.6, Point2{10.0, 20.0}, 1.0, 0.5));
}

TEST(TrajectoryRepair, ExecutableSuffixKeepsSoftRiskPathExecutable) {
  OccupancyGrid2D grid = freeGrid();
  grid.setOccupied(GridIndex{20, 12});
  ExecutableTrajectoryArtifact artifact{
      .path_id = 7U,
      .samples = lineSamples(30.0),
      .current_s_m = 5.0,
  };
  const ExecutableTrajectoryProgress progress =
      updateExecutableTrajectoryProgress(artifact, Point2{5.0, 0.0}, 3.0);
  const ExecutableSuffixDecision decision =
      evaluateExecutableSuffix(grid, artifact, progress, 0.5);

  EXPECT_FALSE(decision.blocked);
}

TEST(TrajectoryRepair, ExecutableSuffixReportsExhaustedAtTerminal) {
  OccupancyGrid2D grid = freeGrid();
  ExecutableTrajectoryArtifact artifact{
      .path_id = 7U,
      .samples = lineSamples(30.0),
      .current_s_m = 29.0,
  };
  const ExecutableTrajectoryProgress progress =
      updateExecutableTrajectoryProgress(artifact, Point2{30.0, 0.0}, 3.0);
  const ExecutableSuffixDecision decision =
      evaluateExecutableSuffix(grid, artifact, progress, 0.5);

  EXPECT_TRUE(decision.exhausted);
  EXPECT_FALSE(decision.blocked);
}

TEST(TrajectoryRepair, ProhibitedSpanEndsAtFirstSafeStation) {
  OccupancyGrid2D grid = freeGrid();
  for (int x = 10; x <= 15; ++x) {
    grid.setOccupied(GridIndex{x, 10});
  }
  const std::vector<TrajectoryPointSample> samples = lineSamples(40.0);

  const auto span = findFirstRawOccupiedBlockedSpan(grid, samples, 0.0);

  ASSERT_TRUE(span.has_value());
  EXPECT_EQ(span->trigger, BlockedSpanTrigger::kRawOccupied);
  EXPECT_GE(span->first_blocked_s_m, 9.0);
  EXPECT_LE(span->first_blocked_s_m, 10.5);
  EXPECT_GT(span->last_blocked_s_m, 15.0);
  EXPECT_LE(span->last_blocked_s_m, 16.5);
}

TEST(TrajectoryRepair, ProhibitedScannerReturnsFirstOfTwoSpans) {
  OccupancyGrid2D grid = freeGrid();
  for (int x = 10; x <= 15; ++x) {
    grid.setOccupied(GridIndex{x, 10});
  }
  for (int x = 30; x <= 35; ++x) {
    grid.setOccupied(GridIndex{x, 10});
  }

  const auto span = findFirstRawOccupiedBlockedSpan(grid, lineSamples(50.0), 5.0);

  ASSERT_TRUE(span.has_value());
  EXPECT_LT(span->first_blocked_s_m, 16.0);
  EXPECT_LT(span->last_blocked_s_m, 20.0);
}

TEST(TrajectoryRepair, ProhibitedScannerKeepsExitOfBlockedEscapePrefix) {
  OccupancyGrid2D grid = freeGrid();
  for (int x = 0; x <= 8; ++x) {
    grid.setOccupied(GridIndex{x, 10});
  }

  const auto span = findFirstRawOccupiedBlockedSpan(grid, lineSamples(30.0), 4.0);

  ASSERT_TRUE(span.has_value());
  EXPECT_NEAR(span->first_blocked_s_m, 4.0, 0.26);
  EXPECT_GT(span->last_blocked_s_m, 8.0);
}

TEST(TrajectoryRepair, ProhibitedScannerVisitsEveryRuntimeLineCell) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 10, 10}};
  grid.setOccupied(GridIndex{1, 4});
  const Point2 start{0.058, 0.5074};
  const Point2 end{1.0859, 7.0699};
  const std::vector<TrajectoryPointSample> samples =
      trajectoryPointSamplesFromPoints(std::vector<Point2>{start, end});

  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  const std::optional<GridIndex> end_cell = grid.worldToCell(end);
  ASSERT_TRUE(start_cell.has_value());
  ASSERT_TRUE(end_cell.has_value());
  const std::vector<GridIndex> runtime_cells =
      grid.cellsOnLine(start_cell.value(), end_cell.value());
  ASSERT_NE(std::ranges::find(runtime_cells, GridIndex{1, 4}), runtime_cells.end());

  const auto span = findFirstRawOccupiedBlockedSpan(grid, samples, 0.0);

  ASSERT_TRUE(span.has_value());
  const BlockedSpan& blocked = span.value();
  ASSERT_TRUE(blocked.first_cell_available);
  ASSERT_TRUE(blocked.last_cell_available);
  EXPECT_EQ(blocked.first_cell, (GridIndex{1, 4}));
  EXPECT_EQ(blocked.last_cell, (GridIndex{1, 5}));
  EXPECT_GT(blocked.last_blocked_s_m, blocked.first_blocked_s_m);
}

TEST(TrajectoryRepair, ReconnectCandidatesUseBlockedSpanEndAndGridFallback) {
  OccupancyGrid2D planning = freeGrid();
  OccupancyGrid2D runtime = freeGrid();
  planning.setOccupied(GridIndex{30, 10});
  const ExecutableTrajectoryArtifact artifact{
      .path_id = 7U,
      .mission_goal = Point2{100.0, 0.0},
      .samples = lineSamples(100.0),
      .current_s_m = 5.0,
  };
  const BlockedSpan blocked{
      .first_blocked_s_m = 15.0,
      .last_blocked_s_m = 20.0,
  };
  const std::array margins{10.0, 20.0, 90.0};
  const std::array<const OccupancyGrid2D*, 2U> grids{&planning, &runtime};

  const std::vector<ReconnectCandidate> candidates =
      makeReconnectCandidates(artifact, blocked, 10.0, margins, grids);

  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_DOUBLE_EQ(candidates[0].reconnect_s_m, 30.0);
  EXPECT_DOUBLE_EQ(candidates[1].reconnect_s_m, 40.0);
}

TEST(TrajectoryRepair, GeneratesB10ThroughB100FromBlockedSpanEnd) {
  const ExecutableTrajectoryArtifact artifact{
      .path_id = 7U,
      .mission_goal = Point2{150.0, 0.0},
      .samples = lineSamples(150.0),
      .current_s_m = 12.0,
  };
  const BlockedSpan blocked{
      .first_blocked_s_m = 20.0,
      .last_blocked_s_m = 30.0,
  };
  const std::array margins{10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0};
  const OccupancyGrid2D grid = freeGrid();
  const std::array<const OccupancyGrid2D*, 1U> grids{&grid};

  const std::vector<ReconnectCandidate> candidates =
      makeReconnectCandidates(artifact, blocked, 15.0, margins, grids);

  ASSERT_EQ(candidates.size(), margins.size());
  for (std::size_t index = 0U; index < margins.size(); ++index) {
    EXPECT_DOUBLE_EQ(candidates[index].margin_m, margins[index]);
    EXPECT_DOUBLE_EQ(candidates[index].reconnect_s_m, 30.0 + margins[index]);
    EXPECT_GT(candidates[index].reconnect_s_m, std::max(artifact.current_s_m, 15.0));
  }
}

TEST(TrajectoryRepair, SkipsReconnectBlockedOnBothGridsAndPastGoal) {
  OccupancyGrid2D planning = freeGrid();
  OccupancyGrid2D runtime = freeGrid();
  planning.setOccupied(GridIndex{30, 10});
  runtime.setOccupied(GridIndex{30, 10});
  const ExecutableTrajectoryArtifact artifact{
      .path_id = 7U,
      .mission_goal = Point2{50.0, 0.0},
      .samples = lineSamples(50.0),
      .current_s_m = 5.0,
  };
  const BlockedSpan blocked{
      .first_blocked_s_m = 15.0,
      .last_blocked_s_m = 20.0,
  };
  const std::array margins{10.0, 40.0};
  const std::array<const OccupancyGrid2D*, 2U> grids{&planning, &runtime};

  const std::vector<ReconnectCandidate> candidates =
      makeReconnectCandidates(artifact, blocked, 10.0, margins, grids);

  EXPECT_TRUE(candidates.empty());
}

TEST(TrajectoryRepair, KeepsReconnectInsideHardWindowForFinalValidation) {
  ExecutableTrajectoryArtifact artifact{
      .path_id = 7U,
      .mission_goal = Point2{100.0, 0.0},
      .samples = lineSamples(100.0),
      .current_s_m = 5.0,
  };
  artifact.samples.front().vertical_hard_window_active = true;
  artifact.samples.back().vertical_hard_window_active = true;
  const BlockedSpan blocked{
      .first_blocked_s_m = 15.0,
      .last_blocked_s_m = 20.0,
  };
  const std::array margins{10.0};
  const OccupancyGrid2D grid = freeGrid();
  const std::array<const OccupancyGrid2D*, 1U> grids{&grid};

  const std::vector<ReconnectCandidate> candidates =
      makeReconnectCandidates(artifact, blocked, 10.0, margins, grids);

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_DOUBLE_EQ(candidates.front().reconnect_s_m, 30.0);
  EXPECT_TRUE(candidates.front().reconnect_sample.vertical_hard_window_active);
}

TEST(TrajectoryRepair, StitchKeepsOldSuffixGeometry) {
  const ExecutableTrajectoryArtifact artifact{
      .path_id = 8U,
      .mission_goal = Point2{100.0, 0.0},
      .samples = trajectoryPointSamplesFromPoints(std::vector<Point2>{
          {0.0, 0.0}, {30.0, 0.0}, {40.0, 10.0}, {70.0, 10.0}, {100.0, 0.0}}),
  };
  const std::vector<TrajectoryPointSample> repaired = trajectoryPointSamplesFromPoints(
      std::vector<Point2>{{10.0, 0.0}, {20.0, 15.0}, {40.0, 10.0}});

  const TrajectoryRepairStitchResult result =
      stitchTrajectoryRepair(repaired, artifact, artifact.samples[2].s_m);

  ASSERT_TRUE(result.valid);
  ASSERT_GE(result.samples.size(), 5U);
  EXPECT_EQ(result.samples.front().point.x, 10.0);
  EXPECT_EQ(result.samples.back().point.x, 100.0);
  EXPECT_EQ(result.samples.back().point.y, 0.0);
  EXPECT_TRUE(trajectorySamplesAreUsable(result.samples));
}

TEST(TrajectoryRepair, StitchedOldSuffixStillExposesSecondConflict) {
  OccupancyGrid2D grid = freeGrid();
  grid.setOccupied(GridIndex{80, 10});
  const ExecutableTrajectoryArtifact artifact{
      .path_id = 9U,
      .mission_goal = Point2{100.0, 0.0},
      .samples = lineSamples(100.0),
  };
  const std::vector<TrajectoryPointSample> repaired = trajectoryPointSamplesFromPoints(
      std::vector<Point2>{{10.0, 0.0}, {25.0, 5.0}, {40.0, 0.0}});

  const TrajectoryRepairStitchResult stitched =
      stitchTrajectoryRepair(repaired, artifact, 40.0);

  ASSERT_TRUE(stitched.valid);
  std::vector<Point2> stitched_points;
  stitched_points.reserve(stitched.samples.size());
  for (const TrajectoryPointSample& sample : stitched.samples) {
    stitched_points.push_back(sample.point);
  }
  EXPECT_FALSE(pathIsTraversable(grid, stitched_points));
}

} // namespace drone_city_nav
