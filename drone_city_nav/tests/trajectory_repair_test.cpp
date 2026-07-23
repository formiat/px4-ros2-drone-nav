#include "drone_city_nav/trajectory_repair.hpp"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D freeGrid() {
  OccupancyGrid2D grid{GridBounds{-0.5, -10.5, 1.0, 121, 22}};
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

  ASSERT_TRUE(updateExecutableTrajectoryProgress(artifact, Point2{5.0, 5.0}));
  EXPECT_GE(artifact.current_s_m, 20.0);
}

TEST(TrajectoryRepair, ProhibitedSpanEndsAtFirstSafeStation) {
  OccupancyGrid2D grid = freeGrid();
  for (int x = 10; x <= 15; ++x) {
    grid.setOccupied(GridIndex{x, 10});
  }
  const std::vector<TrajectoryPointSample> samples = lineSamples(40.0);

  const auto span = findFirstProhibitedBlockedSpan(
      grid, samples, 0.0, BlockedSpanScanConfig{.sample_step_m = 0.25});

  ASSERT_TRUE(span.has_value());
  EXPECT_EQ(span->trigger, BlockedSpanTrigger::kProhibited);
  EXPECT_GE(span->first_blocked_s_m, 9.0);
  EXPECT_LE(span->first_blocked_s_m, 10.5);
  EXPECT_GT(span->last_blocked_s_m, 15.0);
  EXPECT_LE(span->last_blocked_s_m, 16.5);
}

TEST(TrajectoryRepair, RawClearanceSkipsShortRunAndReturnsLongRunExit) {
  OccupancyGrid2D grid = freeGrid();
  grid.setOccupied(GridIndex{8, 13});
  for (int x = 25; x <= 35; ++x) {
    grid.setOccupied(GridIndex{x, 13});
  }
  const std::vector<TrajectoryPointSample> samples = lineSamples(50.0);

  const auto span =
      findFirstRawClearanceBlockedSpan(grid, samples, 0.0,
                                       BlockedSpanScanConfig{
                                           .sample_step_m = 0.25,
                                           .raw_clearance_trigger_m = 5.0,
                                           .raw_min_violation_length_m = 12.0,
                                       });

  ASSERT_TRUE(span.has_value());
  EXPECT_EQ(span->trigger, BlockedSpanTrigger::kRawClearance);
  EXPECT_GT(span->first_blocked_s_m, 15.0);
  EXPECT_GT(span->last_blocked_s_m - span->first_blocked_s_m, 12.0);
  EXPECT_GT(span->last_blocked_s_m, 35.0);
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

} // namespace drone_city_nav
