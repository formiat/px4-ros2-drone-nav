#include "drone_city_nav/corner_rounding.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] CornerRoundingConfig testConfig() {
  CornerRoundingConfig config{};
  config.enabled = true;
  config.min_radius_m = 1.0;
  config.max_radius_m = 5.0;
  config.min_segment_remainder_m = 0.5;
  config.collision_sample_step_m = 0.25;
  return config;
}

[[nodiscard]] OccupancyGrid2D emptyGrid() {
  OccupancyGrid2D grid{GridBounds{-20.0, -20.0, 0.5, 100, 100}};
  grid.reset(CellState::kFree);
  return grid;
}

} // namespace

TEST(CornerRounding, RightAngleCreatesLineArcLineTrajectory) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  OccupancyGrid2D grid = emptyGrid();

  const CornerRoundingResult result = roundCorners(path, testConfig(), &grid);

  ASSERT_EQ(result.segments.size(), 3U);
  EXPECT_EQ(result.segments[0].kind, TrajectorySegmentKind::kLine);
  EXPECT_EQ(result.segments[1].kind, TrajectorySegmentKind::kArc);
  EXPECT_EQ(result.segments[2].kind, TrajectorySegmentKind::kLine);
  EXPECT_EQ(result.stats.corners_seen, 1U);
  EXPECT_EQ(result.stats.corners_rounded, 1U);
  EXPECT_NEAR(result.segments[1].radius_m, testConfig().max_radius_m, 1.0e-9);
  EXPECT_GT(result.segments[1].length_m, 0.0);
}

TEST(CornerRounding, DisabledModeReturnsLineOnlyTrajectory) {
  CornerRoundingConfig config = testConfig();
  config.enabled = false;
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};

  const CornerRoundingResult result = roundCorners(path, config, nullptr);

  ASSERT_EQ(result.segments.size(), 2U);
  EXPECT_EQ(result.segments[0].kind, TrajectorySegmentKind::kLine);
  EXPECT_EQ(result.segments[1].kind, TrajectorySegmentKind::kLine);
  EXPECT_EQ(result.stats.corners_rounded, 0U);
}

TEST(CornerRounding, ShortSegmentsKeepSharpJoin) {
  const std::vector<Point2> path{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  OccupancyGrid2D grid = emptyGrid();

  const CornerRoundingResult result = roundCorners(path, testConfig(), &grid);

  ASSERT_EQ(result.segments.size(), 2U);
  EXPECT_EQ(result.stats.corners_seen, 1U);
  EXPECT_EQ(result.stats.corners_rounded, 0U);
  EXPECT_GT(result.stats.skipped_short_segments, 0U);
}

TEST(CornerRounding, ProhibitedCellOnArcRejectsRounding) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  CornerRoundingConfig config = testConfig();
  config.min_radius_m = 5.0;
  config.max_radius_m = 5.0;
  OccupancyGrid2D grid = emptyGrid();
  const auto blocked_cell = grid.worldToCell(Point2{8.5, 1.5});
  ASSERT_TRUE(blocked_cell.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): asserted above.
  grid.setOccupied(blocked_cell.value());
  grid.rebuildInflation(0.0);

  const CornerRoundingResult result = roundCorners(path, config, &grid);

  EXPECT_EQ(result.stats.corners_seen, 1U);
  EXPECT_EQ(result.stats.corners_rounded, 0U);
  EXPECT_GT(result.stats.skipped_collision, 0U);
  ASSERT_FALSE(result.segments.empty());
  EXPECT_TRUE(
      std::ranges::all_of(result.segments, [](const TrajectorySegment& segment) {
        return segment.kind == TrajectorySegmentKind::kLine;
      }));
}

TEST(CornerRounding, NearlyStraightPathStaysLineOnly) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
  OccupancyGrid2D grid = emptyGrid();

  const CornerRoundingResult result = roundCorners(path, testConfig(), &grid);

  ASSERT_EQ(result.segments.size(), 1U);
  EXPECT_EQ(result.segments[0].kind, TrajectorySegmentKind::kLine);
  EXPECT_EQ(result.stats.skipped_straight, 1U);
}

} // namespace drone_city_nav
