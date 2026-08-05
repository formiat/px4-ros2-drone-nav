#include "drone_city_nav/tracking_objective.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(TrackingObjective, LeavesClearTwoDimensionalPredictionUnchanged) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 10, 10}};
  const Point3 predicted{8.5, 1.5, 18.0};

  const TrackingObjectiveResolution result =
      resolveTrackingObjective(grid, Point3{1.5, 1.5, 18.0}, predicted);

  EXPECT_EQ(result.status, TrackingObjectiveResolutionStatus::kUnchanged);
  EXPECT_DOUBLE_EQ(result.resolved_position.x, predicted.x);
  EXPECT_DOUBLE_EQ(result.resolved_fraction, 1.0);
}

TEST(TrackingObjective, StopsAtFirstRawOccupiedCell) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 10, 10}};
  grid.setOccupied(GridIndex{4, 1});

  const TrackingObjectiveResolution result =
      resolveTrackingObjective(grid, Point3{1.5, 1.5, 18.0}, Point3{8.5, 1.5, 18.0});

  EXPECT_EQ(result.status, TrackingObjectiveResolutionStatus::kClippedRawOccupied);
  EXPECT_GT(result.resolved_position.x, 1.5);
  EXPECT_LT(result.resolved_position.x, 4.0);
  EXPECT_LT(result.resolved_fraction, 1.0);
}

TEST(TrackingObjective, NeverSelectsFreeSpaceBeyondRawWall) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 12, 3}};
  grid.setOccupied(GridIndex{5, 1});

  const TrackingObjectiveResolution result =
      resolveTrackingObjective(grid, Point3{1.5, 1.5, 10.0}, Point3{10.5, 1.5, 10.0});

  EXPECT_LT(result.resolved_position.x, 5.0);
}

TEST(TrackingObjective, TreatsUnknownAndOutsideGridAsNotRawOccupied) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 4, 4}};
  const Point3 predicted{8.0, 1.5, 18.0};

  const TrackingObjectiveResolution result =
      resolveTrackingObjective(grid, Point3{1.5, 1.5, 18.0}, predicted);

  EXPECT_EQ(result.status, TrackingObjectiveResolutionStatus::kUnchanged);
  EXPECT_DOUBLE_EQ(result.resolved_position.x, predicted.x);
}

TEST(TrackingObjective, FallsBackWhenNoPositiveRawFreeSegmentExists) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 4, 4}};
  grid.setOccupied(GridIndex{1, 1});
  const Point3 observed{1.5, 1.5, 18.0};

  const TrackingObjectiveResolution result =
      resolveTrackingObjective(grid, observed, Point3{3.5, 1.5, 18.0});

  EXPECT_EQ(result.status, TrackingObjectiveResolutionStatus::kFallbackObserved);
  EXPECT_DOUBLE_EQ(result.resolved_position.x, observed.x);
  EXPECT_DOUBLE_EQ(result.resolved_fraction, 0.0);
}

TEST(TrackingObjective, ClipsThreeDimensionalPredictionAgainstRawVoxel) {
  OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 1.0, 10, 4, 4}};
  grid.setOccupied(GridIndex3D{4, 1, 1});

  const TrackingObjectiveResolution result =
      resolveTrackingObjective(grid, Point3{1.5, 1.5, 1.5}, Point3{8.5, 1.5, 1.5});

  EXPECT_EQ(result.status, TrackingObjectiveResolutionStatus::kClippedRawOccupied);
  EXPECT_LT(result.resolved_position.x, 4.0);
}

TEST(TrackingObjective, RejectsInvalidSampleSpacing) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 4, 4}};

  const TrackingObjectiveResolution result =
      resolveTrackingObjective(grid, Point3{1.5, 1.5, 1.5}, Point3{2.5, 1.5, 1.5}, 0.0);

  EXPECT_EQ(result.status, TrackingObjectiveResolutionStatus::kInvalidInput);
}

} // namespace
} // namespace drone_city_nav
