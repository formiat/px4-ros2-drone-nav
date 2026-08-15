#include "drone_city_nav/tracking_objective.hpp"

#include <gtest/gtest.h>

#include <cstdint>

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

TEST(TrackingObjective, ReportsRawClearLineOfSight) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 10, 10}};

  EXPECT_TRUE(
      trackingLineOfSightRawClear(grid, Point3{1.5, 1.5, 5.0}, Point3{8.5, 1.5, 5.0}));
  grid.setOccupied(GridIndex{4, 1});
  EXPECT_FALSE(
      trackingLineOfSightRawClear(grid, Point3{1.5, 1.5, 5.0}, Point3{8.5, 1.5, 5.0}));
}

TEST(TrackingObjective, SweptLineOfSightRejectsRotorContactOffCenterline) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 10, 6}};
  grid.reset(CellState::kFree);
  grid.setOccupied(GridIndex{4, 2});
  const Point3 from{1.5, 1.5, 5.0};
  const Point3 to{8.5, 1.5, 5.0};

  EXPECT_TRUE(trackingLineOfSightRawClear(grid, from, to));
  EXPECT_FALSE(trackingLineOfSightSweptRawClear(
      grid, from, to, SweptFootprintConfig{.radius_m = 0.82, .sweep_step_m = 0.25}));
}

TEST(TrackingObjective, SweptThreeDimensionalLineOfSightUsesVehicleVolume) {
  OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 1.0, 10, 6, 6}};
  grid.setOccupied(GridIndex3D{4, 2, 2});
  const Point3 from{1.5, 1.5, 2.5};
  const Point3 to{8.5, 1.5, 2.5};

  EXPECT_TRUE(trackingLineOfSightRawClear(grid, from, to));
  EXPECT_FALSE(trackingLineOfSightSweptRawClear(
      grid, from, to, SweptFootprintConfig{.radius_m = 0.82, .sweep_step_m = 0.25}));
}

TEST(TrackingObjective, KeepsCurrentTargetVisibleWhenFullPredictionIsBlocked) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 14, 6}};
  grid.reset(CellState::kFree);
  grid.setOccupied(GridIndex{8, 1});

  const DirectTrackingTargetResolution result = resolveDirectTrackingTarget(
      grid, Point3{1.5, 1.5, 5.0}, Point3{5.5, 1.5, 5.0}, Point3{11.5, 1.5, 5.0},
      SweptFootprintConfig{.radius_m = 0.1, .sweep_step_m = 0.25});

  EXPECT_TRUE(result.observed_target_visible);
  EXPECT_FALSE(result.predicted_intercept_path_clear);
  EXPECT_EQ(result.status, DirectTrackingTargetStatus::kShortenedPrediction);
  EXPECT_GT(result.selected_prediction_fraction, 0.0);
  EXPECT_LT(result.selected_prediction_fraction, 1.0);
  EXPECT_GT(result.selected_position.x, 5.5);
  EXPECT_LT(result.selected_position.x, 8.0);

  TrackingLineOfSightLifecycle lifecycle{
      TrackingLineOfSightConfig{.clear_confirmations = 2U}};
  EXPECT_FALSE(lifecycle.update(result.observed_target_visible).active);
  EXPECT_TRUE(lifecycle.update(result.observed_target_visible).active);
}

TEST(TrackingObjective, ShortensLeadWhenDirectInterceptChordIsBlocked) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 12, 12}};
  grid.reset(CellState::kFree);
  grid.setOccupied(GridIndex{3, 4});

  const DirectTrackingTargetResolution result = resolveDirectTrackingTarget(
      grid, Point3{1.5, 1.5, 5.0}, Point3{5.5, 1.5, 5.0}, Point3{5.5, 9.5, 5.0},
      SweptFootprintConfig{.radius_m = 0.1, .sweep_step_m = 0.25});

  EXPECT_TRUE(result.observed_target_visible);
  EXPECT_FALSE(result.predicted_intercept_path_clear);
  EXPECT_EQ(result.status, DirectTrackingTargetStatus::kShortenedPrediction);
  EXPECT_GT(result.selected_prediction_fraction, 0.0);
  EXPECT_LT(result.selected_prediction_fraction, 1.0);
}

TEST(TrackingObjective, ReportsCurrentTargetOcclusionSeparately) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 12, 6}};
  grid.reset(CellState::kFree);
  grid.setOccupied(GridIndex{3, 1});

  const DirectTrackingTargetResolution result = resolveDirectTrackingTarget(
      grid, Point3{1.5, 1.5, 5.0}, Point3{5.5, 1.5, 5.0}, Point3{9.5, 1.5, 5.0},
      SweptFootprintConfig{.radius_m = 0.1, .sweep_step_m = 0.25});

  EXPECT_FALSE(result.observed_target_visible);
  EXPECT_FALSE(result.predicted_intercept_path_clear);
  EXPECT_EQ(result.status, DirectTrackingTargetStatus::kObservedTargetOccluded);
  EXPECT_DOUBLE_EQ(result.selected_prediction_fraction, 0.0);
}

TEST(TrackingObjective, UsesFullPredictionWhenBothPathsAreClear) {
  OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 8, 8}};

  const Point3 predicted{11.5, 4.5, 4.0};
  const DirectTrackingTargetResolution result = resolveDirectTrackingTarget(
      grid, Point3{1.5, 1.5, 4.0}, Point3{5.5, 1.5, 4.0}, predicted,
      SweptFootprintConfig{.radius_m = 0.1, .sweep_step_m = 0.25});

  EXPECT_TRUE(result.observed_target_visible);
  EXPECT_TRUE(result.predicted_intercept_path_clear);
  EXPECT_EQ(result.status, DirectTrackingTargetStatus::kFullPrediction);
  EXPECT_DOUBLE_EQ(result.selected_prediction_fraction, 1.0);
  EXPECT_DOUBLE_EQ(result.selected_position.x, predicted.x);
}

TEST(TrackingLineOfSightLifecycle, ConfirmsEntryAndLeavesImmediatelyWhenBlocked) {
  TrackingLineOfSightLifecycle lifecycle{
      TrackingLineOfSightConfig{.clear_confirmations = 2U}};

  const TrackingLineOfSightUpdate first = lifecycle.update(true);
  EXPECT_FALSE(first.active);
  const TrackingLineOfSightUpdate entered = lifecycle.update(true);
  EXPECT_TRUE(entered.active);
  EXPECT_TRUE(entered.newly_active);
  EXPECT_EQ(entered.generation, 1U);
  const TrackingLineOfSightUpdate blocked = lifecycle.update(false);
  EXPECT_FALSE(blocked.active);
  EXPECT_TRUE(blocked.newly_inactive);
  EXPECT_EQ(blocked.generation, 1U);
}

TEST(TrackingLineOfSightLifecycle, RepeatedFlappingCreatesOneGenerationPerEntry) {
  TrackingLineOfSightLifecycle lifecycle{
      TrackingLineOfSightConfig{.clear_confirmations = 2U}};

  for (std::uint64_t generation = 1U; generation <= 3U; ++generation) {
    EXPECT_FALSE(lifecycle.update(true).active);
    const TrackingLineOfSightUpdate entered = lifecycle.update(true);
    EXPECT_TRUE(entered.active);
    EXPECT_TRUE(entered.newly_active);
    EXPECT_EQ(entered.generation, generation);

    const TrackingLineOfSightUpdate blocked = lifecycle.update(false);
    EXPECT_FALSE(blocked.active);
    EXPECT_TRUE(blocked.newly_inactive);
    EXPECT_EQ(blocked.generation, generation);
  }
}

} // namespace
} // namespace drone_city_nav
