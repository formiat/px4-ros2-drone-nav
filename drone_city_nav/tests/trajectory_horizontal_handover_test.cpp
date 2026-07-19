#include "drone_city_nav/trajectory_horizontal_handover.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<TrajectoryPointSample>
lineSamples(const Point2 start, const Point2 end, const double altitude_m = 12.0) {
  std::vector<Point2> points;
  constexpr std::size_t kSteps = 80U;
  points.reserve(kSteps + 1U);
  for (std::size_t i = 0U; i <= kSteps; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(kSteps);
    points.push_back(Point2{start.x * (1.0 - ratio) + end.x * ratio,
                            start.y * (1.0 - ratio) + end.y * ratio});
  }
  std::vector<TrajectoryPointSample> samples = trajectoryPointSamplesFromPoints(points);
  assignTrajectorySampleAltitude(samples, altitude_m);
  return samples;
}

[[nodiscard]] OccupancyGrid2D freeGrid() {
  OccupancyGrid2D grid{GridBounds{-20.0, -40.0, 0.5, 240, 160}};
  grid.reset(CellState::kFree);
  return grid;
}

[[nodiscard]] HorizontalTrajectoryHandoverState movingState() {
  return HorizontalTrajectoryHandoverState{
      .current_position = Point2{10.0, 0.0},
      .current_horizontal_speed_mps = 10.0,
      .current_position_valid = true,
      .current_horizontal_speed_valid = true,
  };
}

} // namespace

TEST(TrajectoryHorizontalHandover, BuildsTraversablePredictedPrefixBridge) {
  const std::vector<TrajectoryPointSample> current =
      lineSamples(Point2{0.0, 0.0}, Point2{80.0, 0.0});
  const std::vector<TrajectoryPointSample> candidate =
      lineSamples(Point2{0.0, 4.0}, Point2{80.0, 4.0});
  const OccupancyGrid2D grid = freeGrid();

  const HorizontalTrajectoryHandoverResult result =
      buildHorizontalTrajectoryHandover(current, candidate, movingState(), {}, &grid);

  ASSERT_TRUE(result.applied) << result.reason
                              << " heading=" << result.max_sample_heading_delta_rad
                              << " curvature=" << result.max_abs_curvature_1pm;
  EXPECT_TRUE(result.attempted);
  EXPECT_STREQ(result.reason, "predicted_prefix_bridge");
  EXPECT_NEAR(result.old_projection_s_m, 10.0, 0.1);
  EXPECT_NEAR(result.old_join_s_m, 16.0, 0.1);
  EXPECT_GT(result.stitched_join_s_m, result.old_join_s_m);
  EXPECT_LT(result.join_distance_m, 15.0);
  ASSERT_TRUE(trajectorySamplesAreUsable(result.samples));
  EXPECT_NEAR(result.samples.front().point.x, 10.0, 0.1);
  EXPECT_NEAR(result.samples.front().point.y, 0.0, 0.1);
  EXPECT_NEAR(result.samples.back().point.x, 80.0, 0.1);
  EXPECT_NEAR(result.samples.back().point.y, 4.0, 0.1);
}

TEST(TrajectoryHorizontalHandover, DoesNotRewriteCompatibleUpdate) {
  const std::vector<TrajectoryPointSample> current =
      lineSamples(Point2{0.0, 0.0}, Point2{80.0, 0.0});
  const std::vector<TrajectoryPointSample> candidate =
      lineSamples(Point2{0.0, 0.5}, Point2{80.0, 0.5});
  const OccupancyGrid2D grid = freeGrid();

  const HorizontalTrajectoryHandoverResult result =
      buildHorizontalTrajectoryHandover(current, candidate, movingState(), {}, &grid);

  EXPECT_FALSE(result.applied);
  EXPECT_TRUE(result.attempted);
  EXPECT_STREQ(result.reason, "already_compatible");
}

TEST(TrajectoryHorizontalHandover, RejectsDirectionChangeThatExceedsCurvatureLimit) {
  const std::vector<TrajectoryPointSample> current =
      lineSamples(Point2{0.0, 0.0}, Point2{80.0, 0.0});
  const std::vector<TrajectoryPointSample> candidate =
      lineSamples(Point2{10.0, 0.0}, Point2{66.0, 56.0});
  const OccupancyGrid2D grid = freeGrid();

  const HorizontalTrajectoryHandoverResult result =
      buildHorizontalTrajectoryHandover(current, candidate, movingState(), {}, &grid);

  EXPECT_FALSE(result.applied);
  EXPECT_STREQ(result.reason, "geometry_limit_exceeded");
  EXPECT_GT(result.max_abs_curvature_1pm, 0.15);
}

TEST(TrajectoryHorizontalHandover, RejectsBridgeBlockedByCurrentGrid) {
  const std::vector<TrajectoryPointSample> current =
      lineSamples(Point2{0.0, 0.0}, Point2{80.0, 0.0});
  const std::vector<TrajectoryPointSample> candidate =
      lineSamples(Point2{0.0, 4.0}, Point2{80.0, 4.0});
  OccupancyGrid2D grid = freeGrid();
  const std::optional<GridIndex> blocker = grid.worldToCell(Point2{22.0, 2.0});
  ASSERT_TRUE(blocker.has_value());
  if (!blocker.has_value()) {
    return;
  }
  grid.setOccupied(blocker.value());
  grid.rebuildInflation(1.0);

  const HorizontalTrajectoryHandoverResult result =
      buildHorizontalTrajectoryHandover(current, candidate, movingState(), {}, &grid);

  EXPECT_FALSE(result.applied);
  EXPECT_STREQ(result.reason, "non_traversable");
}

TEST(TrajectoryHorizontalHandover, RejectsDistantCandidate) {
  const std::vector<TrajectoryPointSample> current =
      lineSamples(Point2{0.0, 0.0}, Point2{80.0, 0.0});
  const std::vector<TrajectoryPointSample> candidate =
      lineSamples(Point2{0.0, 30.0}, Point2{80.0, 30.0});
  const OccupancyGrid2D grid = freeGrid();

  const HorizontalTrajectoryHandoverResult result =
      buildHorizontalTrajectoryHandover(current, candidate, movingState(), {}, &grid);

  EXPECT_FALSE(result.applied);
  EXPECT_STREQ(result.reason, "join_distance_exceeded");
}

TEST(TrajectoryHorizontalHandover, RequiresValidationGridByDefault) {
  const std::vector<TrajectoryPointSample> current =
      lineSamples(Point2{0.0, 0.0}, Point2{80.0, 0.0});
  const std::vector<TrajectoryPointSample> candidate =
      lineSamples(Point2{0.0, 4.0}, Point2{80.0, 4.0});

  const HorizontalTrajectoryHandoverResult result =
      buildHorizontalTrajectoryHandover(current, candidate, movingState());

  EXPECT_FALSE(result.applied);
  EXPECT_STREQ(result.reason, "validation_grid_unavailable");
}

TEST(TrajectoryHorizontalHandover, PreservesUpcomingHardWindowMetadata) {
  const std::vector<TrajectoryPointSample> current =
      lineSamples(Point2{0.0, 0.0}, Point2{80.0, 0.0}, 15.0);
  std::vector<TrajectoryPointSample> candidate =
      lineSamples(Point2{0.0, 4.0}, Point2{80.0, 4.0}, 8.0);
  for (TrajectoryPointSample& sample : candidate) {
    if (sample.s_m >= 40.0 && sample.s_m <= 55.0) {
      sample.vertical_hard_window_active = true;
      sample.vertical_safe_min_z_m = 7.0;
      sample.vertical_safe_max_z_m = 9.0;
      sample.vertical_gate_z_m = 8.0;
      sample.vertical_profile_passage_id = "opening";
    }
  }
  const OccupancyGrid2D grid = freeGrid();

  const HorizontalTrajectoryHandoverResult result =
      buildHorizontalTrajectoryHandover(current, candidate, movingState(), {}, &grid);

  ASSERT_TRUE(result.applied) << result.reason
                              << " heading=" << result.max_sample_heading_delta_rad
                              << " curvature=" << result.max_abs_curvature_1pm;
  EXPECT_TRUE(std::ranges::any_of(result.samples, [](const auto& sample) {
    return sample.vertical_hard_window_active &&
           sample.vertical_profile_passage_id == "opening";
  }));
  EXPECT_TRUE(std::isfinite(result.candidate_station_offset_m));
}

TEST(TrajectoryHorizontalHandover, PreservesActiveHardWindowBeforeBridge) {
  std::vector<TrajectoryPointSample> current =
      lineSamples(Point2{0.0, 0.0}, Point2{80.0, 0.0}, 8.0);
  std::vector<TrajectoryPointSample> candidate =
      lineSamples(Point2{0.0, 4.0}, Point2{80.0, 4.0}, 8.0);
  for (std::vector<TrajectoryPointSample>* samples : {&current, &candidate}) {
    for (TrajectoryPointSample& sample : *samples) {
      if (sample.s_m >= 8.0 && sample.s_m <= 25.0) {
        sample.vertical_hard_window_active = true;
        sample.vertical_safe_min_z_m = 7.0;
        sample.vertical_safe_max_z_m = 9.0;
        sample.vertical_gate_z_m = 8.0;
        sample.vertical_profile_passage_id = "opening";
      }
    }
  }
  const OccupancyGrid2D grid = freeGrid();

  const HorizontalTrajectoryHandoverResult result =
      buildHorizontalTrajectoryHandover(current, candidate, movingState(), {}, &grid);

  ASSERT_TRUE(result.applied) << result.reason;
  EXPECT_TRUE(result.hard_window_prefix_preserved);
  EXPECT_GE(result.old_join_s_m, 28.0);
  EXPECT_GE(result.candidate_join_s_m, 28.0);
  EXPECT_TRUE(std::ranges::any_of(result.samples, [](const auto& sample) {
    return sample.vertical_hard_window_active &&
           sample.vertical_profile_passage_id == "opening";
  }));
}

TEST(TrajectoryHorizontalHandover, RejectsDifferentActiveHardWindows) {
  std::vector<TrajectoryPointSample> current =
      lineSamples(Point2{0.0, 0.0}, Point2{80.0, 0.0}, 8.0);
  std::vector<TrajectoryPointSample> candidate =
      lineSamples(Point2{0.0, 4.0}, Point2{80.0, 4.0}, 8.0);
  for (TrajectoryPointSample& sample : current) {
    if (sample.s_m >= 8.0 && sample.s_m <= 25.0) {
      sample.vertical_hard_window_active = true;
      sample.vertical_profile_passage_id = "opening_a";
    }
  }
  for (TrajectoryPointSample& sample : candidate) {
    if (sample.s_m >= 8.0 && sample.s_m <= 25.0) {
      sample.vertical_hard_window_active = true;
      sample.vertical_profile_passage_id = "opening_b";
    }
  }
  const OccupancyGrid2D grid = freeGrid();

  const HorizontalTrajectoryHandoverResult result =
      buildHorizontalTrajectoryHandover(current, candidate, movingState(), {}, &grid);

  EXPECT_FALSE(result.applied);
  EXPECT_STREQ(result.reason, "hard_window_prefix_not_reconnectable");
}

} // namespace drone_city_nav
