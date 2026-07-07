#include "drone_city_nav/trajectory_passage_insertion.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeGrid() {
  OccupancyGrid2D grid{GridBounds{-30.0, -30.0, 1.0, 80, 80}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  return grid;
}

[[nodiscard]] PassageOpening makeOpening() {
  PassageOpening opening{};
  opening.id = "main";
  opening.structure_id = "arch";
  opening.center = Point3{0.0, 0.0, 10.0};
  opening.normal_xy = Point2{1.0, 0.0};
  opening.width_m = 4.0;
  opening.height_m = 4.0;
  opening.depth_m = 4.0;
  opening.min_z_m = 8.0;
  opening.max_z_m = 12.0;
  opening.approach_distance_m = 8.0;
  opening.exit_distance_m = 8.0;
  return opening;
}

[[nodiscard]] KnownPassageMap makeMap() {
  PassageStructure structure{};
  structure.id = "arch";
  structure.center = Point2{0.0, 0.0};
  structure.size_x_m = 10.0;
  structure.size_y_m = 12.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 20.0;
  structure.openings.push_back(makeOpening());

  KnownPassageMap map{};
  map.frame_id = "map";
  map.structures.push_back(structure);
  return map;
}

[[nodiscard]] std::vector<TrajectoryPointSample> makeLineSamples(const double y_m) {
  std::vector<TrajectoryPointSample> samples;
  for (int i = 0; i <= 20; ++i) {
    TrajectoryPointSample sample{};
    sample.point = Point2{-20.0 + static_cast<double>(i) * 2.0, y_m};
    sample.z_m = 10.0;
    sample.left_bound_m = 10.0;
    sample.right_bound_m = 10.0;
    sample.lateral_offset_m = 0.0;
    samples.push_back(sample);
  }
  populateTrajectorySampleGeometry(samples);
  return samples;
}

[[nodiscard]] PassageInsertionConfig insertionConfig() {
  PassageInsertionConfig config{};
  config.enabled = true;
  config.sample_step_m = 1.0;
  config.min_anchor_margin_m = 8.0;
  config.max_anchor_margin_m = 20.0;
  config.max_join_tangent_delta_rad = std::numbers::pi;
  config.max_join_curvature_jump_1pm = 10.0;
  config.max_lateral_shift_m = 20.0;
  config.max_candidates = 8U;
  config.max_diagnostics = 8U;
  return config;
}

} // namespace

TEST(TrajectoryPassageInsertion, DisabledReturnsNoopSamples) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);
  PassageInsertionConfig config = insertionConfig();
  config.enabled = false;

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, config, 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_FALSE(result.stats.applied);
  EXPECT_EQ(result.stats.final_reason, PassageInsertionRejectReason::kDisabled);
  ASSERT_EQ(result.samples.size(), samples.size());
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    EXPECT_DOUBLE_EQ(result.samples[i].point.x, samples[i].point.x);
    EXPECT_DOUBLE_EQ(result.samples[i].point.y, samples[i].point.y);
  }
}

TEST(TrajectoryPassageInsertion, InsertsLocalSegmentThroughKnownOpening) {
  const OccupancyGrid2D grid = makeGrid();
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);
  const KnownPassageValidationSummary before =
      validateKnownPassageTraversal(samples, &map, KnownPassageValidationConfig{});
  ASSERT_FALSE(before.valid);
  ASSERT_EQ(before.worst_reason, KnownPassageValidationReason::kOpeningVolumeMiss);

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, insertionConfig(), 10.0);

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.applied);
  EXPECT_TRUE(result.stats.applied);
  EXPECT_EQ(result.stats.inserted_count, 1U);
  EXPECT_EQ(result.stats.final_reason, PassageInsertionRejectReason::kNone);
  ASSERT_FALSE(result.stats.diagnostics.empty());
  EXPECT_TRUE(result.stats.diagnostics.front().accepted);
  EXPECT_LT(result.stats.diagnostics.front().lateral_miss_after_m,
            result.stats.diagnostics.front().lateral_miss_before_m);
  EXPECT_NEAR(distance(result.samples.front().point, samples.front().point), 0.0,
              1.0e-6);
  EXPECT_NEAR(distance(result.samples.back().point, samples.back().point), 0.0, 1.0e-6);

  const KnownPassageValidationSummary after = validateKnownPassageTraversal(
      result.samples, &map, KnownPassageValidationConfig{});
  EXPECT_TRUE(after.valid);
  EXPECT_EQ(after.worst_reason, KnownPassageValidationReason::kMatchedOpening);
}

TEST(TrajectoryPassageInsertion, RejectsCandidateThatCrossesProhibitedGrid) {
  OccupancyGrid2D grid = makeGrid();
  const GridIndex blocked_cell{30, 30};
  ASSERT_TRUE(grid.contains(blocked_cell));
  grid.setOccupied(blocked_cell);
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples = makeLineSamples(4.0);

  const PassageInsertionResult result = insertLocalPassageSegments(
      samples, grid, &map, KnownPassageValidationConfig{}, insertionConfig(), 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_FALSE(result.stats.applied);
  EXPECT_GT(result.stats.rejected_traversability, 0U);
  ASSERT_FALSE(result.stats.diagnostics.empty());
  EXPECT_EQ(result.stats.diagnostics.front().reason,
            PassageInsertionRejectReason::kNonTraversable);
}

} // namespace drone_city_nav
