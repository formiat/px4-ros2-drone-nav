#include "drone_city_nav/clearance_field.hpp"
#include "drone_city_nav/corridor.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D corridorGrid() {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 20, 12}};
  for (int x = 0; x < grid.width(); ++x) {
    grid.setOccupied(GridIndex{x, 2});
    grid.setOccupied(GridIndex{x, 9});
  }
  return grid;
}

[[nodiscard]] CorridorConfig testConfig() {
  CorridorConfig config{};
  config.max_radius_m = 10.0;
  config.sample_step_m = 2.0;
  return config;
}

void expectSameSamples(const CorridorResult& lhs, const CorridorResult& rhs) {
  ASSERT_EQ(lhs.samples.size(), rhs.samples.size());
  for (std::size_t i = 0U; i < lhs.samples.size(); ++i) {
    EXPECT_DOUBLE_EQ(lhs.samples[i].s_m, rhs.samples[i].s_m);
    EXPECT_DOUBLE_EQ(lhs.samples[i].center.x, rhs.samples[i].center.x);
    EXPECT_DOUBLE_EQ(lhs.samples[i].center.y, rhs.samples[i].center.y);
    EXPECT_DOUBLE_EQ(lhs.samples[i].left_bound_m, rhs.samples[i].left_bound_m);
    EXPECT_DOUBLE_EQ(lhs.samples[i].right_bound_m, rhs.samples[i].right_bound_m);
    EXPECT_DOUBLE_EQ(lhs.samples[i].clearance_m, rhs.samples[i].clearance_m);
  }
}

} // namespace

TEST(Corridor, StraightPassageHasSymmetricBounds) {
  const OccupancyGrid2D grid = corridorGrid();
  const std::vector<Point2> route{{1.5, 5.5}, {18.5, 5.5}};

  const CorridorResult result = buildCorridor(route, grid, testConfig());

  ASSERT_TRUE(result.valid);
  ASSERT_GT(result.samples.size(), 2U);
  const CorridorSample middle = result.samples[result.samples.size() / 2U];
  EXPECT_NEAR(middle.left_bound_m, middle.right_bound_m, 1.0);
  EXPECT_GT(result.stats.mean_width_m, 5.0);
}

TEST(Corridor, RouteInsideProhibitedIsInvalid) {
  OccupancyGrid2D grid = corridorGrid();
  grid.setOccupied(GridIndex{5, 5});
  CorridorConfig config = testConfig();
  config.center_recovery_max_m = 0.0;
  const std::vector<Point2> route{{5.5, 5.5}, {12.5, 5.5}};

  const CorridorResult result = buildCorridor(route, grid, config);

  EXPECT_FALSE(result.valid);
  EXPECT_GT(result.stats.route_prohibited_samples, 0U);
}

TEST(Corridor, ProhibitedRouteSampleCanRecoverToNearbyFreeCenter) {
  OccupancyGrid2D grid = corridorGrid();
  grid.setOccupied(GridIndex{5, 5});
  CorridorConfig config = testConfig();
  config.sample_step_m = 1.0;
  config.center_recovery_max_m = 1.5;
  const std::vector<Point2> route{{4.5, 5.5}, {8.5, 5.5}};

  const CorridorResult result = buildCorridor(route, grid, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.stats.route_prohibited_samples, 0U);
  EXPECT_GT(result.stats.center_recovered_samples, 0U);
  EXPECT_EQ(result.stats.center_unrecoverable_samples, 0U);
  EXPECT_GT(result.stats.max_center_recovery_m, 0.0);
  EXPECT_TRUE(std::any_of(result.samples.begin(), result.samples.end(),
                          [](const CorridorSample& sample) {
                            return sample.center_recovery_m > 0.0 &&
                                   sample.center.y != sample.route_center.y;
                          }));
}

TEST(Corridor, OutsideGridLimitsBounds) {
  const OccupancyGrid2D grid = corridorGrid();
  CorridorConfig config = testConfig();
  config.max_radius_m = 30.0;
  const std::vector<Point2> route{{1.5, 5.5}, {1.5, 8.5}};

  const CorridorResult result = buildCorridor(route, grid, config);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.stats.outside_grid_samples, 0U);
}

TEST(Corridor, LocalLateralLimitClipsSideOpening) {
  OccupancyGrid2D grid = corridorGrid();
  grid.setFree(GridIndex{10, 9});
  CorridorConfig config = testConfig();
  config.sample_step_m = 1.0;
  config.lateral_limit_window_m = 4.0;
  config.lateral_limit_ratio = 1.0;
  config.lateral_limit_margin_m = 0.0;
  const std::vector<Point2> route{{1.5, 5.5}, {18.5, 5.5}};

  const CorridorResult result = buildCorridor(route, grid, config);

  ASSERT_TRUE(result.valid);
  ASSERT_GT(result.stats.lateral_limited_samples, 0U);
  ASSERT_GT(result.stats.max_lateral_bound_reduction_m, 1.0);
  const auto opening_sample = std::min_element(
      result.samples.begin(), result.samples.end(),
      [](const CorridorSample& lhs, const CorridorSample& rhs) {
        return std::abs(lhs.center.x - 10.5) < std::abs(rhs.center.x - 10.5);
      });
  ASSERT_NE(opening_sample, result.samples.end());
  EXPECT_LE(opening_sample->left_bound_m, 3.5);
}

TEST(Corridor, ParallelSampleBuildMatchesSerialResult) {
  const OccupancyGrid2D grid = corridorGrid();
  const std::vector<Point2> route{{1.5, 5.5}, {18.5, 5.5}};
  CorridorConfig serial_config = testConfig();
  serial_config.parallel_workers = 1U;
  CorridorConfig parallel_config = testConfig();
  parallel_config.parallel_workers = 3U;

  const CorridorResult serial_result = buildCorridor(route, grid, serial_config);
  const CorridorResult parallel_result = buildCorridor(route, grid, parallel_config);

  ASSERT_TRUE(serial_result.valid);
  ASSERT_TRUE(parallel_result.valid);
  EXPECT_EQ(parallel_result.stats.parallel_workers_used, 3U);
  expectSameSamples(serial_result, parallel_result);
  EXPECT_EQ(serial_result.stats.outside_grid_samples,
            parallel_result.stats.outside_grid_samples);
  EXPECT_EQ(serial_result.stats.lateral_limited_samples,
            parallel_result.stats.lateral_limited_samples);
}

TEST(Corridor, ReusesProvidedClearanceFieldWhenItCoversCorridorRadius) {
  const OccupancyGrid2D grid = corridorGrid();
  const std::vector<Point2> route{{1.5, 5.5}, {18.5, 5.5}};
  const CorridorConfig config = testConfig();
  const ClearanceField2D clearance_field =
      ClearanceField2D::build(grid, config.max_radius_m, ClearanceSource::kProhibited);

  const CorridorResult result =
      buildCorridor(CorridorInput{std::span<const Point2>{route.data(), route.size()},
                                  &grid, &clearance_field, true},
                    config);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.stats.clearance_field_reused);
  EXPECT_TRUE(result.stats.clearance_field_cache_hit);
  EXPECT_DOUBLE_EQ(result.stats.clearance_field_build_duration_ms, 0.0);
}

TEST(Corridor, RebuildsProvidedClearanceFieldWhenItDoesNotCoverCorridorRadius) {
  const OccupancyGrid2D grid = corridorGrid();
  const std::vector<Point2> route{{1.5, 5.5}, {18.5, 5.5}};
  const CorridorConfig config = testConfig();
  const ClearanceField2D clearance_field =
      ClearanceField2D::build(grid, 1.0, ClearanceSource::kProhibited);

  const CorridorResult result =
      buildCorridor(CorridorInput{std::span<const Point2>{route.data(), route.size()},
                                  &grid, &clearance_field, true},
                    config);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.stats.clearance_field_reused);
  EXPECT_FALSE(result.stats.clearance_field_cache_hit);
}

TEST(Corridor, RebuildsProvidedClearanceFieldWhenSourceIsNotProhibited) {
  const OccupancyGrid2D grid = corridorGrid();
  const std::vector<Point2> route{{1.5, 5.5}, {18.5, 5.5}};
  const CorridorConfig config = testConfig();
  const ClearanceField2D clearance_field =
      ClearanceField2D::build(grid, config.max_radius_m, ClearanceSource::kOccupied);

  const CorridorResult result =
      buildCorridor(CorridorInput{std::span<const Point2>{route.data(), route.size()},
                                  &grid, &clearance_field, true},
                    config);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.stats.clearance_field_reused);
  EXPECT_FALSE(result.stats.clearance_field_cache_hit);
}

} // namespace drone_city_nav
