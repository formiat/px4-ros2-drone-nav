#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <ranges>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(SweptFootprintTest, RejectsRotorSweepWhileCenterlineRemainsFree) {
  OccupancyGrid2D occupancy{GridBounds{0.0, 0.0, 1.0, 12, 6}};
  occupancy.reset(CellState::kFree);
  occupancy.setOccupied(GridIndex{6, 3});
  const DistanceField2D field =
      DistanceField2D::build(occupancy, 20.0, DistanceFieldSource::kOccupied);
  std::vector<float> esdf;
  std::ranges::transform(field.distancesM(), std::back_inserter(esdf),
                         [](const double value) { return static_cast<float>(value); });
  const mppi::EsdfGrid grid{12, 6, 1.0F, 0.0F, 0.0F};

  const SweptFootprintResult point_mass = validateSweptFootprint(
      grid, esdf, Point3{1.5, 2.5, 0.0}, Point3{10.5, 2.5, 0.0},
      SweptFootprintConfig{.radius_m = 0.0, .perimeter_samples = 0U});
  const SweptFootprintResult physical = validateSweptFootprint(
      grid, esdf, Point3{1.5, 2.5, 0.0}, Point3{10.5, 2.5, 0.0},
      SweptFootprintConfig{
          .radius_m = 1.0, .perimeter_samples = 8U, .sweep_step_m = 0.25});

  EXPECT_TRUE(point_mass.accepted());
  EXPECT_EQ(physical.status, SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, ReportsOutsideGridSeparatelyFromPhysicalCollision) {
  const mppi::EsdfGrid grid{4, 4, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(16U, 10.0F);

  const SweptFootprintResult result = validateFootprintAt(
      grid, esdf, Point3{0.1, 2.0, 0.0},
      SweptFootprintConfig{.radius_m = 0.5, .perimeter_samples = 8U});

  EXPECT_EQ(result.status, SweptFootprintStatus::kOutsideGrid);
}

TEST(SweptFootprintTest, DetectsCollisionAboveVehicleReferencePoint) {
  const mppi::EsdfGrid grid{8, 8, 1.0F, 0.0F, 0.0F, 8, 0.0F};
  std::vector<float> esdf(std::size_t{8U} * 8U * 8U,
                          std::numeric_limits<float>::infinity());
  esdf[(4U * 8U + 3U) * 8U + 3U] = 0.0F;
  const Point3 position{3.5, 3.5, 3.5};

  EXPECT_TRUE(validateFootprintAt(
                  grid, esdf, position,
                  SweptFootprintConfig{.radius_m = 0.0, .perimeter_samples = 0U})
                  .accepted());
  EXPECT_EQ(validateFootprintAt(grid, esdf, position,
                                SweptFootprintConfig{.radius_m = 0.2,
                                                     .lower_extent_m = 0.2,
                                                     .upper_extent_m = 1.0,
                                                     .perimeter_samples = 8U,
                                                     .radial_rings = 1U,
                                                     .axial_samples = 3U})
                .status,
            SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, RotatesPhysicalVolumeWithBodyAxis) {
  const mppi::EsdfGrid grid{8, 8, 1.0F, 0.0F, 0.0F, 8, 0.0F};
  std::vector<float> esdf(std::size_t{8U} * 8U * 8U,
                          std::numeric_limits<float>::infinity());
  esdf[(3U * 8U + 3U) * 8U + 4U] = 0.0F;
  const SweptFootprintConfig config{.radius_m = 0.2,
                                    .lower_extent_m = 0.2,
                                    .upper_extent_m = 1.0,
                                    .perimeter_samples = 8U,
                                    .radial_rings = 1U,
                                    .axial_samples = 3U};
  const Point3 position{3.5, 3.5, 3.5};

  EXPECT_TRUE(validateFootprintAt(grid, esdf, position, FootprintBodyAxis{}, config)
                  .accepted());
  EXPECT_EQ(validateFootprintAt(grid, esdf, position, FootprintBodyAxis{1.0, 0.0, 0.0},
                                config)
                .status,
            SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, ClearanceBroadPhaseReturnsAConservativeSafeBound) {
  const mppi::EsdfGrid grid{8, 8, 1.0F, 0.0F, 0.0F, 8, 0.0F};
  const std::vector<float> esdf(std::size_t{8U} * 8U * 8U, 10.0F);
  const Point3 position{3.5, 3.5, 3.5};
  SweptFootprintConfig exact_config{.radius_m = 0.82,
                                    .lower_extent_m = 0.23,
                                    .upper_extent_m = 0.35,
                                    .perimeter_samples = 12U,
                                    .radial_rings = 2U,
                                    .axial_samples = 3U};
  SweptFootprintConfig broad_phase_config = exact_config;
  broad_phase_config.safe_clearance_threshold_m = 6.0;

  const SweptFootprintResult exact =
      validateFootprintAt(grid, esdf, position, exact_config);
  const SweptFootprintResult broad_phase =
      validateFootprintAt(grid, esdf, position, broad_phase_config);

  ASSERT_TRUE(exact.accepted());
  ASSERT_TRUE(broad_phase.accepted());
  EXPECT_GE(broad_phase.minimum_clearance_m, 6.0);
  EXPECT_LE(broad_phase.minimum_clearance_m, exact.minimum_clearance_m);
}

TEST(SweptFootprintTest, RawTwoDimensionalSweepRejectsSideContact) {
  OccupancyGrid2D occupancy{GridBounds{0.0, 0.0, 1.0, 12, 6}};
  occupancy.reset(CellState::kFree);
  occupancy.setOccupied(GridIndex{6, 3});

  const SweptFootprintResult center = validateRawSweptFootprint(
      occupancy, Point3{1.5, 2.5, 5.0}, Point3{10.5, 2.5, 5.0},
      SweptFootprintConfig{.radius_m = 0.0, .sweep_step_m = 0.25});
  const SweptFootprintResult physical = validateRawSweptFootprint(
      occupancy, Point3{1.5, 2.5, 5.0}, Point3{10.5, 2.5, 5.0},
      SweptFootprintConfig{.radius_m = 1.0, .sweep_step_m = 0.25});

  EXPECT_TRUE(center.accepted());
  EXPECT_EQ(physical.status, SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, MiddlewareRawViewMatchesOwnedGridCollisionSemantics) {
  const GridBounds bounds{0.0, 0.0, 1.0, 8, 6};
  OccupancyGrid2D owned{bounds};
  owned.reset(CellState::kFree);
  owned.setOccupied(GridIndex{4, 3});
  std::vector<std::int8_t> middleware_cells(48U, 0);
  middleware_cells[3U * 8U + 4U] = 100;
  const RawOccupancyGridView2D view{bounds, middleware_cells, 100};
  const SweptFootprintConfig footprint{.radius_m = 0.75};

  const SweptFootprintResult owned_result =
      validateRawFootprintAt(owned, Point3{3.5, 3.5, 5.0}, footprint);
  const SweptFootprintResult view_result =
      validateRawFootprintAt(view, Point3{3.5, 3.5, 5.0}, footprint);

  EXPECT_EQ(view_result.status, owned_result.status);
  EXPECT_EQ(view_result.status, SweptFootprintStatus::kRawCollision);
  EXPECT_TRUE(
      validateRawFootprintAt(view, Point3{20.0, 20.0, 5.0}, footprint).accepted());
}

TEST(SweptFootprintTest, RawThreeDimensionalSweepUsesAxialExtent) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 10, 4, 6}};
  occupancy.setOccupied(GridIndex3D{4, 1, 3});
  const SweptFootprintConfig footprint{.radius_m = 0.25,
                                       .lower_extent_m = 0.2,
                                       .upper_extent_m = 1.2,
                                       .sweep_step_m = 0.25};

  const SweptFootprintResult result =
      validateRawSweptFootprint(occupancy, Point3{1.5, 1.5, 2.5}, FootprintBodyAxis{},
                                Point3{8.5, 1.5, 2.5}, FootprintBodyAxis{}, footprint);

  EXPECT_EQ(result.status, SweptFootprintStatus::kRawCollision);
}

TEST(SweptFootprintTest, RawWorldBoundaryIsNotAnArtificialObstacle) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4}};

  EXPECT_TRUE(validateRawSweptFootprint(
                  occupancy, Point3{1.5, 1.5, 1.5}, FootprintBodyAxis{},
                  Point3{8.0, 1.5, 1.5}, FootprintBodyAxis{},
                  SweptFootprintConfig{.radius_m = 0.82, .sweep_step_m = 0.25})
                  .accepted());
}

TEST(SweptFootprintTest, RawPointCloudSweepRejectsPhysicalSideContact) {
  const std::vector<Point3> obstacle_points{{5.0, 0.75, 5.0}};
  const SweptFootprintConfig footprint{.radius_m = 0.82,
                                       .lower_extent_m = 0.23,
                                       .upper_extent_m = 0.35,
                                       .sweep_step_m = 0.25};

  const SweptFootprintResult result = validateRawPointCloudSweptFootprint(
      obstacle_points, Point3{0.0, 0.0, 5.0}, FootprintBodyAxis{},
      Point3{10.0, 0.0, 5.0}, FootprintBodyAxis{}, footprint);

  EXPECT_EQ(result.status, SweptFootprintStatus::kRawCollision);
  EXPECT_DOUBLE_EQ(result.failure_point.x, obstacle_points.front().x);
  EXPECT_DOUBLE_EQ(result.failure_point.y, obstacle_points.front().y);
}

TEST(SweptFootprintTest, RawPointCloudFootprintUsesRequestedBodyAxis) {
  const std::vector<Point3> obstacle_points{{0.8, 0.0, 0.0}};
  const SweptFootprintConfig footprint{
      .radius_m = 0.2, .lower_extent_m = 0.2, .upper_extent_m = 1.0};

  EXPECT_TRUE(validateRawPointCloudFootprintAt(obstacle_points, Point3{},
                                               FootprintBodyAxis{}, footprint)
                  .accepted());
  EXPECT_EQ(validateRawPointCloudFootprintAt(
                obstacle_points, Point3{}, FootprintBodyAxis{1.0, 0.0, 0.0}, footprint)
                .status,
            SweptFootprintStatus::kRawCollision);
}

} // namespace
} // namespace drone_city_nav
