#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
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

} // namespace
} // namespace drone_city_nav
