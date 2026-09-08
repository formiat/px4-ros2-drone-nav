#include "drone_city_nav/executed_horizon_clearance_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace drone_city_nav {
namespace {

// A 1 m lattice, three cells wide and deep, twenty cells long: the first ten
// cells along x are observed free, far from anything; the rest are unknown.
constexpr int kLength{20};
constexpr int kKnownLength{10};

[[nodiscard]] EsdfGrid3D grid() {
  return EsdfGrid3D{.width = kLength,
                    .height = 3,
                    .resolution_m = 1.0F,
                    .origin_x_m = 0.0F,
                    .origin_y_m = 0.0F,
                    .depth = 3,
                    .origin_z_m = 0.0F,
                    .outside_is_unknown = true};
}

[[nodiscard]] std::vector<float> esdf() {
  std::vector<float> field(static_cast<std::size_t>(kLength) * 3U * 3U, 5.0F);
  for (int z = 0; z < 3; ++z) {
    for (int y = 0; y < 3; ++y) {
      for (int x = kKnownLength; x < kLength; ++x) {
        field[(static_cast<std::size_t>(z) * 3U + static_cast<std::size_t>(y)) *
                  static_cast<std::size_t>(kLength) +
              static_cast<std::size_t>(x)] = kUnknownEsdfDistanceM;
      }
    }
  }
  return field;
}

[[nodiscard]] SweptFootprintConfig pointFootprint() {
  SweptFootprintConfig footprint;
  footprint.radius_m = 0.0;
  footprint.lower_extent_m = 0.0;
  footprint.upper_extent_m = 0.0;
  footprint.perimeter_samples = 0U;
  footprint.radial_rings = 0U;
  footprint.axial_samples = 0U;
  footprint.sweep_step_m = 0.5;
  return footprint;
}

[[nodiscard]] FiniteMotionHorizon3D horizonAlongX(const float first_x,
                                                  const std::size_t states) {
  FiniteMotionHorizon3D horizon;
  for (std::size_t index = 0U; index < states; ++index) {
    MotionState3D state;
    state.x = first_x + static_cast<float>(index);
    state.y = 1.5F;
    state.z = 1.5F;
    horizon.states.push_back(state);
  }
  return horizon;
}

TEST(ExecutedHorizonClearance3DTest, TheFirstUnobservedSampleEndsTheObservedRange) {
  // From x = 0.5 the motion runs one metre per state. The segment leaving
  // x = 9.5 sweeps into the unknown cells from x = 10: the observed range
  // along the motion is the nine metres travelled before that segment.
  const ExecutedHorizonClearance3D clearance = measureExecutedHorizonClearance3D(
      horizonAlongX(0.5F, 15U), 0U, grid(), esdf(), pointFootprint(), 0.1);
  ASSERT_TRUE(clearance.available);
  EXPECT_TRUE(clearance.unobserved());
  EXPECT_NEAR(clearance.distanceToUnobservedM(), 9.0, 1.0e-6);
  // Unknown space is not a clearance constraint.
  EXPECT_FALSE(clearance.constrained());
  EXPECT_TRUE(std::isfinite(clearance.minimum_clearance_m));
}

TEST(ExecutedHorizonClearance3DTest, AMotionThroughObservedSpaceHasNoFrontier) {
  const ExecutedHorizonClearance3D clearance = measureExecutedHorizonClearance3D(
      horizonAlongX(0.5F, 8U), 0U, grid(), esdf(), pointFootprint(), 0.1);
  ASSERT_TRUE(clearance.available);
  EXPECT_FALSE(clearance.unobserved());
  EXPECT_TRUE(std::isinf(clearance.distanceToUnobservedM()));
}

TEST(ExecutedHorizonClearance3DTest, TheRangeIsMeasuredFromTheRemainingMotion) {
  // Execution has advanced four states: the observed range is what remains.
  const ExecutedHorizonClearance3D clearance = measureExecutedHorizonClearance3D(
      horizonAlongX(0.5F, 15U), 4U, grid(), esdf(), pointFootprint(), 0.1);
  ASSERT_TRUE(clearance.available);
  EXPECT_NEAR(clearance.distanceToUnobservedM(), 5.0, 1.0e-6);
}

[[nodiscard]] std::vector<RouteSample3D> routeAlongX(const double first_x,
                                                     const std::size_t samples) {
  std::vector<RouteSample3D> route;
  for (std::size_t index = 0U; index < samples; ++index) {
    route.push_back(RouteSample3D{
        .position = {first_x + static_cast<double>(index), 1.5, 1.5},
        .tangent = {1.0, 0.0, 0.0},
        .station_m = static_cast<double>(index),
    });
  }
  return route;
}

TEST(ExecutedHorizonClearance3DTest, TheRouteAheadReportsTheFrontierBeyondTheHorizon) {
  // The route runs from x = 0.5 one metre per sample; the vehicle projects at
  // station 2. The segment leaving x = 9.5 (station 9) sweeps into the unknown
  // cells from x = 10: the observed range along the route is seven metres.
  const std::optional<double> range_m = measureRouteObservedRange3D(
      routeAlongX(0.5, 20U), 2.0, 30.0, grid(), esdf(), pointFootprint());
  ASSERT_TRUE(range_m.has_value());
  EXPECT_NEAR(*range_m, 7.0, 1.0e-6);
}

TEST(ExecutedHorizonClearance3DTest, TheRouteIsProbedNoFartherThanTheLookahead) {
  EXPECT_FALSE(measureRouteObservedRange3D(routeAlongX(0.5, 20U), 2.0, 5.0, grid(),
                                           esdf(), pointFootprint())
                   .has_value());
  EXPECT_TRUE(measureRouteObservedRange3D(routeAlongX(0.5, 20U), 2.0, 8.0, grid(),
                                          esdf(), pointFootprint())
                  .has_value());
}

TEST(ExecutedHorizonClearance3DTest, ARouteThroughObservedSpaceHasNoFrontier) {
  EXPECT_FALSE(measureRouteObservedRange3D(routeAlongX(0.5, 8U), 0.0, 30.0, grid(),
                                           esdf(), pointFootprint())
                   .has_value());
}

} // namespace
} // namespace drone_city_nav
