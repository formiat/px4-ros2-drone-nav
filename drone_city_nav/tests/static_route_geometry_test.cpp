#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/static_route_geometry.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <ranges>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(StaticRouteGeometryTest, ShortcutsOpenUnconstrainedZigzag) {
  const mppi::EsdfGrid grid{80, 80, 1.0F, 0.0F, 0.0F, 20, 0.0F};
  const std::vector<float> esdf(
      static_cast<std::size_t>(grid.width * grid.height * grid.depth),
      std::numeric_limits<float>::infinity());
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{
          {5.0, 5.0, 5.0}, {20.0, 10.0, 5.0}, {35.0, 5.0, 5.0}, {50.0, 5.0, 5.0}},
      0.5, 20.0);

  const StaticRouteGeometryResult result = optimizeStaticRouteGeometry(
      route, {}, grid, esdf,
      SweptFootprintConfig{.radius_m = 0.0, .perimeter_samples = 0U},
      StaticRouteGeometryConfig{}, RouteEnvelopeConfig{});

  ASSERT_GE(result.route.size(), 2U);
  EXPECT_GT(result.shortcuts_applied, 0U);
  EXPECT_LT(result.route.back().station_m, route.back().station_m);
}

TEST(StaticRouteGeometryTest, PreservesConstrainedChannelGeometry) {
  const mppi::EsdfGrid grid{80, 80, 1.0F, 0.0F, 0.0F, 20, 0.0F};
  const std::vector<float> esdf(
      static_cast<std::size_t>(grid.width * grid.height * grid.depth),
      std::numeric_limits<float>::infinity());
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{
          {5.0, 5.0, 5.0}, {15.0, 5.0, 5.0}, {15.0, 15.0, 5.0}, {25.0, 15.0, 5.0}},
      0.5, 20.0);
  const std::vector<SelectedChannelTraversal> traversals{
      SelectedChannelTraversal{.channel_id = "channel",
                               .begin_station_m = 10.0,
                               .end_station_m = 20.0,
                               .min_z_m = 2.0,
                               .max_z_m = 8.0,
                               .minimum_clearance_m = 3.0,
                               .speed_limit_mps = 10.0}};
  const std::vector<ConstrainedRouteSpan> spans =
      makeConstrainedRouteSpans(route, traversals, 2U, RouteEnvelopeConfig{});

  const StaticRouteGeometryResult result = optimizeStaticRouteGeometry(
      route, spans, grid, esdf,
      SweptFootprintConfig{.radius_m = 0.0, .perimeter_samples = 0U},
      StaticRouteGeometryConfig{}, RouteEnvelopeConfig{});

  ASSERT_EQ(result.constrained_spans.size(), 1U);
  EXPECT_EQ(result.constrained_spans.front().channel_id, "channel");
  EXPECT_TRUE(std::ranges::any_of(result.route, [](const RouteSample3D& sample) {
    return distance3D(sample.position, Point3{15.0, 5.0, 5.0}) < 0.25;
  }));
}

TEST(StaticRouteGeometryTest, ParallelValidationPreservesDeterministicGeometry) {
  const mppi::EsdfGrid grid{100, 100, 1.0F, 0.0F, 0.0F, 20, 0.0F};
  const std::vector<float> esdf(
      static_cast<std::size_t>(grid.width * grid.height * grid.depth),
      std::numeric_limits<float>::infinity());
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{5.0, 5.0, 5.0},
                                        {20.0, 10.0, 5.0},
                                        {35.0, 5.0, 5.0},
                                        {50.0, 15.0, 5.0},
                                        {65.0, 5.0, 5.0}},
                    0.5, 20.0);
  const SweptFootprintConfig footprint{.radius_m = 0.0, .perimeter_samples = 0U};
  const StaticRouteGeometryConfig geometry_config{};
  const RouteEnvelopeConfig envelope_config{};

  const StaticRouteGeometryResult serial = optimizeStaticRouteGeometry(
      route, {}, grid, esdf, footprint, geometry_config, envelope_config);
  BoundedWorkerPool worker_pool{4U};
  const StaticRouteGeometryResult parallel = optimizeStaticRouteGeometry(
      route, {}, grid, esdf, footprint, geometry_config, envelope_config, &worker_pool);

  EXPECT_EQ(routeFingerprint(parallel.route), routeFingerprint(serial.route));
  EXPECT_EQ(parallel.shortcuts_applied, serial.shortcuts_applied);
  EXPECT_EQ(parallel.corners_smoothed, serial.corners_smoothed);
  EXPECT_GT(parallel.parallel_shortcut_candidates, 0U);
  EXPECT_LE(parallel.parallel_shortcut_candidates, parallel.shortcut_candidates);
  EXPECT_LE(parallel.parallel_corner_candidates, parallel.corner_candidates);
}

} // namespace
} // namespace drone_city_nav
