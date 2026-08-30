#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/static_route_geometry.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <ranges>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr StaticRouteGeometryConfig enabledGeometryConfig() {
  StaticRouteGeometryConfig config;
  config.enabled = true;
  return config;
}

[[nodiscard]] OccupiedCollisionWorld3D
staticCollisionWorld(const OccupancyGrid3D& occupancy,
                     const SweptFootprintConfig& footprint) noexcept {
  return OccupiedCollisionWorld3D{
      .observed_occupancy = nullptr,
      .static_occupancy = &occupancy,
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = footprint,
      .flight_envelope = std::nullopt,
  };
}

[[nodiscard]] OccupiedCollisionWorld3D
observedCollisionWorld(const ObservedOccupancyGrid3D& occupancy,
                       const SweptFootprintConfig& footprint) noexcept {
  return OccupiedCollisionWorld3D{
      .observed_occupancy = &occupancy,
      .static_occupancy = nullptr,
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = footprint,
      .flight_envelope = std::nullopt,
  };
}

TEST(StaticRouteGeometryTest, DefaultPolicyPreservesPlannerGeometry) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{
          {5.0, 5.0, 5.0}, {20.0, 10.0, 5.0}, {35.0, 5.0, 5.0}, {50.0, 5.0, 5.0}},
      0.5, 20.0);

  const StaticRouteGeometryResult result =
      optimizeStaticRouteGeometry(route, {}, OccupiedCollisionWorld3D{},
                                  StaticRouteGeometryConfig{}, RouteEnvelopeConfig{});

  EXPECT_EQ(routeFingerprint(result.route), routeFingerprint(route));
  EXPECT_EQ(result.shortcuts_applied, 0U);
  EXPECT_EQ(result.corners_smoothed, 0U);
}

TEST(StaticRouteGeometryTest, ShortcutsOpenUnconstrainedZigzag) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{
          {5.0, 5.0, 5.0}, {20.0, 10.0, 5.0}, {35.0, 5.0, 5.0}, {50.0, 5.0, 5.0}},
      0.5, 20.0);
  const SweptFootprintConfig footprint{.radius_m = 0.0, .perimeter_samples = 0U};
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 80, 80, 20}};

  const StaticRouteGeometryResult result =
      optimizeStaticRouteGeometry(route, {}, staticCollisionWorld(occupancy, footprint),
                                  enabledGeometryConfig(), RouteEnvelopeConfig{});

  ASSERT_GE(result.route.size(), 2U);
  EXPECT_GT(result.shortcuts_applied, 0U);
  EXPECT_LT(result.route.back().station_m, route.back().station_m);
  EXPECT_LT(result.sparse_anchor_count, route.size());
  EXPECT_GT(result.sparse_samples_removed, 0U);
}

TEST(StaticRouteGeometryTest, SparseBatchesAvoidDenseAllPairsOnLongRoute) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{5.0, 5.0, 5.0}, {125.0, 5.0, 5.0}}, 0.5, 20.0);
  const SweptFootprintConfig footprint{.radius_m = 0.0, .perimeter_samples = 0U};
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 140, 20, 10}};

  const StaticRouteGeometryResult result =
      optimizeStaticRouteGeometry(route, {}, staticCollisionWorld(occupancy, footprint),
                                  enabledGeometryConfig(), RouteEnvelopeConfig{});

  EXPECT_EQ(result.sparse_anchor_count, 2U);
  EXPECT_EQ(result.sparse_samples_removed, route.size() - 2U);
  EXPECT_LE(result.shortcut_candidates, 5U);
  EXPECT_EQ(result.shortcut_validation_batches, result.shortcut_candidates);
  EXPECT_GE(result.shortcuts_applied, 4U);
  ASSERT_FALSE(result.route.empty());
  EXPECT_NEAR(result.route.back().position.x, 125.0, 1.0e-9);
}

TEST(StaticRouteGeometryTest, MaterializesRawSafeRightAngleAsFillet) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{
          {5.0, 5.0, 5.0}, {20.0, 5.0, 5.0}, {20.0, 20.0, 5.0}, {35.0, 20.0, 5.0}},
      0.5, 20.0);
  const SweptFootprintConfig footprint{.radius_m = 0.0, .perimeter_samples = 0U};
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 80, 80, 20}};

  const StaticRouteGeometryResult result =
      optimizeStaticRouteGeometry(route, {}, staticCollisionWorld(occupancy, footprint),
                                  enabledGeometryConfig(), RouteEnvelopeConfig{});
  const OccupiedCollisionOracle3D collision_oracle{
      staticCollisionWorld(occupancy, footprint)};

  EXPECT_GT(result.corners_smoothed, 0U);
  ASSERT_GE(result.route.size(), 2U);
  for (std::size_t index = 1U; index < result.route.size(); ++index) {
    EXPECT_TRUE(collision_oracle
                    .validateSegment(result.route[index - 1U].position,
                                     FootprintBodyAxis{}, result.route[index].position,
                                     FootprintBodyAxis{})
                    .clear());
  }
}

TEST(StaticRouteGeometryTest, CornerSmoothingDoesNotRequireShortcutOptimization) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{5.0, 5.0, 5.0}, {20.0, 5.0, 5.0}, {20.0, 20.0, 5.0}}, 0.5,
      20.0);
  StaticRouteGeometryConfig config = enabledGeometryConfig();
  config.shortcut_optimization_enabled = false;
  const SweptFootprintConfig footprint{.radius_m = 0.0, .perimeter_samples = 0U};
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 80, 80, 20}};

  const StaticRouteGeometryResult result =
      optimizeStaticRouteGeometry(route, {}, staticCollisionWorld(occupancy, footprint),
                                  config, RouteEnvelopeConfig{});

  EXPECT_EQ(result.shortcuts_applied, 0U);
  EXPECT_EQ(result.shortcut_candidates, 0U);
  EXPECT_GT(result.corners_smoothed, 0U);
}

TEST(StaticRouteGeometryTest, SparseShortcutsRemainRawFootprintSafe) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 40, 10}};
  for (int y = 0; y <= 25; ++y) {
    for (int z = 0; z < 10; ++z) {
      occupancy.setOccupied(GridIndex3D{20, y, z});
    }
  }
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{
          {5.5, 5.5, 5.5}, {15.5, 30.5, 5.5}, {25.5, 30.5, 5.5}, {35.5, 5.5, 5.5}},
      0.5, 20.0);
  const SweptFootprintConfig footprint{.radius_m = 0.0, .perimeter_samples = 0U};

  const StaticRouteGeometryResult result =
      optimizeStaticRouteGeometry(route, {}, staticCollisionWorld(occupancy, footprint),
                                  enabledGeometryConfig(), RouteEnvelopeConfig{});
  const OccupiedCollisionOracle3D collision_oracle{
      staticCollisionWorld(occupancy, footprint)};

  ASSERT_GE(result.route.size(), 2U);
  for (std::size_t index = 1U; index < result.route.size(); ++index) {
    EXPECT_TRUE(collision_oracle
                    .validateSegment(result.route[index - 1U].position,
                                     FootprintBodyAxis{}, result.route[index].position,
                                     FootprintBodyAxis{})
                    .clear());
  }
}

TEST(StaticRouteGeometryTest, FreshRawWorldRejectsShortcutAcceptedInEmptyRawWorld) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 40, 40, 10};
  ObservedOccupancyGrid3D empty_raw_world{bounds};
  ObservedOccupancyGrid3D raw_world{bounds};
  ASSERT_TRUE(
      raw_world.setState(GridIndex3D{12, 19, 5}, ObservedVoxelState::kOccupied));
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{5.5, 5.5, 5.5}, {5.5, 25.5, 5.5}, {25.5, 25.5, 5.5}}, 0.5,
      20.0);
  const SweptFootprintConfig footprint{.radius_m = 0.0,
                                       .lower_extent_m = 0.0,
                                       .upper_extent_m = 0.0,
                                       .perimeter_samples = 0U,
                                       .radial_rings = 0U,
                                       .axial_samples = 1U,
                                       .sweep_step_m = 0.25};
  StaticRouteGeometryConfig geometry_config;
  geometry_config.enabled = true;
  geometry_config.maximum_shortcut_turn_increase_rad = 10.0;
  const StaticRouteGeometryResult empty_world_geometry = optimizeStaticRouteGeometry(
      route, {}, observedCollisionWorld(empty_raw_world, footprint), geometry_config,
      RouteEnvelopeConfig{});
  const StaticRouteGeometryResult occupied_world_geometry = optimizeStaticRouteGeometry(
      route, {}, observedCollisionWorld(raw_world, footprint), geometry_config,
      RouteEnvelopeConfig{});
  const OccupiedCollisionOracle3D collision_oracle{
      observedCollisionWorld(raw_world, footprint)};

  ASSERT_GE(empty_world_geometry.route.size(), 2U);
  bool empty_world_result_collides_with_fresh_raw = false;
  for (std::size_t index = 1U; index < empty_world_geometry.route.size(); ++index) {
    empty_world_result_collides_with_fresh_raw =
        empty_world_result_collides_with_fresh_raw ||
        !collision_oracle
             .validateSegment(
                 empty_world_geometry.route[index - 1U].position, FootprintBodyAxis{},
                 empty_world_geometry.route[index].position, FootprintBodyAxis{})
             .clear();
  }
  EXPECT_TRUE(empty_world_result_collides_with_fresh_raw);
  ASSERT_GE(occupied_world_geometry.route.size(), 2U);
  for (std::size_t index = 1U; index < occupied_world_geometry.route.size(); ++index) {
    EXPECT_TRUE(collision_oracle
                    .validateSegment(occupied_world_geometry.route[index - 1U].position,
                                     FootprintBodyAxis{},
                                     occupied_world_geometry.route[index].position,
                                     FootprintBodyAxis{})
                    .clear());
  }
}

TEST(StaticRouteGeometryTest, PreservesConstrainedPassageGeometry) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{
          {5.0, 5.0, 5.0}, {15.0, 5.0, 5.0}, {15.0, 15.0, 5.0}, {25.0, 15.0, 5.0}},
      0.5, 20.0);
  const std::vector<SelectedPassageTraversal> traversals{
      SelectedPassageTraversal{.passage_traversal_id = "passage",
                               .direction_sign = 1,
                               .begin_station_m = 10.0,
                               .end_station_m = 20.0,
                               .min_z_m = 2.0,
                               .max_z_m = 8.0,
                               .width_m = 18.0,
                               .height_m = 6.0,
                               .minimum_clearance_m = 3.0,
                               .speed_limit_mps = 10.0,
                               .segment_spans = {PassageTraversalSegmentSpan{
                                   .passage_segment_id = "segment:corner",
                                   .begin_station_m = 10.0,
                                   .end_station_m = 20.0}}}};
  std::vector<ConstrainedRouteSpan> spans =
      makeConstrainedRouteSpans(route, traversals, 2U, RouteEnvelopeConfig{});
  ASSERT_EQ(spans.size(), 1U);
  for (RouteEnvelopeSample& envelope : spans.front().envelope) {
    const double ratio = (envelope.station_m - spans.front().begin_station_m) /
                         (spans.front().end_station_m - spans.front().begin_station_m);
    envelope.min_z_m = 2.0 + ratio;
    envelope.max_z_m = 8.0 + ratio;
  }
  const SweptFootprintConfig footprint{.radius_m = 0.0, .perimeter_samples = 0U};
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 80, 80, 20}};

  const StaticRouteGeometryResult result = optimizeStaticRouteGeometry(
      route, spans, staticCollisionWorld(occupancy, footprint), enabledGeometryConfig(),
      RouteEnvelopeConfig{});

  ASSERT_EQ(result.constrained_spans.size(), 1U);
  EXPECT_EQ(result.constrained_spans.front().passage_traversal_id, "passage");
  ASSERT_EQ(result.constrained_spans.front().segment_spans.size(), 1U);
  EXPECT_EQ(result.constrained_spans.front().segment_spans.front().passage_segment_id,
            "segment:corner");
  EXPECT_GT(result.constrained_spans.front().segment_spans.front().end_station_m,
            result.constrained_spans.front().segment_spans.front().begin_station_m);
  EXPECT_EQ(result.constrained_spans.front().direction_sign, 1);
  ASSERT_GT(result.constrained_spans.front().envelope.size(), 1U);
  EXPECT_LT(result.constrained_spans.front().envelope.front().min_z_m,
            result.constrained_spans.front().envelope.back().min_z_m);
  EXPECT_FALSE(std::ranges::any_of(result.route, [](const RouteSample3D& sample) {
    return distance3D(sample.position, Point3{15.0, 5.0, 5.0}) < 0.05;
  }));
}

TEST(StaticRouteGeometryTest, FreezesPrefixByRouteStationNotEuclideanDistance) {
  // The third corner returns close to the start in Euclidean space, while its
  // station is still inside the frozen prefix.
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{10.0, 10.0, 5.0},
                                        {40.0, 10.0, 5.0},
                                        {40.0, 40.0, 5.0},
                                        {10.0, 40.0, 5.0},
                                        {10.0, 70.0, 5.0}},
                    0.5, 20.0);
  const SweptFootprintConfig footprint{.radius_m = 0.0, .perimeter_samples = 0U};
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 100, 100, 20}};
  const StaticRouteGeometryResult result = optimizeStaticRouteGeometry(
      route, {}, staticCollisionWorld(occupancy, footprint),
      StaticRouteGeometryConfig{.enabled = true, .frozen_prefix_end_station_m = 90.0},
      RouteEnvelopeConfig{});

  ASSERT_FALSE(result.route.empty());
  // Geometry after the frozen boundary remains optimizable; only the prefix
  // through station 90 is immutable.
  EXPECT_EQ(result.corners_smoothed, 1U);
  const RouteSample3D frozen_corner = sampleRoute3DAtStation(route, 90.0);
  const RouteSample3D optimized_corner = sampleRoute3DAtStation(result.route, 90.0);
  EXPECT_NEAR(optimized_corner.position.x, frozen_corner.position.x, 1.0e-9);
  EXPECT_NEAR(optimized_corner.position.y, frozen_corner.position.y, 1.0e-9);
  EXPECT_NEAR(optimized_corner.position.z, frozen_corner.position.z, 1.0e-9);
}

TEST(StaticRouteGeometryTest, ParallelValidationPreservesDeterministicGeometry) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{5.0, 5.0, 5.0},
                                        {20.0, 10.0, 5.0},
                                        {35.0, 5.0, 5.0},
                                        {50.0, 15.0, 5.0},
                                        {65.0, 5.0, 5.0}},
                    0.5, 20.0);
  const SweptFootprintConfig footprint{.radius_m = 0.0, .perimeter_samples = 0U};
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 100, 100, 20}};
  const OccupiedCollisionWorld3D collision_world =
      staticCollisionWorld(occupancy, footprint);
  const StaticRouteGeometryConfig geometry_config = enabledGeometryConfig();
  const RouteEnvelopeConfig envelope_config{};

  const StaticRouteGeometryResult serial = optimizeStaticRouteGeometry(
      route, {}, collision_world, geometry_config, envelope_config);
  BoundedWorkerPool worker_pool{4U};
  const StaticRouteGeometryResult parallel = optimizeStaticRouteGeometry(
      route, {}, collision_world, geometry_config, envelope_config, &worker_pool);

  EXPECT_EQ(routeFingerprint(parallel.route), routeFingerprint(serial.route));
  EXPECT_EQ(parallel.shortcuts_applied, serial.shortcuts_applied);
  EXPECT_EQ(parallel.corners_smoothed, serial.corners_smoothed);
  EXPECT_GT(parallel.parallel_shortcut_candidates, 0U);
  EXPECT_LE(parallel.parallel_shortcut_candidates, parallel.shortcut_candidates);
  EXPECT_LE(parallel.parallel_corner_candidates, parallel.corner_candidates);
}

} // namespace
} // namespace drone_city_nav
