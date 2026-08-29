#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/route_compiler_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>

namespace drone_city_nav {
namespace {

[[nodiscard]] GridBounds3D observedBounds() {
  return GridBounds3D{
      .origin_x = -1.0,
      .origin_y = -2.0,
      .origin_z = 0.0,
      .resolution_m = 0.1,
      .width_cells = 110,
      .height_cells = 40,
      .depth_cells = 40,
  };
}

void addTrackingWalls(ObservedOccupancyGrid3D& occupancy) {
  const GridBounds3D& bounds = occupancy.bounds();
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int x = 0; x < bounds.width_cells; ++x) {
      static_cast<void>(occupancy.setState({x, 7, z}, ObservedVoxelState::kOccupied));
      static_cast<void>(occupancy.setState({x, 32, z}, ObservedVoxelState::kOccupied));
    }
  }
}

[[nodiscard]] TrackingErrorTubeWorld3D
trackingWorld(const ObservedOccupancyGrid3D& occupancy) {
  return TrackingErrorTubeWorld3D{
      .observed_occupancy = &occupancy,
      .occupied_content_fingerprint = occupancy.occupiedSnapshot().contentFingerprint(),
  };
}

TEST(RouteCompiler3DTest, CompilesHardCornerAsExplicitStopTurnGeometry) {
  std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {5.0, 0.0, 5.0}, {5.0, 5.0, 5.0}}, 0.5, 4.0);
  const std::uint64_t materialized_fingerprint = routeFingerprint(route);

  const RouteCompilationResult3D compilation =
      compileExecutionRoute3D(RouteCompilerInput3D{
          .route = route,
          .constrained_spans = {},
          .passage_volumes = {},
          .cooperative_passage_assignments = {},
          .selected_passage_traversal_ids = {},
          .passage_volume_config = PassageVolumeConfig{},
          .endpoint_semantics = RouteEndpointSemantics3D::kContinuation,
          .materialized_route_fingerprint = materialized_fingerprint,
          .config = RouteCompilerConfig3D{},
      });

  ASSERT_TRUE(compilation.compiled())
      << executionRouteGeometryFailureReasonName3D(compilation.validation.reason);
  ASSERT_NE(compilation.geometry, nullptr);
  EXPECT_EQ(compilation.stop_turn_count, 1U);
  const auto corner = std::ranges::find_if(
      *compilation.geometry->route, [](const RouteSample3D& sample) {
        return distance3D(sample.position, Point3{5.0, 0.0, 5.0}) < 1.0e-9;
      });
  ASSERT_NE(corner, compilation.geometry->route->end());
  EXPECT_EQ(corner->transition, RouteKinematicTransition3D::kStopAndTurn);
  const std::size_t corner_index = static_cast<std::size_t>(
      std::distance(compilation.geometry->route->begin(), corner));
  EXPECT_FLOAT_EQ((*compilation.geometry->mppi_route)[corner_index].reference_speed_mps,
                  0.0F);
  EXPECT_GT(corner->tangent.y, 0.99);

  const ActivatedRouteIdentity3D identity{
      .generation = 1U,
      .proposal =
          MaterializedRouteProposal3D{
              .objective = StaticRouteObjective{.continuous_tracking = true},
              .route_fingerprint = materialized_fingerprint,
              .route_sample_count = compilation.geometry->route->size(),
              .activation_eligible = true,
          },
  };
  EXPECT_TRUE(executionRouteGeometryValid3D(*compilation.geometry, identity));
}

TEST(RouteCompiler3DTest, RejectsSpanWithoutCompleteBoundaryEnvelope) {
  std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {5.0, 0.0, 5.0}}, 0.5, 4.0);
  ConstrainedRouteSpan incomplete{
      .passage_traversal_id = PassageTraversalId{"passage"},
      .route_generation = 1U,
      .direction_sign = 1,
      .begin_station_m = 1.0,
      .end_station_m = 4.0,
      .envelope = {RouteEnvelopeSample{.station_m = 2.0}},
      .segment_spans = {},
  };

  const RouteCompilationResult3D compilation =
      compileExecutionRoute3D(RouteCompilerInput3D{
          .route = route,
          .constrained_spans = {incomplete},
          .passage_volumes = {PassageVolume{}},
          .cooperative_passage_assignments = {},
          .selected_passage_traversal_ids = {PassageTraversalId{"passage"}},
          .passage_volume_config = PassageVolumeConfig{},
          .endpoint_semantics = RouteEndpointSemantics3D::kContinuation,
          .materialized_route_fingerprint = routeFingerprint(route),
          .config = RouteCompilerConfig3D{},
      });

  EXPECT_EQ(compilation.validation.reason,
            ExecutionRouteGeometryFailureReason3D::kInvalidConstrainedSpans);
  EXPECT_EQ(compilation.geometry, nullptr);
}

TEST(RouteCompiler3DTest, DistinguishesInvalidTrackingWorldFromTimeProfileFailure) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 2.0}, {5.0, 0.0, 2.0}}, 0.5, 5.0);
  ObservedOccupancyGrid3D occupancy{observedBounds()};

  const RouteCompilationResult3D compilation =
      compileExecutionRoute3D(RouteCompilerInput3D{
          .route = route,
          .constrained_spans = {},
          .passage_volumes = {},
          .cooperative_passage_assignments = {},
          .selected_passage_traversal_ids = {},
          .passage_volume_config = PassageVolumeConfig{},
          .endpoint_semantics = RouteEndpointSemantics3D::kContinuation,
          .materialized_route_fingerprint = routeFingerprint(route),
          .tracking_world =
              TrackingErrorTubeWorld3D{
                  .observed_occupancy = &occupancy,
                  .occupied_content_fingerprint =
                      occupancy.occupiedSnapshot().contentFingerprint() + 1U,
              },
          .config = RouteCompilerConfig3D{},
      });

  EXPECT_EQ(compilation.validation.reason,
            ExecutionRouteGeometryFailureReason3D::kInvalidTrackingErrorTube);
  EXPECT_EQ(compilation.geometry, nullptr);
}

TEST(RouteCompiler3DTest, RecompilesAnUnchangedSpatialRouteForNewTrackingEvidence) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 2.0}, {5.0, 0.0, 2.0}, {9.0, 0.0, 2.0}}, 0.5, 5.0);
  ObservedOccupancyGrid3D initial_occupancy{observedBounds()};
  RouteCompilerConfig3D config;
  config.unconstrained_speed_mps = 5.0;
  config.speed_policy.cruise_speed_mps = 5.0;
  config.speed_policy.absolute_speed_limit_mps = 5.0;
  config.tracking_error_tube.response_time_s = 0.15;
  const RouteCompilationResult3D initial = compileExecutionRoute3D(RouteCompilerInput3D{
      .route = route,
      .constrained_spans = {},
      .passage_volumes = {},
      .cooperative_passage_assignments = {},
      .selected_passage_traversal_ids = {},
      .passage_volume_config = PassageVolumeConfig{},
      .endpoint_semantics = RouteEndpointSemantics3D::kContinuation,
      .materialized_route_fingerprint = routeFingerprint(route),
      .tracking_world = trackingWorld(initial_occupancy),
      .config = config,
  });
  ASSERT_TRUE(initial.compiled());

  ObservedOccupancyGrid3D changed_occupancy = initial_occupancy;
  addTrackingWalls(changed_occupancy);
  ASSERT_FALSE(trackingErrorTubeProfile3DMatchesWorld(
      *initial.geometry->route, *initial.geometry->tracking_error_tube,
      trackingWorld(changed_occupancy)));

  const RouteCompilationResult3D rebound = recompileExecutionRouteDynamics3D(
      *initial.geometry, RouteEndpointSemantics3D::kContinuation,
      trackingWorld(changed_occupancy), config);

  ASSERT_TRUE(rebound.compiled());
  EXPECT_EQ(rebound.geometry->physical_route_fingerprint,
            initial.geometry->physical_route_fingerprint);
  EXPECT_EQ(rebound.geometry->materialized_route_fingerprint,
            initial.geometry->materialized_route_fingerprint);
  EXPECT_TRUE(trackingErrorTubeProfile3DMatchesWorld(
      *rebound.geometry->route, *rebound.geometry->tracking_error_tube,
      trackingWorld(changed_occupancy)));
  EXPECT_LT(rebound.geometry->tracking_error_tube->minimum_speed_limit_mps,
            initial.geometry->tracking_error_tube->minimum_speed_limit_mps);
  EXPECT_GT(rebound.geometry->tracking_error_tube->minimum_speed_limit_mps, 0.0);
}

} // namespace
} // namespace drone_city_nav
