#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/route_compiler_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>

namespace drone_city_nav {
namespace {

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
          .route_purpose = Lattice3DRoutePurpose::kMissionTransit,
          .observation_frontier = std::nullopt,
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
          .route_purpose = Lattice3DRoutePurpose::kMissionTransit,
          .observation_frontier = std::nullopt,
          .endpoint_semantics = RouteEndpointSemantics3D::kContinuation,
          .materialized_route_fingerprint = routeFingerprint(route),
          .config = RouteCompilerConfig3D{},
      });

  EXPECT_EQ(compilation.validation.reason,
            ExecutionRouteGeometryFailureReason3D::kInvalidConstrainedSpans);
  EXPECT_EQ(compilation.geometry, nullptr);
}

} // namespace
} // namespace drone_city_nav
