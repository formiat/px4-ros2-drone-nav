#include "drone_city_nav/compiled_trajectory_views_3d.hpp"
#include "drone_city_nav/execution_route_certification_3d.hpp"
#include "drone_city_nav/route_decoration_compiler_3d.hpp"
#include "drone_city_nav/trajectory_compiler_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <type_traits>

namespace drone_city_nav {
namespace {

[[nodiscard]] VehicleState3D testInitialState(const Point3 position = {},
                                              const Vec3 velocity = {}) {
  return VehicleState3D{
      .identity =
          VehicleStateIdentity3D{
              .revision = 7U,
              .source_timestamp_us = 11U,
              .receive_stamp_ns = 13,
          },
      .position = position,
      .velocity = velocity,
      .yaw_rad = 0.25,
      .yaw_rate_radps = -0.1,
  };
}

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

[[nodiscard]] TrackingErrorTubeWorld3D
trackingWorld(const ObservedOccupancyGrid3D& occupancy) {
  return TrackingErrorTubeWorld3D{
      .observed_occupancy = &occupancy,
      .occupied_content_fingerprint = occupancy.occupiedSnapshot().contentFingerprint(),
  };
}

[[nodiscard]] TrajectoryCompilationResult3D
compileUnconstrained(std::vector<RouteSample3D> route,
                     const VehicleState3D& initial_state,
                     const TrackingErrorTubeWorld3D tracking_world = {},
                     const RouteEndpointSemantics3D endpoint_semantics =
                         RouteEndpointSemantics3D::kContinuation,
                     const TrajectoryCompilerConfig3D& config = {}) {
  const std::uint64_t materialized_fingerprint = routeFingerprint(route);
  return TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
      .exact_initial_state = initial_state,
      .route_generation = 1U,
      .route = std::move(route),
      .constrained_spans = {},
      .endpoint_semantics = endpoint_semantics,
      .materialized_route_fingerprint = materialized_fingerprint,
      .tracking_world = tracking_world,
      .config = config,
  });
}

static_assert(!std::is_default_constructible_v<CompiledTrajectory3D>);
static_assert(!std::is_copy_constructible_v<CompiledTrajectory3D>);
static_assert(!std::is_copy_assignable_v<CompiledTrajectory3D>);
static_assert(!std::is_move_constructible_v<CompiledTrajectory3D>);
static_assert(!std::is_move_assignable_v<CompiledTrajectory3D>);
static_assert(!std::is_default_constructible_v<RouteDecorations3D>);
static_assert(!std::is_copy_constructible_v<RouteDecorations3D>);
static_assert(!std::is_copy_assignable_v<RouteDecorations3D>);
static_assert(!std::is_move_constructible_v<RouteDecorations3D>);
static_assert(!std::is_move_assignable_v<RouteDecorations3D>);

TEST(TrajectoryCompiler3DTest, SealsHardCornerAndExactInitialState) {
  std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {5.0, 0.0, 5.0}, {5.0, 5.0, 5.0}}, 0.5, 4.0);
  const std::uint64_t materialized_fingerprint = routeFingerprint(route);
  const VehicleState3D initial_state =
      testInitialState(route.front().position, Vec3{2.0, 0.0, 0.0});

  const TrajectoryCompilationResult3D compilation =
      compileUnconstrained(route, initial_state);

  ASSERT_TRUE(compilation.compiled())
      << compiledTrajectoryFailureReason3DName(compilation.validation.reason);
  ASSERT_NE(compilation.trajectory, nullptr);
  EXPECT_EQ(compilation.trajectory->exact_initial_state, initial_state);
  EXPECT_EQ(compilation.stop_turn_count, 1U);
  const auto corner = std::ranges::find_if(
      *compilation.trajectory->route, [](const RouteSample3D& sample) {
        return distance3D(sample.position, Point3{5.0, 0.0, 5.0}) < 1.0e-9;
      });
  ASSERT_NE(corner, compilation.trajectory->route->end());
  EXPECT_EQ(corner->transition, RouteKinematicTransition3D::kStopAndTurn);
  EXPECT_DOUBLE_EQ(corner->reference_speed_mps, 0.0);
  EXPECT_GT(corner->tangent.y, 0.99);

  const ActivatedRouteIdentity3D identity{
      .generation = 1U,
      .proposal =
          MaterializedRouteProposal3D{
              .objective = StaticRouteObjective{.continuous_tracking = true},
              .route_fingerprint = materialized_fingerprint,
              .route_sample_count = compilation.trajectory->route->size(),
              .activation_eligible = true,
          },
  };
  EXPECT_TRUE(compiledTrajectoryValid3D(*compilation.trajectory, identity));
}

TEST(TrajectoryCompiler3DTest, RejectsMissingExactInitialState) {
  std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {5.0, 0.0, 5.0}}, 0.5, 4.0);

  const TrajectoryCompilationResult3D compilation =
      compileUnconstrained(std::move(route), VehicleState3D{});

  EXPECT_EQ(compilation.validation.reason,
            CompiledTrajectoryFailureReason3D::kInvalidInitialState);
  EXPECT_EQ(compilation.trajectory, nullptr);
}

TEST(TrajectoryCompiler3DTest, RejectsSpanWithoutCompleteBoundaryEnvelope) {
  std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {5.0, 0.0, 5.0}}, 0.5, 4.0);
  const std::uint64_t materialized_fingerprint = routeFingerprint(route);
  const ConstrainedRouteSpan incomplete{
      .passage_traversal_id = PassageTraversalId{"passage"},
      .route_generation = 1U,
      .direction_sign = 1,
      .begin_station_m = 1.0,
      .end_station_m = 4.0,
      .envelope = {RouteEnvelopeSample{.station_m = 2.0}},
      .segment_spans = {},
  };

  const TrajectoryCompilationResult3D compilation =
      TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
          .exact_initial_state = testInitialState(route.front().position),
          .route_generation = 1U,
          .route = route,
          .constrained_spans = {incomplete},
          .materialized_route_fingerprint = materialized_fingerprint,
      });

  EXPECT_EQ(compilation.validation.reason,
            CompiledTrajectoryFailureReason3D::kInvalidConstrainedSpans);
  EXPECT_EQ(compilation.trajectory, nullptr);
}

TEST(TrajectoryCompiler3DTest, DistinguishesInvalidTrackingWorld) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 2.0}, {5.0, 0.0, 2.0}}, 0.5, 5.0);
  ObservedOccupancyGrid3D occupancy{observedBounds()};
  TrackingErrorTubeWorld3D invalid_world = trackingWorld(occupancy);
  ++invalid_world.occupied_content_fingerprint;

  const TrajectoryCompilationResult3D compilation = compileUnconstrained(
      route, testInitialState(route.front().position), invalid_world);

  EXPECT_EQ(compilation.validation.reason,
            CompiledTrajectoryFailureReason3D::kInvalidTrackingErrorTubeWorld);
  EXPECT_EQ(compilation.trajectory, nullptr);
}

TEST(TrajectoryCompiler3DTest, ExactInitialVelocityChangesTheSingleSealedTimeProfile) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 2.0}, {40.0, 0.0, 2.0}}, 0.5, 8.0);

  const TrajectoryCompilationResult3D from_rest =
      compileUnconstrained(route, testInitialState(route.front().position, Vec3{}));
  VehicleState3D moving_state =
      testInitialState(route.front().position, Vec3{5.0, 0.0, 0.0});
  moving_state.identity.revision = 8U;
  const TrajectoryCompilationResult3D already_moving =
      compileUnconstrained(route, moving_state);

  ASSERT_TRUE(from_rest.compiled());
  ASSERT_TRUE(already_moving.compiled());
  EXPECT_DOUBLE_EQ(from_rest.trajectory->route->front().reference_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(already_moving.trajectory->route->front().reference_speed_mps, 5.0);
  EXPECT_LT(already_moving.trajectory->time_profile.travel_time_s,
            from_rest.trajectory->time_profile.travel_time_s);
  EXPECT_EQ(already_moving.trajectory->physical_route_fingerprint,
            from_rest.trajectory->physical_route_fingerprint);
  EXPECT_NE(already_moving.trajectory->compiled_trajectory_revision,
            from_rest.trajectory->compiled_trajectory_revision);
}

TEST(TrajectoryCompiler3DTest,
     PassageMetadataHasAnIndependentImmutableDecorationRevision) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 2.0}, {10.0, 0.0, 2.0}}, 0.5, 5.0);
  const TrajectoryCompilationResult3D trajectory =
      compileUnconstrained(route, testInitialState(route.front().position));
  ASSERT_TRUE(trajectory.compiled());
  const std::uint64_t trajectory_revision =
      trajectory.trajectory->compiled_trajectory_revision;

  PassageVolumeConfig first_config;
  PassageVolumeConfig second_config = first_config;
  second_config.minimum_wall_clearance_m += 0.1;
  const RouteDecorationCompilationResult3D first =
      RouteDecorationCompiler3D::compile(RouteDecorationCompilerInput3D{
          .trajectory = trajectory.trajectory,
          .route_generation = 1U,
          .passage_volumes = {},
          .cooperative_passage_assignments = {},
          .selected_passage_traversal_ids = {},
          .passage_volume_config = first_config,
      });
  const RouteDecorationCompilationResult3D second =
      RouteDecorationCompiler3D::compile(RouteDecorationCompilerInput3D{
          .trajectory = trajectory.trajectory,
          .route_generation = 1U,
          .passage_volumes = {},
          .cooperative_passage_assignments = {},
          .selected_passage_traversal_ids = {},
          .passage_volume_config = second_config,
      });

  ASSERT_TRUE(first.compiled())
      << routeDecorationFailureReason3DName(first.validation.reason);
  ASSERT_TRUE(second.compiled())
      << routeDecorationFailureReason3DName(second.validation.reason);
  EXPECT_EQ(trajectory.trajectory->compiled_trajectory_revision, trajectory_revision);
  EXPECT_NE(first.decorations->route_decorations_revision,
            second.decorations->route_decorations_revision);
}

TEST(TrajectoryCompiler3DTest,
     RouteDecorationsAreBoundToTheExactCompiledGeometryRevision) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 2.0}, {10.0, 0.0, 2.0}}, 0.5, 5.0);
  const TrajectoryCompilationResult3D first =
      compileUnconstrained(route, testInitialState(route.front().position));
  VehicleState3D newer_state = testInitialState(route.front().position);
  ++newer_state.identity.revision;
  const TrajectoryCompilationResult3D second = compileUnconstrained(route, newer_state);
  ASSERT_TRUE(first.compiled());
  ASSERT_TRUE(second.compiled());
  ASSERT_EQ(first.trajectory->physical_route_fingerprint,
            second.trajectory->physical_route_fingerprint);
  ASSERT_NE(first.trajectory->compiled_trajectory_revision,
            second.trajectory->compiled_trajectory_revision);

  const RouteDecorationCompilationResult3D decorations =
      RouteDecorationCompiler3D::compile(RouteDecorationCompilerInput3D{
          .trajectory = first.trajectory,
          .route_generation = 1U,
          .passage_volumes = {},
          .cooperative_passage_assignments = {},
          .selected_passage_traversal_ids = {},
      });
  ASSERT_TRUE(decorations.compiled());
  EXPECT_TRUE(routeDecorationsValid3D(*decorations.decorations, *first.trajectory, 1U));
  EXPECT_FALSE(
      routeDecorationsValid3D(*decorations.decorations, *second.trajectory, 1U));
}

TEST(TrajectoryCompiler3DTest, SealsSampleAlignedRemainingTimeAuthority) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 2.0}, {40.0, 0.0, 2.0}}, 0.5, 8.0);
  const TrajectoryCompilationResult3D compilation =
      compileUnconstrained(route, testInitialState(route.front().position));

  ASSERT_TRUE(compilation.compiled());
  ASSERT_NE(compilation.trajectory, nullptr);
  const CompiledTrajectory3D& trajectory = *compilation.trajectory;
  ASSERT_EQ(trajectory.time_profile.arrival_times_s.size(), trajectory.route->size());
  ASSERT_EQ(trajectory.time_profile.departure_times_s.size(), trajectory.route->size());
  const std::optional<double> complete = remainingCompiledTrajectoryTime3D(
      trajectory, trajectory.route->front().station_m);
  const std::optional<double> partial =
      remainingCompiledTrajectoryTime3D(trajectory, 20.0);
  const std::optional<double> finished =
      remainingCompiledTrajectoryTime3D(trajectory, trajectory.route->back().station_m);
  ASSERT_TRUE(complete.has_value());
  ASSERT_TRUE(partial.has_value());
  ASSERT_TRUE(finished.has_value());
  const double complete_time_s =
      complete.value(); // NOLINT(bugprone-unchecked-optional-access)
  const double partial_time_s =
      partial.value(); // NOLINT(bugprone-unchecked-optional-access)
  const double finished_time_s =
      finished.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_DOUBLE_EQ(complete_time_s, trajectory.time_profile.travel_time_s);
  EXPECT_GT(partial_time_s, 0.0);
  EXPECT_LT(partial_time_s, complete_time_s);
  EXPECT_DOUBLE_EQ(finished_time_s, 0.0);
}

TEST(TrajectoryCompiler3DTest, RecompilationPublishesANewImmutableWorldBinding) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 2.0}, {9.0, 0.0, 2.0}}, 0.5, 5.0);
  ObservedOccupancyGrid3D first_world{observedBounds()};
  const VehicleState3D initial_state = testInitialState(route.front().position);
  const TrajectoryCompilationResult3D first =
      compileUnconstrained(route, initial_state, trackingWorld(first_world));
  ASSERT_TRUE(first.compiled());

  ObservedOccupancyGrid3D second_world = first_world;
  const GridBounds3D& bounds = second_world.bounds();
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int x = 0; x < bounds.width_cells; ++x) {
      ASSERT_TRUE(second_world.setState({x, 7, z}, ObservedVoxelState::kOccupied));
      ASSERT_TRUE(second_world.setState({x, 32, z}, ObservedVoxelState::kOccupied));
    }
  }
  const TrajectoryCompilationResult3D second =
      compileUnconstrained(route, initial_state, trackingWorld(second_world));

  ASSERT_TRUE(second.compiled());
  EXPECT_TRUE(trackingErrorTubeProfile3DMatchesWorld(
      *first.trajectory->route, *first.trajectory->tracking_error_tube,
      trackingWorld(first_world)));
  EXPECT_TRUE(trackingErrorTubeProfile3DMatchesWorld(
      *second.trajectory->route, *second.trajectory->tracking_error_tube,
      trackingWorld(second_world)));
  EXPECT_NE(first.trajectory->compiled_trajectory_revision,
            second.trajectory->compiled_trajectory_revision);
}

} // namespace
} // namespace drone_city_nav
