#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "route_trajectory_compiler_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] GridBounds3D testBounds() {
  return GridBounds3D{
      .origin_x = 0.0,
      .origin_y = 0.0,
      .origin_z = 0.0,
      .resolution_m = 0.5,
      .width_cells = 32,
      .height_cells = 16,
      .depth_cells = 16,
  };
}

[[nodiscard]] VehicleState3D initialState(const Point3& position) {
  return VehicleState3D{
      .identity =
          VehicleStateIdentity3D{
              .revision = 5U,
              .source_timestamp_us = 7U,
              .receive_stamp_ns = 11,
          },
      .position = position,
      .velocity = Vec3{1.0, 0.0, 0.0},
      .yaw_rad = 0.0,
      .yaw_rate_radps = 0.0,
  };
}

[[nodiscard]] MaterializedRoute3D
materializedRoute(std::shared_ptr<const WorldSnapshot3D> world) {
  auto route = std::make_shared<const std::vector<RouteSample3D>>(
      sampleRoute3D(std::vector<Point3>{{1.0, 2.0, 2.0}, {12.0, 2.0, 2.0}}, 0.5, 5.0));
  return MaterializedRoute3D{
      .world = std::move(world),
      .route = route,
      .constrained_spans = std::make_shared<const std::vector<ConstrainedRouteSpan>>(),
      .passage_volumes = std::make_shared<const std::vector<PassageVolume>>(),
      .cooperative_passage_assignments =
          std::make_shared<const std::vector<CooperativePassageAssignment>>(),
      .selected_passage_traversal_ids =
          std::make_shared<const std::vector<PassageTraversalId>>(),
      .candidate_generation = 3U,
      .fingerprint = routeFingerprint(*route),
      .reaches_mission_goal = true,
      .planner_executable = true,
  };
}

[[nodiscard]] std::shared_ptr<const WorldSnapshot3D> staticWorld() {
  auto world = std::make_shared<WorldSnapshot3D>();
  world->static_occupancy = std::make_shared<const OccupancyGrid3D>(testBounds(), 1U);
  return world;
}

struct ObservedFixture3D {
  std::shared_ptr<const WorldSnapshot3D> world;
  std::shared_ptr<const VersionedObservedRawWorld3D> owner;
};

[[nodiscard]] ObservedFixture3D observedWorld() {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(testBounds());
  const std::shared_ptr<const VersionedObservedRawWorld3D> owner =
      VersionedObservedRawWorld3D::captureOwned(
          RawMapVersion{
              .producer_instance_id = 17U,
              .base_snapshot_revision = 4U,
              .revision = 4U,
          },
          occupancy, std::nullopt, std::nullopt);
  auto world = std::make_shared<WorldSnapshot3D>();
  world->observed_occupancy = occupancy;
  world->observed_raw_world_owner = owner;
  return {.world = std::move(world), .owner = owner};
}

TEST(RouteTrajectoryCompiler3DTest, SealsStaticRouteWithExactInitialState) {
  RouteTrajectoryCompiler3D compiler{RouteTrajectoryCompilerConfig3D{}};
  MaterializedRoute3D materialized = materializedRoute(staticWorld());
  const VehicleState3D initial = initialState(materialized.route->front().position);

  const RouteTrajectoryCompilationResult3D result =
      compiler.compile(RouteTrajectoryCompilationRequest3D{
          .materialized = materialized,
          .exact_initial_state = initial,
          .endpoint_semantics = RouteEndpointSemantics3D::kMissionStop,
          .observed_raw_world = nullptr,
      });

  ASSERT_TRUE(result.compiled())
      << compiledTrajectoryFailureReason3DName(result.validation.reason);
  ASSERT_NE(result.trajectory, nullptr);
  ASSERT_NE(result.decorations, nullptr);
  EXPECT_TRUE(result.decorations->passage_volumes->empty());
  EXPECT_TRUE(routeDecorationsValid3D(*result.decorations, *result.trajectory,
                                      materialized.candidate_generation));
  EXPECT_EQ(result.trajectory->exact_initial_state, initial);
  EXPECT_EQ(result.trajectory->endpoint_semantics,
            RouteEndpointSemantics3D::kMissionStop);
  EXPECT_EQ(result.trajectory->materialized_route_fingerprint,
            materialized.fingerprint);
}

TEST(RouteTrajectoryCompiler3DTest, BindsTrackingTubeToExactObservedOwner) {
  const ObservedFixture3D observed = observedWorld();
  ASSERT_NE(observed.owner, nullptr);
  RouteTrajectoryCompiler3D compiler{RouteTrajectoryCompilerConfig3D{}};
  MaterializedRoute3D materialized = materializedRoute(observed.world);

  const RouteTrajectoryCompilationResult3D result =
      compiler.compile(RouteTrajectoryCompilationRequest3D{
          .materialized = materialized,
          .exact_initial_state = initialState(materialized.route->front().position),
          .endpoint_semantics = RouteEndpointSemantics3D::kContinuation,
          .observed_raw_world = observed.owner,
      });

  ASSERT_TRUE(result.compiled())
      << compiledTrajectoryFailureReason3DName(result.validation.reason);
  ASSERT_NE(result.trajectory, nullptr);
  ASSERT_NE(result.decorations, nullptr);
  EXPECT_TRUE(trackingErrorTubeProfile3DMatchesWorld(
      *result.trajectory->route, *result.trajectory->tracking_error_tube,
      TrackingErrorTubeWorld3D{
          .observed_occupancy = &observed.owner->occupancy(),
          .occupied_content_fingerprint = observed.owner->occupiedContentFingerprint(),
      }));
}

TEST(RouteTrajectoryCompiler3DTest, RejectsObservedRouteWithoutOwnedRawWorld) {
  const ObservedFixture3D observed = observedWorld();
  RouteTrajectoryCompiler3D compiler{RouteTrajectoryCompilerConfig3D{}};
  MaterializedRoute3D materialized = materializedRoute(observed.world);

  const RouteTrajectoryCompilationResult3D result =
      compiler.compile(RouteTrajectoryCompilationRequest3D{
          .materialized = materialized,
          .exact_initial_state = initialState(materialized.route->front().position),
          .endpoint_semantics = RouteEndpointSemantics3D::kContinuation,
          .observed_raw_world = nullptr,
      });

  EXPECT_EQ(result.trajectory, nullptr);
  EXPECT_EQ(result.decorations, nullptr);
  EXPECT_EQ(result.validation.reason,
            CompiledTrajectoryFailureReason3D::kInvalidTrackingErrorTube);
}

} // namespace
} // namespace drone_city_nav
