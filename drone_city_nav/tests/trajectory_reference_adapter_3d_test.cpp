#include "drone_city_nav/mppi/trajectory_reference_adapter_3d.hpp"
#include "drone_city_nav/trajectory_compiler_3d.hpp"

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::shared_ptr<const CompiledTrajectory3D> compiledContinuation() {
  const std::array<Point3, 3> points{
      Point3{0.0, 0.0, 5.0},
      Point3{5.0, 0.0, 5.0},
      Point3{10.0, 0.0, 5.0},
  };
  std::vector<RouteSample3D> route = sampleRoute3D(points, 5.0, 8.0);
  const std::uint64_t fingerprint = routeFingerprint(route);
  TrajectoryCompilerConfig3D config;
  config.unconstrained_speed_mps = 8.0;
  config.constrained_speed_mps = 4.0;
  const TrajectoryCompilationResult3D compilation =
      TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
          .exact_initial_state =
              VehicleState3D{
                  .identity =
                      VehicleStateIdentity3D{
                          .revision = 1U,
                          .source_timestamp_us = 2U,
                          .receive_stamp_ns = 3,
                      },
                  .position = route.front().position,
              },
          .route_generation = 1U,
          .route = std::move(route),
          .constrained_spans = {},
          .endpoint_semantics = RouteEndpointSemantics3D::kContinuation,
          .materialized_route_fingerprint = fingerprint,
          .config = config,
      });
  return compilation.compiled() ? compilation.trajectory : nullptr;
}

TEST(TrajectoryReferenceAdapter3DTest,
     ControllerReferenceIsDerivedFromTheCanonicalSealedProfile) {
  const std::shared_ptr<const CompiledTrajectory3D> trajectory = compiledContinuation();
  ASSERT_NE(trajectory, nullptr);

  const auto reference = mppi::adaptTrajectoryReference3D(*trajectory);

  ASSERT_NE(reference, nullptr);
  ASSERT_EQ(reference->size(), trajectory->route->size());
  for (std::size_t index = 0U; index < reference->size(); ++index) {
    EXPECT_FLOAT_EQ(
        (*reference)[index].reference_speed_mps,
        static_cast<float>((*trajectory->route)[index].reference_speed_mps));
  }
  EXPECT_GT(reference->back().reference_speed_mps, 0.0F);
}

TEST(TrajectoryReferenceAdapter3DTest, CachesReferenceByTrajectoryIdentity) {
  const std::shared_ptr<const CompiledTrajectory3D> trajectory = compiledContinuation();
  ASSERT_NE(trajectory, nullptr);
  mppi::TrajectoryReferenceAdapter3D adapter;

  const auto first = adapter.adapt(trajectory);
  const auto second = adapter.adapt(trajectory);

  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first, second);
  EXPECT_EQ(adapter.adapt(nullptr), nullptr);
}

} // namespace
} // namespace drone_city_nav
