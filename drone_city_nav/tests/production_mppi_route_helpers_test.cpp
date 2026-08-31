#include "drone_city_nav/compiled_trajectory_views_3d.hpp"
#include "drone_city_nav/execution_route_certification_3d.hpp"
#include "drone_city_nav/mppi/trajectory_reference_adapter_3d.hpp"
#include "drone_city_nav/trajectory_compiler_3d.hpp"

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <vector>

#include "mppi_controller_3d.hpp"
#include "production_mppi_route_helpers.hpp"
#include "route_activation_coordinator_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<RouteSample3D> straightRoute() {
  const std::array<Point3, 3> points{
      Point3{0.0, 0.0, 5.0},
      Point3{5.0, 0.0, 5.0},
      Point3{10.0, 0.0, 5.0},
  };
  return sampleRoute3D(points, 5.0, 8.0);
}

[[nodiscard]] VehicleState3D initialState(const Point3& position) {
  return VehicleState3D{
      .identity =
          VehicleStateIdentity3D{
              .revision = 1U,
              .source_timestamp_us = 2U,
              .receive_stamp_ns = 3,
          },
      .position = position,
  };
}

[[nodiscard]] TrajectoryCompilationResult3D
compileProfile(const RouteEndpointSemantics3D semantics) {
  std::vector<RouteSample3D> route = straightRoute();
  const std::uint64_t fingerprint = routeFingerprint(route);
  TrajectoryCompilerConfig3D config;
  config.unconstrained_speed_mps = 8.0;
  config.constrained_speed_mps = 4.0;
  return TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
      .exact_initial_state = initialState(route.front().position),
      .route_generation = 1U,
      .route = std::move(route),
      .constrained_spans = {},
      .endpoint_semantics = semantics,
      .materialized_route_fingerprint = fingerprint,
      .config = config,
  });
}

TEST(ProductionMppiRouteHelpersTest,
     ControllerReferenceIsDerivedFromTheCanonicalSealedProfile) {
  const TrajectoryCompilationResult3D continuation =
      compileProfile(RouteEndpointSemantics3D::kContinuation);
  ASSERT_TRUE(continuation.compiled());

  const auto reference = mppi::adaptTrajectoryReference3D(*continuation.trajectory);

  ASSERT_NE(reference, nullptr);
  ASSERT_EQ(reference->size(), continuation.trajectory->route->size());
  for (std::size_t index = 0U; index < reference->size(); ++index) {
    EXPECT_FLOAT_EQ((*reference)[index].reference_speed_mps,
                    static_cast<float>(
                        (*continuation.trajectory->route)[index].reference_speed_mps));
  }
  EXPECT_GT(reference->back().reference_speed_mps, 0.0F);
}

TEST(ProductionMppiRouteHelpersTest, RealStopsTaperTheSealedProfileToRest) {
  for (const RouteEndpointSemantics3D semantics :
       {RouteEndpointSemantics3D::kLocalStop, RouteEndpointSemantics3D::kMissionStop,
        RouteEndpointSemantics3D::kEmergencyBrakeTail}) {
    const TrajectoryCompilationResult3D compilation = compileProfile(semantics);

    ASSERT_TRUE(compilation.compiled());
    ASSERT_FALSE(compilation.trajectory->route->empty());
    EXPECT_DOUBLE_EQ(compilation.trajectory->route->back().reference_speed_mps, 0.0);
  }
}

TEST(ProductionMppiRouteHelpersTest,
     PlanarProjectionIsAnOnDemandViewWithoutIndependentAuthority) {
  const TrajectoryCompilationResult3D compilation =
      compileProfile(RouteEndpointSemantics3D::kContinuation);
  ASSERT_TRUE(compilation.compiled());

  const std::vector<Point2> projection =
      projectCompiledTrajectoryTo2D(*compilation.trajectory);

  ASSERT_EQ(projection.size(), compilation.trajectory->route->size());
  for (std::size_t index = 0U; index < projection.size(); ++index) {
    EXPECT_DOUBLE_EQ(projection[index].x,
                     (*compilation.trajectory->route)[index].position.x);
    EXPECT_DOUBLE_EQ(projection[index].y,
                     (*compilation.trajectory->route)[index].position.y);
  }
}

TEST(ProductionMppiRouteHelpersTest,
     ControllerOwnsReferenceCacheKeyedByTrajectoryIdentity) {
  const TrajectoryCompilationResult3D compilation =
      compileProfile(RouteEndpointSemantics3D::kContinuation);
  ASSERT_TRUE(compilation.compiled());
  mppi::BenchmarkConfig config;
  config.rollouts = 64U;
  config.steps = 8U;
  MppiController3D controller{config};

  const auto first = controller.adaptTrajectoryReference(compilation.trajectory);
  const auto second = controller.adaptTrajectoryReference(compilation.trajectory);

  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first, second);
  EXPECT_EQ(controller.adaptTrajectoryReference(nullptr), nullptr);
}

TEST(ProductionMppiRouteHelpersTest,
     RouteProgressProjectionDisambiguatesVerticallyOverlappingSegments) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::array<Point3, 6>{Point3{0.0, 0.0, 0.0}, Point3{10.0, 0.0, 0.0},
                            Point3{10.0, 10.0, 0.0}, Point3{10.0, 10.0, 10.0},
                            Point3{10.0, 0.0, 10.0}, Point3{0.0, 0.0, 10.0}},
      1.0, 4.0);

  const RouteProgressProjection3D projection =
      projectOntoRouteProgress3D(route, Point3{5.0, 0.0, 9.8});

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.station_m, 45.0, 1.0e-9);
  EXPECT_NEAR(projection.total_length_m, 50.0, 1.0e-9);
  EXPECT_NEAR(projection.remaining_m, 5.0, 1.0e-9);
  EXPECT_NEAR(projection.cross_track_m, 0.2, 1.0e-9);
  EXPECT_LT(projection.tangent.x, 0.0);
}

TEST(ProductionMppiRouteHelpersTest,
     CompiledTrajectoryPassesTheCompletePreArbitrationContract) {
  const TrajectoryCompilationResult3D compilation =
      compileProfile(RouteEndpointSemantics3D::kContinuation);
  ASSERT_TRUE(compilation.compiled());
  const std::uint64_t fingerprint =
      compilation.trajectory->materialized_route_fingerprint;
  const RouteDecorationCompilationResult3D decoration_compilation =
      RouteDecorationCompiler3D::compile(RouteDecorationCompilerInput3D{
          .trajectory = compilation.trajectory,
          .route_generation = 1U,
          .passage_volumes = {},
          .cooperative_passage_assignments = {},
          .selected_passage_traversal_ids = {},
      });
  ASSERT_TRUE(decoration_compilation.compiled());

  ProductionMaterializedRouteProposal3D proposal{
      .identity =
          MaterializedRouteProposal3D{
              .route_fingerprint = fingerprint,
              .route_sample_count = compilation.trajectory->route->size(),
              .activation_eligible = true,
          },
      .trajectory = compilation.trajectory,
      .decorations = decoration_compilation.decorations,
  };
  RouteAdmissionReport3D admission;
  admission.assessment = RouteActivationAssessment3D{
      .publication =
          RoutePublicationAssessment3D{.status = RoutePublicationStatus3D::kCompatible},
      .projection = RouteProjection3D{.valid = true},
      .raw_validation =
          RawRouteSuffixValidation3D{.status = RawRouteSuffixStatus3D::kValid},
      .objective_matches = true,
      .cross_track_accepted = true,
      .raw_world_compatible = true,
  };
  admission.handoff = DynamicHandoffResult3D{
      .status = DynamicHandoffStatus3D::kAccepted,
  };
  admission.trajectory_validation = compilation.validation;
  admission.decoration_validation = decoration_compilation.validation;
  admission.world_compatible = true;
  admission.objective_matches = true;

  EXPECT_TRUE(admission.compiledTrajectoryValid());
  EXPECT_TRUE(admission.readyForArbitration(proposal));
}

TEST(ProductionMppiRouteHelpersTest,
     FailedCompilationCannotMutateAnEarlierSealedTrajectory) {
  const TrajectoryCompilationResult3D compiled =
      compileProfile(RouteEndpointSemantics3D::kContinuation);
  ASSERT_TRUE(compiled.compiled());
  const auto retained = compiled.trajectory;
  const std::uint64_t retained_revision = retained->compiled_trajectory_revision;

  std::vector<RouteSample3D> invalid_route = straightRoute();
  const TrajectoryCompilationResult3D rejected =
      TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
          .route_generation = 1U,
          .route = std::move(invalid_route),
          .constrained_spans = {},
          .materialized_route_fingerprint = retained->materialized_route_fingerprint,
      });

  EXPECT_FALSE(rejected.compiled());
  EXPECT_EQ(rejected.validation.reason,
            CompiledTrajectoryFailureReason3D::kInvalidInitialState);
  EXPECT_EQ(retained, compiled.trajectory);
  EXPECT_EQ(retained->compiled_trajectory_revision, retained_revision);
}

TEST(ProductionMppiRouteHelpersTest,
     PendingPublicationAllowsRawAdvanceButRejectsTransactionBaseChanges) {
  PendingRoutePublicationCurrentness3D currentness{
      .resident_world_current = true,
      .objective_current = true,
      .raw_world_current = true,
      .execution_base_current = true,
      .pending_current = true,
      .candidate_world_coherent = true,
  };
  EXPECT_TRUE(pendingRoutePublicationBaseCurrent3D(currentness));

  currentness.raw_world_current = false;
  EXPECT_TRUE(pendingRoutePublicationBaseCurrent3D(currentness));
  currentness.raw_world_current = true;

  currentness.resident_world_current = false;
  EXPECT_FALSE(pendingRoutePublicationBaseCurrent3D(currentness));
  currentness.resident_world_current = true;
  currentness.objective_current = false;
  EXPECT_FALSE(pendingRoutePublicationBaseCurrent3D(currentness));
  currentness.objective_current = true;
  currentness.execution_base_current = false;
  EXPECT_FALSE(pendingRoutePublicationBaseCurrent3D(currentness));
  currentness.execution_base_current = true;
  currentness.pending_current = false;
  EXPECT_FALSE(pendingRoutePublicationBaseCurrent3D(currentness));
  currentness.pending_current = true;
  currentness.candidate_world_coherent = false;
  EXPECT_FALSE(pendingRoutePublicationBaseCurrent3D(currentness));
}

} // namespace
} // namespace drone_city_nav
