#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/route_compiler_3d.hpp"

#include <gtest/gtest.h>

#include <array>
#include <span>
#include <vector>

#include "production_mppi_route_activation.hpp"
#include "production_mppi_route_helpers.hpp"

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

[[nodiscard]] std::shared_ptr<const std::vector<mppi::RouteSample3D>>
profile(const RouteEndpointSemantics3D semantics) {
  const std::vector<RouteSample3D> route = straightRoute();
  return makeMppiRoute3D(route, std::span<const ConstrainedRouteSpan>{}, 8.0, 4.0,
                         semantics, MppiSpeedPolicyConfig{});
}

TEST(ProductionMppiRouteHelpersTest,
     ContinuationUsesTheCanonicalPolicyLimitedSpeedAtTheLocalBoundary) {
  const auto route = profile(RouteEndpointSemantics3D::kContinuation);

  ASSERT_NE(route, nullptr);
  ASSERT_EQ(route->size(), 3U);
  EXPECT_FLOAT_EQ(route->front().reference_speed_mps, 5.0F);
  EXPECT_FLOAT_EQ(route->back().reference_speed_mps, 5.0F);
}

TEST(ProductionMppiRouteHelpersTest, RealStopsTaperTheNominalProfileToRest) {
  for (const RouteEndpointSemantics3D semantics :
       {RouteEndpointSemantics3D::kObservationStop,
        RouteEndpointSemantics3D::kMissionStop,
        RouteEndpointSemantics3D::kEmergencyBrakeTail}) {
    const auto route = profile(semantics);

    ASSERT_NE(route, nullptr);
    ASSERT_EQ(route->size(), 3U);
    EXPECT_GT(route->front().reference_speed_mps, 0.0F);
    EXPECT_GT((*route)[1].reference_speed_mps, 0.0F);
    EXPECT_FLOAT_EQ(route->back().reference_speed_mps, 0.0F);
  }
}

TEST(ProductionMppiRouteHelpersTest, UnknownEndpointSemanticsFailsClosedToRest) {
  const auto route = profile(static_cast<RouteEndpointSemantics3D>(255U));

  ASSERT_NE(route, nullptr);
  ASSERT_FALSE(route->empty());
  EXPECT_FLOAT_EQ(route->back().reference_speed_mps, 0.0F);
}

TEST(ProductionMppiRouteHelpersTest,
     TwoDimensionalContinuationAlsoUsesTheCanonicalPolicyLimitedSpeed) {
  const std::array<Point2, 3> points{
      Point2{0.0, 0.0},
      Point2{5.0, 0.0},
      Point2{10.0, 0.0},
  };

  const auto route =
      makeMppiRoute2D(points, 5.0, 6.0, RouteEndpointSemantics3D::kContinuation);

  ASSERT_NE(route, nullptr);
  ASSERT_FALSE(route->empty());
  EXPECT_FLOAT_EQ(route->back().reference_speed_mps, 5.0F);
}

TEST(ProductionMppiRouteHelpersTest,
     RouteProgressProjectionDisambiguatesVerticallyOverlappingSegments) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::array<Point3, 6>{Point3{0.0, 0.0, 0.0}, Point3{10.0, 0.0, 0.0},
                            Point3{10.0, 10.0, 0.0}, Point3{10.0, 10.0, 10.0},
                            Point3{10.0, 0.0, 10.0}, Point3{0.0, 0.0, 10.0}},
      1.0, 4.0);

  const GlobalGuideProjection projection =
      projectOntoRouteProgress3D(route, Point3{5.0, 0.0, 9.8});

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.station_m, 45.0, 1.0e-9);
  EXPECT_NEAR(projection.total_length_m, 50.0, 1.0e-9);
  EXPECT_NEAR(projection.remaining_m, 5.0, 1.0e-9);
  EXPECT_NEAR(projection.cross_track_m, 0.2, 1.0e-9);
  EXPECT_LT(projection.tangent.x, 0.0);
}

TEST(ProductionMppiRouteHelpersTest,
     CompiledCandidatePassesTheCompletePreArbitrationGeometryContract) {
  const std::vector<RouteSample3D> candidate =
      sampleRoute3D(std::array<Point3, 3>{Point3{0.0, 0.0, 5.0}, Point3{5.0, 0.0, 5.0},
                                          Point3{5.0, 5.0, 5.0}},
                    0.5, 4.0);
  const std::uint64_t fingerprint = routeFingerprint(candidate);
  RouteCompilationResult3D compilation = compileExecutionRoute3D(RouteCompilerInput3D{
      .route = candidate,
      .constrained_spans = {},
      .passage_volumes = {},
      .cooperative_passage_assignments = {},
      .selected_passage_traversal_ids = {},
      .passage_volume_config = PassageVolumeConfig{},
      .endpoint_semantics = RouteEndpointSemantics3D::kContinuation,
      .materialized_route_fingerprint = fingerprint,
      .config = RouteCompilerConfig3D{},
  });
  ASSERT_TRUE(compilation.compiled());
  ASSERT_NE(compilation.geometry, nullptr);

  ProductionRouteActivationResult3D prepared;
  prepared.proposal.identity = MaterializedRouteProposal3D{
      .route_fingerprint = fingerprint,
      .route_sample_count = compilation.geometry->route->size(),
      .activation_eligible = true,
  };
  prepared.proposal.geometry = *compilation.geometry;
  prepared.assessment = RouteActivationAssessment3D{
      .publication =
          RoutePublicationAssessment3D{.status = RoutePublicationStatus3D::kCompatible},
      .projection = RouteProjection3D{.valid = true},
      .raw_validation =
          RawRouteSuffixValidation3D{.status = RawRouteSuffixStatus3D::kValid},
      .objective_matches = true,
      .cross_track_accepted = true,
      .raw_world_compatible = true,
  };
  prepared.handoff = mppi::StaticRouteHandoffResult{
      .status = mppi::StaticRouteHandoffStatus::kAccepted,
      .accepted = true,
  };
  prepared.geometry_validation = compilation.validation;
  prepared.world_compatible = true;
  prepared.objective_matches = true;

  EXPECT_TRUE(prepared.executionGeometryValid());
  EXPECT_TRUE(prepared.readyForArbitration());
}

} // namespace
} // namespace drone_city_nav
