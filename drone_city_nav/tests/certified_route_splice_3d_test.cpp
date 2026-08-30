#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] CertifiedRouteSpliceConfig3D
spliceConfig(const double required_overlap_m = 2.0) noexcept {
  return CertifiedRouteSpliceConfig3D{
      .required_overlap_m = required_overlap_m,
      .sample_step_m = 0.5,
      .maximum_position_separation_m = 2.0,
      .minimum_tangent_alignment = 0.5,
      .activation_station_tolerance_m = 1.0,
  };
}

[[nodiscard]] std::optional<CertifiedRouteSuffix3D>
certifySuccessor(SnapshotFixture3D& fixture, const std::vector<RouteSample3D>& route) {
  std::vector<RouteSample3D> continuation_route = route;
  continuation_route.back().reference_speed_mps = 4.0;
  ExecutionRouteActivation3D activation = fixture.activation();
  activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
  activation.proposal.evidence.reaches_mission_target = false;
  activation.proposal.reaches_mission_goal = false;
  activation.proposal.route_fingerprint = routeFingerprint(continuation_route);
  activation.proposal.route_sample_count = continuation_route.size();
  activation.proposal.evidence.route_length_m = continuation_route.back().station_m;
  activation.geometry = makeGeometry(
      continuation_route, activation.proposal.route_fingerprint,
      TrackingErrorTubeWorld3D{
          .observed_occupancy = &fixture.raw_occupancy,
          .occupied_content_fingerprint =
              fixture.raw_occupancy.occupiedSnapshot().contentFingerprint(),
      });
  return certifyExecutionRoute3D(activation);
}

TEST(CertifiedRouteSplice3DTest, CertifiesBoundOverlapAndAdmitsOnlyTheSharedWindow) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifySuccessor(fixture, fixture.route);
  ASSERT_TRUE(successor.has_value());

  const RouteSpliceCertificationResult3D certification = certifyRouteSplice3D(
      *active->route(), *successor, Point3{2.0, 0.0, 5.0}, spliceConfig());

  ASSERT_TRUE(certification.certified());
  ASSERT_TRUE(certification.splice.has_value());
  EXPECT_TRUE(certification.splice->validFor(*active->route(), *successor));
  EXPECT_DOUBLE_EQ(certification.splice->base_begin_station_m, 2.0);
  EXPECT_DOUBLE_EQ(certification.splice->base_end_station_m, 4.0);
  EXPECT_DOUBLE_EQ(certification.splice->required_overlap_m, 2.0);
  const RouteSpliceReadiness3D readiness = assessRouteSpliceReadiness3D(
      *certification.splice, *active->route(), *successor, Point3{2.0, 0.0, 5.0});
  EXPECT_TRUE(readiness.ready());
  EXPECT_DOUBLE_EQ(readiness.position_separation_m, 0.0);
  EXPECT_DOUBLE_EQ(readiness.tangent_alignment, 1.0);
}

TEST(CertifiedRouteSplice3DTest, RejectsInsufficientCertifiedOverlap) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifySuccessor(fixture, fixture.route);
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(successor.has_value());

  const RouteSpliceCertificationResult3D certification = certifyRouteSplice3D(
      *active->route(), *successor, Point3{2.0, 0.0, 5.0}, spliceConfig(9.0));

  EXPECT_EQ(certification.status,
            RouteSpliceCertificationStatus3D::kInsufficientOverlap);
  EXPECT_FALSE(certification.splice.has_value());
}

TEST(CertifiedRouteSplice3DTest, RejectsACommonStartThatDivergesBeforeOverlapEnd) {
  SnapshotFixture3D fixture;
  const std::vector<RouteSample3D> diverged = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {5.0, 3.0, 5.0}, {10.0, 3.0, 5.0}}, 0.5,
      4.0);
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifySuccessor(fixture, diverged);
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(successor.has_value());

  const RouteSpliceCertificationResult3D certification = certifyRouteSplice3D(
      *active->route(), *successor, Point3{2.0, 0.0, 5.0}, spliceConfig());

  EXPECT_EQ(certification.status, RouteSpliceCertificationStatus3D::kGeometryDiverged);
  EXPECT_FALSE(certification.splice.has_value());
}

TEST(CertifiedRouteSplice3DTest,
     StrictFrozenPrefixContractRejectsAParallelButDisplacedSuccessor) {
  SnapshotFixture3D fixture;
  const std::vector<RouteSample3D> displaced =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.04, 5.0}, {10.0, 0.04, 5.0}}, 0.5, 4.0);
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifySuccessor(fixture, displaced);
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(successor.has_value());

  CertifiedRouteSpliceConfig3D strict = spliceConfig();
  strict.maximum_position_separation_m = 0.01;
  strict.minimum_tangent_alignment = 0.995;
  const RouteSpliceCertificationResult3D certification =
      certifyRouteSplice3D(*active->route(), *successor, Point3{2.0, 0.0, 5.0}, strict);

  EXPECT_EQ(certification.status, RouteSpliceCertificationStatus3D::kGeometryDiverged);
  EXPECT_FALSE(certification.splice.has_value());
}

TEST(CertifiedRouteSplice3DTest, RejectsOpposedTangentsInsideSharedGeometry) {
  SnapshotFixture3D fixture;
  const std::vector<RouteSample3D> opposed =
      sampleRoute3D(std::vector<Point3>{{10.0, 0.0, 5.0}, {0.0, 0.0, 5.0}}, 0.5, 4.0);
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifySuccessor(fixture, opposed);
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(successor.has_value());

  const RouteSpliceCertificationResult3D certification = certifyRouteSplice3D(
      *active->route(), *successor, Point3{2.0, 0.0, 5.0}, spliceConfig());

  EXPECT_EQ(certification.status,
            RouteSpliceCertificationStatus3D::kTangentDiscontinuity);
  EXPECT_FALSE(certification.splice.has_value());
}

TEST(CertifiedRouteSplice3DTest, ExpiredWindowCannotAuthorizeReplacement) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifySuccessor(fixture, fixture.route);
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(successor.has_value());
  const RouteSpliceCertificationResult3D certification = certifyRouteSplice3D(
      *active->route(), *successor, Point3{2.0, 0.0, 5.0}, spliceConfig());
  ASSERT_TRUE(certification.certified());

  CertifiedRouteSuffix3D advanced = *active->route();
  advanced.progress.station_m = 5.5;
  advanced.progress.last_observed_position = {5.5, 0.0, 5.0};
  EXPECT_TRUE(routeSpliceWindowExpired3D(*certification.splice, advanced));
  EXPECT_FALSE(certification.splice->validFor(advanced, *successor));
}

} // namespace
} // namespace drone_city_nav
