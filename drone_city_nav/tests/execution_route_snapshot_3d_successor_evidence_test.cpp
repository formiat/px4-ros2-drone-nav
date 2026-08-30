#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(ExecutionRouteSnapshot3DTest,
     SuccessorRequiresFreshEvidenceAndRejectsUnauthenticatedProducerSwitch) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_TRUE(active);
  const ExecutionRouteTransitionResult3D following =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(*active));
  ASSERT_TRUE(following.applied());
  const std::uint64_t current_raw_revision = SnapshotFixture3D::kLatestRawRevision + 1U;
  const ExecutionRouteTransitionResult3D advanced = advanceCertifiedRoute3D(
      *following.next, SnapshotFixture3D::guard(*following.next),
      fixture.executionObservation({4.0, 0.0, 5.0}, current_raw_revision,
                                   &fixture.raw_occupancy),
      SnapshotFixture3D::progressInput(*following.next, {4.0, 0.0, 5.0}),
      fixture.rawWorld(current_raw_revision));
  ASSERT_TRUE(advanced.applied());

  ExecutionRouteActivation3D stale_activation = fixture.activation();
  stale_activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
  stale_activation.observation.position = {4.0, 0.0, 5.0};
  const std::optional<CertifiedRouteSuffix3D> stale_successor =
      certifyExecutionRoute3D(stale_activation);
  ASSERT_TRUE(stale_successor.has_value());
  const FiniteExecutionState3D stale_execution =
      SnapshotFixture3D::finiteExecutionForRoute(*advanced.next, *stale_successor,
                                                 FiniteExecutionKind3D::kNominal, true,
                                                 102U);
  EXPECT_EQ(replaceCertifiedRoute3D(
                *advanced.next, SnapshotFixture3D::guard(*advanced.next),
                *stale_successor, stale_execution,
                testRouteSplice(*advanced.next->route(), *stale_successor))
                .status,
            ExecutionRouteTransitionStatus3D::kCertificateRegression);

  ExecutionRouteActivation3D fresh_activation = stale_activation;
  fresh_activation.observation.latest_raw_revision = current_raw_revision;
  fresh_activation.observed_raw_world = fixture.rawWorld(current_raw_revision);
  const std::optional<CertifiedRouteSuffix3D> fresh_successor =
      certifyExecutionRoute3D(fresh_activation);
  ASSERT_TRUE(fresh_successor.has_value());
  const FiniteExecutionState3D fresh_execution =
      SnapshotFixture3D::finiteExecutionForRoute(*advanced.next, *fresh_successor,
                                                 FiniteExecutionKind3D::kNominal, true,
                                                 102U);
  const ExecutionRouteTransitionResult3D accepted = replaceCertifiedRoute3D(
      *advanced.next, SnapshotFixture3D::guard(*advanced.next), *fresh_successor,
      fresh_execution, testRouteSplice(*advanced.next->route(), *fresh_successor));
  EXPECT_TRUE(accepted.applied());

  constexpr std::uint64_t kUnauthenticatedProducer{SnapshotFixture3D::kRawProducer +
                                                   100U};
  ExecutionRouteActivation3D switched_activation = stale_activation;
  switched_activation.proposal.planned_world.producer_instance_id =
      kUnauthenticatedProducer;
  switched_activation.proposal.validated_world.producer_instance_id =
      kUnauthenticatedProducer;
  switched_activation.observation.resident_world =
      switched_activation.proposal.validated_world;
  switched_activation.observation.latest_raw_producer_instance_id =
      kUnauthenticatedProducer;
  switched_activation.observed_raw_world = fixture.rawWorld(
      SnapshotFixture3D::kLatestRawRevision, nullptr, kUnauthenticatedProducer);
  const std::optional<CertifiedRouteSuffix3D> switched_successor =
      certifyExecutionRoute3D(switched_activation);
  ASSERT_TRUE(switched_successor.has_value());
  const FiniteExecutionState3D switched_execution =
      SnapshotFixture3D::finiteExecutionForRoute(*advanced.next, *switched_successor,
                                                 FiniteExecutionKind3D::kNominal, true,
                                                 102U);
  EXPECT_EQ(replaceCertifiedRoute3D(
                *advanced.next, SnapshotFixture3D::guard(*advanced.next),
                *switched_successor, switched_execution,
                testRouteSplice(*advanced.next->route(), *switched_successor))
                .status,
            ExecutionRouteTransitionStatus3D::kCertificateRegression);
}

} // namespace
} // namespace drone_city_nav
