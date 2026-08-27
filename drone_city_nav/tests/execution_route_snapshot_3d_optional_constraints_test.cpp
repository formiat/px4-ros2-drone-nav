#include "execution_route_snapshot_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(ExecutionRouteSnapshot3DTest,
     OptionalCrossTrackConstraintDoesNotRejectAnIndependentlySafeHorizon) {
  SnapshotFixture3D fixture;
  fixture.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      fixture.validation_policy->flightEnvelope(),
      fixture.validation_policy->dynamics(),
      fixture.validation_policy->altitudeEnvelope(),
      fixture.validation_policy->sweptFootprint(),
      fixture.validation_policy->latestLidarMaximumAgeMs(),
      fixture.validation_policy->executionInputMaximumPoseAgeMs(),
      fixture.validation_policy->executionInputMaximumControlAgeMs(), false);
  ASSERT_NE(fixture.validation_policy, nullptr);
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);
  FiniteExecutionCertification3D certification =
      SnapshotFixture3D::finiteCertificationForRoute(
          *suffix, FiniteExecutionKind3D::kNominal, 102U);

  constexpr std::size_t kAccelerationControlCount{50U};
  constexpr float kTerminalLateralOffsetM{3.0F};
  const float lateral_acceleration_mps2 = kTerminalLateralOffsetM / 25.0F;
  for (std::size_t index = 0U; index < kAccelerationControlCount; ++index) {
    certification.horizon.controls[index].ay = lateral_acceleration_mps2;
  }
  for (std::size_t index = kAccelerationControlCount;
       index < 2U * kAccelerationControlCount; ++index) {
    certification.horizon.controls[index].ay = -lateral_acceleration_mps2;
  }
  certification.horizon.states.front() = certification.execution_input->state();
  for (std::size_t index = 0U; index < certification.horizon.controls.size(); ++index) {
    certification.horizon.states[index + 1U] = mppi::integrateReference(
        certification.horizon.states[index], certification.horizon.controls[index],
        suffix->validation_policy->dynamics());
  }
  ASSERT_TRUE(mppi::finiteHorizonHasTerminalRestState(certification.horizon));
  ASSERT_GT(certification.horizon.states.back().y,
            static_cast<float>(kFiniteExecutionRouteCrossTrackToleranceM3D));

  const FiniteExecutionCertificationResult3D result =
      certifyFiniteExecution3DDetailed(*initial, *suffix, std::move(certification));

  EXPECT_TRUE(result.certified())
      << "status=" << finiteExecutionCertificationStatus3DName(result.status)
      << " route_adherence_status="
      << finiteExecutionRouteAdherenceStatus3DName(result.route_adherence_status)
      << " route_adherence_failure_distance_m="
      << result.route_adherence_failure_distance_m;
}

} // namespace
} // namespace drone_city_nav
