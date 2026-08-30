#include <algorithm>

#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(ExecutionRouteSnapshot3DTest,
     TrackingTubeConstraintIsIndependentAndCanBeDisabledExplicitly) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const VersionedExecutionValidationPolicy3D> base_policy =
      fixture.validation_policy;
  ASSERT_NE(base_policy, nullptr);
  const auto policy = [&base_policy](const bool tracking_tube_enabled) {
    return VersionedExecutionValidationPolicy3D::capture(
        base_policy->flightEnvelope(), base_policy->dynamics(),
        base_policy->altitudeEnvelope(), base_policy->sweptFootprint(),
        base_policy->latestLidarMaximumAgeMs(),
        base_policy->executionInputMaximumPoseAgeMs(),
        base_policy->executionInputMaximumControlAgeMs(), false, true,
        tracking_tube_enabled);
  };
  const auto lateral_detour = [](const CertifiedRouteSuffix3D& suffix,
                                 const std::uint64_t revision) {
    FiniteExecutionCertification3D certification =
        SnapshotFixture3D::finiteCertificationForRoute(
            suffix, FiniteExecutionKind3D::kNominal, revision);
    constexpr std::size_t kQuarterControlCount{25U};
    constexpr float kLateralAccelerationMps2{0.48F};
    for (std::size_t index = 0U; index < 4U * kQuarterControlCount; ++index) {
      certification.horizon.controls[index].ay =
          (index < kQuarterControlCount || index >= 3U * kQuarterControlCount)
              ? kLateralAccelerationMps2
              : -kLateralAccelerationMps2;
    }
    certification.horizon.states.front() = certification.execution_input->state();
    for (std::size_t index = 0U; index < certification.horizon.controls.size();
         ++index) {
      certification.horizon.states[index + 1U] = mppi::integrateReference(
          certification.horizon.states[index], certification.horizon.controls[index],
          suffix.validation_policy->dynamics());
    }
    return certification;
  };

  fixture.validation_policy = policy(true);
  ASSERT_NE(fixture.validation_policy, nullptr);
  const std::optional<CertifiedRouteSuffix3D> strict_suffix = fixture.certify();
  ASSERT_TRUE(strict_suffix.has_value());
  const std::shared_ptr<const ExecutionPlan3D> strict_initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(strict_initial, nullptr);
  FiniteExecutionCertification3D strict_candidate =
      lateral_detour(*strict_suffix, 102U);
  ASSERT_TRUE(mppi::finiteHorizonHasTerminalRestState(strict_candidate.horizon));
  const auto maximum_lateral_state =
      std::ranges::max_element(strict_candidate.horizon.states, {}, &mppi::State::y);
  ASSERT_NE(maximum_lateral_state, strict_candidate.horizon.states.end());
  ASSERT_GT(maximum_lateral_state->y, 2.5F);
  EXPECT_NEAR(strict_candidate.horizon.states.back().y, 0.0F, 1.0e-4F);

  const FiniteExecutionCertificationResult3D strict_result =
      certifyFiniteExecution3DDetailed(*strict_initial, *strict_suffix,
                                       std::move(strict_candidate));

  EXPECT_FALSE(strict_result.certified())
      << "status=" << finiteExecutionCertificationStatus3DName(strict_result.status)
      << " route_adherence_status="
      << finiteExecutionRouteAdherenceStatus3DName(strict_result.route_adherence_status)
      << " route_adherence_failure_distance_m="
      << strict_result.route_adherence_failure_distance_m;
  EXPECT_EQ(strict_result.status,
            FiniteExecutionCertificationStatus3D::kRouteAdherenceRejected);
  EXPECT_EQ(strict_result.route_adherence_status,
            FiniteExecutionRouteAdherenceStatus3D::kTrackingTubeExceeded);

  fixture.validation_policy = policy(false);
  ASSERT_NE(fixture.validation_policy, nullptr);
  const std::optional<CertifiedRouteSuffix3D> permissive_suffix = fixture.certify();
  ASSERT_TRUE(permissive_suffix.has_value());
  const std::shared_ptr<const ExecutionPlan3D> permissive_initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(permissive_initial, nullptr);
  const FiniteExecutionCertificationResult3D permissive_result =
      certifyFiniteExecution3DDetailed(*permissive_initial, *permissive_suffix,
                                       lateral_detour(*permissive_suffix, 103U));

  EXPECT_TRUE(permissive_result.certified())
      << "status=" << finiteExecutionCertificationStatus3DName(permissive_result.status)
      << " route_adherence_status="
      << finiteExecutionRouteAdherenceStatus3DName(
             permissive_result.route_adherence_status)
      << " route_adherence_failure_distance_m="
      << permissive_result.route_adherence_failure_distance_m;
}

} // namespace
} // namespace drone_city_nav
