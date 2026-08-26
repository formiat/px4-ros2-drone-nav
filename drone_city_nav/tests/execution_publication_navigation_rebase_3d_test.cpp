#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>

#include "execution_publication_navigation_rebase_3d.hpp"
#include "execution_route_snapshot_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::shared_ptr<const VersionedExecutionInput3D>
advanceExecutionInput(const VersionedExecutionInput3D& source,
                      const std::int64_t publication_now_ns,
                      std::optional<mppi::State> state = std::nullopt) {
  return VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
      .capture_sequence = source.captureSequence() + 1U,
      .pose_revision = source.poseRevision() + 1U,
      .pose_source_timestamp_us = source.poseSourceTimestampUs() + 1U,
      .pose_receive_stamp_ns = source.poseReceiveStampNs() + 10'000,
      .effective_stamp_ns = publication_now_ns,
      .state = state.value_or(source.state()),
      .full_state_authoritative = source.fullStateAuthoritative(),
      .state_provenance = source.stateProvenance(),
      .previous_control = source.previousControl(),
      .previous_control_source = source.previousControlSource(),
      .previous_control_source_producer_instance_id =
          source.previousControlSourceProducerInstanceId(),
      .previous_control_source_sequence = source.previousControlSourceSequence() + 1U,
      .previous_control_source_stamp_ns =
          source.previousControlSourceStampNs() + 10'000,
      .previous_control_receive_stamp_ns =
          source.previousControlReceiveStampNs() + 10'000,
  });
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     RecertifiesPendingActivationFromTheCurrentNavigationInput) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_NE(initial, nullptr);
  ASSERT_TRUE(route.has_value());

  FiniteExecutionState3D candidate_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *initial,
          route.value(), // NOLINT(bugprone-unchecked-optional-access)
          FiniteExecutionKind3D::kNominal, true, 100U);
  const ExecutionRouteTransitionResult3D candidate = activateCertifiedRoute3D(
      *initial, initial->version,
      route.value(), // NOLINT(bugprone-unchecked-optional-access)
      std::move(candidate_execution));
  ASSERT_TRUE(candidate.applied());
  ASSERT_NE(candidate.next, nullptr);
  ASSERT_TRUE(candidate.next->finite_execution.has_value());
  const FiniteExecutionState3D& candidate_owner =
      candidate
          .next // NOLINT(bugprone-unchecked-optional-access)
          ->finite_execution.value();
  ASSERT_NE(candidate_owner.execution_input, nullptr);
  ASSERT_NE(candidate_owner.latest_lidar_evidence, nullptr);
  ASSERT_NE(candidate_owner.observed_raw_world, nullptr);
  ASSERT_LT(candidate_owner.execution_input->captureSequence(),
            std::numeric_limits<std::uint64_t>::max());

  const std::int64_t publication_now_ns = candidate_owner.valid_from_ns + 20'000'000;
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(*candidate_owner.execution_input, publication_now_ns);
  ASSERT_NE(current_input, nullptr);
  mppi::FiniteHorizonConfig finite_horizon_config;

  ExecutionPublicationNavigationRebaseResult3D result =
      rebaseExecutionPublicationForCurrentNavigation3D(
          ExecutionPublicationNavigationRebaseRequest3D{
              .expected_snapshot = initial.get(),
              .candidate_snapshot = candidate.next.get(),
              .expected_pending = nullptr,
              .current_execution_input = current_input,
              .current_lidar_evidence = candidate_owner.latest_lidar_evidence,
              .current_observed_raw_world = candidate_owner.observed_raw_world,
              .publication_now_ns = publication_now_ns,
              .arrival_search_step_controls = 5U,
              .finite_horizon_config = &finite_horizon_config,
              .terminal_boundary = std::nullopt,
          });

  ASSERT_TRUE(result.rebased());
  EXPECT_EQ(result.path_validation_status, mppi::FiniteExecutionPathStatus::kValid);
  EXPECT_EQ(result.route_certification_status,
            FiniteExecutionCertificationStatus3D::kCertified);
  EXPECT_EQ(result.route_adherence_status,
            FiniteExecutionRouteAdherenceStatus3D::kAccepted);
  EXPECT_EQ(result.transition_status, ExecutionRouteTransitionStatus3D::kApplied);
  ASSERT_TRUE(result.transition.has_value());
  const ExecutionRouteTransitionResult3D& transition =
      result.transition.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_NE(transition.next, nullptr);
  ASSERT_TRUE(transition.next->finite_execution.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const FiniteExecutionState3D& rebased = transition.next->finite_execution.value();
  EXPECT_EQ(rebased.execution_input, current_input);
  EXPECT_EQ(rebased.valid_from_ns, publication_now_ns);
  ASSERT_NE(rebased.horizon, nullptr);
  EXPECT_EQ(rebased.horizon->states.front().x, current_input->state().x);
  EXPECT_TRUE(mppi::finiteHorizonHasTerminalRestState(*rebased.horizon));
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     RecertifiesAnAlreadyActiveRouteWithoutLosingItsOwnerIdentity) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  ASSERT_TRUE(active->finite_execution.has_value());

  const ExecutionRouteTransitionResult3D candidate = replaceFiniteExecution3D(
      *active, SnapshotFixture3D::guard(*active),
      SnapshotFixture3D::finiteExecution(*active, FiniteExecutionKind3D::kNominal, true,
                                         101U));
  ASSERT_TRUE(candidate.applied());
  ASSERT_NE(candidate.next, nullptr);
  ASSERT_TRUE(candidate.next->finite_execution.has_value());
  const FiniteExecutionState3D& candidate_owner =
      candidate
          .next // NOLINT(bugprone-unchecked-optional-access)
          ->finite_execution.value();
  ASSERT_NE(candidate_owner.execution_input, nullptr);
  ASSERT_NE(candidate_owner.latest_lidar_evidence, nullptr);
  ASSERT_NE(candidate_owner.observed_raw_world, nullptr);

  const std::int64_t publication_now_ns = candidate_owner.valid_from_ns + 20'000'000;
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(*candidate_owner.execution_input, publication_now_ns);
  ASSERT_NE(current_input, nullptr);
  mppi::FiniteHorizonConfig finite_horizon_config;

  const ExecutionPublicationNavigationRebaseResult3D result =
      rebaseExecutionPublicationForCurrentNavigation3D(
          ExecutionPublicationNavigationRebaseRequest3D{
              .expected_snapshot = active.get(),
              .candidate_snapshot = candidate.next.get(),
              .expected_pending = nullptr,
              .current_execution_input = current_input,
              .current_lidar_evidence = candidate_owner.latest_lidar_evidence,
              .current_observed_raw_world = candidate_owner.observed_raw_world,
              .publication_now_ns = publication_now_ns,
              .arrival_search_step_controls = 5U,
              .finite_horizon_config = &finite_horizon_config,
              .terminal_boundary = std::nullopt,
          });

  ASSERT_TRUE(result.rebased());
  EXPECT_EQ(result.path_validation_status, mppi::FiniteExecutionPathStatus::kValid);
  EXPECT_EQ(result.route_certification_status,
            FiniteExecutionCertificationStatus3D::kCertified);
  EXPECT_EQ(result.route_adherence_status,
            FiniteExecutionRouteAdherenceStatus3D::kAccepted);
  EXPECT_EQ(result.transition_status, ExecutionRouteTransitionStatus3D::kApplied);
  ASSERT_TRUE(result.transition.has_value());
  ASSERT_NE(result.transition->next, nullptr);
  ASSERT_TRUE(result.transition->next->route.has_value());
  ASSERT_TRUE(result.transition->next->finite_execution.has_value());
  EXPECT_EQ(result.transition->next->route->identity.generation,
            active->route->identity.generation);
  EXPECT_EQ(result.transition->next->finite_execution->execution_input, current_input);
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     ReportsCertificationThatRejectsEveryPhysicalCandidate) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  const std::optional<CertifiedRouteSuffix3D> route = fixture.certify();
  ASSERT_NE(initial, nullptr);
  ASSERT_TRUE(route.has_value());

  FiniteExecutionState3D candidate_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *initial,
          route.value(), // NOLINT(bugprone-unchecked-optional-access)
          FiniteExecutionKind3D::kNominal, true, 100U);
  const ExecutionRouteTransitionResult3D candidate = activateCertifiedRoute3D(
      *initial, initial->version,
      route.value(), // NOLINT(bugprone-unchecked-optional-access)
      std::move(candidate_execution));
  ASSERT_TRUE(candidate.applied());
  ASSERT_NE(candidate.next, nullptr);
  const FiniteExecutionState3D& candidate_owner =
      candidate
          .next // NOLINT(bugprone-unchecked-optional-access)
          ->finite_execution.value();
  ASSERT_NE(candidate_owner.execution_input, nullptr);
  ASSERT_NE(candidate_owner.latest_lidar_evidence, nullptr);
  ASSERT_NE(candidate_owner.observed_raw_world, nullptr);
  const std::int64_t publication_now_ns = candidate_owner.valid_from_ns + 20'000'000;
  mppi::State off_route_state = candidate_owner.execution_input->state();
  off_route_state.y = 3.0F;
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(*candidate_owner.execution_input, publication_now_ns,
                            off_route_state);
  ASSERT_NE(current_input, nullptr);
  mppi::FiniteHorizonConfig finite_horizon_config;

  const ExecutionPublicationNavigationRebaseResult3D result =
      rebaseExecutionPublicationForCurrentNavigation3D(
          ExecutionPublicationNavigationRebaseRequest3D{
              .expected_snapshot = initial.get(),
              .candidate_snapshot = candidate.next.get(),
              .current_execution_input = current_input,
              .current_lidar_evidence = candidate_owner.latest_lidar_evidence,
              .current_observed_raw_world = candidate_owner.observed_raw_world,
              .publication_now_ns = publication_now_ns,
              .arrival_search_step_controls = 5U,
              .finite_horizon_config = &finite_horizon_config,
              .terminal_boundary = std::nullopt,
          });

  EXPECT_FALSE(result.rebased());
  EXPECT_EQ(result.status, ExecutionPublicationNavigationRebaseStatus3D::kPathRejected);
  EXPECT_EQ(result.path_validation_status,
            mppi::FiniteExecutionPathStatus::kCandidateRejected);
  EXPECT_EQ(result.route_certification_status,
            FiniteExecutionCertificationStatus3D::kExecutionBindingRejected);
  EXPECT_EQ(result.route_adherence_status,
            FiniteExecutionRouteAdherenceStatus3D::kNotEvaluated);
  EXPECT_LT(result.route_adherence_failure_distance_m, 0.0);
}

TEST(ExecutionPublicationNavigationRebase3DTest, InvalidRequestFailsClosed) {
  const ExecutionPublicationNavigationRebaseResult3D result =
      rebaseExecutionPublicationForCurrentNavigation3D({});
  EXPECT_FALSE(result.rebased());
  EXPECT_EQ(result.status,
            ExecutionPublicationNavigationRebaseStatus3D::kInvalidRequest);
  EXPECT_STREQ(executionPublicationNavigationRebaseStatus3DName(result.status),
               "late_rebase_invalid_request");
}

} // namespace
} // namespace drone_city_nav
