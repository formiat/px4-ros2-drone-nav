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
                      std::optional<mppi::State> state = std::nullopt,
                      const bool refresh_receive_witness = false,
                      std::optional<mppi::Control> previous_control = std::nullopt) {
  const std::int64_t receive_stamp_ns = refresh_receive_witness
                                            ? publication_now_ns - 5'000'000LL
                                            : source.poseReceiveStampNs() + 10'000;
  const std::int64_t control_source_stamp_ns =
      refresh_receive_witness ? publication_now_ns - 10'000'000LL
                              : source.previousControlSourceStampNs() + 10'000;
  const std::int64_t control_receive_stamp_ns =
      refresh_receive_witness ? publication_now_ns - 5'000'000LL
                              : source.previousControlReceiveStampNs() + 10'000;
  return VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
      .capture_sequence = source.captureSequence() + 1U,
      .pose_revision = source.poseRevision() + 1U,
      .pose_source_timestamp_us = source.poseSourceTimestampUs() + 1U,
      .pose_receive_stamp_ns = receive_stamp_ns,
      .effective_stamp_ns = publication_now_ns,
      .state = state.value_or(source.state()),
      .full_state_authoritative = source.fullStateAuthoritative(),
      .state_provenance = source.stateProvenance(),
      .previous_control = previous_control.value_or(source.previousControl()),
      .previous_control_source = source.previousControlSource(),
      .previous_control_source_producer_instance_id =
          source.previousControlSourceProducerInstanceId(),
      .previous_control_source_sequence = source.previousControlSourceSequence() + 1U,
      .previous_control_source_stamp_ns = control_source_stamp_ns,
      .previous_control_receive_stamp_ns = control_receive_stamp_ns,
  });
}

[[nodiscard]] std::shared_ptr<const VersionedLatestLidarEvidence3D>
advanceLidarEvidence(const VersionedLatestLidarEvidence3D& source,
                     const std::int64_t publication_now_ns) {
  return VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
      .producer_instance_id = source.producerInstanceId(),
      .sequence = source.sequence() + 1U,
      .pose_generation = source.poseGeneration() + 1U,
      .acquisition_stamp_ns = publication_now_ns - 15'000'000LL,
      .receive_stamp_ns = publication_now_ns - 5'000'000LL,
      .source_beam_count = source.sourceBeamCount(),
      .hit_points_map_m = source.hitPointsMapM(),
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
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const FiniteExecutionState3D& candidate_owner =
      candidate.next->finite_execution.value();
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

  ASSERT_TRUE(result.rebased())
      << executionPublicationNavigationRebaseStatus3DName(result.status) << ' '
      << mppi::finiteExecutionPathStatusName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
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
     RecertifiesPendingRouteHandoffFromTheCurrentNavigationInput) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route.has_value());
  const CertifiedRouteSuffix3D& active_route =
      active->route.value(); // NOLINT(bugprone-unchecked-optional-access)

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = active->routeGenerationHighWater() + 1U;
  ++successor_activation.proposal.objective.mission_epoch;
  successor_activation.proposal.intent.strategic_plan_id =
      successor_activation.proposal.objective.mission_epoch;
  successor_activation.observation.current_objective =
      successor_activation.proposal.objective;
  successor_activation.continuity_lineage.mission_epoch =
      successor_activation.proposal.objective.mission_epoch;
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const CertifiedRouteSuffix3D& successor_route =
      successor.value(); // NOLINT(bugprone-unchecked-optional-access)
  const PendingCertifiedRoute3D pending{
      .publication_sequence = 1U,
      .base_execution_owner_epoch = active->execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kRouteHandoff,
      .base_route_generation = active_route.identity.generation,
      .base_geometry_revision = active_route.geometry->executable_geometry_revision,
      .base_continuity_id = active_route.continuity_id,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = std::nullopt,
      .strategy_decision = std::nullopt,
      .topology_effect = {},
      .route = successor_route,
  };
  ASSERT_TRUE(pending.valid());

  FiniteExecutionState3D candidate_execution =
      SnapshotFixture3D::finiteExecutionForRoute(
          *active, successor_route, FiniteExecutionKind3D::kNominal, true, 101U);
  const ExecutionRouteTransitionResult3D candidate =
      replaceCertifiedRouteAtHandoff3D(*active, SnapshotFixture3D::guard(*active),
                                       successor_route, std::move(candidate_execution));
  ASSERT_TRUE(candidate.applied());
  ASSERT_NE(candidate.next, nullptr);
  ASSERT_TRUE(candidate.next->finite_execution.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const auto& candidate_owner = candidate.next->finite_execution.value();
  ASSERT_NE(candidate_owner.execution_input, nullptr);
  ASSERT_NE(candidate_owner.latest_lidar_evidence, nullptr);
  ASSERT_NE(candidate_owner.observed_raw_world, nullptr);
  ASSERT_NE(candidate_owner.horizon, nullptr);
  constexpr std::size_t kProgressControlIndex{20U};
  ASSERT_GT(candidate_owner.horizon->controls.size(), kProgressControlIndex);

  const std::int64_t publication_now_ns =
      candidate_owner.valid_from_ns + 2'500'000'000LL;
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(
          *candidate_owner.execution_input, publication_now_ns,
          candidate_owner.horizon->states[kProgressControlIndex], true,
          candidate_owner.horizon->controls[kProgressControlIndex - 1U]);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      advanceLidarEvidence(*candidate_owner.latest_lidar_evidence, publication_now_ns);
  ASSERT_NE(current_input, nullptr);
  ASSERT_NE(current_lidar, nullptr);
  mppi::FiniteHorizonConfig finite_horizon_config;

  const ExecutionPublicationNavigationRebaseResult3D result =
      rebaseExecutionPublicationForCurrentNavigation3D(
          ExecutionPublicationNavigationRebaseRequest3D{
              .expected_snapshot = active.get(),
              .candidate_snapshot = candidate.next.get(),
              .expected_pending = &pending,
              .current_execution_input = current_input,
              .current_lidar_evidence = current_lidar,
              .current_observed_raw_world = candidate_owner.observed_raw_world,
              .publication_now_ns = publication_now_ns,
              .arrival_search_step_controls = 5U,
              .finite_horizon_config = &finite_horizon_config,
              .terminal_boundary = std::nullopt,
          });

  ASSERT_TRUE(result.rebased())
      << executionPublicationNavigationRebaseStatus3DName(result.status) << ' '
      << mppi::finiteExecutionPathStatusName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
  EXPECT_EQ(result.transition_status, ExecutionRouteTransitionStatus3D::kApplied);
  ASSERT_TRUE(result.transition.has_value());
  const ExecutionRouteTransitionResult3D& transition =
      result.transition.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_NE(transition.next, nullptr);
  ASSERT_TRUE(transition.next->route.has_value());
  ASSERT_TRUE(transition.next->finite_execution.has_value());
  const ExecutionRouteSnapshot3D& rebased_snapshot = *transition.next;
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const auto& rebased_route = rebased_snapshot.route.value();
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  EXPECT_EQ(rebased_snapshot.route->route_instance_id,
            successor_route.route_instance_id);
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  EXPECT_EQ(rebased_snapshot.finite_execution->execution_input, current_input);
  EXPECT_GT(rebased_route.progress.station_m,
            successor_route.progress.station_m + 0.25);
  EXPECT_EQ(rebased_route.progress.execution_input, current_input);
  EXPECT_GT(transition.next->execution_owner_epoch, active->execution_owner_epoch);
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     PlanningLatencyDoesNotConsumeUnpublishedControls) {
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
      candidate.next
          ->finite_execution // NOLINT(bugprone-unchecked-optional-access)
          .value();
  ASSERT_NE(candidate_owner.horizon, nullptr);
  ASSERT_NE(candidate_owner.execution_input, nullptr);
  ASSERT_NE(candidate_owner.latest_lidar_evidence, nullptr);
  ASSERT_NE(candidate_owner.observed_raw_world, nullptr);

  const std::int64_t publication_now_ns =
      candidate_owner.valid_from_ns + 1'500'000'000LL;
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(*candidate_owner.execution_input, publication_now_ns,
                            std::nullopt, true);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
          .producer_instance_id =
              candidate_owner.latest_lidar_evidence->producerInstanceId(),
          .sequence = candidate_owner.latest_lidar_evidence->sequence() + 1U,
          .pose_generation =
              candidate_owner.latest_lidar_evidence->poseGeneration() + 1U,
          .acquisition_stamp_ns = publication_now_ns - 15'000'000LL,
          .receive_stamp_ns = publication_now_ns - 5'000'000LL,
          .source_beam_count = 1U,
          .hit_points_map_m = {},
      });
  ASSERT_NE(current_input, nullptr);
  ASSERT_NE(current_lidar, nullptr);
  mppi::FiniteHorizonConfig finite_horizon_config;

  const ExecutionPublicationNavigationRebaseResult3D result =
      rebaseExecutionPublicationForCurrentNavigation3D(
          ExecutionPublicationNavigationRebaseRequest3D{
              .expected_snapshot = initial.get(),
              .candidate_snapshot = candidate.next.get(),
              .current_execution_input = current_input,
              .current_lidar_evidence = current_lidar,
              .current_observed_raw_world = candidate_owner.observed_raw_world,
              .publication_now_ns = publication_now_ns,
              .arrival_search_step_controls = 5U,
              .finite_horizon_config = &finite_horizon_config,
              .terminal_boundary = std::nullopt,
          });

  ASSERT_TRUE(result.rebased())
      << executionPublicationNavigationRebaseStatus3DName(result.status) << ' '
      << mppi::finiteExecutionPathStatusName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
  ASSERT_TRUE(result.transition.has_value());
  const ExecutionRouteTransitionResult3D& transition =
      result.transition.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_NE(transition.next, nullptr);
  ASSERT_TRUE(transition.next->finite_execution.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const FiniteExecutionState3D& rebased = transition.next->finite_execution.value();
  ASSERT_NE(rebased.horizon, nullptr);
  EXPECT_EQ(rebased.horizon->controls.size(), candidate_owner.horizon->controls.size());
  EXPECT_EQ(rebased.valid_from_ns, publication_now_ns);
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     PhysicalProgressAlignsAnUnpublishedCandidateToItsControlSuffix) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->finite_execution.has_value());
  const FiniteExecutionState3D& execution = active->finite_execution.value();
  ASSERT_NE(execution.horizon, nullptr);
  ASSERT_GT(execution.horizon->controls.size(), 12U);
  ASSERT_NE(execution.validation_policy, nullptr);

  constexpr std::size_t kExpectedControlIndex{10U};
  const mppi::State current_state = execution.horizon->states[kExpectedControlIndex];
  EXPECT_EQ(closestFiniteExecutionRebaseControlIndex3D(
                *execution.horizon, current_state,
                execution.validation_policy->dynamics().dt_s),
            kExpectedControlIndex);
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     RetainedCandidateContinuesThePublishedExecutionClock) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionRouteSnapshot3D> active =
      fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->finite_execution.has_value());
  const FiniteExecutionState3D& active_execution = active->finite_execution.value();
  ASSERT_NE(active_execution.horizon, nullptr);
  ASSERT_GT(active_execution.horizon->controls.size(), 5U);

  const ExecutionRouteTransitionResult3D candidate =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(
                                   *active, FiniteExecutionKind3D::kRetained, true,
                                   active_execution.trajectory_revision + 1U));
  ASSERT_TRUE(candidate.applied());
  ASSERT_NE(candidate.next, nullptr);
  ASSERT_TRUE(candidate.next->finite_execution.has_value());
  const FiniteExecutionState3D& candidate_execution =
      candidate.next->finite_execution.value();
  ASSERT_NE(candidate_execution.execution_input, nullptr);
  ASSERT_NE(candidate_execution.latest_lidar_evidence, nullptr);
  ASSERT_NE(candidate_execution.observed_raw_world, nullptr);

  constexpr std::size_t kExecutedControlCount{5U};
  const std::int64_t publication_now_ns =
      active_execution.valid_from_ns + 500'000'000LL;
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(
          *candidate_execution.execution_input, publication_now_ns,
          active_execution.horizon->states[kExecutedControlCount], true,
          active_execution.horizon->controls[kExecutedControlCount - 1U]);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
          .producer_instance_id =
              candidate_execution.latest_lidar_evidence->producerInstanceId(),
          .sequence = candidate_execution.latest_lidar_evidence->sequence() + 1U,
          .pose_generation =
              candidate_execution.latest_lidar_evidence->poseGeneration() + 1U,
          .acquisition_stamp_ns = publication_now_ns - 15'000'000LL,
          .receive_stamp_ns = publication_now_ns - 5'000'000LL,
          .source_beam_count = 1U,
          .hit_points_map_m = {},
      });
  ASSERT_NE(current_input, nullptr);
  ASSERT_NE(current_lidar, nullptr);
  mppi::FiniteHorizonConfig finite_horizon_config;

  const ExecutionPublicationNavigationRebaseResult3D result =
      rebaseExecutionPublicationForCurrentNavigation3D(
          ExecutionPublicationNavigationRebaseRequest3D{
              .expected_snapshot = active.get(),
              .candidate_snapshot = candidate.next.get(),
              .current_execution_input = current_input,
              .current_lidar_evidence = current_lidar,
              .current_observed_raw_world = candidate_execution.observed_raw_world,
              .publication_now_ns = publication_now_ns,
              .arrival_search_step_controls = 5U,
              .finite_horizon_config = &finite_horizon_config,
              .terminal_boundary = std::nullopt,
          });

  ASSERT_TRUE(result.rebased())
      << executionPublicationNavigationRebaseStatus3DName(result.status) << ' '
      << mppi::finiteExecutionPathStatusName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
  ASSERT_TRUE(result.transition.has_value());
  const auto& transition =
      result.transition.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_NE(transition.next, nullptr);
  ASSERT_TRUE(transition.next->finite_execution.has_value());
  const FiniteExecutionState3D& rebased =
      transition.next->finite_execution.value(); // NOLINT
  ASSERT_NE(rebased.horizon, nullptr);
  EXPECT_EQ(rebased.horizon->nominal_prefix_control_count,
            active_execution.horizon->nominal_prefix_control_count -
                kExecutedControlCount);
  EXPECT_EQ(rebased.valid_from_ns, publication_now_ns);
  EXPECT_LE(rebased.valid_until_ns, active_execution.valid_until_ns);
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
  ASSERT_NE(candidate_owner.horizon, nullptr);
  constexpr std::size_t kProgressControlIndex{20U};
  ASSERT_GT(candidate_owner.horizon->controls.size(), kProgressControlIndex);

  const std::int64_t publication_now_ns =
      candidate_owner.valid_from_ns + 2'500'000'000LL;
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(
          *candidate_owner.execution_input, publication_now_ns,
          candidate_owner.horizon->states[kProgressControlIndex], true,
          candidate_owner.horizon->controls[kProgressControlIndex - 1U]);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      advanceLidarEvidence(*candidate_owner.latest_lidar_evidence, publication_now_ns);
  ASSERT_NE(current_input, nullptr);
  ASSERT_NE(current_lidar, nullptr);
  mppi::FiniteHorizonConfig finite_horizon_config;

  const ExecutionPublicationNavigationRebaseResult3D result =
      rebaseExecutionPublicationForCurrentNavigation3D(
          ExecutionPublicationNavigationRebaseRequest3D{
              .expected_snapshot = active.get(),
              .candidate_snapshot = candidate.next.get(),
              .expected_pending = nullptr,
              .current_execution_input = current_input,
              .current_lidar_evidence = current_lidar,
              .current_observed_raw_world = candidate_owner.observed_raw_world,
              .publication_now_ns = publication_now_ns,
              .arrival_search_step_controls = 5U,
              .finite_horizon_config = &finite_horizon_config,
              .terminal_boundary = std::nullopt,
          });

  ASSERT_TRUE(result.rebased())
      << executionPublicationNavigationRebaseStatus3DName(result.status) << ' '
      << mppi::finiteExecutionPathStatusName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
  EXPECT_EQ(result.path_validation_status, mppi::FiniteExecutionPathStatus::kValid);
  EXPECT_EQ(result.route_certification_status,
            FiniteExecutionCertificationStatus3D::kCertified);
  EXPECT_EQ(result.route_adherence_status,
            FiniteExecutionRouteAdherenceStatus3D::kAccepted);
  EXPECT_EQ(result.transition_status, ExecutionRouteTransitionStatus3D::kApplied);
  ASSERT_TRUE(result.transition.has_value());
  const auto& transition =
      result.transition.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_NE(transition.next, nullptr);
  ASSERT_TRUE(transition.next->route.has_value());
  ASSERT_TRUE(transition.next->finite_execution.has_value());
  const ExecutionRouteSnapshot3D& rebased_snapshot = *transition.next;
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const auto& rebased_route = rebased_snapshot.route.value();
  const CertifiedRouteSuffix3D& active_route =
      active->route.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_EQ(rebased_route.identity.generation, active_route.identity.generation);
  const auto& rebased_execution = rebased_snapshot.finite_execution.value(); // NOLINT
  EXPECT_EQ(rebased_execution.execution_input, current_input);
  EXPECT_GT(rebased_route.progress.station_m, active_route.progress.station_m + 0.25);
  EXPECT_EQ(rebased_route.progress.execution_input, current_input);
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
