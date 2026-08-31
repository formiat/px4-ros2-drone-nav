#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>

#include "execution_publication_navigation_rebase_3d.hpp"
#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::shared_ptr<const VersionedExecutionInput3D>
advanceExecutionInput(const VersionedExecutionInput3D& source,
                      const std::int64_t publication_now_ns,
                      std::optional<MotionState3D> state = std::nullopt,
                      const bool refresh_receive_witness = false,
                      std::optional<MotionControl3D> previous_control = std::nullopt) {
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
  const std::shared_ptr<const ExecutionPlan3D> initial =
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
  ASSERT_TRUE(candidate.next->finiteExecution() != nullptr);
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const FiniteExecutionState3D& candidate_owner = candidate.next->finiteExecution()[0];
  ASSERT_NE(candidate_owner.execution_input, nullptr);
  ASSERT_NE(candidate_owner.latest_lidar_evidence, nullptr);
  ASSERT_NE(candidate_owner.observed_raw_world, nullptr);
  ASSERT_LT(candidate_owner.execution_input->captureSequence(),
            std::numeric_limits<std::uint64_t>::max());

  const std::int64_t publication_now_ns = candidate_owner.valid_from_ns + 20'000'000;
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(*candidate_owner.execution_input, publication_now_ns);
  ASSERT_NE(current_input, nullptr);
  FiniteMotionHorizonConfig3D finite_horizon_config;

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
      << finiteExecutionPathStatus3DName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
  EXPECT_EQ(result.path_validation_status, FiniteExecutionPathStatus3D::kValid);
  EXPECT_EQ(result.route_certification_status,
            FiniteExecutionCertificationStatus3D::kCertified);
  EXPECT_EQ(result.route_adherence_status,
            FiniteExecutionRouteAdherenceStatus3D::kAccepted);
  EXPECT_EQ(result.transition_status, ExecutionRouteTransitionStatus3D::kApplied);
  ASSERT_TRUE(result.transition.has_value());
  const ExecutionRouteTransitionResult3D& transition =
      result.transition.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_NE(transition.next, nullptr);
  ASSERT_TRUE(transition.next->finiteExecution() != nullptr);
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const FiniteExecutionState3D& rebased = transition.next->finiteExecution()[0];
  EXPECT_EQ(rebased.execution_input, current_input);
  EXPECT_EQ(rebased.valid_from_ns, publication_now_ns);
  ASSERT_NE(rebased.horizon, nullptr);
  EXPECT_EQ(rebased.horizon->states.front().x, current_input->state().x);
  EXPECT_TRUE(finiteMotionHorizonHasTerminalRestState3D(*rebased.horizon));
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     RecertifiesPendingRouteHandoffFromTheCurrentNavigationInput) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  const CertifiedRouteSuffix3D& active_route =
      *active->route(); // NOLINT(bugprone-unchecked-optional-access)

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = active->routeGenerationHighWater() + 1U;
  successor_activation =
      rebindUnconstrainedDecorations(std::move(successor_activation));
  ++successor_activation.proposal.objective.mission_epoch;
  successor_activation.proposal.intent.id =
      makeRouteIntentId3D(successor_activation.proposal.intent.mission_target,
                          successor_activation.proposal.objective.mission_epoch);
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
      .base_geometry_revision = active_route.geometry->compiled_trajectory_revision,
      .base_continuity_id = active_route.continuity_id,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = std::nullopt,
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
  ASSERT_TRUE(candidate.next->finiteExecution() != nullptr);
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const auto& candidate_owner = candidate.next->finiteExecution()[0];
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
  FiniteMotionHorizonConfig3D finite_horizon_config;

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
      << finiteExecutionPathStatus3DName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
  EXPECT_EQ(result.transition_status, ExecutionRouteTransitionStatus3D::kApplied);
  ASSERT_TRUE(result.transition.has_value());
  const ExecutionRouteTransitionResult3D& transition =
      result.transition.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_NE(transition.next, nullptr);
  ASSERT_TRUE(transition.next->route() != nullptr);
  ASSERT_TRUE(transition.next->finiteExecution() != nullptr);
  const ExecutionPlan3D& rebased_snapshot = *transition.next;
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const auto& rebased_route = rebased_snapshot.route()[0];
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  EXPECT_EQ(rebased_snapshot.route()->route_instance_id,
            successor_route.route_instance_id);
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  EXPECT_EQ(rebased_snapshot.finiteExecution()->execution_input, current_input);
  EXPECT_GT(rebased_route.progress.station_m,
            successor_route.progress.station_m + 0.25);
  EXPECT_EQ(rebased_route.progress.execution_input, current_input);
  EXPECT_GT(transition.next->execution_owner_epoch, active->execution_owner_epoch);
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     MotionUnderAnotherOwnerDoesNotConsumeUnpublishedControls) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> initial =
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
  ASSERT_TRUE(candidate.next->finiteExecution() != nullptr);
  const FiniteExecutionState3D& candidate_owner =
      candidate.next
          ->finiteExecution()[0]; // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_NE(candidate_owner.horizon, nullptr);
  ASSERT_NE(candidate_owner.execution_input, nullptr);
  ASSERT_NE(candidate_owner.latest_lidar_evidence, nullptr);
  ASSERT_NE(candidate_owner.observed_raw_world, nullptr);

  const std::int64_t publication_now_ns =
      candidate_owner.valid_from_ns + 1'500'000'000LL;
  constexpr std::size_t kSpatiallyMatchingControlIndex{10U};
  ASSERT_GT(candidate_owner.horizon->controls.size(), kSpatiallyMatchingControlIndex);
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(
          *candidate_owner.execution_input, publication_now_ns,
          candidate_owner.horizon->states[kSpatiallyMatchingControlIndex], true,
          candidate_owner.horizon->controls[kSpatiallyMatchingControlIndex - 1U]);
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
  FiniteMotionHorizonConfig3D finite_horizon_config;

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
      << finiteExecutionPathStatus3DName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
  ASSERT_TRUE(result.transition.has_value());
  const ExecutionRouteTransitionResult3D& transition =
      result.transition.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_NE(transition.next, nullptr);
  ASSERT_TRUE(transition.next->finiteExecution() != nullptr);
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const FiniteExecutionState3D& rebased = transition.next->finiteExecution()[0];
  ASSERT_NE(rebased.horizon, nullptr);
  EXPECT_EQ(result.source_control_index, 0U);
  EXPECT_EQ(rebased.horizon->controls.size(), candidate_owner.horizon->controls.size());
  EXPECT_EQ(rebased.valid_from_ns, publication_now_ns);
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     RetainedCandidateContinuesThePublishedExecutionClock) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->finiteExecution() != nullptr);
  const FiniteExecutionState3D& active_execution = active->finiteExecution()[0];
  ASSERT_NE(active_execution.horizon, nullptr);
  ASSERT_GT(active_execution.horizon->controls.size(), 5U);

  const ExecutionRouteTransitionResult3D candidate =
      replaceFiniteExecution3D(*active, SnapshotFixture3D::guard(*active),
                               SnapshotFixture3D::finiteExecution(
                                   *active, FiniteExecutionKind3D::kRetained, true,
                                   active_execution.trajectory_revision + 1U));
  ASSERT_TRUE(candidate.applied());
  ASSERT_NE(candidate.next, nullptr);
  ASSERT_TRUE(candidate.next->finiteExecution() != nullptr);
  const FiniteExecutionState3D& candidate_execution =
      candidate.next->finiteExecution()[0];
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
  FiniteMotionHorizonConfig3D finite_horizon_config;

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
      << finiteExecutionPathStatus3DName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
  ASSERT_TRUE(result.transition.has_value());
  const auto& transition =
      result.transition.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_NE(transition.next, nullptr);
  ASSERT_TRUE(transition.next->finiteExecution() != nullptr);
  const FiniteExecutionState3D& rebased =
      transition.next->finiteExecution()[0]; // NOLINT
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
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(active->finiteExecution() != nullptr);

  const ExecutionRouteTransitionResult3D candidate = replaceFiniteExecution3D(
      *active, SnapshotFixture3D::guard(*active),
      SnapshotFixture3D::finiteExecution(*active, FiniteExecutionKind3D::kNominal, true,
                                         101U));
  ASSERT_TRUE(candidate.applied());
  ASSERT_NE(candidate.next, nullptr);
  ASSERT_TRUE(candidate.next->finiteExecution() != nullptr);
  const FiniteExecutionState3D& candidate_owner =
      candidate
          .next // NOLINT(bugprone-unchecked-optional-access)
          ->finiteExecution()[0];
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
  FiniteMotionHorizonConfig3D finite_horizon_config;

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
      << finiteExecutionPathStatus3DName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
  EXPECT_EQ(result.path_validation_status, FiniteExecutionPathStatus3D::kValid);
  EXPECT_EQ(result.route_certification_status,
            FiniteExecutionCertificationStatus3D::kCertified);
  EXPECT_EQ(result.route_adherence_status,
            FiniteExecutionRouteAdherenceStatus3D::kAccepted);
  EXPECT_EQ(result.transition_status, ExecutionRouteTransitionStatus3D::kApplied);
  ASSERT_TRUE(result.transition.has_value());
  const auto& transition =
      result.transition.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_NE(transition.next, nullptr);
  ASSERT_TRUE(transition.next->route() != nullptr);
  ASSERT_TRUE(transition.next->finiteExecution() != nullptr);
  const ExecutionPlan3D& rebased_snapshot = *transition.next;
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const auto& rebased_route = rebased_snapshot.route()[0];
  const CertifiedRouteSuffix3D& active_route =
      *active->route(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_EQ(rebased_route.identity.generation, active_route.identity.generation);
  const auto& rebased_execution = rebased_snapshot.finiteExecution()[0]; // NOLINT
  EXPECT_EQ(rebased_execution.execution_input, current_input);
  EXPECT_GT(rebased_route.progress.station_m, active_route.progress.station_m + 0.25);
  EXPECT_EQ(rebased_route.progress.execution_input, current_input);
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     PreparedProgressAndRawCertificateSurviveLateRebaseComposition) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> resident = fixture.activeSnapshot();
  ASSERT_NE(resident, nullptr);
  ASSERT_TRUE(resident->route() != nullptr);
  constexpr std::uint64_t kAdvancedRawRevision =
      SnapshotFixture3D::kLatestRawRevision + 1U;
  const ExecutionRouteTransitionResult3D progress = advanceCertifiedRoute3D(
      *resident, SnapshotFixture3D::guard(*resident),
      fixture.executionObservation({4.0, 0.0, 5.0}, kAdvancedRawRevision,
                                   &fixture.raw_occupancy),
      SnapshotFixture3D::progressInput(*resident, {4.0, 0.0, 5.0}),
      fixture.rawWorld(kAdvancedRawRevision));
  ASSERT_TRUE(progress.applied());
  ASSERT_NE(progress.next, nullptr);
  ASSERT_TRUE(progress.next->route() != nullptr);
  ASSERT_NE(progress.next->route()->observed_raw_world, nullptr);
  ASSERT_EQ(progress.next->route()->observed_raw_world->version().revision,
            kAdvancedRawRevision);
  ASSERT_FALSE(progress.next->publishable());

  const ExecutionRouteTransitionResult3D prepared = replaceFiniteExecutionPlan3D(
      *progress.next, SnapshotFixture3D::guard(*progress.next),
      SnapshotFixture3D::finitePlanForRoute(*progress.next, *progress.next->route(),
                                            FiniteExecutionKind3D::kNominal, 101U));
  ASSERT_TRUE(prepared.applied());
  const ExecutionRouteTransitionResult3D candidate =
      composeExecutionPlanTransition3D(*resident, progress, prepared);
  ASSERT_TRUE(candidate.applied());
  ASSERT_NE(candidate.next, nullptr);
  ASSERT_TRUE(candidate.next->finiteExecution() != nullptr);
  const FiniteExecutionState3D& candidate_execution =
      *candidate.next->finiteExecution();
  ASSERT_NE(candidate_execution.execution_input, nullptr);
  ASSERT_NE(candidate_execution.latest_lidar_evidence, nullptr);
  ASSERT_NE(candidate_execution.observed_raw_world, nullptr);
  const std::int64_t publication_now_ns =
      candidate_execution.valid_from_ns + 20'000'000LL;
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(*candidate_execution.execution_input, publication_now_ns);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      advanceLidarEvidence(*candidate_execution.latest_lidar_evidence,
                           publication_now_ns);
  ASSERT_NE(current_input, nullptr);
  ASSERT_NE(current_lidar, nullptr);
  FiniteMotionHorizonConfig3D finite_horizon_config;

  const ExecutionPublicationNavigationRebaseResult3D result =
      rebaseExecutionPublicationForCurrentNavigation3D(
          ExecutionPublicationNavigationRebaseRequest3D{
              .expected_snapshot = resident.get(),
              .certification_snapshot = progress.next.get(),
              .progress_preparation = &progress,
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
      << finiteExecutionPathStatus3DName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
  ASSERT_TRUE(result.transition.has_value());
  const ExecutionRouteTransitionResult3D& rebased = *result.transition;
  ASSERT_NE(rebased.next, nullptr);
  ASSERT_TRUE(rebased.next->route() != nullptr);
  ASSERT_TRUE(rebased.next->finiteExecution() != nullptr);
  ASSERT_TRUE(rebased.next->brakingFallback() != nullptr);
  EXPECT_EQ(rebased.predecessor, resident.get());
  EXPECT_EQ(rebased.next->version, resident->version + 2U);
  EXPECT_TRUE(rebased.next->publishable());
  EXPECT_EQ(rebased.next->route()->progress.execution_input, current_input);
  EXPECT_EQ(rebased.next->route()->observed_raw_world,
            progress.next->route()->observed_raw_world);
  EXPECT_EQ(rebased.next->route()->observed_raw_world->version().revision,
            kAdvancedRawRevision);
  EXPECT_EQ(rebased.next->finiteExecution()->execution_input, current_input);
  EXPECT_EQ(rebased.next->brakingFallback()->execution_input, current_input);
  EXPECT_EQ(rebased.next->finiteExecution()->observed_raw_world,
            progress.next->route()->observed_raw_world);
  EXPECT_EQ(rebased.next->brakingFallback()->observed_raw_world,
            progress.next->route()->observed_raw_world);
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     LifecycleBrakingOwnerSurvivesTheFinalNavigationRace) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);
  ASSERT_TRUE(active->route() != nullptr);
  ASSERT_TRUE(active->finiteExecution() != nullptr);
  FiniteExecutionCertification3D braking_certification =
      SnapshotFixture3D::finiteCertificationForRoute(
          *active->route(), FiniteExecutionKind3D::kEmergencyBrakeTail, 101U, 56U);
  ASSERT_GT(braking_certification.horizon.controls.size(), 1U);
  braking_certification.horizon.controls.pop_back();
  braking_certification.horizon.states.pop_back();
  --braking_certification.horizon.arrival_control_count;
  const RouteLifecycleEvent3D superseded{
      .kind = RouteLifecycleEventKind3D::kObjectiveSuperseded,
      .generation = active->route()->identity.generation,
  };
  const FiniteExecutionCertificationResult3D certified =
      certifyLifecycleBrakingFiniteExecution3DDetailed(
          *active, LifecycleBrakingFiniteExecutionCertification3D{
                       .lifecycle_event = superseded,
                       .finite_execution = std::move(braking_certification),
                   });
  ASSERT_TRUE(certified.certified())
      << finiteExecutionCertificationStatus3DName(certified.status);
  const ExecutionRouteTransitionResult3D candidate = retireCertifiedRoute3D(
      *active, SnapshotFixture3D::guard(*active), superseded, certified.execution);
  ASSERT_TRUE(candidate.applied())
      << executionRouteTransitionStatus3DName(candidate.status);
  ASSERT_NE(candidate.next, nullptr);
  ASSERT_TRUE(candidate.next->finiteExecution() != nullptr);
  ASSERT_TRUE(candidate.next->brakingFallback() != nullptr);
  const FiniteExecutionState3D& candidate_execution =
      *candidate.next->finiteExecution();
  ASSERT_NE(candidate_execution.execution_input, nullptr);
  ASSERT_NE(candidate_execution.latest_lidar_evidence, nullptr);
  ASSERT_NE(candidate_execution.observed_raw_world, nullptr);
  const std::int64_t publication_now_ns =
      candidate_execution.valid_from_ns + 20'000'000LL;
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(*candidate_execution.execution_input, publication_now_ns,
                            std::nullopt, true);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      advanceLidarEvidence(*candidate_execution.latest_lidar_evidence,
                           publication_now_ns);
  ASSERT_NE(current_input, nullptr);
  ASSERT_NE(current_lidar, nullptr);
  FiniteMotionHorizonConfig3D finite_horizon_config;

  const ExecutionPublicationNavigationRebaseResult3D result =
      rebaseExecutionPublicationForCurrentNavigation3D(
          ExecutionPublicationNavigationRebaseRequest3D{
              .expected_snapshot = active.get(),
              .candidate_snapshot = candidate.next.get(),
              .lifecycle_event = &superseded,
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
      << finiteExecutionPathStatus3DName(result.path_validation_status) << ' '
      << finiteExecutionCertificationStatus3DName(result.route_certification_status)
      << ' ' << executionRouteTransitionStatus3DName(result.transition_status);
  ASSERT_TRUE(result.transition.has_value());
  ASSERT_NE(result.transition->next, nullptr);
  const ExecutionPlan3D& rebased = *result.transition->next;
  ASSERT_TRUE(rebased.finiteExecution() != nullptr);
  ASSERT_TRUE(rebased.brakingFallback() != nullptr);
  EXPECT_EQ(rebased.phase(), ExecutionRoutePhase3D::kBraking);
  EXPECT_EQ(rebased.finiteExecution()->kind,
            FiniteExecutionKind3D::kEmergencyBrakeTail);
  EXPECT_EQ(rebased.finiteExecution()->execution_input, current_input);
  EXPECT_EQ(rebased.brakingFallback()->execution_input, current_input);
  EXPECT_LE(rebased.finiteExecution()->valid_until_ns,
            candidate_execution.valid_until_ns);
  EXPECT_EQ(rebased.finiteExecution()->validation_proof.artifact_fingerprint,
            rebased.brakingFallback()->validation_proof.artifact_fingerprint);
}

TEST(ExecutionPublicationNavigationRebase3DTest,
     ReportsCertificationThatRejectsEveryPhysicalCandidate) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> initial =
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
          ->finiteExecution()[0];
  ASSERT_NE(candidate_owner.execution_input, nullptr);
  ASSERT_NE(candidate_owner.latest_lidar_evidence, nullptr);
  ASSERT_NE(candidate_owner.observed_raw_world, nullptr);
  const std::int64_t publication_now_ns = candidate_owner.valid_from_ns + 20'000'000;
  MotionState3D off_route_state = candidate_owner.execution_input->state();
  off_route_state.y = 3.0F;
  const std::shared_ptr<const VersionedExecutionInput3D> current_input =
      advanceExecutionInput(*candidate_owner.execution_input, publication_now_ns,
                            off_route_state);
  ASSERT_NE(current_input, nullptr);
  FiniteMotionHorizonConfig3D finite_horizon_config;

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
            FiniteExecutionPathStatus3D::kCandidateRejected);
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
