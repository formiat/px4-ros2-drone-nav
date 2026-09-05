#include "drone_city_nav/execution_stop_3d.hpp"
#include "drone_city_nav/execution_supervisor_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "execution_route_snapshot_3d_plan_test_support.hpp"
#include "execution_supervisor_horizon_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

constexpr std::int64_t kEvidenceStepNs{10'000LL};

[[nodiscard]] std::shared_ptr<const ExecutionPlan3D>
installRouteOwner(ExecutionSupervisor3D& supervisor, SnapshotFixture3D& fixture) {
  const std::optional<CertifiedRouteSuffix3D> certified = fixture.certify();
  if (!certified.has_value()) {
    return nullptr;
  }
  const std::shared_ptr<const CommittedExecutionAuthority3D> initial_authority =
      supervisor.authority();
  const std::shared_ptr<const ExecutionPlan3D> initial =
      initial_authority != nullptr ? initial_authority->plan() : nullptr;
  if (initial == nullptr) {
    return nullptr;
  }
  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *initial, initial->version, *certified,
      SnapshotFixture3D::finitePlanForRoute(*initial, *certified,
                                            FiniteExecutionKind3D::kNominal, 100U));
  if (!activation.applied() || activation.next == nullptr) {
    return nullptr;
  }
  const ExecutionHorizonCommitResult3D publication = commitExecutionHorizonForTest(
      supervisor, ExecutionHorizonTestTransaction3D{
                      .kind = ExecutionHorizonCommitKind3D::kTransition,
                      .expected_authority = initial_authority,
                      .expected_plan = initial,
                      .transition = activation,
                      .expected_pending = nullptr,
                      .owner = SnapshotFixture3D::committedOwner(*activation.next),
                      .input = SnapshotFixture3D::committedInput(*activation.next),
                  });
  return publication.committed() ? supervisor.plan() : nullptr;
}

// The vehicle observed a step later, moving along the route it was following.
[[nodiscard]] std::shared_ptr<const VersionedExecutionInput3D>
movingInput(const VersionedExecutionInput3D& previous, const float speed_mps) {
  MotionState3D state = previous.state();
  state.vx = speed_mps;
  constexpr ExecutionStateFieldProvenance3D kSample{
      ExecutionStateFieldProvenance3D::kSourceSample};
  return VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
      .capture_sequence = previous.captureSequence() + 1U,
      .pose_revision = previous.poseRevision() + 1U,
      .pose_source_timestamp_us = previous.poseSourceTimestampUs() + 1U,
      .pose_receive_stamp_ns = previous.poseReceiveStampNs() + kEvidenceStepNs,
      .effective_stamp_ns = previous.effectiveStampNs() + kEvidenceStepNs,
      .state = state,
      .full_state_authoritative = true,
      .state_provenance =
          ExecutionStateProvenance3D{
              .x = kSample,
              .y = kSample,
              .z = kSample,
              .vx = kSample,
              .vy = kSample,
              .vz = kSample,
              .yaw = kSample,
              .yaw_rate = kSample,
          },
      .previous_control = {},
      .previous_control_source =
          ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration,
      .previous_control_source_sequence = previous.previousControlSourceSequence() + 1U,
      .previous_control_source_stamp_ns =
          previous.previousControlSourceStampNs() + kEvidenceStepNs,
      .previous_control_receive_stamp_ns =
          previous.previousControlReceiveStampNs() + kEvidenceStepNs,
  });
}

[[nodiscard]] ExecutionStopRequest3D
stopRequest(const SnapshotFixture3D& fixture,
            const std::shared_ptr<const ExecutionPlan3D>& source,
            const std::shared_ptr<const VersionedExecutionInput3D>& input) {
  const FiniteExecutionState3D* const active =
      source != nullptr ? source->finiteExecution() : nullptr;
  const StopExecution3D* const resident =
      source != nullptr ? source->stopExecution() : nullptr;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> lidar_evidence;
  if (active != nullptr) {
    lidar_evidence = active->latest_lidar_evidence;
  } else if (resident != nullptr) {
    lidar_evidence = resident->latest_lidar_evidence;
  }
  return ExecutionStopRequest3D{
      .cycle_source_plan = source,
      .execution_input = input,
      .latest_lidar_evidence = std::move(lidar_evidence),
      .observed_raw_world = fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision),
      .static_world = nullptr,
      .validation_policy = fixture.validation_policy,
      .exact_initial_state = input->state(),
      .exact_previous_control = input->previousControl(),
      .finite_horizon_config = {},
      .maximum_control_count = 100U,
      .now_ns = input->effectiveStampNs(),
  };
}

TEST(ExecutionSupervisorStop3DTest, AStandingVehicleIsLeftToTheStationaryHold) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);

  const ExecutionStopPreparation3D prepared = supervisor.prepareStop(
      stopRequest(fixture, active, active->finiteExecution()->execution_input));

  // Nothing to stop: deriving a zero-motion stop every tick would only churn
  // the lease the stationary hold already owns.
  EXPECT_EQ(prepared.status, ExecutionStopStatus3D::kAtRest);
  EXPECT_FALSE(prepared.prepared());
  EXPECT_EQ(supervisor.plan(), active);
}

TEST(ExecutionSupervisorStop3DTest, AMovingVehicleStopsAlongAValidatedTrajectory) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      movingInput(*active->finiteExecution()->execution_input, 4.0F);

  const ExecutionStopPreparation3D prepared =
      supervisor.prepareStop(stopRequest(fixture, active, input));

  ASSERT_EQ(prepared.status, ExecutionStopStatus3D::kPrepared)
      << executionStopStatus3DName(prepared.status);
  ASSERT_TRUE(prepared.prepared());
  // Preparation is side-effect free: the resident plan is replaced only by the
  // commit that publishes the stop on the wire.
  EXPECT_EQ(supervisor.plan(), active);
  ASSERT_NE(prepared.transition, nullptr);
  ASSERT_NE(prepared.transition->next, nullptr);
  EXPECT_EQ(prepared.transition->next->phase(), ExecutionRoutePhase3D::kStopping);
  const StopExecution3D* const stop = prepared.stopExecution();
  ASSERT_NE(stop, nullptr);
  EXPECT_TRUE(stop->valid());
  EXPECT_NEAR(prepared.initial_speed_mps, 4.0, 1.0e-6);
  EXPECT_GT(prepared.stop_distance_m, 0.0);
  // A stop is route-free by construction and ends at rest.
  EXPECT_EQ(prepared.transition->next->route(), nullptr);
  EXPECT_EQ(prepared.transition->next->finiteExecution(), nullptr);
  EXPECT_EQ(prepared.transition->next->brakingFallback(), nullptr);
  ASSERT_NE(stop->horizon, nullptr);
  EXPECT_TRUE(finiteMotionHorizonHasTerminalRestState3D(*stop->horizon));
  EXPECT_GT(stop->trajectory_revision, active->finiteExecution()->trajectory_revision);
}

TEST(ExecutionSupervisorStop3DTest, TheCommittedStopOwnsTheVehicleWhileItIsExecutable) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      movingInput(*active->finiteExecution()->execution_input, 4.0F);
  const ExecutionStopPreparation3D prepared =
      supervisor.prepareStop(stopRequest(fixture, active, input));
  ASSERT_TRUE(prepared.prepared()) << executionStopStatus3DName(prepared.status);

  ASSERT_EQ(commitExecutionHorizonForTest(
                supervisor,
                ExecutionHorizonTestTransaction3D{
                    .kind = ExecutionHorizonCommitKind3D::kTransition,
                    .expected_authority = prepared.expected_authority,
                    .expected_plan = prepared.expectedPlan(),
                    .transition = *prepared.transition,
                    .expected_pending = nullptr,
                    .owner = SnapshotFixture3D::committedOwner(
                        *prepared.transition->next, 2U),
                    .input = input,
                })
                .status,
            ExecutionHorizonCommitStatus3D::kCommitted);
  const std::shared_ptr<const ExecutionPlan3D> stopping = supervisor.plan();
  ASSERT_NE(stopping, nullptr);
  ASSERT_NE(stopping->stopExecution(), nullptr);

  // The stop that is already bringing the vehicle to rest is the answer to the
  // next tick's request, not a reason to publish a second one.
  const ExecutionStopPreparation3D again =
      supervisor.prepareStop(stopRequest(fixture, stopping, movingInput(*input, 3.0F)));

  EXPECT_EQ(again.status, ExecutionStopStatus3D::kResidentStopCurrent);
  EXPECT_FALSE(again.prepared());
  EXPECT_EQ(supervisor.plan(), stopping);
}

TEST(ExecutionSupervisorStop3DTest, ACertifiedRouteTakesTheVehicleBackFromAStop) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      movingInput(*active->finiteExecution()->execution_input, 4.0F);
  const ExecutionStopPreparation3D prepared =
      supervisor.prepareStop(stopRequest(fixture, active, input));
  ASSERT_TRUE(prepared.prepared()) << executionStopStatus3DName(prepared.status);
  const std::shared_ptr<const ExecutionPlan3D> stopping = prepared.transition->next;
  ASSERT_NE(stopping, nullptr);
  ASSERT_NE(stopping->stopExecution(), nullptr);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = SnapshotFixture3D::kRouteGeneration + 1U;
  successor_activation =
      rebindUnconstrainedDecorations(std::move(successor_activation));
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const CertifiedRouteSuffix3D& successor_route =
      successor.value(); // NOLINT(bugprone-unchecked-optional-access)
  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *stopping, stopping->version, successor_route,
      SnapshotFixture3D::finitePlanForRoute(
          *stopping, successor_route, FiniteExecutionKind3D::kNominal,
          stopping->stopExecution()->trajectory_revision + 1U));

  // A stop never withholds movement: the moment a route is certified from the
  // vehicle it takes the vehicle back, whether or not the stop has been flown
  // to rest.
  ASSERT_TRUE(activation.applied());
  ASSERT_NE(activation.next, nullptr);
  EXPECT_EQ(activation.next->phase(), ExecutionRoutePhase3D::kFollowing);
  EXPECT_EQ(activation.next->stopExecution(), nullptr);
}

} // namespace
} // namespace drone_city_nav
