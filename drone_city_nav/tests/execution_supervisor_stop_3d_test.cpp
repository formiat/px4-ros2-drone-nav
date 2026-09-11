#include "drone_city_nav/execution_hold_3d.hpp"
#include "drone_city_nav/execution_stop_3d.hpp"
#include "drone_city_nav/execution_supervisor_3d.hpp"
#include "drone_city_nav/pending_certified_route_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "execution_route_snapshot_3d_internal.hpp"
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
            const std::shared_ptr<const VersionedExecutionInput3D>& input,
            const std::size_t minimum_control_count = 100U) {
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
      .minimum_control_count = minimum_control_count,
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

TEST(ExecutionSupervisorStop3DTest, AVehicleCreepingToRestIsLeftToTheStationaryHold) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);

  // Just above the rest speed, the vehicle would stop within the distance a
  // hold pins its position with: a stop would change nothing but the lease.
  const ExecutionStopPreparation3D prepared = supervisor.prepareStop(stopRequest(
      fixture, active, movingInput(*active->finiteExecution()->execution_input, 0.3F)));

  EXPECT_EQ(prepared.status, ExecutionStopStatus3D::kAtRest);
  EXPECT_FALSE(prepared.prepared());
  EXPECT_LE(prepared.stop_distance_m, kStationaryExecutionHoldPositionToleranceM);
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
  // A stop executes no route and ends at rest, but it keeps the route it
  // took the vehicle over from: the lifecycle is replacing that route, and
  // the replacement is weighed against it while the vehicle brakes.
  EXPECT_EQ(prepared.transition->next->route(), nullptr);
  ASSERT_NE(prepared.transition->next->suspendedRoute(), nullptr);
  EXPECT_EQ(prepared.transition->next->suspendedRoute()->identity.generation,
            active->route()->identity.generation);
  EXPECT_EQ(prepared.transition->next->finiteExecution(), nullptr);
  EXPECT_EQ(prepared.transition->next->brakingFallback(), nullptr);
  ASSERT_NE(stop->horizon, nullptr);
  EXPECT_TRUE(finiteMotionHorizonHasTerminalRestState3D(*stop->horizon));
  EXPECT_GT(stop->trajectory_revision, active->finiteExecution()->trajectory_revision);
}

TEST(ExecutionSupervisorStop3DTest, TheStopOutlivesATruncatedControlSequence) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      movingInput(*active->finiteExecution()->execution_input, 4.0F);

  // The controller sequence a stop replaces is often the truncated remainder
  // of a route that just stopped being executable. The stop follows the
  // physics of stopping instead, so it is as long as bringing this vehicle to
  // rest takes.
  const ExecutionStopPreparation3D prepared =
      supervisor.prepareStop(stopRequest(fixture, active, input, 8U));

  ASSERT_EQ(prepared.status, ExecutionStopStatus3D::kPrepared)
      << executionStopStatus3DName(prepared.status);
  const StopExecution3D* const stop = prepared.stopExecution();
  ASSERT_NE(stop, nullptr);
  ASSERT_NE(stop->horizon, nullptr);
  EXPECT_GT(stop->horizon->controls.size(), 8U);
  EXPECT_TRUE(finiteMotionHorizonHasTerminalRestState3D(*stop->horizon));
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

// The clearance envelope is what a route keeps from evidence; a stop is the
// last motion the vehicle can be given. Evidence the envelope would sweep
// through while the physical body passes clear does not leave the vehicle
// flying its stale horizon: the same trajectory is certified on the body, and
// that stop stays resident while the body's path remains clear.
TEST(ExecutionSupervisorStop3DTest,
     AStopTheEnvelopeCannotClearKeepsWhatClearanceItCan) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      movingInput(*active->finiteExecution()->execution_input, 6.0F);
  // A voxel one metre beside the braking line, well past the contact volume
  // around the vehicle's own pose: inside a 1.2 m envelope, clear of a 0.5 m
  // body.
  ObservedOccupancyGrid3D beside_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> beside =
      beside_occupancy.worldToCell(Point3{5.5, 1.5, 5.5});
  ASSERT_TRUE(beside.has_value());
  ASSERT_TRUE(beside_occupancy.setState(*beside, ObservedVoxelState::kOccupied));
  SweptFootprintConfig footprint = fixture.execution_footprint;
  footprint.radius_m = 1.2;
  footprint.body_radius_m = 0.5;
  footprint.perimeter_samples = 12U;
  footprint.radial_rings = 2U;
  ExecutionStopRequest3D request = stopRequest(fixture, active, input);
  request.observed_raw_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &beside_occupancy);
  request.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      fixture.validation_policy->flightEnvelope(),
      fixture.validation_policy->dynamics(),
      fixture.validation_policy->altitudeEnvelope(), footprint, 100.0, 1000.0, 1000.0,
      true);
  ASSERT_NE(request.validation_policy, nullptr);

  const ExecutionStopPreparation3D prepared = supervisor.prepareStop(request);

  ASSERT_EQ(prepared.status, ExecutionStopStatus3D::kPrepared)
      << executionStopStatus3DName(prepared.status) << " certification="
      << stopCertificationStatus3DName(prepared.certification.status) << " path="
      << finiteExecutionPathStatus3DName(prepared.certification.path_validation_status);
  EXPECT_GT(prepared.stop_distance_m, 4.0);
  // The envelope was refused; the stop gave up part of its clearance and kept
  // the rest, so its sweep stays wider than the bare hull.
  EXPECT_GT(prepared.certification.clearance_reduction, 0.0);
  const StopExecution3D* const stop = prepared.stopExecution();
  ASSERT_NE(stop, nullptr);
  EXPECT_GT(stop->clearance_reduction, 0.0);
  EXPECT_GT(stop->validation_footprint.radius_m, 0.5);
  EXPECT_LT(stop->validation_footprint.radius_m, 1.2);
  EXPECT_NEAR(stop->validation_footprint.body_radius_m, 0.5, 1.0e-9);

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

  // The resident stop is judged with the body it was certified with, so the
  // envelope's failure does not derive a new stop every tick.
  ExecutionStopRequest3D again_request =
      stopRequest(fixture, stopping, movingInput(*input, 5.0F));
  again_request.observed_raw_world = request.observed_raw_world;
  again_request.validation_policy = request.validation_policy;
  const ExecutionStopPreparation3D again = supervisor.prepareStop(again_request);

  EXPECT_EQ(again.status, ExecutionStopStatus3D::kResidentStopCurrent)
      << executionStopStatus3DName(again.status);
  EXPECT_EQ(supervisor.plan(), stopping);
}

// Where the vehicle comes to rest it stays, and a vehicle at rest drifts
// within the position error its controller holds it to. A stop that would rest
// the hull inside the margin the envelope carries over it is refused, so a
// vehicle flying in the clear never chooses such a rest point.
TEST(ExecutionSupervisorStop3DTest, AStopDoesNotRestInsideTheEnvelopeMargin) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      movingInput(*active->finiteExecution()->execution_input, 4.0F);
  const ExecutionStopPreparation3D clear =
      supervisor.prepareStop(stopRequest(fixture, active, input));
  ASSERT_TRUE(clear.prepared()) << executionStopStatus3DName(clear.status);
  const StopExecution3D* const clear_stop = clear.stopExecution();
  ASSERT_NE(clear_stop, nullptr);

  // A voxel beside the rest point whose near face lies half a metre from the
  // hull: the hull sweeps clear of it, the envelope does not, and the rest
  // pose keeps less than the margin the envelope carries over the hull.
  const Point3 rest = clear_stop->rest_position;
  SweptFootprintConfig footprint = fixture.execution_footprint;
  footprint.radius_m = 1.2;
  footprint.body_radius_m = 0.5;
  footprint.perimeter_samples = 12U;
  footprint.radial_rings = 2U;
  ObservedOccupancyGrid3D beside_occupancy = fixture.raw_occupancy;
  const std::optional<GridIndex3D> beside =
      beside_occupancy.worldToCell(Point3{rest.x, rest.y + 1.5, rest.z});
  ASSERT_TRUE(beside.has_value());
  ASSERT_TRUE(beside_occupancy.setState(*beside, ObservedVoxelState::kOccupied));
  ExecutionStopRequest3D request = stopRequest(fixture, active, input);
  request.observed_raw_world =
      fixture.rawWorld(SnapshotFixture3D::kLatestRawRevision + 1U, &beside_occupancy);
  request.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      fixture.validation_policy->flightEnvelope(),
      fixture.validation_policy->dynamics(),
      fixture.validation_policy->altitudeEnvelope(), footprint, 100.0, 1000.0, 1000.0,
      true);
  ASSERT_NE(request.validation_policy, nullptr);

  const ExecutionStopPreparation3D prepared = supervisor.prepareStop(request);

  EXPECT_FALSE(prepared.prepared());
  EXPECT_EQ(prepared.certification.status,
            StopCertificationStatus3D::kRestClearanceRejected)
      << stopCertificationStatus3DName(prepared.certification.status) << " path="
      << finiteExecutionPathStatus3DName(prepared.certification.path_validation_status)
      << " reduction=" << prepared.certification.clearance_reduction << " rest=("
      << rest.x << "," << rest.y << "," << rest.z << ")";
}

// The stop that owns the vehicle after it has been committed on the wire.
[[nodiscard]] std::shared_ptr<const ExecutionPlan3D>
commitStop(ExecutionSupervisor3D& supervisor,
           const ExecutionStopPreparation3D& prepared,
           const std::shared_ptr<const VersionedExecutionInput3D>& input) {
  if (!prepared.prepared() || prepared.transition == nullptr ||
      prepared.transition->next == nullptr) {
    return nullptr;
  }
  const ExecutionHorizonCommitResult3D committed = commitExecutionHorizonForTest(
      supervisor,
      ExecutionHorizonTestTransaction3D{
          .kind = ExecutionHorizonCommitKind3D::kTransition,
          .expected_authority = prepared.expected_authority,
          .expected_plan = prepared.expectedPlan(),
          .transition = *prepared.transition,
          .expected_pending = nullptr,
          .owner = SnapshotFixture3D::committedOwner(*prepared.transition->next, 2U),
          .input = input,
      });
  return committed.committed() ? supervisor.plan() : nullptr;
}

TEST(ExecutionSupervisorStop3DTest, AStopFlownToRestBecomesAStationaryHold) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      movingInput(*active->finiteExecution()->execution_input, 4.0F);
  const std::shared_ptr<const ExecutionPlan3D> stopping = commitStop(
      supervisor, supervisor.prepareStop(stopRequest(fixture, active, input)), input);
  ASSERT_NE(stopping, nullptr);
  const StopExecution3D* const stop = stopping->stopExecution();
  ASSERT_NE(stop, nullptr);

  // The vehicle observed at rest where the stop ends, once the stop has been
  // flown out: the hold takes the vehicle over from the stop exactly as it
  // takes it over from a route that reached its own rest.
  const StationaryExecutionHoldCertification3D certification =
      SnapshotFixture3D::holdCertification(*stopping);
  const ExecutionHoldPreparation3D hold = supervisor.prepareHold(ExecutionHoldRequest3D{
      .intent = ExecutionHoldIntent3D::kExplicitTransfer,
      .requested_position = stop->rest_position,
      .cycle_source_plan = stopping,
      .execution_input = certification.execution_input,
      .latest_lidar_evidence = certification.latest_lidar_evidence,
      .current_lidar_evidence = certification.latest_lidar_evidence,
      .current_observed_raw_world = certification.observed_raw_world,
      .stationary_capture_observed_raw_world = nullptr,
      .stationary_capture_static_world = nullptr,
      .selected_validation_policy = nullptr,
      .stationary_capture_validation_policy = certification.validation_policy,
      .validation_now_ns = certification.execution_input->effectiveStampNs(),
  });

  ASSERT_EQ(hold.status, ExecutionHoldPreparationStatus3D::kPrepared)
      << executionHoldPreparationStatus3DName(hold.status) << " "
      << executionRouteTransitionStatus3DName(hold.transition_status);
  ASSERT_EQ(hold.kind, ExecutionHoldPreparationKind3D::kTransition);
  ASSERT_NE(hold.transition, nullptr);
  ASSERT_NE(hold.transition->next, nullptr);
  EXPECT_EQ(hold.transition->next->phase(), ExecutionRoutePhase3D::kStopped);
  EXPECT_NE(hold.transition->next->stationaryHold(), nullptr);
  EXPECT_EQ(hold.transition->next->stopExecution(), nullptr);
  EXPECT_NEAR(hold.position.x, stop->rest_position.x, 1.0e-6);
  EXPECT_NEAR(hold.position.y, stop->rest_position.y, 1.0e-6);
}

TEST(ExecutionSupervisorStop3DTest, TheHoldPinsWhereTheVehicleActuallyStopped) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      movingInput(*active->finiteExecution()->execution_input, 4.0F);
  const std::shared_ptr<const ExecutionPlan3D> stopping = commitStop(
      supervisor, supervisor.prepareStop(stopRequest(fixture, active, input)), input);
  ASSERT_NE(stopping, nullptr);
  const StopExecution3D* const stop = stopping->stopExecution();
  ASSERT_NE(stop, nullptr);

  // Braking left the vehicle well past the rest point the stop predicted: the
  // stop's rest is a model prediction, and the hold is pinned where the
  // vehicle measurably rests once the stop's lease commands nothing else.
  const Point3 actual_rest{stop->rest_position.x + 0.6, stop->rest_position.y,
                           stop->rest_position.z};
  const StationaryExecutionHoldCertification3D evidence =
      SnapshotFixture3D::holdCertification(*stopping);
  const std::shared_ptr<const VersionedExecutionInput3D> resting =
      restingInputAt(*stop->execution_input, actual_rest, stop->valid_until_ns + 1);
  ASSERT_NE(resting, nullptr);
  const ExecutionHoldPreparation3D hold = supervisor.prepareHold(ExecutionHoldRequest3D{
      .intent = ExecutionHoldIntent3D::kExplicitTransfer,
      .requested_position = actual_rest,
      .cycle_source_plan = stopping,
      .execution_input = resting,
      .latest_lidar_evidence = evidence.latest_lidar_evidence,
      .current_lidar_evidence = evidence.latest_lidar_evidence,
      .current_observed_raw_world = evidence.observed_raw_world,
      .stationary_capture_observed_raw_world = nullptr,
      .stationary_capture_static_world = nullptr,
      .selected_validation_policy = nullptr,
      .stationary_capture_validation_policy = evidence.validation_policy,
      .validation_now_ns = resting->effectiveStampNs(),
  });

  ASSERT_EQ(hold.status, ExecutionHoldPreparationStatus3D::kPrepared)
      << executionHoldPreparationStatus3DName(hold.status) << " "
      << executionRouteTransitionStatus3DName(hold.transition_status) << " "
      << executionRouteTransitionDetail3DName(hold.transition_detail);
  ASSERT_NE(hold.transition, nullptr);
  ASSERT_NE(hold.transition->next, nullptr);
  ASSERT_NE(hold.transition->next->stationaryHold(), nullptr);
  EXPECT_NEAR(hold.transition->next->stationaryHold()->position.x, actual_rest.x,
              1.0e-6);
}

// A hold taken over from a stop carries that stop's trajectory revision, and
// the route that takes the resting vehicle back is numbered after it. Numbering
// it as if nothing had owned the vehicle left one recorded flight resting for
// half a minute with a certified route pending.
TEST(ExecutionSupervisorStop3DTest, ACertifiedRouteTakesTheVehicleBackFromARestHold) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      movingInput(*active->finiteExecution()->execution_input, 4.0F);
  const std::shared_ptr<const ExecutionPlan3D> stopping = commitStop(
      supervisor, supervisor.prepareStop(stopRequest(fixture, active, input)), input);
  ASSERT_NE(stopping, nullptr);
  const StopExecution3D* const stop = stopping->stopExecution();
  ASSERT_NE(stop, nullptr);
  EXPECT_EQ(stopping->ownerTrajectoryRevision(), stop->trajectory_revision);

  const Point3 rest_position = stop->rest_position;
  const StationaryExecutionHoldCertification3D evidence =
      SnapshotFixture3D::holdCertification(*stopping);
  const std::shared_ptr<const VersionedExecutionInput3D> resting_input =
      restingInputAt(*stop->execution_input, rest_position, stop->valid_until_ns + 1);
  ASSERT_NE(resting_input, nullptr);
  const ExecutionHoldPreparation3D hold = supervisor.prepareHold(ExecutionHoldRequest3D{
      .intent = ExecutionHoldIntent3D::kExplicitTransfer,
      .requested_position = rest_position,
      .cycle_source_plan = stopping,
      .execution_input = resting_input,
      .latest_lidar_evidence = evidence.latest_lidar_evidence,
      .current_lidar_evidence = evidence.latest_lidar_evidence,
      .current_observed_raw_world = evidence.observed_raw_world,
      .stationary_capture_observed_raw_world = nullptr,
      .stationary_capture_static_world = nullptr,
      .selected_validation_policy = nullptr,
      .stationary_capture_validation_policy = evidence.validation_policy,
      .validation_now_ns = resting_input->effectiveStampNs(),
  });
  ASSERT_TRUE(hold.prepared())
      << executionHoldPreparationStatus3DName(hold.status) << " "
      << executionRouteTransitionStatus3DName(hold.transition_status) << " "
      << executionRouteTransitionDetail3DName(hold.transition_detail);
  ASSERT_NE(hold.transition, nullptr);
  const std::shared_ptr<const ExecutionPlan3D> resting = hold.transition->next;
  ASSERT_NE(resting, nullptr);
  ASSERT_NE(resting->stationaryHold(), nullptr);
  EXPECT_EQ(resting->stationaryHold()->source_trajectory_revision,
            stop->trajectory_revision);
  EXPECT_EQ(resting->ownerTrajectoryRevision(), stop->trajectory_revision);

  // The successor is certified from where the vehicle rests, along the route.
  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = resting->routeGenerationHighWater() + 1U;
  successor_activation =
      rebindUnconstrainedDecorations(std::move(successor_activation));
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const CertifiedRouteSuffix3D& successor_route =
      successor.value(); // NOLINT(bugprone-unchecked-optional-access)
  const ExecutionRouteTransitionResult3D activation = activateCertifiedRoute3D(
      *resting, resting->version, successor_route,
      SnapshotFixture3D::finitePlanForRoute(
          *resting, successor_route, FiniteExecutionKind3D::kNominal,
          resting->ownerTrajectoryRevision() + 1U, 55U, 0U, rest_position.x));

  ASSERT_TRUE(activation.applied())
      << executionRouteTransitionStatus3DName(activation.status) << " "
      << executionRouteTransitionDetail3DName(activation.detail);
  ASSERT_NE(activation.next, nullptr);
  EXPECT_EQ(activation.next->phase(), ExecutionRoutePhase3D::kFollowing);
  EXPECT_EQ(activation.next->stationaryHold(), nullptr);
  ASSERT_NE(activation.next->finiteExecution(), nullptr);
  EXPECT_GT(activation.next->finiteExecution()->trajectory_revision,
            stop->trajectory_revision);
}

// A held vehicle drifts within the position error its controller holds it to.
// The route takes it back from wherever it actually is: a takeover refused
// because the vehicle had drifted past the hold's tolerance is a hold the
// vehicle can never leave, since the hold cannot be refreshed at its own stale
// position either.
TEST(ExecutionSupervisorStop3DTest, AHoldDoesNotHoldADriftedVehicleAgainstItsRoute) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      movingInput(*active->finiteExecution()->execution_input, 4.0F);
  const std::shared_ptr<const ExecutionPlan3D> stopping = commitStop(
      supervisor, supervisor.prepareStop(stopRequest(fixture, active, input)), input);
  ASSERT_NE(stopping, nullptr);
  const StopExecution3D* const stop = stopping->stopExecution();
  ASSERT_NE(stop, nullptr);

  // The hold is pinned where the vehicle rested; the vehicle has since drifted
  // along the route, well past the tolerance the hold pins with.
  const Point3 rest_position = stop->rest_position;
  const double drift_m = 4.0 * kStationaryExecutionHoldPositionToleranceM;
  const StationaryExecutionHoldCertification3D evidence =
      SnapshotFixture3D::holdCertification(*stopping);
  const std::shared_ptr<const VersionedExecutionInput3D> resting_input =
      restingInputAt(*stop->execution_input, rest_position, stop->valid_until_ns + 1);
  ASSERT_NE(resting_input, nullptr);
  const ExecutionHoldPreparation3D hold = supervisor.prepareHold(ExecutionHoldRequest3D{
      .intent = ExecutionHoldIntent3D::kExplicitTransfer,
      .requested_position = rest_position,
      .cycle_source_plan = stopping,
      .execution_input = resting_input,
      .latest_lidar_evidence = evidence.latest_lidar_evidence,
      .current_lidar_evidence = evidence.latest_lidar_evidence,
      .current_observed_raw_world = evidence.observed_raw_world,
      .stationary_capture_observed_raw_world = nullptr,
      .stationary_capture_static_world = nullptr,
      .selected_validation_policy = nullptr,
      .stationary_capture_validation_policy = evidence.validation_policy,
      .validation_now_ns = resting_input->effectiveStampNs(),
  });
  ASSERT_TRUE(hold.prepared()) << executionHoldPreparationStatus3DName(hold.status);
  ASSERT_NE(hold.transition, nullptr);
  const std::shared_ptr<const ExecutionPlan3D> resting = hold.transition->next;
  ASSERT_NE(resting, nullptr);
  const StationaryExecutionHold3D* const held = resting->stationaryHold();
  ASSERT_NE(held, nullptr);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = resting->routeGenerationHighWater() + 1U;
  successor_activation =
      rebindUnconstrainedDecorations(std::move(successor_activation));
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  const CertifiedRouteSuffix3D& successor_route =
      successor.value(); // NOLINT(bugprone-unchecked-optional-access)
  const FiniteExecutionPlan3D plan = SnapshotFixture3D::finitePlanForRoute(
      *resting, successor_route, FiniteExecutionKind3D::kNominal,
      resting->ownerTrajectoryRevision() + 1U, 55U, 0U, rest_position.x + drift_m);

  EXPECT_GT(distance3D(execution_route_snapshot_3d_internal::executionInputPosition(
                           *plan.command_horizon.execution_input),
                       held->position),
            kStationaryExecutionHoldPositionToleranceM);
  EXPECT_TRUE(
      execution_route_snapshot_3d_internal::routeExecutionEvidenceNotOlderThanHold(
          plan.command_horizon, *held));
}

TEST(ExecutionSupervisorStop3DTest, ACertifiedSuccessorIsPublishedPendingAgainstAStop) {
  SnapshotFixture3D fixture;
  ExecutionSupervisor3D supervisor;
  const std::shared_ptr<const ExecutionPlan3D> active =
      installRouteOwner(supervisor, fixture);
  ASSERT_NE(active, nullptr);
  ASSERT_NE(active->finiteExecution(), nullptr);
  const std::shared_ptr<const VersionedExecutionInput3D> input =
      movingInput(*active->finiteExecution()->execution_input, 4.0F);
  const std::shared_ptr<const ExecutionPlan3D> stopping = commitStop(
      supervisor, supervisor.prepareStop(stopRequest(fixture, active, input)), input);
  ASSERT_NE(stopping, nullptr);
  ASSERT_NE(stopping->stopExecution(), nullptr);

  ExecutionRouteActivation3D successor_activation = fixture.activation();
  successor_activation.route_generation = stopping->routeGenerationHighWater() + 1U;
  successor_activation =
      rebindUnconstrainedDecorations(std::move(successor_activation));
  const std::optional<CertifiedRouteSuffix3D> successor =
      certifyExecutionRoute3D(successor_activation);
  ASSERT_TRUE(successor.has_value());
  // A route certified while the vehicle stops is planned from the vehicle and
  // hands off from the stop: it follows no route of its own and carries no
  // splice, like a successor offered against a hold.
  PendingCertifiedRoute3D pending{
      .publication_sequence = 0U,
      .base_execution_owner_epoch = stopping->execution_owner_epoch,
      .base_kind = PendingExecutionBaseKind3D::kStop,
      .base_route_generation = stopping->routeGenerationHighWater(),
      .base_geometry_revision = 0U,
      .base_continuity_id = 0U,
      .base_direct_tracking_identity = std::nullopt,
      .route_splice = std::nullopt,
      .route = successor.value(), // NOLINT(bugprone-unchecked-optional-access)
  };

  const PendingRoutePublicationResult3D published =
      supervisor.publishPendingForCurrentBase(stopping, pending);

  EXPECT_EQ(published.status, PendingRoutePublicationStatus3D::kPublished);
  ASSERT_NE(published.pending, nullptr);
  EXPECT_TRUE(published.pending->valid());
  EXPECT_TRUE(pendingCertifiedRouteEligible3D(*published.pending, *stopping));
  EXPECT_EQ(supervisor.pending(), published.pending);
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
