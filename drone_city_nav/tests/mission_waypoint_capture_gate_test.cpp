#include "drone_city_nav/mission_waypoint_capture_gate.hpp"
#include "drone_city_nav/mission_waypoint_sequence.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] MissionWaypointCaptureObservation
validObservation(const std::int64_t stamp_ns) {
  return MissionWaypointCaptureObservation{
      .stamp_ns = stamp_ns,
      .goal = Point3{10.0, 20.0, 18.0},
      .position = Point3{10.1, 19.9, 18.1},
      .velocity = Point3{0.1, 0.1, 0.1},
      .route_target = Point3{10.0, 20.0, 18.0},
      .stationary_hold_position = Point3{10.0, 20.0, 18.0},
      .pose_receive_stamp_ns = stamp_ns - 10'000'000,
      .vehicle_status_receive_stamp_ns = stamp_ns - 10'000'000,
      .horizon_valid_from_ns = 500'000'000,
      .horizon_valid_until_ns = 10'000'000'000,
      .feedback_source_stamp_ns = stamp_ns - 10'000'000,
      .feedback_receive_stamp_ns = stamp_ns - 5'000'000,
      .horizon_producer_instance_id = 17U,
      .horizon_sequence = 4U,
      .target_offboard_instance_id = 31U,
      .feedback_horizon_producer_instance_id = 17U,
      .feedback_horizon_sequence = 4U,
      .feedback_offboard_instance_id = 31U,
      .feedback_continuity_generation = 7U,
      .goal_capture_latched = true,
      .position_velocity_authoritative = true,
      .vehicle_status_valid = true,
      .vehicle_status_epoch_stable = true,
      .armed = true,
      .horizon_valid = true,
      .horizon_position_hold = true,
      .horizon_goal_capture = true,
      .horizon_stationary_position_hold = true,
      .feedback_valid = true,
      .feedback_position_hold = true,
      .feedback_control_authoritative = false,
      .feedback_continuity_generation_valid = true,
  };
}

[[nodiscard]] MissionWaypointCaptureGateResult
observe(MissionWaypointCaptureGate& gate,
        const MissionWaypointCaptureObservation& observation) {
  gate.beginTick(observation.stamp_ns);
  return gate.update(observation);
}

[[nodiscard]] MissionWaypointStationaryRearmObservation validRearmObservation() {
  return MissionWaypointStationaryRearmObservation{
      .stamp_ns = 5'000'000'000,
      .mission_goal = Point3{10.0, 20.0, 18.0},
      .active_waypoint_goal = Point3{10.0, 20.0, 18.0},
      .position = Point3{10.1, 19.9, 18.1},
      .velocity = Point3{0.1, 0.1, 0.1},
      .pose_receive_stamp_ns = 4'990'000'000,
      .vehicle_status_receive_stamp_ns = 4'990'000'000,
      .offboard_session_source_stamp_ns = 4'990'000'000,
      .offboard_session_receive_stamp_ns = 4'995'000'000,
      .offboard_instance_id = 31U,
      .yaw_rate_radps = 0.1,
      .objective_eligible = true,
      .goal_capture_latched = true,
      .execution_input_state_authoritative = true,
      .position_velocity_authoritative = true,
      .yaw_rate_authoritative = true,
      .vehicle_status_valid = true,
      .vehicle_status_epoch_stable = true,
      .armed = true,
      .offboard_session_valid = true,
      .applied_control_empty = true,
      .horizon_owner_empty = true,
      .execution_snapshot_revoked_empty = true,
      .validation_policy_current = true,
      .world_evidence_current = true,
      .lidar_evidence_current = true,
  };
}

TEST(MissionWaypointCaptureGateTest,
     RequiresContinuousExactThreeDimensionalArmedStopEvidence) {
  MissionWaypointCaptureGate gate;
  EXPECT_TRUE(observe(gate, validObservation(1'000'000'000)).evidence_valid);
  EXPECT_FALSE(observe(gate, validObservation(2'900'000'000)).ready);
  EXPECT_TRUE(observe(gate, validObservation(3'000'000'000)).ready);
}

TEST(MissionWaypointCaptureGateTest, AcceptsBoundedReceiptClockLag) {
  MissionWaypointCaptureGate gate;
  MissionWaypointCaptureObservation observation = validObservation(1'000'000'000);
  observation.feedback_receive_stamp_ns = observation.feedback_source_stamp_ns - 1;
  EXPECT_TRUE(observe(gate, observation).evidence_valid);
}

TEST(MissionWaypointCaptureGateTest,
     AcceptsNominalTwoHertzVehicleStatusAndRejectsAfterBudget) {
  MissionWaypointCaptureGate gate;
  MissionWaypointCaptureObservation nominal = validObservation(2'000'000'000);
  nominal.vehicle_status_receive_stamp_ns = 1'500'000'000;
  EXPECT_TRUE(observe(gate, nominal).evidence_valid);

  MissionWaypointCaptureObservation stale = validObservation(3'000'000'000);
  stale.vehicle_status_receive_stamp_ns = 1'999'999'999;
  EXPECT_FALSE(observe(gate, stale).evidence_valid);
}

TEST(MissionWaypointCaptureGateTest, RejectsUnrepresentableDurations) {
  EXPECT_THROW(MissionWaypointCaptureGate(MissionWaypointCaptureGateConfig{
                   .stop_hold_s = std::numeric_limits<double>::max(),
               }),
               std::invalid_argument);
}

TEST(MissionWaypointCaptureGateTest, RejectsWrongAltitudeAndVerticalSpeed) {
  MissionWaypointCaptureGate gate;
  MissionWaypointCaptureObservation wrong_z = validObservation(1'000'000'000);
  wrong_z.position.z = 21.0;
  EXPECT_FALSE(observe(gate, wrong_z).evidence_valid);

  MissionWaypointCaptureObservation wrong_vz = validObservation(2'000'000'000);
  wrong_vz.velocity.z = 0.81;
  EXPECT_FALSE(observe(gate, wrong_vz).evidence_valid);
}

TEST(MissionWaypointCaptureGateTest,
     RejectsStalePoseStatusDisarmAndMismatchedGoalOrWitness) {
  MissionWaypointCaptureGate gate;
  MissionWaypointCaptureObservation observation = validObservation(1'000'000'000);
  observation.pose_receive_stamp_ns = 800'000'000;
  EXPECT_FALSE(observe(gate, observation).evidence_valid);

  observation = validObservation(2'000'000'000);
  observation.vehicle_status_receive_stamp_ns = 999'999'999;
  EXPECT_FALSE(observe(gate, observation).evidence_valid);

  observation = validObservation(3'000'000'000);
  observation.armed = false;
  EXPECT_FALSE(observe(gate, observation).evidence_valid);

  observation = validObservation(4'000'000'000);
  observation.vehicle_status_epoch_stable = false;
  EXPECT_FALSE(observe(gate, observation).evidence_valid);

  observation = validObservation(5'000'000'000);
  observation.stationary_hold_position.z += 0.01;
  EXPECT_FALSE(observe(gate, observation).evidence_valid);

  observation = validObservation(6'000'000'000);
  observation.route_target.x += 0.01;
  EXPECT_FALSE(observe(gate, observation).evidence_valid);

  observation = validObservation(7'000'000'000);
  ++observation.feedback_horizon_sequence;
  EXPECT_FALSE(observe(gate, observation).evidence_valid);

  observation = validObservation(8'000'000'000);
  observation.feedback_source_stamp_ns = observation.stamp_ns - 300'000'000;
  observation.feedback_receive_stamp_ns = observation.stamp_ns - 290'000'000;
  EXPECT_FALSE(observe(gate, observation).evidence_valid);
}

TEST(MissionWaypointCaptureGateTest, AnyWitnessLossRestartsTheFullHold) {
  MissionWaypointCaptureGate gate;
  EXPECT_TRUE(observe(gate, validObservation(1'000'000'000)).evidence_valid);
  EXPECT_TRUE(observe(gate, validObservation(2'900'000'000)).evidence_valid);
  MissionWaypointCaptureObservation missing = validObservation(3'000'000'000);
  missing.feedback_valid = false;
  const MissionWaypointCaptureGateResult broken = observe(gate, missing);
  EXPECT_TRUE(broken.continuity_broken);
  EXPECT_FALSE(broken.ready);
  EXPECT_FALSE(observe(gate, validObservation(3'100'000'000)).ready);
  EXPECT_FALSE(observe(gate, validObservation(5'000'000'000)).ready);
  EXPECT_TRUE(observe(gate, validObservation(5'100'000'000)).ready);
}

TEST(MissionWaypointCaptureGateTest,
     HiddenFeedbackFallbackGenerationRestartsTheFullHold) {
  MissionWaypointCaptureGate gate;
  EXPECT_TRUE(observe(gate, validObservation(1'000'000'000)).evidence_valid);
  EXPECT_FALSE(observe(gate, validObservation(2'900'000'000)).ready);

  // The exact tuple recovered between planner ticks, but a fallback heartbeat
  // changed the sticky continuity generation.
  MissionWaypointCaptureObservation recovered = validObservation(3'000'000'000);
  ++recovered.feedback_continuity_generation;
  const MissionWaypointCaptureGateResult restarted = observe(gate, recovered);
  EXPECT_TRUE(restarted.evidence_valid);
  EXPECT_TRUE(restarted.continuity_broken);
  EXPECT_TRUE(restarted.continuity_started);
  EXPECT_FALSE(restarted.ready);

  recovered.stamp_ns = 5'000'000'000;
  recovered.pose_receive_stamp_ns = recovered.stamp_ns - 10'000'000;
  recovered.vehicle_status_receive_stamp_ns = recovered.stamp_ns - 10'000'000;
  recovered.feedback_source_stamp_ns = recovered.stamp_ns - 10'000'000;
  recovered.feedback_receive_stamp_ns = recovered.stamp_ns - 5'000'000;
  EXPECT_TRUE(observe(gate, recovered).ready);
}

TEST(MissionWaypointCaptureGateTest,
     ExhaustedFeedbackContinuityGenerationRejectsEvidence) {
  MissionWaypointCaptureGate gate;
  MissionWaypointCaptureObservation exhausted = validObservation(1'000'000'000);
  exhausted.feedback_continuity_generation_valid = false;
  EXPECT_FALSE(observe(gate, exhausted).evidence_valid);
}

TEST(MissionWaypointCaptureGateTest, MissingPlanningTickBreaksContinuity) {
  MissionWaypointCaptureGate gate;
  EXPECT_TRUE(observe(gate, validObservation(1'000'000'000)).evidence_valid);
  gate.beginTick(2'000'000'000);
  const MissionWaypointCaptureGateResult recovered =
      observe(gate, validObservation(3'000'000'000));
  EXPECT_TRUE(recovered.continuity_broken);
  EXPECT_FALSE(recovered.ready);
}

TEST(MissionWaypointCaptureGateTest, ExactOwnerTransitionRestartsTheFullHold) {
  MissionWaypointCaptureGate gate;
  EXPECT_TRUE(observe(gate, validObservation(1'000'000'000)).evidence_valid);
  EXPECT_FALSE(observe(gate, validObservation(2'900'000'000)).ready);

  MissionWaypointCaptureObservation transitioned = validObservation(3'000'000'000);
  ++transitioned.horizon_sequence;
  ++transitioned.feedback_horizon_sequence;
  const MissionWaypointCaptureGateResult restarted = observe(gate, transitioned);
  EXPECT_TRUE(restarted.evidence_valid);
  EXPECT_TRUE(restarted.continuity_broken);
  EXPECT_TRUE(restarted.continuity_started);
  EXPECT_FALSE(restarted.ready);

  transitioned.stamp_ns = 5'000'000'000;
  transitioned.pose_receive_stamp_ns = transitioned.stamp_ns - 10'000'000;
  transitioned.vehicle_status_receive_stamp_ns = transitioned.stamp_ns - 10'000'000;
  transitioned.feedback_source_stamp_ns = transitioned.stamp_ns - 10'000'000;
  transitioned.feedback_receive_stamp_ns = transitioned.stamp_ns - 5'000'000;
  EXPECT_TRUE(observe(gate, transitioned).ready);
}

TEST(MissionWaypointCaptureGateTest, DrivesTwoWaypointsOnlyAfterSeparateWitnesses) {
  MissionWaypointSequence sequence{
      {Point3{10.0, 20.0, 18.0}, Point3{30.0, 40.0, 20.0}}};
  MissionWaypointCaptureGate gate;
  EXPECT_FALSE(observe(gate, validObservation(1'000'000'000)).ready);
  ASSERT_TRUE(observe(gate, validObservation(3'000'000'000)).ready);
  const MissionWaypointUpdate first = sequence.acknowledgeGoalCapture();
  ASSERT_TRUE(first.advanced);
  gate.reset();

  MissionWaypointCaptureObservation second = validObservation(4'000'000'000);
  second.goal = sequence.activeGoal();
  second.position = second.goal;
  second.route_target = second.goal;
  second.stationary_hold_position = second.goal;
  second.horizon_sequence = 5U;
  second.feedback_horizon_sequence = 5U;
  EXPECT_FALSE(observe(gate, second).ready);
  second.stamp_ns = 6'000'000'000;
  second.pose_receive_stamp_ns = second.stamp_ns - 10'000'000;
  second.vehicle_status_receive_stamp_ns = second.stamp_ns - 10'000'000;
  second.feedback_source_stamp_ns = second.stamp_ns - 10'000'000;
  second.feedback_receive_stamp_ns = second.stamp_ns - 5'000'000;
  ASSERT_TRUE(observe(gate, second).ready);
  EXPECT_TRUE(sequence.acknowledgeGoalCapture().mission_completed);
}

TEST(MissionWaypointCaptureGateTest,
     StationaryRearmRequiresEveryAuthorityAndEvidenceGate) {
  const MissionWaypointStationaryRearmGateConfig config;
  ASSERT_TRUE(missionWaypointStationaryRearmEligible(config, validRearmObservation()));

  const auto expect_rejected = [&](const auto& mutate) {
    MissionWaypointStationaryRearmObservation observation = validRearmObservation();
    mutate(observation);
    EXPECT_FALSE(missionWaypointStationaryRearmEligible(config, observation));
  };
  expect_rejected([](auto& value) { value.objective_eligible = false; });
  expect_rejected([](auto& value) { value.goal_capture_latched = false; });
  expect_rejected(
      [](auto& value) { value.execution_input_state_authoritative = false; });
  expect_rejected([](auto& value) { value.position_velocity_authoritative = false; });
  expect_rejected([](auto& value) { value.yaw_rate_authoritative = false; });
  expect_rejected([](auto& value) { value.vehicle_status_valid = false; });
  expect_rejected([](auto& value) { value.vehicle_status_epoch_stable = false; });
  expect_rejected([](auto& value) { value.armed = false; });
  expect_rejected([](auto& value) { value.offboard_session_valid = false; });
  expect_rejected([](auto& value) { value.applied_control_empty = false; });
  expect_rejected([](auto& value) { value.horizon_owner_empty = false; });
  expect_rejected([](auto& value) { value.execution_snapshot_revoked_empty = false; });
  expect_rejected([](auto& value) { value.validation_policy_current = false; });
  expect_rejected([](auto& value) { value.world_evidence_current = false; });
  expect_rejected([](auto& value) { value.lidar_evidence_current = false; });
}

TEST(MissionWaypointCaptureGateTest,
     StationaryRearmRequiresFreshExactStoppedStateAtGoal) {
  const MissionWaypointStationaryRearmGateConfig config;
  const auto expect_rejected = [&](const auto& mutate) {
    MissionWaypointStationaryRearmObservation observation = validRearmObservation();
    mutate(observation);
    EXPECT_FALSE(missionWaypointStationaryRearmEligible(config, observation));
  };
  expect_rejected([](auto& value) { value.active_waypoint_goal.z += 0.01; });
  expect_rejected([](auto& value) { value.position.x += 0.3; });
  expect_rejected([](auto& value) { value.velocity.z = 0.26; });
  expect_rejected([](auto& value) { value.yaw_rate_radps = 0.26; });
  expect_rejected([](auto& value) { value.pose_receive_stamp_ns -= 200'000'000; });
  expect_rejected(
      [](auto& value) { value.vehicle_status_receive_stamp_ns -= 1'100'000'000; });
  expect_rejected(
      [](auto& value) { value.offboard_session_source_stamp_ns -= 300'000'000; });
  expect_rejected([](auto& value) { value.offboard_session_receive_stamp_ns = 0; });
  expect_rejected([](auto& value) { value.offboard_instance_id = 0U; });
}

} // namespace
} // namespace drone_city_nav
