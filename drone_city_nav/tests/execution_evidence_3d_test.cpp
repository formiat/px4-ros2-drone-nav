#include "drone_city_nav/execution_evidence_3d.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] ExecutionStateProvenance3D authoritativeStateProvenance() {
  return ExecutionStateProvenance3D{
      .x = ExecutionStateFieldProvenance3D::kSourceSample,
      .y = ExecutionStateFieldProvenance3D::kSourceSample,
      .z = ExecutionStateFieldProvenance3D::kSourceSample,
      .vx = ExecutionStateFieldProvenance3D::kEffectiveTimePrediction,
      .vy = ExecutionStateFieldProvenance3D::kEffectiveTimePrediction,
      .vz = ExecutionStateFieldProvenance3D::kEffectiveTimePrediction,
      .yaw = ExecutionStateFieldProvenance3D::kSourceSample,
      .yaw_rate = ExecutionStateFieldProvenance3D::kSourceSample,
  };
}

[[nodiscard]] ExecutionInputCapture3D validExecutionInputCapture() {
  return ExecutionInputCapture3D{
      .capture_sequence = 17U,
      .pose_revision = 23U,
      .pose_source_timestamp_us = 5'000'000U,
      .pose_receive_stamp_ns = 5'010'000'000,
      .effective_stamp_ns = 5'020'000'000,
      .state = mppi::State{.x = 1.0F,
                           .y = 2.0F,
                           .z = 3.0F,
                           .vx = 4.0F,
                           .vy = 5.0F,
                           .vz = 6.0F,
                           .yaw = 0.25F,
                           .yaw_rate = -0.5F},
      .full_state_authoritative = true,
      .state_provenance = authoritativeStateProvenance(),
      .previous_control =
          mppi::Control{.ax = 0.1F, .ay = 0.2F, .az = 0.3F, .yaw_accel = 0.4F},
      .previous_control_source =
          ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback,
      .previous_control_source_producer_instance_id = 29U,
      .previous_control_source_sequence = 31U,
      .previous_control_source_stamp_ns = 5'015'000'000,
      .previous_control_receive_stamp_ns = 5'016'000'000,
  };
}

[[nodiscard]] LatestLidarEvidenceCapture3D validLatestLidarCapture() {
  return LatestLidarEvidenceCapture3D{
      .producer_instance_id = 41U,
      .sequence = 43U,
      .pose_generation = 47U,
      .acquisition_stamp_ns = 6'000'000'000,
      .receive_stamp_ns = 6'010'000'000,
      .source_beam_count = 5U,
      .invalid_beam_count = 1U,
      .hit_points_map_m = {Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}},
  };
}

TEST(ExecutionEvidence3DTest,
     ValidationPolicyOwnsExactValuesAndHasContentSensitiveFingerprint) {
  FlightEnvelopeConfig flight{.minimum_target_z_m = 2.0, .maximum_target_z_m = 40.0};
  mppi::DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  mppi::AltitudeEnvelopeConfig altitude{
      .minimum_z_m = 2.0F,
      .maximum_z_m = 40.0F,
      .guaranteed_vertical_deceleration_mps2 = 3.0F,
      .reaction_latency_s = 0.2F,
  };
  SweptFootprintConfig footprint;
  footprint.radius_m = 0.9;

  const auto policy = VersionedExecutionValidationPolicy3D::capture(
      flight, dynamics, altitude, footprint);
  const auto repeated = VersionedExecutionValidationPolicy3D::capture(
      flight, dynamics, altitude, footprint);
  ASSERT_NE(policy, nullptr);
  ASSERT_NE(repeated, nullptr);
  ASSERT_TRUE(policy->valid());
  EXPECT_TRUE(policy->policyId().valid());
  EXPECT_EQ(policy->policyId(), repeated->policyId());
  EXPECT_NE(policy->contentFingerprint(), 0U);
  EXPECT_EQ(policy->contentFingerprint(), repeated->contentFingerprint());

  flight.minimum_target_z_m = 3.0;
  dynamics.dt_s = 0.2F;
  altitude.reaction_latency_s = 0.4F;
  footprint.radius_m = 1.1;
  EXPECT_DOUBLE_EQ(policy->flightEnvelope().minimum_target_z_m, 2.0);
  EXPECT_FLOAT_EQ(policy->dynamics().dt_s, 0.1F);
  EXPECT_FLOAT_EQ(policy->altitudeEnvelope().reaction_latency_s, 0.2F);
  EXPECT_DOUBLE_EQ(policy->sweptFootprint().radius_m, 0.9);
  EXPECT_DOUBLE_EQ(policy->latestLidarMaximumAgeMs(), 1000.0);
  EXPECT_DOUBLE_EQ(policy->executionInputMaximumPoseAgeMs(), 1000.0);
  EXPECT_DOUBLE_EQ(policy->executionInputMaximumControlAgeMs(), 1000.0);
  EXPECT_FALSE(policy->routeCrossTrackConstraintsEnabled());
  EXPECT_TRUE(policy->latestLidarFreshnessRequired());
  EXPECT_TRUE(policy->routeTrackingTubeConstraintsEnabled());

  const auto changed = VersionedExecutionValidationPolicy3D::capture(
      FlightEnvelopeConfig{.minimum_target_z_m = 2.0, .maximum_target_z_m = 40.0},
      policy->dynamics(), policy->altitudeEnvelope(), footprint);
  ASSERT_NE(changed, nullptr);
  EXPECT_NE(changed->policyId(), policy->policyId());
  EXPECT_NE(changed->contentFingerprint(), policy->contentFingerprint());

  const auto changed_pose_age = VersionedExecutionValidationPolicy3D::capture(
      policy->flightEnvelope(), policy->dynamics(), policy->altitudeEnvelope(),
      policy->sweptFootprint(), policy->latestLidarMaximumAgeMs(), 999.0,
      policy->executionInputMaximumControlAgeMs());
  const auto changed_control_age = VersionedExecutionValidationPolicy3D::capture(
      policy->flightEnvelope(), policy->dynamics(), policy->altitudeEnvelope(),
      policy->sweptFootprint(), policy->latestLidarMaximumAgeMs(),
      policy->executionInputMaximumPoseAgeMs(), 999.0);
  const auto strict_route_adherence = VersionedExecutionValidationPolicy3D::capture(
      policy->flightEnvelope(), policy->dynamics(), policy->altitudeEnvelope(),
      policy->sweptFootprint(), policy->latestLidarMaximumAgeMs(),
      policy->executionInputMaximumPoseAgeMs(),
      policy->executionInputMaximumControlAgeMs(), true);
  const auto diagnostic_lidar_freshness = VersionedExecutionValidationPolicy3D::capture(
      policy->flightEnvelope(), policy->dynamics(), policy->altitudeEnvelope(),
      policy->sweptFootprint(), policy->latestLidarMaximumAgeMs(),
      policy->executionInputMaximumPoseAgeMs(),
      policy->executionInputMaximumControlAgeMs(), false, false);
  const auto diagnostic_tracking_tube = VersionedExecutionValidationPolicy3D::capture(
      policy->flightEnvelope(), policy->dynamics(), policy->altitudeEnvelope(),
      policy->sweptFootprint(), policy->latestLidarMaximumAgeMs(),
      policy->executionInputMaximumPoseAgeMs(),
      policy->executionInputMaximumControlAgeMs(), false, true, false);
  ASSERT_NE(changed_pose_age, nullptr);
  ASSERT_NE(changed_control_age, nullptr);
  ASSERT_NE(strict_route_adherence, nullptr);
  ASSERT_NE(diagnostic_lidar_freshness, nullptr);
  ASSERT_NE(diagnostic_tracking_tube, nullptr);
  EXPECT_NE(changed_pose_age->contentFingerprint(), policy->contentFingerprint());
  EXPECT_NE(changed_control_age->contentFingerprint(), policy->contentFingerprint());
  EXPECT_TRUE(strict_route_adherence->routeCrossTrackConstraintsEnabled());
  EXPECT_NE(strict_route_adherence->contentFingerprint(), policy->contentFingerprint());
  EXPECT_FALSE(diagnostic_lidar_freshness->latestLidarFreshnessRequired());
  EXPECT_NE(diagnostic_lidar_freshness->contentFingerprint(),
            policy->contentFingerprint());
  EXPECT_FALSE(diagnostic_tracking_tube->routeTrackingTubeConstraintsEnabled());
  EXPECT_NE(diagnostic_tracking_tube->contentFingerprint(),
            policy->contentFingerprint());
}

TEST(ExecutionEvidence3DTest,
     PublicationTimeFreshnessExpiresPoseAndControlAtExactBoundaries) {
  const auto input = VersionedExecutionInput3D::capture(validExecutionInputCapture());
  ASSERT_NE(input, nullptr);
  const auto exact_policy = VersionedExecutionValidationPolicy3D::capture(
      FlightEnvelopeConfig{}, mppi::DynamicsConfig{}, mppi::AltitudeEnvelopeConfig{},
      SweptFootprintConfig{}, 1000.0, 10.0, 4.0);
  ASSERT_NE(exact_policy, nullptr);

  constexpr std::int64_t kExactBoundaryNs{5'020'000'000};
  EXPECT_TRUE(executionInputFreshAt(*input, *exact_policy, kExactBoundaryNs));
  EXPECT_FALSE(executionInputFreshAt(*input, *exact_policy, kExactBoundaryNs + 1));
  EXPECT_FALSE(executionInputFreshAt(*input, *exact_policy, 5'009'999'999));

  const auto pose_limited = VersionedExecutionValidationPolicy3D::capture(
      exact_policy->flightEnvelope(), exact_policy->dynamics(),
      exact_policy->altitudeEnvelope(), exact_policy->sweptFootprint(), 1000.0, 10.0,
      100.0);
  const auto control_limited = VersionedExecutionValidationPolicy3D::capture(
      exact_policy->flightEnvelope(), exact_policy->dynamics(),
      exact_policy->altitudeEnvelope(), exact_policy->sweptFootprint(), 1000.0, 100.0,
      4.0);
  ASSERT_NE(pose_limited, nullptr);
  ASSERT_NE(control_limited, nullptr);
  EXPECT_FALSE(executionInputFreshAt(*input, *pose_limited, kExactBoundaryNs + 1));
  EXPECT_FALSE(executionInputFreshAt(*input, *control_limited, kExactBoundaryNs + 1));
}

TEST(ExecutionEvidence3DTest, ValidationPolicyRejectsInvalidConfiguration) {
  FlightEnvelopeConfig flight;
  mppi::DynamicsConfig dynamics;
  mppi::AltitudeEnvelopeConfig altitude;
  SweptFootprintConfig footprint;

  flight.maximum_target_z_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(VersionedExecutionValidationPolicy3D::capture(flight, dynamics, altitude,
                                                          footprint),
            nullptr);
  flight = FlightEnvelopeConfig{};

  dynamics.maximum_control_jerk_mps3 = 0.0F;
  EXPECT_EQ(VersionedExecutionValidationPolicy3D::capture(flight, dynamics, altitude,
                                                          footprint),
            nullptr);
  dynamics = mppi::DynamicsConfig{};

  dynamics.maximum_translational_speed_mps = 0.0F;
  EXPECT_EQ(VersionedExecutionValidationPolicy3D::capture(flight, dynamics, altitude,
                                                          footprint),
            nullptr);
  dynamics = mppi::DynamicsConfig{};

  altitude.maximum_z_m = altitude.minimum_z_m;
  EXPECT_EQ(VersionedExecutionValidationPolicy3D::capture(flight, dynamics, altitude,
                                                          footprint),
            nullptr);
  altitude = mppi::AltitudeEnvelopeConfig{};

  footprint.sweep_step_m = 0.0;
  EXPECT_EQ(VersionedExecutionValidationPolicy3D::capture(flight, dynamics, altitude,
                                                          footprint),
            nullptr);

  footprint = SweptFootprintConfig{};
  footprint.radius_m = 0.5;
  footprint.perimeter_samples = 0U;
  EXPECT_EQ(VersionedExecutionValidationPolicy3D::capture(flight, dynamics, altitude,
                                                          footprint),
            nullptr);

  footprint = SweptFootprintConfig{};
  footprint.radius_m = 0.5;
  footprint.radial_rings = 0U;
  EXPECT_EQ(VersionedExecutionValidationPolicy3D::capture(flight, dynamics, altitude,
                                                          footprint),
            nullptr);

  footprint = SweptFootprintConfig{};
  EXPECT_EQ(VersionedExecutionValidationPolicy3D::capture(flight, dynamics, altitude,
                                                          footprint, 0.0),
            nullptr);
  EXPECT_EQ(VersionedExecutionValidationPolicy3D::capture(flight, dynamics, altitude,
                                                          footprint, 1000.0, 0.0),
            nullptr);
  EXPECT_EQ(VersionedExecutionValidationPolicy3D::capture(
                flight, dynamics, altitude, footprint, 1000.0, 1000.0, 0.0),
            nullptr);
}

TEST(ExecutionEvidence3DTest, ExecutionInputOwnsExactStateControlAndProvenanceValues) {
  ExecutionInputCapture3D capture = validExecutionInputCapture();
  const auto input = VersionedExecutionInput3D::capture(capture);
  const auto repeated = VersionedExecutionInput3D::capture(capture);
  ASSERT_NE(input, nullptr);
  ASSERT_NE(repeated, nullptr);
  ASSERT_TRUE(input->valid());
  EXPECT_TRUE(input->nominalStateAuthoritative());
  EXPECT_FALSE(input->stationaryCaptureStateAuthoritative());
  EXPECT_EQ(input->purpose(), ExecutionInputPurpose3D::kGeneralExecution);
  EXPECT_TRUE(input->stateProvenance().yawAuthoritative());
  EXPECT_EQ(input->captureSequence(), 17U);
  EXPECT_EQ(input->poseRevision(), 23U);
  EXPECT_EQ(input->poseSourceTimestampUs(), 5'000'000U);
  EXPECT_EQ(input->poseReceiveStampNs(), 5'010'000'000);
  EXPECT_EQ(input->effectiveStampNs(), 5'020'000'000);
  EXPECT_EQ(input->previousControlSourceProducerInstanceId(), 29U);
  EXPECT_EQ(input->previousControlSourceSequence(), 31U);
  EXPECT_EQ(input->previousControlSourceStampNs(), 5'015'000'000);
  EXPECT_EQ(input->previousControlReceiveStampNs(), 5'016'000'000);
  EXPECT_EQ(input->contentFingerprint(), repeated->contentFingerprint());

  capture.state.yaw = 1.5F;
  capture.state_provenance.yaw = ExecutionStateFieldProvenance3D::kAssumedZero;
  capture.previous_control.ax = 9.0F;
  EXPECT_FLOAT_EQ(input->state().yaw, 0.25F);
  EXPECT_EQ(input->stateProvenance().yaw,
            ExecutionStateFieldProvenance3D::kSourceSample);
  EXPECT_FLOAT_EQ(input->previousControl().ax, 0.1F);
  EXPECT_EQ(input->previousControlSource(),
            ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback);
}

TEST(ExecutionEvidence3DTest,
     ExecutionInputAllowsExplicitNonAuthoritativeStateButRejectsFalseAuthority) {
  ExecutionInputCapture3D capture = validExecutionInputCapture();
  capture.full_state_authoritative = false;
  capture.state.yaw_rate = 0.0F;
  capture.state_provenance.yaw_rate = ExecutionStateFieldProvenance3D::kAssumedZero;
  const auto non_authoritative = VersionedExecutionInput3D::capture(capture);
  ASSERT_NE(non_authoritative, nullptr);
  EXPECT_TRUE(non_authoritative->valid());
  EXPECT_FALSE(non_authoritative->fullStateAuthoritative());
  EXPECT_FALSE(non_authoritative->stateProvenance().yawAuthoritative());
  EXPECT_FALSE(non_authoritative->nominalStateAuthoritative());

  capture.full_state_authoritative = true;
  EXPECT_EQ(VersionedExecutionInput3D::capture(capture), nullptr);
}

TEST(ExecutionEvidence3DTest, ExecutionInputRejectsInvalidStateControlAndSource) {
  ExecutionInputCapture3D capture = validExecutionInputCapture();
  capture.state.vx = std::numeric_limits<float>::infinity();
  EXPECT_EQ(VersionedExecutionInput3D::capture(capture), nullptr);

  capture = validExecutionInputCapture();
  capture.previous_control.az = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(VersionedExecutionInput3D::capture(capture), nullptr);

  for (const ExecutionPreviousControlEvidenceSource3D source : {
           ExecutionPreviousControlEvidenceSource3D::kUnknown,
           ExecutionPreviousControlEvidenceSource3D::kAssumedZero,
           ExecutionPreviousControlEvidenceSource3D::kEngineFallback,
       }) {
    capture = validExecutionInputCapture();
    capture.previous_control_source = source;
    EXPECT_EQ(VersionedExecutionInput3D::capture(capture), nullptr);
  }

  capture = validExecutionInputCapture();
  capture.pose_receive_stamp_ns = capture.effective_stamp_ns + 1;
  EXPECT_EQ(VersionedExecutionInput3D::capture(capture), nullptr);

  capture = validExecutionInputCapture();
  capture.previous_control_source_producer_instance_id = 0U;
  EXPECT_EQ(VersionedExecutionInput3D::capture(capture), nullptr);

  capture = validExecutionInputCapture();
  capture.previous_control_source =
      ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration;
  capture.previous_control_source_producer_instance_id = 0U;
  EXPECT_NE(VersionedExecutionInput3D::capture(capture), nullptr);
  capture.previous_control_source_producer_instance_id = 29U;
  EXPECT_EQ(VersionedExecutionInput3D::capture(capture), nullptr);

  capture = validExecutionInputCapture();
  capture.previous_control_source_sequence = 0U;
  EXPECT_EQ(VersionedExecutionInput3D::capture(capture), nullptr);
}

TEST(ExecutionEvidence3DTest,
     StationaryCaptureInputAllowsOnlyNamedExactZeroControlAuthority) {
  ExecutionInputCapture3D capture = validExecutionInputCapture();
  capture.state.vx = 0.1F;
  capture.state.vy = 0.0F;
  capture.state.vz = 0.0F;
  capture.state.yaw_rate = 0.1F;
  capture.state_provenance = ExecutionStateProvenance3D{
      .x = ExecutionStateFieldProvenance3D::kSourceSample,
      .y = ExecutionStateFieldProvenance3D::kSourceSample,
      .z = ExecutionStateFieldProvenance3D::kSourceSample,
      .vx = ExecutionStateFieldProvenance3D::kSourceSample,
      .vy = ExecutionStateFieldProvenance3D::kSourceSample,
      .vz = ExecutionStateFieldProvenance3D::kSourceSample,
      .yaw = ExecutionStateFieldProvenance3D::kSourceSample,
      .yaw_rate = ExecutionStateFieldProvenance3D::kSourceSample,
  };
  capture.purpose = ExecutionInputPurpose3D::kStationaryCaptureRearm;
  capture.previous_control = {};
  capture.previous_control_source =
      ExecutionPreviousControlEvidenceSource3D::kAssumedZero;
  capture.previous_control_source_producer_instance_id = 0U;
  capture.previous_control_source_sequence = capture.capture_sequence;
  capture.previous_control_source_stamp_ns = capture.effective_stamp_ns;
  capture.previous_control_receive_stamp_ns = capture.effective_stamp_ns;

  const auto stationary_capture = VersionedExecutionInput3D::capture(capture);

  ASSERT_NE(stationary_capture, nullptr);
  EXPECT_TRUE(stationary_capture->valid());
  EXPECT_TRUE(stationary_capture->stationaryCaptureStateAuthoritative());
  EXPECT_FALSE(stationary_capture->nominalStateAuthoritative());
  EXPECT_EQ(stationary_capture->purpose(),
            ExecutionInputPurpose3D::kStationaryCaptureRearm);

  ExecutionInputCapture3D invalid = capture;
  invalid.previous_control.ax = 0.01F;
  EXPECT_EQ(VersionedExecutionInput3D::capture(invalid), nullptr);
  invalid = capture;
  invalid.previous_control_source =
      ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration;
  EXPECT_EQ(VersionedExecutionInput3D::capture(invalid), nullptr);
  invalid = capture;
  invalid.previous_control_source_sequence += 1U;
  EXPECT_EQ(VersionedExecutionInput3D::capture(invalid), nullptr);
  invalid = capture;
  --invalid.previous_control_source_stamp_ns;
  EXPECT_EQ(VersionedExecutionInput3D::capture(invalid), nullptr);
  invalid = capture;
  invalid.state_provenance.vx =
      ExecutionStateFieldProvenance3D::kEffectiveTimePrediction;
  EXPECT_EQ(VersionedExecutionInput3D::capture(invalid), nullptr);
}

TEST(ExecutionEvidence3DTest,
     SameExecutionInputIdentityWithDifferentContentHasDifferentFingerprint) {
  ExecutionInputCapture3D first_capture = validExecutionInputCapture();
  ExecutionInputCapture3D second_capture = first_capture;
  second_capture.state.yaw = 0.5F;
  second_capture.previous_control.yaw_accel = 0.8F;
  second_capture.previous_control_source_producer_instance_id = 37U;

  const auto first = VersionedExecutionInput3D::capture(first_capture);
  const auto second = VersionedExecutionInput3D::capture(second_capture);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(first->captureSequence(), second->captureSequence());
  EXPECT_EQ(first->poseRevision(), second->poseRevision());
  EXPECT_NE(first->contentFingerprint(), second->contentFingerprint());
}

TEST(ExecutionEvidence3DTest, LatestLidarEvidenceDeepCopiesPointsAndHashesAllContent) {
  LatestLidarEvidenceCapture3D capture = validLatestLidarCapture();
  const auto lidar = VersionedLatestLidarEvidence3D::capture(capture);
  const auto repeated = VersionedLatestLidarEvidence3D::capture(capture);
  ASSERT_NE(lidar, nullptr);
  ASSERT_NE(repeated, nullptr);
  ASSERT_TRUE(lidar->valid());
  ASSERT_EQ(lidar->hitPointsMapM().size(), 2U);
  EXPECT_EQ(lidar->producerInstanceId(), 41U);
  EXPECT_EQ(lidar->sequence(), 43U);
  EXPECT_EQ(lidar->poseGeneration(), 47U);
  EXPECT_EQ(lidar->acquisitionStampNs(), 6'000'000'000);
  EXPECT_EQ(lidar->receiveStampNs(), 6'010'000'000);
  EXPECT_EQ(lidar->sourceBeamCount(), 5U);
  EXPECT_EQ(lidar->invalidBeamCount(), 1U);
  EXPECT_TRUE(lidar->evidenceId().valid());
  EXPECT_EQ(lidar->evidenceId(), repeated->evidenceId());
  EXPECT_DOUBLE_EQ(lidar->hitPointsMapM().front().x, 1.0);
  EXPECT_EQ(lidar->contentFingerprint(), repeated->contentFingerprint());

  capture.hit_points_map_m.front().x = 9.0;
  const auto changed = VersionedLatestLidarEvidence3D::capture(capture);
  ASSERT_NE(changed, nullptr);
  EXPECT_DOUBLE_EQ(lidar->hitPointsMapM().front().x, 1.0);
  EXPECT_EQ(lidar->producerInstanceId(), changed->producerInstanceId());
  EXPECT_EQ(lidar->sequence(), changed->sequence());
  EXPECT_EQ(lidar->evidenceId(), changed->evidenceId());
  EXPECT_NE(lidar->contentFingerprint(), changed->contentFingerprint());
}

TEST(ExecutionEvidence3DTest,
     LatestLidarEvidenceMoveCaptureKeepsTheOriginalPointStorage) {
  LatestLidarEvidenceCapture3D capture = validLatestLidarCapture();
  const Point3* const original_storage = capture.hit_points_map_m.data();

  const auto lidar = VersionedLatestLidarEvidence3D::capture(std::move(capture));

  ASSERT_NE(lidar, nullptr);
  EXPECT_EQ(lidar->hitPointsMapM().data(), original_storage);
}

TEST(ExecutionEvidence3DTest,
     LatestLidarEvidenceUpdateRejectsReplayAndDoesNotRejuvenateDuplicates) {
  const auto initial =
      VersionedLatestLidarEvidence3D::capture(validLatestLidarCapture());
  ASSERT_NE(initial, nullptr);
  EXPECT_EQ(assessLatestLidarEvidenceUpdate3D(nullptr, *initial),
            LatestLidarEvidenceUpdateStatus3D::kAcceptedInitial);

  LatestLidarEvidenceCapture3D duplicate_capture = validLatestLidarCapture();
  duplicate_capture.receive_stamp_ns += 1'000'000'000;
  const auto duplicate =
      VersionedLatestLidarEvidence3D::capture(std::move(duplicate_capture));
  ASSERT_NE(duplicate, nullptr);
  EXPECT_NE(duplicate->contentFingerprint(), initial->contentFingerprint());
  EXPECT_EQ(duplicate->sourceContentFingerprint(), initial->sourceContentFingerprint());
  EXPECT_EQ(assessLatestLidarEvidenceUpdate3D(initial.get(), *duplicate),
            LatestLidarEvidenceUpdateStatus3D::kIdempotentDuplicate);

  LatestLidarEvidenceCapture3D conflict_capture = validLatestLidarCapture();
  conflict_capture.hit_points_map_m.front().x += 0.5;
  const auto conflict =
      VersionedLatestLidarEvidence3D::capture(std::move(conflict_capture));
  ASSERT_NE(conflict, nullptr);
  EXPECT_EQ(assessLatestLidarEvidenceUpdate3D(initial.get(), *conflict),
            LatestLidarEvidenceUpdateStatus3D::kRejectedIdentityConflict);

  LatestLidarEvidenceCapture3D timestamp_conflict_capture = validLatestLidarCapture();
  ++timestamp_conflict_capture.acquisition_stamp_ns;
  const auto timestamp_conflict =
      VersionedLatestLidarEvidence3D::capture(std::move(timestamp_conflict_capture));
  ASSERT_NE(timestamp_conflict, nullptr);
  EXPECT_EQ(assessLatestLidarEvidenceUpdate3D(initial.get(), *timestamp_conflict),
            LatestLidarEvidenceUpdateStatus3D::kRejectedIdentityConflict);

  LatestLidarEvidenceCapture3D regression_capture = validLatestLidarCapture();
  --regression_capture.sequence;
  --regression_capture.acquisition_stamp_ns;
  const auto regression =
      VersionedLatestLidarEvidence3D::capture(std::move(regression_capture));
  ASSERT_NE(regression, nullptr);
  EXPECT_EQ(assessLatestLidarEvidenceUpdate3D(initial.get(), *regression),
            LatestLidarEvidenceUpdateStatus3D::kRejectedRegression);

  LatestLidarEvidenceCapture3D newer_capture = validLatestLidarCapture();
  ++newer_capture.sequence;
  ++newer_capture.pose_generation;
  newer_capture.acquisition_stamp_ns -= 1'000'000;
  ++newer_capture.receive_stamp_ns;
  const auto newer = VersionedLatestLidarEvidence3D::capture(std::move(newer_capture));
  ASSERT_NE(newer, nullptr);
  EXPECT_EQ(assessLatestLidarEvidenceUpdate3D(initial.get(), *newer),
            LatestLidarEvidenceUpdateStatus3D::kAcceptedNewer);

  LatestLidarEvidenceCapture3D new_producer_capture = validLatestLidarCapture();
  ++new_producer_capture.producer_instance_id;
  ++new_producer_capture.sequence;
  ++new_producer_capture.acquisition_stamp_ns;
  const auto new_producer =
      VersionedLatestLidarEvidence3D::capture(std::move(new_producer_capture));
  ASSERT_NE(new_producer, nullptr);
  EXPECT_EQ(assessLatestLidarEvidenceUpdate3D(initial.get(), *new_producer),
            LatestLidarEvidenceUpdateStatus3D::kRejectedUnauthenticatedProducer);
  EXPECT_EQ(latestLidarEvidenceUpdateStatus3DName(
                LatestLidarEvidenceUpdateStatus3D::kAcceptedNewer),
            "accepted_newer");
  EXPECT_EQ(latestLidarEvidenceUpdateStatus3DName(
                static_cast<LatestLidarEvidenceUpdateStatus3D>(255U)),
            "unknown");
}

TEST(ExecutionEvidence3DTest,
     LatestLidarEvidenceFutureAcquisitionDoesNotPoisonSequenceAdmission) {
  LatestLidarEvidenceAdmissionState3D state;
  const auto initial =
      VersionedLatestLidarEvidence3D::capture(validLatestLidarCapture());
  ASSERT_NE(initial, nullptr);
  const LatestLidarEvidenceAdmissionResult3D initial_admission =
      admitLatestLidarEvidence3D(state, nullptr, *initial, 6'020'000'000, 100.0);
  ASSERT_TRUE(initial_admission.install_candidate);
  state = initial_admission.next_state;

  LatestLidarEvidenceCapture3D future_capture = validLatestLidarCapture();
  ++future_capture.sequence;
  ++future_capture.pose_generation;
  future_capture.acquisition_stamp_ns = 60'000'000'000;
  future_capture.receive_stamp_ns = 6'020'000'000;
  const auto future =
      VersionedLatestLidarEvidence3D::capture(std::move(future_capture));
  ASSERT_NE(future, nullptr);
  const LatestLidarEvidenceAdmissionResult3D future_admission =
      admitLatestLidarEvidence3D(state, initial.get(), *future, 6'020'000'000, 100.0);
  ASSERT_EQ(future_admission.status, LatestLidarEvidenceUpdateStatus3D::kAcceptedNewer);
  ASSERT_TRUE(future_admission.install_candidate);
  state = future_admission.next_state;

  LatestLidarEvidenceCapture3D recovered_capture = validLatestLidarCapture();
  recovered_capture.sequence += 2U;
  recovered_capture.pose_generation += 2U;
  recovered_capture.acquisition_stamp_ns = 6'025'000'000;
  recovered_capture.receive_stamp_ns = 6'030'000'000;
  const auto recovered =
      VersionedLatestLidarEvidence3D::capture(std::move(recovered_capture));
  ASSERT_NE(recovered, nullptr);
  const LatestLidarEvidenceAdmissionResult3D recovered_admission =
      admitLatestLidarEvidence3D(state, future.get(), *recovered, 6'030'000'000, 100.0);
  EXPECT_EQ(recovered_admission.status,
            LatestLidarEvidenceUpdateStatus3D::kAcceptedAcquisitionEpochReset);
  EXPECT_TRUE(recovered_admission.install_candidate);
  EXPECT_TRUE(recovered_admission.acquisition_epoch_reset);
  EXPECT_FALSE(recovered_admission.producer_handoff);
  EXPECT_TRUE(
      assessLatestLidarEvidenceFreshness3D(*recovered, 6'030'000'000, 100.0).fresh);
}

TEST(ExecutionEvidence3DTest,
     LatestLidarIdentityConflictQuarantinesCurrentUntilStrictlyNewerSequence) {
  LatestLidarEvidenceAdmissionState3D state;
  const auto current =
      VersionedLatestLidarEvidence3D::capture(validLatestLidarCapture());
  ASSERT_NE(current, nullptr);
  const LatestLidarEvidenceAdmissionResult3D initial =
      admitLatestLidarEvidence3D(state, nullptr, *current, 6'020'000'000, 100.0);
  ASSERT_TRUE(initial.install_candidate);
  state = initial.next_state;

  const LatestLidarEvidenceClaimResult3D conflicted =
      claimLatestLidarEvidenceIdentity3D(
          state, current.get(),
          LatestLidarEvidenceIdentityClaim3D{
              .producer_instance_id = current->producerInstanceId(),
              .sequence = current->sequence(),
              .raw_wire_fingerprint =
                  state.current_raw_wire_fingerprint == 1U ? 2U : 1U,
              .first_receive_stamp_ns = 6'020'000'000,
          });
  EXPECT_EQ(conflicted.status,
            LatestLidarEvidenceClaimStatus3D::kRejectedIdentityConflict);
  EXPECT_TRUE(conflicted.authority_quarantine_opened);
  EXPECT_TRUE(conflicted.next_state.current_identity_conflicted);
  EXPECT_TRUE(latestLidarEvidenceAuthorityQuarantined3D(conflicted.next_state));
  EXPECT_FALSE(conflicted.assess_candidate);

  const LatestLidarEvidenceClaimResult3D replayed_original =
      claimLatestLidarEvidenceIdentity3D(
          conflicted.next_state, current.get(),
          LatestLidarEvidenceIdentityClaim3D{
              .producer_instance_id = current->producerInstanceId(),
              .sequence = current->sequence(),
              .raw_wire_fingerprint = state.current_raw_wire_fingerprint,
              .first_receive_stamp_ns = 6'030'000'000,
          });
  EXPECT_EQ(replayed_original.status,
            LatestLidarEvidenceClaimStatus3D::kRejectedIdentityConflict);
  EXPECT_FALSE(replayed_original.authority_quarantine_opened);
  EXPECT_TRUE(replayed_original.next_state.current_identity_conflicted);
  EXPECT_FALSE(replayed_original.assess_candidate);

  LatestLidarEvidenceCapture3D newer_capture = validLatestLidarCapture();
  ++newer_capture.sequence;
  ++newer_capture.pose_generation;
  newer_capture.acquisition_stamp_ns = 6'030'000'000;
  newer_capture.receive_stamp_ns = 6'030'000'000;
  const auto newer = VersionedLatestLidarEvidence3D::capture(std::move(newer_capture));
  ASSERT_NE(newer, nullptr);
  const LatestLidarEvidenceAdmissionResult3D recovered = admitLatestLidarEvidence3D(
      replayed_original.next_state, current.get(), *newer, 6'030'000'000, 100.0);
  EXPECT_EQ(recovered.status, LatestLidarEvidenceUpdateStatus3D::kAcceptedNewer);
  EXPECT_TRUE(recovered.install_candidate);
  EXPECT_FALSE(recovered.next_state.current_identity_conflicted);
}

TEST(ExecutionEvidence3DTest,
     LatestLidarEvidenceHandoffRequiresStalenessAndTombstonesRetiredProducer) {
  LatestLidarEvidenceAdmissionState3D state;
  const auto first = VersionedLatestLidarEvidence3D::capture(validLatestLidarCapture());
  ASSERT_NE(first, nullptr);
  const LatestLidarEvidenceAdmissionResult3D initial =
      admitLatestLidarEvidence3D(state, nullptr, *first, 6'020'000'000, 20.0);
  ASSERT_TRUE(initial.install_candidate);
  state = initial.next_state;

  LatestLidarEvidenceCapture3D restart_capture = validLatestLidarCapture();
  restart_capture.producer_instance_id = 53U;
  restart_capture.sequence = 1U;
  restart_capture.pose_generation = 1U;
  restart_capture.acquisition_stamp_ns = 6'019'000'000;
  restart_capture.receive_stamp_ns = 6'019'000'000;
  const auto premature = VersionedLatestLidarEvidence3D::capture(restart_capture);
  ASSERT_NE(premature, nullptr);
  const LatestLidarEvidenceAdmissionResult3D fresh_owner_rejection =
      admitLatestLidarEvidence3D(state, first.get(), *premature, 6'019'000'000, 20.0);
  EXPECT_EQ(fresh_owner_rejection.status,
            LatestLidarEvidenceUpdateStatus3D::kRejectedUnauthenticatedProducer);
  EXPECT_FALSE(fresh_owner_rejection.install_candidate);
  ASSERT_EQ(fresh_owner_rejection.next_state.prospective_claim_count, 1U);
  EXPECT_EQ(fresh_owner_rejection.next_state.prospective_claims.front()
                .identity.first_receive_stamp_ns,
            6'019'000'000);
  state = fresh_owner_rejection.next_state;

  ++restart_capture.sequence;
  ++restart_capture.pose_generation;
  restart_capture.acquisition_stamp_ns = 6'040'000'000;
  restart_capture.receive_stamp_ns = 6'040'000'000;
  const auto probation = VersionedLatestLidarEvidence3D::capture(restart_capture);
  ASSERT_NE(probation, nullptr);
  const LatestLidarEvidenceAdmissionResult3D pending =
      admitLatestLidarEvidence3D(state, first.get(), *probation, 6'040'000'000, 20.0);
  ASSERT_EQ(pending.status, LatestLidarEvidenceUpdateStatus3D::kPendingProducerHandoff);
  ASSERT_FALSE(pending.install_candidate);
  state = pending.next_state;

  ++restart_capture.sequence;
  ++restart_capture.pose_generation;
  restart_capture.acquisition_stamp_ns = 6'050'000'000;
  restart_capture.receive_stamp_ns = 6'050'000'000;
  const auto confirmed = VersionedLatestLidarEvidence3D::capture(restart_capture);
  ASSERT_NE(confirmed, nullptr);
  const LatestLidarEvidenceAdmissionResult3D handoff =
      admitLatestLidarEvidence3D(state, first.get(), *confirmed, 6'050'000'000, 20.0);
  ASSERT_EQ(handoff.status,
            LatestLidarEvidenceUpdateStatus3D::kAcceptedProducerHandoff);
  ASSERT_TRUE(handoff.install_candidate);
  ASSERT_TRUE(handoff.producer_handoff);
  ASSERT_EQ(handoff.next_state.current_producer_instance_id, 53U);
  ASSERT_EQ(handoff.next_state.retired_producer_count, 1U);
  ASSERT_EQ(handoff.next_state.retired_producer_instance_ids.front(), 41U);
  state = handoff.next_state;

  LatestLidarEvidenceCapture3D replay_capture = validLatestLidarCapture();
  replay_capture.sequence = 1'000U;
  replay_capture.pose_generation = 1'000U;
  replay_capture.acquisition_stamp_ns = 6'100'000'000;
  replay_capture.receive_stamp_ns = 6'100'000'000;
  const auto retired_replay =
      VersionedLatestLidarEvidence3D::capture(std::move(replay_capture));
  ASSERT_NE(retired_replay, nullptr);
  const LatestLidarEvidenceAdmissionResult3D replay_rejection =
      admitLatestLidarEvidence3D(state, confirmed.get(), *retired_replay, 6'100'000'000,
                                 20.0);
  EXPECT_EQ(replay_rejection.status,
            LatestLidarEvidenceUpdateStatus3D::kRejectedRetiredProducer);
  EXPECT_FALSE(replay_rejection.install_candidate);
}

TEST(ExecutionEvidence3DTest,
     LatestLidarEvidenceHandoffProbationRejectsReorderAndIdentityConflict) {
  LatestLidarEvidenceAdmissionState3D state;
  const auto current =
      VersionedLatestLidarEvidence3D::capture(validLatestLidarCapture());
  ASSERT_NE(current, nullptr);
  state = admitLatestLidarEvidence3D(state, nullptr, *current, 6'020'000'000, 20.0)
              .next_state;

  LatestLidarEvidenceCapture3D candidate_capture = validLatestLidarCapture();
  candidate_capture.producer_instance_id = 59U;
  candidate_capture.sequence = 7U;
  candidate_capture.acquisition_stamp_ns = 6'040'000'000;
  candidate_capture.receive_stamp_ns = 6'040'000'000;
  const auto candidate = VersionedLatestLidarEvidence3D::capture(candidate_capture);
  ASSERT_NE(candidate, nullptr);
  const LatestLidarEvidenceAdmissionResult3D pending =
      admitLatestLidarEvidence3D(state, current.get(), *candidate, 6'040'000'000, 20.0);
  ASSERT_EQ(pending.status, LatestLidarEvidenceUpdateStatus3D::kPendingProducerHandoff);

  candidate_capture.sequence = 6U;
  candidate_capture.receive_stamp_ns = 6'041'000'000;
  const auto reordered = VersionedLatestLidarEvidence3D::capture(candidate_capture);
  ASSERT_NE(reordered, nullptr);
  EXPECT_EQ(admitLatestLidarEvidence3D(pending.next_state, current.get(), *reordered,
                                       6'041'000'000, 20.0)
                .status,
            LatestLidarEvidenceUpdateStatus3D::kRejectedRegression);

  candidate_capture.sequence = 7U;
  candidate_capture.hit_points_map_m.front().x += 1.0;
  const auto conflict = VersionedLatestLidarEvidence3D::capture(candidate_capture);
  ASSERT_NE(conflict, nullptr);
  const LatestLidarEvidenceAdmissionResult3D conflicted = admitLatestLidarEvidence3D(
      pending.next_state, current.get(), *conflict, 6'041'000'000, 20.0);
  EXPECT_EQ(conflicted.status,
            LatestLidarEvidenceUpdateStatus3D::kRejectedIdentityConflict);
  EXPECT_EQ(conflicted.next_state.pending_producer_instance_id, 59U);
  EXPECT_EQ(conflicted.next_state.pending_sequence, 7U);
  EXPECT_EQ(conflicted.next_state.pending_confirmation_count, 1U);
  EXPECT_TRUE(conflicted.next_state.pending_identity_conflicted);

  const LatestLidarEvidenceAdmissionResult3D original_replay =
      admitLatestLidarEvidence3D(conflicted.next_state, current.get(), *candidate,
                                 6'041'000'000, 20.0);
  EXPECT_EQ(original_replay.status,
            LatestLidarEvidenceUpdateStatus3D::kRejectedIdentityConflict);
  EXPECT_TRUE(original_replay.next_state.pending_identity_conflicted);

  candidate_capture.sequence = 8U;
  ++candidate_capture.pose_generation;
  candidate_capture.acquisition_stamp_ns = 6'050'000'000;
  candidate_capture.receive_stamp_ns = 6'050'000'000;
  const auto higher = VersionedLatestLidarEvidence3D::capture(candidate_capture);
  ASSERT_NE(higher, nullptr);
  const LatestLidarEvidenceAdmissionResult3D restarted = admitLatestLidarEvidence3D(
      original_replay.next_state, current.get(), *higher, 6'050'000'000, 20.0);
  EXPECT_EQ(restarted.status,
            LatestLidarEvidenceUpdateStatus3D::kPendingProducerHandoff);
  EXPECT_EQ(restarted.next_state.pending_sequence, 8U);
  EXPECT_FALSE(restarted.next_state.pending_identity_conflicted);
}

TEST(ExecutionEvidence3DTest,
     LatestLidarEvidenceExpiredHandoffProbationNeedsFreshConfirmation) {
  LatestLidarEvidenceAdmissionState3D state;
  const auto current =
      VersionedLatestLidarEvidence3D::capture(validLatestLidarCapture());
  ASSERT_NE(current, nullptr);
  state = admitLatestLidarEvidence3D(state, nullptr, *current, 6'020'000'000, 20.0)
              .next_state;

  LatestLidarEvidenceCapture3D candidate_capture = validLatestLidarCapture();
  candidate_capture.producer_instance_id = 67U;
  candidate_capture.sequence = 1U;
  candidate_capture.pose_generation = 1U;
  candidate_capture.acquisition_stamp_ns = 6'040'000'000;
  candidate_capture.receive_stamp_ns = 6'040'000'000;
  const auto first = VersionedLatestLidarEvidence3D::capture(candidate_capture);
  ASSERT_NE(first, nullptr);
  const LatestLidarEvidenceAdmissionResult3D first_pending =
      admitLatestLidarEvidence3D(state, current.get(), *first, 6'040'000'000, 20.0);
  ASSERT_EQ(first_pending.status,
            LatestLidarEvidenceUpdateStatus3D::kPendingProducerHandoff);

  ++candidate_capture.sequence;
  ++candidate_capture.pose_generation;
  candidate_capture.acquisition_stamp_ns = 6'070'000'000;
  candidate_capture.receive_stamp_ns = 6'070'000'000;
  const auto expired_confirmation =
      VersionedLatestLidarEvidence3D::capture(candidate_capture);
  ASSERT_NE(expired_confirmation, nullptr);
  const LatestLidarEvidenceAdmissionResult3D restarted =
      admitLatestLidarEvidence3D(first_pending.next_state, current.get(),
                                 *expired_confirmation, 6'070'000'000, 20.0);
  EXPECT_EQ(restarted.status,
            LatestLidarEvidenceUpdateStatus3D::kPendingProducerHandoff);
  EXPECT_FALSE(restarted.install_candidate);
  EXPECT_EQ(restarted.next_state.pending_confirmation_count, 1U);
  EXPECT_EQ(restarted.next_state.pending_sequence, 2U);

  ++candidate_capture.sequence;
  ++candidate_capture.pose_generation;
  candidate_capture.acquisition_stamp_ns = 6'080'000'000;
  candidate_capture.receive_stamp_ns = 6'080'000'000;
  const auto fresh_confirmation =
      VersionedLatestLidarEvidence3D::capture(std::move(candidate_capture));
  ASSERT_NE(fresh_confirmation, nullptr);
  const LatestLidarEvidenceAdmissionResult3D handoff = admitLatestLidarEvidence3D(
      restarted.next_state, current.get(), *fresh_confirmation, 6'080'000'000, 20.0);
  EXPECT_EQ(handoff.status,
            LatestLidarEvidenceUpdateStatus3D::kAcceptedProducerHandoff);
  EXPECT_TRUE(handoff.install_candidate);
  EXPECT_TRUE(handoff.producer_handoff);
}

TEST(ExecutionEvidence3DTest, LatestLidarEvidenceTombstoneCapacityFailsClosed) {
  const auto current =
      VersionedLatestLidarEvidence3D::capture(validLatestLidarCapture());
  ASSERT_NE(current, nullptr);
  LatestLidarEvidenceAdmissionState3D state =
      admitLatestLidarEvidence3D({}, nullptr, *current, 6'020'000'000, 20.0).next_state;
  state.retired_producer_count = state.retired_producer_instance_ids.size();
  for (std::size_t index = 0U; index < state.retired_producer_instance_ids.size();
       ++index) {
    state.retired_producer_instance_ids[index] = 100U + index;
  }

  LatestLidarEvidenceCapture3D candidate_capture = validLatestLidarCapture();
  candidate_capture.producer_instance_id = 61U;
  candidate_capture.sequence = 1U;
  candidate_capture.acquisition_stamp_ns = 6'040'000'000;
  candidate_capture.receive_stamp_ns = 6'040'000'000;
  const auto candidate =
      VersionedLatestLidarEvidence3D::capture(std::move(candidate_capture));
  ASSERT_NE(candidate, nullptr);
  const LatestLidarEvidenceAdmissionResult3D admission =
      admitLatestLidarEvidence3D(state, current.get(), *candidate, 6'040'000'000, 20.0);
  EXPECT_EQ(admission.status,
            LatestLidarEvidenceUpdateStatus3D::kRejectedHandoffCapacity);
  EXPECT_FALSE(admission.install_candidate);
  EXPECT_EQ(admission.next_state.retired_producer_count,
            kLatestLidarRetiredProducerCapacity3D);
}

TEST(ExecutionEvidence3DTest,
     LatestLidarMalformedClaimCannotRejuvenateAndMutationNeedsHigherSequence) {
  const LatestLidarEvidenceIdentityClaim3D malformed{
      .producer_instance_id = 71U,
      .sequence = 1U,
      .raw_wire_fingerprint = 101U,
      .first_receive_stamp_ns = 6'010'000'000,
  };
  const LatestLidarEvidenceClaimResult3D claimed =
      claimLatestLidarEvidenceIdentity3D({}, nullptr, malformed);
  ASSERT_EQ(claimed.status, LatestLidarEvidenceClaimStatus3D::kClaimed);
  ASSERT_TRUE(claimed.assess_candidate);
  ASSERT_EQ(claimed.next_state.prospective_claim_count, 1U);

  LatestLidarEvidenceIdentityClaim3D retransmission = malformed;
  retransmission.first_receive_stamp_ns = 6'020'000'000;
  const LatestLidarEvidenceClaimResult3D replay =
      claimLatestLidarEvidenceIdentity3D(claimed.next_state, nullptr, retransmission);
  EXPECT_EQ(replay.status, LatestLidarEvidenceClaimStatus3D::kRejectedReplay);
  EXPECT_FALSE(replay.assess_candidate);
  EXPECT_EQ(replay.claim.first_receive_stamp_ns, malformed.first_receive_stamp_ns);

  LatestLidarEvidenceIdentityClaim3D mutation = retransmission;
  mutation.raw_wire_fingerprint = 103U;
  const LatestLidarEvidenceClaimResult3D conflicted =
      claimLatestLidarEvidenceIdentity3D(replay.next_state, nullptr, mutation);
  EXPECT_EQ(conflicted.status,
            LatestLidarEvidenceClaimStatus3D::kRejectedIdentityConflict);
  EXPECT_TRUE(conflicted.next_state.prospective_claims.front().conflicted);
  EXPECT_EQ(
      claimLatestLidarEvidenceIdentity3D(conflicted.next_state, nullptr, malformed)
          .status,
      LatestLidarEvidenceClaimStatus3D::kRejectedIdentityConflict);

  LatestLidarEvidenceIdentityClaim3D higher = mutation;
  ++higher.sequence;
  higher.raw_wire_fingerprint = 107U;
  higher.first_receive_stamp_ns = 6'030'000'000;
  const LatestLidarEvidenceClaimResult3D recovered_claim =
      claimLatestLidarEvidenceIdentity3D(conflicted.next_state, nullptr, higher);
  ASSERT_EQ(recovered_claim.status, LatestLidarEvidenceClaimStatus3D::kClaimed);
  EXPECT_FALSE(recovered_claim.next_state.prospective_claims.front().conflicted);

  LatestLidarEvidenceCapture3D capture = validLatestLidarCapture();
  capture.producer_instance_id = higher.producer_instance_id;
  capture.sequence = higher.sequence;
  capture.acquisition_stamp_ns = 6'030'000'000;
  capture.receive_stamp_ns = higher.first_receive_stamp_ns;
  const auto evidence = VersionedLatestLidarEvidence3D::capture(std::move(capture));
  ASSERT_NE(evidence, nullptr);
  const LatestLidarEvidenceAdmissionResult3D recovered =
      admitClaimedLatestLidarEvidence3D(recovered_claim.next_state, nullptr, *evidence,
                                        recovered_claim.claim, 6'030'000'000, 20.0);
  EXPECT_TRUE(recovered.install_candidate);
  EXPECT_EQ(recovered.next_state.current_raw_wire_fingerprint,
            higher.raw_wire_fingerprint);
}

TEST(ExecutionEvidence3DTest, LatestLidarStaleRejectedReplayPreservesOriginalReceipt) {
  const LatestLidarEvidenceIdentityClaim3D identity{
      .producer_instance_id = 73U,
      .sequence = 1U,
      .raw_wire_fingerprint = 109U,
      .first_receive_stamp_ns = 6'010'000'000,
  };
  const LatestLidarEvidenceClaimResult3D claimed =
      claimLatestLidarEvidenceIdentity3D({}, nullptr, identity);
  ASSERT_TRUE(claimed.assess_candidate);
  LatestLidarEvidenceCapture3D capture = validLatestLidarCapture();
  capture.producer_instance_id = identity.producer_instance_id;
  capture.sequence = identity.sequence;
  capture.receive_stamp_ns = identity.first_receive_stamp_ns;
  const auto evidence = VersionedLatestLidarEvidence3D::capture(std::move(capture));
  ASSERT_NE(evidence, nullptr);
  const LatestLidarEvidenceAdmissionResult3D stale = admitClaimedLatestLidarEvidence3D(
      claimed.next_state, nullptr, *evidence, claimed.claim, 6'050'000'000, 20.0);
  ASSERT_EQ(stale.status, LatestLidarEvidenceUpdateStatus3D::kRejectedStaleCandidate);

  LatestLidarEvidenceIdentityClaim3D retransmission = identity;
  retransmission.first_receive_stamp_ns = 6'050'000'000;
  const LatestLidarEvidenceClaimResult3D replay =
      claimLatestLidarEvidenceIdentity3D(stale.next_state, nullptr, retransmission);
  EXPECT_EQ(replay.status, LatestLidarEvidenceClaimStatus3D::kRejectedReplay);
  EXPECT_FALSE(replay.assess_candidate);
  EXPECT_EQ(replay.claim.first_receive_stamp_ns, identity.first_receive_stamp_ns);
  EXPECT_EQ(replay.next_state.current_producer_instance_id, 0U);
}

TEST(ExecutionEvidence3DTest,
     LatestLidarProspectiveClaimCapacityLatchesAuthorityClosed) {
  LatestLidarEvidenceAdmissionState3D state;
  for (std::size_t index = 0U; index < kLatestLidarProspectiveClaimCapacity3D;
       ++index) {
    const LatestLidarEvidenceClaimResult3D claimed = claimLatestLidarEvidenceIdentity3D(
        state, nullptr,
        LatestLidarEvidenceIdentityClaim3D{
            .producer_instance_id = 100U + index,
            .sequence = 1U,
            .raw_wire_fingerprint = 1'000U + index,
            .first_receive_stamp_ns = 6'000'000'000 + static_cast<std::int64_t>(index),
        });
    ASSERT_EQ(claimed.status, LatestLidarEvidenceClaimStatus3D::kClaimed);
    state = claimed.next_state;
  }
  const LatestLidarEvidenceClaimResult3D exhausted =
      claimLatestLidarEvidenceIdentity3D(state, nullptr,
                                         LatestLidarEvidenceIdentityClaim3D{
                                             .producer_instance_id = 999U,
                                             .sequence = 1U,
                                             .raw_wire_fingerprint = 2'000U,
                                             .first_receive_stamp_ns = 6'100'000'000,
                                         });
  EXPECT_EQ(exhausted.status, LatestLidarEvidenceClaimStatus3D::kRejectedCapacity);
  EXPECT_TRUE(exhausted.authority_quarantine_opened);
  EXPECT_TRUE(latestLidarEvidenceAuthorityQuarantined3D(exhausted.next_state));
  EXPECT_FALSE(exhausted.assess_candidate);

  const LatestLidarEvidenceClaimResult3D later_higher =
      claimLatestLidarEvidenceIdentity3D(exhausted.next_state, nullptr,
                                         LatestLidarEvidenceIdentityClaim3D{
                                             .producer_instance_id = 100U,
                                             .sequence = 2U,
                                             .raw_wire_fingerprint = 3'000U,
                                             .first_receive_stamp_ns = 6'200'000'000,
                                         });
  EXPECT_EQ(later_higher.status, LatestLidarEvidenceClaimStatus3D::kRejectedCapacity);
  EXPECT_FALSE(later_higher.authority_quarantine_opened);
}

TEST(ExecutionEvidence3DTest, LatestLidarEvidenceRejectsInvalidContract) {
  LatestLidarEvidenceCapture3D capture = validLatestLidarCapture();
  capture.producer_instance_id = 0U;
  EXPECT_EQ(VersionedLatestLidarEvidence3D::capture(capture), nullptr);

  capture = validLatestLidarCapture();
  capture.sequence = 0U;
  EXPECT_EQ(VersionedLatestLidarEvidence3D::capture(capture), nullptr);

  capture = validLatestLidarCapture();
  capture.pose_generation = 0U;
  EXPECT_NE(VersionedLatestLidarEvidence3D::capture(capture), nullptr);

  capture = validLatestLidarCapture();
  capture.acquisition_stamp_ns = 0;
  EXPECT_EQ(VersionedLatestLidarEvidence3D::capture(capture), nullptr);

  capture = validLatestLidarCapture();
  capture.receive_stamp_ns = 0;
  EXPECT_EQ(VersionedLatestLidarEvidence3D::capture(capture), nullptr);

  capture = validLatestLidarCapture();
  capture.acquisition_stamp_ns = capture.receive_stamp_ns + 1'000'000'000LL;
  EXPECT_NE(VersionedLatestLidarEvidence3D::capture(capture), nullptr);

  capture = validLatestLidarCapture();
  capture.invalid_beam_count = capture.source_beam_count;
  capture.hit_points_map_m.clear();
  EXPECT_EQ(VersionedLatestLidarEvidence3D::capture(capture), nullptr);

  capture = validLatestLidarCapture();
  capture.hit_points_map_m.front().z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(VersionedLatestLidarEvidence3D::capture(capture), nullptr);
}

TEST(ExecutionEvidence3DTest,
     LatestLidarEvidenceFreshnessUsesBothAcquisitionAndReceiptTimes) {
  const auto evidence =
      VersionedLatestLidarEvidence3D::capture(validLatestLidarCapture());
  ASSERT_NE(evidence, nullptr);

  const LatestLidarEvidenceFreshness3D fresh =
      assessLatestLidarEvidenceFreshness3D(*evidence, 6'020'000'000, 20.0);
  EXPECT_TRUE(fresh.fresh);
  EXPECT_FALSE(fresh.receive_time_fallback);
  EXPECT_DOUBLE_EQ(fresh.age_ms, 20.0);

  const LatestLidarEvidenceFreshness3D stale =
      assessLatestLidarEvidenceFreshness3D(*evidence, 6'020'000'001, 20.0);
  EXPECT_FALSE(stale.fresh);
  EXPECT_GT(stale.age_ms, 20.0);
}

TEST(ExecutionEvidence3DTest,
     LatestLidarEvidenceFreshnessFallsBackForFutureAcquisitionTime) {
  LatestLidarEvidenceCapture3D capture = validLatestLidarCapture();
  capture.acquisition_stamp_ns = 7'000'000'000;
  const auto evidence = VersionedLatestLidarEvidence3D::capture(std::move(capture));
  ASSERT_NE(evidence, nullptr);

  const LatestLidarEvidenceFreshness3D freshness =
      assessLatestLidarEvidenceFreshness3D(*evidence, 6'020'000'000, 20.0);
  EXPECT_TRUE(freshness.fresh);
  EXPECT_TRUE(freshness.receive_time_fallback);
  EXPECT_DOUBLE_EQ(freshness.age_ms, 10.0);
}

TEST(ExecutionEvidence3DTest, LatestLidarEvidenceFreshnessRejectsInvalidTimeContracts) {
  const auto evidence =
      VersionedLatestLidarEvidence3D::capture(validLatestLidarCapture());
  ASSERT_NE(evidence, nullptr);

  EXPECT_FALSE(
      assessLatestLidarEvidenceFreshness3D(*evidence, 6'000'000'000, 20.0).fresh);
  EXPECT_FALSE(
      assessLatestLidarEvidenceFreshness3D(*evidence, 6'020'000'000, 0.0).fresh);
  EXPECT_FALSE(assessLatestLidarEvidenceFreshness3D(
                   *evidence, 6'020'000'000, std::numeric_limits<double>::quiet_NaN())
                   .fresh);
  EXPECT_FALSE(assessLatestLidarEvidenceFreshness3D(*evidence, 6'020'000'000,
                                                    std::numeric_limits<double>::max())
                   .fresh);
}

} // namespace
} // namespace drone_city_nav
