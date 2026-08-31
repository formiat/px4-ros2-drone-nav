#include <array>

#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(ExecutionRouteSnapshot3DTest,
     InitialFiniteCertificationUsesTheCertifiedHandoffEnvelope) {
  SnapshotFixture3D fixture;
  ExecutionRouteActivation3D route_activation = fixture.activation();
  route_activation.observation.position = Point3{2.0, 4.0, 5.0};
  route_activation.observation.maximum_cross_track_m = 15.0;
  const std::optional<CertifiedRouteSuffix3D> suffix =
      certifyExecutionRoute3D(route_activation);
  ASSERT_TRUE(suffix.has_value());
  EXPECT_DOUBLE_EQ(suffix->progress.last_observed_position.y, 4.0);
  const std::shared_ptr<const ExecutionPlan3D> initial =
      makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(initial, nullptr);

  const auto handoff_certification =
      [&suffix](const bool converge_to_route, const std::uint64_t revision,
                const FiniteExecutionKind3D kind = FiniteExecutionKind3D::kNominal,
                const std::size_t extra_stationary_control_count = 0U) {
        FiniteExecutionCertification3D certification =
            SnapshotFixture3D::finiteCertificationForRoute(
                *suffix, kind, revision, revision, extra_stationary_control_count);
        constexpr std::size_t kAccelerationControlCount{50U};
        constexpr float kInitialCrossTrackM{4.0F};
        const float lateral_acceleration_mps2 = kInitialCrossTrackM / 25.0F;
        for (std::size_t index = 0U; index < kAccelerationControlCount; ++index) {
          certification.horizon.controls[index].ay =
              converge_to_route ? -lateral_acceleration_mps2 : 0.0F;
        }
        for (std::size_t index = kAccelerationControlCount;
             index < 2U * kAccelerationControlCount; ++index) {
          certification.horizon.controls[index].ay =
              converge_to_route ? lateral_acceleration_mps2 : 0.0F;
        }
        certification.horizon.states.front().y = kInitialCrossTrackM;
        for (std::size_t index = 0U; index < certification.horizon.controls.size();
             ++index) {
          certification.horizon.states[index + 1U] =
              integrateMotionState3D(certification.horizon.states[index],
                                     certification.horizon.controls[index],
                                     suffix->validation_policy->dynamics());
        }
        const std::shared_ptr<const VersionedExecutionInput3D> source_input =
            certification.execution_input;
        if (source_input == nullptr) {
          return FiniteExecutionCertification3D{};
        }
        certification.execution_input =
            VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
                .capture_sequence = source_input->captureSequence(),
                .pose_revision = source_input->poseRevision(),
                .pose_source_timestamp_us = source_input->poseSourceTimestampUs(),
                .pose_receive_stamp_ns = source_input->poseReceiveStampNs(),
                .effective_stamp_ns = source_input->effectiveStampNs(),
                .state = certification.horizon.states.front(),
                .full_state_authoritative = source_input->fullStateAuthoritative(),
                .state_provenance = source_input->stateProvenance(),
                .previous_control = source_input->previousControl(),
                .previous_control_source = source_input->previousControlSource(),
                .previous_control_source_producer_instance_id =
                    source_input->previousControlSourceProducerInstanceId(),
                .previous_control_source_sequence =
                    source_input->previousControlSourceSequence(),
                .previous_control_source_stamp_ns =
                    source_input->previousControlSourceStampNs(),
                .previous_control_receive_stamp_ns =
                    source_input->previousControlReceiveStampNs(),
            });
        return certification;
      };

  FiniteExecutionCertificationResult3D accepted = certifyFiniteExecution3DDetailed(
      *initial, *suffix,
      handoff_certification(true, 102U, FiniteExecutionKind3D::kNominal, 10U));
  ASSERT_TRUE(accepted.certified())
      << "status=" << finiteExecutionCertificationStatus3DName(accepted.status)
      << " route_adherence_status="
      << finiteExecutionRouteAdherenceStatus3DName(accepted.route_adherence_status)
      << " route_adherence_failure_distance_m="
      << accepted.route_adherence_failure_distance_m;
  ASSERT_TRUE(accepted.execution.has_value());
  EXPECT_TRUE(accepted.execution->validFor(&*suffix));
  EXPECT_LT(accepted.execution->stop_boundary.position.y, 0.01);

  FiniteExecutionCertification3D tube_blocked = handoff_certification(true, 106U);
  ASSERT_NE(tube_blocked.execution_input, nullptr);
  ASSERT_NE(tube_blocked.latest_lidar_evidence, nullptr);
  constexpr std::size_t kTubeOnlyObstacleSegment{25U};
  ASSERT_LT(kTubeOnlyObstacleSegment, tube_blocked.horizon.states.size());
  const MotionState3D& obstacle_begin =
      tube_blocked.horizon.states[kTubeOnlyObstacleSegment - 1U];
  const MotionState3D& obstacle_stop =
      tube_blocked.horizon.states[kTubeOnlyObstacleSegment];
  const double connector_speed_mps =
      std::max(std::hypot(std::hypot(static_cast<double>(obstacle_begin.vx),
                                     static_cast<double>(obstacle_begin.vy)),
                          static_cast<double>(obstacle_begin.vz)),
               std::hypot(std::hypot(static_cast<double>(obstacle_stop.vx),
                                     static_cast<double>(obstacle_stop.vy)),
                          static_cast<double>(obstacle_stop.vz)));
  const double connector_tracking_radius_m = trackingErrorTubeRadiusM(
      suffix->geometry->tracking_error_tube->config, connector_speed_mps);
  ASSERT_GT(connector_tracking_radius_m, 0.05);
  const SweptFootprintConfig& physical_footprint =
      suffix->validation_policy->sweptFootprint();
  const Point3 tube_only_lidar_point{
      .x = 0.5 * (static_cast<double>(obstacle_begin.x) + obstacle_stop.x),
      .y = 0.5 * (static_cast<double>(obstacle_begin.y) + obstacle_stop.y),
      .z = 0.5 * (static_cast<double>(obstacle_begin.z) + obstacle_stop.z) +
           physical_footprint.upper_extent_m + 0.5 * connector_tracking_radius_m,
  };
  for (std::size_t index = 1U; index < tube_blocked.horizon.states.size(); ++index) {
    const MotionControl3D& start_control =
        index == 1U ? tube_blocked.execution_input->previousControl()
                    : tube_blocked.horizon.controls[index - 2U];
    const MotionControl3D& stop_control = tube_blocked.horizon.controls[index - 1U];
    const MotionState3D& first = tube_blocked.horizon.states[index - 1U];
    const MotionState3D& second = tube_blocked.horizon.states[index];
    ASSERT_TRUE(validateRawPointCloudSweptFootprint(
                    std::array<Point3, 1U>{tube_only_lidar_point},
                    Point3{first.x, first.y, first.z},
                    bodyAxisFromWorldAcceleration(
                        Vec3{start_control.ax, start_control.ay, start_control.az}),
                    Point3{second.x, second.y, second.z},
                    bodyAxisFromWorldAcceleration(
                        Vec3{stop_control.ax, stop_control.ay, stop_control.az}),
                    physical_footprint)
                    .accepted())
        << "physical collision at segment " << index;
  }
  const VersionedLatestLidarEvidence3D& clear_lidar =
      *tube_blocked.latest_lidar_evidence;
  tube_blocked.latest_lidar_evidence =
      VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
          .producer_instance_id = clear_lidar.producerInstanceId(),
          .sequence = clear_lidar.sequence(),
          .pose_generation = clear_lidar.poseGeneration(),
          .acquisition_stamp_ns = clear_lidar.acquisitionStampNs(),
          .receive_stamp_ns = clear_lidar.receiveStampNs(),
          .source_beam_count = 1U,
          .hit_points_map_m = {tube_only_lidar_point},
      });
  ASSERT_NE(tube_blocked.latest_lidar_evidence, nullptr);
  const FiniteExecutionCertificationResult3D blocked =
      certifyFiniteExecution3DDetailed(*initial, *suffix, std::move(tube_blocked));
  EXPECT_FALSE(blocked.certified());
  EXPECT_EQ(blocked.status,
            FiniteExecutionCertificationStatus3D::kTrackingTubeHandoffRejected);

  const ExecutionRouteTransitionResult3D activated = activateCertifiedRoute3D(
      *initial, initial->version, *suffix, *accepted.execution);
  ASSERT_TRUE(activated.applied())
      << "transition_status=" << executionRouteTransitionStatus3DName(activated.status);
  ASSERT_NE(activated.next, nullptr);
  ASSERT_TRUE(activated.next->route() != nullptr);
  ASSERT_TRUE(activated.next->finiteExecution() != nullptr);
  const FiniteExecutionState3D& resident_handoff = *activated.next->finiteExecution();
  ASSERT_NE(resident_handoff.horizon, nullptr);
  std::size_t acquisition_state_index{0U};
  for (std::size_t index = 1U; index < resident_handoff.horizon->states.size();
       ++index) {
    const MotionState3D& state = resident_handoff.horizon->states[index];
    const RouteProjection3D projection = projectOntoRoute3D(
        *activated.next->route()->geometry->route, Point3{state.x, state.y, state.z});
    const TrackingErrorTubeExecutionAssessment3D tube =
        projection.valid
            ? assessTrackingErrorTubeExecution3D(
                  *activated.next->route()->geometry->route,
                  *activated.next->route()->geometry->tracking_error_tube,
                  TrackingErrorTubeExecutionObservation3D{
                      .station_m = projection.station_m,
                      .cross_track_error_m = projection.distance_m,
                      .speed_mps = std::hypot(std::hypot(static_cast<double>(state.vx),
                                                         static_cast<double>(state.vy)),
                                              static_cast<double>(state.vz)),
                  })
            : TrackingErrorTubeExecutionAssessment3D{};
    if (tube.accepted() && projection.distance_m + 0.1 < tube.tube_radius_m) {
      acquisition_state_index = index;
      break;
    }
  }
  ASSERT_GT(acquisition_state_index, 1U);
  ASSERT_LT(acquisition_state_index, resident_handoff.horizon->controls.size());
  const VersionedExecutionInput3D& source_input =
      *activated.next->route()->progress.execution_input;
  const auto handoff_input_at = [&](const std::size_t clock_state_index,
                                    const MotionState3D& state) {
    const std::int64_t effective_stamp_ns =
        resident_handoff.valid_from_ns + static_cast<std::int64_t>(clock_state_index) *
                                             resident_handoff.control_interval_ns;
    return VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
        .capture_sequence = source_input.captureSequence() + 1U,
        .pose_revision = source_input.poseRevision() + 1U,
        .pose_source_timestamp_us = source_input.poseSourceTimestampUs() + 1U,
        .pose_receive_stamp_ns = effective_stamp_ns - 30'000LL,
        .effective_stamp_ns = effective_stamp_ns,
        .state = state,
        .full_state_authoritative = source_input.fullStateAuthoritative(),
        .state_provenance = source_input.stateProvenance(),
        .previous_control = resident_handoff.horizon->controls[clock_state_index - 1U],
        .previous_control_source = source_input.previousControlSource(),
        .previous_control_source_producer_instance_id =
            source_input.previousControlSourceProducerInstanceId(),
        .previous_control_source_sequence =
            source_input.previousControlSourceSequence() + 1U,
        .previous_control_source_stamp_ns = effective_stamp_ns - 20'000LL,
        .previous_control_receive_stamp_ns = effective_stamp_ns - 10'000LL,
    });
  };
  const MotionState3D acquired_state =
      resident_handoff.horizon->states[acquisition_state_index];
  const std::shared_ptr<const VersionedExecutionInput3D> premature_join =
      handoff_input_at(1U, acquired_state);
  ASSERT_NE(premature_join, nullptr);
  const TrackingErrorTubeHandoffAssessment3D premature_assessment =
      assessCertifiedTrackingTubeHandoff3D(*activated.next, *activated.next->route(),
                                           *premature_join);
  EXPECT_FALSE(premature_assessment.active());
  EXPECT_NE(premature_assessment.status,
            TrackingErrorTubeHandoffStatus3D::kReferenceAcquiredRouteTube);
  EXPECT_EQ(advanceCertifiedRoute3D(
                *activated.next, SnapshotFixture3D::guard(*activated.next),
                fixture.executionObservation(
                    Point3{acquired_state.x, acquired_state.y, acquired_state.z},
                    SnapshotFixture3D::kLatestRawRevision, &fixture.raw_occupancy),
                premature_join, activated.next->route()->observed_raw_world)
                .status,
            ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected);

  const std::shared_ptr<const VersionedExecutionInput3D> acquired_input =
      handoff_input_at(acquisition_state_index, acquired_state);
  ASSERT_NE(acquired_input, nullptr);
  EXPECT_EQ(assessCertifiedTrackingTubeHandoff3D(
                *activated.next, *activated.next->route(), *acquired_input)
                .status,
            TrackingErrorTubeHandoffStatus3D::kReferenceAcquiredRouteTube);
  const ExecutionRouteTransitionResult3D acquired = advanceCertifiedRoute3D(
      *activated.next, SnapshotFixture3D::guard(*activated.next),
      fixture.executionObservation(
          Point3{acquired_state.x, acquired_state.y, acquired_state.z},
          SnapshotFixture3D::kLatestRawRevision, &fixture.raw_occupancy),
      acquired_input, activated.next->route()->observed_raw_world);
  ASSERT_TRUE(acquired.applied())
      << executionRouteTransitionStatus3DName(acquired.status);
  ASSERT_NE(acquired.next, nullptr);
  ASSERT_TRUE(acquired.next->route() != nullptr);
  EXPECT_EQ(acquired.next->route()->progress.execution_input, acquired_input);

  const FiniteExecutionCertificationResult3D continued =
      certifyFiniteExecution3DDetailed(*activated.next, *activated.next->route(),
                                       handoff_certification(true, 103U));
  EXPECT_TRUE(continued.certified())
      << "status=" << finiteExecutionCertificationStatus3DName(continued.status)
      << " route_adherence_status="
      << finiteExecutionRouteAdherenceStatus3DName(continued.route_adherence_status)
      << " route_adherence_failure_distance_m="
      << continued.route_adherence_failure_distance_m;

  const FiniteExecutionCertificationResult3D retained =
      certifyFiniteExecution3DDetailed(
          *activated.next, *activated.next->route(),
          handoff_certification(true, 104U, FiniteExecutionKind3D::kRetained));
  ASSERT_TRUE(retained.certified());
  ASSERT_TRUE(retained.execution.has_value());
  EXPECT_EQ(retained.execution->source_snapshot_version, activated.next->version);
  EXPECT_TRUE(retained.execution->validFor(&*activated.next->route()));
  EXPECT_NEAR(retained.execution->begin_route_station_m,
              activated.next->route()->progress.station_m, 0.25);
  ASSERT_NE(retained.execution->horizon, nullptr);
  EXPECT_LE(distance3D(Point3{retained.execution->horizon->states.front().x,
                              retained.execution->horizon->states.front().y,
                              retained.execution->horizon->states.front().z},
                       activated.next->route()->progress.last_observed_position),
            0.25);
  EXPECT_GT(retained.execution->trajectory_revision,
            activated.next->finiteExecution()->trajectory_revision);
  EXPECT_GE(retained.execution->source_navigation_revision,
            activated.next->finiteExecution()->source_navigation_revision);
  EXPECT_GE(retained.execution->valid_from_ns,
            activated.next->finiteExecution()->valid_from_ns);
  EXPECT_LE(retained.execution->valid_until_ns,
            activated.next->finiteExecution()->valid_until_ns);
  const ExecutionRouteTransitionResult3D retained_transition = replaceFiniteExecution3D(
      *activated.next, SnapshotFixture3D::guard(*activated.next), retained.execution);
  ASSERT_TRUE(retained_transition.applied());
  ASSERT_NE(retained_transition.next, nullptr);
  ASSERT_TRUE(retained_transition.next->route() != nullptr);
  const FiniteExecutionCertificationResult3D retained_continuation =
      certifyFiniteExecution3DDetailed(
          *retained_transition.next, *retained_transition.next->route(),
          handoff_certification(true, 105U, FiniteExecutionKind3D::kRetained));
  EXPECT_TRUE(retained_continuation.certified())
      << "status="
      << finiteExecutionCertificationStatus3DName(retained_continuation.status)
      << " route_adherence_status="
      << finiteExecutionRouteAdherenceStatus3DName(
             retained_continuation.route_adherence_status)
      << " route_adherence_failure_distance_m="
      << retained_continuation.route_adherence_failure_distance_m;

  const FiniteExecutionCertificationResult3D nonconverging =
      certifyFiniteExecution3DDetailed(*initial, *suffix,
                                       handoff_certification(false, 103U));
  EXPECT_FALSE(nonconverging.certified());
  EXPECT_EQ(nonconverging.status,
            FiniteExecutionCertificationStatus3D::kRouteAdherenceRejected);
  EXPECT_EQ(nonconverging.route_adherence_status,
            FiniteExecutionRouteAdherenceStatus3D::kTrackingTubeExceeded);
}

} // namespace
} // namespace drone_city_nav
