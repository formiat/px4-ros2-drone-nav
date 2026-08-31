#include <gtest/gtest.h>

#include "execution_horizon_assembler_3d.hpp"

namespace drone_city_nav {
namespace {

struct ExecutionCycleFixture3D {
  mppi::MppiTickInput input{};
  mppi::MppiTickResult result{};
  WorldSnapshot3D world{};
  ProductionMppiExecutionPublication publication{};
  ProductionMppiExecutionCycle cycle{
      .controller =
          ControllerCycle3D{
              .input = &input,
              .result = &result,
              .world = &world,
          },
      .publication = &publication,
  };
};

[[nodiscard]] std::shared_ptr<const VersionedLatestLidarEvidence3D>
latestLidarEvidence() {
  return VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
      .producer_instance_id = 41U,
      .sequence = 43U,
      .pose_generation = 47U,
      .acquisition_stamp_ns = 6'000'000'000,
      .receive_stamp_ns = 6'010'000'000,
      .source_beam_count = 1U,
      .hit_points_map_m = {},
  });
}

void bindExactEvidence(ExecutionCycleFixture3D& fixture,
                       const FlightEnvelopeConfig& flight_envelope,
                       const mppi::DynamicsConfig& dynamics,
                       const mppi::AltitudeEnvelopeConfig& altitude_envelope,
                       const SweptFootprintConfig& footprint) {
  fixture.cycle.evidence.latest_lidar_evidence = latestLidarEvidence();
  fixture.cycle.evidence.latest_lidar_obstacle_fresh = true;
  fixture.cycle.evidence.exact_snapshot_world = true;
  fixture.cycle.evidence.execution_flight_envelope = &flight_envelope;
  fixture.cycle.evidence.execution_dynamics = &dynamics;
  fixture.cycle.evidence.execution_altitude_envelope = &altitude_envelope;
  fixture.cycle.evidence.execution_footprint = &footprint;
}

TEST(ExecutionHorizonAssembler3DTest, RejectsIncompleteCycleWithoutSideEffects) {
  const ExecutionHorizonAssembler3D assembler{{}};

  const HorizonCandidate3D candidate = assembler.assemble({}, nullptr);

  EXPECT_EQ(candidate.status, HorizonCandidateStatus3D::kMissingExactEvidence);
  EXPECT_EQ(candidate.failure_reason,
            ProductionMppiExecutionReason::kNoExecutableHorizon);
  EXPECT_FALSE(candidate.planned());
}

TEST(ExecutionHorizonAssembler3DTest,
     ResolvesExplicitPlanningDispositionsBeforeControllerEvidence) {
  const ExecutionHorizonAssembler3D assembler{{}};
  ExecutionCycleFixture3D fixture;
  fixture.input.target = mppi::State{.x = 7.0F, .y = 8.0F, .z = 9.0F};
  fixture.cycle.route.mission_goal = Point3{1.0, 2.0, 3.0};
  fixture.cycle.publication = nullptr;
  ASSERT_TRUE(fixture.cycle.validForAssembly());
  ASSERT_FALSE(fixture.cycle.valid());

  fixture.cycle.route.planning_state =
      ProductionMppiPlanningState::kMissionGoalPositionHold;
  const HorizonCandidate3D goal_candidate = assembler.assemble(fixture.cycle, nullptr);
  EXPECT_EQ(goal_candidate.status, HorizonCandidateStatus3D::kExplicitHold);
  EXPECT_EQ(goal_candidate.explicit_hold_reason,
            ProductionMppiExecutionReason::kGoalCapture);
  EXPECT_DOUBLE_EQ(goal_candidate.explicit_hold_position.x,
                   fixture.cycle.route.mission_goal.x);
  EXPECT_DOUBLE_EQ(goal_candidate.explicit_hold_position.y,
                   fixture.cycle.route.mission_goal.y);
  EXPECT_DOUBLE_EQ(goal_candidate.explicit_hold_position.z,
                   fixture.cycle.route.mission_goal.z);

  fixture.cycle.route.planning_state =
      ProductionMppiPlanningState::kMissionCommandPositionHold;
  const HorizonCandidate3D command_candidate =
      assembler.assemble(fixture.cycle, nullptr);
  EXPECT_EQ(command_candidate.status, HorizonCandidateStatus3D::kExplicitHold);
  EXPECT_EQ(command_candidate.explicit_hold_reason,
            ProductionMppiExecutionReason::kGoalCapture);
  EXPECT_DOUBLE_EQ(command_candidate.explicit_hold_position.x, 7.0);
  EXPECT_DOUBLE_EQ(command_candidate.explicit_hold_position.y, 8.0);
  EXPECT_DOUBLE_EQ(command_candidate.explicit_hold_position.z, 9.0);

  fixture.cycle.route.planning_state =
      ProductionMppiPlanningState::kCooperativePassageYieldHold;
  const HorizonCandidate3D yield_candidate = assembler.assemble(fixture.cycle, nullptr);
  EXPECT_EQ(yield_candidate.status, HorizonCandidateStatus3D::kExplicitHold);
  EXPECT_EQ(yield_candidate.explicit_hold_reason,
            ProductionMppiExecutionReason::kCooperativePassageYield);
  EXPECT_DOUBLE_EQ(yield_candidate.explicit_hold_position.x, 7.0);
  EXPECT_DOUBLE_EQ(yield_candidate.explicit_hold_position.y, 8.0);
  EXPECT_DOUBLE_EQ(yield_candidate.explicit_hold_position.z, 9.0);
}

TEST(ExecutionHorizonAssembler3DTest,
     ReturnsTypedNoRouteDispositionWithoutInspectingControllerOutput) {
  const ExecutionHorizonAssembler3D assembler{{}};
  ExecutionCycleFixture3D fixture;
  fixture.cycle.route.planning_state =
      ProductionMppiPlanningState::kNoExecutableRouteHold;

  const HorizonCandidate3D candidate = assembler.assemble(fixture.cycle, nullptr);

  EXPECT_EQ(candidate.status, HorizonCandidateStatus3D::kNoExecutableRoute);
  EXPECT_EQ(candidate.failure_reason,
            ProductionMppiExecutionReason::kNoExecutableRoute);
  EXPECT_FALSE(candidate.planned());
}

TEST(ExecutionHorizonAssembler3DTest,
     RejectsInvalidControllerOutputBeforeSnapshotCertification) {
  const ExecutionHorizonAssembler3D assembler{{}};
  ExecutionCycleFixture3D fixture;
  const FlightEnvelopeConfig flight_envelope{};
  const mppi::DynamicsConfig dynamics{};
  const mppi::AltitudeEnvelopeConfig altitude_envelope{};
  const SweptFootprintConfig footprint{};
  bindExactEvidence(fixture, flight_envelope, dynamics, altitude_envelope, footprint);

  const HorizonCandidate3D candidate = assembler.assemble(fixture.cycle, nullptr);

  EXPECT_EQ(candidate.status, HorizonCandidateStatus3D::kInvalidControllerOutput);
  EXPECT_FALSE(candidate.planned());
}

TEST(ExecutionHorizonAssembler3DTest,
     RejectsStaleSnapshotBeforeFinitePathCertification) {
  const ExecutionHorizonAssembler3D assembler{{}};
  ExecutionCycleFixture3D fixture;
  const FlightEnvelopeConfig flight_envelope{};
  const mppi::DynamicsConfig dynamics{};
  const mppi::AltitudeEnvelopeConfig altitude_envelope{};
  const SweptFootprintConfig footprint{};
  bindExactEvidence(fixture, flight_envelope, dynamics, altitude_envelope, footprint);
  fixture.result.horizon = {
      mppi::State{.z = 5.0F},
      mppi::State{.z = 5.0F},
  };
  fixture.result.controls = {mppi::Control{}};

  const HorizonCandidate3D candidate = assembler.assemble(fixture.cycle, nullptr);

  EXPECT_EQ(candidate.status, HorizonCandidateStatus3D::kStaleExecutionSnapshot);
  EXPECT_FALSE(candidate.planned());
}

TEST(ExecutionHorizonAssembler3DTest, NamesEveryTypedDisposition) {
  EXPECT_EQ(horizonCandidateStatus3DName(HorizonCandidateStatus3D::kPlanned),
            "planned");
  EXPECT_EQ(horizonCandidateStatus3DName(HorizonCandidateStatus3D::kExplicitHold),
            "explicit_hold");
  EXPECT_EQ(horizonCandidateStatus3DName(HorizonCandidateStatus3D::kNoExecutableRoute),
            "no_executable_route");
  EXPECT_EQ(
      horizonCandidateStatus3DName(HorizonCandidateStatus3D::kMissingExactEvidence),
      "missing_exact_evidence");
  EXPECT_EQ(
      horizonCandidateStatus3DName(HorizonCandidateStatus3D::kInvalidControllerOutput),
      "invalid_controller_output");
  EXPECT_EQ(
      horizonCandidateStatus3DName(HorizonCandidateStatus3D::kStaleExecutionSnapshot),
      "stale_execution_snapshot");
  EXPECT_EQ(horizonCandidateStatus3DName(HorizonCandidateStatus3D::kFinitePathRejected),
            "finite_path_rejected");
  EXPECT_EQ(
      horizonCandidateStatus3DName(HorizonCandidateStatus3D::kCertificationRejected),
      "certification_rejected");
  EXPECT_EQ(horizonCandidateStatus3DName(HorizonCandidateStatus3D::kTransitionRejected),
            "transition_rejected");
}

} // namespace
} // namespace drone_city_nav
