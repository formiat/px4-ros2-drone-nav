#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace drone_city_nav::mppi {
namespace {

constexpr std::int64_t kSecondNs{1'000'000'000LL};

[[nodiscard]] std::vector<TimedExecutionPathPoint> testPath() {
  return {
      TimedExecutionPathPoint{
          .time_from_start_s = 0.0,
          .state = State{.x = 1.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F},
          .control = Control{},
      },
      TimedExecutionPathPoint{
          .time_from_start_s = 1.0,
          .state = State{.x = 3.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F},
          .control = Control{.ax = -1.0F},
      },
      TimedExecutionPathPoint{
          .time_from_start_s = 2.0,
          .state = State{.x = 4.0F, .y = 1.0F, .z = 5.0F},
          .control = Control{},
      },
  };
}

struct TestWorld {
  FlightEnvelopeConfig envelope{.minimum_target_z_m = 1.0, .maximum_target_z_m = 32.0};
  DynamicsConfig dynamics{.dt_s = 1.0F};
  AltitudeEnvelopeConfig altitude_envelope{.minimum_z_m = 1.0F, .maximum_z_m = 32.0F};
  SweptFootprintConfig footprint{.radius_m = 0.25,
                                 .lower_extent_m = 0.2,
                                 .upper_extent_m = 0.35,
                                 .sweep_step_m = 0.1};
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 0.5, 20, 20, 20}};
  ObservedOccupancyGrid3D observed_occupancy{
      GridBounds3D{0.0, 0.0, 0.0, 0.5, 20, 20, 20}};

  [[nodiscard]] FiniteExecutionPathWorld
  view(const std::span<const Point3> latest_lidar = {}) const noexcept {
    return FiniteExecutionPathWorld{
        .flight_envelope = &envelope,
        .dynamics = &dynamics,
        .altitude_envelope = &altitude_envelope,
        .footprint = &footprint,
        .static_occupancy = &occupancy,
        .latest_lidar_obstacle_points = latest_lidar,
        .terminal_boundary = std::nullopt,
    };
  }

  [[nodiscard]] FiniteExecutionPathWorld observedView() const noexcept {
    return FiniteExecutionPathWorld{
        .flight_envelope = &envelope,
        .dynamics = &dynamics,
        .altitude_envelope = &altitude_envelope,
        .footprint = &footprint,
        .observed_occupancy = &observed_occupancy,
        .latest_lidar_obstacle_points = {},
        .terminal_boundary = std::nullopt,
    };
  }
};

TEST(FiniteExecutionPathTest, RetainsOnlyRemainingPartOfActiveTerminalPath) {
  const TestWorld world;
  const std::vector<TimedExecutionPathPoint> path = testPath();

  const FiniteExecutionPathValidation result = validateFiniteExecutionPathContinuation(
      path, 10 * kSecondNs, 12 * kSecondNs, 10 * kSecondNs + 600'000'000LL,
      State{.x = 2.2F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}, Control{}, world.view());

  ASSERT_TRUE(result.accepted());
  EXPECT_EQ(result.first_remaining_point_index, 1U);
  EXPECT_DOUBLE_EQ(result.remaining_duration_s, 1.4);
}

TEST(FiniteExecutionPathTest,
     DistinguishesTrackedTrajectoryFromUnsafeActualStateContinuation) {
  TestWorld world;
  world.occupancy.setOccupied(GridIndex3D{6, 6, 10});
  const std::vector<TimedExecutionPathPoint> path = testPath();
  const State actual{
      .x = 2.2F,
      .y = 0.5F,
      .z = 5.0F,
      .vx = 2.0F,
      .vy = 2.0F,
  };

  const FiniteExecutionPathValidation tracked_trajectory =
      validateFiniteExecutionTrajectoryContinuation(
          path, 10 * kSecondNs, 12 * kSecondNs, 10 * kSecondNs + 600'000'000LL, actual,
          Control{}, world.view());
  const FiniteExecutionPathValidation actual_state_continuation =
      validateFiniteExecutionPathContinuation(path, 10 * kSecondNs, 12 * kSecondNs,
                                              10 * kSecondNs + 600'000'000LL, actual,
                                              Control{}, world.view());

  EXPECT_TRUE(tracked_trajectory.accepted());
  EXPECT_EQ(actual_state_continuation.status, FiniteExecutionPathStatus::kRawCollision);
  EXPECT_EQ(actual_state_continuation.failure_segment_index, 1U);
}

TEST(FiniteExecutionPathTest, RejectsExpiredPathWithoutExtendingItsLifetime) {
  const TestWorld world;
  const std::vector<TimedExecutionPathPoint> path = testPath();

  const FiniteExecutionPathValidation result = validateFiniteExecutionPathContinuation(
      path, 10 * kSecondNs, 12 * kSecondNs, 12 * kSecondNs,
      State{.x = 4.0F, .y = 1.0F, .z = 5.0F}, Control{}, world.view());

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kNotActive);
}

TEST(FiniteExecutionPathTest, RejectsPathWithoutTerminalRest) {
  const TestWorld world;
  std::vector<TimedExecutionPathPoint> path = testPath();
  path.back().state.vx = 0.2F;

  const FiniteExecutionPathValidation result = validateFiniteExecutionPathContinuation(
      path, 10 * kSecondNs, 12 * kSecondNs, 11 * kSecondNs,
      State{.x = 3.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}, Control{}, world.view());

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kInvalidContract);
}

TEST(FiniteExecutionPathTest, RejectsNewRawObstacleOnRemainingPath) {
  TestWorld world;
  world.occupancy.setOccupied(GridIndex3D{7, 2, 10});
  const std::vector<TimedExecutionPathPoint> path = testPath();

  const FiniteExecutionPathValidation result = validateFiniteExecutionPathContinuation(
      path, 10 * kSecondNs, 12 * kSecondNs, 11 * kSecondNs,
      State{.x = 3.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}, Control{}, world.view());

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kRawCollision);
}

TEST(FiniteExecutionPathTest, RejectsFreshLidarObstacleOnRemainingPath) {
  const TestWorld world;
  const std::vector<TimedExecutionPathPoint> path = testPath();
  const std::vector<Point3> lidar_hits{{3.5, 1.0, 5.0}};

  const FiniteExecutionPathValidation result = validateFiniteExecutionPathContinuation(
      path, 10 * kSecondNs, 12 * kSecondNs, 11 * kSecondNs,
      State{.x = 3.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}, Control{},
      world.view(lidar_hits));

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kLatestLidarRawCollision);
}

TEST(FiniteExecutionPathTest, CompleteValidationChecksEveryRawPathSegment) {
  TestWorld world;
  world.occupancy.setOccupied(GridIndex3D{7, 2, 10});
  const std::vector<TimedExecutionPathPoint> path = testPath();

  const FiniteExecutionPathValidation result =
      validateCompleteFiniteExecutionPath(path, Control{}, world.view());

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kRawCollision);
  EXPECT_EQ(result.failure_segment_index, 1U);
}

TEST(FiniteExecutionPathTest,
     CompleteValidationRequiresExactPreviousControlAtPointZero) {
  const TestWorld world;
  std::vector<TimedExecutionPathPoint> path = testPath();
  path.front().control.ax = 0.25F;

  const FiniteExecutionPathValidation result =
      validateCompleteFiniteExecutionPath(path, Control{}, world.view());

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kInvalidContract);
}

TEST(FiniteExecutionPathTest,
     ArrivalControlAxisDetectsFirstIntervalObstacleInCompleteAndContinuationChecks) {
  TestWorld world;
  world.footprint.radius_m = 0.1;
  world.footprint.upper_extent_m = 0.8;
  std::vector<TimedExecutionPathPoint> tilted_path = testPath();
  tilted_path[1].control = Control{.ay = 4.0F};
  const std::vector<Point3> tilted_body_obstacle{{3.0, 1.25, 5.6}};

  EXPECT_TRUE(validateCompleteFiniteExecutionPath(testPath(), Control{},
                                                  world.view(tilted_body_obstacle))
                  .accepted());
  const FiniteExecutionPathValidation complete_validation =
      validateCompleteFiniteExecutionPath(tilted_path, Control{},
                                          world.view(tilted_body_obstacle));
  EXPECT_EQ(complete_validation.status,
            FiniteExecutionPathStatus::kLatestLidarRawCollision);
  EXPECT_EQ(complete_validation.failure_segment_index, 0U);
  const FiniteExecutionPathValidation continuation_validation =
      validateFiniteExecutionPathContinuation(
          tilted_path, 10 * kSecondNs, 12 * kSecondNs, 10 * kSecondNs + 100'000'000LL,
          State{.x = 1.2F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}, Control{},
          world.view(tilted_body_obstacle));
  EXPECT_EQ(continuation_validation.status,
            FiniteExecutionPathStatus::kLatestLidarRawCollision);
}

TEST(FiniteExecutionPathTest, UnknownObservedFrontierIsAlwaysTraversable) {
  TestWorld world;
  const GridBounds3D& bounds = world.observed_occupancy.bounds();
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        static_cast<void>(world.observed_occupancy.setState(GridIndex3D{x, y, z},
                                                            ObservedVoxelState::kFree));
      }
    }
  }
  static_cast<void>(world.observed_occupancy.setState(GridIndex3D{7, 2, 10},
                                                      ObservedVoxelState::kUnknown));

  const FiniteExecutionPathValidation result =
      validateCompleteFiniteExecutionPath(testPath(), Control{}, world.observedView());

  EXPECT_TRUE(result.accepted());
}

TEST(FiniteExecutionPathTest, DefaultModeAllowsUnknownObservedFrontier) {
  TestWorld world;
  const GridBounds3D& bounds = world.observed_occupancy.bounds();
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        static_cast<void>(world.observed_occupancy.setState(GridIndex3D{x, y, z},
                                                            ObservedVoxelState::kFree));
      }
    }
  }
  static_cast<void>(world.observed_occupancy.setState(GridIndex3D{7, 2, 10},
                                                      ObservedVoxelState::kUnknown));

  const FiniteExecutionPathValidation result =
      validateCompleteFiniteExecutionPath(testPath(), Control{}, world.observedView());

  EXPECT_TRUE(result.accepted());
}

TEST(FiniteExecutionPathTest, DefaultModeStillRejectsObservedRawCollision) {
  TestWorld world;
  static_cast<void>(world.observed_occupancy.setState(GridIndex3D{7, 2, 10},
                                                      ObservedVoxelState::kOccupied));

  const FiniteExecutionPathValidation result =
      validateCompleteFiniteExecutionPath(testPath(), Control{}, world.observedView());

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kRawCollision);
}

TEST(FiniteExecutionPathTest, AcceptsAValidatedVerticalFiniteSegmentWithTerminalRest) {
  const TestWorld world;
  const std::vector<TimedExecutionPathPoint> path{
      {.time_from_start_s = 0.0,
       .state = State{.x = 2.0F, .y = 2.0F, .z = 2.0F, .vz = 1.0F},
       .control = Control{}},
      {.time_from_start_s = 1.0,
       .state = State{.x = 2.0F, .y = 2.0F, .z = 3.0F, .vz = 1.0F},
       .control = Control{.az = -1.0F}},
      {.time_from_start_s = 2.0,
       .state = State{.x = 2.0F, .y = 2.0F, .z = 3.5F},
       .control = Control{}},
  };

  const FiniteExecutionPathValidation result =
      validateCompleteFiniteExecutionPath(path, Control{}, world.view());

  EXPECT_TRUE(result.accepted());
}

TEST(FiniteExecutionPathTest,
     MovesArrivalProfileEarlierUntilCompletePathAvoidsNewObstacle) {
  TestWorld world;
  world.dynamics.dt_s = 0.1F;
  std::vector<Control> planned_controls(40U);
  std::vector<State> planned_states{State{.x = 1.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}};
  for (const Control& control : planned_controls) {
    planned_states.push_back(
        integrateReference(planned_states.back(), control, world.dynamics));
  }
  const std::vector<Point3> latest_lidar_hits{{5.0, 1.0, 5.0}};

  const ValidatedFiniteExecutionPath path = buildValidatedFiniteExecutionPath(
      planned_states, planned_controls, Control{}, world.dynamics, 5U,
      FiniteHorizonConfig{}, world.view(latest_lidar_hits));

  ASSERT_TRUE(path.accepted());
  ASSERT_TRUE(path.horizon.has_value());
  const FiniteHorizon horizon = path.horizon.value_or(FiniteHorizon{});
  EXPECT_LT(horizon.nominal_prefix_control_count, planned_controls.size());
  EXPECT_TRUE(path.path_validation_backoff);
  EXPECT_FALSE(path.persistent_raw_path_validation_backoff);
  EXPECT_TRUE(path.latest_lidar_path_validation_backoff);
  EXPECT_TRUE(path.physicalObstacleValidationBackoff());
  EXPECT_TRUE(finiteHorizonHasTerminalRestState(horizon));
  EXPECT_LT(horizon.states.back().x, 4.75F);
}

TEST(FiniteExecutionPathTest, ADischargedLeadingPrefixIsNotSweptAgain) {
  // The occupied cell sits under the path's second segment, so a full sweep
  // rejects it. Telling the validation that both leading points are already
  // known clear leaves nothing for it to sweep, and it accepts.
  TestWorld world;
  world.occupancy.setOccupied(GridIndex3D{7, 2, 10});
  const std::vector<TimedExecutionPathPoint> path = testPath();

  const FiniteExecutionPathValidation swept =
      validateCompleteFiniteExecutionPath(path, Control{}, world.view());
  ASSERT_EQ(swept.status, FiniteExecutionPathStatus::kRawCollision);
  // The first two points are clear; the segment reaching the third is not.
  EXPECT_EQ(swept.physically_validated_point_count, 2U);

  const FiniteExecutionPathValidation discharged =
      validateCompleteFiniteExecutionPath3D(path, Control{}, world.view(), path.size());
  EXPECT_TRUE(discharged.accepted());
  EXPECT_EQ(discharged.physically_validated_point_count, path.size());

  // A discharge that stops short still sweeps the rest, and still rejects.
  const FiniteExecutionPathValidation partial =
      validateCompleteFiniteExecutionPath3D(path, Control{}, world.view(), 2U);
  EXPECT_EQ(partial.status, FiniteExecutionPathStatus::kRawCollision);
}

TEST(FiniteExecutionPathTest, TheArrivalSearchStopsWhenItsBudgetIsSpent) {
  // A deadline already in the past: the first attempt still runs, and the
  // search then returns without shortening the prefix any further.
  TestWorld world;
  world.dynamics.dt_s = 0.1F;
  world.occupancy.setOccupied(GridIndex3D{10, 2, 10});
  std::vector<Control> planned_controls(40U);
  std::vector<State> planned_states{State{.x = 1.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}};
  for (const Control& control : planned_controls) {
    planned_states.push_back(
        integrateReference(planned_states.back(), control, world.dynamics));
  }

  const ValidatedFiniteExecutionPath unbounded = buildValidatedFiniteExecutionPath(
      planned_states, planned_controls, Control{}, world.dynamics, 5U,
      FiniteHorizonConfig{}, world.view());
  ASSERT_TRUE(unbounded.accepted());
  ASSERT_GT(unbounded.arrival_shaping_attempts, 1U);
  EXPECT_FALSE(unbounded.arrival_shaping_budget_exhausted);

  const ValidatedFiniteExecutionPath bounded = buildValidatedFiniteExecutionPath(
      planned_states, planned_controls, Control{}, world.dynamics, 5U,
      FiniteHorizonConfig{}, world.view(), {},
      FiniteExecutionPathBudget{.deadline = std::chrono::steady_clock::now() -
                                            std::chrono::seconds{1}});

  EXPECT_TRUE(bounded.arrival_shaping_budget_exhausted);
  EXPECT_EQ(bounded.arrival_shaping_attempts, 1U);
  EXPECT_FALSE(bounded.accepted());
}

TEST(FiniteExecutionPathTest,
     ReportsPersistentRawCollisionEvenWhenEarlierArrivalRemainsExecutable) {
  TestWorld world;
  world.dynamics.dt_s = 0.1F;
  world.occupancy.setOccupied(GridIndex3D{10, 2, 10});
  std::vector<Control> planned_controls(40U);
  std::vector<State> planned_states{State{.x = 1.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}};
  for (const Control& control : planned_controls) {
    planned_states.push_back(
        integrateReference(planned_states.back(), control, world.dynamics));
  }

  const ValidatedFiniteExecutionPath path = buildValidatedFiniteExecutionPath(
      planned_states, planned_controls, Control{}, world.dynamics, 5U,
      FiniteHorizonConfig{}, world.view());

  ASSERT_TRUE(path.accepted());
  ASSERT_TRUE(path.horizon.has_value());
  const FiniteHorizon horizon = path.horizon.value_or(FiniteHorizon{});
  EXPECT_TRUE(path.path_validation_backoff);
  EXPECT_TRUE(path.persistent_raw_path_validation_backoff);
  EXPECT_FALSE(path.latest_lidar_path_validation_backoff);
  EXPECT_TRUE(path.physicalObstacleValidationBackoff());
  EXPECT_EQ(path.first_failed_validation_status,
            FiniteExecutionPathStatus::kRawCollision);
  EXPECT_LT(horizon.nominal_prefix_control_count, planned_controls.size());
  EXPECT_TRUE(finiteHorizonHasTerminalRestState(horizon));
}

TEST(FiniteExecutionPathTest,
     MovesArrivalProfileEarlierUntilCandidateContractAcceptsIt) {
  TestWorld world;
  world.dynamics.dt_s = 0.1F;
  std::vector<Control> planned_controls(40U);
  std::vector<State> planned_states{State{.x = 1.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}};
  for (const Control& control : planned_controls) {
    planned_states.push_back(
        integrateReference(planned_states.back(), control, world.dynamics));
  }
  std::size_t validation_attempts{0U};

  const ValidatedFiniteExecutionPath path = buildValidatedFiniteExecutionPath(
      planned_states, planned_controls, Control{}, world.dynamics, 5U,
      FiniteHorizonConfig{}, world.view(),
      [&validation_attempts](const FiniteHorizon& candidate) {
        ++validation_attempts;
        return candidate.nominal_prefix_control_count <= 20U;
      });

  ASSERT_TRUE(path.accepted());
  ASSERT_TRUE(path.horizon.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  EXPECT_EQ(path.horizon.value().nominal_prefix_control_count, 20U);
  EXPECT_EQ(path.arrival_shaping_attempts, 5U);
  EXPECT_EQ(validation_attempts, 3U);
  EXPECT_EQ(path.first_failed_validation_status,
            FiniteExecutionPathStatus::kCandidateRejected);
  EXPECT_TRUE(path.path_validation_backoff);
}

TEST(FiniteExecutionPathTest, RejectsDynamicallyUnrecoverableCurrentAltitude) {
  const TestWorld world;
  const std::vector<TimedExecutionPathPoint> path = testPath();

  const FiniteExecutionPathValidation result = validateFiniteExecutionPathContinuation(
      path, 10 * kSecondNs, 12 * kSecondNs, 10 * kSecondNs + 600'000'000LL,
      State{.x = 2.2F, .y = 1.0F, .z = 1.5F, .vx = 2.0F, .vz = -3.0F}, Control{},
      world.view());

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kDynamicFlightEnvelopeViolation);
}

TEST(FiniteExecutionPathTest, RejectsRemainingControlsUnsafeFromActualState) {
  const TestWorld world;
  std::vector<TimedExecutionPathPoint> path = testPath();
  path[1].control.az = -4.0F;

  const FiniteExecutionPathValidation result = validateFiniteExecutionPathContinuation(
      path, 10 * kSecondNs, 12 * kSecondNs, 10 * kSecondNs + 100'000'000LL,
      State{.x = 1.2F, .y = 1.0F, .z = 6.0F, .vx = 2.0F, .vz = -1.0F}, Control{},
      world.view());

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kDynamicFlightEnvelopeViolation);
}

TEST(FiniteExecutionPathTest, CompleteValidationChecksStoppingRoomInsidePath) {
  const TestWorld world;
  std::vector<TimedExecutionPathPoint> path = testPath();
  path[1].state.z = 1.5F;
  path[1].state.vz = -3.0F;

  const FiniteExecutionPathValidation result =
      validateCompleteFiniteExecutionPath(path, Control{}, world.view());

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kDynamicFlightEnvelopeViolation);
  EXPECT_EQ(result.failure_segment_index, 0U);
}

TEST(FiniteExecutionPathTest, RejectsTerminalRestBeyondFiniteRouteEndpoint) {
  TestWorld world;
  FiniteExecutionPathWorld view = world.view();
  view.terminal_boundary = FiniteExecutionPathTerminalBoundary{
      .endpoint = Point3{3.5, 1.0, 5.0},
      .forward = Vec3{1.0, 0.0, 0.0},
      .tolerance_m = 0.0,
      .activation_route = {},
  };

  const FiniteExecutionPathValidation result =
      validateCompleteFiniteExecutionPath(testPath(), Control{}, view);

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kRouteEndpointExceeded);
  EXPECT_EQ(result.failure_segment_index, 1U);
}

TEST(FiniteExecutionPathTest, AcceptsTerminalRestBeforeFiniteRouteEndpoint) {
  TestWorld world;
  FiniteExecutionPathWorld view = world.view();
  view.terminal_boundary = FiniteExecutionPathTerminalBoundary{
      .endpoint = Point3{4.5, 1.0, 5.0},
      .forward = Vec3{1.0, 0.0, 0.0},
      .tolerance_m = 0.0,
      .activation_route = {},
  };

  EXPECT_TRUE(
      validateCompleteFiniteExecutionPath(testPath(), Control{}, view).accepted());
}

TEST(FiniteExecutionPathTest, AcceptsCurvedApproachOutsideTerminalSegmentCorridor) {
  TestWorld world;
  FiniteExecutionPathWorld view = world.view();
  view.terminal_boundary = FiniteExecutionPathTerminalBoundary{
      .endpoint = Point3{0.0, 0.0, 5.0},
      .forward = Vec3{1.0, 0.0, 0.0},
      .tolerance_m = 0.0,
      .activation_distance_m = 10.0,
      .maximum_cross_track_m = 0.5,
      .activation_route = {},
  };

  EXPECT_TRUE(
      validateCompleteFiniteExecutionPath(testPath(), Control{}, view).accepted());
}

TEST(FiniteExecutionPathTest,
     DoesNotApplyFoldedRouteEndpointBeforeReachingItsFinalSegment) {
  TestWorld world;
  const std::vector<RouteSample3D> route{
      RouteSample3D{.x_m = 4.0F, .y_m = 2.5F, .z_m = 5.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 1.0F, .y_m = 2.5F, .z_m = 5.0F, .station_m = 3.0F},
      RouteSample3D{.x_m = 1.0F, .y_m = 3.0F, .z_m = 5.0F, .station_m = 3.5F},
      RouteSample3D{.x_m = 3.0F, .y_m = 3.0F, .z_m = 5.0F, .station_m = 5.5F},
  };
  FiniteExecutionPathWorld view = world.view();
  view.terminal_boundary = FiniteExecutionPathTerminalBoundary{
      .endpoint = Point3{3.0, 3.0, 5.0},
      .forward = Vec3{1.0, 0.0, 0.0},
      .tolerance_m = 0.0,
      .activation_distance_m = 10.0,
      .maximum_cross_track_m = 2.0,
      .activation_route = route,
      .initial_route_station_m = 0.0F,
      .activation_route_station_m = 3.5F,
  };
  const std::vector<TimedExecutionPathPoint> approach{
      TimedExecutionPathPoint{
          .time_from_start_s = 0.0,
          .state = State{.x = 4.0F, .y = 2.5F, .z = 5.0F, .vx = -0.5F},
      },
      TimedExecutionPathPoint{
          .time_from_start_s = 1.0,
          .state = State{.x = 3.5F, .y = 2.5F, .z = 5.0F, .vx = -0.5F},
      },
      TimedExecutionPathPoint{
          .time_from_start_s = 2.0,
          .state = State{.x = 3.0F, .y = 2.5F, .z = 5.0F},
      },
  };

  EXPECT_TRUE(
      validateCompleteFiniteExecutionPath(approach, Control{}, view).accepted());
}

TEST(FiniteExecutionPathTest, RouteAwareBoundaryStillRejectsFinalSegmentOvershoot) {
  TestWorld world;
  const std::vector<RouteSample3D> route{
      RouteSample3D{.x_m = 4.0F, .y_m = 2.5F, .z_m = 5.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 1.0F, .y_m = 2.5F, .z_m = 5.0F, .station_m = 3.0F},
      RouteSample3D{.x_m = 1.0F, .y_m = 3.0F, .z_m = 5.0F, .station_m = 3.5F},
      RouteSample3D{.x_m = 3.0F, .y_m = 3.0F, .z_m = 5.0F, .station_m = 5.5F},
  };
  FiniteExecutionPathWorld view = world.view();
  view.terminal_boundary = FiniteExecutionPathTerminalBoundary{
      .endpoint = Point3{3.0, 3.0, 5.0},
      .forward = Vec3{1.0, 0.0, 0.0},
      .tolerance_m = 0.0,
      .activation_route = route,
      .initial_route_station_m = 3.5F,
      .activation_route_station_m = 3.5F,
  };
  std::vector<TimedExecutionPathPoint> path{
      TimedExecutionPathPoint{
          .time_from_start_s = 0.0,
          .state = State{.x = 1.0F, .y = 3.0F, .z = 5.0F, .vx = 1.5F},
      },
      TimedExecutionPathPoint{
          .time_from_start_s = 1.0,
          .state = State{.x = 3.5F, .y = 3.0F, .z = 5.0F, .vx = 1.0F},
      },
      TimedExecutionPathPoint{
          .time_from_start_s = 2.0,
          .state = State{.x = 3.0F, .y = 3.0F, .z = 5.0F},
      },
  };

  const FiniteExecutionPathValidation result =
      validateCompleteFiniteExecutionPath(path, Control{}, view);

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kRouteEndpointExceeded);
  EXPECT_EQ(result.failure_segment_index, 0U);
}

TEST(FiniteExecutionPathTest, RejectsOvershootBeforeTerminalRestReturnsInsideRoute) {
  TestWorld world;
  FiniteExecutionPathWorld view = world.view();
  view.terminal_boundary = FiniteExecutionPathTerminalBoundary{
      .endpoint = Point3{4.5, 1.0, 5.0},
      .forward = Vec3{1.0, 0.0, 0.0},
      .tolerance_m = 0.0,
      .activation_route = {},
  };
  std::vector<TimedExecutionPathPoint> path = testPath();
  path[1].state.x = 5.0F;

  const FiniteExecutionPathValidation result =
      validateCompleteFiniteExecutionPath(path, Control{}, view);

  EXPECT_EQ(result.status, FiniteExecutionPathStatus::kRouteEndpointExceeded);
  EXPECT_EQ(result.failure_segment_index, 0U);
  EXPECT_DOUBLE_EQ(result.failure_point.x, 5.0);
}

TEST(FiniteExecutionPathTest, RebuildsContinuationFromActualStateWithoutExtension) {
  TestWorld world;
  world.dynamics.dt_s = 0.1F;
  std::vector<Control> source_controls(20U);
  std::vector<State> source_states{State{.x = 1.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}};
  for (const Control& control : source_controls) {
    source_states.push_back(
        integrateReference(source_states.back(), control, world.dynamics));
  }
  const std::optional<FiniteHorizon> source =
      buildFiniteHorizon(source_states, source_controls, 0U, world.dynamics, Control{});
  ASSERT_TRUE(source.has_value());
  std::vector<TimedExecutionPathPoint> points;
  for (std::size_t index = 0U; index < source->states.size(); ++index) {
    points.push_back(TimedExecutionPathPoint{
        .time_from_start_s = static_cast<double>(index) * world.dynamics.dt_s,
        .state = source->states[index],
        .control = index == 0U ? Control{} : source->controls[index - 1U],
    });
  }
  const State actual{.x = 1.8F, .y = 1.5F, .z = 5.0F, .vx = 2.0F};

  const RebuiltFiniteExecutionPathContinuation rebuilt =
      rebuildFiniteExecutionPathContinuation(
          points, 10 * kSecondNs, 12 * kSecondNs, 10 * kSecondNs + 500'000'000LL,
          actual, Control{}, source->nominal_prefix_control_count,
          source->nominal_prefix_control_count, world.dynamics, 2U,
          FiniteHorizonConfig{}, world.view());

  ASSERT_TRUE(rebuilt.accepted());
  ASSERT_TRUE(rebuilt.horizon.has_value());
  EXPECT_FLOAT_EQ(rebuilt.horizon->states.front().x, actual.x);
  EXPECT_FLOAT_EQ(rebuilt.horizon->states.front().y, actual.y);
  EXPECT_TRUE(finiteHorizonHasTerminalRestState(*rebuilt.horizon));
  EXPECT_LE(rebuilt.valid_until_ns, 12 * kSecondNs);
  EXPECT_EQ(rebuilt.source_control_index, 5U);
}

TEST(FiniteExecutionPathTest,
     RebasePreservesAnUnpublishedCertifiedArrivalSequenceFirst) {
  TestWorld world;
  world.dynamics.dt_s = 0.1F;
  std::vector<Control> planned_controls(20U);
  std::vector<State> planned_states{State{.x = 1.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}};
  for (const Control& control : planned_controls) {
    planned_states.push_back(
        integrateReference(planned_states.back(), control, world.dynamics));
  }
  const std::optional<FiniteHorizon> source = buildFiniteHorizon(
      planned_states, planned_controls, 0U, world.dynamics, Control{});
  ASSERT_TRUE(source.has_value());
  ASSERT_EQ(source->nominal_prefix_control_count, 0U);
  ASSERT_EQ(source->arrival_control_count, source->controls.size());
  std::vector<TimedExecutionPathPoint> points;
  points.reserve(source->states.size());
  for (std::size_t index = 0U; index < source->states.size(); ++index) {
    points.push_back(TimedExecutionPathPoint{
        .time_from_start_s = static_cast<double>(index) * world.dynamics.dt_s,
        .state = source->states[index],
        .control = index == 0U ? Control{} : source->controls[index - 1U],
    });
  }

  const RebuiltFiniteExecutionPathContinuation rebuilt =
      rebuildFiniteExecutionPathContinuation(
          points, 10 * kSecondNs, 12 * kSecondNs, 10 * kSecondNs,
          source->states.front(), Control{}, source->nominal_prefix_control_count,
          source->controls.size(), world.dynamics, 2U, FiniteHorizonConfig{},
          world.view());

  ASSERT_TRUE(rebuilt.accepted());
  ASSERT_TRUE(rebuilt.horizon.has_value());
  ASSERT_EQ(rebuilt.horizon->controls.size(), source->controls.size());
  EXPECT_EQ(rebuilt.horizon->nominal_prefix_control_count, 0U);
  EXPECT_EQ(rebuilt.horizon->arrival_control_count, source->controls.size());
  for (std::size_t index = 0U; index < source->controls.size(); ++index) {
    EXPECT_FLOAT_EQ(rebuilt.horizon->controls[index].ax, source->controls[index].ax);
    EXPECT_FLOAT_EQ(rebuilt.horizon->controls[index].ay, source->controls[index].ay);
    EXPECT_FLOAT_EQ(rebuilt.horizon->controls[index].az, source->controls[index].az);
    EXPECT_FLOAT_EQ(rebuilt.horizon->controls[index].yaw_accel,
                    source->controls[index].yaw_accel);
  }
}

TEST(FiniteExecutionPathTest, RebuildsAFiniteRawSafeBrakingTailBeforeANewObstacle) {
  TestWorld world;
  world.dynamics.dt_s = 0.1F;
  std::vector<Control> planned_controls(50U);
  std::vector<State> planned_states{State{.x = 1.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}};
  for (const Control& control : planned_controls) {
    planned_states.push_back(
        integrateReference(planned_states.back(), control, world.dynamics));
  }
  const std::optional<FiniteHorizon> source = buildFiniteHorizon(
      planned_states, planned_controls, 20U, world.dynamics, Control{});
  ASSERT_TRUE(source.has_value());
  std::vector<TimedExecutionPathPoint> points;
  points.reserve(source->states.size());
  for (std::size_t index = 0U; index < source->states.size(); ++index) {
    points.push_back(TimedExecutionPathPoint{
        .time_from_start_s = static_cast<double>(index) * world.dynamics.dt_s,
        .state = source->states[index],
        .control = index == 0U ? Control{} : source->controls[index - 1U],
    });
  }
  constexpr std::size_t kCurrentControlIndex{5U};
  const State current_state = source->states[kCurrentControlIndex];
  const Control current_control = source->controls[kCurrentControlIndex - 1U];
  const std::vector<Point3> latest_lidar_hits{{4.0, 1.0, 5.0}};

  const RebuiltFiniteExecutionPathContinuation braking =
      rebuildFiniteExecutionPathContinuation(
          points, 10 * kSecondNs, 20 * kSecondNs,
          10 * kSecondNs +
              static_cast<std::int64_t>(kCurrentControlIndex) * 100'000'000LL,
          current_state, current_control, source->nominal_prefix_control_count,
          source->nominal_prefix_control_count, world.dynamics, 5U,
          FiniteHorizonConfig{}, world.view(latest_lidar_hits));

  ASSERT_TRUE(braking.accepted());
  ASSERT_TRUE(braking.horizon.has_value());
  EXPECT_TRUE(braking.path_validation_backoff);
  EXPECT_TRUE(braking.latest_lidar_path_validation_backoff);
  EXPECT_TRUE(finiteHorizonHasTerminalRestState(*braking.horizon));
  EXPECT_FLOAT_EQ(braking.horizon->states.front().x, current_state.x);
  EXPECT_LT(braking.horizon->states.back().x,
            static_cast<float>(latest_lidar_hits.front().x - world.footprint.radius_m));
  EXPECT_LT(braking.horizon->nominal_prefix_control_count,
            planned_controls.size() - kCurrentControlIndex);
  EXPECT_LE(braking.valid_until_ns, 20 * kSecondNs);
}

TEST(FiniteExecutionPathTest, RebuildPreservesTheRemainingNominalPhaseBoundary) {
  TestWorld world;
  world.dynamics.dt_s = 0.1F;
  std::vector<Control> planned_controls(50U);
  std::vector<State> planned_states{State{.x = 1.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}};
  for (const Control& control : planned_controls) {
    planned_states.push_back(
        integrateReference(planned_states.back(), control, world.dynamics));
  }
  const std::optional<FiniteHorizon> source = buildFiniteHorizon(
      planned_states, planned_controls, 20U, world.dynamics, Control{});
  ASSERT_TRUE(source.has_value());
  std::vector<TimedExecutionPathPoint> points;
  points.reserve(source->states.size());
  for (std::size_t index = 0U; index < source->states.size(); ++index) {
    points.push_back(TimedExecutionPathPoint{
        .time_from_start_s = static_cast<double>(index) * world.dynamics.dt_s,
        .state = source->states[index],
        .control = index == 0U ? Control{} : source->controls[index - 1U],
    });
  }
  constexpr std::size_t kCurrentControlIndex{5U};

  const RebuiltFiniteExecutionPathContinuation rebuilt =
      rebuildFiniteExecutionPathContinuation(
          points, 10 * kSecondNs, 15 * kSecondNs,
          10 * kSecondNs +
              static_cast<std::int64_t>(kCurrentControlIndex) * 100'000'000LL,
          source->states[kCurrentControlIndex],
          source->controls[kCurrentControlIndex - 1U],
          source->nominal_prefix_control_count, source->controls.size(), world.dynamics,
          5U, FiniteHorizonConfig{}, world.view());

  ASSERT_TRUE(rebuilt.accepted());
  ASSERT_TRUE(rebuilt.horizon.has_value());
  EXPECT_EQ(rebuilt.source_control_index, kCurrentControlIndex);
  EXPECT_EQ(rebuilt.horizon->nominal_prefix_control_count,
            source->nominal_prefix_control_count - kCurrentControlIndex);
  EXPECT_EQ(rebuilt.horizon->arrival_control_count, source->arrival_control_count);
  EXPECT_FALSE(rebuilt.path_validation_backoff);
  EXPECT_EQ(rebuilt.arrival_shaping_attempts, 1U);
  ASSERT_EQ(rebuilt.horizon->controls.size(),
            source->controls.size() - kCurrentControlIndex);
  for (std::size_t index = 0U; index < rebuilt.horizon->controls.size(); ++index) {
    const Control& expected = source->controls[kCurrentControlIndex + index];
    const Control& actual = rebuilt.horizon->controls[index];
    EXPECT_FLOAT_EQ(actual.ax, expected.ax);
    EXPECT_FLOAT_EQ(actual.ay, expected.ay);
    EXPECT_FLOAT_EQ(actual.az, expected.az);
    EXPECT_FLOAT_EQ(actual.yaw_accel, expected.yaw_accel);
  }
  EXPECT_TRUE(finiteHorizonHasTerminalRestState(*rebuilt.horizon));
}

TEST(FiniteExecutionPathTest,
     RebuildBacksOffACertifiedSuffixUntilCandidateContractAcceptsIt) {
  TestWorld world;
  world.dynamics.dt_s = 0.1F;
  std::vector<Control> source_controls(50U);
  std::vector<State> source_states{State{.x = 1.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}};
  for (const Control& control : source_controls) {
    source_states.push_back(
        integrateReference(source_states.back(), control, world.dynamics));
  }
  const std::optional<FiniteHorizon> source = buildFiniteHorizon(
      source_states, source_controls, 20U, world.dynamics, Control{});
  ASSERT_TRUE(source.has_value());
  std::vector<TimedExecutionPathPoint> points;
  points.reserve(source->states.size());
  for (std::size_t index = 0U; index < source->states.size(); ++index) {
    points.push_back(TimedExecutionPathPoint{
        .time_from_start_s = static_cast<double>(index) * world.dynamics.dt_s,
        .state = source->states[index],
        .control = index == 0U ? Control{} : source->controls[index - 1U],
    });
  }
  constexpr std::size_t kCurrentControlIndex{5U};
  std::size_t certification_attempts{0U};

  const RebuiltFiniteExecutionPathContinuation rebuilt =
      rebuildFiniteExecutionPathContinuation(
          points, 10 * kSecondNs, 15 * kSecondNs,
          10 * kSecondNs +
              static_cast<std::int64_t>(kCurrentControlIndex) * 100'000'000LL,
          source->states[kCurrentControlIndex],
          source->controls[kCurrentControlIndex - 1U],
          source->nominal_prefix_control_count, source->controls.size(), world.dynamics,
          5U, FiniteHorizonConfig{}, world.view(),
          [&certification_attempts](const FiniteHorizon&) {
            ++certification_attempts;
            return certification_attempts >= 2U;
          });

  ASSERT_TRUE(rebuilt.accepted());
  ASSERT_TRUE(rebuilt.horizon.has_value());
  EXPECT_EQ(certification_attempts, 2U);
  EXPECT_EQ(rebuilt.arrival_shaping_attempts, 2U);
  EXPECT_TRUE(rebuilt.path_validation_backoff);
  EXPECT_EQ(rebuilt.validation.status, FiniteExecutionPathStatus::kValid);
  EXPECT_TRUE(finiteHorizonHasTerminalRestState(*rebuilt.horizon));
}

TEST(FiniteExecutionPathTest, TheArrivalSearchContinuesFromTheFailingSegment) {
  // The obstacle lies in the middle of the planned motion. Every prefix that
  // still reaches it fails at the same segment, so the search continues from
  // that segment instead of stepping down through prefixes that must fail.
  TestWorld world;
  world.dynamics.dt_s = 0.1F;
  world.occupancy.setOccupied(GridIndex3D{10, 2, 10});
  std::vector<Control> planned_controls(40U);
  std::vector<State> planned_states{State{.x = 1.0F, .y = 1.0F, .z = 5.0F, .vx = 2.0F}};
  for (const Control& control : planned_controls) {
    planned_states.push_back(
        integrateReference(planned_states.back(), control, world.dynamics));
  }

  const ValidatedFiniteExecutionPath path = buildValidatedFiniteExecutionPath(
      planned_states, planned_controls, Control{}, world.dynamics, 5U,
      FiniteHorizonConfig{}, world.view());

  ASSERT_TRUE(path.accepted());
  ASSERT_TRUE(path.horizon.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  EXPECT_LT(path.horizon->nominal_prefix_control_count, 20U);
  // Forty and thirty-five controls leave the arrival no room to rest, thirty
  // reaches the obstacle at segment twenty, and the search continues from
  // twenty: five attempts, where the ladder of every five controls from
  // forty down to fifteen took six.
  EXPECT_EQ(path.first_failed_validation.failure_segment_index, 20U);
  EXPECT_LT(path.arrival_shaping_attempts, 6U);
}

} // namespace
} // namespace drone_city_nav::mppi
