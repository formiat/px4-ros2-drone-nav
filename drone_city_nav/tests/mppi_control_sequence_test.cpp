#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/mppi/mppi_route_projection.hpp"
#include "drone_city_nav/mppi/mppi_separation_acquisition.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <vector>

namespace drone_city_nav::mppi {
namespace {

TEST(MppiControlSequenceTest, UploadsInteriorEsdfDirtyRegion) {
  BenchmarkConfig config;
  config.rollouts = 64U;
  config.steps = 8U;
  MppiCudaEngine engine{config};
  const EsdfGrid grid{
      .width = 161,
      .height = 161,
      .resolution_m = 0.25F,
      .depth = 121,
  };
  std::vector<float> esdf(static_cast<std::size_t>(grid.width) *
                              static_cast<std::size_t>(grid.height) *
                              static_cast<std::size_t>(grid.depth),
                          20.0F);

  ASSERT_TRUE(engine.updateEsdf(EsdfSnapshot{grid, esdf, 1U}).accepted);
  esdf[(60U * static_cast<std::size_t>(grid.height) + 80U) *
           static_cast<std::size_t>(grid.width) +
       80U] = 0.0F;
  const std::array dirty_regions{EsdfDirtyRegion{
      .minimum_x = 80,
      .minimum_y = 80,
      .minimum_z = 60,
      .maximum_x_exclusive = 81,
      .maximum_y_exclusive = 81,
      .maximum_z_exclusive = 61,
  }};

  const EsdfUploadResult patched =
      engine.updateEsdf(EsdfSnapshot{grid, esdf, 2U, dirty_regions});

  EXPECT_TRUE(patched.accepted);
  EXPECT_EQ(patched.revision, 2U);
}

TEST(MppiControlSequenceTest, FractionalShiftInterpolatesWithoutDroppingWholeTick) {
  const std::array<Control, 3> controls{
      Control{.ax = 0.0F},
      Control{.ax = 10.0F},
      Control{.ax = 20.0F},
  };

  const std::vector<Control> shifted = shiftControlSequence(controls, 1.0F, 0.5);

  ASSERT_EQ(shifted.size(), controls.size());
  EXPECT_FLOAT_EQ(shifted[0].ax, 5.0F);
  EXPECT_FLOAT_EQ(shifted[1].ax, 15.0F);
  EXPECT_FLOAT_EQ(shifted[2].ax, 20.0F);
}

TEST(MppiControlSequenceTest, ShiftBeyondHorizonDropsStaleNominal) {
  const std::array<Control, 2> controls{
      Control{.ax = 1.0F},
      Control{.ax = 2.0F},
  };

  const std::vector<Control> shifted = shiftControlSequence(controls, 0.05F, 0.2);

  ASSERT_EQ(shifted.size(), controls.size());
  EXPECT_FLOAT_EQ(shifted[0].ax, 0.0F);
  EXPECT_FLOAT_EQ(shifted[1].ax, 0.0F);
}

TEST(MppiControlSequenceTest, ReseedFollowsRouteWithoutAlternatingLateralBias) {
  DynamicsConfig dynamics;
  const State initial{};
  const State target{.x = 10.0F};
  const std::array route{
      RouteSample3D{.x_m = 0.0F, .y_m = 0.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 10.0F, .y_m = 0.0F, .station_m = 10.0F},
  };

  const std::vector<Control> seed = buildGuideDirectedNominalSeed(
      initial, target, route, 0.0F, 5.0F, dynamics, 8U, Control{});

  ASSERT_FALSE(seed.empty());
  EXPECT_GT(seed.front().ax, 0.0F);
  EXPECT_NEAR(seed.front().ay, 0.0F, 1.0e-6F);
}

TEST(MppiControlSequenceTest, ReseedUsesCurrentRouteAltitudeProfile) {
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  const State initial{.x = 5.0F, .z = 5.0F};
  const State distant_target{.x = 30.0F, .z = 18.0F};
  const std::array route{
      RouteSample3D{.x_m = 0.0F, .y_m = 0.0F, .z_m = 5.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 30.0F, .y_m = 0.0F, .z_m = 5.0F, .station_m = 30.0F},
  };

  const std::vector<Control> seed = buildGuideDirectedNominalSeed(
      initial, distant_target, route, 5.0F, 5.0F, dynamics, 8U, Control{});

  ASSERT_FALSE(seed.empty());
  EXPECT_NEAR(seed.front().az, 0.0F, 1.0e-6F);
}

TEST(MppiControlSequenceTest, RouteProjectionSupportsPureVerticalSegments) {
  const std::array route{
      RouteSample3D{.x_m = 4.0F,
                    .y_m = 7.0F,
                    .z_m = 2.0F,
                    .station_m = 0.0F,
                    .reference_speed_mps = 3.0F},
      RouteSample3D{.x_m = 4.0F,
                    .y_m = 7.0F,
                    .z_m = 12.0F,
                    .station_m = 10.0F,
                    .reference_speed_mps = 5.0F},
  };

  const MppiRouteProjection3D projection =
      projectOntoMppiRoute3D(State{.x = 4.5F, .y = 7.0F, .z = 8.0F}, route, 0.0F);

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.station_m, 6.0F, 1.0e-5F);
  EXPECT_NEAR(projection.distance_m, 0.5F, 1.0e-5F);
  EXPECT_NEAR(projection.reference_z_m, 8.0F, 1.0e-5F);
  EXPECT_NEAR(projection.reference_speed_mps, 4.2F, 1.0e-5F);
}

TEST(MppiControlSequenceTest, RouteProjectionUsesZToDisambiguateStackedSegments) {
  const std::array route{
      RouteSample3D{.x_m = 0.0F, .z_m = 0.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 10.0F, .z_m = 0.0F, .station_m = 10.0F},
      RouteSample3D{.x_m = 10.0F, .z_m = 10.0F, .station_m = 20.0F},
      RouteSample3D{.x_m = 0.0F, .z_m = 10.0F, .station_m = 30.0F},
  };

  const MppiRouteProjection3D projection =
      projectOntoMppiRoute3D(State{.x = 2.0F, .z = 9.0F}, route, 0.0F);

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.station_m, 28.0F, 1.0e-5F);
  EXPECT_NEAR(projection.reference_x_m, 2.0F, 1.0e-5F);
  EXPECT_NEAR(projection.reference_z_m, 10.0F, 1.0e-5F);
}

TEST(MppiControlSequenceTest, FoldedRouteCannotCreditMoreThanPhysicalTravel) {
  const std::array route{
      RouteSample3D{.x_m = 0.0F, .y_m = 0.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 10.0F, .y_m = 0.0F, .station_m = 10.0F},
      RouteSample3D{.x_m = 10.0F, .y_m = 1.0F, .station_m = 11.0F},
      RouteSample3D{.x_m = 0.0F, .y_m = 1.0F, .station_m = 21.0F},
  };
  const MppiRouteProjection3D projection =
      projectOntoMppiRoute3D(State{.x = 0.0F, .y = 1.0F}, route, 0.0F);

  ASSERT_TRUE(projection.valid);
  EXPECT_FLOAT_EQ(projection.station_m, 21.0F);
  EXPECT_FLOAT_EQ(creditedRouteProgressM(projection.station_m, 0.0F, 1.0F), 1.0F);
}

TEST(MppiControlSequenceTest, VerticalRouteProjectionNeverMovesBehindPreviousStation) {
  const std::array route{
      RouteSample3D{.z_m = 0.0F, .station_m = 0.0F},
      RouteSample3D{.z_m = 10.0F, .station_m = 10.0F},
      RouteSample3D{.z_m = 0.0F, .station_m = 20.0F},
  };

  const std::optional<float> station =
      projectForwardRouteStation(route, State{.z = 2.0F}, 15.0F);

  EXPECT_EQ(station, std::optional<float>{18.0F});
}

TEST(MppiControlSequenceTest, RouteSeedTracksThreeDimensionalTangentVelocity) {
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  dynamics.maximum_control_jerk_mps3 = 100.0F;
  const float diagonal_tangent = 1.0F / std::numbers::sqrt2_v<float>;
  const std::array route{
      RouteSample3D{.x_m = 0.0F,
                    .z_m = 0.0F,
                    .tangent_x = diagonal_tangent,
                    .tangent_z = diagonal_tangent,
                    .station_m = 0.0F},
      RouteSample3D{.x_m = 10.0F,
                    .z_m = 10.0F,
                    .tangent_x = diagonal_tangent,
                    .tangent_z = diagonal_tangent,
                    .station_m = 10.0F * std::numbers::sqrt2_v<float>},
  };

  const std::vector<Control> seed =
      buildGuideDirectedNominalSeed(State{}, State{.x = 10.0F, .z = 10.0F}, route, 0.0F,
                                    5.0F, dynamics, 8U, Control{});

  ASSERT_FALSE(seed.empty());
  EXPECT_GT(seed.front().ax, 1.0F);
  EXPECT_GT(seed.front().az, 1.0F);
}

TEST(MppiControlSequenceTest, HostLimiterMatchesAccelerationAndJerkContract) {
  std::array<Control, 2> controls{
      Control{.ax = 20.0F, .ay = 20.0F, .az = 20.0F},
      Control{.ax = -20.0F, .ay = -20.0F, .az = -20.0F},
  };
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  dynamics.maximum_horizontal_acceleration_mps2 = 4.0F;
  dynamics.maximum_vertical_acceleration_mps2 = 3.0F;
  dynamics.maximum_control_jerk_mps3 = 10.0F;

  limitControlSequence(controls, dynamics, Control{}, 0.05F);

  EXPECT_NEAR(controls[0].ax, 0.5F, 1.0e-6F);
  EXPECT_NEAR(controls[0].ay, 0.5F, 1.0e-6F);
  EXPECT_NEAR(controls[0].az, 0.5F, 1.0e-6F);
  EXPECT_NEAR(controls[1].ax, -0.5F, 1.0e-6F);
  EXPECT_NEAR(controls[1].ay, -0.5F, 1.0e-6F);
  EXPECT_NEAR(controls[1].az, -0.5F, 1.0e-6F);
}

TEST(MppiControlSequenceTest, BuildsAllDeterministicCooperativeCandidates) {
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  dynamics.maximum_control_jerk_mps3 = 100.0F;
  const std::array<Control, 20> nominal{};
  const State initial{.vx = 5.0F};

  const std::vector<Control> candidates =
      buildCooperativeManeuverCandidates(initial, State{.x = 100.0F}, nominal, dynamics,
                                         CooperativeConfig{}, Control{}, dynamics.dt_s);

  ASSERT_EQ(candidates.size(), kCooperativeManeuverCandidateCount * nominal.size());
  const auto first = [&](const CooperativeManeuver maneuver) -> const Control& {
    return candidates[static_cast<std::size_t>(maneuver) * nominal.size()];
  };
  EXPECT_FLOAT_EQ(first(CooperativeManeuver::kKeep).ax, 0.0F);
  EXPECT_GT(first(CooperativeManeuver::kClimb).az, 0.0F);
  EXPECT_LT(first(CooperativeManeuver::kDescend).az, 0.0F);
  EXPECT_GT(first(CooperativeManeuver::kLeft).ay, 0.0F);
  EXPECT_LT(first(CooperativeManeuver::kRight).ay, 0.0F);
  EXPECT_LT(first(CooperativeManeuver::kSlow).ax, 0.0F);
}

TEST(MppiControlSequenceTest, FiniteRouteSeedStopsAtTemporaryFrontier) {
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  dynamics.maximum_control_jerk_mps3 = 100.0F;
  const State initial{};
  const State target{.x = 5.0F};
  const std::array route{
      RouteSample3D{.x_m = 0.0F, .tangent_x = 1.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 5.0F, .tangent_x = 1.0F, .station_m = 5.0F},
  };

  const std::vector<Control> controls = buildFiniteRouteDirectedSeed(
      initial, target, route, 0.0F, 5.0F, dynamics, 30U, Control{});
  State terminal = initial;
  for (const Control& control : controls) {
    terminal = integrateReference(terminal, control, dynamics);
  }

  EXPECT_LE(terminal.x, route.back().x_m + 0.25F);
  EXPECT_LT(std::abs(terminal.vx), 0.5F);
}

TEST(MppiControlSequenceTest,
     SelectsBestFiniteRouteCandidateInsteadOfDilutingItInWeightedUpdate) {
  BenchmarkConfig config;
  config.rollouts = 512U;
  config.steps = 40U;
  config.dynamics.dt_s = 0.05F;
  config.noise.horizontal_acceleration_sigma_mps2 = 0.01F;
  config.noise.vertical_acceleration_sigma_mps2 = 0.01F;
  config.noise.yaw_acceleration_sigma_radps2 = 0.01F;
  config.costs.temperature = 1000.0F;
  config.seed = 31U;
  MppiCudaEngine engine{config};
  const EsdfGrid grid{.width = 50,
                      .height = 40,
                      .resolution_m = 1.0F,
                      .origin_x_m = 0.0F,
                      .origin_y_m = 0.0F,
                      .depth = 20,
                      .origin_z_m = 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(50U) * 40U * 20U, 20.0F);
  ASSERT_TRUE(engine.updateEsdf(EsdfSnapshot{grid, esdf, 1U}).accepted);
  auto route =
      std::make_shared<const std::vector<RouteSample3D>>(std::vector<RouteSample3D>{
          RouteSample3D{.x_m = 5.0F,
                        .y_m = 20.0F,
                        .z_m = 10.0F,
                        .tangent_x = 1.0F,
                        .station_m = 0.0F},
          RouteSample3D{.x_m = 35.0F,
                        .y_m = 20.0F,
                        .z_m = 10.0F,
                        .tangent_x = 1.0F,
                        .station_m = 30.0F},
      });
  MppiTickInput input;
  input.initial_state = State{.x = 5.0F, .y = 20.0F, .z = 10.0F};
  input.target = State{.x = 35.0F, .y = 20.0F, .z = 10.0F};
  input.planning_stamp_ns = 1;
  input.reference_speed_mps = 5.0F;
  input.route = RouteReference{.points = std::move(route),
                               .generation = 1U,
                               .terminal_cross_track_tolerance_m = std::nullopt};
  input.deterministic_candidate = DeterministicCandidateKind::kRouteDirectedCruise;

  const MppiTickResult result = engine.plan(input);

  EXPECT_TRUE(result.route_directed_candidate_raw_safe);
  EXPECT_TRUE(result.route_directed_candidate_best_feasible);
  EXPECT_EQ(result.control_selection, MppiControlSelection::kRouteDirectedCandidate);
  ASSERT_FALSE(result.controls.empty());
  ASSERT_FALSE(result.horizon.empty());
  EXPECT_GT(result.controls.front().ax, 0.0F);
  EXPECT_GT(result.terminal_progress_m, 1.0F);
}

TEST(MppiControlSequenceTest,
     RepairsPhysicallySafeWeightedUpdateWithRouteConvergentCandidate) {
  BenchmarkConfig config;
  config.rollouts = 128U;
  config.steps = 80U;
  config.dynamics.dt_s = 0.05F;
  config.noise.horizontal_acceleration_sigma_mps2 = 0.0F;
  config.noise.vertical_acceleration_sigma_mps2 = 0.0F;
  config.noise.yaw_acceleration_sigma_radps2 = 0.0F;
  config.costs.guide_deviation_weight = 0.0F;
  config.costs.altitude_tracking_weight = 0.0F;
  config.costs.head_progress_weight = 0.0F;
  config.costs.progress_weight = 0.0F;
  config.costs.route_progress_integral_weight = 0.0F;
  config.costs.speed_tracking_weight = 0.0F;
  config.costs.terminal_weight = 0.0F;
  config.seed = 37U;
  MppiCudaEngine engine{config};
  const EsdfGrid grid{.width = 50,
                      .height = 40,
                      .resolution_m = 1.0F,
                      .origin_x_m = 0.0F,
                      .origin_y_m = 0.0F,
                      .depth = 20,
                      .origin_z_m = 0.0F};
  const std::vector<float> esdf(static_cast<std::size_t>(50U) * 40U * 20U, 20.0F);
  ASSERT_TRUE(engine.updateEsdf(EsdfSnapshot{grid, esdf, 1U}).accepted);
  auto route =
      std::make_shared<const std::vector<RouteSample3D>>(std::vector<RouteSample3D>{
          RouteSample3D{.x_m = 5.0F,
                        .y_m = 20.0F,
                        .z_m = 10.0F,
                        .tangent_x = 1.0F,
                        .station_m = 0.0F},
          RouteSample3D{.x_m = 35.0F,
                        .y_m = 20.0F,
                        .z_m = 10.0F,
                        .tangent_x = 1.0F,
                        .station_m = 30.0F},
      });
  MppiTickInput input;
  input.initial_state = State{.x = 5.0F, .y = 11.5F, .z = 10.0F};
  input.target = State{.x = 35.0F, .y = 20.0F, .z = 10.0F};
  input.planning_stamp_ns = 1;
  input.reference_speed_mps = 5.0F;
  input.route = RouteReference{.points = std::move(route),
                               .generation = 1U,
                               .terminal_cross_track_tolerance_m = 2.0F};
  input.deterministic_candidate = DeterministicCandidateKind::kRouteDirectedCruise;

  const std::vector<Control> route_controls = buildFiniteRouteDirectedSeed(
      input.initial_state, input.target, *input.route->points,
      input.route->initial_station_m, input.reference_speed_mps, config.dynamics,
      config.steps, Control{}, config.stopping_capability);
  State route_terminal = input.initial_state;
  for (const Control& control : route_controls) {
    route_terminal = integrateReference(route_terminal, control, config.dynamics);
  }
  const MppiRouteProjection3D route_terminal_projection = projectOntoMppiRoute3D(
      route_terminal, *input.route->points, input.route->initial_station_m);
  ASSERT_TRUE(route_terminal_projection.valid);
  ASSERT_LE(route_terminal_projection.distance_m, 2.0F);

  const MppiTickResult result = engine.plan(input);

  EXPECT_TRUE(result.route_directed_candidate_raw_safe);
  EXPECT_FALSE(result.route_directed_candidate_best_feasible);
  EXPECT_EQ(result.post_update_repair, MppiPostUpdateRepair::kDeterministicCandidate);
  EXPECT_EQ(result.control_selection, MppiControlSelection::kRouteDirectedCandidate);
  EXPECT_TRUE(result.post_update_classification.executable);
  EXPECT_FALSE(result.route_terminal_cross_track_violation);
  EXPECT_GE(result.terminal_route_cross_track_m, 0.0F);
  EXPECT_LE(result.terminal_route_cross_track_m, 2.0F);
}

TEST(MppiControlSequenceTest, AcquisitionCombinesRouteAccelerationAndClimb) {
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  dynamics.maximum_control_jerk_mps3 = 100.0F;
  const State initial{.z = 10.0F};
  const State target{.x = 100.0F, .z = 10.0F};
  const std::array route{
      RouteSample3D{.x_m = 0.0F, .z_m = 10.0F, .tangent_x = 1.0F, .station_m = 0.0F},
      RouteSample3D{
          .x_m = 100.0F, .z_m = 10.0F, .tangent_x = 1.0F, .station_m = 100.0F},
  };
  const CooperativeSeparationAcquisition acquisition{
      .preference =
          CooperativeManeuverPreference{
              .maneuver = CooperativeManeuver::kClimb,
              .direction_z = 1.0F,
              .generation = 1U,
          },
  };

  const std::vector<Control> candidates =
      buildCooperativeSeparationAcquisitionCandidates(
          initial, target, route, 0.0F, 10.0F, acquisition, dynamics,
          CooperativeConfig{}, 20U, Control{}, dynamics.dt_s, StoppingCapability{});

  ASSERT_EQ(candidates.size(), kCooperativeAcquisitionCandidateCount * 20U);
  EXPECT_GT(candidates.front().ax, 0.0F);
  EXPECT_GT(candidates.front().az, 0.0F);
  EXPECT_LT(candidates[(kCooperativeAcquisitionCandidateCount - 1U) * 20U].ax, 0.0F);
}

TEST(MppiControlSequenceTest, CudaEngineEvaluatesCooperativePeers) {
  BenchmarkConfig config;
  config.rollouts = 64U;
  config.steps = 20U;
  config.dynamics.dt_s = 0.1F;
  config.seed = 17U;
  MppiCudaEngine engine{config};
  const EsdfGrid grid{40, 20, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(800U, 20.0F);
  ASSERT_TRUE(engine.updateEsdf(EsdfSnapshot{grid, esdf, 1U}).accepted);
  auto peer_samples = std::make_shared<const std::vector<DynamicAircraftSample>>(
      config.steps, DynamicAircraftSample{.x = 8.0F, .y = 10.0F});

  MppiTickInput input;
  input.initial_state = State{.x = 5.0F, .y = 10.0F, .vx = 2.0F};
  input.target = State{.x = 30.0F, .y = 10.0F};
  input.planning_stamp_ns = 1;
  input.dynamic_aircraft = {
      DynamicAircraftTrajectory{.samples = std::move(peer_samples),
                                .footprint_radius_m = 0.82F,
                                .active_steps = config.steps}};
  input.cooperative_maneuver = CooperativeManeuverPreference{
      .maneuver = CooperativeManeuver::kLeft,
      .direction_y = 1.0F,
      .generation = 1U,
  };
  input.cooperative_avoidance_active = true;

  const MppiTickResult result = engine.plan(input);

  EXPECT_TRUE(result.cooperative_candidates_injected);
  EXPECT_EQ(result.dynamic_aircraft_count, 1U);
  EXPECT_TRUE(std::isfinite(result.minimum_peer_separation_m));
  EXPECT_GT(result.peer_separation_cost, 0.0F);
}

TEST(MppiControlSequenceTest, ReverseAcquisitionIsOnlyABackwardFallback) {
  BenchmarkConfig config;
  config.steps = 30U;
  config.dynamics.dt_s = 0.1F;
  config.dynamics.maximum_control_jerk_mps3 = 100.0F;
  const EsdfGrid grid{.width = 40,
                      .height = 20,
                      .resolution_m = 1.0F,
                      .origin_x_m = 0.0F,
                      .origin_y_m = 0.0F,
                      .depth = 20,
                      .origin_z_m = 0.0F};
  constexpr std::size_t kEsdfCellCount = static_cast<std::size_t>(40U) * 20U * 20U;
  const std::vector<float> esdf(kEsdfCellCount, 20.0F);
  const std::array route{
      RouteSample3D{.x_m = 5.0F,
                    .y_m = 10.0F,
                    .z_m = 10.0F,
                    .tangent_x = 1.0F,
                    .station_m = 0.0F},
      RouteSample3D{.x_m = 30.0F,
                    .y_m = 10.0F,
                    .z_m = 10.0F,
                    .tangent_x = 1.0F,
                    .station_m = 25.0F},
  };
  const auto peer_samples = std::make_shared<const std::vector<DynamicAircraftSample>>(
      config.steps, DynamicAircraftSample{.x = 25.0F, .y = 10.0F, .z = 10.0F});
  const std::array peers{DynamicAircraftTrajectory{
      .samples = peer_samples,
      .footprint_radius_m = 0.82F,
      .active_steps = config.steps,
  }};

  const CooperativeSeparationAcquisitionResult result =
      evaluateCooperativeSeparationAcquisition(
          CooperativeSeparationAcquisitionEvaluationInput{
              .initial_state = State{.x = 5.0F, .y = 10.0F, .z = 10.0F},
              .target = State{.x = 30.0F, .y = 10.0F, .z = 10.0F},
              .route = route,
              .initial_route_station_m = 0.0F,
              .reference_speed_mps = 5.0F,
              .previous_applied_control = {},
              .first_control_interval_s = config.dynamics.dt_s,
              .grid = grid,
              .esdf = esdf,
              .known_solids = {},
              .aircraft = peers,
              .acquisition = CooperativeSeparationAcquisition{},
              .config = config,
          });

  ASSERT_TRUE(result.available);
  EXPECT_TRUE(result.backward_fallback);
  EXPECT_FALSE(result.positive_progress);
  EXPECT_LT(result.terminal_progress_m, 0.0F);
}

TEST(MppiControlSequenceTest, AvoidanceReseedsOnceOnEntryAndRelease) {
  BenchmarkConfig config;
  config.rollouts = 64U;
  config.steps = 20U;
  config.dynamics.dt_s = 0.1F;
  config.dynamics.maximum_control_jerk_mps3 = 100.0F;
  MppiCudaEngine engine{config};
  const EsdfGrid grid{.width = 40,
                      .height = 20,
                      .resolution_m = 1.0F,
                      .origin_x_m = 0.0F,
                      .origin_y_m = 0.0F,
                      .depth = 20,
                      .origin_z_m = 0.0F};
  constexpr std::size_t kEsdfCellCount = static_cast<std::size_t>(40U) * 20U * 20U;
  const std::vector<float> esdf(kEsdfCellCount, 20.0F);
  ASSERT_TRUE(engine.updateEsdf(EsdfSnapshot{grid, esdf, 1U}).accepted);
  auto route =
      std::make_shared<const std::vector<RouteSample3D>>(std::vector<RouteSample3D>{
          RouteSample3D{.x_m = 5.0F,
                        .y_m = 10.0F,
                        .z_m = 10.0F,
                        .tangent_x = 1.0F,
                        .station_m = 0.0F},
          RouteSample3D{.x_m = 30.0F,
                        .y_m = 10.0F,
                        .z_m = 10.0F,
                        .tangent_x = 1.0F,
                        .station_m = 25.0F},
      });
  auto peers = std::make_shared<const std::vector<DynamicAircraftSample>>(
      config.steps, DynamicAircraftSample{.x = 8.0F, .y = 10.0F, .z = 10.0F});
  MppiTickInput input;
  input.initial_state = State{.x = 5.0F, .y = 10.0F, .z = 10.0F};
  input.target = State{.x = 30.0F, .y = 10.0F, .z = 10.0F};
  input.planning_stamp_ns = 1;
  input.reference_speed_mps = 5.0F;
  input.route = RouteReference{.points = route,
                               .generation = 1U,
                               .terminal_cross_track_tolerance_m = std::nullopt};
  input.dynamic_aircraft = {DynamicAircraftTrajectory{
      .samples = peers, .footprint_radius_m = 0.82F, .active_steps = config.steps}};
  input.cooperative_maneuver = CooperativeManeuverPreference{
      .maneuver = CooperativeManeuver::kClimb,
      .direction_z = 1.0F,
      .generation = 1U,
  };
  input.cooperative_acquisition = CooperativeSeparationAcquisition{
      .preference = *input.cooperative_maneuver,
  };
  input.cooperative_avoidance_active = true;

  const MppiTickResult entered = engine.plan(input);
  EXPECT_TRUE(entered.cooperative_acquisition_reseeded);
  EXPECT_TRUE(entered.cooperative_acquisition_available);
  EXPECT_TRUE(entered.cooperative_acquisition_positive_progress);
  EXPECT_FALSE(entered.cooperative_acquisition_backward_fallback);

  input.planning_stamp_ns = 100'000'001;
  input.dynamic_aircraft.clear();
  input.cooperative_maneuver.reset();
  input.cooperative_acquisition.reset();
  input.cooperative_avoidance_active = false;
  const MppiTickResult released = engine.plan(input);
  EXPECT_TRUE(released.cooperative_release_reseeded);

  input.planning_stamp_ns = 200'000'001;
  const MppiTickResult steady = engine.plan(input);
  EXPECT_FALSE(steady.cooperative_release_reseeded);
}

TEST(MppiControlSequenceTest, AvoidanceWithoutPeerCoverageDoesNotReseed) {
  BenchmarkConfig config;
  config.rollouts = 64U;
  config.steps = 20U;
  MppiCudaEngine engine{config};
  const EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(400U, 20.0F);
  ASSERT_TRUE(engine.updateEsdf(EsdfSnapshot{grid, esdf, 1U}).accepted);
  MppiTickInput input;
  input.initial_state = State{.x = 5.0F, .y = 10.0F};
  input.target = State{.x = 15.0F, .y = 10.0F};
  input.planning_stamp_ns = 1;
  input.cooperative_acquisition = CooperativeSeparationAcquisition{};
  input.cooperative_avoidance_active = true;

  const MppiTickResult entered_without_coverage = engine.plan(input);
  EXPECT_FALSE(entered_without_coverage.cooperative_acquisition_reseeded);

  input.planning_stamp_ns = 100'000'001;
  input.cooperative_acquisition.reset();
  input.cooperative_avoidance_active = false;
  const MppiTickResult released_without_acquisition = engine.plan(input);
  EXPECT_FALSE(released_without_acquisition.cooperative_release_reseeded);
}

TEST(MppiControlSequenceTest, RouteProjectionNeverMovesBehindPreviousStation) {
  const std::array route{
      RouteSample3D{.x_m = 0.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 10.0F, .station_m = 10.0F},
      RouteSample3D{.x_m = 0.0F, .station_m = 20.0F},
  };

  const std::optional<float> station =
      projectForwardRouteStation(route, State{.x = 2.0F}, 15.0F);

  EXPECT_EQ(station, std::optional<float>{18.0F});
}

TEST(MppiControlSequenceTest, RouteTrackingSpeedUsesTheThreeDimensionalTangent) {
  const std::array flat_route{
      RouteSample3D{.x_m = 0.0F, .z_m = 10.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 10.0F, .z_m = 10.0F, .station_m = 10.0F},
  };
  const MppiRouteProjection3D flat =
      projectOntoMppiRoute3D(State{.x = 5.0F, .z = 10.0F}, flat_route, 0.0F);
  ASSERT_TRUE(flat.valid);
  EXPECT_FLOAT_EQ(routeTrackingSpeedMps(State{.vx = 5.0F, .vz = 8.0F}, flat), 5.0F);

  const float diagonal_length = std::sqrt(200.0F);
  const std::array climbing_route{
      RouteSample3D{.station_m = 0.0F},
      RouteSample3D{.x_m = 10.0F, .z_m = 10.0F, .station_m = diagonal_length},
  };
  const MppiRouteProjection3D climbing =
      projectOntoMppiRoute3D(State{.x = 5.0F, .z = 5.0F}, climbing_route, 0.0F);
  ASSERT_TRUE(climbing.valid);
  EXPECT_NEAR(routeTrackingSpeedMps(State{.vx = 5.0F, .vz = 5.0F}, climbing),
              std::sqrt(50.0F), 1.0e-5F);
  EXPECT_FLOAT_EQ(routeTrackingSpeedMps(State{.vx = -5.0F, .vz = -5.0F}, climbing),
                  0.0F);
}

TEST(MppiControlSequenceTest, RequiredRiskTierIsLocalToRouteInterval) {
  const std::array route{
      RouteSample3D{.station_m = 0.0F, .required_risk_tier = RiskTier::kPreferred},
      RouteSample3D{.station_m = 10.0F, .required_risk_tier = RiskTier::kPlanning},
      RouteSample3D{.station_m = 20.0F, .required_risk_tier = RiskTier::kCritical},
      RouteSample3D{.station_m = 30.0F, .required_risk_tier = RiskTier::kPreferred},
  };

  EXPECT_EQ(maximumRequiredRiskTier(route, 0.0F, 8.0F), RiskTier::kPlanning);
  EXPECT_EQ(maximumRequiredRiskTier(route, 12.0F, 25.0F), RiskTier::kCritical);
}

} // namespace
} // namespace drone_city_nav::mppi
