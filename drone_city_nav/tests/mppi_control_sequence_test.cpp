#include "drone_city_nav/mppi/mppi_control_sequence.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <vector>

namespace drone_city_nav::mppi {
namespace {

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
  auto peer_samples = std::make_shared<const std::vector<CooperativePeerSample>>(
      config.steps, CooperativePeerSample{.x = 8.0F, .y = 10.0F});

  MppiTickInput input;
  input.initial_state = State{.x = 5.0F, .y = 10.0F, .vx = 2.0F};
  input.target = State{.x = 30.0F, .y = 10.0F};
  input.planning_stamp_ns = 1;
  input.conflicting_peers = {CooperativePeerTrajectory{
      .samples = std::move(peer_samples), .footprint_radius_m = 0.82F}};
  input.cooperative_maneuver = CooperativeManeuverPreference{
      .maneuver = CooperativeManeuver::kLeft,
      .direction_y = 1.0F,
      .generation = 1U,
  };

  const MppiTickResult result = engine.plan(input);

  EXPECT_TRUE(result.cooperative_candidates_injected);
  EXPECT_EQ(result.cooperative_peer_count, 1U);
  EXPECT_TRUE(std::isfinite(result.minimum_peer_separation_m));
  EXPECT_GT(result.peer_separation_cost, 0.0F);
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
