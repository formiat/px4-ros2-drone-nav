#include "drone_city_nav/mppi/mppi_noncooperative_acquisition.hpp"

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <vector>

namespace drone_city_nav::mppi {
namespace {

TEST(MppiNonCooperativeAcquisitionTest,
     MaximizesSeparationWhenNoCandidateCanPreserveStrongThreshold) {
  BenchmarkConfig config;
  config.steps = 40U;
  config.dynamics.dt_s = 0.1F;
  config.dynamics.linear_drag_1ps = 0.0F;
  config.dynamics.maximum_control_jerk_mps3 = 100.0F;
  const EsdfGrid grid{80, 40, 1.0F, -20.0F, -20.0F};
  const std::vector<float> esdf(3200U, 40.0F);
  const std::array route{
      RouteSample3D{.x_m = 0.0F,
                    .tangent_x = 1.0F,
                    .station_m = 0.0F,
                    .reference_speed_mps = 8.0F},
      RouteSample3D{.x_m = 40.0F,
                    .tangent_x = 1.0F,
                    .station_m = 40.0F,
                    .reference_speed_mps = 8.0F},
  };
  auto aircraft_samples = std::make_shared<std::vector<DynamicAircraftSample>>();
  aircraft_samples->reserve(config.steps);
  for (std::size_t step = 0U; step < config.steps; ++step) {
    aircraft_samples->push_back(
        DynamicAircraftSample{.x = 8.0F - static_cast<float>(step) * 0.2F});
  }
  const std::array aircraft{DynamicAircraftTrajectory{
      .samples = aircraft_samples,
      .footprint_radius_m = 0.82F,
      .active_steps = config.steps,
  }};

  const NonCooperativeAcquisitionResult result =
      evaluateNonCooperativeAcquisition(NonCooperativeAcquisitionEvaluationInput{
          .initial_state = State{},
          .target = State{.x = 40.0F},
          .route = route,
          .initial_route_station_m = 0.0F,
          .reference_speed_mps = 8.0F,
          .previous_applied_control = {},
          .first_control_interval_s = config.dynamics.dt_s,
          .grid = grid,
          .esdf = esdf,
          .known_solids = {},
          .aircraft = aircraft,
          .acquisition =
              NonCooperativeSeparationAcquisition{
                  .threat_direction_x = 1.0F,
                  .candidate_acceleration_fraction = 0.95F,
                  .candidate_duration_s = 1.5F,
                  .generation = 1U,
              },
          .cost_policy =
              DynamicAircraftCostPolicy{
                  .strong_separation_m = 10.0F,
                  .anticipation_separation_m = 20.0F,
                  .strong_weight = 4000.0F,
                  .anticipation_weight = 40.0F,
                  .time_to_collision_gain_s = 1.0F,
                  .maximum_time_to_collision_multiplier = 4.0F,
              },
          .config = config,
      });

  ASSERT_TRUE(result.available);
  EXPECT_NE(result.maneuver, NonCooperativeManeuver::kRouteCruise);
  EXPECT_GT(result.minimum_separation_m, 0.0F);
  EXPECT_GE(result.separation_gain_m, 0.0F);
  EXPECT_EQ(result.controls.size(), config.steps);
}

TEST(MppiNonCooperativeAcquisitionTest,
     PrefersRouteProgressOnceStrongSeparationIsPreserved) {
  BenchmarkConfig config;
  config.steps = 40U;
  config.dynamics.dt_s = 0.1F;
  config.dynamics.linear_drag_1ps = 0.0F;
  config.dynamics.maximum_control_jerk_mps3 = 100.0F;
  const EsdfGrid grid{80, 80, 1.0F, -20.0F, -40.0F};
  const std::vector<float> esdf(6400U, 40.0F);
  const std::array route{
      RouteSample3D{.x_m = 0.0F,
                    .tangent_x = 1.0F,
                    .station_m = 0.0F,
                    .reference_speed_mps = 8.0F},
      RouteSample3D{.x_m = 40.0F,
                    .tangent_x = 1.0F,
                    .station_m = 40.0F,
                    .reference_speed_mps = 8.0F},
  };
  auto aircraft_samples = std::make_shared<std::vector<DynamicAircraftSample>>();
  aircraft_samples->reserve(config.steps);
  for (std::size_t step = 0U; step < config.steps; ++step) {
    aircraft_samples->push_back(DynamicAircraftSample{
        .x = 20.0F,
        .y = -30.0F + static_cast<float>(step + 1U) * 0.5F,
    });
  }
  const std::array aircraft{DynamicAircraftTrajectory{
      .samples = aircraft_samples,
      .footprint_radius_m = 0.82F,
      .active_steps = config.steps,
  }};

  const NonCooperativeAcquisitionResult result =
      evaluateNonCooperativeAcquisition(NonCooperativeAcquisitionEvaluationInput{
          .initial_state = State{},
          .target = State{.x = 40.0F},
          .route = route,
          .initial_route_station_m = 0.0F,
          .reference_speed_mps = 8.0F,
          .previous_applied_control = {},
          .first_control_interval_s = config.dynamics.dt_s,
          .grid = grid,
          .esdf = esdf,
          .known_solids = {},
          .aircraft = aircraft,
          .acquisition =
              NonCooperativeSeparationAcquisition{
                  .threat_direction_x = 0.55F,
                  .threat_direction_y = -0.83F,
                  .candidate_acceleration_fraction = 0.95F,
                  .candidate_duration_s = 1.5F,
                  .generation = 1U,
              },
          .cost_policy =
              DynamicAircraftCostPolicy{
                  .strong_separation_m = 10.0F,
                  .anticipation_separation_m = 20.0F,
                  .strong_weight = 4000.0F,
                  .anticipation_weight = 40.0F,
                  .time_to_collision_gain_s = 1.0F,
                  .maximum_time_to_collision_multiplier = 4.0F,
              },
          .config = config,
      });

  ASSERT_TRUE(result.available);
  EXPECT_GE(result.minimum_separation_m, 10.0F);
  EXPECT_GE(result.head_progress_m, 0.0F);
  EXPECT_GE(result.terminal_progress_m, 0.0F);
  EXPECT_EQ(result.controls.size(), config.steps);
}

TEST(MppiNonCooperativeAcquisitionTest, LifecycleReseedsOnceOnEntryAndRelease) {
  NonCooperativeAcquisitionLifecycle lifecycle;
  NonCooperativeAcquisitionEvaluationInput evaluation;
  evaluation.config.steps = 2U;
  evaluation.config.dynamics.dt_s = 0.1F;
  evaluation.reference_speed_mps = 1.0F;
  evaluation.grid = EsdfGrid{4, 4, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(16U, 10.0F);
  evaluation.esdf = esdf;
  const auto samples = std::make_shared<const std::vector<DynamicAircraftSample>>(
      2U, DynamicAircraftSample{.x = 2.0F});
  const std::array aircraft{DynamicAircraftTrajectory{
      .samples = samples, .footprint_radius_m = 0.82F, .active_steps = 2U}};
  evaluation.aircraft = aircraft;
  const NonCooperativeSeparationAcquisition acquisition{
      .threat_direction_x = 1.0F,
      .generation = 1U,
  };

  const NonCooperativeAcquisitionLifecycleResult entered =
      lifecycle.update(NonCooperativeAcquisitionLifecycleInput{
          .avoidance_active = true,
          .acquisition = acquisition,
          .evaluation = evaluation,
      });
  const NonCooperativeAcquisitionLifecycleResult held =
      lifecycle.update(NonCooperativeAcquisitionLifecycleInput{
          .avoidance_active = true,
          .acquisition = acquisition,
          .evaluation = evaluation,
      });
  const NonCooperativeAcquisitionLifecycleResult released =
      lifecycle.update(NonCooperativeAcquisitionLifecycleInput{
          .avoidance_active = false,
          .acquisition = std::nullopt,
          .evaluation = evaluation,
      });

  EXPECT_TRUE(entered.acquisition_reseeded);
  EXPECT_FALSE(held.acquisition_reseeded);
  EXPECT_TRUE(released.release_reseeded);
}

} // namespace
} // namespace drone_city_nav::mppi
