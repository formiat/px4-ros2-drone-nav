#include "drone_city_nav/mppi_horizon_safety.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(MppiHorizonSafetyTest, ExecutesCollisionFreeHorizon) {
  const mppi::EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(400U, 10.0F);
  const std::vector<mppi::State> horizon{mppi::State{1.0F, 1.0F},
                                         mppi::State{2.0F, 1.0F}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      mppi::State{1.0F, 1.0F}, horizon, esdf, grid, MppiHorizonSafetyConfig{});

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_TRUE(result.fallback_horizon.empty());
}

TEST(MppiHorizonSafetyTest, RejectsRotorFootprintContactWhenCenterCellIsFree) {
  const mppi::EsdfGrid grid{10, 10, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(100U, 10.0F);
  esdf[2U * 10U + 3U] = 0.0F;
  const mppi::State state{2.5F, 2.5F};
  const std::vector<mppi::State> horizon{state, state};

  const MppiHorizonSafetyResult point_result = evaluateMppiHorizonSafety(
      state, horizon, esdf, grid,
      MppiHorizonSafetyConfig{.physical_footprint_radius_m = 0.0});
  const MppiHorizonSafetyResult footprint_result = evaluateMppiHorizonSafety(
      state, horizon, esdf, grid,
      MppiHorizonSafetyConfig{.physical_footprint_radius_m = 0.82});

  EXPECT_EQ(point_result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_NE(footprint_result.decision, MppiHorizonSafetyDecision::kExecute);
}

TEST(MppiHorizonSafetyTest, DelaysInterventionForDistantCollision) {
  const mppi::EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(400U, 10.0F);
  esdf[1U * 20U + 6U] = 0.0F;
  std::vector<mppi::State> horizon;
  horizon.reserve(8U);
  for (int index = 0; index < 8; ++index) {
    horizon.push_back(mppi::State{static_cast<float>(index), 1.0F});
  }
  mppi::State current{1.0F, 1.0F};
  current.vx = 2.0F;

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, esdf, grid,
      MppiHorizonSafetyConfig{.minimum_time_to_collision_s = 0.1, .dt_s = 0.2});

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kExecuteUntilDeadline);
  EXPECT_GT(result.latest_safe_intervention_time_s, 0.0);
  ASSERT_FALSE(result.fallback_horizon.empty());
  EXPECT_LT(result.fallback_horizon.back().vx, current.vx);
}

TEST(MppiHorizonSafetyTest, KeepsEarliestInterventionDeadlineAcrossTicks) {
  MppiSafetyInterventionTracker tracker;
  MppiHorizonSafetyResult first;
  first.decision = MppiHorizonSafetyDecision::kExecuteUntilDeadline;
  first.latest_safe_intervention_time_s = 1.0;
  const MppiSafetyInterventionUpdate first_update =
      tracker.update(1'000'000'000, first);

  MppiHorizonSafetyResult later = first;
  later.latest_safe_intervention_time_s = 2.0;
  const MppiSafetyInterventionUpdate later_update =
      tracker.update(1'500'000'000, later);
  const MppiSafetyInterventionUpdate expired = tracker.update(2'000'000'000, later);

  ASSERT_TRUE(first_update.deadline_ns.has_value());
  EXPECT_EQ(later_update.deadline_ns, first_update.deadline_ns);
  EXPECT_EQ(later_update.decision, MppiHorizonSafetyDecision::kExecuteUntilDeadline);
  EXPECT_EQ(expired.decision, MppiHorizonSafetyDecision::kBrake);
}

TEST(MppiHorizonSafetyTest, ProducesFallbackForEngineOnlyCollision) {
  const mppi::EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(400U, 10.0F);
  const std::vector<mppi::State> horizon{mppi::State{1.0F, 1.0F},
                                         mppi::State{2.0F, 1.0F}};
  mppi::State current{1.0F, 1.0F};
  current.vx = 2.0F;

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, esdf, grid, MppiHorizonSafetyConfig{}, true);

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kBrake);
  ASSERT_FALSE(result.fallback_controls.empty());
  ASSERT_EQ(result.fallback_horizon.size(), result.fallback_controls.size() + 1U);
  EXPECT_LT(result.fallback_horizon.back().vx, current.vx);
}

TEST(MppiHorizonSafetyTest, RejectsHorizonIntersectingKnownThreeDimensionalSolid) {
  const mppi::EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(400U, 10.0F);
  const std::vector<mppi::State> horizon{
      mppi::State{1.0F, 1.0F, 5.0F},
      mppi::State{5.0F, 5.0F, 5.0F},
  };
  const std::vector<mppi::KnownSolid> solids{
      mppi::KnownSolid{
          .center_x_m = 5.0F,
          .center_y_m = 5.0F,
          .normal_x = 1.0F,
          .normal_y = 0.0F,
          .lateral_x = 0.0F,
          .lateral_y = 1.0F,
          .half_depth_m = 1.0F,
          .half_width_m = 1.0F,
          .min_z_m = 4.0F,
          .max_z_m = 6.0F,
      },
  };

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      horizon.front(), horizon, esdf, grid, MppiHorizonSafetyConfig{}, false, solids);

  EXPECT_NE(result.decision, MppiHorizonSafetyDecision::kExecute);
}

TEST(MppiHorizonSafetyTest, StaticFallbackStopsFromTwentyMetersPerSecond) {
  const mppi::EsdfGrid grid{200, 2, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(400U, 10.0F);
  esdf[100U] = 0.0F;
  const std::vector<mppi::State> horizon{mppi::State{0.5F, 0.5F},
                                         mppi::State{100.5F, 0.5F}};
  mppi::State current{0.5F, 0.5F};
  current.vx = 20.0F;

  const MppiHorizonSafetyResult result =
      evaluateMppiHorizonSafety(current, horizon, esdf, grid,
                                MppiHorizonSafetyConfig{
                                    .minimum_time_to_collision_s = 0.1,
                                    .fallback_duration_s = 3.0,
                                    .dt_s = 0.05,
                                });

  ASSERT_FALSE(result.fallback_horizon.empty());
  EXPECT_NEAR(result.fallback_horizon.back().vx, 0.0F, 1.0e-4F);
}

TEST(MppiHorizonSafetyTest, DoesNotTreatComputationalGridBoundaryAsCollision) {
  const mppi::EsdfGrid grid{2, 2, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(4U, 10.0F);
  const std::vector<mppi::State> horizon{mppi::State{1.0F, 1.0F},
                                         mppi::State{3.0F, 1.0F}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      mppi::State{1.0F, 1.0F}, horizon, esdf, grid, MppiHorizonSafetyConfig{});

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kExecute);
}

TEST(MppiHorizonSafetyTest, UsesGlobalRawOccupancyOutsideLocalStaticEsdf) {
  const mppi::EsdfGrid local_grid{2, 2, 1.0F, 0.0F, 0.0F, 4, 0.0F};
  const std::vector<float> local_esdf(16U, 10.0F);
  OccupancyGrid3D global_raw{GridBounds3D{0.0, 0.0, 0.0, 1.0, 8, 4, 4}};
  global_raw.setOccupied(GridIndex3D{4, 1, 1});
  const mppi::State current{1.0F, 1.5F, 1.5F};
  const std::vector<mppi::State> horizon{current, mppi::State{6.0F, 1.5F, 1.5F}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, local_esdf, local_grid,
      MppiHorizonSafetyConfig{.swept_validation_step_m = 0.25,
                              .physical_footprint_radius_m = 0.25},
      false, {}, &global_raw);

  EXPECT_NE(result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_TRUE(result.global_raw_collision);
  EXPECT_GT(result.global_raw_fallback_samples, 0U);
}

TEST(MppiHorizonSafetyTest, AllowsRawFreeSpaceOutsideLocalStaticEsdf) {
  const mppi::EsdfGrid local_grid{2, 2, 1.0F, 0.0F, 0.0F, 4, 0.0F};
  const std::vector<float> local_esdf(16U, 10.0F);
  const OccupancyGrid3D global_raw{GridBounds3D{0.0, 0.0, 0.0, 1.0, 8, 4, 4}};
  const mppi::State current{1.0F, 1.5F, 1.5F};
  const std::vector<mppi::State> horizon{current, mppi::State{6.0F, 1.5F, 1.5F}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, local_esdf, local_grid,
      MppiHorizonSafetyConfig{.swept_validation_step_m = 0.25,
                              .physical_footprint_radius_m = 0.25},
      false, {}, &global_raw);

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_FALSE(result.global_raw_collision);
  EXPECT_GT(result.global_raw_fallback_samples, 0U);
}

TEST(MppiHorizonSafetyTest, ValidatesEveryHorizonSampleAgainstNewestRawSnapshot) {
  const mppi::EsdfGrid local_grid{2, 2, 1.0F, 0.0F, 0.0F};
  const std::vector<float> local_esdf(4U, 10.0F);
  const GridBounds raw_bounds{0.0, 0.0, 1.0, 8, 4};
  OccupancyGrid2D latest_raw{raw_bounds};
  latest_raw.reset(CellState::kFree);
  latest_raw.setOccupied(GridIndex{4, 1});
  const mppi::State current{1.0F, 1.5F, 5.0F};
  const std::vector<mppi::State> horizon{current, mppi::State{6.0F, 1.5F, 5.0F}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, local_esdf, local_grid,
      MppiHorizonSafetyConfig{.swept_validation_step_m = 0.25,
                              .physical_footprint_radius_m = 0.25},
      false, {}, nullptr, &latest_raw);

  EXPECT_NE(result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_TRUE(result.global_raw_collision);
  EXPECT_GT(result.global_raw_validation_samples, 0U);
  EXPECT_EQ(result.global_raw_fallback_samples, 0U);
}

TEST(MppiHorizonSafetyTest, LatestRawBoundaryDoesNotBecomeAProhibitedZone) {
  const mppi::EsdfGrid local_grid{2, 2, 1.0F, 0.0F, 0.0F};
  const std::vector<float> local_esdf(4U, 10.0F);
  const GridBounds raw_bounds{0.0, 0.0, 1.0, 4, 4};
  OccupancyGrid2D latest_raw{raw_bounds};
  latest_raw.reset(CellState::kFree);
  const mppi::State current{1.0F, 1.5F, 5.0F};
  const std::vector<mppi::State> horizon{current, mppi::State{8.0F, 1.5F, 5.0F}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, local_esdf, local_grid,
      MppiHorizonSafetyConfig{.swept_validation_step_m = 0.25,
                              .physical_footprint_radius_m = 0.25},
      false, {}, nullptr, &latest_raw);

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_FALSE(result.global_raw_collision);
  EXPECT_GT(result.global_raw_validation_samples, 0U);
}

TEST(MppiHorizonSafetyTest, RejectsPhysicalFootprintContactWithLatestLidarHit) {
  const mppi::EsdfGrid local_grid{2, 2, 1.0F, 0.0F, 0.0F};
  const std::vector<float> local_esdf(4U, 10.0F);
  const mppi::State current{1.0F, 1.0F, 5.0F};
  const std::vector<mppi::State> horizon{current, mppi::State{5.0F, 1.0F, 5.0F}};
  const std::vector<Point3> latest_lidar_hits{Point3{3.0, 1.0, 5.0}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, local_esdf, local_grid,
      MppiHorizonSafetyConfig{.swept_validation_step_m = 0.25,
                              .physical_footprint_radius_m = 0.25,
                              .physical_footprint_lower_extent_m = 0.2,
                              .physical_footprint_upper_extent_m = 0.3},
      false, {}, nullptr, nullptr, latest_lidar_hits);

  EXPECT_NE(result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_TRUE(result.latest_lidar_collision);
  EXPECT_GT(result.latest_lidar_validation_samples, 0U);
  EXPECT_GT(result.latest_lidar_point_checks, 0U);
}

TEST(MppiHorizonSafetyTest, LatestLidarHitDoesNotCreateAnInfiniteVerticalColumn) {
  const mppi::EsdfGrid local_grid{2, 2, 1.0F, 0.0F, 0.0F};
  const std::vector<float> local_esdf(4U, 10.0F);
  const mppi::State current{1.0F, 1.0F, 5.0F};
  const std::vector<mppi::State> horizon{current, mppi::State{5.0F, 1.0F, 5.0F}};
  const std::vector<Point3> latest_lidar_hits{Point3{3.0, 1.0, 6.0}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, local_esdf, local_grid,
      MppiHorizonSafetyConfig{.swept_validation_step_m = 0.25,
                              .physical_footprint_radius_m = 0.25,
                              .physical_footprint_lower_extent_m = 0.2,
                              .physical_footprint_upper_extent_m = 0.3},
      false, {}, nullptr, nullptr, latest_lidar_hits);

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_FALSE(result.latest_lidar_collision);
  EXPECT_GT(result.latest_lidar_validation_samples, 0U);
}

TEST(MppiHorizonSafetyTest,
     BrakesForLatestLidarHitOnStoppingPathWhenNominalHorizonTurnsAway) {
  const mppi::EsdfGrid local_grid{20, 20, 1.0F, 0.0F, 0.0F};
  const std::vector<float> local_esdf(400U, 10.0F);
  mppi::State current{1.0F, 1.0F, 5.0F};
  current.vx = 8.0F;
  const std::vector<mppi::State> horizon{current, mppi::State{1.0F, 5.0F, 5.0F}};
  const std::vector<Point3> latest_lidar_hits{Point3{4.0, 1.0, 5.0}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, local_esdf, local_grid,
      MppiHorizonSafetyConfig{.reaction_latency_s = 0.1,
                              .maximum_braking_acceleration_mps2 = 4.0,
                              .minimum_time_to_collision_s = 0.1,
                              .fallback_duration_s = 3.0,
                              .dt_s = 0.05,
                              .swept_validation_step_m = 0.25,
                              .physical_footprint_radius_m = 0.25,
                              .physical_footprint_lower_extent_m = 0.2,
                              .physical_footprint_upper_extent_m = 0.3},
      false, {}, nullptr, nullptr, latest_lidar_hits);

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kBrake);
  EXPECT_TRUE(result.latest_lidar_collision);
  EXPECT_TRUE(result.latest_lidar_stopping_path_collision);
  EXPECT_TRUE(std::isfinite(result.latest_lidar_stopping_time_to_collision_s));
  EXPECT_GT(result.latest_lidar_stopping_validation_samples, 0U);
  EXPECT_GT(result.latest_lidar_stopping_point_checks, 0U);
}

TEST(MppiHorizonSafetyTest, AllowsLatestLidarHitOutsideStoppingFootprint) {
  const mppi::EsdfGrid local_grid{20, 20, 1.0F, 0.0F, 0.0F};
  const std::vector<float> local_esdf(400U, 10.0F);
  mppi::State current{1.0F, 1.0F, 5.0F};
  current.vx = 8.0F;
  const std::vector<mppi::State> horizon{current, mppi::State{5.0F, 1.0F, 5.0F}};
  const std::vector<Point3> latest_lidar_hits{Point3{4.0, 2.0, 5.0}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, local_esdf, local_grid,
      MppiHorizonSafetyConfig{.reaction_latency_s = 0.1,
                              .maximum_braking_acceleration_mps2 = 4.0,
                              .fallback_duration_s = 3.0,
                              .dt_s = 0.05,
                              .swept_validation_step_m = 0.25,
                              .physical_footprint_radius_m = 0.25,
                              .physical_footprint_lower_extent_m = 0.2,
                              .physical_footprint_upper_extent_m = 0.3},
      false, {}, nullptr, nullptr, latest_lidar_hits);

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_FALSE(result.latest_lidar_stopping_path_collision);
  EXPECT_GT(result.latest_lidar_stopping_validation_samples, 0U);
}

TEST(MppiHorizonSafetyTest, UsesAltitudeForThreeDimensionalCollision) {
  mppi::EsdfGrid grid{2, 2, 1.0F, 0.0F, 0.0F};
  grid.depth = 2;
  grid.origin_z_m = 0.0F;
  const std::vector<float> esdf{0.0F, 4.0F, 4.0F, 4.0F, 4.0F, 4.0F, 4.0F, 4.0F};
  const mppi::State above_obstacle{0.5F, 0.5F, 1.5F};
  const std::vector<mppi::State> horizon{above_obstacle, above_obstacle};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      above_obstacle, horizon, esdf, grid, MppiHorizonSafetyConfig{});

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kExecute);
}

TEST(MppiHorizonSafetyTest, ExplicitFallbackBrakesWithoutAPlannedHorizon) {
  mppi::State current{1.0F, 2.0F, 3.0F};
  current.vx = 10.0F;

  const MppiHorizonSafetyResult result =
      buildMppiBrakingFallback(current, MppiHorizonSafetyConfig{
                                            .maximum_braking_acceleration_mps2 = 4.0,
                                            .fallback_duration_s = 3.0,
                                            .dt_s = 0.05,
                                        });

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kBrake);
  ASSERT_FALSE(result.fallback_controls.empty());
  ASSERT_EQ(result.fallback_horizon.size(), result.fallback_controls.size() + 1U);
  EXPECT_FLOAT_EQ(result.fallback_horizon.front().vx, 10.0F);
  EXPECT_NEAR(result.fallback_horizon.back().vx, 0.0F, 1.0e-4F);
}

TEST(MppiHorizonSafetyTest, ExplicitFallbackHoldsAnAlreadyStationaryVehicle) {
  const MppiHorizonSafetyResult result =
      buildMppiBrakingFallback(mppi::State{}, MppiHorizonSafetyConfig{});

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kHold);
  ASSERT_FALSE(result.fallback_horizon.empty());
  EXPECT_FLOAT_EQ(result.fallback_horizon.back().vx, 0.0F);
  EXPECT_FLOAT_EQ(result.fallback_horizon.back().vy, 0.0F);
  EXPECT_FLOAT_EQ(result.fallback_horizon.back().vz, 0.0F);
}

TEST(MppiHorizonSafetyTest, SweptValidationFindsCollisionBetweenHorizonStates) {
  const mppi::EsdfGrid grid{5, 1, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf{2.0F, 1.0F, 0.0F, 1.0F, 2.0F};
  const mppi::State current{0.5F, 0.5F};
  const std::vector<mppi::State> horizon{current, mppi::State{4.5F, 0.5F}};

  const MppiHorizonSafetyResult result =
      evaluateMppiHorizonSafety(current, horizon, esdf, grid,
                                MppiHorizonSafetyConfig{
                                    .minimum_time_to_collision_s = 0.01,
                                    .dt_s = 1.0,
                                    .swept_validation_step_m = 0.25,
                                });

  EXPECT_NE(result.decision, MppiHorizonSafetyDecision::kExecute);
  EXPECT_GT(result.time_to_collision_s, 0.0);
  EXPECT_LT(result.time_to_collision_s, 1.0);
}

TEST(MppiHorizonSafetyTest, PointModelDoesNotTreatNearWallFreeCellAsCollision) {
  const mppi::EsdfGrid grid{2, 1, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf{1.0F, 0.0F};
  const mppi::State current{0.5F, 0.5F};
  const std::vector<mppi::State> horizon{current, mppi::State{0.99F, 0.5F}};

  const MppiHorizonSafetyResult result = evaluateMppiHorizonSafety(
      current, horizon, esdf, grid,
      MppiHorizonSafetyConfig{.physical_footprint_radius_m = 0.0});

  EXPECT_EQ(result.decision, MppiHorizonSafetyDecision::kExecute);
}

TEST(MppiHorizonSafetyTest, RejectsAndRepairsHorizonBelowFlightEnvelope) {
  const mppi::EsdfGrid grid{20, 20, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(400U, 10.0F);
  mppi::State current{2.0F, 2.0F, 1.5F};
  current.vz = -2.0F;
  const std::vector<mppi::State> horizon{current, mppi::State{2.0F, 2.0F, 0.5F}};
  const MppiHorizonSafetyConfig config{
      .dt_s = 0.2,
      .physical_footprint_radius_m = 0.0,
      .flight_envelope =
          FlightEnvelopeConfig{.minimum_target_z_m = 1.0, .maximum_target_z_m = 32.0}};

  const MppiHorizonSafetyResult result =
      evaluateMppiHorizonSafety(current, horizon, esdf, grid, config);

  EXPECT_TRUE(result.flight_envelope_violation);
  EXPECT_NE(result.decision, MppiHorizonSafetyDecision::kExecute);
  ASSERT_FALSE(result.fallback_horizon.empty());
  for (const mppi::State& state : result.fallback_horizon) {
    EXPECT_GE(state.z, 1.0F);
    EXPECT_LT(state.z, 32.0F);
  }
}

TEST(MppiHorizonSafetyTest, BrakingLifecycleLatchesPositionAfterVehicleSlows) {
  MppiBrakeHoldLifecycle lifecycle;
  const FlightEnvelopeConfig flight_envelope{};
  mppi::State moving{2.0F, 3.0F, 4.0F};
  moving.vx = 1.0F;
  EXPECT_FALSE(lifecycle.update(true, moving, 0.2, flight_envelope).position_hold);

  mppi::State stopped = moving;
  stopped.x = 2.5F;
  stopped.vx = 0.1F;
  const MppiBrakeHoldUpdate captured =
      lifecycle.update(true, stopped, 0.2, flight_envelope);
  ASSERT_TRUE(captured.position_hold);
  EXPECT_FLOAT_EQ(captured.hold_state.x, 2.5F);

  stopped.x = 2.7F;
  const MppiBrakeHoldUpdate retained =
      lifecycle.update(true, stopped, 0.2, flight_envelope);
  EXPECT_FLOAT_EQ(retained.hold_state.x, 2.5F);
  EXPECT_FALSE(lifecycle.update(false, stopped, 0.2, flight_envelope).position_hold);
}

TEST(MppiHorizonSafetyTest, BrakingLifecycleDoesNotRetainGroundHoldAfterTakeoff) {
  MppiBrakeHoldLifecycle lifecycle;
  const FlightEnvelopeConfig flight_envelope{};
  mppi::State ground{2.0F, 3.0F, 0.0F};

  EXPECT_FALSE(lifecycle.update(true, ground, 0.2, flight_envelope).position_hold);

  mppi::State climbing = ground;
  climbing.z = 2.0F;
  climbing.vz = 1.0F;
  EXPECT_FALSE(lifecycle.update(true, climbing, 0.2, flight_envelope).position_hold);

  mppi::State airborne = climbing;
  airborne.z = 18.0F;
  airborne.vz = 0.1F;
  const MppiBrakeHoldUpdate captured =
      lifecycle.update(true, airborne, 0.2, flight_envelope);
  ASSERT_TRUE(captured.position_hold);
  EXPECT_FLOAT_EQ(captured.hold_state.z, 18.0F);
}

} // namespace
} // namespace drone_city_nav
