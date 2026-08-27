#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "risk_aware_lattice_3d_cost.hpp"
#include "risk_aware_lattice_3d_geometry.hpp"

namespace drone_city_nav {
namespace {

TEST(Route3DTest, EqualCostSearchKeepsLevelAltitudeBeforeVerticalAlternatives) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 30, 10, 10}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{1.5, 4.5, 5.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{20.5, 4.5, 5.5}, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_FALSE(result.route.empty());
  for (const RouteSample3D& sample : result.route) {
    EXPECT_DOUBLE_EQ(sample.position.z, 5.5);
    EXPECT_GE(sample.position.z, config.flight_envelope.minimum_target_z_m);
    EXPECT_LT(sample.position.z, config.flight_envelope.maximum_target_z_m);
  }
}

TEST(Route3DTest, VerticalMotionAndPitchReversalCarryExplicitCost) {
  RiskAwareLattice3DConfig config;
  config.nominal_horizontal_speed_mps = 10.0;
  config.nominal_vertical_speed_mps = 2.0;
  config.vertical_alignment_cost_weight = 0.5;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.route_shape_vertical_turn_cost_per_rad = 1.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.planning_exposure_cost_per_m = 0.0;
  config.critical_exposure_cost_per_m = 0.0;
  const detail::Lattice3DEdgeEvaluation exposure{
      .status = detail::Lattice3DEdgeEvaluationStatus::kValid};

  const detail::Lattice3DCostMetrics climb = detail::evaluateLattice3DEdgeCost(
      Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 1.0}, Vec3{1.0, 0.0, 0.0},
      Vec3{1.0, 0.0, 0.0}, exposure, config);
  const detail::Lattice3DCostMetrics reversal = detail::evaluateLattice3DEdgeCost(
      Point3{1.0, 0.0, 1.0}, Point3{2.0, 0.0, 0.0},
      detail::lattice3DUnitDirection(Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 1.0}),
      Vec3{1.0, 0.0, 0.0}, exposure, config);

  EXPECT_DOUBLE_EQ(climb.vertical_alignment_time_s, 0.5);
  EXPECT_GT(climb.objective_cost, climb.travel_time_s);
  EXPECT_GT(climb.turn_cost, 0.0);
  EXPECT_GT(reversal.turn_cost, climb.turn_cost);
  EXPECT_NEAR(detail::lattice3DTravelHeuristic(Point3{0.0, 0.0, 0.0},
                                               Point3{1.0, 0.0, 1.0}, config),
              climb.travel_time_s + config.vertical_alignment_cost_weight *
                                        climb.vertical_alignment_time_s,
              1.0e-9);
}

TEST(Route3DTest, VerticalCostDoesNotForbidRequiredAltitudeChange) {
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 30, 10, 12}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;
  config.vertical_alignment_cost_weight = 0.5;
  config.route_shape_vertical_turn_cost_per_rad = 0.5;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{1.5, 4.5, 2.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{20.5, 4.5, 7.5}, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_FALSE(result.route.empty());
  EXPECT_GT(result.vertical_alignment_time_s, 0.0);
  EXPECT_NEAR(result.route.back().position.z, 7.5, 1.0e-6);
}

TEST(Route3DTest, EdgeFootprintContinuouslyCoversCoarseAndFineSweepIntervals) {
  constexpr int width{30};
  constexpr int height{5};
  constexpr int depth{30};
  const mppi::EsdfGrid grid{width, height, 0.1F, 0.0F, 0.0F, depth, 0.0F};
  std::vector<float> esdf(static_cast<std::size_t>(width * height * depth),
                          std::numeric_limits<float>::infinity());
  const std::size_t occupied_index =
      (std::size_t{22U} * static_cast<std::size_t>(height) + std::size_t{2U}) *
          static_cast<std::size_t>(width) +
      std::size_t{12U};
  esdf[occupied_index] = 0.0F;

  RiskAwareLattice3DConfig coarse;
  coarse.sample_step_m = 0.5;
  coarse.physical_footprint_radius_m = 0.0;
  coarse.physical_footprint_samples = 0U;
  coarse.physical_footprint_sweep_step_m = 0.5;
  coarse.preferred_distance_m = 0.0;
  coarse.critical_distance_m = 0.0;
  RiskAwareLattice3DConfig safety = coarse;
  safety.physical_footprint_sweep_step_m = 0.1;
  const Point3 first{0.05, 0.25, 2.25};
  const Point3 second{2.05, 0.25, 2.25};

  EXPECT_EQ(detail::evaluateLattice3DEdge(grid, esdf, first, second,
                                          Lattice3DRiskStage::kPreferredOnly, coarse)
                .status,
            detail::Lattice3DEdgeEvaluationStatus::kRawCollision);
  EXPECT_EQ(detail::evaluateLattice3DEdge(grid, esdf, first, second,
                                          Lattice3DRiskStage::kPreferredOnly, safety)
                .status,
            detail::Lattice3DEdgeEvaluationStatus::kRawCollision);
}

TEST(Route3DTest, EdgeSearchRejectsCollisionFromAuthoritativeRawSnapshot) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 12, 6, 6};
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  const std::vector<float> stale_esdf(
      static_cast<std::size_t>(grid.width * grid.height * grid.depth),
      std::numeric_limits<float>::infinity());
  ObservedOccupancyGrid3D latest_raw{bounds};
  ASSERT_TRUE(latest_raw.setState(GridIndex3D{5, 2, 2}, ObservedVoxelState::kOccupied));
  RiskAwareLattice3DConfig config;
  config.physical_footprint_radius_m = 0.25;
  config.physical_footprint_lower_extent_m = 0.25;
  config.physical_footprint_upper_extent_m = 0.25;
  config.critical_distance_m = 0.0;
  config.preferred_distance_m = 0.0;
  config.raw_validation.occupancy = &latest_raw;

  const detail::Lattice3DEdgeEvaluation result = detail::evaluateLattice3DEdge(
      grid, stale_esdf, Point3{1.5, 2.5, 2.5}, Point3{8.5, 2.5, 2.5},
      Lattice3DRiskStage::kCriticalAllowed, config);

  EXPECT_EQ(result.status, detail::Lattice3DEdgeEvaluationStatus::kRawCollision);
}

TEST(Route3DTest, InitialConnectorBuildsRouteReserveFromAPhysicallySafePose) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 12, 6, 6};
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  const std::vector<float> stale_esdf(
      static_cast<std::size_t>(grid.width * grid.height * grid.depth),
      std::numeric_limits<float>::infinity());
  ObservedOccupancyGrid3D latest_raw{bounds};
  ASSERT_TRUE(latest_raw.setState(GridIndex3D{2, 2, 2}, ObservedVoxelState::kOccupied));
  const Point3 start{3.5, 2.5, 2.5};
  const Point3 endpoint{6.5, 2.5, 2.5};
  RiskAwareLattice3DConfig config;
  config.physical_footprint_radius_m = 1.25;
  config.physical_footprint_lower_extent_m = 0.25;
  config.physical_footprint_upper_extent_m = 0.25;
  config.critical_distance_m = 0.0;
  config.preferred_distance_m = 0.0;
  config.raw_validation.occupancy = &latest_raw;

  EXPECT_EQ(detail::evaluateLattice3DEdge(grid, stale_esdf, start, endpoint,
                                          Lattice3DRiskStage::kCriticalAllowed, config)
                .status,
            detail::Lattice3DEdgeEvaluationStatus::kRawCollision);

  config.raw_validation.initial_connector_start = start;
  config.raw_validation.initial_connector_footprint = SweptFootprintConfig{
      .radius_m = 0.25,
      .lower_extent_m = 0.25,
      .upper_extent_m = 0.25,
      .perimeter_samples = config.physical_footprint_samples,
      .radial_rings = config.physical_footprint_radial_rings,
      .axial_samples = config.physical_footprint_axial_samples,
      .sweep_step_m = config.physical_footprint_sweep_step_m,
  };
  EXPECT_EQ(detail::evaluateLattice3DEdge(grid, stale_esdf, start, endpoint,
                                          Lattice3DRiskStage::kCriticalAllowed, config)
                .status,
            detail::Lattice3DEdgeEvaluationStatus::kValid);
}

TEST(Route3DTest, FrontierSelectionPreservesADeepLateralDetour) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 30, 30, 10}};
  for (int y = 0; y < occupancy.bounds().height_cells; ++y) {
    for (int z = 0; z < occupancy.bounds().depth_cells; ++z) {
      occupancy.setOccupied(GridIndex3D{6, y, z});
    }
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.clearance_tier_constraints_enabled = false;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.route_shape_vertical_turn_cost_per_rad = 0.0;
  config.vertical_alignment_cost_weight = 0.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_lower_extent_m = 0.0;
  config.physical_footprint_upper_extent_m = 0.0;
  config.physical_footprint_samples = 0U;
  config.maximum_expansions = 120U;
  config.maximum_search_time_ms = 1000.0;
  config.frontier_minimum_reachable_depth_m = 2.0;
  const Point3 start{3.5, 15.5, 5.5};
  const Point3 goal{24.5, 15.5, 5.5};

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kViableFrontier);
  ASSERT_FALSE(result.points.empty());
  EXPECT_GE(std::abs(result.points.back().y - start.y), 4.0);
  EXPECT_DOUBLE_EQ(result.points.back().z, start.z);
  EXPECT_GT(result.frontier_net_coordinate_progress_m, 2.0);
}

TEST(Route3DTest, EdgeRiskStageUsesPhysicalFootprintClearance) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 16, 12, 10}};
  for (int y = 0; y < 12; ++y) {
    for (int z = 0; z < 10; ++z) {
      occupancy.setOccupied(GridIndex3D{6, y, z});
    }
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 20.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.critical_distance_m = 1.25;
  config.preferred_distance_m = 4.0;
  const Point3 first{3.5, 4.5, 4.5};
  const Point3 second{3.5, 6.5, 4.5};

  const EsdfQueryResult center =
      queryConservativeEsdf3D(grid, field.distancesM(), static_cast<float>(first.x),
                              static_cast<float>(first.y), static_cast<float>(first.z));
  ASSERT_EQ(center.status, EsdfQueryStatus::kValid);
  ASSERT_GE(center.clearance_m, config.critical_distance_m);

  const detail::Lattice3DEdgeEvaluation planning =
      detail::evaluateLattice3DEdge(grid, field.distancesM(), first, second,
                                    Lattice3DRiskStage::kPlanningAllowed, config);
  const detail::Lattice3DEdgeEvaluation critical =
      detail::evaluateLattice3DEdge(grid, field.distancesM(), first, second,
                                    Lattice3DRiskStage::kCriticalAllowed, config);

  EXPECT_EQ(planning.status, detail::Lattice3DEdgeEvaluationStatus::kRiskStageRejected);
  ASSERT_EQ(critical.status, detail::Lattice3DEdgeEvaluationStatus::kValid);
  EXPECT_LT(critical.minimum_clearance_m, config.critical_distance_m);
  EXPECT_GT(critical.critical_exposure_m, 0.0);
}

} // namespace
} // namespace drone_city_nav
