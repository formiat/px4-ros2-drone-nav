#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"
#include "drone_city_nav/route_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "risk_aware_lattice_3d_continuation.hpp"
#include "risk_aware_lattice_3d_geometry.hpp"

namespace drone_city_nav {
namespace {

TEST(Route3DTest, LatticeTraversesLShapedPassage) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 20, 4}};
  for (int y = 0; y < 12; ++y) {
    for (int z = 0; z < 4; ++z) {
      occupancy.setOccupied(GridIndex3D{8, y, z});
      occupancy.setOccupied(GridIndex3D{9, y, z});
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
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.maximum_search_time_ms = 1000.0;
  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{3.5, 3.5, 1.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{15.5, 15.5, 1.5}, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_FALSE(result.points.empty());
  EXPECT_TRUE(std::any_of(result.points.begin(), result.points.end(),
                          [](const Point3& p) { return p.y > 11.0 && p.x < 10.0; }));
  EXPECT_NEAR(result.points.back().x, 15.5, 1.0e-6);
  EXPECT_NEAR(result.points.back().y, 15.5, 1.0e-6);
}

TEST(Route3DTest, FullPlanningRouteBeatsPreferredFrontier) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 13, 4}};
  for (int y = 0; y < 13; ++y) {
    if (y >= 5 && y <= 7) {
      continue;
    }
    for (int z = 0; z < 4; ++z) {
      occupancy.setOccupied(GridIndex3D{10, y, z});
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
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 2.0;
  config.critical_distance_m = 0.0;
  config.maximum_search_time_ms = 1000.0;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{2.5, 6.5, 1.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{17.5, 6.5, 1.5}, {}, config);

  EXPECT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.risk_stage, Lattice3DRiskStage::kPlanningAllowed);
  EXPECT_TRUE(result.reached_mission_goal);
}

TEST(Route3DTest, FartherPlanningFrontierBeatsBlockedPreferredFrontier) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 13, 4}};
  for (int y = 0; y < 13; ++y) {
    if (y >= 5 && y <= 7) {
      continue;
    }
    for (int z = 0; z < 4; ++z) {
      occupancy.setOccupied(GridIndex3D{10, y, z});
    }
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 40.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 2.0;
  config.critical_distance_m = 0.0;
  config.maximum_search_time_ms = 1000.0;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{2.5, 6.5, 1.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{30.5, 6.5, 1.5}, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kViableFrontier);
  EXPECT_EQ(result.risk_stage, Lattice3DRiskStage::kPlanningAllowed);
  ASSERT_FALSE(result.points.empty());
  EXPECT_GT(result.points.back().x, 10.0);
  EXPECT_GT(result.successor_profiling.continuation.collection_calls, 0U);
  EXPECT_GT(result.successor_profiling.continuation.candidates, 0U);
  EXPECT_GT(result.successor_profiling.continuation.maximum_candidates, 0U);
  EXPECT_GE(result.successor_profiling.continuation.worker_ms, 0.0);
}

TEST(Route3DTest, BuildsTypedSpanFromSelectedPassageTraversal) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {5.0, 0.0, 5.0}, {10.0, 0.0, 5.0}}, 0.5,
      20.0);
  const std::vector<SelectedPassageTraversal> traversals{
      SelectedPassageTraversal{.passage_traversal_id = "test_passage",
                               .direction_sign = -1,
                               .begin_station_m = 2.0,
                               .end_station_m = 8.0,
                               .min_z_m = 1.5,
                               .max_z_m = 8.5,
                               .width_m = 24.0,
                               .height_m = 7.0,
                               .minimum_clearance_m = 3.5,
                               .speed_limit_mps = 10.0,
                               .segment_spans = {}}};

  const std::vector<ConstrainedRouteSpan> spans =
      makeConstrainedRouteSpans(route, traversals, 12U, RouteEnvelopeConfig{});

  ASSERT_EQ(spans.size(), 1U);
  EXPECT_EQ(spans.front().passage_traversal_id, "test_passage");
  EXPECT_EQ(spans.front().route_generation, 12U);
  EXPECT_EQ(spans.front().direction_sign, -1);
  ASSERT_FALSE(spans.front().envelope.empty());
  EXPECT_DOUBLE_EQ(spans.front().envelope.front().min_z_m, 1.5);
  EXPECT_DOUBLE_EQ(spans.front().envelope.front().max_z_m, 8.5);
  EXPECT_DOUBLE_EQ(spans.front().envelope.front().reference_speed_mps, 10.0);
}

TEST(Route3DTest, ComparesReachedRoutesAcrossAllRiskStages) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 20, 4}};
  for (int y = 2; y <= 16; ++y) {
    if (y >= 8 && y <= 10) {
      continue;
    }
    for (int z = 0; z < 4; ++z) {
      occupancy.setOccupied(GridIndex3D{11, y, z});
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
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 2.0;
  config.critical_distance_m = 0.1;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_lower_extent_m = 0.0;
  config.physical_footprint_upper_extent_m = 0.0;
  config.physical_footprint_samples = 0U;
  config.maximum_search_time_ms = 3000.0;
  config.maximum_expansions = 500000U;
  config.heading_bias_cost_per_rad = 0.0;

  const RiskAwareLattice3DResult result =
      planRiskAwareLattice3D(grid, field.distancesM(), Point3{2.5, 9.5, 1.5},
                             Vec3{1.0, 0.0, 0.0}, Point3{21.5, 9.5, 1.5}, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal)
      << "termination=" << static_cast<int>(result.termination)
      << " expansions=" << result.expansions << " records_peak=" << result.records_peak
      << " topology_candidates=" << result.topology_candidates.size();
  EXPECT_EQ(result.topology_candidates.size(), 3U);
  EXPECT_TRUE(
      std::ranges::any_of(result.topology_candidates,
                          [](const auto& candidate) { return candidate.selected; }));
  for (std::size_t rank = 0U; rank < result.topology_candidates.size(); ++rank) {
    EXPECT_EQ(result.topology_candidates[rank].candidate_rank, rank);
  }
  EXPECT_NE(result.route_fingerprint, 0U);
  EXPECT_GE(result.search_ms, 0.0);
  EXPECT_NE(result.risk_stage, Lattice3DRiskStage::kPreferredOnly);
}

TEST(Route3DTest, SelectsEmbeddedPassageEdgeWhenItsObjectiveCostIsLower) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 20, 10}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  const std::vector<PassageTraversalEdge> passages{PassageTraversalEdge{
      .id = "direct_passage",
      .region_id = "direct_region",
      .entry_portal_id = "direct_entry",
      .exit_portal_id = "direct_exit",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{0.5, 0.5, 5.5}, {12.5, 8.5, 5.5}}, 0.5, 10.0),
      .entry = Point3{0.5, 0.5, 5.5},
      .exit = Point3{12.5, 8.5, 5.5},
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .width_m = 24.0,
      .height_m = 7.0,
      .minimum_clearance_m = 3.5,
      .speed_limit_mps = 10.0,
      .segment_spans = {}}};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 4.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.passage_connection_distance_m = 1.0;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), Point3{0.5, 0.5, 5.5}, Vec3{1.0, 0.0, 0.0},
      Point3{12.5, 8.5, 5.5}, passages, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_EQ(result.selected_passage_traversals.size(), 1U);
  EXPECT_EQ(result.selected_passage_traversals.front().passage_traversal_id,
            "direct_passage");
  EXPECT_GT(result.selected_passage_traversals.front().end_station_m,
            result.selected_passage_traversals.front().begin_station_m);
}

TEST(Route3DTest, SeedsCollisionValidatedPassageBeyondLocalConnectionRadius) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 8, 10}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  const std::vector<PassageTraversalEdge> passages{PassageTraversalEdge{
      .id = "far_passage",
      .region_id = "far_region",
      .entry_portal_id = "far_entry",
      .exit_portal_id = "far_exit",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{8.5, 2.5, 5.5}, {18.5, 2.5, 5.5}}, 0.5, 10.0),
      .entry = Point3{8.5, 2.5, 5.5},
      .exit = Point3{18.5, 2.5, 5.5},
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .width_m = 24.0,
      .height_m = 7.0,
      .minimum_clearance_m = 3.5,
      .speed_limit_mps = 10.0,
      .segment_spans = {}}};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 4.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.passage_connection_distance_m = 1.0;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), Point3{0.5, 2.5, 8.5}, Vec3{1.0, 0.0, 0.0},
      Point3{22.5, 2.5, 5.5}, passages, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_EQ(result.selected_passage_traversals.size(), 1U);
  EXPECT_EQ(result.selected_passage_traversals.front().passage_traversal_id,
            "far_passage");
}

TEST(Route3DTest, MaterializesRequiredPassageWhenUnconstrainedSlicePrefersFrontier) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 80, 12, 10}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 100.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  const std::vector<PassageTraversalEdge> passages{PassageTraversalEdge{
      .id = "required_passage",
      .region_id = "required_region",
      .entry_portal_id = "required_entry",
      .exit_portal_id = "required_exit",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{30.5, 5.5, 5.5}, {60.5, 5.5, 5.5}}, 0.5, 10.0),
      .entry = Point3{30.5, 5.5, 5.5},
      .exit = Point3{60.5, 5.5, 5.5},
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .width_m = 24.0,
      .height_m = 7.0,
      .minimum_clearance_m = 3.5,
      .speed_limit_mps = 10.0,
      .segment_spans = {}}};
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 100.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.heading_bias_cost_per_rad = 0.0;
  config.route_shape_turn_cost_per_rad = 0.0;
  config.passage_topology_transition_cost = 100.0;
  config.maximum_topology_search_groups = 1U;
  config.maximum_expansions = 8U;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), Point3{2.5, 5.5, 5.5}, Vec3{1.0, 0.0, 0.0},
      Point3{74.5, 5.5, 5.5}, passages, config);

  EXPECT_EQ(result.topology_searches, 2U);
  ASSERT_EQ(result.selected_passage_traversals.size(), 1U);
  EXPECT_EQ(result.selected_passage_traversals.front().passage_traversal_id,
            "required_passage");
  EXPECT_GT(result.achieved_progress_m, 50.0);
}

TEST(Route3DTest, ParallelTopologyGroupsPreserveBestCompleteRoute) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 20, 10}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  const auto passage = [](const std::string& id, const std::vector<Point3>& points) {
    return PassageTraversalEdge{.id = id,
                                .region_id = id + "_region",
                                .entry_portal_id = id + "_entry",
                                .exit_portal_id = id + "_exit",
                                .centerline = sampleRoute3D(points, 0.5, 10.0),
                                .entry = points.front(),
                                .exit = points.back(),
                                .min_z_m = 1.5,
                                .max_z_m = 8.5,
                                .width_m = 24.0,
                                .height_m = 7.0,
                                .minimum_clearance_m = 3.5,
                                .speed_limit_mps = 10.0,
                                .segment_spans = {}};
  };
  const std::vector<PassageTraversalEdge> passages{
      passage("direct", {{0.5, 2.5, 5.5}, {18.5, 2.5, 5.5}}),
      passage("detour", {{0.5, 2.5, 5.5}, {9.5, 15.5, 5.5}, {18.5, 2.5, 5.5}})};
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
  const Point3 start{0.5, 2.5, 5.5};
  const Point3 goal{20.5, 2.5, 5.5};

  const RiskAwareLattice3DResult serial = planRiskAwareLattice3D(
      grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal, passages, config);
  BoundedWorkerPool two_worker_pool{2U};
  const RiskAwareLattice3DResult two_worker =
      planRiskAwareLattice3D(grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal,
                             passages, config, &two_worker_pool);
  BoundedWorkerPool worker_pool{4U};
  const RiskAwareLattice3DResult parallel =
      planRiskAwareLattice3D(grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal,
                             passages, config, &worker_pool);

  ASSERT_EQ(serial.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(serial.topology_searches, 3U);
  EXPECT_EQ(serial.parallel_topology_searches, 0U);
  ASSERT_EQ(two_worker.status, serial.status);
  EXPECT_EQ(two_worker.route_fingerprint, serial.route_fingerprint);
  EXPECT_EQ(two_worker.topology_searches, 3U);
  EXPECT_EQ(two_worker.parallel_topology_searches, two_worker.topology_searches);
  ASSERT_EQ(parallel.status, serial.status);
  EXPECT_EQ(parallel.route_fingerprint, serial.route_fingerprint);
  EXPECT_EQ(parallel.topology_searches, 3U);
  EXPECT_EQ(parallel.parallel_topology_searches, parallel.topology_searches);
  EXPECT_GT(parallel.topology_search_worker_ms, 0.0);
}

TEST(Route3DTest, SpatiallyIndexesPassageEntriesDuringLatticeExpansion) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 20, 10}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 30.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  std::vector<PassageTraversalEdge> passages;
  for (std::size_t index = 0U; index < 40U; ++index) {
    const double y = 100.5 + 4.0 * static_cast<double>(index);
    const std::string id = "remote_" + std::to_string(index);
    passages.push_back(PassageTraversalEdge{
        .id = id,
        .region_id = id + "_region",
        .entry_portal_id = id + "_entry",
        .exit_portal_id = id + "_exit",
        .centerline = sampleRoute3D(std::vector<Point3>{{4.5, y, 5.5}, {12.5, y, 5.5}},
                                    0.5, 10.0),
        .entry = Point3{4.5, y, 5.5},
        .exit = Point3{12.5, y, 5.5},
        .min_z_m = 1.5,
        .max_z_m = 8.5,
        .width_m = 8.0,
        .height_m = 7.0,
        .minimum_clearance_m = 3.5,
        .speed_limit_mps = 10.0,
        .segment_spans = {},
    });
  }
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 2.0;
  config.vertical_step_m = 1.0;
  config.planning_goal_distance_m = 30.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.maximum_topology_search_groups = 0U;
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), Point3{2.5, 2.5, 5.5}, Vec3{1.0, 0.0, 0.0},
      Point3{20.5, 2.5, 5.5}, passages, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.successor_profiling.search.maximum_candidates, passages.size() * 2U);
  EXPECT_TRUE(result.selected_passage_traversals.empty());
}

TEST(Route3DTest, MaterializesValidatedContinuationWhenSearchSliceExpires) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 20, 12}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 40.0);
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
  config.maximum_search_time_ms = 0.001;
  config.frontier_minimum_reachable_depth_m = 8.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;
  const Point3 start{2.5, 10.5, 5.5};
  const Point3 goal{30.5, 10.5, 5.5};

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kViableFrontier);
  EXPECT_EQ(result.termination, Lattice3DSearchTermination::kDeadlineReached);
  EXPECT_GE(result.continuation_reachable_depth_m,
            config.frontier_minimum_reachable_depth_m);
  EXPECT_GE(result.route_length_m, config.frontier_minimum_reachable_depth_m);
  ASSERT_GE(result.points.size(), 2U);
  EXPECT_NEAR(result.achieved_progress_m,
              distance3D(start, goal) - distance3D(result.points.back(), goal), 1.0e-9);
  EXPECT_LT(distance3D(result.points.back(), goal), distance3D(start, goal));
  EXPECT_NEAR(result.points.back().y, start.y, 1.0e-9);
  EXPECT_NEAR(result.points.back().z, start.z, 1.0e-9);
}

TEST(Route3DTest, ContinuationSelectsGoalProgressThatExtendsTheRouteFrontier) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 30, 30, 12}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 40.0);
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
  config.frontier_minimum_reachable_depth_m = 8.0;
  config.frontier_validation_maximum_states = 256U;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;
  const Point3 route_origin{2.5, 15.5, 5.5};
  const Point3 terminal{10.5, 15.5, 5.5};

  const detail::Lattice3DContinuationMetrics continuation =
      detail::evaluateLattice3DContinuation(
          grid, field.distancesM(), route_origin, terminal, Vec3{1.0, 0.0, 0.0},
          Point3{25.5, 15.5, 5.5}, Lattice3DRiskStage::kPreferredOnly, config, nullptr);

  ASSERT_GE(continuation.path.size(), 2U);
  EXPECT_GT(distance3D(route_origin, continuation.path.back()),
            distance3D(route_origin, terminal));
  EXPECT_LT(distance3D(continuation.path.back(), Point3{25.5, 15.5, 5.5}),
            distance3D(terminal, Point3{25.5, 15.5, 5.5}));
}

TEST(Route3DTest, ContinuationRejectsReachableSpaceThatOnlyLoopsBack) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 5, 3}};
  for (int x = 0; x < 32; ++x) {
    for (int y = 0; y < 5; ++y) {
      for (int z = 0; z < 3; ++z) {
        occupancy.setOccupied(GridIndex3D{x, y, z});
      }
    }
  }
  for (int x = 2; x <= 28; ++x) {
    occupancy.clearOccupied(GridIndex3D{x, 2, 1});
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 40.0);
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
  config.frontier_minimum_reachable_depth_m = 8.0;
  config.frontier_validation_maximum_states = 256U;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;
  const Point3 route_origin{2.5, 2.5, 1.5};
  const Point3 terminal{28.5, 2.5, 1.5};

  const detail::Lattice3DContinuationMetrics continuation =
      detail::evaluateLattice3DContinuation(
          grid, field.distancesM(), route_origin, terminal, Vec3{0.0, 1.0, 0.0},
          Point3{40.5, 2.5, 1.5}, Lattice3DRiskStage::kCriticalAllowed, config,
          nullptr);

  EXPECT_GT(continuation.immediate_successors, 0U);
  EXPECT_GE(continuation.reachable_depth_m, config.frontier_minimum_reachable_depth_m);
  EXPECT_TRUE(continuation.path.empty());
}

TEST(Route3DTest, ContinuationValidationHonorsItsTimeBudget) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 30, 30, 12}};
  const DistanceField3D field = DistanceField3D::build(occupancy, 40.0);
  const GridBounds3D& bounds = field.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  RiskAwareLattice3DConfig config;
  config.frontier_validation_maximum_states = 100000U;
  config.frontier_validation_maximum_time_ms = 1.0e-9;
  config.frontier_minimum_reachable_depth_m = 100.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;

  const detail::Lattice3DContinuationMetrics continuation =
      detail::evaluateLattice3DContinuation(
          grid, field.distancesM(), Point3{2.5, 15.5, 5.5}, Point3{10.5, 15.5, 5.5},
          Vec3{1.0, 0.0, 0.0}, Point3{25.5, 15.5, 5.5},
          Lattice3DRiskStage::kPreferredOnly, config, nullptr);

  EXPECT_TRUE(continuation.time_budget_exhausted);
  EXPECT_TRUE(continuation.path.empty());
}

TEST(Route3DTest, MaterializedContinuationCommitsToGoalDirectedWallDetour) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 30, 12}};
  for (int y = 11; y <= 19; ++y) {
    for (int z = 0; z < 8; ++z) {
      occupancy.setOccupied(GridIndex3D{8, y, z});
    }
  }
  const DistanceField3D field = DistanceField3D::build(occupancy, 40.0);
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
  config.maximum_search_time_ms = 0.001;
  config.frontier_minimum_reachable_depth_m = 8.0;
  config.physical_footprint_radius_m = 0.0;
  config.physical_footprint_samples = 0U;
  const Point3 start{2.5, 15.5, 5.5};
  const Point3 goal{30.5, 15.5, 5.5};

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      grid, field.distancesM(), start, Vec3{1.0, 0.0, 0.0}, goal, {}, config);

  ASSERT_EQ(result.status, Lattice3DStatus::kViableFrontier);
  EXPECT_EQ(result.termination, Lattice3DSearchTermination::kDeadlineReached);
  ASSERT_GE(result.points.size(), 2U);
  EXPECT_NEAR(result.achieved_progress_m,
              distance3D(start, goal) - distance3D(result.points.back(), goal), 1.0e-9);
  EXPECT_LT(distance3D(result.points.back(), goal), distance3D(start, goal));
  EXPECT_GT(std::abs(result.points.back().y - start.y), 1.0);
  EXPECT_NEAR(result.points.back().z, start.z, 1.0e-9);
}

TEST(Route3DTest, ClipsSpanWithInterpolatedBoundaryEnvelopeSamples) {
  ConstrainedRouteSpan span{
      .passage_traversal_id = PassageTraversalId{"passage"},
      .route_generation = 7U,
      .direction_sign = 1,
      .begin_station_m = 1.0,
      .end_station_m = 5.0,
      .envelope =
          {
              RouteEnvelopeSample{
                  .station_m = 1.0, .reference_z_m = 2.0, .reference_speed_mps = 3.0},
              RouteEnvelopeSample{
                  .station_m = 5.0, .reference_z_m = 6.0, .reference_speed_mps = 7.0},
          },
      .segment_spans = {},
  };

  const std::vector<ConstrainedRouteSpan> clipped =
      clipConstrainedRouteSpans({&span, 1U}, 2.0, 4.0);

  ASSERT_EQ(clipped.size(), 1U);
  ASSERT_EQ(clipped.front().envelope.size(), 2U);
  EXPECT_DOUBLE_EQ(clipped.front().begin_station_m, 2.0);
  EXPECT_DOUBLE_EQ(clipped.front().end_station_m, 4.0);
  EXPECT_DOUBLE_EQ(clipped.front().envelope.front().station_m, 2.0);
  EXPECT_DOUBLE_EQ(clipped.front().envelope.front().reference_z_m, 3.0);
  EXPECT_DOUBLE_EQ(clipped.front().envelope.back().station_m, 4.0);
  EXPECT_DOUBLE_EQ(clipped.front().envelope.back().reference_speed_mps, 6.0);
}

} // namespace
} // namespace drone_city_nav
