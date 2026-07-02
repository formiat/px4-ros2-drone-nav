#include "drone_city_nav/grid_config.hpp"
#include "drone_city_nav/planner_node_config.hpp"

#include <rclcpp/parameter.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <numbers>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

class PlannerNodeConfigTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  [[nodiscard]] static std::shared_ptr<rclcpp::Node>
  makeNode(const std::string& name,
           const std::vector<rclcpp::Parameter>& parameters = {}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    return std::make_shared<rclcpp::Node>(name, options);
  }
};

} // namespace

TEST_F(PlannerNodeConfigTest, UsesDocumentedDefaults) {
  const auto node = makeNode("planner_node_config_defaults");

  const PlannerNodeConfig config = loadPlannerNodeConfig(*node);

  EXPECT_EQ(config.frame_id, "map");
  EXPECT_DOUBLE_EQ(config.start.x, 0.0);
  EXPECT_DOUBLE_EQ(config.start.y, 0.0);
  EXPECT_DOUBLE_EQ(config.goal.x, 85.0);
  EXPECT_DOUBLE_EQ(config.goal.y, 0.0);
  EXPECT_DOUBLE_EQ(config.cruise_altitude_m, 12.0);
  EXPECT_DOUBLE_EQ(config.inflation_radius_m, 1.0);
  EXPECT_DOUBLE_EQ(config.planning_clearance_m, 3.0);
  EXPECT_DOUBLE_EQ(config.planning_grid_builder.inflation_radius_m, 1.0);
  EXPECT_DOUBLE_EQ(config.planning_grid_builder.planning_clearance_m, 3.0);
  EXPECT_TRUE(config.static_map.enabled);
  EXPECT_TRUE(config.planning_grid_builder.use_static_map);
  EXPECT_EQ(config.static_map.configured_path.string(), "worlds/generated_city.map2d");
  EXPECT_EQ(config.topics.prohibited_grid, "/drone_city_nav/prohibited_grid");
  EXPECT_EQ(config.topics.path, "/drone_city_nav/path");
  EXPECT_EQ(config.topics.trajectory_diagnostics,
            "/drone_city_nav/trajectory_diagnostics");
  EXPECT_DOUBLE_EQ(config.timing.path_prohibited_intersection_check_period_s, 0.5);
  EXPECT_DOUBLE_EQ(config.planner_core.astar.heuristic_weight, 1.0);
  EXPECT_FALSE(config.planner_core.astar.evasive_maneuvering_enabled);
  EXPECT_TRUE(config.planner_core.astar.initial_heading_bias_enabled);
  EXPECT_DOUBLE_EQ(config.planner_core.astar.initial_heading_bias_min_speed_mps, 0.5);
  EXPECT_DOUBLE_EQ(config.planner_core.astar.initial_heading_bias_weight, 50.0);
  EXPECT_TRUE(config.current_lidar.use_px4_heading_for_scan);
  EXPECT_TRUE(config.current_lidar.motion_compensate_lidar_pose);
  EXPECT_DOUBLE_EQ(config.current_lidar.lidar_pose_latency_s, 0.05);
  EXPECT_TRUE(config.lidar_projection.compensate_attitude);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.speed_profile.cruise_speed_mps, 12.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.corridor.max_radius_m, 40.0);
  EXPECT_EQ(config.trajectory_planner.corridor.parallel_workers, 0U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.weight_curvature, 300.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.weight_curvature_change,
                   130.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.weight_offset_second_change,
                   6.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.weight_offset_slope, 100.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.max_offset_slope_per_m, 0.32);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.weight_time, 40.0);
  EXPECT_EQ(config.trajectory_planner.racing_line.parallel_workers, 0U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_pre_margin_m, 25.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_post_margin_m, 25.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_heading_threshold_rad,
                   10.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.racing_line.window_width_change_threshold_m, 2.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_min_heading_span_rad,
                   10.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_min_curvature_1pm,
                   0.01);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_min_width_asymmetry_m,
                   1.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_offset_step_m, 1.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_coarse_offset_step_m, 2.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_fine_offset_step_m, 0.75);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_fine_radius_m, 1.5);
  EXPECT_EQ(config.trajectory_planner.racing_line.top_n_full_score_candidates, 0U);
  EXPECT_EQ(config.trajectory_planner.racing_line.async_refinement_workers, 1U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.trigger_heading_delta_rad,
                   37.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.entry_distance_m, 45.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.exit_distance_m, 45.0);
}

TEST_F(PlannerNodeConfigTest, ClampsUnsafeValues) {
  const auto node = makeNode(
      "planner_node_config_clamps",
      {rclcpp::Parameter{"max_pose_staleness_s", -5.0},
       rclcpp::Parameter{"max_current_lidar_staleness_s", 9999.0},
       rclcpp::Parameter{"memory_occupied_value", 500},
       rclcpp::Parameter{"memory_free_value", -20},
       rclcpp::Parameter{"static_map_min_blocking_height_m", -1.0},
       rclcpp::Parameter{"planning_grid_resolution_m", -0.5},
       rclcpp::Parameter{"planning_grid_width_m", -10.0},
       rclcpp::Parameter{"planning_grid_height_m", 0.0},
       rclcpp::Parameter{"astar_heuristic_weight", 5000.0},
       rclcpp::Parameter{"astar_turn_cost_weight", 5000.0},
       rclcpp::Parameter{"astar_evasive_maneuvering_straight_cost_weight", 5000.0},
       rclcpp::Parameter{"astar_initial_heading_bias_min_speed_mps", -2.0},
       rclcpp::Parameter{"astar_initial_heading_bias_weight", 5000.0},
       rclcpp::Parameter{"lidar_pose_latency_s", 5.0},
       rclcpp::Parameter{"cruise_speed_mps", 5000.0},
       rclcpp::Parameter{"min_turn_speed_mps", 5000.0},
       rclcpp::Parameter{"corridor_max_radius_m", -10.0},
       rclcpp::Parameter{"corridor_parallel_workers", 5000},
       rclcpp::Parameter{"racing_line_weight_time", -2.0},
       rclcpp::Parameter{"racing_line_parallel_workers", 5000},
       rclcpp::Parameter{"racing_line_window_pre_margin_m", -1.0},
       rclcpp::Parameter{"racing_line_window_post_margin_m", 9999.0},
       rclcpp::Parameter{"racing_line_window_heading_threshold_deg", 500.0},
       rclcpp::Parameter{"racing_line_window_width_change_threshold_m", -1.0},
       rclcpp::Parameter{"racing_line_window_min_heading_span_deg", 500.0},
       rclcpp::Parameter{"racing_line_window_min_curvature_1pm", -1.0},
       rclcpp::Parameter{"racing_line_window_min_width_asymmetry_m", -1.0},
       rclcpp::Parameter{"racing_line_dp_offset_step_m", -1.0},
       rclcpp::Parameter{"racing_line_dp_coarse_offset_step_m", -1.0},
       rclcpp::Parameter{"racing_line_dp_fine_offset_step_m", -1.0},
       rclcpp::Parameter{"racing_line_dp_fine_radius_m", -1.0},
       rclcpp::Parameter{"racing_line_top_n_full_score_candidates", 500000},
       rclcpp::Parameter{"racing_line_async_refinement_workers", 5000},
       rclcpp::Parameter{"turn_smoothing_trigger_heading_delta_deg", 500.0},
       rclcpp::Parameter{"turn_smoothing_entry_distance_m", -5.0},
       rclcpp::Parameter{"turn_smoothing_max_length_ratio", -2.0},
       rclcpp::Parameter{"static_map_debug_publish_period_s", 100.0}});

  const PlannerNodeConfig config = loadPlannerNodeConfig(*node);

  EXPECT_EQ(config.timing.max_pose_staleness_ns, 0);
  EXPECT_EQ(config.timing.max_current_lidar_staleness_ns, 3'600'000'000'000LL);
  EXPECT_EQ(config.memory_grid.occupied_value, 100);
  EXPECT_EQ(config.memory_grid.free_value, 0);
  EXPECT_DOUBLE_EQ(config.static_map.min_blocking_height_m, 0.0);
  EXPECT_DOUBLE_EQ(config.planning_grid_builder.fallback_bounds.resolution_m, 0.01);
  EXPECT_EQ(config.planning_grid_builder.fallback_bounds.width_cells, 1);
  EXPECT_EQ(config.planning_grid_builder.fallback_bounds.height_cells, 1);
  EXPECT_DOUBLE_EQ(config.planner_core.astar.heuristic_weight, 10.0);
  EXPECT_DOUBLE_EQ(config.planner_core.astar.turn_cost_weight, 1000.0);
  EXPECT_DOUBLE_EQ(config.planner_core.astar.evasive_maneuvering_straight_cost_weight,
                   1000.0);
  EXPECT_DOUBLE_EQ(config.planner_core.astar.initial_heading_bias_min_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(config.planner_core.astar.initial_heading_bias_weight, 1000.0);
  EXPECT_DOUBLE_EQ(config.current_lidar.lidar_pose_latency_s, 1.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.speed_profile.cruise_speed_mps, 100.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.speed_profile.min_turn_speed_mps, 100.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.corridor.max_radius_m, 1.0);
  EXPECT_EQ(config.trajectory_planner.corridor.parallel_workers, 1024U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.weight_time, 0.0);
  EXPECT_EQ(config.trajectory_planner.racing_line.parallel_workers, 1024U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_pre_margin_m, 0.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_post_margin_m, 5000.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_heading_threshold_rad,
                   std::numbers::pi);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.racing_line.window_width_change_threshold_m, 0.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_min_heading_span_rad,
                   std::numbers::pi);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_min_curvature_1pm, 0.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_min_width_asymmetry_m,
                   0.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_offset_step_m, 0.05);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_coarse_offset_step_m, 0.05);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_fine_offset_step_m, 0.05);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_fine_radius_m, 0.05);
  EXPECT_EQ(config.trajectory_planner.racing_line.top_n_full_score_candidates, 100000U);
  EXPECT_EQ(config.trajectory_planner.racing_line.async_refinement_workers, 1U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.trigger_heading_delta_rad,
                   std::numbers::pi);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.entry_distance_m, 0.1);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.max_length_ratio, 1.0);
  EXPECT_DOUBLE_EQ(config.timing.static_map_debug_publish_period_s, 60.0);
}

TEST_F(PlannerNodeConfigTest, BuildsNestedCoreConfigs) {
  const auto node =
      makeNode("planner_node_config_nested",
               {rclcpp::Parameter{"astar_heuristic_weight", 1.2},
                rclcpp::Parameter{"astar_turn_cost_weight", 2.0},
                rclcpp::Parameter{"astar_evasive_maneuvering_enabled", true},
                rclcpp::Parameter{"astar_initial_heading_bias_enabled", true},
                rclcpp::Parameter{"astar_initial_heading_bias_min_speed_mps", 1.25},
                rclcpp::Parameter{"astar_initial_heading_bias_weight", 75.0},
                rclcpp::Parameter{"use_static_map", false},
                rclcpp::Parameter{"path_prohibited_intersection_check_period_s", 0.25},
                rclcpp::Parameter{"racing_line_weight_curvature", 125.0},
                rclcpp::Parameter{"racing_line_parallel_workers", 2},
                rclcpp::Parameter{"racing_line_window_pre_margin_m", 30.0},
                rclcpp::Parameter{"racing_line_window_post_margin_m", 35.0},
                rclcpp::Parameter{"racing_line_window_heading_threshold_deg", 12.5},
                rclcpp::Parameter{"racing_line_window_width_change_threshold_m", 3.5},
                rclcpp::Parameter{"racing_line_window_min_heading_span_deg", 15.0},
                rclcpp::Parameter{"racing_line_window_min_curvature_1pm", 0.02},
                rclcpp::Parameter{"racing_line_window_min_width_asymmetry_m", 2.5},
                rclcpp::Parameter{"racing_line_dp_offset_step_m", 0.75},
                rclcpp::Parameter{"racing_line_dp_coarse_offset_step_m", 2.5},
                rclcpp::Parameter{"racing_line_dp_fine_offset_step_m", 0.5},
                rclcpp::Parameter{"racing_line_dp_fine_radius_m", 2.25},
                rclcpp::Parameter{"racing_line_top_n_full_score_candidates", 64},
                rclcpp::Parameter{"racing_line_async_refinement_workers", 2},
                rclcpp::Parameter{"turn_smoothing_outer_bias_ratio", 0.7},
                rclcpp::Parameter{"turn_smoothing_max_outer_shift_m", 9.0},
                rclcpp::Parameter{"corridor_sample_step_m", 2.0},
                rclcpp::Parameter{"corridor_parallel_workers", 3},
                rclcpp::Parameter{"max_lidar_range_m", 22.0},
                rclcpp::Parameter{"scan_yaw_offset_rad", 0.3},
                rclcpp::Parameter{"motion_compensate_lidar_pose", false},
                rclcpp::Parameter{"lidar_pose_latency_s", 0.25},
                rclcpp::Parameter{"compensate_lidar_attitude", true}});

  const PlannerNodeConfig config = loadPlannerNodeConfig(*node);

  EXPECT_DOUBLE_EQ(config.planner_core.astar.heuristic_weight, 1.2);
  EXPECT_DOUBLE_EQ(config.planner_core.astar.turn_cost_weight, 2.0);
  EXPECT_TRUE(config.planner_core.astar.evasive_maneuvering_enabled);
  EXPECT_TRUE(config.planner_core.astar.initial_heading_bias_enabled);
  EXPECT_DOUBLE_EQ(config.planner_core.astar.initial_heading_bias_min_speed_mps, 1.25);
  EXPECT_DOUBLE_EQ(config.planner_core.astar.initial_heading_bias_weight, 75.0);
  EXPECT_DOUBLE_EQ(config.planner_core.clearance_diagnostic_radius_m, 40.0);
  EXPECT_FALSE(config.static_map.enabled);
  EXPECT_FALSE(config.planning_grid_builder.use_static_map);
  EXPECT_DOUBLE_EQ(config.timing.path_prohibited_intersection_check_period_s, 0.25);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.weight_curvature, 125.0);
  EXPECT_EQ(config.trajectory_planner.racing_line.parallel_workers, 2U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_pre_margin_m, 30.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_post_margin_m, 35.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_heading_threshold_rad,
                   12.5 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.racing_line.window_width_change_threshold_m, 3.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_min_heading_span_rad,
                   15.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_min_curvature_1pm,
                   0.02);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.window_min_width_asymmetry_m,
                   2.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_offset_step_m, 0.75);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_coarse_offset_step_m, 2.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_fine_offset_step_m, 0.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.racing_line.dp_fine_radius_m, 2.25);
  EXPECT_EQ(config.trajectory_planner.racing_line.top_n_full_score_candidates, 64U);
  EXPECT_EQ(config.trajectory_planner.racing_line.async_refinement_workers, 1U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.outer_bias_ratio, 0.7);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.max_outer_shift_m, 9.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.corridor.sample_step_m, 2.0);
  EXPECT_EQ(config.trajectory_planner.corridor.parallel_workers, 3U);
  EXPECT_DOUBLE_EQ(config.lidar_projection.max_lidar_range_m, 22.0);
  EXPECT_DOUBLE_EQ(config.lidar_projection.scan_yaw_offset_rad, 0.3);
  EXPECT_FALSE(config.current_lidar.motion_compensate_lidar_pose);
  EXPECT_DOUBLE_EQ(config.current_lidar.lidar_pose_latency_s, 0.25);
  EXPECT_TRUE(config.lidar_projection.compensate_attitude);
}

TEST_F(PlannerNodeConfigTest, AllowsAsyncRefinementDisableContract) {
  const auto node =
      makeNode("planner_node_config_async_refinement_disabled",
               {rclcpp::Parameter{"racing_line_async_refinement_workers", 0}});

  const PlannerNodeConfig config = loadPlannerNodeConfig(*node);

  EXPECT_EQ(config.trajectory_planner.racing_line.async_refinement_workers, 0U);
}

TEST_F(PlannerNodeConfigTest, LoadsRawAndProhibitedTopicContractParameters) {
  const auto node =
      makeNode("planner_node_config_topic_contract",
               {rclcpp::Parameter{"prohibited_grid_topic", "/custom/prohibited_grid"},
                rclcpp::Parameter{"trajectory_diagnostics_topic",
                                  "/custom/trajectory_diagnostics"},
                rclcpp::Parameter{"memory_occupied_value", 100},
                rclcpp::Parameter{"memory_free_value", 0}});

  const PlannerNodeConfig config = loadPlannerNodeConfig(*node);

  EXPECT_EQ(config.topics.prohibited_grid, "/custom/prohibited_grid");
  EXPECT_EQ(config.topics.trajectory_diagnostics, "/custom/trajectory_diagnostics");
  EXPECT_EQ(config.memory_grid.occupied_value, 100);
  EXPECT_EQ(config.memory_grid.free_value, 0);
}

TEST_F(PlannerNodeConfigTest, CapsHugePlanningGridFromParameters) {
  const auto node = makeNode("planner_node_config_huge_grid",
                             {rclcpp::Parameter{"planning_grid_resolution_m", 0.01},
                              rclcpp::Parameter{"planning_grid_width_m", 1.0e9},
                              rclcpp::Parameter{"planning_grid_height_m", 1.0e9}});

  const PlannerNodeConfig config = loadPlannerNodeConfig(*node);
  const auto& bounds = config.planning_grid_builder.fallback_bounds;
  const auto cell_count = static_cast<std::size_t>(bounds.width_cells) *
                          static_cast<std::size_t>(bounds.height_cells);

  EXPECT_EQ(bounds.width_cells, kMaxGridAxisCells);
  EXPECT_LE(cell_count, kMaxGridCellCount);
}

} // namespace drone_city_nav
