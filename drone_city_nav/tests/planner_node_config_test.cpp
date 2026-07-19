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
  EXPECT_DOUBLE_EQ(config.initial_altitude_m, 12.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.initial_altitude_m,
                   config.initial_altitude_m);
  EXPECT_DOUBLE_EQ(config.inflation_radius_m, 1.0);
  EXPECT_DOUBLE_EQ(config.planning_clearance_m, 3.0);
  EXPECT_EQ(config.async_trajectory_build_workers, 1U);
  EXPECT_DOUBLE_EQ(config.planning_grid_builder.inflation_radius_m, 1.0);
  EXPECT_DOUBLE_EQ(config.planning_grid_builder.planning_clearance_m, 3.0);
  EXPECT_TRUE(config.static_map.enabled);
  EXPECT_TRUE(config.planning_grid_builder.use_static_map);
  EXPECT_EQ(config.static_map.configured_path.string(), "worlds/generated_city.map2d");
  EXPECT_TRUE(config.known_passages.enabled);
  EXPECT_EQ(config.known_passages.configured_path.string(),
            "worlds/known_passages.passages3d");
  EXPECT_TRUE(config.known_passage_validation.enabled);
  EXPECT_DOUBLE_EQ(config.known_passage_validation.min_opening_overlap_m, 0.5);
  EXPECT_DOUBLE_EQ(config.known_passage_validation.min_opening_depth_fraction, 0.75);
  EXPECT_DOUBLE_EQ(config.known_passage_validation.clearance_margin_m, 0.5);
  EXPECT_EQ(config.known_passage_validation.max_diagnostics, 8U);
  EXPECT_DOUBLE_EQ(config.known_static_lidar_hit_closer_range_tolerance_m, 0.5);
  EXPECT_DOUBLE_EQ(config.known_static_lidar_hit_farther_range_tolerance_m, 1.5);
  EXPECT_DOUBLE_EQ(config.known_static_lidar_hit_endpoint_volume_tolerance_m, 0.5);
  EXPECT_EQ(config.current_lidar.ambiguous_hit_confirmation.required_independent_scans,
            3U);
  EXPECT_EQ(config.current_lidar.ambiguous_hit_confirmation.max_scan_gap_ns,
            500'000'000);
  EXPECT_EQ(config.current_lidar.ambiguous_hit_confirmation.retention_ns,
            2'000'000'000);
  EXPECT_TRUE(config.trajectory_planner.vertical_profile.enabled);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.gate_clearance_margin_m,
                   0.5);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.vertical_profile.preferred_gate_clearance_margin_m,
      1.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_climb_speed_mps, 3.2);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_descent_speed_mps,
                   3.2);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_vertical_accel_mps2,
                   3.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_vertical_jerk_mps3,
                   9.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_climb_angle_rad,
                   35.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.pre_gate_hold_time_s,
                   1.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.vertical_profile.pre_gate_hold_min_distance_m, 15.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.vertical_profile.pre_gate_hold_max_distance_m, 80.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.speed_profile.vertical_profile_max_climb_speed_mps,
      3.2);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.speed_profile.vertical_profile_max_descent_speed_mps,
      3.2);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.speed_profile.known_passage_traversal_speed_limit_mps,
      10.0);
  EXPECT_TRUE(config.trajectory_planner.passage_insertion.enabled);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.sample_step_m, 1.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.min_anchor_margin_m,
                   8.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.max_anchor_margin_m,
                   60.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.opening_lateral_target_margin_m, 1.5);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.repair_clearance_margin_m, 1.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.max_lateral_shift_m,
                   80.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.max_join_tangent_delta_rad,
      20.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.max_join_curvature_jump_1pm, 0.08);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.min_inserted_radius_m,
                   0.0);
  EXPECT_EQ(config.trajectory_planner.passage_insertion.max_candidates, 8U);
  EXPECT_EQ(config.trajectory_planner.passage_insertion.max_diagnostics, 8U);
  EXPECT_EQ(config.topics.prohibited_grid, "/drone_city_nav/prohibited_grid");
  EXPECT_EQ(config.topics.obstacle_memory_snapshot,
            "/drone_city_nav/obstacle_memory_snapshot");
  EXPECT_DOUBLE_EQ(config.memory_snapshot_transport.diagnostic_period_s, 5.0);
  EXPECT_DOUBLE_EQ(config.memory_snapshot_transport.max_age_ms, 350.0);
  EXPECT_DOUBLE_EQ(config.memory_snapshot_transport.max_callback_time_ms, 100.0);
  EXPECT_DOUBLE_EQ(config.memory_snapshot_transport.max_apply_delay_ms, 300.0);
  EXPECT_DOUBLE_EQ(config.memory_snapshot_transport.min_apply_rate_hz, 1.0);
  EXPECT_EQ(config.topics.static_building_markers,
            "/drone_city_nav/static_building_markers");
  EXPECT_EQ(config.topics.known_passage_markers,
            "/drone_city_nav/known_passage_markers");
  EXPECT_EQ(config.topics.path, "/drone_city_nav/path");
  EXPECT_EQ(config.topics.trajectory_diagnostics,
            "/drone_city_nav/trajectory_diagnostics");
  EXPECT_DOUBLE_EQ(config.timing.path_prohibited_intersection_check_period_s, 0.5);
  EXPECT_DOUBLE_EQ(config.timing.known_passage_debug_publish_period_s, 1.0);
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
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.weight_curvature,
                   300.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.weight_curvature_change, 220.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.preferred_min_radius_m, 30.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.weight_radius_shortfall, 70.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.weight_offset_second_change, 6.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.weight_offset_slope,
                   100.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.max_offset_slope_per_m, 0.32);
  EXPECT_EQ(config.trajectory_planner.trajectory_optimizer.parallel_workers, 0U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.window_pre_margin_m,
                   25.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.window_post_margin_m,
                   25.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_heading_threshold_rad,
      10.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_width_change_threshold_m,
      2.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_min_heading_span_rad,
      10.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_min_curvature_1pm, 0.01);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_min_width_asymmetry_m, 1.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.dp_offset_step_m,
                   1.5);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.dp_coarse_offset_step_m, 2.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.dp_fine_offset_step_m,
                   0.75);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.dp_fine_radius_m,
                   1.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.trigger_heading_delta_rad,
                   37.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.trigger_min_radius_m, 16.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.trigger_speed_limit_mps,
                   12.0);
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
       rclcpp::Parameter{"async_trajectory_build_workers", 5000},
       rclcpp::Parameter{"lidar_pose_latency_s", 5.0},
       rclcpp::Parameter{"cruise_speed_mps", 5000.0},
       rclcpp::Parameter{"min_turn_speed_mps", 5000.0},
       rclcpp::Parameter{"corridor_max_radius_m", -10.0},
       rclcpp::Parameter{"corridor_parallel_workers", 5000},
       rclcpp::Parameter{"trajectory_optimizer_parallel_workers", 5000},
       rclcpp::Parameter{"trajectory_optimizer_window_pre_margin_m", -1.0},
       rclcpp::Parameter{"trajectory_optimizer_window_post_margin_m", 9999.0},
       rclcpp::Parameter{"trajectory_optimizer_window_heading_threshold_deg", 500.0},
       rclcpp::Parameter{"trajectory_optimizer_window_width_change_threshold_m", -1.0},
       rclcpp::Parameter{"trajectory_optimizer_window_min_heading_span_deg", 500.0},
       rclcpp::Parameter{"trajectory_optimizer_window_min_curvature_1pm", -1.0},
       rclcpp::Parameter{"trajectory_optimizer_window_min_width_asymmetry_m", -1.0},
       rclcpp::Parameter{"trajectory_optimizer_dp_offset_step_m", -1.0},
       rclcpp::Parameter{"trajectory_optimizer_dp_coarse_offset_step_m", -1.0},
       rclcpp::Parameter{"trajectory_optimizer_dp_fine_offset_step_m", -1.0},
       rclcpp::Parameter{"trajectory_optimizer_dp_fine_radius_m", -1.0},
       rclcpp::Parameter{"known_passage_validation_min_opening_overlap_m", -1.0},
       rclcpp::Parameter{"known_passage_validation_min_opening_depth_fraction", 5.0},
       rclcpp::Parameter{"known_passage_validation_clearance_margin_m", 9999.0},
       rclcpp::Parameter{"known_passage_validation_max_diagnostics", 5000},
       rclcpp::Parameter{"known_passage_traversal_speed_limit_mps", 9999.0},
       rclcpp::Parameter{"known_static_lidar_hit_closer_range_tolerance_m", 9999.0},
       rclcpp::Parameter{"known_static_lidar_hit_farther_range_tolerance_m", 9999.0},
       rclcpp::Parameter{"vertical_profile_preferred_gate_clearance_margin_m", 9999.0},
       rclcpp::Parameter{"passage_insertion_sample_step_m", -1.0},
       rclcpp::Parameter{"passage_insertion_min_anchor_margin_m", -1.0},
       rclcpp::Parameter{"passage_insertion_max_anchor_margin_m", -1.0},
       rclcpp::Parameter{"passage_insertion_opening_lateral_target_margin_m", -1.0},
       rclcpp::Parameter{"passage_insertion_repair_clearance_margin_m", 9999.0},
       rclcpp::Parameter{"passage_insertion_max_lateral_shift_m", 9999.0},
       rclcpp::Parameter{"passage_insertion_max_join_tangent_delta_deg", 9999.0},
       rclcpp::Parameter{"passage_insertion_max_join_curvature_jump_1pm", 9999.0},
       rclcpp::Parameter{"passage_insertion_min_inserted_radius_m", -1.0},
       rclcpp::Parameter{"passage_insertion_max_candidates", 5000},
       rclcpp::Parameter{"passage_insertion_max_diagnostics", 5000},
       rclcpp::Parameter{"turn_smoothing_trigger_heading_delta_deg", 500.0},
       rclcpp::Parameter{"turn_smoothing_trigger_min_radius_m", -5.0},
       rclcpp::Parameter{"turn_smoothing_trigger_speed_limit_mps", -5.0},
       rclcpp::Parameter{"turn_smoothing_entry_distance_m", -5.0},
       rclcpp::Parameter{"known_passage_debug_publish_period_s", 100.0},
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
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.speed_profile.known_passage_traversal_speed_limit_mps,
      100.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.corridor.max_radius_m, 1.0);
  EXPECT_EQ(config.trajectory_planner.corridor.parallel_workers, 1024U);
  EXPECT_EQ(config.trajectory_planner.trajectory_optimizer.parallel_workers, 1024U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.window_pre_margin_m,
                   0.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.window_post_margin_m,
                   5000.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_heading_threshold_rad,
      std::numbers::pi);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_width_change_threshold_m,
      0.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_min_heading_span_rad,
      std::numbers::pi);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_min_curvature_1pm, 0.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_min_width_asymmetry_m, 0.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.dp_offset_step_m,
                   0.05);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.dp_coarse_offset_step_m, 0.05);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.dp_fine_offset_step_m,
                   0.05);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.dp_fine_radius_m,
                   0.05);
  EXPECT_EQ(config.async_trajectory_build_workers, 1U);
  EXPECT_DOUBLE_EQ(config.known_passage_validation.min_opening_overlap_m, 0.0);
  EXPECT_DOUBLE_EQ(config.known_passage_validation.min_opening_depth_fraction, 1.0);
  EXPECT_DOUBLE_EQ(config.known_passage_validation.clearance_margin_m, 1000.0);
  EXPECT_EQ(config.known_passage_validation.max_diagnostics, 100U);
  EXPECT_DOUBLE_EQ(config.known_static_lidar_hit_closer_range_tolerance_m, 100.0);
  EXPECT_DOUBLE_EQ(config.known_static_lidar_hit_farther_range_tolerance_m, 100.0);
  EXPECT_DOUBLE_EQ(config.known_static_lidar_hit_endpoint_volume_tolerance_m, 0.5);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.vertical_profile.preferred_gate_clearance_margin_m,
      100.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.sample_step_m, 0.1);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.min_anchor_margin_m,
                   0.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.max_anchor_margin_m,
                   0.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.opening_lateral_target_margin_m, 0.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.repair_clearance_margin_m, 1000.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.max_lateral_shift_m,
                   5000.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.max_join_tangent_delta_rad,
      std::numbers::pi);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.max_join_curvature_jump_1pm, 1000.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.min_inserted_radius_m,
                   0.0);
  EXPECT_EQ(config.trajectory_planner.passage_insertion.max_candidates, 100U);
  EXPECT_EQ(config.trajectory_planner.passage_insertion.max_diagnostics, 100U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.trigger_heading_delta_rad,
                   std::numbers::pi);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.trigger_min_radius_m, 0.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.trigger_speed_limit_mps,
                   0.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.turn_smoothing.entry_distance_m, 0.1);
  EXPECT_DOUBLE_EQ(config.timing.known_passage_debug_publish_period_s, 60.0);
  EXPECT_DOUBLE_EQ(config.timing.static_map_debug_publish_period_s, 60.0);
}

TEST_F(PlannerNodeConfigTest, BuildsNestedCoreConfigs) {
  const auto node = makeNode(
      "planner_node_config_nested",
      {rclcpp::Parameter{"astar_heuristic_weight", 1.2},
       rclcpp::Parameter{"astar_turn_cost_weight", 2.0},
       rclcpp::Parameter{"astar_evasive_maneuvering_enabled", true},
       rclcpp::Parameter{"astar_initial_heading_bias_enabled", true},
       rclcpp::Parameter{"astar_initial_heading_bias_min_speed_mps", 1.25},
       rclcpp::Parameter{"astar_initial_heading_bias_weight", 75.0},
       rclcpp::Parameter{"use_static_map", false},
       rclcpp::Parameter{"known_passages_enabled", false},
       rclcpp::Parameter{"known_passages_path", "worlds/custom.passages3d"},
       rclcpp::Parameter{"known_passage_validation_enabled", false},
       rclcpp::Parameter{"known_passage_validation_min_opening_overlap_m", 1.25},
       rclcpp::Parameter{"known_passage_validation_min_opening_depth_fraction", 0.5},
       rclcpp::Parameter{"known_passage_validation_clearance_margin_m", 0.75},
       rclcpp::Parameter{"known_passage_validation_max_diagnostics", 3},
       rclcpp::Parameter{"known_passage_traversal_speed_limit_mps", 8.5},
       rclcpp::Parameter{"vertical_profile_enabled", false},
       rclcpp::Parameter{"vertical_profile_gate_clearance_margin_m", 0.8},
       rclcpp::Parameter{"vertical_profile_preferred_gate_clearance_margin_m", 1.8},
       rclcpp::Parameter{"vertical_profile_max_climb_speed_mps", 3.5},
       rclcpp::Parameter{"vertical_profile_max_descent_speed_mps", 2.5},
       rclcpp::Parameter{"vertical_profile_max_vertical_accel_mps2", 2.75},
       rclcpp::Parameter{"vertical_profile_max_vertical_jerk_mps3", 9.0},
       rclcpp::Parameter{"vertical_profile_max_climb_angle_deg", 15.0},
       rclcpp::Parameter{"vertical_profile_min_transition_distance_m", 12.0},
       rclcpp::Parameter{"vertical_profile_max_transition_distance_m", 55.0},
       rclcpp::Parameter{"vertical_profile_pre_gate_hold_time_s", 2.0},
       rclcpp::Parameter{"vertical_profile_pre_gate_hold_min_distance_m", 18.0},
       rclcpp::Parameter{"vertical_profile_pre_gate_hold_max_distance_m", 65.0},
       rclcpp::Parameter{"vertical_profile_max_diagnostics", 4},
       rclcpp::Parameter{"passage_insertion_enabled", true},
       rclcpp::Parameter{"passage_insertion_sample_step_m", 0.75},
       rclcpp::Parameter{"passage_insertion_min_anchor_margin_m", 9.0},
       rclcpp::Parameter{"passage_insertion_max_anchor_margin_m", 45.0},
       rclcpp::Parameter{"passage_insertion_opening_lateral_target_margin_m", 0.5},
       rclcpp::Parameter{"passage_insertion_repair_clearance_margin_m", 1.75},
       rclcpp::Parameter{"passage_insertion_max_lateral_shift_m", 25.0},
       rclcpp::Parameter{"passage_insertion_max_join_tangent_delta_deg", 15.0},
       rclcpp::Parameter{"passage_insertion_max_join_curvature_jump_1pm", 0.12},
       rclcpp::Parameter{"passage_insertion_min_inserted_radius_m", 18.0},
       rclcpp::Parameter{"passage_insertion_max_candidates", 5},
       rclcpp::Parameter{"passage_insertion_max_diagnostics", 6},
       rclcpp::Parameter{"static_building_markers_topic", "/custom/static_buildings"},
       rclcpp::Parameter{"known_passage_markers_topic", "/custom/known_passages"},
       rclcpp::Parameter{"known_passage_debug_publish_period_s", 0.25},
       rclcpp::Parameter{"path_prohibited_intersection_check_period_s", 0.25},
       rclcpp::Parameter{"trajectory_optimizer_weight_curvature", 125.0},
       rclcpp::Parameter{"trajectory_optimizer_parallel_workers", 2},
       rclcpp::Parameter{"trajectory_optimizer_window_pre_margin_m", 30.0},
       rclcpp::Parameter{"trajectory_optimizer_window_post_margin_m", 35.0},
       rclcpp::Parameter{"trajectory_optimizer_window_heading_threshold_deg", 12.5},
       rclcpp::Parameter{"trajectory_optimizer_window_width_change_threshold_m", 3.5},
       rclcpp::Parameter{"trajectory_optimizer_window_min_heading_span_deg", 15.0},
       rclcpp::Parameter{"trajectory_optimizer_window_min_curvature_1pm", 0.02},
       rclcpp::Parameter{"trajectory_optimizer_window_min_width_asymmetry_m", 2.5},
       rclcpp::Parameter{"trajectory_optimizer_dp_offset_step_m", 0.75},
       rclcpp::Parameter{"trajectory_optimizer_dp_coarse_offset_step_m", 2.5},
       rclcpp::Parameter{"trajectory_optimizer_dp_fine_offset_step_m", 0.5},
       rclcpp::Parameter{"trajectory_optimizer_dp_fine_radius_m", 2.25},
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
  EXPECT_FALSE(config.known_passages.enabled);
  EXPECT_EQ(config.known_passages.configured_path.string(), "worlds/custom.passages3d");
  EXPECT_FALSE(config.known_passage_validation.enabled);
  EXPECT_DOUBLE_EQ(config.known_passage_validation.min_opening_overlap_m, 1.25);
  EXPECT_DOUBLE_EQ(config.known_passage_validation.min_opening_depth_fraction, 0.5);
  EXPECT_DOUBLE_EQ(config.known_passage_validation.clearance_margin_m, 0.75);
  EXPECT_EQ(config.known_passage_validation.max_diagnostics, 3U);
  EXPECT_FALSE(config.trajectory_planner.vertical_profile.enabled);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.gate_clearance_margin_m,
                   0.8);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.vertical_profile.preferred_gate_clearance_margin_m,
      1.8);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_climb_speed_mps, 3.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_descent_speed_mps,
                   2.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_vertical_accel_mps2,
                   2.75);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_vertical_jerk_mps3,
                   9.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_climb_angle_rad,
                   15.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.min_transition_distance_m,
                   12.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_transition_distance_m,
                   55.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.pre_gate_hold_time_s,
                   2.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.vertical_profile.pre_gate_hold_min_distance_m, 18.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.vertical_profile.pre_gate_hold_max_distance_m, 65.0);
  EXPECT_EQ(config.trajectory_planner.vertical_profile.max_diagnostics, 4U);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.speed_profile.vertical_profile_max_climb_speed_mps,
      3.5);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.speed_profile.vertical_profile_max_descent_speed_mps,
      2.5);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.speed_profile.known_passage_traversal_speed_limit_mps,
      8.5);
  EXPECT_TRUE(config.trajectory_planner.passage_insertion.enabled);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.sample_step_m, 0.75);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.min_anchor_margin_m,
                   9.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.max_anchor_margin_m,
                   45.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.opening_lateral_target_margin_m, 0.5);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.repair_clearance_margin_m, 1.75);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.max_lateral_shift_m,
                   25.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.max_join_tangent_delta_rad,
      15.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.passage_insertion.max_join_curvature_jump_1pm, 0.12);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.passage_insertion.min_inserted_radius_m,
                   18.0);
  EXPECT_EQ(config.trajectory_planner.passage_insertion.max_candidates, 5U);
  EXPECT_EQ(config.trajectory_planner.passage_insertion.max_diagnostics, 6U);
  EXPECT_EQ(config.topics.static_building_markers, "/custom/static_buildings");
  EXPECT_EQ(config.topics.known_passage_markers, "/custom/known_passages");
  EXPECT_DOUBLE_EQ(config.timing.known_passage_debug_publish_period_s, 0.25);
  EXPECT_DOUBLE_EQ(config.timing.path_prohibited_intersection_check_period_s, 0.25);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.weight_curvature,
                   125.0);
  EXPECT_EQ(config.trajectory_planner.trajectory_optimizer.parallel_workers, 2U);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.window_pre_margin_m,
                   30.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.window_post_margin_m,
                   35.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_heading_threshold_rad,
      12.5 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_width_change_threshold_m,
      3.5);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_min_heading_span_rad,
      15.0 * std::numbers::pi / 180.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_min_curvature_1pm, 0.02);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.window_min_width_asymmetry_m, 2.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.dp_offset_step_m,
                   0.75);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.trajectory_optimizer.dp_coarse_offset_step_m, 2.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.dp_fine_offset_step_m,
                   0.5);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.trajectory_optimizer.dp_fine_radius_m,
                   2.25);
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

TEST_F(PlannerNodeConfigTest, LoadsRawAndProhibitedTopicContractParameters) {
  const auto node = makeNode(
      "planner_node_config_topic_contract",
      {rclcpp::Parameter{"obstacle_memory_snapshot_topic", "/custom/memory_snapshot"},
       rclcpp::Parameter{"obstacle_memory_snapshot_diagnostic_period_s", 2.0},
       rclcpp::Parameter{"obstacle_memory_snapshot_max_age_ms", 150.0},
       rclcpp::Parameter{"obstacle_memory_snapshot_max_callback_time_ms", 75.0},
       rclcpp::Parameter{"obstacle_memory_snapshot_max_apply_delay_ms", 125.0},
       rclcpp::Parameter{"obstacle_memory_snapshot_min_apply_rate_hz", 8.0},
       rclcpp::Parameter{"prohibited_grid_topic", "/custom/prohibited_grid"},
       rclcpp::Parameter{"trajectory_diagnostics_topic",
                         "/custom/trajectory_diagnostics"},
       rclcpp::Parameter{"memory_occupied_value", 100},
       rclcpp::Parameter{"memory_free_value", 0}});

  const PlannerNodeConfig config = loadPlannerNodeConfig(*node);

  EXPECT_EQ(config.topics.prohibited_grid, "/custom/prohibited_grid");
  EXPECT_EQ(config.topics.obstacle_memory_snapshot, "/custom/memory_snapshot");
  EXPECT_DOUBLE_EQ(config.memory_snapshot_transport.diagnostic_period_s, 2.0);
  EXPECT_DOUBLE_EQ(config.memory_snapshot_transport.max_age_ms, 150.0);
  EXPECT_DOUBLE_EQ(config.memory_snapshot_transport.max_callback_time_ms, 75.0);
  EXPECT_DOUBLE_EQ(config.memory_snapshot_transport.max_apply_delay_ms, 125.0);
  EXPECT_DOUBLE_EQ(config.memory_snapshot_transport.min_apply_rate_hz, 8.0);
  EXPECT_EQ(config.topics.trajectory_diagnostics, "/custom/trajectory_diagnostics");
  EXPECT_EQ(config.memory_grid.occupied_value, 100);
  EXPECT_EQ(config.memory_grid.free_value, 0);
}

TEST_F(PlannerNodeConfigTest, AllowsAsyncTrajectoryBuildDisableContract) {
  const auto node = makeNode("planner_node_config_async_trajectory_build_disabled",
                             {rclcpp::Parameter{"async_trajectory_build_workers", 0}});

  const PlannerNodeConfig config = loadPlannerNodeConfig(*node);

  EXPECT_EQ(config.async_trajectory_build_workers, 0U);
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

TEST_F(PlannerNodeConfigTest, CapsVerticalProfileSpeedByRuntimeSetpointLimit) {
  const auto node =
      makeNode("planner_node_config_vertical_runtime_cap",
               {rclcpp::Parameter{"vertical_profile_max_climb_speed_mps", 3.5},
                rclcpp::Parameter{"vertical_profile_max_descent_speed_mps", 3.0},
                rclcpp::Parameter{"vertical_setpoint_max_climb_speed_mps", 2.0},
                rclcpp::Parameter{"vertical_setpoint_max_descent_speed_mps", 1.5}});

  const PlannerNodeConfig config = loadPlannerNodeConfig(*node);

  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_climb_speed_mps, 2.0);
  EXPECT_DOUBLE_EQ(config.trajectory_planner.vertical_profile.max_descent_speed_mps,
                   1.5);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.speed_profile.vertical_profile_max_climb_speed_mps,
      2.0);
  EXPECT_DOUBLE_EQ(
      config.trajectory_planner.speed_profile.vertical_profile_max_descent_speed_mps,
      1.5);
}

} // namespace drone_city_nav
