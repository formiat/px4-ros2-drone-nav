#!/usr/bin/env python3
"""Static launch-contract tests for the drone navigation simulator runner."""

from __future__ import annotations

import json
import unittest
from pathlib import Path


RUNNER = Path(__file__).resolve().parents[1] / "run_drone_nav_sim.sh"
RUNTIME_HELPERS = RUNNER.with_name("simulation_runtime_helpers.sh")
LIDAR_PROFILE_RUNTIME = RUNNER.with_name("lidar_profile_runtime.sh")
LIDAR_PROFILE_SUPPORT_FILE = (
    RUNNER.parents[1] / "drone_city_nav" / "launch" / "lidar_profile.py"
)
RESOURCE_RUNTIME = RUNNER.with_name("simulation_resource_runtime.sh")
MAKEFILE = RUNNER.parents[1] / "Makefile"
INTERCEPT_RUNTIME_HELPER = RUNNER.with_name("multi_vehicle_sim_runtime.sh")
GAZEBO_SPECTATOR_FOLLOW = RUNNER.with_name("gazebo_spectator_follow.py")
GAZEBO_GUI_CAMERA_RUNTIME = RUNNER.with_name("gazebo_gui_camera_runtime.sh")
CONTAINER_RUNNER = Path(__file__).resolve().parents[1] / "container_run.sh"
LAUNCH_FILE = (
    Path(__file__).resolve().parents[2]
    / "drone_city_nav"
    / "launch"
    / "city_nav.launch.py"
)
INTERCEPT_LAUNCH_FILE = (
    Path(__file__).resolve().parents[2]
    / "drone_city_nav"
    / "launch"
    / "intercept.launch.py"
)
INTERCEPT_TRACKING_LAUNCH_FILE = INTERCEPT_LAUNCH_FILE.with_name(
    "intercept_tracking_launch.py"
)
MULTI_VEHICLE_LAUNCH_FILE = INTERCEPT_LAUNCH_FILE.with_name(
    "multi_vehicle.launch.py"
)
MULTI_VEHICLE_MISSION_LAUNCH_FILE = INTERCEPT_LAUNCH_FILE.with_name(
    "multi_vehicle_mission_launch.py"
)
MULTI_VEHICLE_LIDAR_LAUNCH_FILE = INTERCEPT_LAUNCH_FILE.with_name(
    "multi_vehicle_lidar_launch.py"
)
MULTI_VEHICLE_LAUNCH_VALUES_FILE = INTERCEPT_LAUNCH_FILE.with_name(
    "multi_vehicle_launch_values.py"
)
INTERCEPT_DIAGNOSTICS_LAUNCH_FILE = INTERCEPT_LAUNCH_FILE.with_name(
    "intercept_diagnostics_launch.py"
)
INTERCEPT_SCENARIO = (
    Path(__file__).resolve().parents[2]
    / "drone_city_nav"
    / "config"
    / "intercept_scenario.json"
)
NAV_CONFIG = (
    Path(__file__).resolve().parents[2]
    / "drone_city_nav"
    / "config"
    / "urban_mvp.yaml"
)
PRODUCTION_MPPI_SOURCE = (
    Path(__file__).resolve().parents[2]
    / "drone_city_nav"
    / "src"
    / "runtime"
    / "ros"
    / "production_mppi_node.cpp"
)
PRODUCTION_MPPI_CONFIG_SOURCE = PRODUCTION_MPPI_SOURCE.with_name(
    "production_mppi_config_ros.cpp"
)
PRODUCTION_MPPI_RUNTIME_SOURCE = (
    Path(__file__).resolve().parents[2]
    / "drone_city_nav"
    / "src"
    / "runtime"
    / "ros"
    / "production_mppi_node_runtime.cpp"
)
RVIZ_CONFIGS = (
    Path(__file__).resolve().parents[2]
    / "drone_city_nav"
    / "rviz"
    / "city_nav_debug.rviz",
    Path(__file__).resolve().parents[2]
    / "drone_city_nav"
    / "rviz"
    / "city_nav_debug_top_down.rviz",
)


class RunDroneNavSimLaunchContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.text = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (RUNNER, LIDAR_PROFILE_RUNTIME, RESOURCE_RUNTIME)
        )
        cls.makefile_text = MAKEFILE.read_text(encoding="utf-8")
        cls.lidar_profile_support_text = LIDAR_PROFILE_SUPPORT_FILE.read_text(
            encoding="utf-8"
        )
        cls.intercept_runtime_text = cls.text + INTERCEPT_RUNTIME_HELPER.read_text(
            encoding="utf-8"
        )
        cls.gazebo_spectator_follow_text = GAZEBO_SPECTATOR_FOLLOW.read_text(
            encoding="utf-8"
        )
        cls.gazebo_gui_camera_runtime_text = GAZEBO_GUI_CAMERA_RUNTIME.read_text(
            encoding="utf-8"
        )
        cls.container_text = CONTAINER_RUNNER.read_text(encoding="utf-8")
        cls.launch_text = LAUNCH_FILE.read_text(encoding="utf-8")
        cls.intercept_launch_text = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (
                INTERCEPT_LAUNCH_FILE,
                MULTI_VEHICLE_LIDAR_LAUNCH_FILE,
                MULTI_VEHICLE_LAUNCH_VALUES_FILE,
                MULTI_VEHICLE_LAUNCH_FILE,
                MULTI_VEHICLE_MISSION_LAUNCH_FILE,
                INTERCEPT_DIAGNOSTICS_LAUNCH_FILE,
            )
        )
        cls.intercept_tracking_launch_text = (
            INTERCEPT_TRACKING_LAUNCH_FILE.read_text(encoding="utf-8")
        )
        cls.intercept_scenario = json.loads(
            INTERCEPT_SCENARIO.read_text(encoding="utf-8")
        )
        cls.nav_config_text = NAV_CONFIG.read_text(encoding="utf-8")
        cls.production_mppi_source_text = PRODUCTION_MPPI_SOURCE.read_text(
            encoding="utf-8"
        )
        cls.production_mppi_config_source_text = (
            PRODUCTION_MPPI_CONFIG_SOURCE.read_text(encoding="utf-8")
        )
        cls.production_mppi_runtime_source_text = (
            PRODUCTION_MPPI_RUNTIME_SOURCE.read_text(encoding="utf-8")
        )

    def test_gazebo_gui_launch_uses_native_follow_camera(self) -> None:
        self.assertIn("configure_gazebo_gui_follow_camera", self.text)
        self.assertIn("wait_for_gazebo_scene_entity", self.text)
        self.assertIn("wait_for_gazebo_world", self.text)
        self.assertIn('GZ_WORLD_READY_WAIT_S:-180', self.text)
        self.assertIn("gazebo_gui_control.py", self.gazebo_gui_camera_runtime_text)
        self.assertNotIn("--gui-config", self.text)
        self.assertNotIn("GZ_GUI_PLUGIN_PATH", self.text)

    def test_gazebo_follow_camera_defaults_match_forward_view(self) -> None:
        self.assertIn(
            'gazebo_gui_follow_offset="${GZ_GUI_FOLLOW_OFFSET:--12 0 6}"',
            self.text,
        )
        self.assertIn(
            'parser.add_argument("--offset", default="-12 0 6")',
            self.gazebo_spectator_follow_text,
        )

    def test_point_to_point_default_spawn_is_above_the_ground_plane(self) -> None:
        self.assertIn(
            'point_to_point_gazebo_spawn_pose:--171.0,-81.0,0.3,0,0,0',
            self.intercept_runtime_text,
        )

    def test_headless_point_to_point_result_stops_its_launch(self) -> None:
        self.assertIn("OnProcessExit", self.launch_text)
        self.assertIn("target_action=mission_monitor", self.launch_text)
        self.assertIn('Shutdown(reason="point-to-point mission result")', self.launch_text)

    def test_only_point_to_point_activates_the_configured_mission_objective(
        self,
    ) -> None:
        startup_objective = self.production_mppi_source_text.split(
            "navigation_objective_state_.store", maxsplit=1
        )[1].split("objective_replan_anchor_", maxsplit=1)[0]
        normalized_startup_objective = " ".join(startup_objective.split())

        self.assertIn(
            '"configured_mission_objective_enabled", false',
            self.production_mppi_config_source_text,
        )
        self.assertIn(
            ".mission_epoch = config_.planning.configured_mission_objective_enabled "
            "? 1U : 0U,",
            normalized_startup_objective,
        )
        self.assertIn(".sample_sequence = 0U", startup_objective)
        self.assertIn(
            '"configured_mission_objective_enabled": True', self.launch_text
        )
        self.assertIn(
            '"configured_mission_objective_enabled": False',
            self.intercept_launch_text,
        )

    def test_intercept_spectator_selection_is_launch_configurable(self) -> None:
        for variable in (
            "INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID",
            "INTERCEPT_SPECTATOR_RESELECTION_DELAY_S",
            "INTERCEPT_SPECTATOR_RESELECTION_POLICY",
        ):
            with self.subTest(variable=variable):
                self.assertIn(variable, self.text)
                self.assertIn(variable, self.container_text)
        self.assertIn('spectator_initial_vehicle_id:=', self.text)
        self.assertIn('spectator_reselection_policy:=', self.text)
        self.assertIn('spectator_reselection_delay_s:=', self.text)
        self.assertIn(
            'DeclareLaunchArgument("spectator_initial_vehicle_id"',
            self.intercept_launch_text,
        )
        self.assertIn(
            '"spectator_reselection_policy", default_value="first_living"',
            self.intercept_launch_text,
        )
        self.assertIn(
            '"spectator_reselection_delay_s", default_value="3.0"',
            self.intercept_launch_text,
        )

    def test_single_intercept_gui_observes_attacker_first(self) -> None:
        self.assertIn(
            'INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID="$${'
            'INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID:-evader}"',
            self.makefile_text,
        )

    def test_multi_intercept_observes_first_evader_then_next_living(self) -> None:
        self.assertIn(
            'INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID="$${'
            'INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID:-evader_0}"',
            self.makefile_text,
        )
        self.assertIn(
            'INTERCEPT_SPECTATOR_RESELECTION_POLICY="$${'
            'INTERCEPT_SPECTATOR_RESELECTION_POLICY:-next_living}"',
            self.makefile_text,
        )

    def test_gazebo_gui_launch_uses_direct_gui_command(self) -> None:
        self.assertIn("gz sim -g", self.text)
        self.assertNotIn("gz sim -g > /dev/null", self.text)
        self.assertIn('gz sim -g >> "${gz_gui_log_file}"', self.text)

    def test_gazebo_gui_log_is_separate_from_server_log(self) -> None:
        self.assertIn("gz_gui_log_file=", self.text)
        self.assertIn('echo "Gazebo GUI log: ${gz_gui_log_file}"', self.text)
        self.assertIn(': > "${gz_gui_log_file}"', self.text)

    def test_gui_camera_pose_is_logged_at_a_configurable_cadence(self) -> None:
        self.assertIn("gazebo_gui_camera_runtime.sh", self.text)
        self.assertIn("start_gazebo_gui_camera_logger", self.text)
        self.assertIn("GZ_GUI_CAMERA_LOG_INTERVAL_S:-1", self.gazebo_gui_camera_runtime_text)
        self.assertIn("gazebo_gui_camera_logger.py", self.gazebo_gui_camera_runtime_text)
        self.assertIn("gz_camera_logger_pid", self.text)
        self.assertIn("GZ_GUI_CAMERA_LOG_INTERVAL_S", self.container_text)

    def test_gui_ros_output_is_written_directly_without_terminal_tee(self) -> None:
        self.assertIn('> "${ros_log_file}" 2>&1', self.text)
        self.assertNotIn('2>&1 | tee "${ros_log_file}"', self.text)

    def test_gazebo_scene_diagnostics_are_captured(self) -> None:
        self.assertIn("ENABLE_GZ_SCENE_DIAGNOSTICS", self.text)
        self.assertIn("capture_gazebo_scene_diagnostics", self.text)
        self.assertIn(
            "scripts/capture_gazebo_scene_diagnostics.py",
            self.gazebo_gui_camera_runtime_text,
        )
        diagnostics_text = (
            RUNNER.parent / "capture_gazebo_scene_diagnostics.py"
        ).read_text(encoding="utf-8")
        self.assertIn("/gui/currently_tracked", diagnostics_text)

    def test_cleanup_has_no_unbounded_wait(self) -> None:
        self.assertNotIn('wait "${pids[@]}"', self.text)
        self.assertNotIn("wait ${job_pids}", self.text)
        self.assertIn("terminate_pids_bounded", self.text)

    def test_world_unpause_uses_world_control_pause_false(self) -> None:
        self.assertIn("world-running", self.gazebo_gui_camera_runtime_text)
        helper_text = (RUNNER.parent / "gazebo_gui_control.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("/world/{world}/control", helper_text)
        self.assertIn("pause: false", helper_text)

    def test_stale_cleanup_runs_before_gazebo_launch(self) -> None:
        cleanup_index = self.text.index("clean_stale_gazebo_processes | tee")
        launch_index = self.text.index('gz sim "${gz_args[@]}"')
        self.assertLess(cleanup_index, launch_index)

    def test_gazebo_log_preserves_cleanup_diagnostics(self) -> None:
        truncate_statement = ': > "${gz_log_file}"'
        self.assertEqual(1, self.text.count(truncate_statement))
        truncate_index = self.text.index(truncate_statement)
        cleanup_index = self.text.index("clean_stale_gazebo_processes | tee")
        launch_redirect_index = self.text.index(') >> "${gz_log_file}" 2>&1 &')
        self.assertLess(truncate_index, cleanup_index)
        self.assertLess(cleanup_index, launch_redirect_index)

    def test_lidar_debug_uses_per_run_directory_without_cleanup_race(self) -> None:
        self.assertIn("run_id=", self.text)
        self.assertIn(
            'lidar_debug_dir="${LIDAR_DEBUG_DIR:-${run_log_dir}/lidar_debug/${run_id}}"',
            self.text,
        )
        self.assertNotIn('rm -rf "${lidar_debug_dir}"', self.text)

    def test_memory_hit_diagnostics_use_a_per_run_dump(self) -> None:
        self.assertIn("LIDAR_MEMORY_HIT_DUMP_PATH", self.text)
        self.assertIn("lidar_memory_hits/${run_id}.jsonl", self.text)
        self.assertIn('lidar_memory_hit_dump_path:=', self.text)
        self.assertIn("lidar_memory_hit_dump_path", self.launch_text)

    def test_rviz_follow_camera_defaults_on_and_can_be_disabled(self) -> None:
        self.assertIn("ENABLE_RVIZ_FOLLOW_CAMERA:-true", self.text)
        self.assertIn("city_nav_debug.rviz", self.text)
        self.assertIn("city_nav_debug_top_down.rviz", self.text)
        self.assertIn("rviz_drone_follow_tf_enabled:=", self.text)
        self.assertIn("ENABLE_RVIZ_FOLLOW_CAMERA", self.container_text)

    def test_rviz_static_map_receives_latched_point_cloud(self) -> None:
        for config_path in RVIZ_CONFIGS:
            with self.subTest(config=config_path.name):
                config = config_path.read_text(encoding="utf-8")
                static_map_display = config.split(
                    "Name: Static City Map", 1
                )[1].split("Name: Static Building Volumes", 1)[0]
                display = config.split("Name: Static City Map Points", 1)[1].split(
                    "Name: Static Building Volumes", 1
                )[0]
                self.assertIn("Alpha: 0.12", static_map_display)
                self.assertIn("Durability Policy: Transient Local", display)
                self.assertIn("Reliability Policy: Reliable", display)
                self.assertIn("History Policy: Keep Last", display)
                self.assertIn("Depth: 1", display)

    def test_rviz_3d_lidar_uses_bounded_selected_clouds(self) -> None:
        self.assertIn(
            '--lidar-profile "${lidar_profile}"', self.intercept_runtime_text
        )
        self.assertIn(
            "REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING",
            self.intercept_runtime_text,
        )
        self.assertIn(
            "OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M",
            self.intercept_runtime_text,
        )
        self.assertNotIn("REQUIRE_INCREMENTAL_TOPOLOGY_EVIDENCE", self.intercept_runtime_text)
        self.assertNotIn("REQUIRE_ONLINE_3D_PASSAGE", self.intercept_runtime_text)
        self.assertIn(
            '"current_lidar_3d_pointcloud_topics": [', self.intercept_launch_text
        )
        for config_path in RVIZ_CONFIGS:
            with self.subTest(config=config_path.name):
                config = config_path.read_text(encoding="utf-8")
                current = config.split(
                    "Name: Current 3D Lidar Returns", 1
                )[1].split("Name: Remembered Lidar Hits", 1)[0]
                accumulated = config.split(
                    "Name: Accumulated 3D Obstacle Memory", 1
                )[1].split("Name: Interceptor 0 Memory", 1)[0]
                self.assertIn("Decay Time: 0", current)
                self.assertIn("Style: Points", current)
                self.assertIn("Depth: 1", current)
                self.assertIn("Reliability Policy: Best Effort", current)
                self.assertIn(
                    "Value: /drone_city_nav/current_lidar_returns_3d", current
                )
                self.assertIn("Decay Time: 0", accumulated)
                self.assertIn("Style: Points", accumulated)
                self.assertIn("Depth: 1", accumulated)
                self.assertIn("Durability Policy: Transient Local", accumulated)
                self.assertIn("History Policy: Keep Last", accumulated)
                self.assertIn("Reliability Policy: Best Effort", accumulated)
                self.assertIn(
                    "Value: /drone_city_nav/raw_memory_obstacle_points_3d",
                    accumulated,
                )

    def test_every_simulation_entrypoint_defaults_to_3d_lidar(self) -> None:
        self.assertIn('lidar_profile="${LIDAR_PROFILE:-3d}"', self.text)
        self.assertIn(
            'DEFAULT_LIDAR_PROFILE = "3d"', self.lidar_profile_support_text
        )
        self.assertIn(
            '"lidar_profile",\n                default_value=DEFAULT_LIDAR_PROFILE',
            self.launch_text,
        )
        self.assertIn(
            '"lidar_profile", default_value=_DEFAULT_LIDAR_PROFILE',
            self.intercept_launch_text,
        )

    def test_launch_uses_offboard_flight_control_backend(self) -> None:
        self.assertIn('executable="mppi_offboard_node"', self.launch_text)
        self.assertIn("nodes.append(", self.launch_text)
        self.assertIn('executable="production_mppi_node"', self.launch_text)

    def test_planner_exit_is_a_terminal_launch_failure(self) -> None:
        self.assertIn("target_action=production_mppi", self.launch_text)
        self.assertIn('Shutdown(reason="production MPPI exited")', self.launch_text)

    def test_simulation_performs_cuda_allocation_preflight_before_gazebo(self) -> None:
        preflight = RUNNER.with_name("cuda_preflight.py")
        self.assertTrue(preflight.is_file())
        source = preflight.read_text(encoding="utf-8")
        self.assertIn("cudaMalloc", source)
        self.assertIn("cudaFree", source)
        invocation = self.text.index("cuda_preflight.py")
        gazebo = self.text.index("gz sim")
        self.assertLess(invocation, gazebo)

    def test_point_to_point_scenario_drives_spawn_and_navigation_contract(self) -> None:
        self.assertIn("POINT_TO_POINT_SCENARIO_PATH", self.text)
        self.assertIn("load_point_to_point_sim_scenario", self.intercept_runtime_text)
        self.assertIn("point_to_point_scenario_path:=", self.text)
        self.assertIn("load_point_to_point_scenario", self.launch_text)
        self.assertIn('DeclareLaunchArgument(\n                "point_to_point_scenario_path"', self.launch_text)
        self.assertIn('"px4_local_origin_x_m": start_x_m', self.launch_text)
        self.assertIn('"px4_local_origin_z_m": start_z_m', self.launch_text)
        self.assertIn('px4_to_map_matrix = scenario["px4_to_map_matrix"]', self.launch_text)
        for index, entry in enumerate(("m00", "m01", "m10", "m11")):
            self.assertIn(
                f'"px4_to_map_{entry}": px4_to_map_matrix[{index}]', self.launch_text
            )
        self.assertIn("**px4_frame_overrides,", self.launch_text)
        # The RViz gazebo_map transform and every overlay convention follow the
        # canonical world instead of the legacy generated-city rotation.
        self.assertIn(
            'gazebo_axes_swapped = scenario["gazebo_axes_swapped"]', self.launch_text
        )
        self.assertIn(
            '"gazebo_aligned_rviz_axes_swapped": gazebo_axes_swapped', self.launch_text
        )
        self.assertIn(
            "gazebo_aligned_map_transform_arguments(", self.launch_text
        )
        self.assertNotIn('"0.7071067811865476"', self.launch_text)
        self.assertNotIn('"initial_altitude_m"', self.launch_text)
        self.assertIn('"mission_goal_sequence_xyz_m": [', self.launch_text)
        self.assertNotIn('"goal_x_m": goal_x_m', self.launch_text)
        self.assertIn(
            "mission_goal_sequence_xyz_m: [216.0, 378.0, 18.0]",
            self.nav_config_text,
        )
        self.assertNotIn("goal_x_m:", self.nav_config_text)
        self.assertIn("static_esdf_3d_cache_path", self.launch_text)
        self.assertIn("optional_nonnegative_float_override", self.launch_text)
        self.assertIn("cruise_speed_mps", self.launch_text)
        self.assertIn("absolute_speed_limit_mps", self.launch_text)
        self.assertIn("maximum_horizontal_acceleration_mps2", self.launch_text)
        self.assertNotIn("static_cruise_speed_mps", self.launch_text)
        self.assertNotIn("static_absolute_speed_limit_mps", self.launch_text)
        self.assertIn("cruise_speed_mps: 6.5", self.nav_config_text)
        self.assertIn("absolute_speed_limit_mps: 10.0", self.nav_config_text)
        self.assertIn(
            "maximum_horizontal_acceleration_mps2: 5.0", self.nav_config_text
        )
        self.assertIn('lidar_profile = LaunchConfiguration("lidar_profile")', self.launch_text)
        self.assertIn("gz.msgs.PointCloudPacked", self.launch_text)

    def test_base_sim_uses_environment_configurable_terminal_waypoints(self) -> None:
        self.assertIn("MISSION_GOALS_XYZ_M", self.makefile_text)
        self.assertIn(
            "216,378,18;216,54,18;54,378,18;54,54,18", self.makefile_text
        )
        self.assertIn(
            '-z "$${POINT_TO_POINT_SCENARIO_PATH:-}"', self.makefile_text
        )
        self.assertNotIn(
            'MISSION_GOALS_XYZ_M="$${MISSION_GOALS_XYZ_M:-216,378,18',
            self.makefile_text,
        )
        runtime_helpers = RUNTIME_HELPERS.read_text(encoding="utf-8")
        self.assertIn("simulation_runtime_helpers.sh", self.text)
        self.assertIn("format_mission_goal_sequence", runtime_helpers)
        self.assertIn("mission_goal_sequence_xyz_m:=", self.text)
        self.assertIn("POINT_TO_POINT_SHUTDOWN_ON_MISSION_RESULT", self.text)
        self.assertIn("mission_goal_sequence_xyz_m", self.launch_text)
        self.assertIn("shutdown_on_mission_result", self.launch_text)
        self.assertIn(
            "ENABLE_LIVENESS_RECOVERY:-true",
            self.text,
        )
        self.assertIn(
            "ENABLE_ROUTE_STALL_RECOVERY:-false",
            self.text,
        )
        self.assertNotIn("default_no_static_progress_recovery", self.text)
        self.assertNotIn("default_no_static_cycle_recovery", self.text)
        self.assertIn("route_stall_recovery_enabled", self.launch_text)
        self.assertNotIn("no_static_cycle_recovery_enabled", self.launch_text)
        self.assertNotIn("ENABLE_NO_STATIC_CYCLE_RECOVERY", self.text)
        self.assertIn(
            'if [[ -n "${point_to_point_scenario_path}" ]]', self.text
        )

    def test_intercept_evader_route_crosses_city_diagonally(self) -> None:
        evader = next(
            vehicle
            for vehicle in self.intercept_scenario["vehicles"]
            if vehicle["id"] == "evader"
        )
        self.assertEqual(evader["map_start_m"][:2], [216.0, 54.0])
        self.assertEqual(self.intercept_scenario["evader_goal_m"][:2], [54.0, 378.0])

    def test_interceptor_2_and_evader_start_on_adjacent_east_lanes(self) -> None:
        interceptor = next(
            vehicle
            for vehicle in self.intercept_scenario["vehicles"]
            if vehicle["id"] == "interceptor_2"
        )
        evader = next(
            vehicle
            for vehicle in self.intercept_scenario["vehicles"]
            if vehicle["id"] == "evader"
        )
        self.assertEqual(interceptor["map_start_m"][:2], [270.0, 378.0])
        self.assertEqual(evader["map_start_m"][:2], [216.0, 54.0])
        self.assertEqual(interceptor["map_start_m"][0] - evader["map_start_m"][0], 54.0)

    def test_intercept_evader_defaults_to_interceptor_speed(self) -> None:
        self.assertIn(
            'evader_speed_scale="${EVADER_SPEED_SCALE:-1.0}"', self.text
        )
        self.assertIn(
            'DeclareLaunchArgument("evader_speed_scale", default_value="1.0")',
            self.intercept_launch_text,
        )

    def test_intercept_evader_uses_a_red_gazebo_marker_variant(self) -> None:
        self.assertIn(
            'evader_model_name="${multi_vehicle_px4_model_targets[instance]#gz_}"',
            self.intercept_runtime_text,
        )
        self.assertIn(
            "configure_drone_marker_color.py", self.intercept_runtime_text
        )
        self.assertIn(
            'PX4_SIM_MODEL="${multi_vehicle_px4_model_targets[instance]}"',
            self.text,
        )
        evader = self.intercept_scenario["vehicles"][-1]
        self.assertEqual(evader["px4_model_target"], "gz_x500_lidar_2d_evader")
        self.assertEqual(evader["gazebo_model_name"], "x500_lidar_2d_evader_3")

    def test_intercept_launch_configures_adaptive_predictive_guidance(self) -> None:
        expected_defaults = {
            "intercept_minimum_prediction_horizon_s": "0.0",
            "intercept_maximum_prediction_horizon_s": "15.0",
            "intercept_ahead_maximum_prediction_horizon_s": "1.0",
            "intercept_fallback_prediction_horizon_s": "1.0",
            "intercept_minimum_target_speed_mps": "0.5",
            "intercept_ahead_enter_m": "5.0",
            "intercept_ahead_exit_m": "0.0",
            "intercept_ahead_corridor_enter_m": "15.0",
            "intercept_ahead_corridor_exit_m": "20.0",
            "intercept_horizon_smoothing_time_constant_s": "0.5",
        }
        for name, default in expected_defaults.items():
            with self.subTest(parameter=name):
                self.assertRegex(
                    self.intercept_launch_text,
                    rf'"{name}",\s*default_value="{default}"',
                )
                self.assertIn(f'"{name}",', self.intercept_launch_text)
                self.assertRegex(
                    self.intercept_tracking_launch_text,
                    rf'settings\[\s*"{name}"\s*\]',
                )

    def test_intercept_directional_hypotheses_are_disabled_by_default(self) -> None:
        self.assertIn(
            'INTERCEPT_DIRECTIONAL_HYPOTHESES_ENABLED:-false', self.text
        )
        self.assertIn(
            'intercept_directional_hypotheses_enabled:="${intercept_directional_hypotheses_enabled}"',
            self.text,
        )
        self.assertIn(
            "INTERCEPT_DIRECTIONAL_HYPOTHESES_ENABLED", self.container_text
        )
        self.assertRegex(
            self.intercept_launch_text,
            r'"intercept_directional_hypotheses_enabled",\s*default_value="false"',
        )
        self.assertIn(
            "return tuple(0.0 for _ in range(interceptor_count))",
            self.intercept_launch_text,
        )
        self.assertIn(
            "return (0.0, angle_rad, -angle_rad)",
            self.intercept_launch_text,
        )
        self.assertIn(
            "offsets[interceptor_index]",
            self.intercept_launch_text,
        )
        self.assertIn(
            '"prediction_heading_offset_rad": heading_offset',
            self.intercept_launch_text,
        )

    def test_intercept_attacker_avoidance_is_disabled_by_default(self) -> None:
        self.assertIn(
            "INTERCEPT_NONCOOPERATIVE_AVOIDANCE_ENABLED:-false", self.text
        )
        self.assertIn(
            "intercept_noncooperative_avoidance_enabled:="
            '"${intercept_noncooperative_avoidance_enabled}"',
            self.text,
        )
        self.assertIn(
            "INTERCEPT_NONCOOPERATIVE_AVOIDANCE_ENABLED", self.container_text
        )
        self.assertRegex(
            self.intercept_launch_text,
            r'"intercept_noncooperative_avoidance_enabled",\s*'
            r'default_value="false"',
        )
        self.assertIn(
            '--expect-noncooperative-avoidance '
            '"${intercept_noncooperative_avoidance_enabled}"',
            self.intercept_runtime_text,
        )

    def test_intercept_launch_configures_los_driven_radar_track_mode(self) -> None:
        self.assertRegex(
            self.intercept_launch_text,
            r'"radar_track_interval_s",\s*default_value="0.05"',
        )
        self.assertIn(
            '"track_interval_s": settings[', self.intercept_tracking_launch_text
        )
        self.assertIn(
            '"track_mode_command_topic": (', self.intercept_tracking_launch_text
        )
        self.assertIn(
            '"high_rate_velocity_correction_gain": 1.0',
            self.intercept_tracking_launch_text,
        )
        self.assertNotIn("radar_track_enter_range_m", self.intercept_launch_text)
        self.assertNotIn("radar_track_exit_range_m", self.intercept_launch_text)

    def test_navigation_nodes_use_gazebo_simulation_clock(self) -> None:
        self.assertIn(
            'obstacle_memory_overrides = {"use_sim_time": True}', self.launch_text
        )
        self.assertGreaterEqual(self.launch_text.count('"use_sim_time": True'), 7)

    def test_static_headless_disables_diagnostic_mapping(self) -> None:
        self.assertIn("ENABLE_OBSTACLE_MEMORY", self.text)
        self.assertIn('enable_obstacle_memory="false"', self.text)
        self.assertIn('enable_obstacle_memory:="${enable_obstacle_memory}"', self.text)
        self.assertIn('elif ! bool_is_true "${active_static_map}" ||', self.text)
        self.assertIn("No-static navigation requires ENABLE_OBSTACLE_MEMORY=true", self.text)
        self.assertIn(
            "No-static navigation requires LIDAR_PROFILE=3d",
            self.text,
        )
        self.assertIn("LIDAR_PROFILE", self.container_text)
        self.assertIn(
            'DeclareLaunchArgument("enable_obstacle_memory", default_value="true")',
            self.intercept_launch_text,
        )
        self.assertNotIn("ENABLE_2D_LIDAR", self.text)
        self.assertNotIn("ENABLE_2D_LIDAR", self.container_text)
        self.assertNotIn("enable_2d_lidar", self.launch_text)
        self.assertNotIn("enable_2d_lidar", self.intercept_launch_text)
        self.assertIn(
            '"persistent_memory_enabled": obstacle_memory_enabled',
            self.intercept_launch_text,
        )
        self.assertIn(
            'obstacle_memory_overrides["persistent_memory_enabled"]',
            self.launch_text,
        )
        self.assertNotIn("expected_current_lidar", self.text)

    def test_static_map_override_reaches_production_mppi(self) -> None:
        self.assertIn("production_mppi_parameters", self.launch_text)
        self.assertIn(
            'production_mppi_parameters.append(\n'
            '                {"use_static_map": static_map_override}',
            self.launch_text,
        )

    def test_static_map_is_opt_in_for_all_simulation_entry_points(self) -> None:
        self.assertIn(
            'active_static_map="$(normalize_bool "${ENABLE_STATIC_MAP:-false}")"',
            self.text,
        )
        self.assertIn(
            'ros_launch_args+=(use_static_map:="${active_static_map}")', self.text
        )
        self.assertNotIn("read_ros_bool_parameter", self.text)
        self.assertRegex(
            self.launch_text,
            r'DeclareLaunchArgument\(\s*"use_static_map",\s*'
            r'default_value="false"',
        )
        self.assertRegex(
            self.intercept_launch_text,
            r'DeclareLaunchArgument\("use_static_map", default_value="false"\)',
        )

    def test_static_map_override_reaches_mission_monitor(self) -> None:
        self.assertIn("mission_monitor_parameters", self.launch_text)
        self.assertIn(
            'mission_monitor_parameters.append(\n'
            '                {"use_static_map": static_map_override}',
            self.launch_text,
        )

    def test_production_mppi_loads_3d_world_only_in_static_mode(self) -> None:
        self.assertIn(
            "if (!config.world.use_static_map)", self.production_mppi_source_text
        )
        self.assertIn("OccupancyGrid3D::load", self.production_mppi_source_text)

    def test_runtime_lidar_visibility_follows_resolved_static_map_mode(self) -> None:
        self.assertIn('lidar_visibility_mode="no-static-3d"', self.text)
        self.assertIn('lidar_visibility_mode="static"', self.text)
        self.assertIn("configure_lidar_visibility.py", self.text)
        self.assertIn('--mode "${lidar_visibility_mode}"', self.text)
        self.assertIn('[[ "${lidar_profile}" == "none" ]]', self.text)
        self.assertIn(
            'models/${runtime_sensor_model_name}',
            self.text,
        )
        self.assertIn('if [[ "${lidar_profile}" == "3d" ]]', self.text)
        self.assertIn('"${px4_build_dir}/bin/px4" -i 0', self.text)
        self.assertIn('PX4_SIM_MODEL="${px4_model_target}"', self.text)
        self.assertIn(
            'default_point_to_point_model_name="x500_lidar_3d_0"', self.text
        )

    def test_legacy_frontier_and_soft_tabu_route_producers_are_removed(self) -> None:
        parameter = "global_lattice_frontier_blacklist_enabled"
        self.assertNotIn(parameter, self.nav_config_text)
        self.assertNotIn(parameter, self.production_mppi_source_text)
        self.assertNotIn("frontier_blacklist_enabled_", self.production_mppi_runtime_source_text)
        planning = PRODUCTION_MPPI_SOURCE.with_name(
            "production_mppi_node_route_planning.cpp"
        ).read_text()
        self.assertNotIn("Lattice3DSoftTabuEntry", planning)
        self.assertNotIn("observation_frontier", planning)
        self.assertNotIn("cycle_classification", planning)
        self.assertNotIn("no_static_cycle", planning)
        for retired_parameter in (
            "global_lattice_",
            "frontier_viability_constraints_enabled",
            "observation_frontier_stops_enabled",
            "route_replacement_progress_enabled",
        ):
            self.assertNotIn(retired_parameter, self.nav_config_text)

    def test_px4_vertical_velocity_limits_follow_active_ros_config(self) -> None:
        self.assertIn("read_ros_float_parameter()", self.text)
        self.assertIn(
            "production_mppi_node maximum_vertical_speed_mps",
            self.text,
        )
        self.assertIn(
            'param set MPC_Z_VEL_MAX_UP ${px4_max_climb_speed_mps}',
            self.text,
        )
        self.assertIn(
            'param set MPC_Z_VEL_MAX_DN ${px4_max_descent_speed_mps}',
            self.text,
        )
        self.assertIn('param show MPC_Z_VEL_MAX_DN', self.text)
        self.assertIn('param show MPC_Z_VEL_MAX_UP', self.text)

    def test_speed_profile_aligns_px4_horizontal_dynamics_with_mppi(self) -> None:
        self.assertIn(
            "production_mppi_node absolute_speed_limit_mps",
            self.text,
        )
        self.assertIn(
            "production_mppi_node cruise_speed_mps",
            self.text,
        )
        self.assertIn(
            "production_mppi_node maximum_horizontal_acceleration_mps2",
            self.text,
        )
        self.assertIn(
            "production_mppi_node maximum_control_jerk_mps3",
            self.text,
        )
        self.assertNotIn("static_cruise_speed_mps", self.text)
        self.assertNotIn("no_static_cruise_speed_mps", self.text)
        self.assertIn(
            'echo "param set MPC_XY_CRUISE ${cruise_speed}"',
            self.text,
        )
        self.assertIn(
            'echo "param set MPC_XY_VEL_MAX ${maximum_speed}"',
            self.text,
        )
        self.assertIn('"${px4_active_cruise_speed_mps}"', self.text)
        self.assertIn('"${evader_px4_cruise_speed_mps}"', self.text)
        self.assertIn(
            'px4_parameter_stream "${multi_vehicle_px4_cruise_speeds[instance]}"',
            self.text,
        )
        self.assertIn(
            "param set MPC_ACC_HOR_MAX "
            "${px4_active_max_horizontal_acceleration_mps2}",
            self.text,
        )
        self.assertIn(
            "param set MPC_ACC_HOR "
            "${px4_active_max_horizontal_acceleration_mps2}",
            self.text,
        )
        self.assertIn(
            'param set MPC_JERK_AUTO ${px4_active_maximum_jerk_mps3}',
            self.text,
        )

    def test_intercept_mode_launches_isolated_px4_instances(self) -> None:
        self.assertIn('mission_type="${MISSION_TYPE:-point_to_point}"', self.text)
        self.assertIn("intercept_scenario.py", self.intercept_runtime_text)
        self.assertIn("multi_vehicle_px4_namespaces", self.intercept_runtime_text)
        self.assertIn(
            'for instance in "${!multi_vehicle_ids[@]}"', self.text
        )
        self.assertIn('PX4_UXRCE_DDS_NS="${multi_vehicle_px4_namespaces[instance]}"', self.text)
        self.assertIn('run_px4_instance "${instance}"', self.text)
        self.assertIn(
            'validation_args+=(--expected-vehicles "${#multi_vehicle_ids[@]}")',
            self.intercept_runtime_text,
        )
        self.assertIn(
            'validation_args+=(--px4-log "${multi_vehicle_px4_logs[instance]}")',
            self.intercept_runtime_text,
        )

    def test_px4_sitl_state_is_reset_before_each_launch(self) -> None:
        self.assertIn("reset_px4_instance_state()", self.text)
        self.assertIn('"${rootfs_dir}/parameters.bson"', self.text)
        self.assertIn('"${rootfs_dir}/parameters_backup.bson"', self.text)
        self.assertIn('"${rootfs_dir}/dataman"', self.text)
        reset_index = self.text.index('reset_px4_instance_state "${instance}"')
        launch_index = self.text.index('run_px4_instance "${instance}"')
        self.assertLess(reset_index, launch_index)
        self.assertIn("reset_px4_instance_state 0", self.text)

    def test_intercept_launch_keeps_vehicle_state_isolated(self) -> None:
        for interceptor in ("interceptor_0", "interceptor_1", "interceptor_2"):
            vehicle = next(
                entry
                for entry in self.intercept_scenario["vehicles"]
                if entry["id"] == interceptor
            )
            self.assertEqual(vehicle["px4_namespace"], interceptor)
        self.assertIn('f"{prefix}/state"', self.intercept_launch_text)
        self.assertIn('scenario["vehicles"]', self.intercept_launch_text)
        self.assertEqual(
            self.intercept_scenario["vehicles"][-1]["px4_namespace"], "evader"
        )
        self.assertIn('"require_mission_start_signal": True', self.intercept_launch_text)
        self.assertIn('"rviz_drone_follow_tf_enabled": False', self.intercept_launch_text)
        diagnostics_launch = INTERCEPT_LAUNCH_FILE.with_name(
            "intercept_diagnostics_launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'plugin="drone_city_nav::InterceptSpectatorNode"', diagnostics_launch
        )

    def test_intercept_gui_observes_terminal_fall_and_headless_exits(self) -> None:
        self.assertIn(
            'multi_vehicle_shutdown_on_terminal_outcome="false"', self.text
        )
        self.assertIn(
            'multi_vehicle_shutdown_on_terminal_outcome="true"', self.text
        )
        self.assertIn(
            'shutdown_on_terminal_outcome:="${multi_vehicle_shutdown_on_terminal_outcome}"',
            self.text,
        )
        self.assertIn(
            '"shutdown_on_terminal_outcome": shutdown_on_terminal_outcome',
            self.intercept_launch_text,
        )


if __name__ == "__main__":
    unittest.main()
