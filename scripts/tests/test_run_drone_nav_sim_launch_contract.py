#!/usr/bin/env python3
"""Static launch-contract tests for the drone navigation simulator runner."""

from __future__ import annotations

import unittest
from pathlib import Path


RUNNER = Path(__file__).resolve().parents[1] / "run_drone_nav_sim.sh"
CONTAINER_RUNNER = Path(__file__).resolve().parents[1] / "container_run.sh"
LAUNCH_FILE = (
    Path(__file__).resolve().parents[2]
    / "drone_city_nav"
    / "launch"
    / "city_nav.launch.py"
)


class RunDroneNavSimLaunchContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.text = RUNNER.read_text(encoding="utf-8")
        cls.container_text = CONTAINER_RUNNER.read_text(encoding="utf-8")
        cls.launch_text = LAUNCH_FILE.read_text(encoding="utf-8")

    def test_gazebo_gui_launch_does_not_use_gui_config_override(self) -> None:
        self.assertNotIn("--gui-config", self.text)

    def test_gazebo_gui_launch_uses_direct_gui_command(self) -> None:
        self.assertIn("gz sim -g", self.text)
        self.assertNotIn("gz sim -g > /dev/null", self.text)
        self.assertIn('gz sim -g >> "${gz_gui_log_file}" 2>&1 &', self.text)

    def test_gazebo_gui_log_is_separate_from_server_log(self) -> None:
        self.assertIn("gz_gui_log_file=", self.text)
        self.assertIn('echo "Gazebo GUI log: ${gz_gui_log_file}"', self.text)
        self.assertIn(': > "${gz_gui_log_file}"', self.text)

    def test_gazebo_scene_diagnostics_are_captured(self) -> None:
        self.assertIn("ENABLE_GZ_SCENE_DIAGNOSTICS", self.text)
        self.assertIn("capture_gazebo_scene_diagnostics", self.text)
        self.assertIn("scripts/capture_gazebo_scene_diagnostics.py", self.text)

    def test_world_unpause_uses_world_control_pause_false(self) -> None:
        self.assertIn("world-running", self.text)
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

    def test_launch_uses_offboard_flight_control_backend(self) -> None:
        self.assertIn('executable="mppi_offboard_node"', self.launch_text)
        self.assertIn("mppi_offboard,", self.launch_text)
        self.assertIn('executable="production_mppi_node"', self.launch_text)

    def test_navigation_nodes_use_gazebo_simulation_clock(self) -> None:
        self.assertIn(
            'obstacle_memory_overrides = {"use_sim_time": True}', self.launch_text
        )
        self.assertGreaterEqual(self.launch_text.count('"use_sim_time": True'), 7)

    def test_static_map_override_reaches_production_mppi(self) -> None:
        self.assertIn("production_mppi_parameters", self.launch_text)
        self.assertIn(
            'production_mppi_parameters.append(\n'
            '                {"use_static_map": static_map_override}',
            self.launch_text,
        )

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

    def test_static_mode_aligns_px4_horizontal_dynamics_with_mppi(self) -> None:
        self.assertIn("read_ros_bool_parameter()", self.text)
        self.assertIn(
            "production_mppi_node static_absolute_speed_limit_mps",
            self.text,
        )
        self.assertIn(
            "production_mppi_node static_maximum_horizontal_acceleration_mps2",
            self.text,
        )
        self.assertIn(
            "production_mppi_node static_maximum_control_jerk_mps3",
            self.text,
        )
        self.assertIn('if bool_is_true "${active_static_map}"; then', self.text)
        self.assertIn(
            'param set MPC_XY_VEL_MAX ${px4_static_max_horizontal_speed_mps}',
            self.text,
        )
        self.assertIn(
            "param set MPC_ACC_HOR_MAX "
            "${px4_static_max_horizontal_acceleration_mps2}",
            self.text,
        )
        self.assertIn(
            "param set MPC_ACC_HOR "
            "${px4_static_max_horizontal_acceleration_mps2}",
            self.text,
        )
        self.assertIn(
            'param set MPC_JERK_AUTO ${px4_static_maximum_jerk_mps3}',
            self.text,
        )


if __name__ == "__main__":
    unittest.main()
