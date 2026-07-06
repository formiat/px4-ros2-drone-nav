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

    def test_evasive_maneuvering_can_be_overridden_from_environment(self) -> None:
        self.assertIn("ENABLE_EVASIVE_MANEUVERING", self.text)
        self.assertIn("EVASIVE_MANEUVERING_STRAIGHT_COST_WEIGHT", self.text)
        self.assertIn('ros_launch_args+=(evasive_maneuvering:="', self.text)
        self.assertIn(
            'evasive_maneuvering_straight_cost_weight:="',
            self.text,
        )

    def test_launch_uses_offboard_flight_control_backend(self) -> None:
        self.assertIn('executable="px4_offboard_node"', self.launch_text)
        self.assertIn("px4_offboard,", self.launch_text)


if __name__ == "__main__":
    unittest.main()
