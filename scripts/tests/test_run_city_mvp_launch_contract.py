#!/usr/bin/env python3
"""Static launch-contract tests for the city MVP runner."""

from __future__ import annotations

import unittest
from pathlib import Path


RUNNER = Path(__file__).resolve().parents[1] / "run_city_mvp.sh"


class RunCityMvpLaunchContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.text = RUNNER.read_text(encoding="utf-8")

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

    def test_free_camera_pose_uses_gui_move_to_pose(self) -> None:
        self.assertIn("GZ_GUI_FREE_CAMERA_POSE", self.text)
        self.assertIn("free-camera-pose", self.text)
        helper_text = (RUNNER.parent / "gazebo_gui_control.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("/gui/move_to/pose", helper_text)
        self.assertIn("gz.msgs.GUICamera", helper_text)

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


if __name__ == "__main__":
    unittest.main()
