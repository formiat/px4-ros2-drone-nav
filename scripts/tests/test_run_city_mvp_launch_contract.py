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


if __name__ == "__main__":
    unittest.main()
