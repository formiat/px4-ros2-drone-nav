#!/usr/bin/env python3
"""Tests for Gazebo GUI/world command helpers."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "gazebo_gui_control.py"
SPEC = importlib.util.spec_from_file_location("gazebo_gui_control", SCRIPT_PATH)
assert SPEC is not None
gui = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = gui
assert SPEC.loader is not None
SPEC.loader.exec_module(gui)


class FakeRunner:
    def __init__(self, responses: list[gui.CommandResult] | None = None) -> None:
        self.responses = responses or []
        self.calls: list[list[str]] = []

    def __call__(self, args: list[str], timeout_s: float) -> gui.CommandResult:
        del timeout_s
        self.calls.append(args)
        if self.responses:
            return self.responses.pop(0)
        return gui.CommandResult(0, "data: true\n", "")


class GazeboGuiControlTest(unittest.TestCase):
    def test_world_running_requires_three_confirmations(self) -> None:
        runner = FakeRunner()

        exit_code = gui.configure_world_running(
            world="generated_city",
            wait_s=5,
            runner=runner,
            required_confirmations=3,
        )

        self.assertEqual(exit_code, 0)
        self.assertEqual(len(runner.calls), 3)
        self.assertIn("/world/generated_city/control", runner.calls[0])
        self.assertIn("pause: false", runner.calls[0])

    def test_follow_camera_publishes_expected_commands(self) -> None:
        runner = FakeRunner(
            [
                gui.CommandResult(0, "data: true\n", ""),
                gui.CommandResult(0, "data: true\n", ""),
                gui.CommandResult(0, "", ""),
                gui.CommandResult(0, "", ""),
                gui.CommandResult(0, "data: true\n", ""),
                gui.CommandResult(0, "data: true\n", ""),
                gui.CommandResult(0, "", ""),
                gui.CommandResult(0, "", ""),
                gui.CommandResult(0, "data: true\n", ""),
                gui.CommandResult(0, "data: true\n", ""),
                gui.CommandResult(0, "", ""),
                gui.CommandResult(0, "", ""),
            ]
        )

        exit_code = gui.configure_follow_camera(
            target="x500_lidar_2d_0",
            offset_text="-12 0 6",
            wait_s=5,
            runner=runner,
            required_accepted_attempts=3,
        )

        self.assertEqual(exit_code, 0)
        flat_calls = [" ".join(call) for call in runner.calls]
        self.assertTrue(any("/gui/follow " in f"{call} " for call in flat_calls))
        self.assertTrue(
            any("/gui/follow/offset " in f"{call} " for call in flat_calls)
        )
        self.assertTrue(any("/gui/track " in f"{call} " for call in flat_calls))
        self.assertTrue(
            any("x500_lidar_2d_0" in call and "follow_target" in call for call in flat_calls)
        )

    def test_follow_camera_rejects_malformed_offset_before_calling_gz(self) -> None:
        runner = FakeRunner()

        exit_code = gui.configure_follow_camera(
            target="x500_lidar_2d_0",
            offset_text="-12 0",
            wait_s=5,
            runner=runner,
        )

        self.assertEqual(exit_code, 0)
        self.assertEqual(runner.calls, [])


if __name__ == "__main__":
    unittest.main()
