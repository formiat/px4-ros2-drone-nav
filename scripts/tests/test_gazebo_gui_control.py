#!/usr/bin/env python3
"""Tests for Gazebo GUI/world command helpers."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from unittest import mock


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
                gui.CommandResult(
                    0,
                    'header {}\nfollow_target { name: "x500_lidar_2d_0" }\n',
                    "",
                ),
            ]
        )

        exit_code = gui.configure_follow_camera(
            target="x500_lidar_2d_0",
            offset_text="-12 0 6",
            wait_s=5,
            runner=runner,
            required_accepted_attempts=3,
            tracking_confirmation_attempts=1,
        )

        self.assertEqual(exit_code, 0)
        flat_calls = [" ".join(call) for call in runner.calls]
        self.assertTrue(any("/gui/follow " in f"{call} " for call in flat_calls))
        self.assertTrue(
            any("/gui/follow/offset " in f"{call} " for call in flat_calls)
        )
        self.assertTrue(any("/gui/track " in f"{call} " for call in flat_calls))
        self.assertTrue(
            any(
                "x500_lidar_2d_0" in call and "follow_target" in call
                for call in flat_calls
            )
        )
        self.assertTrue(any("/gui/currently_tracked" in call for call in flat_calls))

    def test_follow_camera_logs_unavailable_tracking_state(self) -> None:
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
        stdout = StringIO()

        with redirect_stdout(stdout):
            exit_code = gui.configure_follow_camera(
                target="x500_lidar_2d_0",
                offset_text="-12 0 6",
                wait_s=5,
                runner=runner,
                required_accepted_attempts=3,
                tracking_confirmation_attempts=1,
            )

        self.assertEqual(exit_code, 0)
        self.assertIn(
            "state confirmation is unavailable",
            stdout.getvalue(),
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

    def test_free_camera_pose_calls_move_to_pose_service(self) -> None:
        runner = FakeRunner()

        exit_code = gui.configure_free_camera_pose(
            pose_text="-69 -27 6 0 0.45 0",
            wait_s=5,
            runner=runner,
        )

        self.assertEqual(exit_code, 0)
        self.assertEqual(len(runner.calls), 1)
        flat_call = " ".join(runner.calls[0])
        self.assertIn("/gui/move_to/pose", flat_call)
        self.assertIn("gz.msgs.GUICamera", flat_call)
        self.assertIn("position: { x: -69 y: -27 z: 6 }", flat_call)
        self.assertIn("orientation: { x: 0 y: 0.223106", flat_call)
        self.assertIn("w: 0.974794", flat_call)

    def test_free_camera_pose_rejects_malformed_pose_before_calling_gz(self) -> None:
        runner = FakeRunner()

        exit_code = gui.configure_free_camera_pose(
            pose_text="-69 -27 6",
            wait_s=5,
            runner=runner,
        )

        self.assertEqual(exit_code, 0)
        self.assertEqual(runner.calls, [])

    def test_default_runner_converts_timeout_to_retryable_result(self) -> None:
        timeout = gui.subprocess.TimeoutExpired(
            cmd=["gz", "service"],
            timeout=2.0,
            output=b"partial stdout",
            stderr=b"partial stderr",
        )
        with mock.patch.object(gui.subprocess, "run", side_effect=timeout):
            result = gui.default_runner(["service"], 2.0)

        self.assertEqual(result.returncode, 124)
        self.assertIn("partial stdout", result.stdout)
        self.assertIn("partial stderr", result.stderr)
        self.assertIn("timed out", result.stderr)


if __name__ == "__main__":
    unittest.main()
