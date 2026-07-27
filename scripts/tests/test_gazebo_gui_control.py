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
                gui.CommandResult(
                    0,
                    'model {\n name: "x500_lidar_2d_0"\n id: 245\n}\n',
                    "",
                ),
                gui.CommandResult(0, "", ""),
                gui.CommandResult(
                    0,
                    'header {}\nfollow_target { name: "x500_lidar_2d_0" }\n',
                    "",
                ),
            ]
        )

        exit_code = gui.configure_follow_camera(
            world="generated_city",
            target="x500_lidar_2d_0",
            offset_text="-12 0 6",
            wait_s=5,
            runner=runner,
            tracking_confirmation_attempts=1,
            required_confirmations=1,
        )

        self.assertEqual(exit_code, 0)
        flat_calls = [" ".join(call) for call in runner.calls]
        self.assertFalse(any("/gui/follow " in f"{call} " for call in flat_calls))
        track_call = next(call for call in flat_calls if "/gui/track " in f"{call} ")
        self.assertIn("id: 245", track_call)
        self.assertIn('name: "x500_lidar_2d_0"', track_call)
        self.assertTrue(any("/gui/currently_tracked" in call for call in flat_calls))

    def test_follow_camera_retries_until_wait_expires_without_confirmation(
        self,
    ) -> None:
        runner = FakeRunner(
            sum(
                (
                    [
                        gui.CommandResult(
                            0,
                            'model {\n name: "drone"\n id: 245\n}\n',
                            "",
                        ),
                        gui.CommandResult(0, "", ""),
                        gui.CommandResult(0, "", ""),
                    ]
                    for _ in range(3)
                ),
                [],
            )
        )
        stdout = StringIO()

        with redirect_stdout(stdout), mock.patch.object(gui.time, "sleep"):
            exit_code = gui.configure_follow_camera(
                world="generated_city",
                target="drone",
                offset_text="-12 0 6",
                wait_s=3,
                runner=runner,
                tracking_confirmation_attempts=1,
            )

        self.assertEqual(exit_code, 0)
        self.assertIn(
            "published 3 times but state was not confirmed",
            stdout.getvalue(),
        )

    def test_follow_camera_requires_consecutive_confirmations(self) -> None:
        target_seen = gui.CommandResult(
            0,
            'follow_target { name: "x500_lidar_2d_0" }\n',
            "",
        )
        runner = FakeRunner(
            sum(
                (
                    [
                        gui.CommandResult(
                            0,
                            'model {\n name: "x500_lidar_2d_0"\n id: 245\n}\n',
                            "",
                        ),
                        gui.CommandResult(0, "", ""),
                        target_seen,
                    ]
                    for _ in range(3)
                ),
                [],
            )
        )

        with mock.patch.object(gui.time, "sleep"):
            exit_code = gui.configure_follow_camera(
                world="generated_city",
                target="x500_lidar_2d_0",
                offset_text="-12 0 6",
                wait_s=3,
                runner=runner,
                tracking_confirmation_attempts=1,
            )

        self.assertEqual(exit_code, 0)
        track_calls = [
            call for call in runner.calls if "/gui/track" in call
        ]
        self.assertEqual(len(track_calls), 3)

    def test_follow_camera_rejects_malformed_offset_before_calling_gz(self) -> None:
        runner = FakeRunner()

        exit_code = gui.configure_follow_camera(
            world="generated_city",
            target="x500_lidar_2d_0",
            offset_text="-12 0",
            wait_s=5,
            runner=runner,
        )

        self.assertEqual(exit_code, 0)
        self.assertEqual(runner.calls, [])

    def test_wait_for_scene_entity_requires_named_model(self) -> None:
        runner = FakeRunner(
            [
                gui.CommandResult(0, 'model { name: "ground" id: 4 }\n', ""),
                gui.CommandResult(
                    0,
                    'model {\n name: "x500_lidar_2d_0"\n id: 245\n}\n',
                    "",
                ),
            ]
        )

        with mock.patch.object(gui.time, "sleep"):
            exit_code = gui.wait_for_scene_entity(
                world="generated_city",
                target="x500_lidar_2d_0",
                wait_s=2,
                runner=runner,
            )

        self.assertEqual(exit_code, 0)
        self.assertEqual(len(runner.calls), 2)
        self.assertIn("/world/generated_city/scene/info", runner.calls[0])

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
