#!/usr/bin/env python3
"""Tests for Gazebo scene diagnostics capture helper."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "capture_gazebo_scene_diagnostics.py"
SPEC = importlib.util.spec_from_file_location("gazebo_scene_diagnostics", SCRIPT_PATH)
assert SPEC is not None
diagnostics = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = diagnostics
assert SPEC.loader is not None
SPEC.loader.exec_module(diagnostics)


class FakeRunner:
    def __init__(self, responses: list[diagnostics.CommandResult]) -> None:
        self.responses = responses
        self.calls: list[list[str]] = []

    def __call__(
        self, args: list[str], timeout_s: float
    ) -> diagnostics.CommandResult:
        del timeout_s
        self.calls.append(args)
        return self.responses.pop(0)


class GazeboSceneDiagnosticsTest(unittest.TestCase):
    def test_capture_writes_topic_files_and_summary(self) -> None:
        runner = FakeRunner(
            [
                diagnostics.CommandResult(
                    0,
                    (
                        'name: "x500_lidar_2d_0"\n'
                        'name: "base_link_visual"\n'
                        'name: "yellow_ground_projection_disc"\n'
                    ),
                    "",
                ),
                diagnostics.CommandResult(0, 'name: "yellow_body_plate"\n', ""),
                diagnostics.CommandResult(
                    0,
                    'follow_target { name: "x500_lidar_2d_0" }\n',
                    "",
                ),
            ]
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir)
            summary = diagnostics.capture_diagnostics(
                world="generated_city",
                target="x500_lidar_2d_0",
                output_dir=output_dir,
                topic_duration_s=0.2,
                command_timeout_s=1.0,
                runner=runner,
            )

            self.assertTrue((output_dir / "pose_info.txt").is_file())
            self.assertTrue((output_dir / "scene_info.txt").is_file())
            self.assertTrue((output_dir / "currently_tracked.txt").is_file())
            self.assertTrue((output_dir / "summary.txt").is_file())

        joined_summary = "\n".join(summary)
        self.assertIn("target_model_seen=true", joined_summary)
        self.assertIn("target_visual_seen=true", joined_summary)
        self.assertIn("yellow_visual_seen=true", joined_summary)
        self.assertIn("gui_tracking_target_seen=true", joined_summary)
        self.assertTrue(
            any("/world/generated_city/pose/info" in call for call in runner.calls)
        )
        self.assertTrue(any("/world/generated_city/scene/info" in call for call in runner.calls))
        self.assertTrue(any("/gui/currently_tracked" in call for call in runner.calls))

    def test_failed_topic_capture_is_written_as_warning(self) -> None:
        runner = FakeRunner(
            [
                diagnostics.CommandResult(124, "", "timed out"),
                diagnostics.CommandResult(0, "", ""),
                diagnostics.CommandResult(0, "", ""),
            ]
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            summary = diagnostics.capture_diagnostics(
                world="generated_city",
                target="x500_lidar_2d_0",
                output_dir=Path(temp_dir),
                topic_duration_s=0.2,
                command_timeout_s=1.0,
                runner=runner,
            )

        joined_summary = "\n".join(summary)
        self.assertIn("pose_info_status=failed", joined_summary)
        self.assertIn("WARNING: pose_info capture failed", joined_summary)


if __name__ == "__main__":
    unittest.main()
