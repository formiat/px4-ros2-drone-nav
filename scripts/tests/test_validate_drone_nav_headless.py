#!/usr/bin/env python3
"""Tests for the drone navigation headless log validator."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "validate_drone_nav_headless.py"
SPEC = importlib.util.spec_from_file_location("drone_nav_validator", SCRIPT_PATH)
assert SPEC is not None
validator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = validator
assert SPEC.loader is not None
SPEC.loader.exec_module(validator)


PX4_OK_LOG = "INFO [gz_bridge] Gazebo world is ready\n"


def make_ros_log(
    *,
    memory: bool = True,
    current_lidar: bool = True,
    static_map: bool = True,
    mission_success: bool = True,
    current_lidar_used_summary: bool = False,
) -> str:
    lines = [
        "[planner_node]: First valid PX4 local position: x=0.0 y=0.0 z=0.0",
        (
            "[planner_node]: Planner obstacle sources: "
            f"static={str(static_map).lower()} "
            f"memory={str(memory).lower()} "
            f"current_lidar={str(current_lidar).lower()}"
        ),
        "[planner_node]: Published path: waypoints=12 length=34.5m",
        (
            "[px4_offboard_node]: Sent PX4 command: "
            "VEHICLE_CMD_DO_SET_MODE param1=1 param2=6"
        ),
        (
            "[px4_offboard_node]: Sent PX4 command: "
            "VEHICLE_CMD_COMPONENT_ARM_DISARM param1=1"
        ),
        "[px4_offboard_node]: Offboard summary: armed=true offboard=true",
    ]

    if static_map:
        lines.extend(
            [
                "[planner_node]: Static city map loaded: path=generated_city.map2d",
                (
                    "[planner_node]: Planning summary: "
                    "static[enabled=true loaded=true used=true]"
                ),
            ]
        )
    else:
        lines.append("[planner_node]: Planning summary: static[enabled=false]")

    if memory:
        lines.extend(
            [
                "[obstacle_memory_node]: First lidar scan: beams=720",
                "[planner_node]: First obstacle memory grid: size=230x350",
                "[obstacle_memory_node]: Obstacle memory update: hits=42",
            ]
        )
    else:
        lines.extend(
            [
                "[obstacle_memory_node]: Obstacle memory ready: enabled=false",
                "[planner_node]: Planning summary: memory[enabled=false]",
            ]
        )

    if current_lidar:
        lines.append("[planner_node]: First planner lidar scan: beams=720")
        if current_lidar_used_summary:
            lines.append(
                "[planner_node]: Planning summary: "
                "current_lidar[enabled=true used=true]"
            )
    else:
        lines.append(
            "[planner_node]: Planning summary: "
            "current_lidar[enabled=false used=false]"
        )

    lines.append("[planner_node]: LIDAR_DEBUG snapshot=snapshot_000001")

    if mission_success:
        lines.append("[mission_monitor_node]: MISSION_RESULT success=true")
    else:
        lines.append("[mission_monitor_node]: MISSION_RESULT success=false")

    return "\n".join(lines) + "\n"


class DroneNavHeadlessValidatorTest(unittest.TestCase):
    def test_active_lidar_stable_path_reuse_passes_without_used_summary(self) -> None:
        result = validator.validate_logs(
            ros_log=make_ros_log(current_lidar_used_summary=False),
            px4_log=PX4_OK_LOG,
            options=validator.ValidationOptions(
                expected_static=True,
                expected_memory=True,
                expected_current_lidar=True,
                enable_lidar_debug=True,
                mission_check=True,
            ),
        )

        self.assertTrue(result.ok, result.errors)
        self.assertIn("OK: current lidar is available to planner", result.messages)
        self.assertIn(
            "OK: obstacle memory is available to planner", result.messages
        )

    def test_current_planner_path_log_format_passes(self) -> None:
        ros_log = make_ros_log().replace(
            "[planner_node]: Published path: waypoints=12 length=34.5m",
            (
                "[planner_node]: Published path: path_id=4 path_stamp_ns=123 "
                "reason=initial_plan waypoints=12 segments=11 "
                "length=34.5m segment_lengths[min=3.00 mean=8.00 max=15.00 "
                "lt2=0 lt5=1 lt10=5]"
            ),
        )

        result = validator.validate_logs(
            ros_log=ros_log,
            px4_log=PX4_OK_LOG,
            options=validator.ValidationOptions(
                expected_static=True,
                expected_memory=True,
                expected_current_lidar=True,
                enable_lidar_debug=True,
                mission_check=True,
            ),
        )

        self.assertTrue(result.ok, result.errors)
        self.assertIn("OK: planner publishes a path", result.messages)

    def test_current_lidar_expected_fails_without_planner_scan(self) -> None:
        ros_log = make_ros_log(current_lidar=False).replace(
            "current_lidar=false", "current_lidar=true"
        )

        result = validator.validate_logs(
            ros_log=ros_log,
            px4_log=PX4_OK_LOG,
            options=validator.ValidationOptions(
                expected_static=True,
                expected_memory=True,
                expected_current_lidar=True,
                enable_lidar_debug=True,
                mission_check=True,
            ),
        )

        self.assertFalse(result.ok)
        self.assertIn("FAIL: current lidar is available to planner", result.errors)

    def test_static_only_skips_lidar_requirements(self) -> None:
        ros_log = make_ros_log(memory=False, current_lidar=False)

        result = validator.validate_logs(
            ros_log=ros_log,
            px4_log=PX4_OK_LOG,
            options=validator.ValidationOptions(
                expected_static=True,
                expected_memory=False,
                expected_current_lidar=False,
                enable_lidar_debug=True,
                mission_check=True,
            ),
        )

        self.assertTrue(result.ok, result.errors)
        self.assertTrue(
            any(
                message.startswith("SKIP: lidar scans are received")
                for message in result.messages
            )
        )

    def test_mission_failure_is_allowed_when_flag_is_enabled(self) -> None:
        result = validator.validate_logs(
            ros_log=make_ros_log(mission_success=False),
            px4_log=PX4_OK_LOG,
            options=validator.ValidationOptions(
                expected_static=True,
                expected_memory=True,
                expected_current_lidar=True,
                enable_lidar_debug=True,
                mission_check=True,
                allow_mission_failure=True,
            ),
        )

        self.assertTrue(result.ok, result.errors)
        self.assertTrue(
            any(
                "allowed by ALLOW_MISSION_FAILURE=true" in message
                for message in result.messages
            )
        )

    def test_mission_failure_fails_without_allow_flag(self) -> None:
        result = validator.validate_logs(
            ros_log=make_ros_log(mission_success=False),
            px4_log=PX4_OK_LOG,
            options=validator.ValidationOptions(
                expected_static=True,
                expected_memory=True,
                expected_current_lidar=True,
                enable_lidar_debug=True,
                mission_check=True,
                allow_mission_failure=False,
            ),
        )

        self.assertFalse(result.ok)
        self.assertIn("FAIL: mission monitor reported failure", result.errors)

    def test_px4_critical_error_is_allowed_when_flag_is_enabled(self) -> None:
        result = validator.validate_logs(
            ros_log=make_ros_log(),
            px4_log=PX4_OK_LOG + "ERROR [commander] Found 0 compass\n",
            options=validator.ValidationOptions(
                expected_static=True,
                expected_memory=True,
                expected_current_lidar=True,
                enable_lidar_debug=True,
                mission_check=True,
                allow_mission_failure=True,
            ),
        )

        self.assertTrue(result.ok, result.errors)
        self.assertTrue(
            any(
                "PX4 log contains critical simulator/preflight errors" in message
                and "allowed by ALLOW_MISSION_FAILURE=true" in message
                for message in result.messages
            )
        )

    def test_cli_returns_success_for_valid_logs(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            ros_log = temp_path / "ros.log"
            px4_log = temp_path / "px4.log"
            ros_log.write_text(make_ros_log(), encoding="utf-8")
            px4_log.write_text(PX4_OK_LOG, encoding="utf-8")

            with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
                exit_code = validator.main(
                    [
                        "--ros-log",
                        str(ros_log),
                        "--px4-log",
                        str(px4_log),
                        "--expected-static",
                        "true",
                        "--expected-memory",
                        "true",
                        "--expected-current-lidar",
                        "true",
                        "--enable-lidar-debug",
                        "true",
                        "--mission-check",
                    ]
                )

        self.assertEqual(exit_code, 0)

    def test_cli_reports_missing_log_file_as_validation_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            missing_ros_log = temp_path / "missing_ros.log"
            px4_log = temp_path / "px4.log"
            px4_log.write_text(PX4_OK_LOG, encoding="utf-8")

            stderr = StringIO()
            with redirect_stdout(StringIO()), redirect_stderr(stderr):
                exit_code = validator.main(
                    [
                        "--ros-log",
                        str(missing_ros_log),
                        "--px4-log",
                        str(px4_log),
                    ]
                )

        self.assertEqual(exit_code, 1)
        self.assertIn("FAIL: cannot read log file", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
