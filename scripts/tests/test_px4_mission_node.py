#!/usr/bin/env python3
"""Unit tests for the PX4 Mission backend pure logic."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[2]
    / "drone_city_nav"
    / "scripts"
    / "px4_mission_node.py"
)
SPEC = importlib.util.spec_from_file_location("px4_mission_node", SCRIPT_PATH)
assert SPEC is not None
mission_node = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = mission_node
assert SPEC.loader is not None
SPEC.loader.exec_module(mission_node)


class FakeMissionClient:
    def __init__(self, result: object | None = None) -> None:
        self.result = result or mission_node.UploadResult(
            True, "MAV_MISSION_ACCEPTED", "accepted"
        )
        self.uploaded_items: list[object] = []
        self.calls: list[tuple[str, object]] = []

    def upload_mission(self, items: list[object], timeout_s: float) -> object:
        self.uploaded_items = list(items)
        self.calls.append(("upload_mission", len(items)))
        self.calls.append(("upload_timeout_s", timeout_s))
        return self.result

    def set_auto_mission_mode(self) -> None:
        self.calls.append(("set_auto_mission_mode", None))

    def arm(self) -> None:
        self.calls.append(("arm", None))

    def disarm(self, *, force: bool = False) -> None:
        self.calls.append(("disarm", force))

    def poll_progress(self) -> None:
        self.calls.append(("poll_progress", None))
        return None


class Px4MissionNodeLogicTest(unittest.TestCase):
    def make_config(self, **overrides: object) -> object:
        values = {
            "acceptance_radius_m": 1.25,
            "cruise_altitude_m": 18.0,
            "home_latitude_deg": 47.0,
            "home_longitude_deg": 8.0,
            "px4_local_origin_x_m": 27.0,
            "px4_local_origin_y_m": 27.0,
            "upload_timeout_s": 3.5,
            "auto_arm": True,
            "auto_mission": True,
            "emergency_stop_command_resend_period_s": 2.0,
        }
        values.update(overrides)
        return mission_node.MissionBackendConfig(**values)

    def test_build_mission_items_converts_map_points_to_global_relative_items(
        self,
    ) -> None:
        config = self.make_config()
        home = mission_node.HomePosition(47.0, 8.0, 0.0)
        points = [
            mission_node.Point2(27.0, 27.0),
            mission_node.Point2(127.0, 27.0),
            mission_node.Point2(27.0, 127.0),
        ]

        items = mission_node.build_mission_items(points, home, config)

        self.assertEqual([0, 1, 2], [item.seq for item in items])
        self.assertEqual(1, items[0].current)
        self.assertEqual(0, items[1].current)
        self.assertEqual(
            mission_node.mavlink_attr("MAV_FRAME_GLOBAL_RELATIVE_ALT_INT", 6),
            items[0].frame,
        )
        self.assertEqual(
            mission_node.mavlink_attr("MAV_CMD_NAV_WAYPOINT", 16),
            items[0].command,
        )
        self.assertEqual(config.acceptance_radius_m, items[0].param2)
        self.assertEqual(config.cruise_altitude_m, items[0].z)
        self.assertEqual(round(home.latitude_deg * 1.0e7), items[0].x)
        self.assertEqual(round(home.longitude_deg * 1.0e7), items[0].y)
        self.assertGreater(items[1].x, items[0].x)
        self.assertEqual(items[1].y, items[0].y)
        self.assertEqual(items[2].x, items[0].x)
        self.assertGreater(items[2].y, items[0].y)

    def test_empty_path_is_logged_and_not_uploaded(self) -> None:
        client = FakeMissionClient()
        logs: list[str] = []
        core = mission_node.MissionBackendCore(
            self.make_config(), client, logger=logs.append
        )

        result = core.handle_path_points(
            [], 10, mission_node.HomePosition(47.0, 8.0)
        )

        self.assertFalse(result.success)
        self.assertEqual("SKIPPED", result.ack_type)
        self.assertEqual([], client.calls)
        self.assertTrue(any("empty_path_skip" in line for line in logs))

    def test_duplicate_path_id_is_not_uploaded_twice(self) -> None:
        client = FakeMissionClient()
        core = mission_node.MissionBackendCore(self.make_config(), client)
        points = [mission_node.Point2(27.0, 27.0), mission_node.Point2(28.0, 27.0)]
        home = mission_node.HomePosition(47.0, 8.0)

        first = core.handle_path_points(points, 4, home)
        second = core.handle_path_points(points, 4, home)

        self.assertTrue(first.success)
        self.assertFalse(second.success)
        self.assertEqual(1, [call[0] for call in client.calls].count("upload_mission"))

    def test_failed_upload_does_not_set_mode_or_arm(self) -> None:
        client = FakeMissionClient(
            mission_node.UploadResult(False, "TIMEOUT", "mission upload timed out")
        )
        core = mission_node.MissionBackendCore(self.make_config(), client)

        result = core.handle_path_points(
            [mission_node.Point2(27.0, 27.0)],
            5,
            mission_node.HomePosition(47.0, 8.0),
        )

        self.assertFalse(result.success)
        call_names = [call[0] for call in client.calls]
        self.assertIn("upload_mission", call_names)
        self.assertNotIn("set_auto_mission_mode", call_names)
        self.assertNotIn("arm", call_names)

    def test_successful_upload_sets_auto_mission_and_arm_in_order(self) -> None:
        client = FakeMissionClient()
        core = mission_node.MissionBackendCore(self.make_config(), client)

        result = core.handle_path_points(
            [mission_node.Point2(27.0, 27.0)],
            6,
            mission_node.HomePosition(47.0, 8.0),
        )

        self.assertTrue(result.success)
        self.assertEqual(
            ["upload_mission", "upload_timeout_s", "set_auto_mission_mode", "arm"],
            [call[0] for call in client.calls],
        )

    def test_emergency_stop_disarms_once_then_respects_resend_period(self) -> None:
        client = FakeMissionClient()
        now = [100.0]
        core = mission_node.MissionBackendCore(
            self.make_config(),
            client,
            clock=lambda: now[0],
        )

        core.handle_emergency_stop(True)
        core.handle_emergency_stop(True)
        self.assertEqual([("disarm", True)], self.disarm_calls(client))

        now[0] = 101.0
        self.assertFalse(core.maybe_send_emergency_disarm())
        self.assertEqual([("disarm", True)], self.disarm_calls(client))

        now[0] = 102.1
        self.assertTrue(core.maybe_send_emergency_disarm())
        self.assertEqual([("disarm", True), ("disarm", True)], self.disarm_calls(client))

    def test_emergency_stop_false_does_not_disarm(self) -> None:
        client = FakeMissionClient()
        core = mission_node.MissionBackendCore(self.make_config(), client)

        core.handle_emergency_stop(False)

        self.assertEqual([], self.disarm_calls(client))

    def test_emergency_stop_prevents_new_uploads(self) -> None:
        client = FakeMissionClient()
        core = mission_node.MissionBackendCore(self.make_config(), client)

        core.handle_emergency_stop(True)
        result = core.handle_path_points(
            [mission_node.Point2(27.0, 27.0)],
            7,
            mission_node.HomePosition(47.0, 8.0),
        )

        self.assertFalse(result.success)
        self.assertEqual(0, [call[0] for call in client.calls].count("upload_mission"))

    @staticmethod
    def disarm_calls(client: FakeMissionClient) -> list[tuple[str, object]]:
        return [call for call in client.calls if call[0] == "disarm"]


if __name__ == "__main__":
    unittest.main()
