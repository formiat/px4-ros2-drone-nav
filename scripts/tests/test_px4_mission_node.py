#!/usr/bin/env python3
"""Unit tests for the PX4 Mission backend pure logic."""

from __future__ import annotations

import importlib.util
import sys
import threading
import unittest
from pathlib import Path
from typing import Callable


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
    def __init__(
        self,
        result: object | None = None,
        on_upload_mission: Callable[[], None] | None = None,
        on_set_auto_mission_mode: Callable[[], None] | None = None,
    ) -> None:
        self.result = result or mission_node.UploadResult(
            True, "MAV_MISSION_ACCEPTED", "accepted"
        )
        self.on_upload_mission = on_upload_mission
        self.on_set_auto_mission_mode = on_set_auto_mission_mode
        self.uploaded_items: list[object] = []
        self.calls: list[tuple[str, object]] = []

    def set_mission_speed_parameters(self) -> None:
        self.calls.append(("set_mission_speed_parameters", None))

    def upload_mission(
        self,
        items: list[object],
        timeout_s: float,
        should_cancel: Callable[[], bool] | None = None,
    ) -> object:
        self.uploaded_items = list(items)
        self.calls.append(("upload_mission", len(items)))
        self.calls.append(("upload_timeout_s", timeout_s))
        if self.on_upload_mission is not None:
            self.on_upload_mission()
        if should_cancel is not None and should_cancel():
            return mission_node.UploadResult.skipped(
                "mission upload cancelled by emergency stop"
            )
        return self.result

    def set_auto_mission_mode(self) -> None:
        if self.on_set_auto_mission_mode is not None:
            self.on_set_auto_mission_mode()
        self.calls.append(("set_auto_mission_mode", None))

    def arm(self) -> None:
        self.calls.append(("arm", None))

    def disarm(self, *, force: bool = False) -> None:
        self.calls.append(("disarm", force))

    def poll_progress(self) -> None:
        self.calls.append(("poll_progress", None))
        return None


class Px4MissionNodeLogicTest(unittest.TestCase):
    def test_ros_path_callback_enqueues_upload_work(self) -> None:
        text = SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn("self._path_upload_queue", text)
        self.assertIn('name="px4_mission_upload_worker"', text)
        self.assertIn("mission_path_id_from_stamp(path_stamp_ns", text)
        self.assertIn("planner_path_id_latest", text)
        self.assertIn("create_px4_sensor_qos_profile()", text)
        self.assertIn("create_reliable_volatile_qos_profile(1)", text)
        self.assertIn("ReliabilityPolicy.BEST_EFFORT", text)
        self.assertIn("ReliabilityPolicy.RELIABLE", text)
        self.assertIn("self._path_sub = self.create_subscription", text)
        self.assertIn("self._path_id_sub = self.create_subscription", text)
        self.assertIn("self._emergency_stop_sub = self.create_subscription", text)
        self.assertIn(
            "self._vehicle_local_position_sub = self.create_subscription", text
        )
        self.assertIn("self._progress_timer = self.create_timer", text)
        self.assertIn("path_requires_home_resolution(request.points)", text)
        self.assertIn("_home_resolution_without_mavlink_lookup", text)
        self.assertIn("mission_cruise_speed_mps", text)
        self.assertIn("mission_max_speed_mps", text)
        self.assertIn("MPC_XY_CRUISE", text)
        self.assertIn("MPC_XY_VEL_MAX", text)
        self.assertIn("def _upload_worker_loop", text)
        self.assertIn("self._core.handle_path_points", text)

    def make_config(self, **overrides: object) -> object:
        values = {
            "acceptance_radius_m": 1.25,
            "cruise_altitude_m": 18.0,
            "mission_cruise_speed_mps": 20.0,
            "mission_max_speed_mps": 25.0,
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

    def test_mission_path_id_uses_path_stamp_instead_of_side_topic_id(self) -> None:
        self.assertEqual(
            123456789,
            mission_node.mission_path_id_from_stamp(123456789, 4),
        )
        self.assertEqual(4, mission_node.mission_path_id_from_stamp(0, 4))

    def test_empty_path_does_not_need_home_resolution(self) -> None:
        self.assertFalse(mission_node.path_requires_home_resolution([]))
        self.assertTrue(
            mission_node.path_requires_home_resolution([mission_node.Point2(1.0, 2.0)])
        )

    def test_path_segment_metrics_reports_lengths(self) -> None:
        metrics = mission_node.path_segment_metrics(
            [
                mission_node.Point2(0.0, 0.0),
                mission_node.Point2(3.0, 4.0),
                mission_node.Point2(9.0, 4.0),
            ]
        )

        self.assertEqual(3, metrics["waypoints"])
        self.assertEqual(2, metrics["segments"])
        self.assertEqual(11.0, metrics["total_length_m"])
        self.assertEqual(5.0, metrics["min_segment_len_m"])
        self.assertEqual(5.5, metrics["mean_segment_len_m"])
        self.assertEqual(6.0, metrics["max_segment_len_m"])
        self.assertEqual(0, metrics["segments_shorter_than_5m"])
        self.assertEqual(2, metrics["segments_shorter_than_10m"])

    def test_upload_uses_planner_waypoints_without_extra_intermediate_points(self) -> None:
        client = FakeMissionClient()
        logs: list[str] = []
        core = mission_node.MissionBackendCore(
            self.make_config(
                px4_local_origin_x_m=0.0,
                px4_local_origin_y_m=0.0,
            ),
            client,
            logger=logs.append,
        )

        result = core.handle_path_points(
            [mission_node.Point2(0.0, 0.0), mission_node.Point2(20.0, 0.0)],
            12,
            mission_node.HomePosition(47.0, 8.0),
        )

        self.assertTrue(result.success)
        self.assertIn(("set_mission_speed_parameters", None), client.calls)
        self.assertEqual(2, len(client.uploaded_items))
        self.assertTrue(any("speed_params_sent" in line for line in logs))
        self.assertTrue(any("mission_cruise_speed_mps=20.00" in line for line in logs))
        self.assertTrue(any("mission_max_speed_mps=25.00" in line for line in logs))
        self.assertFalse(any("mission_waypoints=" in line for line in logs))

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
            [
                "set_mission_speed_parameters",
                "upload_mission",
                "upload_timeout_s",
                "set_auto_mission_mode",
                "arm",
            ],
            [call[0] for call in client.calls],
        )

    def test_emergency_stop_during_upload_skips_mode_and_arm(self) -> None:
        core_holder: dict[str, object] = {}

        def trigger_emergency_stop() -> None:
            core_holder["core"].handle_emergency_stop(True)

        client = FakeMissionClient(on_upload_mission=trigger_emergency_stop)
        logs: list[str] = []
        core = mission_node.MissionBackendCore(
            self.make_config(),
            client,
            logger=logs.append,
        )
        core_holder["core"] = core

        result = core.handle_path_points(
            [mission_node.Point2(27.0, 27.0)],
            8,
            mission_node.HomePosition(47.0, 8.0),
        )

        self.assertFalse(result.success)
        call_names = [call[0] for call in client.calls]
        self.assertIn("upload_mission", call_names)
        self.assertIn(("disarm", True), client.calls)
        self.assertNotIn("set_auto_mission_mode", call_names)
        self.assertNotIn("arm", call_names)
        self.assertEqual("SKIPPED", result.ack_type)

    def test_emergency_stop_cannot_complete_between_mode_guard_and_send(self) -> None:
        core_holder: dict[str, object] = {}
        stop_started = threading.Event()
        stop_finished = threading.Event()
        stop_finished_before_mode_send: list[bool] = []
        stop_threads: list[threading.Thread] = []

        def trigger_emergency_stop_from_other_thread() -> None:
            core = core_holder["core"]

            def request_stop() -> None:
                stop_started.set()
                core.handle_emergency_stop(True)
                stop_finished.set()

            thread = threading.Thread(target=request_stop)
            stop_threads.append(thread)
            thread.start()
            self.assertTrue(stop_started.wait(timeout=1.0))
            stop_finished.wait(timeout=0.05)
            stop_finished_before_mode_send.append(stop_finished.is_set())

        client = FakeMissionClient(
            on_set_auto_mission_mode=trigger_emergency_stop_from_other_thread
        )
        core = mission_node.MissionBackendCore(
            self.make_config(auto_arm=False),
            client,
        )
        core_holder["core"] = core

        result = core.handle_path_points(
            [mission_node.Point2(27.0, 27.0)],
            9,
            mission_node.HomePosition(47.0, 8.0),
        )
        for thread in stop_threads:
            thread.join(timeout=1.0)

        self.assertTrue(result.success)
        self.assertEqual([False], stop_finished_before_mode_send)
        self.assertIn(("set_auto_mission_mode", None), client.calls)
        self.assertIn(("disarm", True), client.calls)
        self.assertTrue(core.is_emergency_stop_requested())

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

    def test_mavlink_upload_stops_sending_items_after_cancel(self) -> None:
        config = self.make_config()
        client = mission_node.MavlinkMissionClient(config)
        master = FakeMavlinkMaster()
        client._master = master
        client._mavutil = FakeMavutil
        items = [
            mission_node.MissionItemInt(
                seq=seq,
                frame=6,
                command=16,
                current=1 if seq == 0 else 0,
                autocontinue=1,
                param1=0.0,
                param2=1.0,
                param3=0.0,
                param4=0.0,
                x=470000000,
                y=80000000,
                z=18.0,
            )
            for seq in range(3)
        ]

        result = client.upload_mission(items, 1.0, lambda: master.cancelled)

        self.assertFalse(result.success)
        self.assertEqual("SKIPPED", result.ack_type)
        self.assertEqual([0], master.sent_item_sequences)
        self.assertEqual(1, master.clear_count)
        self.assertEqual([3], master.mission_counts)

    def test_mavlink_home_position_is_requested_and_decoded(self) -> None:
        config = self.make_config()
        client = mission_node.MavlinkMissionClient(config)
        master = FakeHomeMavlinkMaster()
        client._master = master
        client._mavutil = FakeMavutil

        home = client.home_position(1.0)

        self.assertIsNotNone(home)
        assert home is not None
        self.assertAlmostEqual(47.397742, home.latitude_deg)
        self.assertAlmostEqual(8.545594, home.longitude_deg)
        self.assertAlmostEqual(488.123, home.altitude_m)
        self.assertEqual(1, len(master.command_long_calls))
        command = master.command_long_calls[0]
        self.assertEqual(FakeMavlinkModule.MAV_CMD_REQUEST_MESSAGE, command[2])
        self.assertEqual(FakeMavlinkModule.MAVLINK_MSG_ID_HOME_POSITION, command[4])

    def test_mavlink_client_sends_px4_mission_speed_parameters(self) -> None:
        config = self.make_config(
            mission_cruise_speed_mps=20.0,
            mission_max_speed_mps=25.0,
        )
        client = mission_node.MavlinkMissionClient(config)
        master = FakeMavlinkMaster()
        client._master = master
        client._mavutil = FakeMavutil

        client.set_mission_speed_parameters()

        self.assertEqual(
            [
                (b"MPC_XY_VEL_MAX", 25.0, FakeMavlinkModule.MAV_PARAM_TYPE_REAL32),
                (b"MPC_XY_CRUISE", 20.0, FakeMavlinkModule.MAV_PARAM_TYPE_REAL32),
            ],
            [(call[2], call[3], call[4]) for call in master.param_set_calls],
        )

    def test_mavlink_client_never_sets_cruise_above_max_speed(self) -> None:
        config = self.make_config(
            mission_cruise_speed_mps=12.0,
            mission_max_speed_mps=6.0,
        )
        client = mission_node.MavlinkMissionClient(config)
        master = FakeMavlinkMaster()
        client._master = master
        client._mavutil = FakeMavutil

        client.set_mission_speed_parameters()

        self.assertEqual(b"MPC_XY_VEL_MAX", master.param_set_calls[0][2])
        self.assertEqual(12.0, master.param_set_calls[0][3])

    def test_blackbox_upload_event_contains_planned_headless_fields(self) -> None:
        client = FakeMissionClient()
        blackbox = CapturingBlackbox()
        core = mission_node.MissionBackendCore(
            self.make_config(),
            client,
            blackbox=blackbox,
        )
        home = mission_node.HomePosition(47.0, 8.0, 15.0)

        result = core.handle_path_points(
            [mission_node.Point2(27.0, 27.0)],
            11,
            home,
            mission_node.PathUploadMetadata(
                path_stamp_ns=123,
                received_path_count=2,
                latest_planner_path_id=9,
            ),
        )

        self.assertTrue(result.success)
        upload_events = [
            event for event in blackbox.events if event["event"] == "upload_result"
        ]
        self.assertEqual(1, len(upload_events))
        event = upload_events[0]
        self.assertIsInstance(event["time_ns"], int)
        self.assertEqual(
            {
                "latitude_deg": home.latitude_deg,
                "longitude_deg": home.longitude_deg,
                "altitude_m": home.altitude_m,
            },
            event["home"],
        )
        self.assertEqual(0, event["current_seq"])
        self.assertTrue(event["finished"])
        self.assertEqual(123, event["path_stamp_ns"])
        self.assertEqual(2, event["received_path_count"])
        self.assertEqual(9, event["latest_planner_path_id"])
        self.assertIn("upload_duration_s", event)
        self.assertFalse(event["reuploading_after_success"])
        self.assertEqual(20.0, event["mission_cruise_speed_mps"])
        self.assertEqual(25.0, event["mission_max_speed_mps"])
        self.assertEqual(1, event["planner_path_metrics"]["waypoints"])
        self.assertEqual(1, event["mission_path_metrics"]["waypoints"])
        self.assertEqual([{"x": 27.0, "y": 27.0}], event["planner_path_points_map"])
        self.assertEqual([{"x": 27.0, "y": 27.0}], event["mission_points_map"])
        self.assertEqual(1, len(event["mission_items"]))
        self.assertEqual(event["first_item"], event["mission_items"][0])
        self.assertEqual(event["last_item"], event["mission_items"][-1])

    def test_blackbox_upload_event_contains_home_resolution_diagnostics(self) -> None:
        client = FakeMissionClient()
        blackbox = CapturingBlackbox()
        core = mission_node.MissionBackendCore(
            self.make_config(),
            client,
            blackbox=blackbox,
        )
        configured_home = mission_node.HomePosition(47.0, 8.0, 0.0)
        resolved_home = mission_node.HomePosition(47.0001, 8.0002, 2.0)
        home_resolution = mission_node.HomePositionResolution(
            configured_home=configured_home,
            resolved_home=resolved_home,
            configured_source="mavlink_home",
            used_source="mavlink_home",
            fallback_used=False,
        )

        result = core.handle_path_points(
            [mission_node.Point2(27.0, 27.0)],
            15,
            resolved_home,
            home_resolution=home_resolution,
        )

        self.assertTrue(result.success)
        upload_events = [
            event for event in blackbox.events if event["event"] == "upload_result"
        ]
        self.assertEqual(1, len(upload_events))
        event = upload_events[0]
        self.assertEqual("mavlink_home", event["home_source_configured"])
        self.assertEqual("mavlink_home", event["home_source_used"])
        self.assertFalse(event["home_fallback_used"])
        self.assertEqual(
            {
                "latitude_deg": configured_home.latitude_deg,
                "longitude_deg": configured_home.longitude_deg,
                "altitude_m": configured_home.altitude_m,
            },
            event["configured_home"],
        )
        self.assertGreater(event["configured_to_resolved_home_delta_m"]["x"], 0.0)
        self.assertGreater(event["configured_to_resolved_home_delta_m"]["y"], 0.0)
        self.assertEqual(2.0, event["configured_to_resolved_home_delta_alt_m"])
        self.assertEqual({"x": 0.0, "y": 0.0}, event["first_waypoint_local_ne_m"])
        self.assertAlmostEqual(
            resolved_home.latitude_deg,
            event["first_waypoint_global_deg"]["latitude_deg"],
        )
        self.assertAlmostEqual(
            resolved_home.longitude_deg,
            event["first_waypoint_global_deg"]["longitude_deg"],
        )

    def test_blackbox_upload_event_contains_all_uploaded_mission_items(self) -> None:
        client = FakeMissionClient()
        blackbox = CapturingBlackbox()
        core = mission_node.MissionBackendCore(
            self.make_config(
                px4_local_origin_x_m=0.0,
                px4_local_origin_y_m=0.0,
            ),
            client,
            blackbox=blackbox,
        )

        result = core.handle_path_points(
            [
                mission_node.Point2(0.0, 0.0),
                mission_node.Point2(5.0, 0.0),
                mission_node.Point2(5.0, 5.0),
            ],
            13,
            mission_node.HomePosition(47.0, 8.0),
        )

        self.assertTrue(result.success)
        upload_events = [
            event for event in blackbox.events if event["event"] == "upload_result"
        ]
        self.assertEqual(1, len(upload_events))
        mission_items = upload_events[0]["mission_items"]
        self.assertEqual(3, len(mission_items))
        self.assertEqual([0, 1, 2], [item["seq"] for item in mission_items])

    def test_progress_event_contains_vehicle_route_diagnostics(self) -> None:
        client = FakeMissionClient()
        blackbox = CapturingBlackbox()
        core = mission_node.MissionBackendCore(
            self.make_config(
                px4_local_origin_x_m=0.0,
                px4_local_origin_y_m=0.0,
            ),
            client,
            blackbox=blackbox,
        )

        upload = core.handle_path_points(
            [mission_node.Point2(0.0, 0.0), mission_node.Point2(10.0, 0.0)],
            14,
            mission_node.HomePosition(47.0, 8.0),
        )
        self.assertTrue(upload.success)
        core.update_vehicle_telemetry(
            mission_node.VehicleTelemetry(
                position=mission_node.Point2(5.0, 1.0),
                altitude_m=18.0,
            )
        )

        core.log_progress({"type": "MISSION_CURRENT", "seq": 1})

        progress_events = [
            event for event in blackbox.events if event["event"] == "progress"
        ]
        self.assertEqual(1, len(progress_events))
        event = progress_events[0]
        self.assertTrue(event["seq_changed"])
        self.assertEqual(14, event["mission_path_id"])
        self.assertEqual({"x": 5.0, "y": 1.0}, event["vehicle_position_map"])
        self.assertEqual({"x": 10.0, "y": 0.0}, event["mission_target_map"])
        self.assertAlmostEqual(5.099, event["distance_to_mission_target_m"], places=3)
        self.assertAlmostEqual(1.0, event["cross_track_error_m"])
        self.assertAlmostEqual(0.5, event["along_track_fraction"])

    @staticmethod
    def disarm_calls(client: FakeMissionClient) -> list[tuple[str, object]]:
        return [call for call in client.calls if call[0] == "disarm"]


class CapturingBlackbox:
    def __init__(self) -> None:
        self.events: list[dict[str, object]] = []

    def write(self, event: dict[str, object]) -> None:
        self.events.append(event)


class FakeMissionRequest:
    def __init__(self, seq: int) -> None:
        self.seq = seq

    @staticmethod
    def get_type() -> str:
        return "MISSION_REQUEST_INT"


class FakeMavlinkModule:
    MAV_MISSION_ACCEPTED = 0
    MAV_MISSION_TYPE_MISSION = 0
    MAV_CMD_REQUEST_MESSAGE = 512
    MAVLINK_MSG_ID_HOME_POSITION = 242
    MAV_PARAM_TYPE_REAL32 = 9


class FakeMavutil:
    mavlink = FakeMavlinkModule


class FakeMav:
    def __init__(self, master: "FakeMavlinkMaster") -> None:
        self._master = master

    def mission_clear_all_send(self, *_args: object) -> None:
        self._master.clear_count += 1

    def mission_count_send(self, *_args: object) -> None:
        self._master.mission_counts.append(int(_args[2]))

    def mission_item_int_send(self, *_args: object) -> None:
        seq = int(_args[2])
        self._master.sent_item_sequences.append(seq)
        if seq == 0:
            self._master.cancelled = True

    def command_long_send(self, *_args: object) -> None:
        self._master.command_long_calls.append(tuple(_args))

    def param_set_send(self, *_args: object) -> None:
        self._master.param_set_calls.append(tuple(_args))


class FakeMavlinkMaster:
    def __init__(self) -> None:
        self.mav = FakeMav(self)
        self.cancelled = False
        self.clear_count = 0
        self.mission_counts: list[int] = []
        self.sent_item_sequences: list[int] = []
        self.command_long_calls: list[tuple[object, ...]] = []
        self.param_set_calls: list[tuple[object, ...]] = []
        self._next_request_seq = 0

    def recv_match(
        self, *, type: object, blocking: bool, timeout: float
    ) -> FakeMissionRequest | None:
        _ = (type, blocking, timeout)
        seq = self._next_request_seq
        self._next_request_seq += 1
        return FakeMissionRequest(seq)


class FakeHomePosition:
    latitude = 473977420
    longitude = 85455940
    altitude = 488123


class FakeHomeMavlinkMaster(FakeMavlinkMaster):
    def recv_match(
        self, *, type: object, blocking: bool, timeout: float
    ) -> FakeHomePosition | None:
        _ = (blocking, timeout)
        if type == "HOME_POSITION":
            return FakeHomePosition()
        return None


if __name__ == "__main__":
    unittest.main()
