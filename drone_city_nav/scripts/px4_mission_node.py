#!/usr/bin/env python3
"""PX4 Mission backend for city navigation paths."""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path as FilesystemPath
from typing import Any, Callable, Iterable

try:
    import rclpy
    from nav_msgs.msg import Path as PathMsg
    from rclpy.node import Node
    from std_msgs.msg import Bool, UInt64
except ImportError:  # Unit tests can import pure helpers without ROS installed.
    rclpy = None
    PathMsg = None
    Bool = None
    UInt64 = None
    Node = object


EARTH_RADIUS_M = 6378137.0
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1


@dataclass(frozen=True)
class Point2:
    x: float
    y: float


@dataclass(frozen=True)
class HomePosition:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float = 0.0


@dataclass(frozen=True)
class MissionBackendConfig:
    connection_url: str = "udpin:0.0.0.0:14540"
    upload_timeout_s: float = 5.0
    acceptance_radius_m: float = 1.0
    cruise_altitude_m: float = 18.0
    home_source: str = "params"
    home_latitude_deg: float = 47.397742
    home_longitude_deg: float = 8.545594
    home_altitude_m: float = 0.0
    px4_local_origin_x_m: float = 27.0
    px4_local_origin_y_m: float = 27.0
    auto_arm: bool = True
    auto_mission: bool = True
    emergency_stop_command_resend_period_s: float = 2.0
    target_system: int = 1
    target_component: int = 1
    source_system: int = 255
    source_component: int = 190


@dataclass(frozen=True)
class MissionItemInt:
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float


@dataclass(frozen=True)
class UploadResult:
    success: bool
    ack_type: str = "UNKNOWN"
    message: str = ""

    @staticmethod
    def skipped(message: str) -> "UploadResult":
        return UploadResult(False, "SKIPPED", message)

    @staticmethod
    def timeout(message: str) -> "UploadResult":
        return UploadResult(False, "TIMEOUT", message)


def map_to_px4_local(point: Point2, config: MissionBackendConfig) -> Point2:
    return Point2(
        point.x - config.px4_local_origin_x_m,
        point.y - config.px4_local_origin_y_m,
    )


def local_ne_to_global(
    home: HomePosition, north_m: float, east_m: float
) -> tuple[float, float]:
    home_lat_rad = math.radians(home.latitude_deg)
    cos_lat = math.cos(home_lat_rad)
    if abs(cos_lat) < 1.0e-9:
        raise ValueError("home latitude is too close to a pole for flat-earth conversion")

    latitude_deg = home.latitude_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude_deg = home.longitude_deg + math.degrees(east_m / (EARTH_RADIUS_M * cos_lat))
    return latitude_deg, longitude_deg


def mavlink_attr(name: str, default: int) -> int:
    try:
        from pymavlink import mavutil

        return int(getattr(mavutil.mavlink, name, default))
    except ImportError:
        return default


def build_mission_items(
    path_points: Iterable[Point2], home: HomePosition, config: MissionBackendConfig
) -> list[MissionItemInt]:
    frame = mavlink_attr("MAV_FRAME_GLOBAL_RELATIVE_ALT_INT", 6)
    command = mavlink_attr("MAV_CMD_NAV_WAYPOINT", 16)
    items: list[MissionItemInt] = []
    for seq, point in enumerate(path_points):
        local = map_to_px4_local(point, config)
        latitude_deg, longitude_deg = local_ne_to_global(home, local.x, local.y)
        items.append(
            MissionItemInt(
                seq=seq,
                frame=frame,
                command=command,
                current=1 if seq == 0 else 0,
                autocontinue=1,
                param1=0.0,
                param2=config.acceptance_radius_m,
                param3=0.0,
                param4=float("nan"),
                x=int(round(latitude_deg * 1.0e7)),
                y=int(round(longitude_deg * 1.0e7)),
                z=config.cruise_altitude_m,
            )
        )
    return items


class MissionBlackbox:
    def __init__(self, path: str, enabled: bool) -> None:
        self._enabled = enabled
        self._stream = None
        if not enabled:
            return

        blackbox_path = FilesystemPath(path)
        if blackbox_path.parent:
            blackbox_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = blackbox_path.open("a", encoding="utf-8")

    def write(self, event: dict[str, Any]) -> None:
        if self._stream is None:
            return
        self._stream.write(json.dumps(event, sort_keys=True) + "\n")
        self._stream.flush()

    def close(self) -> None:
        if self._stream is not None:
            self._stream.close()
            self._stream = None


class MavlinkMissionClient:
    def __init__(self, config: MissionBackendConfig) -> None:
        self._config = config
        self._master = None
        self._mavutil = None

    def connect(self, timeout_s: float) -> None:
        if self._master is not None:
            return

        from pymavlink import mavutil

        self._mavutil = mavutil
        self._master = mavutil.mavlink_connection(
            self._config.connection_url,
            source_system=self._config.source_system,
            source_component=self._config.source_component,
        )
        heartbeat = self._master.wait_heartbeat(timeout=timeout_s)
        if heartbeat is None:
            raise TimeoutError(
                f"timed out waiting for MAVLink heartbeat on {self._config.connection_url}"
            )

    def upload_mission(
        self, items: list[MissionItemInt], timeout_s: float
    ) -> UploadResult:
        self.connect(timeout_s)
        if not items:
            return UploadResult.skipped("empty mission")

        self._send_mission_clear_all()
        self._send_mission_count(len(items))

        sent_sequences: set[int] = set()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining_s = max(0.0, deadline - time.monotonic())
            msg = self._master.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                blocking=True,
                timeout=remaining_s,
            )
            if msg is None:
                break

            msg_type = msg.get_type()
            if msg_type == "MISSION_ACK":
                ack_result = self._upload_result_from_ack(msg)
                if len(sent_sequences) < len(items) and ack_result.success:
                    continue
                return ack_result

            seq = int(msg.seq)
            if seq < 0 or seq >= len(items):
                return UploadResult(False, "INVALID_SEQUENCE", f"requested seq {seq}")
            self._send_mission_item(items[seq])
            sent_sequences.add(seq)

        if len(sent_sequences) == len(items):
            ack = self._master.recv_match(type="MISSION_ACK", blocking=True, timeout=1.0)
            if ack is not None:
                return self._upload_result_from_ack(ack)
        return UploadResult.timeout("mission upload timed out")

    def set_auto_mission_mode(self) -> None:
        self.connect(self._config.upload_timeout_s)
        mavlink = self._mavutil.mavlink
        self._master.mav.command_long_send(
            self._config.target_system,
            self._config.target_component,
            mavlink.MAV_CMD_DO_SET_MODE,
            0,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_AUTO,
            PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
            0,
            0,
            0,
            0,
        )

    def arm(self) -> None:
        self.connect(self._config.upload_timeout_s)
        self._send_component_arm_disarm(1.0, 0.0)

    def disarm(self, force: bool = False) -> None:
        self.connect(self._config.upload_timeout_s)
        self._send_component_arm_disarm(0.0, 21196.0 if force else 0.0)

    def poll_progress(self) -> dict[str, Any] | None:
        if self._master is None:
            return None
        msg = self._master.recv_match(
            type=["MISSION_CURRENT", "MISSION_ITEM_REACHED"],
            blocking=False,
        )
        if msg is None:
            return None
        payload: dict[str, Any] = {"type": msg.get_type()}
        if hasattr(msg, "seq"):
            payload["seq"] = int(msg.seq)
        return payload

    def home_position(self, timeout_s: float) -> HomePosition | None:
        self.connect(timeout_s)
        msg = self._master.recv_match(
            type="HOME_POSITION", blocking=True, timeout=timeout_s
        )
        if msg is None:
            return None
        return HomePosition(
            latitude_deg=float(msg.latitude) / 1.0e7,
            longitude_deg=float(msg.longitude) / 1.0e7,
            altitude_m=float(msg.altitude) / 1000.0,
        )

    def _send_mission_clear_all(self) -> None:
        mavlink = self._mavutil.mavlink
        mission_type = mavlink.MAV_MISSION_TYPE_MISSION
        try:
            self._master.mav.mission_clear_all_send(
                self._config.target_system,
                self._config.target_component,
                mission_type,
            )
        except TypeError:
            self._master.mav.mission_clear_all_send(
                self._config.target_system,
                self._config.target_component,
            )

    def _send_mission_count(self, count: int) -> None:
        mavlink = self._mavutil.mavlink
        mission_type = mavlink.MAV_MISSION_TYPE_MISSION
        try:
            self._master.mav.mission_count_send(
                self._config.target_system,
                self._config.target_component,
                count,
                mission_type,
            )
        except TypeError:
            self._master.mav.mission_count_send(
                self._config.target_system,
                self._config.target_component,
                count,
            )

    def _send_mission_item(self, item: MissionItemInt) -> None:
        mavlink = self._mavutil.mavlink
        mission_type = mavlink.MAV_MISSION_TYPE_MISSION
        try:
            self._master.mav.mission_item_int_send(
                self._config.target_system,
                self._config.target_component,
                item.seq,
                item.frame,
                item.command,
                item.current,
                item.autocontinue,
                item.param1,
                item.param2,
                item.param3,
                item.param4,
                item.x,
                item.y,
                item.z,
                mission_type,
            )
        except TypeError:
            self._master.mav.mission_item_int_send(
                self._config.target_system,
                self._config.target_component,
                item.seq,
                item.frame,
                item.command,
                item.current,
                item.autocontinue,
                item.param1,
                item.param2,
                item.param3,
                item.param4,
                item.x,
                item.y,
                item.z,
            )

    def _send_component_arm_disarm(self, arm_value: float, force_value: float) -> None:
        mavlink = self._mavutil.mavlink
        self._master.mav.command_long_send(
            self._config.target_system,
            self._config.target_component,
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm_value,
            force_value,
            0,
            0,
            0,
            0,
            0,
        )

    def _upload_result_from_ack(self, msg: Any) -> UploadResult:
        mavlink = self._mavutil.mavlink
        accepted = int(msg.type) == int(mavlink.MAV_MISSION_ACCEPTED)
        ack_name = mission_ack_name(int(msg.type), mavlink)
        return UploadResult(accepted, ack_name, f"MISSION_ACK type={ack_name}")


def mission_ack_name(ack_type: int, mavlink_module: Any | None = None) -> str:
    if mavlink_module is None:
        try:
            from pymavlink import mavutil

            mavlink_module = mavutil.mavlink
        except ImportError:
            mavlink_module = None

    if mavlink_module is not None:
        for name in dir(mavlink_module):
            if name.startswith("MAV_MISSION_") and getattr(mavlink_module, name) == ack_type:
                return name
    return f"MAV_MISSION_{ack_type}"


class MissionBackendCore:
    def __init__(
        self,
        config: MissionBackendConfig,
        client: Any,
        *,
        logger: Callable[[str], None] = print,
        blackbox: MissionBlackbox | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self.client = client
        self.logger = logger
        self.blackbox = blackbox
        self.clock = clock
        self.last_uploaded_path_id: int | None = None
        self.upload_attempt = 0
        self.emergency_stop_requested = False
        self.last_emergency_disarm_s = -math.inf

    def handle_path_points(
        self, path_points: list[Point2], path_id: int, home: HomePosition
    ) -> UploadResult:
        self.logger(
            "MISSION_BACKEND path_received "
            f"path_id={path_id} waypoints={len(path_points)}"
        )
        if self.emergency_stop_requested:
            result = UploadResult.skipped("emergency stop is active")
            self._write_blackbox("upload_skipped", path_id, result, len(path_points))
            return result
        if not path_points:
            result = UploadResult.skipped("empty path")
            self.logger(f"MISSION_BACKEND empty_path_skip path_id={path_id}")
            self._write_blackbox("upload_skipped", path_id, result, 0)
            return result
        if self.last_uploaded_path_id == path_id:
            result = UploadResult.skipped("duplicate path id")
            self.logger(f"MISSION_BACKEND duplicate_path_skip path_id={path_id}")
            self._write_blackbox("upload_skipped", path_id, result, len(path_points))
            return result

        items = build_mission_items(path_points, home, self.config)
        self.upload_attempt += 1
        self.logger(
            "MISSION_BACKEND upload_started "
            f"path_id={path_id} waypoints={len(items)} attempt={self.upload_attempt}"
        )
        result = self.client.upload_mission(items, self.config.upload_timeout_s)
        self.logger(
            "MISSION_BACKEND upload_result "
            f"path_id={path_id} success={str(result.success).lower()} "
            f"ack={result.ack_type} message='{result.message}'"
        )
        self._write_blackbox("upload_result", path_id, result, len(items), items)
        if not result.success:
            return result

        self.last_uploaded_path_id = path_id
        if self.config.auto_mission:
            self.client.set_auto_mission_mode()
            self.logger(
                f"MISSION_BACKEND mode_command path_id={path_id} mode=AUTO.MISSION"
            )
        if self.config.auto_arm:
            self.client.arm()
            self.logger(f"MISSION_BACKEND arm_command path_id={path_id}")
        return result

    def handle_emergency_stop(self, requested: bool) -> None:
        if not requested:
            return
        if not self.emergency_stop_requested:
            self.emergency_stop_requested = True
            self.logger("MISSION_BACKEND emergency_stop requested=true action=disarm")
            self._write_event(
                {
                    "event": "emergency_stop_requested",
                    "time_s": self.clock(),
                    "emergency_stop_requested": True,
                }
            )
        self.maybe_send_emergency_disarm()

    def maybe_send_emergency_disarm(self) -> bool:
        if not self.emergency_stop_requested:
            return False
        now_s = self.clock()
        if (
            now_s - self.last_emergency_disarm_s
            < self.config.emergency_stop_command_resend_period_s
        ):
            return False
        self.client.disarm(force=True)
        self.last_emergency_disarm_s = now_s
        self.logger(
            "MISSION_BACKEND emergency_stop_disarm_sent "
            f"resend_period_s={self.config.emergency_stop_command_resend_period_s:.2f}"
        )
        self._write_event(
            {
                "event": "emergency_stop_disarm_sent",
                "time_s": now_s,
                "emergency_stop_requested": True,
                "emergency_stop_disarm_sent": True,
            }
        )
        return True

    def log_progress(self, progress: dict[str, Any] | None) -> None:
        if not progress:
            return
        fields = " ".join(f"{key}={value}" for key, value in sorted(progress.items()))
        self.logger(f"MISSION_BACKEND progress {fields}")
        self._write_event({"event": "progress", "time_s": self.clock(), **progress})

    def _write_blackbox(
        self,
        event: str,
        path_id: int,
        result: UploadResult,
        waypoints: int,
        items: list[MissionItemInt] | None = None,
    ) -> None:
        payload: dict[str, Any] = {
            "event": event,
            "time_s": self.clock(),
            "path_id": path_id,
            "waypoints": waypoints,
            "upload_attempt": self.upload_attempt,
            "upload_success": result.success,
            "ack_type": result.ack_type,
            "message": result.message,
            "connection_url": self.config.connection_url,
            "emergency_stop_requested": self.emergency_stop_requested,
        }
        if items:
            payload["first_item"] = asdict(items[0])
            payload["last_item"] = asdict(items[-1])
        self._write_event(payload)

    def _write_event(self, payload: dict[str, Any]) -> None:
        if self.blackbox is not None:
            self.blackbox.write(payload)


class Px4MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("px4_mission_node")

        path_topic = self.declare_parameter("path_topic", "/drone_city_nav/path").value
        path_id_topic = self.declare_parameter("path_id_topic", "/drone_city_nav/path_id").value
        emergency_stop_topic = self.declare_parameter(
            "emergency_stop_topic", "/drone_city_nav/emergency_stop"
        ).value

        config = MissionBackendConfig(
            connection_url=self.declare_parameter(
                "mission_connection_url", "udpin:0.0.0.0:14540"
            ).value,
            upload_timeout_s=float(
                self.declare_parameter("mission_upload_timeout_s", 5.0).value
            ),
            acceptance_radius_m=float(
                self.declare_parameter("mission_acceptance_radius_m", 1.0).value
            ),
            cruise_altitude_m=float(
                self.declare_parameter("mission_cruise_altitude_m", 18.0).value
            ),
            home_source=self.declare_parameter("mission_home_source", "params").value,
            home_latitude_deg=float(
                self.declare_parameter("mission_home_latitude_deg", 47.397742).value
            ),
            home_longitude_deg=float(
                self.declare_parameter("mission_home_longitude_deg", 8.545594).value
            ),
            home_altitude_m=float(
                self.declare_parameter("mission_home_altitude_m", 0.0).value
            ),
            px4_local_origin_x_m=float(
                self.declare_parameter("px4_local_origin_x_m", 27.0).value
            ),
            px4_local_origin_y_m=float(
                self.declare_parameter("px4_local_origin_y_m", 27.0).value
            ),
            auto_arm=bool(self.declare_parameter("auto_arm", True).value),
            auto_mission=bool(self.declare_parameter("auto_mission", True).value),
            emergency_stop_command_resend_period_s=float(
                self.declare_parameter("emergency_stop_command_resend_period_s", 2.0).value
            ),
            target_system=int(self.declare_parameter("target_system", 1).value),
            target_component=int(self.declare_parameter("target_component", 1).value),
            source_system=int(self.declare_parameter("source_system", 255).value),
            source_component=int(self.declare_parameter("source_component", 190).value),
        )
        blackbox_enabled = bool(
            self.declare_parameter("mission_blackbox_enabled", True).value
        )
        blackbox_path = self.declare_parameter(
            "mission_blackbox_path", "log/mission_blackbox.jsonl"
        ).value

        self._home = HomePosition(
            config.home_latitude_deg,
            config.home_longitude_deg,
            config.home_altitude_m,
        )
        self._home_fallback_warned = False
        self._latest_path_id = 0
        self._received_path_count = 0
        self._blackbox = MissionBlackbox(str(blackbox_path), blackbox_enabled)
        self._core = MissionBackendCore(
            config,
            MavlinkMissionClient(config),
            logger=lambda message: self.get_logger().info(message),
            blackbox=self._blackbox,
            clock=time.monotonic,
        )

        self.create_subscription(
            PathMsg,
            str(path_topic),
            self._on_path,
            1,
        )
        self.create_subscription(
            UInt64,
            str(path_id_topic),
            self._on_path_id,
            1,
        )
        self.create_subscription(
            Bool,
            str(emergency_stop_topic),
            self._on_emergency_stop,
            1,
        )
        self.create_timer(1.0, self._on_timer)

        self.get_logger().info(
            "Mission backend ready: "
            f"connection={config.connection_url} path_topic='{path_topic}' "
            f"path_id_topic='{path_id_topic}' emergency_stop_topic='{emergency_stop_topic}' "
            f"auto_arm={str(config.auto_arm).lower()} "
            f"auto_mission={str(config.auto_mission).lower()} "
            f"home_source={config.home_source} "
            f"home=({config.home_latitude_deg:.7f}, {config.home_longitude_deg:.7f}, "
            f"{config.home_altitude_m:.1f}) "
            f"px4_local_origin=({config.px4_local_origin_x_m:.2f}, "
            f"{config.px4_local_origin_y_m:.2f}) "
            f"emergency_stop_resend_s={config.emergency_stop_command_resend_period_s:.2f}"
        )

    def destroy_node(self) -> bool:
        self._blackbox.close()
        return super().destroy_node()

    def _on_path_id(self, msg: UInt64) -> None:
        self._latest_path_id = int(msg.data)

    def _on_path(self, msg: PathMsg) -> None:
        self._received_path_count += 1
        path_id = self._latest_path_id or self._received_path_count
        points = [
            Point2(float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        try:
            self._core.handle_path_points(points, path_id, self._resolve_home_position())
        except Exception as exc:  # Keep the node alive and diagnosable in headless logs.
            self.get_logger().error(
                f"MISSION_BACKEND upload_exception path_id={path_id} error='{exc}'"
            )

    def _on_emergency_stop(self, msg: Bool) -> None:
        try:
            self._core.handle_emergency_stop(bool(msg.data))
        except Exception as exc:
            self.get_logger().error(
                f"MISSION_BACKEND emergency_stop_exception error='{exc}'"
            )

    def _on_timer(self) -> None:
        try:
            if self._core.maybe_send_emergency_disarm():
                return
            progress = self._core.client.poll_progress()
            self._core.log_progress(progress)
        except Exception as exc:
            self.get_logger().warn(f"MISSION_BACKEND timer_exception error='{exc}'")

    def _resolve_home_position(self) -> HomePosition:
        if self._core.config.home_source != "mavlink_home":
            return self._home
        home = self._core.client.home_position(self._core.config.upload_timeout_s)
        if home is not None:
            return home
        if not self._home_fallback_warned:
            self.get_logger().warn(
                "MISSION_BACKEND home_position_fallback source=mavlink_home "
                "fallback=params"
            )
            self._home_fallback_warned = True
        return self._home


def main() -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run px4_mission_node")
    rclpy.init()
    node = Px4MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
