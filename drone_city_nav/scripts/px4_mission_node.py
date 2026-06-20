#!/usr/bin/env python3
"""PX4 Mission backend for city navigation paths."""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path as FilesystemPath
from typing import Any, Callable, Iterable

try:
    import rclpy
    from nav_msgs.msg import Path as PathMsg
    from px4_msgs.msg import VehicleLocalPosition
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, UInt64
except ImportError:  # Unit tests can import pure helpers without ROS installed.
    rclpy = None
    PathMsg = None
    VehicleLocalPosition = None
    DurabilityPolicy = None
    HistoryPolicy = None
    Bool = None
    QoSProfile = None
    ReliabilityPolicy = None
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
    mission_cruise_speed_mps: float = 20.0
    mission_max_speed_mps: float = 25.0
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
class PathUploadMetadata:
    path_stamp_ns: int = 0
    received_path_count: int = 0
    latest_planner_path_id: int = 0


@dataclass(frozen=True)
class HomePositionResolution:
    configured_home: HomePosition
    resolved_home: HomePosition
    configured_source: str
    used_source: str
    fallback_used: bool = False


@dataclass(frozen=True)
class VehicleTelemetry:
    position: Point2
    altitude_m: float | None = None


@dataclass(frozen=True)
class PathUploadRequest:
    points: list[Point2]
    path_id: int
    metadata: PathUploadMetadata


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


def px4_local_to_map(point: Point2, config: MissionBackendConfig) -> Point2:
    return Point2(
        point.x + config.px4_local_origin_x_m,
        point.y + config.px4_local_origin_y_m,
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


def home_delta_ne_m(reference: HomePosition, other: HomePosition) -> Point2:
    reference_lat_rad = math.radians(reference.latitude_deg)
    north_m = math.radians(other.latitude_deg - reference.latitude_deg) * EARTH_RADIUS_M
    east_m = (
        math.radians(other.longitude_deg - reference.longitude_deg)
        * EARTH_RADIUS_M
        * math.cos(reference_lat_rad)
    )
    return Point2(north_m, east_m)


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


def distance_2d(first: Point2, second: Point2) -> float:
    return math.hypot(second.x - first.x, second.y - first.y)


def interpolate_2d(first: Point2, second: Point2, fraction: float) -> Point2:
    return Point2(
        first.x + (second.x - first.x) * fraction,
        first.y + (second.y - first.y) * fraction,
    )


def point_to_dict(point: Point2) -> dict[str, float]:
    return {"x": point.x, "y": point.y}


def path_points_to_dicts(points: Iterable[Point2]) -> list[dict[str, float]]:
    return [point_to_dict(point) for point in points]


def path_segment_metrics(points: Iterable[Point2]) -> dict[str, float | int | None]:
    point_list = list(points)
    segment_lengths = [
        distance_2d(first, second)
        for first, second in zip(point_list, point_list[1:])
        if distance_2d(first, second) > 1.0e-9
    ]
    if not segment_lengths:
        return {
            "waypoints": len(point_list),
            "segments": 0,
            "total_length_m": 0.0,
            "min_segment_len_m": None,
            "mean_segment_len_m": None,
            "max_segment_len_m": None,
            "segments_shorter_than_5m": 0,
            "segments_shorter_than_10m": 0,
        }

    total_length_m = sum(segment_lengths)
    return {
        "waypoints": len(point_list),
        "segments": len(segment_lengths),
        "total_length_m": total_length_m,
        "min_segment_len_m": min(segment_lengths),
        "mean_segment_len_m": total_length_m / len(segment_lengths),
        "max_segment_len_m": max(segment_lengths),
        "segments_shorter_than_5m": sum(
            1 for length in segment_lengths if length < 5.0
        ),
        "segments_shorter_than_10m": sum(
            1 for length in segment_lengths if length < 10.0
        ),
    }


def distance_point_to_segment(point: Point2, start: Point2, end: Point2) -> tuple[float, float]:
    dx = end.x - start.x
    dy = end.y - start.y
    segment_length_sq = dx * dx + dy * dy
    if segment_length_sq <= 1.0e-12:
        return distance_2d(point, start), 0.0

    raw_fraction = ((point.x - start.x) * dx + (point.y - start.y) * dy) / segment_length_sq
    fraction = min(1.0, max(0.0, raw_fraction))
    projected = interpolate_2d(start, end, fraction)
    return distance_2d(point, projected), fraction


def mission_progress_diagnostics(
    mission_points: list[Point2],
    current_seq: int | None,
    telemetry: VehicleTelemetry | None,
) -> dict[str, Any]:
    if current_seq is None or telemetry is None or not mission_points:
        return {}

    target_seq = min(max(current_seq, 0), len(mission_points) - 1)
    target = mission_points[target_seq]
    diagnostics: dict[str, Any] = {
        "vehicle_position_map": point_to_dict(telemetry.position),
        "vehicle_altitude_m": telemetry.altitude_m,
        "mission_target_seq": target_seq,
        "mission_target_map": point_to_dict(target),
        "distance_to_mission_target_m": distance_2d(telemetry.position, target),
    }
    if len(mission_points) >= 2:
        segment_start_seq = max(0, min(target_seq - 1, len(mission_points) - 2))
        segment_end_seq = segment_start_seq + 1
        segment_start = mission_points[segment_start_seq]
        segment_end = mission_points[segment_end_seq]
        cross_track_m, along_track_fraction = distance_point_to_segment(
            telemetry.position, segment_start, segment_end
        )
        diagnostics.update(
            {
                "mission_segment_start_seq": segment_start_seq,
                "mission_segment_end_seq": segment_end_seq,
                "mission_segment_start_map": point_to_dict(segment_start),
                "mission_segment_end_map": point_to_dict(segment_end),
                "cross_track_error_m": cross_track_m,
                "along_track_fraction": along_track_fraction,
            }
        )
    return diagnostics


class MissionBlackbox:
    def __init__(self, path: str, enabled: bool) -> None:
        self._enabled = enabled
        self._stream = None
        self._lock = threading.Lock()
        if not enabled:
            return

        blackbox_path = FilesystemPath(path)
        if blackbox_path.parent:
            blackbox_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = blackbox_path.open("a", encoding="utf-8")

    def write(self, event: dict[str, Any]) -> None:
        with self._lock:
            if self._stream is None:
                return
            self._stream.write(json.dumps(event, sort_keys=True) + "\n")
            self._stream.flush()

    def close(self) -> None:
        with self._lock:
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
        self,
        items: list[MissionItemInt],
        timeout_s: float,
        should_cancel: Callable[[], bool] | None = None,
    ) -> UploadResult:
        self.connect(timeout_s)
        if not items:
            return UploadResult.skipped("empty mission")
        if self._upload_cancelled(should_cancel):
            return UploadResult.skipped("mission upload cancelled by emergency stop")

        self._send_mission_clear_all()
        if self._upload_cancelled(should_cancel):
            return UploadResult.skipped("mission upload cancelled by emergency stop")
        self._send_mission_count(len(items))

        sent_sequences: set[int] = set()
        deadline = time.monotonic() + timeout_s
        receive_interval_s = 0.1
        while time.monotonic() < deadline:
            if self._upload_cancelled(should_cancel):
                return UploadResult.skipped("mission upload cancelled by emergency stop")
            remaining_s = max(0.0, deadline - time.monotonic())
            msg = self._master.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                blocking=True,
                timeout=min(receive_interval_s, remaining_s),
            )
            if msg is None:
                continue
            if self._upload_cancelled(should_cancel):
                return UploadResult.skipped("mission upload cancelled by emergency stop")

            msg_type = msg.get_type()
            if msg_type == "MISSION_ACK":
                ack_result = self._upload_result_from_ack(msg)
                if len(sent_sequences) < len(items) and ack_result.success:
                    continue
                return ack_result

            seq = int(msg.seq)
            if seq < 0 or seq >= len(items):
                return UploadResult(False, "INVALID_SEQUENCE", f"requested seq {seq}")
            if self._upload_cancelled(should_cancel):
                return UploadResult.skipped("mission upload cancelled by emergency stop")
            self._send_mission_item(items[seq])
            sent_sequences.add(seq)
            if self._upload_cancelled(should_cancel):
                return UploadResult.skipped("mission upload cancelled by emergency stop")

        return UploadResult.timeout("mission upload timed out")

    def set_mission_speed_parameters(self) -> None:
        self.connect(self._config.upload_timeout_s)
        cruise_speed_mps = max(0.0, float(self._config.mission_cruise_speed_mps))
        max_speed_mps = max(
            cruise_speed_mps, float(self._config.mission_max_speed_mps)
        )
        self._send_float_parameter("MPC_XY_VEL_MAX", max_speed_mps)
        self._send_float_parameter("MPC_XY_CRUISE", cruise_speed_mps)

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
        self._request_home_position()
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

    def _request_home_position(self) -> None:
        mavlink = self._mavutil.mavlink
        self._master.mav.command_long_send(
            self._config.target_system,
            self._config.target_component,
            getattr(mavlink, "MAV_CMD_REQUEST_MESSAGE", 512),
            0,
            getattr(mavlink, "MAVLINK_MSG_ID_HOME_POSITION", 242),
            0,
            0,
            0,
            0,
            0,
            0,
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

    def _send_float_parameter(self, name: str, value: float) -> None:
        mavlink = self._mavutil.mavlink
        param_type = getattr(mavlink, "MAV_PARAM_TYPE_REAL32", 9)
        self._master.mav.param_set_send(
            self._config.target_system,
            self._config.target_component,
            name.encode("ascii"),
            float(value),
            param_type,
        )

    def _upload_result_from_ack(self, msg: Any) -> UploadResult:
        mavlink = self._mavutil.mavlink
        accepted = int(msg.type) == int(mavlink.MAV_MISSION_ACCEPTED)
        ack_name = mission_ack_name(int(msg.type), mavlink)
        return UploadResult(accepted, ack_name, f"MISSION_ACK type={ack_name}")

    @staticmethod
    def _upload_cancelled(should_cancel: Callable[[], bool] | None) -> bool:
        return should_cancel is not None and should_cancel()


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


def mission_path_id_from_stamp(path_stamp_ns: int, fallback_path_count: int) -> int:
    if path_stamp_ns > 0:
        return path_stamp_ns
    return fallback_path_count


def stamp_to_nanoseconds(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def create_px4_sensor_qos_profile() -> Any:
    if QoSProfile is None:
        raise RuntimeError("rclpy QoSProfile is required to create PX4 sensor QoS")
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def create_reliable_volatile_qos_profile(depth: int = 1) -> Any:
    if QoSProfile is None:
        raise RuntimeError("rclpy QoSProfile is required to create ROS QoS")
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def path_requires_home_resolution(path_points: list[Point2]) -> bool:
    return bool(path_points)


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
        self._lock = threading.RLock()
        self.last_uploaded_path_id: int | None = None
        self.upload_attempt = 0
        self.emergency_stop_requested = False
        self.last_emergency_disarm_s = -math.inf
        self.current_mission_points: list[Point2] = []
        self.current_mission_path_id: int | None = None
        self.last_progress_seq: int | None = None
        self.vehicle_telemetry: VehicleTelemetry | None = None

    def handle_path_points(
        self,
        path_points: list[Point2],
        path_id: int,
        home: HomePosition,
        metadata: PathUploadMetadata | None = None,
        home_resolution: HomePositionResolution | None = None,
    ) -> UploadResult:
        self.logger(
            "MISSION_BACKEND path_received "
            f"path_id={path_id} waypoints={len(path_points)}"
        )
        if self.is_emergency_stop_requested():
            result = UploadResult.skipped("emergency stop is active")
            self._write_blackbox(
                "upload_skipped",
                path_id,
                result,
                len(path_points),
                home=home,
                path_points=path_points,
                path_metadata=metadata,
                home_resolution=home_resolution,
            )
            return result
        if not path_points:
            result = UploadResult.skipped("empty path")
            self.logger(f"MISSION_BACKEND empty_path_skip path_id={path_id}")
            self._write_blackbox(
                "upload_skipped",
                path_id,
                result,
                0,
                home=home,
                path_points=path_points,
                path_metadata=metadata,
                home_resolution=home_resolution,
            )
            return result
        with self._lock:
            duplicate_path_id = self.last_uploaded_path_id == path_id
            reuploading_after_success = self.last_uploaded_path_id is not None
        if duplicate_path_id:
            result = UploadResult.skipped("duplicate path id")
            self.logger(f"MISSION_BACKEND duplicate_path_skip path_id={path_id}")
            self._write_blackbox(
                "upload_skipped",
                path_id,
                result,
                len(path_points),
                home=home,
                path_points=path_points,
                path_metadata=metadata,
                home_resolution=home_resolution,
            )
            return result

        mission_points = list(path_points)
        items = build_mission_items(mission_points, home, self.config)
        try:
            self.client.set_mission_speed_parameters()
            self.logger(
                "MISSION_BACKEND speed_params_sent "
                f"path_id={path_id} "
                f"mission_cruise_speed_mps={self.config.mission_cruise_speed_mps:.2f} "
                f"mission_max_speed_mps={self.config.mission_max_speed_mps:.2f}"
            )
        except Exception as exc:
            self.logger(
                "MISSION_BACKEND speed_params_failed "
                f"path_id={path_id} error='{exc}'"
            )
        with self._lock:
            self.upload_attempt += 1
            upload_attempt = self.upload_attempt
        self.logger(
            "MISSION_BACKEND upload_started "
            f"path_id={path_id} waypoints={len(items)} attempt={upload_attempt}"
        )
        upload_started_s = self.clock()
        result = self.client.upload_mission(
            items, self.config.upload_timeout_s, self.is_emergency_stop_requested
        )
        upload_duration_s = self.clock() - upload_started_s
        self.logger(
            "MISSION_BACKEND upload_result "
            f"path_id={path_id} success={str(result.success).lower()} "
            f"ack={result.ack_type} upload_duration_s={upload_duration_s:.3f} "
            f"message='{result.message}'"
        )
        self._write_blackbox(
            "upload_result",
            path_id,
            result,
            len(items),
            items,
            home=home,
            current_seq=len(items) - 1 if result.success and items else None,
            finished=result.success,
            path_points=path_points,
            mission_points=mission_points,
            path_metadata=metadata,
            home_resolution=home_resolution,
            upload_duration_s=upload_duration_s,
            reuploading_after_success=reuploading_after_success,
        )
        if not result.success:
            return result

        if self.is_emergency_stop_requested():
            skipped = UploadResult.skipped("emergency stop became active after upload")
            self.logger(
                "MISSION_BACKEND upload_post_stop_skip "
                f"path_id={path_id} action=skip_mode_arm"
            )
            self._write_blackbox(
                "upload_skipped",
                path_id,
                skipped,
                len(items),
                items,
                home=home,
                path_points=path_points,
                mission_points=mission_points,
                path_metadata=metadata,
                home_resolution=home_resolution,
            )
            return skipped

        with self._lock:
            self.current_mission_points = list(mission_points)
            self.current_mission_path_id = path_id
            self.last_progress_seq = None
            self.last_uploaded_path_id = path_id
        if self.config.auto_mission:
            if not self._run_command_unless_emergency(
                "mode_command", path_id, self.client.set_auto_mission_mode
            ):
                skipped = UploadResult.skipped(
                    "emergency stop became active before mode command"
                )
                self._write_blackbox(
                    "upload_skipped",
                    path_id,
                    skipped,
                    len(items),
                    items,
                    home=home,
                    path_points=path_points,
                    mission_points=mission_points,
                    path_metadata=metadata,
                    home_resolution=home_resolution,
                )
                return skipped
            self.logger(
                f"MISSION_BACKEND mode_command path_id={path_id} mode=AUTO.MISSION"
            )
        if self.config.auto_arm:
            if not self._run_command_unless_emergency(
                "arm_command", path_id, self.client.arm
            ):
                skipped = UploadResult.skipped(
                    "emergency stop became active before arm command"
                )
                self._write_blackbox(
                    "upload_skipped",
                    path_id,
                    skipped,
                    len(items),
                    items,
                    home=home,
                    path_points=path_points,
                    mission_points=mission_points,
                    path_metadata=metadata,
                    home_resolution=home_resolution,
                )
                return skipped
            self.logger(f"MISSION_BACKEND arm_command path_id={path_id}")
        return result

    def handle_emergency_stop(self, requested: bool) -> None:
        if not requested:
            return
        first_request = False
        with self._lock:
            if not self.emergency_stop_requested:
                self.emergency_stop_requested = True
                first_request = True
        if first_request:
            self.logger("MISSION_BACKEND emergency_stop requested=true action=disarm")
            self._write_event(
                {
                    "event": "emergency_stop_requested",
                    "time_s": self.clock(),
                    "time_ns": time.time_ns(),
                    "emergency_stop_requested": True,
                }
            )
        self.maybe_send_emergency_disarm()

    def maybe_send_emergency_disarm(self) -> bool:
        now_s = self.clock()
        with self._lock:
            if not self.emergency_stop_requested:
                return False
            if (
                now_s - self.last_emergency_disarm_s
                < self.config.emergency_stop_command_resend_period_s
            ):
                return False
            self.last_emergency_disarm_s = now_s
        self.client.disarm(force=True)
        self.logger(
            "MISSION_BACKEND emergency_stop_disarm_sent "
            f"resend_period_s={self.config.emergency_stop_command_resend_period_s:.2f}"
        )
        self._write_event(
            {
                "event": "emergency_stop_disarm_sent",
                "time_s": now_s,
                "time_ns": time.time_ns(),
                "emergency_stop_requested": True,
                "emergency_stop_disarm_sent": True,
            }
        )
        return True

    def is_emergency_stop_requested(self) -> bool:
        with self._lock:
            return self.emergency_stop_requested

    def update_vehicle_telemetry(self, telemetry: VehicleTelemetry) -> None:
        with self._lock:
            self.vehicle_telemetry = telemetry

    def log_progress(self, progress: dict[str, Any] | None) -> None:
        if not progress:
            return
        current_seq = progress.get("seq")
        current_seq_int = int(current_seq) if current_seq is not None else None
        with self._lock:
            mission_points = list(self.current_mission_points)
            mission_path_id = self.current_mission_path_id
            telemetry = self.vehicle_telemetry
            previous_seq = self.last_progress_seq
            seq_changed = current_seq_int is not None and current_seq_int != previous_seq
            if current_seq_int is not None:
                self.last_progress_seq = current_seq_int

        diagnostics = mission_progress_diagnostics(
            mission_points, current_seq_int, telemetry
        )
        fields = " ".join(f"{key}={value}" for key, value in sorted(progress.items()))
        if diagnostics:
            fields += (
                f" mission_path_id={mission_path_id} "
                f"distance_to_target_m={diagnostics['distance_to_mission_target_m']:.2f}"
            )
            if "cross_track_error_m" in diagnostics:
                fields += f" cross_track_error_m={diagnostics['cross_track_error_m']:.2f}"
        if seq_changed:
            self.logger(
                "MISSION_BACKEND seq_change "
                f"previous_seq={previous_seq} current_seq={current_seq_int} {fields}"
            )
        else:
            self.logger(f"MISSION_BACKEND progress {fields}")

        payload: dict[str, Any] = {
            "event": "progress",
            "time_s": self.clock(),
            "time_ns": time.time_ns(),
            "current_seq": current_seq,
            "previous_seq": previous_seq,
            "seq_changed": seq_changed,
            "mission_path_id": mission_path_id,
            "finished": progress.get("type") == "MISSION_ITEM_REACHED",
            **progress,
        }
        payload.update(diagnostics)
        self._write_event(
            payload
        )

    def _write_blackbox(
        self,
        event: str,
        path_id: int,
        result: UploadResult,
        waypoints: int,
        items: list[MissionItemInt] | None = None,
        *,
        home: HomePosition | None = None,
        current_seq: int | None = None,
        finished: bool | None = None,
        path_points: list[Point2] | None = None,
        mission_points: list[Point2] | None = None,
        path_metadata: PathUploadMetadata | None = None,
        home_resolution: HomePositionResolution | None = None,
        upload_duration_s: float | None = None,
        reuploading_after_success: bool | None = None,
    ) -> None:
        payload: dict[str, Any] = {
            "event": event,
            "time_s": self.clock(),
            "time_ns": time.time_ns(),
            "path_id": path_id,
            "waypoints": waypoints,
            "upload_attempt": self.upload_attempt,
            "upload_success": result.success,
            "ack_type": result.ack_type,
            "message": result.message,
            "current_seq": current_seq,
            "finished": finished if finished is not None else result.success,
            "home": asdict(home) if home is not None else None,
            "connection_url": self.config.connection_url,
            "emergency_stop_requested": self.is_emergency_stop_requested(),
            "mission_cruise_speed_mps": self.config.mission_cruise_speed_mps,
            "mission_max_speed_mps": self.config.mission_max_speed_mps,
        }
        if home_resolution is not None:
            delta = home_delta_ne_m(
                home_resolution.configured_home, home_resolution.resolved_home
            )
            payload.update(
                {
                    "home_source_configured": home_resolution.configured_source,
                    "home_source_used": home_resolution.used_source,
                    "home_fallback_used": home_resolution.fallback_used,
                    "configured_home": asdict(home_resolution.configured_home),
                    "configured_to_resolved_home_delta_m": point_to_dict(delta),
                    "configured_to_resolved_home_delta_alt_m": (
                        home_resolution.resolved_home.altitude_m
                        - home_resolution.configured_home.altitude_m
                    ),
                }
            )
        if path_metadata is not None:
            payload.update(asdict(path_metadata))
            payload["path_metadata"] = asdict(path_metadata)
        if upload_duration_s is not None:
            payload["upload_duration_s"] = upload_duration_s
        if reuploading_after_success is not None:
            payload["reuploading_after_success"] = reuploading_after_success
        if path_points is not None:
            payload["planner_path_points_map"] = path_points_to_dicts(path_points)
            payload["planner_path_metrics"] = path_segment_metrics(path_points)
            if path_points and home is not None:
                first_local = map_to_px4_local(path_points[0], self.config)
                first_lat_deg, first_lon_deg = local_ne_to_global(
                    home, first_local.x, first_local.y
                )
                payload["first_waypoint_local_ne_m"] = point_to_dict(first_local)
                payload["first_waypoint_global_deg"] = {
                    "latitude_deg": first_lat_deg,
                    "longitude_deg": first_lon_deg,
                }
        if mission_points is not None:
            payload["mission_points_map"] = path_points_to_dicts(mission_points)
            payload["mission_path_metrics"] = path_segment_metrics(mission_points)
        if items:
            payload["first_item"] = asdict(items[0])
            payload["last_item"] = asdict(items[-1])
            payload["mission_items"] = [asdict(item) for item in items]
        self._write_event(payload)

    def _run_command_unless_emergency(
        self, command_name: str, path_id: int, command: Callable[[], None]
    ) -> bool:
        with self._lock:
            if self.emergency_stop_requested:
                self.logger(
                    "MISSION_BACKEND "
                    f"{command_name}_skip path_id={path_id} reason=emergency_stop"
                )
                return False
            command()
            return True

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
        px4_local_position_topic = self.declare_parameter(
            "px4_local_position_topic", "/fmu/out/vehicle_local_position_v1"
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
            mission_cruise_speed_mps=float(
                self.declare_parameter("mission_cruise_speed_mps", 20.0).value
            ),
            mission_max_speed_mps=float(
                self.declare_parameter("mission_max_speed_mps", 25.0).value
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
        self._latest_planner_path_id = 0
        self._received_path_count = 0
        self._blackbox = MissionBlackbox(str(blackbox_path), blackbox_enabled)
        self._worker_join_timeout_s = max(2.0, config.upload_timeout_s + 1.0)
        self._core = MissionBackendCore(
            config,
            MavlinkMissionClient(config),
            logger=lambda message: self.get_logger().info(message),
            blackbox=self._blackbox,
            clock=time.monotonic,
        )
        self._worker_stop = threading.Event()
        self._path_upload_queue: queue.Queue[PathUploadRequest | None] = queue.Queue()
        self._upload_worker = threading.Thread(
            target=self._upload_worker_loop,
            name="px4_mission_upload_worker",
            daemon=True,
        )
        self._upload_worker.start()

        path_qos = create_reliable_volatile_qos_profile(1)
        self._path_sub = self.create_subscription(
            PathMsg,
            str(path_topic),
            self._on_path,
            path_qos,
        )
        self._path_id_sub = self.create_subscription(
            UInt64,
            str(path_id_topic),
            self._on_path_id,
            path_qos,
        )
        self._emergency_stop_sub = self.create_subscription(
            Bool,
            str(emergency_stop_topic),
            self._on_emergency_stop,
            path_qos,
        )
        self._vehicle_local_position_sub = None
        if VehicleLocalPosition is not None:
            self._vehicle_local_position_sub = self.create_subscription(
                VehicleLocalPosition,
                str(px4_local_position_topic),
                self._on_vehicle_local_position,
                create_px4_sensor_qos_profile(),
            )
        else:
            self.get_logger().warn(
                "MISSION_BACKEND vehicle_local_position_unavailable "
                "reason=px4_msgs_import_failed"
            )
        self._progress_timer = self.create_timer(1.0, self._on_timer)

        self.get_logger().info(
            "Mission backend ready: "
            f"connection={config.connection_url} path_topic='{path_topic}' "
            f"path_id_topic='{path_id_topic}' emergency_stop_topic='{emergency_stop_topic}' "
            f"px4_local_position_topic='{px4_local_position_topic}' "
            f"auto_arm={str(config.auto_arm).lower()} "
            f"auto_mission={str(config.auto_mission).lower()} "
            f"home_source={config.home_source} "
            f"home=({config.home_latitude_deg:.7f}, {config.home_longitude_deg:.7f}, "
            f"{config.home_altitude_m:.1f}) "
            f"mission_cruise_speed_mps={config.mission_cruise_speed_mps:.2f} "
            f"mission_max_speed_mps={config.mission_max_speed_mps:.2f} "
            f"px4_local_origin=({config.px4_local_origin_x_m:.2f}, "
            f"{config.px4_local_origin_y_m:.2f}) "
            f"emergency_stop_resend_s={config.emergency_stop_command_resend_period_s:.2f}"
        )

    def destroy_node(self) -> bool:
        self._worker_stop.set()
        self._path_upload_queue.put(None)
        self._upload_worker.join(timeout=self._worker_join_timeout_s)
        if self._upload_worker.is_alive():
            self.get_logger().warn(
                "MISSION_BACKEND worker_shutdown_timeout "
                f"timeout_s={self._worker_join_timeout_s:.1f}"
            )
        self._blackbox.close()
        return super().destroy_node()

    def _on_path_id(self, msg: UInt64) -> None:
        self._latest_planner_path_id = int(msg.data)

    def _on_path(self, msg: PathMsg) -> None:
        self._received_path_count += 1
        path_stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        path_id = mission_path_id_from_stamp(path_stamp_ns, self._received_path_count)
        points = [
            Point2(float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        self._path_upload_queue.put(
            PathUploadRequest(
                points=points,
                path_id=path_id,
                metadata=PathUploadMetadata(
                    path_stamp_ns=path_stamp_ns,
                    received_path_count=self._received_path_count,
                    latest_planner_path_id=self._latest_planner_path_id,
                ),
            )
        )
        self.get_logger().info(
            "MISSION_BACKEND path_enqueued "
            f"path_id={path_id} path_stamp_ns={path_stamp_ns} "
            f"planner_path_id_latest={self._latest_planner_path_id} "
            f"waypoints={len(points)}"
        )

    def _upload_worker_loop(self) -> None:
        while not self._worker_stop.is_set():
            try:
                request = self._path_upload_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if request is None:
                self._path_upload_queue.task_done()
                break

            try:
                if path_requires_home_resolution(request.points):
                    home_resolution = self._resolve_home_position()
                    self._log_home_resolution(home_resolution)
                else:
                    home_resolution = self._home_resolution_without_mavlink_lookup()
                self._core.handle_path_points(
                    request.points,
                    request.path_id,
                    home_resolution.resolved_home,
                    request.metadata,
                    home_resolution,
                )
            except Exception as exc:
                self.get_logger().error(
                    "MISSION_BACKEND upload_exception "
                    f"path_id={request.path_id} error='{exc}'"
                )
            finally:
                self._path_upload_queue.task_done()

    def _home_resolution_without_mavlink_lookup(self) -> HomePositionResolution:
        return HomePositionResolution(
            configured_home=self._home,
            resolved_home=self._home,
            configured_source=self._core.config.home_source,
            used_source="not_required",
            fallback_used=False,
        )

    def _on_emergency_stop(self, msg: Bool) -> None:
        try:
            self._core.handle_emergency_stop(bool(msg.data))
        except Exception as exc:
            self.get_logger().error(
                f"MISSION_BACKEND emergency_stop_exception error='{exc}'"
            )

    def _on_vehicle_local_position(self, msg: Any) -> None:
        if hasattr(msg, "xy_valid") and not bool(msg.xy_valid):
            return
        position = px4_local_to_map(
            Point2(float(msg.x), float(msg.y)),
            self._core.config,
        )
        altitude_m: float | None = None
        if hasattr(msg, "z") and math.isfinite(float(msg.z)):
            altitude_m = -float(msg.z)
        self._core.update_vehicle_telemetry(
            VehicleTelemetry(position=position, altitude_m=altitude_m)
        )

    def _on_timer(self) -> None:
        try:
            if self._core.maybe_send_emergency_disarm():
                return
            progress = self._core.client.poll_progress()
            self._core.log_progress(progress)
        except Exception as exc:
            self.get_logger().warn(f"MISSION_BACKEND timer_exception error='{exc}'")

    def _resolve_home_position(self) -> HomePositionResolution:
        if self._core.config.home_source != "mavlink_home":
            return HomePositionResolution(
                configured_home=self._home,
                resolved_home=self._home,
                configured_source=self._core.config.home_source,
                used_source="params",
                fallback_used=False,
            )
        home = self._core.client.home_position(self._core.config.upload_timeout_s)
        if home is not None:
            return HomePositionResolution(
                configured_home=self._home,
                resolved_home=home,
                configured_source="mavlink_home",
                used_source="mavlink_home",
                fallback_used=False,
            )
        if not self._home_fallback_warned:
            self.get_logger().warn(
                "MISSION_BACKEND home_position_fallback source=mavlink_home "
                "fallback=params"
            )
            self._home_fallback_warned = True
        return HomePositionResolution(
            configured_home=self._home,
            resolved_home=self._home,
            configured_source="mavlink_home",
            used_source="params",
            fallback_used=True,
        )

    def _log_home_resolution(self, resolution: HomePositionResolution) -> None:
        delta = home_delta_ne_m(resolution.configured_home, resolution.resolved_home)
        self.get_logger().info(
            "MISSION_BACKEND home_position_resolved "
            f"configured_source={resolution.configured_source} "
            f"used_source={resolution.used_source} "
            f"fallback_used={str(resolution.fallback_used).lower()} "
            f"resolved_home=({resolution.resolved_home.latitude_deg:.7f}, "
            f"{resolution.resolved_home.longitude_deg:.7f}, "
            f"{resolution.resolved_home.altitude_m:.1f}) "
            f"configured_home=({resolution.configured_home.latitude_deg:.7f}, "
            f"{resolution.configured_home.longitude_deg:.7f}, "
            f"{resolution.configured_home.altitude_m:.1f}) "
            f"configured_to_resolved_delta_ne=({delta.x:.2f}, {delta.y:.2f})"
        )


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
