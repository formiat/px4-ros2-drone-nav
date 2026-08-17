#!/usr/bin/env python3
"""Write the Gazebo GUI user-camera pose to JSONL at a fixed cadence."""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import shlex
import signal
import subprocess
import sys
import threading
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import TextIO


DEFAULT_TOPIC = "/gui/camera/pose"
DEFAULT_INTERVAL_S = 1.0


class CameraLoggerError(ValueError):
    """Raised when camera logger configuration or a camera message is invalid."""


def parse_interval_s(value: str) -> float:
    try:
        interval_s = float(value)
    except ValueError as error:
        raise CameraLoggerError("camera log interval must be a number") from error
    if not math.isfinite(interval_s) or interval_s <= 0.0:
        raise CameraLoggerError("camera log interval must be finite and positive")
    return interval_s


def _finite_coordinate(value: object, label: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise CameraLoggerError(f"camera {label} must be numeric")
    coordinate = float(value)
    if not math.isfinite(coordinate):
        raise CameraLoggerError(f"camera {label} must be finite")
    return coordinate


def _vector(value: object, label: str) -> tuple[float, float, float]:
    if not isinstance(value, dict):
        raise CameraLoggerError(f"camera {label} must be an object")
    return tuple(_finite_coordinate(value.get(axis), f"{label}.{axis}") for axis in "xyz")


def _quaternion(value: object) -> tuple[float, float, float, float]:
    if not isinstance(value, dict):
        raise CameraLoggerError("camera orientation must be an object")
    x, y, z = _vector(value, "orientation")
    w = _finite_coordinate(value.get("w"), "orientation.w")
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if length <= sys.float_info.epsilon:
        raise CameraLoggerError("camera orientation must be non-zero")
    return (x / length, y / length, z / length, w / length)


def parse_camera_pose(payload: object) -> dict[str, dict[str, float]]:
    """Convert a gz.msgs.Pose JSON object into position, orientation, and view direction."""
    if not isinstance(payload, dict):
        raise CameraLoggerError("camera message must be an object")
    pose = payload.get("pose", payload)
    if not isinstance(pose, dict):
        raise CameraLoggerError("camera pose must be an object")
    position = _vector(pose.get("position"), "position")
    orientation = _quaternion(pose.get("orientation"))
    direction = camera_forward_direction(*orientation)
    return {
        "position_m": dict(zip("xyz", position, strict=True)),
        "orientation_xyzw": dict(zip("xyzw", orientation, strict=True)),
        "forward_direction": dict(zip("xyz", direction, strict=True)),
    }


def camera_forward_direction(
    qx: float, qy: float, qz: float, qw: float
) -> tuple[float, float, float]:
    """Rotate the Gazebo camera's local -Z forward axis into world coordinates."""
    tx = -2.0 * qy
    ty = 2.0 * qx
    tz = 0.0
    direction = (
        qw * tx + qy * tz - qz * ty,
        qw * ty + qz * tx - qx * tz,
        qw * tz + qx * ty - qy * tx - 1.0,
    )
    length = math.sqrt(sum(component * component for component in direction))
    return tuple(
        0.0 if abs(component) <= 1e-12 else component / length
        for component in direction
    )


def _reader(
    stream: TextIO, lines: queue.SimpleQueue[str], finished: threading.Event
) -> None:
    try:
        for line in stream:
            lines.put(line)
    finally:
        finished.set()


def _write_record(output: TextIO, record: dict[str, object]) -> None:
    output.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")
    output.flush()


def _utc_now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds")


def _start_subscription(topic: str) -> subprocess.Popen[str]:
    gz_command = shlex.split(os.environ.get("GZ_BIN", "gz"))
    return subprocess.Popen(
        [*gz_command, "topic", "--echo", "--json-output", "--topic", topic],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
    )


def log_camera_pose(*, topic: str, output_path: Path, interval_s: float) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    stop_requested = threading.Event()

    def request_stop(_: int, __: object) -> None:
        stop_requested.set()

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    process: subprocess.Popen[str] | None = None
    try:
        process = _start_subscription(topic)
        assert process.stdout is not None
        lines: queue.SimpleQueue[str] = queue.SimpleQueue()
        reader_finished = threading.Event()
        reader = threading.Thread(
            target=_reader,
            args=(process.stdout, lines, reader_finished),
            daemon=True,
        )
        reader.start()
        latest_pose: dict[str, dict[str, float]] | None = None
        latest_pose_monotonic_s: float | None = None
        next_record_s = time.monotonic()
        decode_failures = 0

        with output_path.open("a", encoding="utf-8") as output:
            while not stop_requested.is_set():
                while True:
                    try:
                        line = lines.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        latest_pose = parse_camera_pose(json.loads(line))
                        latest_pose_monotonic_s = time.monotonic()
                    except (CameraLoggerError, json.JSONDecodeError):
                        decode_failures += 1

                now_s = time.monotonic()
                if now_s >= next_record_s:
                    record: dict[str, object] = {
                        "event": "gazebo_gui_camera_pose",
                        "recorded_at_utc": _utc_now(),
                        "topic": topic,
                        "pose_available": latest_pose is not None,
                        "decode_failures": decode_failures,
                    }
                    if latest_pose is not None and latest_pose_monotonic_s is not None:
                        record.update(latest_pose)
                        record["pose_age_s"] = round(
                            now_s - latest_pose_monotonic_s, 6
                        )
                    _write_record(output, record)
                    next_record_s = now_s + interval_s
                if reader_finished.is_set() and process.poll() is not None:
                    raise RuntimeError(
                        f"Gazebo camera pose subscription exited with {process.returncode}"
                    )
                stop_requested.wait(0.05)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--interval-s", default=str(DEFAULT_INTERVAL_S))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    interval_s = parse_interval_s(args.interval_s)
    print(
        "Gazebo GUI camera logger: "
        f"topic={args.topic} interval_s={interval_s:g} output={args.output}"
    )
    log_camera_pose(topic=args.topic, output_path=args.output, interval_s=interval_s)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (CameraLoggerError, OSError, RuntimeError) as error:
        raise SystemExit(f"Gazebo GUI camera logger error: {error}") from error
