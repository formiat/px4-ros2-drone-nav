#!/usr/bin/env python3
"""Validate the production MPPI headless navigation contract."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


CRITICAL_PX4_PATTERN = re.compile(
    r"(?:ERROR \[|Critical failure|Segmentation fault)",
    re.IGNORECASE,
)


def parse_bool(value: str) -> bool | None:
    normalized = value.strip().lower()
    if not normalized:
        return None
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"invalid boolean value: {value}")


def require(label: str, text: str, pattern: str, errors: list[str]) -> None:
    if re.search(pattern, text):
        print(f"OK: {label}")
    else:
        errors.append(f"FAIL: {label}")


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate production MPPI headless run logs."
    )
    parser.add_argument("--ros-log", required=True, type=Path)
    parser.add_argument("--px4-log", required=True, type=Path)
    parser.add_argument("--expected-static", default="")
    parser.add_argument("--expected-memory", default="")
    parser.add_argument("--expected-current-lidar", default="")
    parser.add_argument("--enable-lidar-debug", default="true")
    parser.add_argument("--mission-check", action="store_true")
    parser.add_argument("--allow-mission-failure", action="store_true")
    args = parser.parse_args()

    ros_log = read_text(args.ros_log)
    px4_log = read_text(args.px4_log)
    expected_static = parse_bool(args.expected_static)
    enable_lidar_debug = parse_bool(args.enable_lidar_debug) is not False
    errors: list[str] = []

    require("Gazebo world is ready", px4_log, r"Gazebo world is ready", errors)
    require(
        "obstacle memory receives lidar",
        ros_log,
        r"First lidar scan|Obstacle memory update:",
        errors,
    )
    require(
        "raw obstacle snapshots are published",
        ros_log,
        r"Raw obstacle snapshot|raw obstacle snapshot|raw_revision=",
        errors,
    )
    require(
        "production MPPI is ready",
        ros_log,
        r"Production MPPI ready:",
        errors,
    )
    require(
        "ESDF is available",
        ros_log,
        r"PRODUCTION_MPPI_ESDF(?:3D)? .*revision=",
        errors,
    )
    require(
        "global guide is available",
        ros_log,
        r"PRODUCTION_MPPI_GUIDE .*guide_valid=true|target_source=global_route_3d",
        errors,
    )
    require(
        "production MPPI publishes collision-free horizons",
        ros_log,
        r"PRODUCTION_MPPI_TICK .*raw_collision=false "
        r".*known_solid_collision=false",
        errors,
    )
    require(
        "production offboard is ready",
        ros_log,
        r"Production MPPI offboard ready:",
        errors,
    )
    require(
        "vehicle is armed by production offboard",
        px4_log,
        r"Armed by external command",
        errors,
    )
    require(
        "vehicle takes off under production MPPI",
        px4_log,
        r"Takeoff detected",
        errors,
    )

    if expected_static is True:
        require(
            "static map contributes to raw occupancy",
            ros_log,
            r"STATIC_WORLD_3D .*\.occupancy3d|"
            r"Published static world visualization:|use_static_map=true",
            errors,
        )
    elif expected_static is False and re.search(r"use_static_map=true", ros_log):
        errors.append("FAIL: static map is disabled")
    else:
        print("OK: static map source contract")

    if enable_lidar_debug:
        require(
            "lidar debug snapshots are written",
            ros_log,
            r"LIDAR_DEBUG snapshot=",
            errors,
        )

    if re.search(r"CRASH_EVENT|crashed=true", ros_log):
        errors.append("FAIL: crash was reported")
    else:
        print("OK: no crash was reported")

    mission_failed = re.search(r"MISSION_RESULT success=false", ros_log) is not None
    if args.mission_check and not args.allow_mission_failure:
        require(
            "mission monitor verifies complete flight",
            ros_log,
            r"MISSION_RESULT success=true",
            errors,
        )
    elif mission_failed:
        print("WARN: mission failure was allowed")

    if CRITICAL_PX4_PATTERN.search(px4_log):
        errors.append("FAIL: PX4 log contains critical simulator errors")
    else:
        print("OK: no critical PX4 simulator errors found")

    for error in errors:
        print(error, file=sys.stderr)
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
