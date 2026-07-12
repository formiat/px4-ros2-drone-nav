#!/usr/bin/env python3
"""Validate deterministic log markers for the drone navigation headless run."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path


CRITICAL_PX4_PATTERN = re.compile(
    r"Sensor [0-9]+ missing"
    r"|Accel Sensor [0-9]+ missing"
    r"|Gyro Sensor [0-9]+ missing"
    r"|barometer [0-9]+ missing"
    r"|Found 0 compass"
    r"|Timed out waiting for Gazebo world"
    r"|gz_bridge failed"
    r"|Attitude failure",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class ValidationOptions:
    expected_static: bool | None = None
    expected_memory: bool | None = None
    expected_current_lidar: bool | None = None
    enable_lidar_debug: bool = True
    mission_check: bool = False
    allow_mission_failure: bool = False


@dataclass
class ValidationResult:
    messages: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    def ok_message(self, label: str) -> None:
        self.messages.append(f"OK: {label}")

    def skip(self, label: str, reason: str) -> None:
        self.messages.append(f"SKIP: {label}: {reason}")

    def warn(self, message: str) -> None:
        self.messages.append(f"WARN: {message}")

    def fail(self, label: str) -> None:
        self.errors.append(f"FAIL: {label}")

    def require(self, label: str, text: str, pattern: str) -> None:
        if re.search(pattern, text, flags=re.MULTILINE):
            self.ok_message(label)
            return
        self.fail(label)


def parse_bool(value: str | None) -> bool | None:
    if value is None or value == "":
        return None
    normalized = value.lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    return None


def source_value_pattern(source_name: str, expected_value: bool) -> str:
    enabled_values = r"(?:true|always|enabled|on)"
    disabled_values = r"(?:false|never|disabled|off)"
    expected_pattern = enabled_values if expected_value else disabled_values
    return (
        rf"Planner obstacle sources: .*{re.escape(source_name)}="
        rf"{expected_pattern}\b"
    )


def require_source_value(
    result: ValidationResult,
    label: str,
    source_name: str,
    expected_value: bool | None,
    ros_log: str,
) -> None:
    if expected_value is None:
        result.skip(label, "source value comes from custom params file")
        return

    result.require(label, ros_log, source_value_pattern(source_name, expected_value))


def validate_known_static_classifier_contract(
    result: ValidationResult, ros_log: str
) -> None:
    matches = re.findall(
        r"Known static lidar classifier: node=(obstacle_memory|planner) "
        r"status=(\S+) path='([^']*)' volumes=(\d+) tolerance=([0-9.]+)m",
        ros_log,
    )
    by_node = {
        node: (status, path, int(volumes), float(tolerance))
        for node, status, path, volumes, tolerance in matches
    }
    if "obstacle_memory" not in by_node or "planner" not in by_node:
        result.fail("known-static classifier effective config is logged by both nodes")
        return
    result.ok_message("known-static classifier effective config is logged by both nodes")

    obstacle_memory = by_node["obstacle_memory"]
    planner = by_node["planner"]
    if obstacle_memory[0] != "ready" or planner[0] != "ready":
        result.fail("known-static classifier is ready in both nodes")
    else:
        result.ok_message("known-static classifier is ready in both nodes")
    if obstacle_memory[1:] != planner[1:]:
        result.fail("known-static classifier effective configs match")
    else:
        result.ok_message("known-static classifier effective configs match")


def validate_logs(
    *,
    ros_log: str,
    px4_log: str,
    options: ValidationOptions,
) -> ValidationResult:
    result = ValidationResult()

    if options.expected_memory is True and options.expected_current_lidar is True:
        validate_known_static_classifier_contract(result, ros_log)

    result.require("Gazebo world is ready", px4_log, r"Gazebo world is ready")
    result.require(
        "PX4 local position is valid",
        ros_log,
        r"First valid PX4 local position",
    )

    if options.expected_memory is True:
        result.require(
            "lidar scans are received by obstacle memory",
            ros_log,
            r"First lidar scan",
        )
    elif options.expected_current_lidar is True:
        result.require(
            "lidar scans are received by planner",
            ros_log,
            r"First planner lidar scan",
        )
    elif (
        options.expected_memory is False
        and options.expected_current_lidar is False
    ):
        result.skip(
            "lidar scans are received",
            "static-only source configuration does not require lidar data",
        )
    else:
        result.require(
            "lidar scans are received",
            ros_log,
            r"First lidar scan|First planner lidar scan|LIDAR_DEBUG snapshot=",
        )

    if options.expected_memory is True:
        result.require(
            "obstacle memory receives lidar",
            ros_log,
            r"Obstacle memory update:",
        )
        result.require(
            "obstacle memory is available to planner",
            ros_log,
            r"First obstacle memory grid|memory\[enabled=true .*has_memory=true",
        )
    elif options.expected_memory is False:
        result.require(
            "obstacle memory source is disabled",
            ros_log,
            r"Obstacle memory mapping is disabled|Obstacle memory ready: enabled=false",
        )
        result.require(
            "obstacle memory is disabled in planning",
            ros_log,
            r"memory\[enabled=false|Planner obstacle sources: .*memory=false",
        )
    else:
        result.skip(
            "obstacle memory source state",
            "source value comes from custom params file",
        )

    result.require(
        "planner obstacle sources are logged",
        ros_log,
        r"Planner obstacle sources:",
    )
    require_source_value(
        result,
        "static source value is logged",
        "static",
        options.expected_static,
        ros_log,
    )
    require_source_value(
        result,
        "memory source value is logged",
        "memory",
        options.expected_memory,
        ros_log,
    )
    require_source_value(
        result,
        "current lidar source value is logged",
        "current_lidar",
        options.expected_current_lidar,
        ros_log,
    )

    if options.expected_static is True:
        result.require(
            "static city map is loaded",
            ros_log,
            r"Static city map loaded:",
        )
        result.require(
            "static map contributes to planning",
            ros_log,
            r"static\[enabled=true loaded=true used=true|Planner obstacle sources: static=true",
        )
    elif options.expected_static is False:
        result.require(
            "static map is disabled in planning",
            ros_log,
            r"static\[enabled=false",
        )
    else:
        result.skip(
            "static map source state",
            "source value comes from custom params file",
        )

    if options.expected_current_lidar is True:
        result.require(
            "current lidar is available to planner",
            ros_log,
            r"First planner lidar scan|current_lidar\[enabled=true used=true",
        )
    elif options.expected_current_lidar is False:
        result.require(
            "current lidar is disabled in planning",
            ros_log,
            r"current_lidar\[enabled=false used=false|Planner obstacle sources: .*current_lidar=false",
        )
    else:
        result.skip(
            "current lidar source state",
            "source value comes from custom params file",
        )

    result.require(
        "planner publishes a path",
        ros_log,
        r"Published path:.*\bwaypoints=[1-9]\d*",
    )
    result.require(
        "offboard command is sent",
        ros_log,
        r"Sent PX4 command: VEHICLE_CMD_DO_SET_MODE",
    )
    result.require(
        "arm command is sent",
        ros_log,
        r"Sent PX4 command: VEHICLE_CMD_COMPONENT_ARM_DISARM",
    )
    result.require(
        "vehicle reaches armed offboard state",
        ros_log,
        r"Offboard summary: .*armed=true.*offboard=true",
    )

    if options.enable_lidar_debug:
        result.require(
            "lidar debug snapshots are written",
            ros_log,
            r"LIDAR_DEBUG snapshot=",
        )

    if re.search(r"MISSION_RESULT success=false", ros_log):
        if not options.allow_mission_failure:
            result.fail("mission monitor reported failure")
        else:
            result.warn(
                "mission monitor reported failure "
                "(allowed by ALLOW_MISSION_FAILURE=true)"
            )

    if options.mission_check:
        if options.allow_mission_failure and re.search(
            r"MISSION_RESULT success=false", ros_log
        ):
            result.warn(
                "mission monitor did not verify complete A-to-B flight "
                "(allowed by ALLOW_MISSION_FAILURE=true)"
            )
        else:
            result.require(
                "mission monitor verifies complete A-to-B flight",
                ros_log,
                r"MISSION_RESULT success=true",
            )
            passage_counts = re.search(
                r"MISSION_RESULT success=true.*actual_passage_openings_seen=(\d+) "
                r"known_passage_openings=(\d+)",
                ros_log,
            )
            if passage_counts is None:
                result.fail("mission result reports passage opening counts")
            elif passage_counts.group(1) != passage_counts.group(2):
                result.fail("all known passage openings are observed")
            else:
                result.ok_message("mission result reports passage opening counts")
                result.ok_message("all known passage openings are observed")
            if (
                options.expected_memory is True
                or options.expected_current_lidar is True
            ):
                result.require(
                    "known-static classifier ignores physical passage masses",
                    ros_log,
                    r"known_static\[ignored=[1-9][0-9]*",
                )

    if CRITICAL_PX4_PATTERN.search(px4_log):
        if not options.allow_mission_failure:
            result.fail("PX4 log contains critical simulator/preflight errors")
        else:
            result.warn(
                "PX4 log contains critical simulator/preflight errors "
                "(allowed by ALLOW_MISSION_FAILURE=true)"
            )
    else:
        result.ok_message("no critical PX4 simulator/preflight errors found")

    return result


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise RuntimeError(f"cannot read log file '{path}': {exc}") from exc


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate drone navigation headless run logs."
    )
    parser.add_argument("--ros-log", required=True, type=Path)
    parser.add_argument("--px4-log", required=True, type=Path)
    parser.add_argument("--expected-static", default="")
    parser.add_argument("--expected-memory", default="")
    parser.add_argument("--expected-current-lidar", default="")
    parser.add_argument("--enable-lidar-debug", default="true")
    parser.add_argument("--mission-check", action="store_true")
    parser.add_argument("--allow-mission-failure", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    options = ValidationOptions(
        expected_static=parse_bool(args.expected_static),
        expected_memory=parse_bool(args.expected_memory),
        expected_current_lidar=parse_bool(args.expected_current_lidar),
        enable_lidar_debug=parse_bool(args.enable_lidar_debug) is not False,
        mission_check=args.mission_check,
        allow_mission_failure=args.allow_mission_failure,
    )
    try:
        result = validate_logs(
            ros_log=read_text(args.ros_log),
            px4_log=read_text(args.px4_log),
            options=options,
        )
    except RuntimeError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

    for message in result.messages:
        print(message)
    for error in result.errors:
        print(error, file=sys.stderr)

    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
